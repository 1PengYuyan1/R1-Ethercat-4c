// =============================================================================
// clamp_position_monitor_main.cpp
//
// 夹爪 (Pitch_Large + Pitch_Small DM-J4310) 实时位置监视器.
//   - 只读位置: 仅向 DM 发 Enter 帧 (无 MIT/控制参数), 电机零力矩自由转
//   - 在终端用 ANSI 控制码刷 4 行 (banner + T+ + Pitch_Large + Pitch_Small)
//   - 刷新率: --rate (默认 20Hz), Enter 帧节拍跟刷新一致
//
// 交互按键 (无需回车):
//   z       触发"零点设置"二次确认 (3s 内按 y 确认, 其他键取消)
//   y       仅在确认窗口内有效, 给 large + small 各发一次 Save_Zero (FF..FE)
//           DM 会把当前物理位置写到 EEPROM 当新零点, 掉电不丢; 不可逆!!
//   q/Esc   退出 (等价 Ctrl+C)
//
// 用法:
//   sudo ./clamp_position_monitor enp86s0           # 默认 20Hz
//   sudo ./clamp_position_monitor enp86s0 --rate 50
// =============================================================================

#include "dvc_motor_dm.h"
#include "ecat_manager.h"
#include "linkx4c_handler.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>


namespace {

constexpr int      kChannelCount = 4;
constexpr uint16_t kAliasFd      = 1;   // FD LinkX (挂夹爪)
constexpr uint16_t kAliasClassic = 2;
constexpr uint8_t  kClampChannel = 0;   // FD-LinkX ch0
constexpr float    kRad2Deg      = 57.2957795f;
constexpr float    kCelsiusOffset = 273.15f;  // 反推 ℃ (driver 存的是 K)

ecat_master_t master;
linkx_t       linkx_dev_classic;
linkx_t       linkx_dev_fd;
Class_Motor_DM_Normal motor_large;
Class_Motor_DM_Normal motor_small;
std::atomic<bool> g_running{true};

// === 终端原始模式: 让 z/y/q 不用回车 ===
termios g_orig_termios;
bool    g_termios_saved = false;

void Restore_Stdin_Termios() {
    if (g_termios_saved) {
        ::tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_termios_saved = false;
    }
}

bool Setup_Raw_Stdin() {
    if (::tcgetattr(STDIN_FILENO, &g_orig_termios) != 0) return false;
    g_termios_saved = true;
    std::atexit(Restore_Stdin_Termios);

    termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;

    // 同时 O_NONBLOCK 让 read() 不阻塞 (双保险)
    int flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags != -1) ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    return true;
}

int Poll_Stdin_Char() {
    pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if (::poll(&pfd, 1, 0) <= 0) return -1;
    if (!(pfd.revents & POLLIN)) return -1;
    char c = 0;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) return static_cast<unsigned char>(c);
    return -1;
}

// === 零点设置二次确认状态机 ===
enum BannerState {
    BANNER_IDLE = 0,
    BANNER_CONFIRM,   // z 已按, 等 y; 超时回 IDLE
    BANNER_SUCCESS,   // y 已确认, 已发 Save_Zero, 展示一会儿回 IDLE
    BANNER_CANCEL,    // 取消(超时或别的键), 展示一会儿回 IDLE
};
BannerState g_banner_state = BANNER_IDLE;
std::chrono::steady_clock::time_point g_banner_deadline;
constexpr std::chrono::seconds kConfirmWindow{3};
constexpr std::chrono::seconds kAfterShowFor{2};

void Signal_Handler(int /*sig*/) {
    g_running.store(false);
    master.is_running = false;
    // 注意: 严格说 tcsetattr 不在 async-signal-safe 列表; 实测 Linux 上对终端
    // 复原很可靠. 这里复原一次, 避免 Ctrl+C 后终端无回显
    Restore_Stdin_Termios();
}

bool Bring_Up_Ecat_And_Linkx(const char *ifname) {
    if (!ecat_master_init(&master, ifname)) {
        std::cerr << "[FATAL] ecat_master_init failed on " << ifname << std::endl;
        return false;
    }
    if (master.ctx.slavecount < 2) {
        std::cerr << "[FATAL] expected >=2 EtherCAT slaves, found "
                  << master.ctx.slavecount << std::endl;
        return false;
    }

    int classic_slave_id = 0;
    int fd_slave_id      = 0;
    for (int i = 1; i <= master.ctx.slavecount; ++i) {
        uint16_t alias = master.ctx.slavelist[i].aliasadr;
        if      (alias == kAliasFd)      fd_slave_id      = i;
        else if (alias == kAliasClassic) classic_slave_id = i;
    }
    if (fd_slave_id == 0 || classic_slave_id == 0) {
        std::cerr << "[WARN] Station Alias 未设, 退回固定拓扑 fd=1 classic=2" << std::endl;
        fd_slave_id      = 1;
        classic_slave_id = 2;
    }
    std::cout << "[ECAT] classic=slave " << classic_slave_id
              << "  fd=slave " << fd_slave_id << std::endl;

    linkx_init(&linkx_dev_classic, (uint32_t)classic_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_classic);
    for (int ch = 0; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_classic, ch, 0, 2, 31, 8, 8, 1, 31, 8, 8);

    linkx_init(&linkx_dev_fd, (uint32_t)fd_slave_id, &master.ctx);
    linkx_hw_wakeup(&linkx_dev_fd);
    linkx_set_can_baudrate(&linkx_dev_fd, 0, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    linkx_set_can_baudrate(&linkx_dev_fd, 1, 0, 2, 31, 8, 8, 1, 31, 8, 8);
    for (int ch = 2; ch < kChannelCount; ++ch)
        linkx_set_can_baudrate(&linkx_dev_fd, ch, 1, 2, 31, 8, 8, 1, 6, 1, 1);

    if (!ecat_master_bring_online(&master)) {
        std::cerr << "[FATAL] ecat_master_bring_online failed" << std::endl;
        return false;
    }
    std::cout << "[ECAT] online" << std::endl;
    return true;
}

// 一行打印一台电机. 调用前保证光标位于该行行首.
void Print_Motor_Line(const char *name, Class_Motor_DM_Normal &m) {
    const float rad   = m.Get_Now_Radian();
    const float deg   = rad * kRad2Deg;
    const float omega = m.Get_Now_Omega();
    const float tau   = m.Get_Now_Torque();
    const float t_mos = m.Get_Now_MOS_Temperature() - kCelsiusOffset;
    const bool  alive = (m.Get_Status() == Motor_DM_Status_ENABLE);
    // \033[K 清行尾, 避免上次窄字符残留;末尾不要 \n, 由 caller 控制
    std::printf("\033[K  %-12s pos=%+7.3f rad (%+7.2f deg)  w=%+6.2f rad/s  "
                "t=%+5.2f Nm  T_mos=%4.1f C  [%s]",
                name, rad, deg, omega, tau, t_mos,
                alive ? "ALIVE" : "OFFLINE");
}

// 打印 banner 行 (第 1 行), 内容随状态机变化. 调用前保证光标位于行首.
void Print_Banner_Line() {
    const auto now = std::chrono::steady_clock::now();
    if (g_banner_state != BANNER_IDLE && now >= g_banner_deadline) {
        if (g_banner_state == BANNER_CONFIRM) g_banner_state = BANNER_CANCEL,
                                              g_banner_deadline = now + kAfterShowFor;
        else g_banner_state = BANNER_IDLE;
    }

    std::printf("\033[K");
    switch (g_banner_state) {
        case BANNER_IDLE:
            std::printf("  [keys] z=set-zero (asks confirm)   q/Esc=quit");
            break;
        case BANNER_CONFIRM: {
            auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                g_banner_deadline - now).count();
            std::printf("  \033[1;33m>>> CONFIRM zero BOTH motors? press 'y' within %.1fs "
                        "(other key cancels) <<<\033[0m", left / 1000.0);
            break;
        }
        case BANNER_SUCCESS:
            std::printf("  \033[1;32m*** ZEROED ***  Save_Zero sent to Pitch_Large + "
                        "Pitch_Small (EEPROM)\033[0m");
            break;
        case BANNER_CANCEL:
            std::printf("  \033[1;31m--- zeroing CANCELED ---\033[0m");
            break;
    }
}

// 处理按键. 返回 true 表示请求退出.
bool Handle_Key(int c, float t_sec_now) {
    if (c < 0) return false;
    // 大小写都接受
    if (c == 'q' || c == 'Q' || c == 27 /* Esc */) return true;

    const auto now = std::chrono::steady_clock::now();
    if (c == 'z' || c == 'Z') {
        if (g_banner_state != BANNER_CONFIRM) {
            g_banner_state = BANNER_CONFIRM;
            g_banner_deadline = now + kConfirmWindow;
        }
        return false;
    }
    if (g_banner_state == BANNER_CONFIRM) {
        if (c == 'y' || c == 'Y') {
            motor_large.CAN_Send_Save_Zero();
            motor_small.CAN_Send_Save_Zero();
            // 立即 flush 这条 TX (CAN_Send_Save_Zero 已写 LinkX TX 队列, 下次 send_pdos 推出)
            std::fprintf(stderr, "\n[ZERO] Save_Zero sent at T+%.2fs "
                         "(large + small EEPROM updated)\n", t_sec_now);
            g_banner_state = BANNER_SUCCESS;
            g_banner_deadline = now + kAfterShowFor;
        } else {
            g_banner_state = BANNER_CANCEL;
            g_banner_deadline = now + kAfterShowFor;
        }
    }
    return false;
}

}  // namespace


int main(int argc, char *argv[]) {
    std::string ifname  = "enp86s0";
    int         rate_hz = 20;

    if (argc > 1) ifname = argv[1];
    for (int i = 2; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        std::string v = argv[i + 1];
        if (k == "--rate") rate_hz = std::stoi(v);
    }
    if (rate_hz <= 0 || rate_hz > 500) {
        std::cerr << "[FATAL] --rate must be in [1, 500]" << std::endl;
        return -1;
    }
    const uint32_t refresh_ticks = static_cast<uint32_t>(1000 / rate_hz);

    std::signal(SIGINT,  Signal_Handler);
    std::signal(SIGTERM, Signal_Handler);

    std::cout << "================================================\n"
              << "  Clamp Position Monitor (read-only + set-zero)\n"
              << "  iface=" << ifname << "  rate=" << rate_hz << "Hz\n"
              << "  keys: z=set-zero (asks 'y' to confirm)   q/Esc=quit\n"
              << "================================================" << std::endl;

    if (!Bring_Up_Ecat_And_Linkx(ifname.c_str())) return -1;

    if (!Setup_Raw_Stdin()) {
        std::cerr << "[WARN] failed to set raw stdin, key input disabled" << std::endl;
    }

    motor_large.Init(&linkx_dev_fd, kClampChannel, /*Rx*/0x11, /*Tx*/0x01,
                     Motor_DM_Control_Method_NORMAL_MIT,
                     12.5f, 20.0f, 15.0f, 10.261194f);
    motor_small.Init(&linkx_dev_fd, kClampChannel, /*Rx*/0x12, /*Tx*/0x02,
                     Motor_DM_Control_Method_NORMAL_MIT,
                     12.5f, 20.0f, 15.0f, 10.261194f);

    // 预占 4 行 (banner + T+ 时间 + 两台电机), 之后每帧 \033[4F 跳回开头重写
    std::printf("\n\n\n\n");
    std::fflush(stdout);

    const auto t0  = std::chrono::steady_clock::now();
    uint32_t   tick = 0;
    auto       next_wakeup = std::chrono::steady_clock::now();

    while (g_running.load() && master.is_running) {
        next_wakeup += std::chrono::milliseconds(1);
        std::this_thread::sleep_until(next_wakeup);

        ecat_master_sync(&master);
        linkx_recv_pdos(&linkx_dev_classic);
        linkx_recv_pdos(&linkx_dev_fd);

        // 收 CAN 反馈 (只看 FD ch0)
        can_msg_t msg;
        while (linkx_quick_recv(&linkx_dev_fd, kClampChannel, &msg)) {
            motor_large.CAN_RxCpltCallback(msg.data);
            motor_small.CAN_RxCpltCallback(msg.data);
        }

        // 100ms: 维护 alive 滑窗 (Get_Status 用)
        if ((tick % 100U) == 0U) {
            motor_large.TIM_Alive_PeriodElapsedCallback();
            motor_small.TIM_Alive_PeriodElapsedCallback();
        }

        // 每 ms 都看一次键 (banner 倒计时也跟得上), 不只在 refresh 节拍
        const float t_sec_now = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t0).count();
        const int key = Poll_Stdin_Char();
        if (Handle_Key(key, t_sec_now)) {
            g_running.store(false);
            master.is_running = false;
            break;
        }

        // 每 refresh_ticks: 发 Enter (唯一 TX, 不含 Save_Zero 那次) + 刷屏
        if ((tick % refresh_ticks) == 0U) {
            motor_large.CAN_Send_Enter();
            motor_small.CAN_Send_Enter();

            // \033[4F: 移到上 4 行行首; 各行末用 \n 推进; \033[K 在打印函数里清行尾
            std::printf("\033[4F");
            Print_Banner_Line();
            std::printf("\n");
            std::printf("\033[K[T+%7.2f s]\n", t_sec_now);
            Print_Motor_Line("Pitch_Large", motor_large);
            std::printf("\n");
            Print_Motor_Line("Pitch_Small", motor_small);
            std::printf("\n");
            std::fflush(stdout);
        }

        linkx_send_pdos(&linkx_dev_classic);
        linkx_send_pdos(&linkx_dev_fd);
        ++tick;
    }

    // 让退出消息从下一行开始, 不要叠在最后那行末尾
    std::printf("\n");
    std::cout << "[MONITOR] shutting down..." << std::endl;
    motor_large.CAN_Send_Exit();
    motor_small.CAN_Send_Exit();
    linkx_send_pdos(&linkx_dev_fd);

    Restore_Stdin_Termios();
    return 0;
}
