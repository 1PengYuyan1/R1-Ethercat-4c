//
// encoder_persistence.cpp — 见同名 .h。函数体从 crt_chassis.cpp 迁移而来，
// 行为 bit-for-bit 不变；仅参数化掉对 Class_Chassis 状态的引用。
//

#include "encoder_persistence.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

inline uint64_t now_ms()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

// CRC-32/IEEE (poly 0xEDB88320, reflected). 直接对一段字节求 CRC，无表，体积小。
inline uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i)
  {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1u)));
  }
  return crc ^ 0xFFFFFFFFu;
}

// 持久化记录的 CRC payload：can_id (4B LE) || total_pulses (8B LE) || raw_abs (4B LE) || seq (4B LE)
inline uint32_t encoder_persist_crc(uint32_t can_id, int64_t total_pulses, int32_t raw_abs, uint32_t seq)
{
  uint8_t buf[4 + 8 + 4 + 4];
  std::memcpy(buf + 0, &can_id, 4);
  std::memcpy(buf + 4, &total_pulses, 8);
  std::memcpy(buf + 12, &raw_abs, 4);
  std::memcpy(buf + 16, &seq, 4);
  return crc32_ieee(buf, sizeof(buf));
}

}  // namespace

namespace encoder_persistence {

bool LoadZeroOffsets(Class_Encoder_BRT (&encoders)[STEER_NUM], const std::string &path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    std::cerr << "\n"
              << "================================================================\n"
              << "[CHASSIS][!! STEER ZERO NOT CALIBRATED !!]\n"
              << "  zero-offset file NOT FOUND at: '" << path << "'\n"
              << "  All 4 steer encoders will use zero_offset=0,\n"
              << "  which means wheel_rad / wheel_deg will NOT read 0\n"
              << "  at your physical midpoint.\n"
              << "\n"
              << "  To calibrate ONCE:\n"
              << "    1) move steering to the desired zero position\n"
              << "       (and set midpoint on the BRT encoders if needed)\n"
              << "    2) restart with:  export CAPTURE_STEER_ZERO=1\n"
              << "       then run the program. it will save the offsets.\n"
              << "    3) restart again WITHOUT CAPTURE_STEER_ZERO to use them.\n"
              << "================================================================\n"
              << std::endl;
    // 仍然打印一次每个编码器当前的 zero_offset（应为 0）以便对账
    for (int i = 0; i < STEER_NUM; ++i)
    {
      std::cout << "[CHASSIS] final zero_offset id=0x" << std::hex
                << static_cast<int>(encoders[i].Get_Can_ID()) << std::dec
                << " offset=" << encoders[i].Get_Zero_Offset()
                << " (default)" << std::endl;
    }
    return false;
  }

  std::string line;
  int loaded = 0;
  int anchor_loaded = 0;
  int anchor_missing = 0;
  while (std::getline(ifs, line))
  {
    // 去掉注释和首尾空白
    auto hash = line.find('#');
    if (hash != std::string::npos)
      line.erase(hash);
    std::istringstream iss(line);
    std::string id_token;
    int64_t offset = 0;
    if (!(iss >> id_token >> offset))
      continue;

    // 第三列：logical_zero_anchor（int64），可选；缺失 = 旧格式
    int64_t anchor = 0;
    bool has_anchor = static_cast<bool>(iss >> anchor);

    // 支持 0x.. 十六进制或十进制
    uint32_t can_id = static_cast<uint32_t>(
      std::strtoul(id_token.c_str(), nullptr, 0));

    for (int i = 0; i < STEER_NUM; ++i)
    {
      if (encoders[i].Get_Can_ID() == can_id)
      {
        encoders[i].Set_Zero_Offset(static_cast<int32_t>(offset));
        if (has_anchor)
        {
          encoders[i].Set_Logical_Zero_Anchor(anchor);
          ++anchor_loaded;
        }
        else
        {
          encoders[i].Clear_Logical_Zero_Anchor();
          ++anchor_missing;
        }
        ++loaded;
        break;
      }
    }
  }

  std::cout << "[CHASSIS] steer zero-offsets loaded: " << loaded << "/"
            << STEER_NUM << " from '" << path << "'"
            << "  (anchor: " << anchor_loaded << " loaded, "
            << anchor_missing << " missing)" << std::endl;
  if (anchor_missing > 0)
  {
    std::cerr << "[CHASSIS][WARN] " << anchor_missing
              << " encoder(s) have NO logical_zero_anchor (old 2-column format).\n"
              << "  → wheel_deg_true 在机械零位不会读 0；请重新跑一次"
                 " CAPTURE_STEER_ZERO=1 以写入新格式。" << std::endl;
  }
  // 不论是否全部命中，逐个打印当前生效的 zero_offset，便于现场对账
  for (int i = 0; i < STEER_NUM; ++i)
  {
    const uint32_t cid = encoders[i].Get_Can_ID();
    const int32_t off = encoders[i].Get_Zero_Offset();
    const int64_t anc = encoders[i].Get_Logical_Zero_Anchor();
    const bool anc_ok = encoders[i].Is_Logical_Zero_Anchor_Valid();
    std::cout << "[CHASSIS] final zero id=0x" << std::hex
              << cid << std::dec << " raw_offset=" << off
              << " logical_anchor=" << anc
              << (anc_ok ? "" : " (INVALID, will use L mod 14336 directly)")
              << (off == 0 ? "  <-- WARN: raw_offset still 0" : "")
              << std::endl;
  }
  if (loaded < STEER_NUM)
  {
    std::cerr << "[CHASSIS][WARN] only " << loaded << "/" << STEER_NUM
              << " encoders matched in '" << path
              << "'. unmatched encoders will read non-zero at midpoint."
              << std::endl;
  }
  return loaded > 0;
}

bool SaveZeroOffsets(Class_Encoder_BRT (&encoders)[STEER_NUM], const std::string &path)
{
  // 先确认每个编码器至少收到过一帧合法数据
  for (int i = 0; i < STEER_NUM; ++i)
  {
    if (!encoders[i].Has_Valid_Wheel_Posture())
    {
      std::cerr << "[CHASSIS] cannot save zero offsets: encoder index " << i
                << " (id=0x" << std::hex
                << static_cast<int>(encoders[i].Get_Can_ID()) << std::dec
                << ") has no valid frame yet" << std::endl;
      return false;
    }
  }

  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs.is_open())
  {
    std::cerr << "[CHASSIS] failed to open '" << path
              << "' for writing zero offsets" << std::endl;
    return false;
  }

  ofs << "# steer encoder zero offsets (auto-generated)\n"
      << "# format: <can_id> <zero_offset_pulses> <logical_zero_anchor>\n"
      << "#   zero_offset_pulses : raw 50圈域的零点 raw 值（0..204799）\n"
      << "#   logical_zero_anchor: 校准瞬间的 total_unwrapped_pulses（int64）\n"
      << "#   角度公式 phase = (L - logical_zero_anchor) mod 14336\n";
  for (int i = 0; i < STEER_NUM; ++i)
  {
    const uint32_t can_id = encoders[i].Get_Can_ID();
    const int32_t offset =
      static_cast<int32_t>(encoders[i].Get_EncoderValue());
    encoders[i].Set_Zero_Offset(offset);  // raw 域零点立即生效
    // 步骤文档 §5 step 3「记录 L0 参考点」方案：
    // 不重置 total_unwrapped_pulses，而是把当前 L 记成角度计算的逻辑原点。
    // 这样 L 仍维持不变量 L mod 204800 == raw_true，恢复算法不会再把零点踩坏。
    encoders[i].Capture_Logical_Zero_Anchor();
    const int64_t anchor = encoders[i].Get_Logical_Zero_Anchor();
    ofs << "0x" << std::hex << std::setw(2) << std::setfill('0') << can_id
        << std::dec << std::setfill(' ') << " " << offset << " " << anchor << "\n";
    std::cout << "[CHASSIS] captured zero id=0x" << std::hex << can_id
              << std::dec << " raw_offset=" << offset
              << " logical_zero_anchor=" << anchor << std::endl;
  }
  std::cout << "[CHASSIS] steer zero-offsets saved to '" << path << "'"
            << std::endl;

  return true;
}

bool LoadUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                         const std::string &path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    std::cout << "[CHASSIS] unwrapped-pulses file not found at '" << path
              << "' — encoders will cold-start (degraded until next save)"
              << std::endl;
    return false;
  }

  std::string line;
  int loaded = 0;
  while (std::getline(ifs, line))
  {
    auto hash = line.find('#');
    if (hash != std::string::npos)
      line.erase(hash);
    std::istringstream iss(line);
    std::string id_token, magic_token, crc_token;
    int64_t total_pulses = 0;
    int32_t raw_abs = 0;
    uint32_t seq = 0;
    if (!(iss >> id_token >> magic_token >> total_pulses >> raw_abs >> seq >> crc_token))
      continue;

    const uint32_t can_id = static_cast<uint32_t>(
      std::strtoul(id_token.c_str(), nullptr, 0));
    const uint32_t magic = static_cast<uint32_t>(
      std::strtoul(magic_token.c_str(), nullptr, 0));
    const uint32_t file_crc = static_cast<uint32_t>(
      std::strtoul(crc_token.c_str(), nullptr, 0));

    if (magic != ENC_PERSIST_MAGIC)
    {
      std::cerr << "[CHASSIS] unwrapped-pulses bad magic for id=0x" << std::hex
                << can_id << std::dec << ", skipped" << std::endl;
      continue;
    }
    const uint32_t calc_crc = encoder_persist_crc(can_id, total_pulses, raw_abs, seq);
    if (calc_crc != file_crc)
    {
      std::cerr << "[CHASSIS] unwrapped-pulses CRC mismatch for id=0x"
                << std::hex << can_id << std::dec
                << " (file=0x" << std::hex << file_crc
                << " calc=0x" << calc_crc << std::dec
                << "), skipped" << std::endl;
      continue;
    }

    for (int i = 0; i < STEER_NUM; ++i)
    {
      if (encoders[i].Get_Can_ID() == can_id)
      {
        encoders[i].Set_Pending_Restore(total_pulses, raw_abs);
        state.last_total[i] = total_pulses;
        state.last_raw[i] = raw_abs;
        state.last_valid[i] = true;
        state.seq[i] = seq;
        ++loaded;
        std::cout << "[CHASSIS] unwrapped-pulses pending id=0x" << std::hex
                  << can_id << std::dec
                  << " total=" << total_pulses
                  << " raw=" << raw_abs
                  << " seq=" << seq << std::endl;
        break;
      }
    }
  }

  std::cout << "[CHASSIS] steer unwrapped-pulses loaded: " << loaded << "/"
            << STEER_NUM << " from '" << path << "'" << std::endl;
  return loaded > 0;
}

bool SaveUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                         const std::string &path, uint32_t min_interval_ms)
{
  const uint64_t cur_ms = now_ms();
  if (state.last_save_ms != 0 &&
      (cur_ms - state.last_save_ms) < min_interval_ms)
    return false;

  // 变化检测
  bool any_changed = false;
  bool any_valid = false;
  int64_t snap_total[STEER_NUM] = {0};
  int32_t snap_raw[STEER_NUM] = {0};
  for (int i = 0; i < STEER_NUM; ++i)
  {
    if (!encoders[i].Is_Restore_Valid())
      continue;
    any_valid = true;
    snap_total[i] = encoders[i].Get_Total_Unwrapped_Pulses();
    snap_raw[i] = encoders[i].Get_Last_Raw_Abs();
    if (!state.last_valid[i] || state.last_total[i] != snap_total[i])
      any_changed = true;
  }
  if (!any_valid || !any_changed)
    return false;

  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs.is_open())
  {
    std::cerr << "[CHASSIS][PERSIST-FAIL] failed to open '" << path
              << "' for writing unwrapped pulses (will retry next tick, "
              "in-memory state preserved)" << std::endl;
    return false;
  }

  ofs << "# steer encoder unwrapped pulses (auto-generated, do not edit)\n"
      << "# format: <can_id> <magic_hex> <total_pulses> <raw_abs> <seq> <crc32_hex>\n";
  for (int i = 0; i < STEER_NUM; ++i)
  {
    if (!encoders[i].Is_Restore_Valid())
      continue;
    const uint32_t can_id = encoders[i].Get_Can_ID();
    const int64_t total = snap_total[i];
    const int32_t raw = snap_raw[i];
    const uint32_t seq = ++state.seq[i];
    const uint32_t crc = encoder_persist_crc(can_id, total, raw, seq);
    ofs << "0x" << std::hex << std::setw(2) << std::setfill('0') << can_id
        << std::dec << std::setfill(' ')
        << " 0x" << std::hex << ENC_PERSIST_MAGIC << std::dec
        << " " << total
        << " " << raw
        << " " << seq
        << " 0x" << std::hex << std::setw(8) << std::setfill('0') << crc
        << std::dec << std::setfill(' ')
        << "\n";

    state.last_total[i] = total;
    state.last_raw[i] = raw;
    state.last_valid[i] = true;
  }
  // ofstream 析构时 close，此处再次确认状态
  ofs.flush();
  if (!ofs.good())
  {
    std::cerr << "[CHASSIS][PERSIST-FAIL] write to '" << path
              << "' reported error after flush; data may be partial" << std::endl;
    return false;
  }
  state.last_save_ms = cur_ms;
  return true;
}

bool ForceSaveUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                              const std::string &path, const char *reason)
{
  state.last_save_ms = 0;
  // 强制写盘需绕过"无变化跳过"。把上次快照置无效，下面的 any_changed 必然为真。
  for (int i = 0; i < STEER_NUM; ++i)
    state.last_valid[i] = false;
  const bool ok = SaveUnwrappedPulses(encoders, state, path, /*min_interval_ms=*/0);
  std::cout << "[CHASSIS][PERSIST-FORCE] reason="
            << (reason ? reason : "(none)")
            << " result=" << (ok ? "OK" : "FAIL") << std::endl;
  return ok;
}

}  // namespace encoder_persistence
