//
// 舵向编码器持久化（零点偏移 + 多圈累计脉冲）—— 与 chassis 状态解耦的纯函数。
//
// 设计：
//   - 零点偏移 (zero_offsets.txt)：3 列文本 — can_id zero_offset [logical_zero_anchor]
//   - 多圈累计脉冲 (unwrapped_pulses.txt)：6 字段 — can_id magic total raw_abs seq crc32
//   - 限频 / last_total / seq 这种跨周期状态由调用方持有 (encoder_persistence::State)，
//     函数只读写传入的 encoders 数组与 state，不依赖 chassis 类本身。
//
// 文件格式与 dvc_encoder.h 中 ENC_PERSIST_MAGIC / pending_restore 接口一致；
// 不允许变更格式（var_data/*.txt 已存在的文件必须仍可读）。
//

#ifndef LINKX_SOEM_DEMO_ENCODER_PERSISTENCE_H
#define LINKX_SOEM_DEMO_ENCODER_PERSISTENCE_H

#include "dvc_encoder.h"

#include <cstdint>
#include <string>

#ifndef STEER_NUM
#define STEER_NUM 4
#endif

namespace encoder_persistence {

// 多圈累计值持久化的跨周期状态：限频窗口 + 上次落盘的快照与序号。
// 由调用方 (chassis) 持有；Save / Force 直接读写其字段。
struct State {
  int64_t last_total[STEER_NUM] = {0};
  int32_t last_raw[STEER_NUM] = {0};
  bool    last_valid[STEER_NUM] = {false, false, false, false};
  uint32_t seq[STEER_NUM] = {0};
  uint64_t last_save_ms = 0;
};

// === 零点偏移 (zero_offsets.txt) ===
// 加载：仅匹配到的 can_id 才会被覆盖；缺失的编码器保持现状。返回是否至少加载到一条。
bool LoadZeroOffsets(Class_Encoder_BRT (&encoders)[STEER_NUM], const std::string &path);

// 保存：要求每个编码器至少收到过一帧合法数据，否则失败返回 false（不写入）。
// 副作用：把当前 raw 设为 zero_offset、Capture_Logical_Zero_Anchor()。
bool SaveZeroOffsets(Class_Encoder_BRT (&encoders)[STEER_NUM], const std::string &path);

// === 多圈累计脉冲 (unwrapped_pulses.txt) ===
// 加载：magic 错 / CRC 错 / parse 错 → 该行跳过；成功 → Set_Pending_Restore + 同步 state。
bool LoadUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                         const std::string &path);

// 保存：限频 (默认 200ms) + 无变化跳过；首次未通过限频则直接 false。
// 仅在 Is_Restore_Valid 的编码器对象内才写入对应行。
bool SaveUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                         const std::string &path, uint32_t min_interval_ms = 200);

// 强制保存：绕过限频 / 无变化跳过。把 state.last_save_ms = 0 + last_valid 全清后调 Save。
// reason 仅作为 stdout 日志区分调用现场（找零完成 / 降级切换 / SIGINT 钩子等）。
bool ForceSaveUnwrappedPulses(Class_Encoder_BRT (&encoders)[STEER_NUM], State &state,
                              const std::string &path, const char *reason);

}  // namespace encoder_persistence

#endif
