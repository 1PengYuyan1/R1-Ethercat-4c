#ifndef RT_TIMING_H
#define RT_TIMING_H

#include <time.h>
#include <stdint.h>
#include <stdbool.h>

// 纳秒级休眠，保证严格周期
void sync_absolute_time(struct timespec *ts, int cycle_ns);

// 统计和打印循环耗时，排查系统抖动
void analyze_loop_frequency(struct timespec start_ts, int warn_threshold_ns);

#endif // RT_TIMING_H