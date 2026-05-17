#include "rt_timing.h"
#include <stdio.h>
#include <inttypes.h>

// 把 *ts 推进 cycle_ns 后用 clock_nanosleep 绝对时刻睡眠，实现无累积漂移的固定周期
// ts 既是输入也是输出（下次循环复用），调用前需 clock_gettime 初始化一次
void sync_absolute_time(struct timespec *ts, int cycle_ns)
{
    ts->tv_nsec += cycle_ns;
    while (ts->tv_nsec >= 1000000000LL) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000LL;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL); 
}

// 测量本次循环耗时；超过 warn_threshold_ns 立刻告警，每 1000 次循环报一次峰值抖动
// start_ts 应是循环开头 clock_gettime 取的时戳；用于诊断 EtherCAT 实时线程是否被抢占
void analyze_loop_frequency(struct timespec start_ts, int warn_threshold_ns)
{
    static int loop_count = 0;
    static int64_t max_dt_ns = 0;
    struct timespec end_ts;
    
    clock_gettime(CLOCK_MONOTONIC, &end_ts);
    
    int64_t dt_ns = (int64_t)(end_ts.tv_sec - start_ts.tv_sec) * 1000000000LL + 
                    (end_ts.tv_nsec - start_ts.tv_nsec);
    
    if (dt_ns > max_dt_ns) max_dt_ns = dt_ns; 
    
    if (dt_ns > warn_threshold_ns) {
        printf("[WARN] High Jitter! Loop took %" PRId64 " ns\n", dt_ns);
    }
    
    if (++loop_count >= 1000) {
        printf("[DEBUG] PDO Thread ALIVE | Max Jitter in 1s: %" PRId64 " ns\n", max_dt_ns);
        max_dt_ns = 0; 
        loop_count = 0;
    }
}