#ifndef MATH_H
#define MATH_H

#include <cstdint>
#include <algorithm>
#include <cmath>

#ifndef CELSIUS_TO_KELVIN
#define CELSIUS_TO_KELVIN (273.15f)
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef CELSIUS_TO_KELVIN
#define CELSIUS_TO_KELVIN (273.15f)
#endif


// 新增：绝对值函数
template<typename T>
static inline T Math_Abs(T val) {
    return (val > 0) ? val : -val;
}
template<typename T>
static inline void Math_Constrain(T *obj, T min, T max) {
    if (*obj < min) *obj = min;
    else if (*obj > max) *obj = max;
}

static inline uint16_t Math_Endian_Reverse_16(void *ptr, uint16_t *out) {
    uint8_t *p = (uint8_t *)ptr;
    uint16_t res = (uint16_t)((p[0] << 8) | p[1]);
    if (out) *out = res;
    return res;
}

static inline float Math_Modulus_Normalization(float val, float modulus) {
    float res = fmodf(val, modulus);
    if (res > modulus / 2.0f) res -= modulus;
    if (res < -modulus / 2.0f) res += modulus;
    return res;
}

static inline float Math_Int_To_Float(int x, int x_min, int x_max, float y_min, float y_max) {
    if (x <= x_min) return y_min;
    if (x >= x_max) return y_max;
    return y_min + (y_max - y_min) * (float)(x - x_min) / (float)(x_max - x_min);
}

static inline int Math_Float_To_Int(float y, float y_min, float y_max, int x_min, int x_max) {
    if (y <= y_min) return x_min;
    if (y >= y_max) return x_max;
    return x_min + (int)((float)(x_max - x_min) * (y - y_min) / (y_max - y_min));
}

#endif 