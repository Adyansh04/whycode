#ifndef COORD_LUT_HPP
#define COORD_LUT_HPP

#include <cstddef>
#include <vector>

#include "xsimd/xsimd.hpp"

namespace whycon::lut {
template <typename T>
using aligned_vec = std::vector<T, xsimd::aligned_allocator<T, xsimd::default_arch::alignment()>>;

// Public Globals
extern aligned_vec<int>   X_OF_IDX;
extern aligned_vec<int>   Y_OF_IDX;
extern aligned_vec<float> XF_OF_IDX;
extern aligned_vec<float> YF_OF_IDX;
extern aligned_vec<int>   ROW_START;
extern aligned_vec<int>   LEFT_OF;
extern aligned_vec<int>   RIGHT_OF;
extern aligned_vec<int>   UP_OF;
extern aligned_vec<int>   DOWN_OF;

extern int    WIDTH;
extern int    HEIGHT;
extern size_t SIZE;

// Build/rebuild only if (width, height) changed
void ensure(int width, int height);

// Fast inline helpers
inline int x_of(int idx) noexcept {
    return X_OF_IDX[idx];
}
inline int y_of(int idx) noexcept {
    return Y_OF_IDX[idx];
}
inline float xf_of(int idx) noexcept {
    return XF_OF_IDX[idx];
}
inline float yf_of(int idx) noexcept {
    return YF_OF_IDX[idx];
}
inline int index_of(int x, int y) noexcept {
    return (y * WIDTH + x);
}

// Neighbor access
inline int left_of(int idx) noexcept {
    return LEFT_OF[idx];
}
inline int right_of(int idx) noexcept {
    return RIGHT_OF[idx];
}
inline int up_of(int idx) noexcept {
    return UP_OF[idx];
}
inline int down_of(int idx) noexcept {
    return DOWN_OF[idx];
}

}  // namespace whycon::lut

#endif  // COORD_LUT_HPP