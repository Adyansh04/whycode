#include "whycode/utils/coord_lut.hpp"

#include <cstddef>

namespace whycon::lut
{

aligned_vec<int>   X_OF_IDX;
aligned_vec<int>   Y_OF_IDX;
aligned_vec<float> XF_OF_IDX;
aligned_vec<float> YF_OF_IDX;
aligned_vec<int>   ROW_START;
aligned_vec<int>   LEFT_OF;
aligned_vec<int>   RIGHT_OF;
aligned_vec<int>   UP_OF;
aligned_vec<int>   DOWN_OF;

int    WIDTH  = 0;
int    HEIGHT = 0;
size_t SIZE   = 0;

void ensure(int width, int height)
{
    if (width == WIDTH && height == HEIGHT && SIZE != 0)
    {
        return;  // No change needed
    }

    WIDTH  = width;
    HEIGHT = height;
    SIZE   = static_cast<size_t>(width) * static_cast<size_t>(height);

    X_OF_IDX.resize(SIZE);
    Y_OF_IDX.resize(SIZE);
    XF_OF_IDX.resize(SIZE);
    YF_OF_IDX.resize(SIZE);
    ROW_START.resize(HEIGHT);

    LEFT_OF.resize(SIZE);
    RIGHT_OF.resize(SIZE);
    UP_OF.resize(SIZE);
    DOWN_OF.resize(SIZE);

    for (int y = 0; y < HEIGHT; ++y)
    {
        const int rs = y * WIDTH;
        ROW_START[y] = rs;
        for (int x = 0; x < WIDTH; ++x)
        {
            const int idx  = rs + x;
            X_OF_IDX[idx]  = x;
            Y_OF_IDX[idx]  = y;
            XF_OF_IDX[idx] = static_cast<float>(x);
            YF_OF_IDX[idx] = static_cast<float>(y);

            LEFT_OF[idx]  = (x - 1 >= 0) ? (idx - 1) : -1;
            RIGHT_OF[idx] = (x + 1 < WIDTH) ? (idx + 1) : -1;
            UP_OF[idx]    = (y - 1 >= 0) ? (idx - WIDTH) : -1;
            DOWN_OF[idx]  = (y + 1 < HEIGHT) ? (idx + WIDTH) : -1;
        }
    }
}
}  // namespace whycon::lut