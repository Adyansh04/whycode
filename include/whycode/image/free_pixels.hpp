#ifndef WHYCON_FREE_PIXELS_HPP
#define WHYCON_FREE_PIXELS_HPP

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace whycon
{
static_assert(std::endian::native == std::endian::little, "bit-packed words assume little endian");

namespace bits
{
/**
 * @brief Loads the 64-bit word of 64 pixels starting at bytes.
 * @param bytes First byte of the word; no alignment needed.
 * @return The word, pixel 0 in bit 0.
 */
inline uint64_t loadWord(const uint8_t* bytes) noexcept
{
    uint64_t word;
    std::memcpy(&word, bytes, sizeof(word));
    return word;
}

/**
 * @brief Mask of bits lo to hi of a 64-bit word, inclusive.
 * @param lo Lowest bit, 0-63.
 * @param hi Highest bit, lo-63.
 * @return The mask.
 */
constexpr uint64_t range(int lo, int hi) noexcept
{
    return (hi == 63 ? ~uint64_t{ 0 } : (uint64_t{ 1 } << (hi + 1)) - 1) & (~uint64_t{ 0 } << lo);
}

/**
 * @brief Sets or clears the bits of pixels x0 to x1, inclusive, in a bit-packed row.
 * @param row First byte of the row, whose length is a whole number of words.
 * @param x0 First pixel.
 * @param x1 Last pixel.
 * @param value True to set the bits, false to clear them.
 */
inline void set(uint8_t* row, int x0, int x1, bool value) noexcept
{
    for (int w = x0 >> 6; w <= x1 >> 6; w++)
    {
        const uint64_t mask = range(std::max(x0 - w * 64, 0), std::min(x1 - w * 64, 63));
        const uint64_t word = loadWord(row + w * 8);
        const uint64_t out  = value ? word | mask : word & ~mask;
        std::memcpy(row + w * 8, &out, sizeof(out));
    }
}
}  // namespace bits

/**
 * @brief Pixels of one colour that no segment has taken yet, searched 64 at a time.
 *
 * Reads a binary image and a visited bitset in PackedBinaryImage's layout: LSB-first rows of whole
 * 64-bit words, a set binary bit is black, a set visited bit is taken. The bits past the image must
 * never be free: binary padding bits are 0 and visited padding bits are 1. Header-only on purpose:
 * the flood fill calls these per run, and out of line (whycon_core is a shared library, so its
 * functions are not inlined across files) the detector took 28% longer.
 */
class FreePixels
{
public:
    /**
     * @brief Views the two bit planes; nothing is copied.
     * @param binary Bit-packed binary image (PackedBinaryImage::data()).
     * @param visited Visited bitset in the same layout.
     * @param width Image width in pixels.
     * @param stride Row stride in bytes, a whole number of words (PackedBinaryImage::strideFor).
     * @param black True to search black pixels, false for white ones.
     */
    FreePixels(const uint8_t* binary, const uint8_t* visited, int width, size_t stride, bool black)
      : binary_(binary)
      , visited_(visited)
      , width_(width)
      , words_(static_cast<int>(stride / 8))
      , stride_(stride)
      , black_(black)
    {}

    /**
     * @brief Free pixels in 64-pixel word w of row y.
     * @return One bit per pixel, pixel w * 64 in bit 0.
     */
    uint64_t word(int y, int w) const noexcept
    {
        const size_t   offset = static_cast<size_t>(y) * stride_ + static_cast<size_t>(w) * 8;
        const uint64_t black  = bits::loadWord(binary_ + offset);
        return (black_ ? black : ~black) & ~bits::loadWord(visited_ + offset);
    }

    /**
     * @brief Whether pixel (x, y) is free.
     * @return True if it has the searched colour and is not taken.
     */
    bool isFree(int y, int x) const noexcept { return (word(y, x >> 6) >> (x & 63)) & 1; }

    /**
     * @brief First free pixel of row y in [x, x_last].
     * @return Its column, or -1 if there is none.
     */
    int next(int y, int x, int x_last) const noexcept
    {
        for (int w = x >> 6; w <= x_last >> 6; w++)
        {
            uint64_t free = word(y, w);
            if (w == x >> 6)
            {
                free &= ~uint64_t{ 0 } << (x & 63);
            }
            if (w == x_last >> 6)
            {
                free &= bits::range(0, x_last & 63);
            }
            if (free)
            {
                return w * 64 + std::countr_zero(free);
            }
        }
        return -1;
    }

    /**
     * @brief Last pixel of the free run through the free pixel x of row y.
     * @return Its column.
     */
    int runEnd(int y, int x) const noexcept
    {
        const uint64_t stop = ~word(y, x >> 6) & (~uint64_t{ 0 } << (x & 63));
        return stop ? (x & ~63) + std::countr_zero(stop) - 1 : endAfter(y, x >> 6);
    }

    /**
     * @brief First pixel of the free run through the free pixel x of row y.
     * @return Its column.
     */
    int runStart(int y, int x) const noexcept
    {
        const uint64_t stop = ~word(y, x >> 6) & ((uint64_t{ 1 } << (x & 63)) - 1);
        return stop ? (x & ~63) + 64 - std::countl_zero(stop) : startBefore(y, x >> 6);
    }

    /**
     * @brief Calls take(first, last) for each free run of row y that overlaps [x0, x1], left to
     * right. A run may reach past either end. Most runs lie in one word, which is then read once.
     * @param take Callable taking the run's first and last column; it may mark the run taken.
     */
    template <typename Take>
    void forEachRun(int y, int x0, int x1, Take&& take) const noexcept
    {
        int      w    = x0 >> 6;
        uint64_t full = word(y, w);
        uint64_t free = full & (~uint64_t{ 0 } << (x0 & 63));
        for (;;)
        {
            while (free == 0)
            {
                if (++w > x1 >> 6)
                {
                    return;
                }
                full = free = word(y, w);
            }
            const int bit = std::countr_zero(free);
            if (w * 64 + bit > x1)
            {
                return;
            }
            const uint64_t below = ~full & ((uint64_t{ 1 } << bit) - 1);
            const uint64_t above = ~full & (~uint64_t{ 0 } << bit);
            const int first = below ? w * 64 + 64 - std::countl_zero(below) : startBefore(y, w);
            const int last  = above ? w * 64 + std::countr_zero(above) - 1 : endAfter(y, w);
            take(first, last);
            if (last >= x1)
            {
                return;
            }
            // Bits up to last are now taken, and the search goes on past them
            if ((last + 1) >> 6 != w)
            {
                w    = (last + 1) >> 6;
                full = word(y, w);
            }
            free = full & (~uint64_t{ 0 } << ((last + 1) & 63));
        }
    }

private:
    // First pixel of a free run that goes on before word w
    int startBefore(int y, int w) const noexcept
    {
        while (--w >= 0)
        {
            if (const uint64_t stop = ~word(y, w))
            {
                return w * 64 + 64 - std::countl_zero(stop);
            }
        }
        return 0;
    }

    // Last pixel of a free run that goes on past word w
    int endAfter(int y, int w) const noexcept
    {
        while (++w < words_)
        {
            if (const uint64_t stop = ~word(y, w))
            {
                return w * 64 + std::countr_zero(stop) - 1;
            }
        }
        return width_ - 1;
    }

    const uint8_t* binary_;
    const uint8_t* visited_;
    int            width_, words_;
    size_t         stride_;
    bool           black_;
};
}  // namespace whycon

#endif  // WHYCON_FREE_PIXELS_HPP
