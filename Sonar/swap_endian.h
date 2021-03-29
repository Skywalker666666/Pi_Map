//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __swap_endian_H__
#define  __swap_endian_H__

#include <cstdint>
#include <climits>

template <typename T>
T swap_endian(T u)
{
    static_assert (CHAR_BIT == 8, "CHAR_BIT != 8");

    union
    {
        T u;
        unsigned char u8[sizeof(T)];
    } source, dest;

    source.u = u;

    for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

#define SwapInt16 swap_endian<int16_t>
#define SwapUInt16 swap_endian<uint16_t>
#define SwapInt32 swap_endian<int32_t>
#define SwapUInt32 swap_endian<uint32_t>


#endif   /* ----- #ifndef __swap_endian_H__  ----- */
