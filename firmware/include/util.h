/* ---------------------------------------------------------------------------
 * util.h -- freestanding replacements for the handful of libc functions this
 * firmware needs.
 *
 * The Homebrew arm-none-eabi-gcc on this machine ships WITHOUT newlib: there
 * is no libc.a and no <string.h>/<math.h>.  We therefore build with
 * -ffreestanding -nostdlib and link only libgcc.  GCC is still allowed to
 * emit calls to memcpy/memset/memmove/memcmp for struct copies and array
 * initialisers, so those four symbols must exist; util.c defines them.
 *
 * Square root uses __builtin_sqrtf, which compiles to a single VSQRT.F32
 * instruction on the Cortex-M4F (verified in the disassembly) as long as
 * -fno-math-errno is on -- so no libm is required either.
 * ------------------------------------------------------------------------- */

#ifndef UTIL_H
#define UTIL_H

#include <stddef.h>
#include <stdint.h>

void *memcpy(void *dst, const void *src, size_t n);
void *memmove(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);
int   memcmp(const void *a, const void *b, size_t n);
size_t strlen(const char *s);

/* 1/sqrt(x) using the FPU's hardware square root.  Madgwick's original code
 * used the Quake fast-inverse-sqrt hack because his target had no FPU; we do,
 * and a real VSQRT is both faster and exact here. */
static inline float inv_sqrtf(float x)
{
    return 1.0f / __builtin_sqrtf(x);
}

/* Compile-time assertion helper (C11 _Static_assert is available). */
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)

#endif /* UTIL_H */
