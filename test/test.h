/*
 * test.h — minimal unit test framework
 */

#ifndef DRONE_TEST_H
#define DRONE_TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

static int _test_pass = 0;
static int _test_fail = 0;
static const char *_cur_test = "";

#define TEST_BEGIN(name) \
    do { _cur_test = (name); printf("  %-50s", (name)); fflush(stdout); } while(0)

#define TEST_END() \
    do { printf("PASS\n"); _test_pass++; } while(0)

#define ASSERT(cond) \
    do { if (!(cond)) { \
        printf("FAIL\n    Assertion failed: %s\n    at %s:%d\n", \
               #cond, __FILE__, __LINE__); \
        _test_fail++; return; \
    } } while(0)

#define ASSERT_EQ(a, b) \
    do { if ((a) != (b)) { \
        printf("FAIL\n    %s == %s  =>  %lld != %lld\n    at %s:%d\n", \
               #a, #b, (long long)(a), (long long)(b), __FILE__, __LINE__); \
        _test_fail++; return; \
    } } while(0)

#define ASSERT_NEAR(a, b, tol) \
    do { long long _d = (long long)(a) - (long long)(b); \
         if (_d < 0) _d = -_d; \
         if (_d > (tol)) { \
            printf("FAIL\n    |%s - %s| = %lld > %lld\n    at %s:%d\n", \
                   #a, #b, _d, (long long)(tol), __FILE__, __LINE__); \
            _test_fail++; return; \
    } } while(0)

#define TEST_SUMMARY() \
    do { \
        printf("\n--- %d passed, %d failed ---\n", _test_pass, _test_fail); \
        return (_test_fail > 0) ? 1 : 0; \
    } while(0)

/* PSNR between two planes, stride-aware (only compares w*h valid pixels) */
static double psnr(const uint8_t *a, int stride_a,
                   const uint8_t *b, int stride_b,
                   int w, int h)
{
    double mse = 0.0;
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++) {
            double d = (double)a[r * stride_a + c] - (double)b[r * stride_b + c];
            mse += d * d;
        }
    mse /= (w * h);
    if (mse < 1e-10) return 100.0;
    return 10.0 * log10(255.0 * 255.0 / mse);
}

#endif /* DRONE_TEST_H */
