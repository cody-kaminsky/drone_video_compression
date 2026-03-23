/*
 * common.h — shared types, constants, and frame structures
 *
 * Design rules (for future VHDL translation):
 *   - Fixed-width integer types only (int8_t, int16_t, int32_t, uint*)
 *   - No dynamic allocation (all buffers statically sized or caller-provided)
 *   - No floating point
 *   - No recursion
 */

#ifndef DRONE_CODEC_COMMON_H
#define DRONE_CODEC_COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Resolution limits                                                            */
/* -------------------------------------------------------------------------- */

#define MAX_WIDTH   3840
#define MAX_HEIGHT  2160

/* Macroblocks are 16x16 luma pixels (4:2:0: 8x8 chroma per MB)              */
#define MB_SIZE     16
#define MAX_MB_COLS (MAX_WIDTH  / MB_SIZE)   /* 240 */
#define MAX_MB_ROWS (MAX_HEIGHT / MB_SIZE)   /* 135 */
#define MAX_MBS     (MAX_MB_COLS * MAX_MB_ROWS)

/* Each 16x16 MB contains four 8x8 luma transform blocks + 1 Cb + 1 Cr       */
#define BLOCK_SIZE  8
#define LUMA_BLOCKS 4
#define TOTAL_BLOCKS_PER_MB 6   /* 4 Y + 1 Cb + 1 Cr */

/* -------------------------------------------------------------------------- */
/* Codec identifier / stream magic                                              */
/* -------------------------------------------------------------------------- */

#define CODEC_MAGIC  "DRN1"
#define CODEC_MAGIC_LEN 4

/* -------------------------------------------------------------------------- */
/* Frame types                                                                  */
/* -------------------------------------------------------------------------- */

typedef enum {
    FRAME_I = 0,   /* Intra — no reference needed, random access point        */
    FRAME_P = 1,   /* Predicted — references previous decoded frame           */
} FrameType;

/* -------------------------------------------------------------------------- */
/* Intra prediction modes (4 modes — FPGA-friendly subset of H.265)           */
/* -------------------------------------------------------------------------- */

typedef enum {
    INTRA_DC      = 0,   /* Average of available neighbors                   */
    INTRA_HORIZ   = 1,   /* Copy left column across                          */
    INTRA_VERT    = 2,   /* Copy top row downward                            */
    INTRA_PLANAR  = 3,   /* Bilinear ramp                                    */
} IntraMode;

#define NUM_INTRA_MODES 4

/* -------------------------------------------------------------------------- */
/* Quantisation parameter                                                       */
/* -------------------------------------------------------------------------- */

#define QP_MIN  1
#define QP_MAX  51
#define QP_DEFAULT 28

/* -------------------------------------------------------------------------- */
/* Stream header (written once at start of file)                               */
/* 12 bytes, fixed                                                              */
/* -------------------------------------------------------------------------- */

typedef struct {
    char     magic[4];      /* "DRN1"                                         */
    uint16_t width;         /* frame width in pixels                          */
    uint16_t height;        /* frame height in pixels                         */
    uint8_t  fps;           /* frames per second                              */
    uint8_t  qp;            /* global quantisation parameter                  */
    uint8_t  reserved[2];
} __attribute__((packed)) StreamHeader;

/* -------------------------------------------------------------------------- */
/* Frame header (written before each frame's bitstream)                        */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  frame_type;    /* FRAME_I or FRAME_P                             */
    uint32_t frame_size;    /* bytes of bitstream data following this header  */
} __attribute__((packed)) FrameHeader;

/* -------------------------------------------------------------------------- */
/* Plane buffer — one component (Y, Cb, or Cr)                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t *data;          /* pointer to pixel data (caller owns)            */
    int      width;
    int      height;
    int      stride;        /* bytes per row (may be padded)                  */
} Plane;

/* -------------------------------------------------------------------------- */
/* YUV 4:2:0 frame                                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
    Plane    y;
    Plane    cb;
    Plane    cr;
    int      width;         /* luma width                                     */
    int      height;        /* luma height                                    */
    int      frame_index;
    FrameType type;
} Frame;

/* -------------------------------------------------------------------------- */
/* Macroblock position                                                          */
/* -------------------------------------------------------------------------- */

typedef struct {
    int mb_x;   /* macroblock column index                                    */
    int mb_y;   /* macroblock row index                                       */
    int px_x;   /* pixel x = mb_x * MB_SIZE                                  */
    int px_y;   /* pixel y = mb_y * MB_SIZE                                  */
} MBPos;

/* -------------------------------------------------------------------------- */
/* Motion vector (half-pixel units — multiply by 0.5 for actual offset)       */
/* -------------------------------------------------------------------------- */

typedef struct {
    int16_t dx;   /* horizontal displacement in half-pixel units              */
    int16_t dy;   /* vertical displacement in half-pixel units                */
} MotionVector;

/* -------------------------------------------------------------------------- */
/* Encoded macroblock descriptor                                                */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t      skip;           /* P-frame: 1 = copy reference, no residual  */
    IntraMode    intra_mode;     /* I-frame only                              */
    MotionVector mv;             /* P-frame only                              */
    uint8_t      has_residual;   /* P-frame: 1 if residual coded              */
    /* transform coefficients [block][64], in zig-zag scan order              */
    int16_t      coeff[TOTAL_BLOCKS_PER_MB][BLOCK_SIZE * BLOCK_SIZE];
} MBData;

/* -------------------------------------------------------------------------- */
/* Utility macros                                                               */
/* -------------------------------------------------------------------------- */

#define CLIP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define CLIP8(v)         CLIP(v, 0, 255)
#define ABS(x)           ((x) < 0 ? -(x) : (x))
#define MIN(a, b)        ((a) < (b) ? (a) : (b))
#define MAX(a, b)        ((a) > (b) ? (a) : (b))
#define ROUND_DIV(n, d)  (((n) + ((d) >> 1)) / (d))

/* Number of MBs in each dimension */
#define MB_COLS(w)  (((w) + MB_SIZE - 1) / MB_SIZE)
#define MB_ROWS(h)  (((h) + MB_SIZE - 1) / MB_SIZE)

/* -------------------------------------------------------------------------- */
/* Zig-zag scan order for 8x8 block (matches JPEG/H.264 standard)             */
/* -------------------------------------------------------------------------- */

static const uint8_t ZIGZAG_8x8[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

/* Inverse zig-zag: position in natural order → position in zig-zag order    */
static const uint8_t IZIGZAG_8x8[64] = {
     0,  1,  5,  6, 14, 15, 27, 28,
     2,  4,  7, 13, 16, 26, 29, 42,
     3,  8, 12, 17, 25, 30, 41, 43,
     9, 11, 18, 24, 31, 40, 44, 53,
    10, 19, 23, 32, 39, 45, 52, 54,
    20, 22, 33, 38, 46, 51, 55, 60,
    21, 34, 37, 47, 50, 56, 59, 61,
    35, 36, 48, 49, 57, 58, 62, 63
};

#endif /* DRONE_CODEC_COMMON_H */
