/*
 * bitstream.h — bit-level read/write into a byte buffer
 *
 * FPGA note: the writer maps to a shift register with parallel load;
 *            the reader maps to a barrel shifter.
 */

#ifndef DRONE_CODEC_BITSTREAM_H
#define DRONE_CODEC_BITSTREAM_H

#include <stdint.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* Writer                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  *buf;       /* output byte buffer (caller-owned)                 */
    size_t    capacity;  /* buffer size in bytes                              */
    size_t    byte_pos;  /* current byte offset                               */
    uint8_t   bit_buf;   /* bits waiting to be flushed                       */
    int       bit_count; /* number of valid bits in bit_buf (0..7)            */
    int       overflow;  /* set to 1 if buffer would overflow                 */
} BitstreamWriter;

void bs_writer_init(BitstreamWriter *w, uint8_t *buf, size_t capacity);
void bs_write_bit(BitstreamWriter *w, int bit);
void bs_write_bits(BitstreamWriter *w, uint32_t val, int nbits);
void bs_write_align(BitstreamWriter *w);   /* pad to byte boundary with zeros */
size_t bs_writer_bytes(const BitstreamWriter *w); /* bytes written so far     */

/* Exponential-Golomb coding (unsigned and signed)                            */
void bs_write_ue(BitstreamWriter *w, uint32_t val);  /* unsigned exp-golomb   */
void bs_write_se(BitstreamWriter *w, int32_t  val);  /* signed exp-golomb     */

/* -------------------------------------------------------------------------- */
/* Reader                                                                       */
/* -------------------------------------------------------------------------- */

typedef struct {
    const uint8_t *buf;
    size_t         size;       /* buffer size in bytes                        */
    size_t         byte_pos;
    uint8_t        bit_buf;
    int            bit_count;  /* bits remaining in bit_buf                   */
    int            error;      /* set to 1 on read past end                   */
} BitstreamReader;

void     bs_reader_init(BitstreamReader *r, const uint8_t *buf, size_t size);
int      bs_read_bit(BitstreamReader *r);
uint32_t bs_read_bits(BitstreamReader *r, int nbits);
void     bs_read_align(BitstreamReader *r);
uint32_t bs_read_ue(BitstreamReader *r);   /* unsigned exp-golomb             */
int32_t  bs_read_se(BitstreamReader *r);   /* signed exp-golomb               */

#endif /* DRONE_CODEC_BITSTREAM_H */
