#include "bitstream.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Writer                                                                       */
/* -------------------------------------------------------------------------- */

void bs_writer_init(BitstreamWriter *w, uint8_t *buf, size_t capacity)
{
    w->buf       = buf;
    w->capacity  = capacity;
    w->byte_pos  = 0;
    w->bit_buf   = 0;
    w->bit_count = 0;
    w->overflow  = 0;
}

void bs_write_bit(BitstreamWriter *w, int bit)
{
    w->bit_buf = (uint8_t)((w->bit_buf << 1) | (bit & 1));
    w->bit_count++;
    if (w->bit_count == 8) {
        if (w->byte_pos >= w->capacity) {
            w->overflow = 1;
            return;
        }
        w->buf[w->byte_pos++] = w->bit_buf;
        w->bit_buf   = 0;
        w->bit_count = 0;
    }
}

void bs_write_bits(BitstreamWriter *w, uint32_t val, int nbits)
{
    for (int i = nbits - 1; i >= 0; i--)
        bs_write_bit(w, (val >> i) & 1);
}

void bs_write_align(BitstreamWriter *w)
{
    while (w->bit_count != 0)
        bs_write_bit(w, 0);
}

size_t bs_writer_bytes(const BitstreamWriter *w)
{
    return w->byte_pos + (w->bit_count > 0 ? 1 : 0);
}

/*
 * Unsigned exponential-Golomb:
 *   encode n+1 in unary prefix (k+1 leading zeros then 1) where k=floor(log2(n+1))
 *   followed by k-bit suffix (remainder)
 *   val=0 → "1", val=1 → "010", val=2 → "011", val=3 → "00100", ...
 */
void bs_write_ue(BitstreamWriter *w, uint32_t val)
{
    uint32_t v = val + 1;
    int bits = 0;
    uint32_t tmp = v;
    while (tmp > 1) { tmp >>= 1; bits++; }
    /* write 'bits' leading zeros */
    for (int i = 0; i < bits; i++)
        bs_write_bit(w, 0);
    /* write v in (bits+1) bits */
    bs_write_bits(w, v, bits + 1);
}

/*
 * Signed exp-Golomb: map signed to unsigned
 *   0→0, 1→1, -1→2, 2→3, -2→4, ...
 */
void bs_write_se(BitstreamWriter *w, int32_t val)
{
    uint32_t uval;
    if (val <= 0)
        uval = (uint32_t)(-val * 2);
    else
        uval = (uint32_t)(val * 2 - 1);
    bs_write_ue(w, uval);
}

/* -------------------------------------------------------------------------- */
/* Reader                                                                       */
/* -------------------------------------------------------------------------- */

void bs_reader_init(BitstreamReader *r, const uint8_t *buf, size_t size)
{
    r->buf       = buf;
    r->size      = size;
    r->byte_pos  = 0;
    r->bit_buf   = 0;
    r->bit_count = 0;
    r->error     = 0;
}

int bs_read_bit(BitstreamReader *r)
{
    if (r->bit_count == 0) {
        if (r->byte_pos >= r->size) {
            r->error = 1;
            return 0;
        }
        r->bit_buf   = r->buf[r->byte_pos++];
        r->bit_count = 8;
    }
    r->bit_count--;
    return (r->bit_buf >> r->bit_count) & 1;
}

uint32_t bs_read_bits(BitstreamReader *r, int nbits)
{
    uint32_t val = 0;
    for (int i = 0; i < nbits; i++)
        val = (val << 1) | (uint32_t)bs_read_bit(r);
    return val;
}

void bs_read_align(BitstreamReader *r)
{
    r->bit_count = 0;
    r->bit_buf   = 0;
}

uint32_t bs_read_ue(BitstreamReader *r)
{
    int leading = 0;
    while (bs_read_bit(r) == 0 && !r->error)
        leading++;
    uint32_t val = bs_read_bits(r, leading);
    return (1u << leading) - 1 + val;
}

int32_t bs_read_se(BitstreamReader *r)
{
    uint32_t uval = bs_read_ue(r);
    if (uval & 1)
        return (int32_t)((uval + 1) >> 1);
    else
        return -(int32_t)(uval >> 1);
}
