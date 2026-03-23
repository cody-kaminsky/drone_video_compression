# codec.mk — shared compilation rules included by all Makefiles

CC     := gcc
CFLAGS := -O2 -Wall -Wextra -std=c11 \
          -Wno-unused-parameter \
          -I$(ROOT)

CODEC_SRCS := \
    $(ROOT)/codec/bitstream.c \
    $(ROOT)/codec/dct.c       \
    $(ROOT)/codec/quant.c     \
    $(ROOT)/codec/predict.c   \
    $(ROOT)/codec/encoder.c   \
    $(ROOT)/codec/decoder.c

CODEC_OBJS := $(CODEC_SRCS:.c=.o)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
