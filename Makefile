# Makefile for h265_decoder
# Requires FFmpeg development libraries.
#
# Linux/macOS:   make
# Cross-compile: make CC=x86_64-w64-mingw32-gcc (MinGW target)

CC      := gcc
TARGET  := h265_decoder
SRC     := h265_decoder.c

# pkg-config pulls the right flags for the installed FFmpeg
PKGS    := libavcodec libavformat libavutil libswscale

CFLAGS  := -O2 -Wall -Wextra $(shell pkg-config --cflags $(PKGS))
LDFLAGS := $(shell pkg-config --libs   $(PKGS))

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)
