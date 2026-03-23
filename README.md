# H.265/HEVC Video Decoder

A C program that decompresses H.265/HEVC encoded HD video using FFmpeg's
`libavcodec` decoder. Reads any container (`.mp4`, `.mkv`, `.hevc`, `.ts`, ...)
and writes raw YUV420p frames to disk.

## How it works

```
Input file
   │
   ▼
avformat  ──► demux packets from container
   │
   ▼
avcodec   ──► HEVC bitstream → decoded YUV frame
   │             (multi-threaded, frame + slice parallelism)
   ▼
swscale   ──► convert any pixel format → YUV420p
   │
   ▼
frames/frameNNNNNN.yuv  (raw planar output)
```

The raw YUV output can be piped into any downstream tool:
- display with `ffplay`
- re-encode with `ffmpeg`
- process frame-by-frame in custom code

## Building

### Windows (easiest — MSYS2 / MinGW-w64)

1. Download and install **MSYS2**: https://www.msys2.org/
2. Open the **MSYS2 MINGW64** shell and run:
   ```bash
   pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-ffmpeg
   gcc -O2 -o h265_decoder.exe h265_decoder.c \
       $(pkg-config --cflags --libs libavcodec libavformat libavutil libswscale)
   ```

### Windows (vcpkg + MSVC)

1. Install [vcpkg](https://vcpkg.io/en/getting-started) and run:
   ```
   vcpkg install ffmpeg:x64-windows
   ```
2. Set `VCPKG_ROOT` and run `build_windows.bat`.

### Linux / macOS

```bash
# Install FFmpeg dev libraries
sudo apt install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev  # Debian/Ubuntu
brew install ffmpeg                                                             # macOS

# Build
make

# Or manually:
gcc -O2 -o h265_decoder h265_decoder.c \
    $(pkg-config --cflags --libs libavcodec libavformat libavutil libswscale)
```

## Usage

```
./h265_decoder <input_file> [output_dir]
```

| Argument      | Description                                      |
|---------------|--------------------------------------------------|
| `input_file`  | H.265-encoded video (`.mp4`, `.mkv`, `.hevc`, …) |
| `output_dir`  | Directory for YUV frames (default: `./frames`)   |

### Examples

```bash
# Decode to ./frames/
./h265_decoder video.mp4

# Decode to a specific directory
./h265_decoder video.mkv /tmp/decoded_frames

# Play decoded frames with FFplay (replace WxH with your resolution)
ffplay -f rawvideo -pix_fmt yuv420p -video_size 1920x1080 frames/frame%06d.yuv

# Re-encode decoded frames to H.264 with FFmpeg
ffmpeg -f rawvideo -pix_fmt yuv420p -video_size 1920x1080 -r 30 \
       -i frames/frame%06d.yuv -c:v libx264 output.mp4
```

## Output format

Each `.yuv` file is raw **YUV420p planar**:
- **Y plane**: `width × height` bytes
- **U plane**: `(width/2) × (height/2)` bytes
- **V plane**: `(width/2) × (height/2)` bytes

Total bytes per frame = `width × height × 3/2`

## Key FFmpeg APIs used

| API | Purpose |
|-----|---------|
| `avformat_open_input` | Open container file |
| `avformat_find_stream_info` | Probe stream metadata |
| `av_find_best_stream` | Select best video stream |
| `avcodec_find_decoder` | Look up HEVC decoder |
| `avcodec_open2` | Initialize decoder with threading |
| `avcodec_send_packet` | Push compressed packet into decoder |
| `avcodec_receive_frame` | Pull decoded frame out |
| `sws_scale` | Pixel format conversion |
