/*
 * H.265/HEVC Video Decoder
 *
 * Decodes an H.265-encoded video file and writes raw YUV420p frames to disk
 * (or pipes them to stdout for further processing).
 *
 * Dependencies: FFmpeg (libavcodec, libavformat, libavutil, libswscale)
 *
 * Build (Linux/macOS):
 *   gcc -O2 -o h265_decoder h265_decoder.c \
 *       $(pkg-config --cflags --libs libavcodec libavformat libavutil libswscale)
 *
 * Build (Windows with vcpkg/msys2):
 *   See README.md
 *
 * Usage:
 *   ./h265_decoder <input.mp4|.mkv|.hevc> [output_dir]
 *
 *   If output_dir is omitted, frames are written to ./frames/
 *   Each frame is saved as frameXXXXXX.yuv (raw YUV420p planar).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>

/* -------------------------------------------------------------------------- */
/* Helpers                                                                      */
/* -------------------------------------------------------------------------- */

static void log_error(const char *msg, int errnum)
{
    char errbuf[128];
    av_strerror(errnum, errbuf, sizeof(errbuf));
    fprintf(stderr, "ERROR: %s: %s\n", msg, errbuf);
}

/* Portable mkdir (single level). Returns 0 on success or if dir exists. */
static int make_dir(const char *path)
{
#ifdef _MSC_VER
    if (_mkdir(path) != 0 && errno != EEXIST)
        return -1;
#else
    if (mkdir(path) != 0 && errno != EEXIST)
        return -1;
#endif
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Frame writer – raw YUV420p planar                                            */
/* -------------------------------------------------------------------------- */

static int write_yuv_frame(AVFrame *frame, int frame_index,
                            const char *out_dir,
                            struct SwsContext **sws_ctx,
                            AVFrame **yuv_frame)
{
    int ret = 0;
    int width  = frame->width;
    int height = frame->height;

    /* (Re)create scaler if needed — converts any pixel format to YUV420p */
    *sws_ctx = sws_getCachedContext(
        *sws_ctx,
        width, height, (enum AVPixelFormat)frame->format,
        width, height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, NULL, NULL, NULL);

    if (!*sws_ctx) {
        fprintf(stderr, "ERROR: could not initialize SwsContext\n");
        return AVERROR(ENOMEM);
    }

    /* Allocate output frame once (or if dimensions changed) */
    if (!*yuv_frame || (*yuv_frame)->width != width ||
                       (*yuv_frame)->height != height) {
        av_frame_free(yuv_frame);
        *yuv_frame = av_frame_alloc();
        if (!*yuv_frame)
            return AVERROR(ENOMEM);

        (*yuv_frame)->format = AV_PIX_FMT_YUV420P;
        (*yuv_frame)->width  = width;
        (*yuv_frame)->height = height;

        ret = av_frame_get_buffer(*yuv_frame, 32);
        if (ret < 0) {
            log_error("av_frame_get_buffer", ret);
            return ret;
        }
    }

    sws_scale(*sws_ctx,
              (const uint8_t * const *)frame->data, frame->linesize,
              0, height,
              (*yuv_frame)->data, (*yuv_frame)->linesize);

    /* Build output path */
    char path[1024];
    snprintf(path, sizeof(path), "%s/frame%06d.yuv", out_dir, frame_index);

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }

    /* Write Y plane */
    for (int y = 0; y < height; y++)
        fwrite((*yuv_frame)->data[0] + y * (*yuv_frame)->linesize[0], 1, width, f);
    /* Write U plane (half width, half height) */
    for (int y = 0; y < height / 2; y++)
        fwrite((*yuv_frame)->data[1] + y * (*yuv_frame)->linesize[1], 1, width / 2, f);
    /* Write V plane */
    for (int y = 0; y < height / 2; y++)
        fwrite((*yuv_frame)->data[2] + y * (*yuv_frame)->linesize[2], 1, width / 2, f);

    fclose(f);
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Decode loop                                                                   */
/* -------------------------------------------------------------------------- */

static int decode_frames(AVCodecContext *codec_ctx, AVPacket *pkt,
                          AVFrame *frame, int *frame_count,
                          const char *out_dir,
                          struct SwsContext **sws_ctx,
                          AVFrame **yuv_frame)
{
    int ret = avcodec_send_packet(codec_ctx, pkt);
    if (ret < 0) {
        log_error("avcodec_send_packet", ret);
        return ret;
    }

    while (ret >= 0) {
        ret = avcodec_receive_frame(codec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return 0;  /* need more data or stream ended */
        if (ret < 0) {
            log_error("avcodec_receive_frame", ret);
            return ret;
        }

        /* Print progress every 100 frames */
        if (*frame_count % 100 == 0) {
            printf("  decoded frame %d  (%dx%d, pts=%" PRId64 ")\n",
                   *frame_count, frame->width, frame->height, frame->pts);
            fflush(stdout);
        }

        ret = write_yuv_frame(frame, *frame_count, out_dir, sws_ctx, yuv_frame);
        if (ret < 0)
            return ret;

        (*frame_count)++;
        av_frame_unref(frame);
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Main                                                                         */
/* -------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <input_file> [output_dir]\n", argv[0]);
        fprintf(stderr, "  input_file  : H.265/HEVC encoded file (.mp4, .mkv, .hevc, ...)\n");
        fprintf(stderr, "  output_dir  : directory for raw YUV frames (default: ./frames)\n");
        return 1;
    }

    const char *input_path = argv[1];
    const char *out_dir    = (argc >= 3) ? argv[2] : "frames";

    /* Create output directory */
    if (make_dir(out_dir) != 0) {
        fprintf(stderr, "ERROR: cannot create directory '%s': %s\n",
                out_dir, strerror(errno));
        return 1;
    }

    printf("H.265/HEVC Decoder\n");
    printf("  Input  : %s\n", input_path);
    printf("  Output : %s/\n", out_dir);

    /* ------------------------------------------------------------------ */
    /* 1. Open input file / find streams                                    */
    /* ------------------------------------------------------------------ */

    AVFormatContext *fmt_ctx = NULL;
    int ret = avformat_open_input(&fmt_ctx, input_path, NULL, NULL);
    if (ret < 0) {
        log_error("avformat_open_input", ret);
        return 1;
    }

    ret = avformat_find_stream_info(fmt_ctx, NULL);
    if (ret < 0) {
        log_error("avformat_find_stream_info", ret);
        avformat_close_input(&fmt_ctx);
        return 1;
    }

    printf("\nContainer info:\n");
    av_dump_format(fmt_ctx, 0, input_path, 0);

    /* ------------------------------------------------------------------ */
    /* 2. Find the best H.265 video stream                                  */
    /* ------------------------------------------------------------------ */

    int video_stream_idx = av_find_best_stream(
        fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);

    if (video_stream_idx < 0) {
        fprintf(stderr, "ERROR: no video stream found in '%s'\n", input_path);
        avformat_close_input(&fmt_ctx);
        return 1;
    }

    AVStream *video_stream = fmt_ctx->streams[video_stream_idx];
    enum AVCodecID codec_id = video_stream->codecpar->codec_id;

    printf("\nVideo stream #%d: codec=%s  %dx%d\n",
           video_stream_idx,
           avcodec_get_name(codec_id),
           video_stream->codecpar->width,
           video_stream->codecpar->height);

    if (codec_id != AV_CODEC_ID_HEVC) {
        fprintf(stderr,
                "WARNING: stream codec is '%s', not HEVC/H.265. "
                "Proceeding anyway.\n",
                avcodec_get_name(codec_id));
    }

    /* ------------------------------------------------------------------ */
    /* 3. Open decoder                                                       */
    /* ------------------------------------------------------------------ */

    const AVCodec *codec = avcodec_find_decoder(codec_id);
    if (!codec) {
        fprintf(stderr, "ERROR: no decoder found for codec '%s'\n",
                avcodec_get_name(codec_id));
        avformat_close_input(&fmt_ctx);
        return 1;
    }

    AVCodecContext *codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        fprintf(stderr, "ERROR: avcodec_alloc_context3 failed\n");
        avformat_close_input(&fmt_ctx);
        return 1;
    }

    ret = avcodec_parameters_to_context(codec_ctx, video_stream->codecpar);
    if (ret < 0) {
        log_error("avcodec_parameters_to_context", ret);
        goto cleanup;
    }

    /* Use multiple threads for faster decoding */
    codec_ctx->thread_count = 0;  /* 0 = auto-detect */
    codec_ctx->thread_type  = FF_THREAD_FRAME | FF_THREAD_SLICE;

    ret = avcodec_open2(codec_ctx, codec, NULL);
    if (ret < 0) {
        log_error("avcodec_open2", ret);
        goto cleanup;
    }

    printf("Decoder opened: %s  (threads: %d)\n",
           codec->long_name, codec_ctx->thread_count);

    /* ------------------------------------------------------------------ */
    /* 4. Allocate packet / frame buffers                                   */
    /* ------------------------------------------------------------------ */

    AVPacket *pkt   = av_packet_alloc();
    AVFrame  *frame = av_frame_alloc();
    if (!pkt || !frame) {
        fprintf(stderr, "ERROR: allocation failed\n");
        ret = AVERROR(ENOMEM);
        goto cleanup;
    }

    struct SwsContext *sws_ctx  = NULL;
    AVFrame           *yuv_frame = NULL;
    int frame_count = 0;

    /* ------------------------------------------------------------------ */
    /* 5. Read & decode packets                                             */
    /* ------------------------------------------------------------------ */

    printf("\nDecoding...\n");

    while (av_read_frame(fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == video_stream_idx) {
            ret = decode_frames(codec_ctx, pkt, frame, &frame_count,
                                out_dir, &sws_ctx, &yuv_frame);
            if (ret < 0)
                break;
        }
        av_packet_unref(pkt);
    }

    /* Flush decoder (drain buffered frames) */
    if (ret >= 0) {
        av_packet_unref(pkt);  /* send NULL packet to flush */
        pkt->data = NULL;
        pkt->size = 0;
        decode_frames(codec_ctx, pkt, frame, &frame_count,
                      out_dir, &sws_ctx, &yuv_frame);
    }

    printf("\nDone. %d frames decoded to %s/\n", frame_count, out_dir);
    printf("Frame size: %dx%d YUV420p\n",
           codec_ctx->width, codec_ctx->height);
    printf("To play frames with FFmpeg:\n");
    printf("  ffplay -f rawvideo -pix_fmt yuv420p -video_size %dx%d %s/frame%%06d.yuv\n",
           codec_ctx->width, codec_ctx->height, out_dir);

    /* ------------------------------------------------------------------ */
    /* Cleanup                                                               */
    /* ------------------------------------------------------------------ */

    sws_freeContext(sws_ctx);
    av_frame_free(&yuv_frame);
    av_frame_free(&frame);
    av_packet_free(&pkt);

cleanup:
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&fmt_ctx);

    return (ret < 0) ? 1 : 0;
}
