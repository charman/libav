/*
 * Copyright (c) 2012 Pavel Koshevoy <pkoshevoy at gmail dot com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * tempo scaling audio filter -- an implementation of WSOLA algorithm
 *
 * Based on MIT licensed yaeAudioTempoFilter.h and yaeAudioFragment.h
 * from Apprentice Video player by Pavel Koshevoy.
 * https://sourceforge.net/projects/apprenticevideo/
 *
 * An explanation of SOLA algorithm is available at
 * http://www.surina.net/article/time-and-pitch-scaling.html
 *
 * WSOLA is very similar to SOLA, only one major difference exists between
 * these algorithms.  SOLA shifts audio fragments along the output stream,
 * where as WSOLA shifts audio fragments along the input stream.
 *
 * The advantage of WSOLA algorithm is that the overlap region size is
 * always the same, therefore the blending function is constant and
 * can be precomputed.
 */

#include <float.h>
#include <fcntl.h>       //  O_RDONLY, O_NONBLOCK
#include "libavcodec/avfft.h"
#include "libavutil/audioconvert.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "avfilter.h"
#include "audio.h"
#include "formats.h"
#include "internal.h"

/**
 * A fragment of audio waveform
 */
typedef struct {
    // index of the first sample of this fragment in the overall waveform;
    // 0: input sample position
    // 1: output sample position
    int64_t position[2];

    // original packed multi-channel samples:
    uint8_t *data;

    // number of samples in this fragment:
    int nsamples;

    // rDFT transform of the down-mixed mono fragment, used for
    // fast waveform alignment via correlation in frequency domain:
    FFTSample *xdat;
} AudioFragment;

/**
 * Filter state machine states
 */
typedef enum {
    YAE_LOAD_FRAGMENT,
    YAE_ADJUST_POSITION,
    YAE_RELOAD_FRAGMENT,
    YAE_OUTPUT_OVERLAP_ADD,
    YAE_FLUSH_OUTPUT,
} FilterState;

/**
 * Filter state machine
 */
typedef struct {
    const AVClass *class;

    // ring-buffer of input samples, necessary because some times
    // input fragment position may be adjusted backwards:
    uint8_t *buffer;

    // ring-buffer maximum capacity, expressed in sample rate time base:
    int ring;

    // ring-buffer house keeping:
    int size;
    int head;
    int tail;

    // 0: input sample position corresponding to the ring buffer tail
    // 1: output sample position
    int64_t position[2];

    // sample format:
    enum AVSampleFormat format;

    // number of channels:
    int channels;

    // row of bytes to skip from one sample to next, across multple channels;
    // stride = (number-of-channels * bits-per-sample-per-channel) / 8
    int stride;

    // fragment window size, power-of-two integer:
    int window;

    // Hann window coefficients, for feathering
    // (blending) the overlapping fragment region:
    float *hann;

    // tempo scaling factor:
    double tempo;

    // cumulative alignment drift:
    int drift;

    // current/previous fragment ring-buffer:
    AudioFragment frag[2];

    // current fragment index:
    uint64_t nfrag;

    // current state:
    FilterState state;

    // for fast correlation calculation in frequency domain:
    RDFTContext *real_to_complex;
    RDFTContext *complex_to_real;
    FFTSample *correlation;

    // for managing AVFilterPad.request_frame and AVFilterPad.filter_samples
    int request_fulfilled;
    AVFilterBufferRef *dst_buffer;
    uint8_t *dst;
    uint8_t *dst_end;
    uint64_t nsamples_in;
    uint64_t nsamples_out;

    FILE *metronomepipe;
    int metronomepipe_fd;
    char *metronomepipe_str;
} TimewarpContext;

#define OFFSET(x) offsetof(TimewarpContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
// #define F AV_OPT_FLAG_FILTERING_PARAM
#define F 0
static const AVOption timewarp_options[] = {
    { "metronomepipe", "Name of FIFO paper used to receive metronome commands.", OFFSET(metronomepipe_str), AV_OPT_TYPE_STRING, .flags = A|F },
    { NULL },
};

static const AVClass timewarp_class = {
  .class_name = "timewarp filter",
  .item_name  = av_default_item_name,
  .option     = timewarp_options,
  .version    = LIBAVUTIL_VERSION_INT,
};



/**
 * Reset filter to initial state, do not deallocate existing local buffers.
 */
static void yae_clear(TimewarpContext *timewarp)
{
    timewarp->size = 0;
    timewarp->head = 0;
    timewarp->tail = 0;

    timewarp->drift = 0;
    timewarp->nfrag = 0;
    timewarp->state = YAE_LOAD_FRAGMENT;

    timewarp->position[0] = 0;
    timewarp->position[1] = 0;

    timewarp->frag[0].position[0] = 0;
    timewarp->frag[0].position[1] = 0;
    timewarp->frag[0].nsamples    = 0;

    timewarp->frag[1].position[0] = 0;
    timewarp->frag[1].position[1] = 0;
    timewarp->frag[1].nsamples    = 0;

    // shift left position of 1st fragment by half a window
    // so that no re-normalization would be required for
    // the left half of the 1st fragment:
    timewarp->frag[0].position[0] = -(int64_t)(timewarp->window / 2);
    timewarp->frag[0].position[1] = -(int64_t)(timewarp->window / 2);

    avfilter_unref_bufferp(&timewarp->dst_buffer);
    timewarp->dst     = NULL;
    timewarp->dst_end = NULL;

    timewarp->request_fulfilled = 0;
    timewarp->nsamples_in       = 0;
    timewarp->nsamples_out      = 0;
}

/**
 * Reset filter to initial state and deallocate all buffers.
 */
static void yae_release_buffers(TimewarpContext *timewarp)
{
    yae_clear(timewarp);

    av_freep(&timewarp->frag[0].data);
    av_freep(&timewarp->frag[1].data);
    av_freep(&timewarp->frag[0].xdat);
    av_freep(&timewarp->frag[1].xdat);

    av_freep(&timewarp->buffer);
    av_freep(&timewarp->hann);
    av_freep(&timewarp->correlation);

    av_rdft_end(timewarp->real_to_complex);
    timewarp->real_to_complex = NULL;

    av_rdft_end(timewarp->complex_to_real);
    timewarp->complex_to_real = NULL;
}

/* av_realloc is not aligned enough; fortunately, the data does not need to
 * be preserved */
#define RE_MALLOC_OR_FAIL(field, field_size)                    \
    do {                                                        \
        av_freep(&field);                                       \
        field = av_malloc(field_size);                          \
        if (!field) {                                           \
            yae_release_buffers(timewarp);                        \
            return AVERROR(ENOMEM);                             \
        }                                                       \
    } while (0)

/**
 * Prepare filter for processing audio data of given format,
 * sample rate and number of channels.
 */
static int yae_reset(TimewarpContext *timewarp,
                     enum AVSampleFormat format,
                     int sample_rate,
                     int channels)
{
    const int sample_size = av_get_bytes_per_sample(format);
    uint32_t nlevels  = 0;
    uint32_t pot;
    int i;

    timewarp->format   = format;
    timewarp->channels = channels;
    timewarp->stride   = sample_size * channels;

    // pick a segment window size:
    timewarp->window = sample_rate / 24;

    // adjust window size to be a power-of-two integer:
    nlevels = av_log2(timewarp->window);
    pot = 1 << nlevels;
    av_assert0(pot <= timewarp->window);

    if (pot < timewarp->window) {
        timewarp->window = pot * 2;
        nlevels++;
    }

    // initialize audio fragment buffers:
    RE_MALLOC_OR_FAIL(timewarp->frag[0].data, timewarp->window * timewarp->stride);
    RE_MALLOC_OR_FAIL(timewarp->frag[1].data, timewarp->window * timewarp->stride);
    RE_MALLOC_OR_FAIL(timewarp->frag[0].xdat, timewarp->window * sizeof(FFTComplex));
    RE_MALLOC_OR_FAIL(timewarp->frag[1].xdat, timewarp->window * sizeof(FFTComplex));

    // initialize rDFT contexts:
    av_rdft_end(timewarp->real_to_complex);
    timewarp->real_to_complex = NULL;

    av_rdft_end(timewarp->complex_to_real);
    timewarp->complex_to_real = NULL;

    timewarp->real_to_complex = av_rdft_init(nlevels + 1, DFT_R2C);
    if (!timewarp->real_to_complex) {
        yae_release_buffers(timewarp);
        return AVERROR(ENOMEM);
    }

    timewarp->complex_to_real = av_rdft_init(nlevels + 1, IDFT_C2R);
    if (!timewarp->complex_to_real) {
        yae_release_buffers(timewarp);
        return AVERROR(ENOMEM);
    }

    RE_MALLOC_OR_FAIL(timewarp->correlation, timewarp->window * sizeof(FFTComplex));

    timewarp->ring = timewarp->window * 3;
    RE_MALLOC_OR_FAIL(timewarp->buffer, timewarp->ring * timewarp->stride);

    // initialize the Hann window function:
    RE_MALLOC_OR_FAIL(timewarp->hann, timewarp->window * sizeof(float));

    for (i = 0; i < timewarp->window; i++) {
        double t = (double)i / (double)(timewarp->window - 1);
        double h = 0.5 * (1.0 - cos(2.0 * M_PI * t));
        timewarp->hann[i] = (float)h;
    }

    yae_clear(timewarp);
    return 0;
}

/*
static int yae_set_tempo(AVFilterContext *ctx, const char *arg_tempo)
{
    TimewarpContext *timewarp = ctx->priv;
    char   *tail = NULL;
    double tempo = av_strtod(arg_tempo, &tail);

    if (tail && *tail) {
        av_log(ctx, AV_LOG_ERROR, "Invalid tempo value '%s'\n", arg_tempo);
        return AVERROR(EINVAL);
    }

    if (tempo < 0.5 || tempo > 2.0) {
        av_log(ctx, AV_LOG_ERROR, "Tempo value %f exceeds [0.5, 2.0] range\n",
               tempo);
        return AVERROR(EINVAL);
    }

    timewarp->tempo = tempo;
    return 0;
}
*/

inline static AudioFragment *yae_curr_frag(TimewarpContext *timewarp)
{
    return &timewarp->frag[timewarp->nfrag % 2];
}

inline static AudioFragment *yae_prev_frag(TimewarpContext *timewarp)
{
    return &timewarp->frag[(timewarp->nfrag + 1) % 2];
}

/**
 * A helper macro for initializing complex data buffer with scalar data
 * of a given type.
 */
#define yae_init_xdat(scalar_type, scalar_max)                          \
    do {                                                                \
        const uint8_t *src_end = src +                                  \
            frag->nsamples * timewarp->channels * sizeof(scalar_type);    \
                                                                        \
        FFTSample *xdat = frag->xdat;                                   \
        scalar_type tmp;                                                \
                                                                        \
        if (timewarp->channels == 1) {                                    \
            for (; src < src_end; xdat++) {                             \
                tmp = *(const scalar_type *)src;                        \
                src += sizeof(scalar_type);                             \
                                                                        \
                *xdat = (FFTSample)tmp;                                 \
            }                                                           \
        } else {                                                        \
            FFTSample s, max, ti, si;                                   \
            int i;                                                      \
                                                                        \
            for (; src < src_end; xdat++) {                             \
                tmp = *(const scalar_type *)src;                        \
                src += sizeof(scalar_type);                             \
                                                                        \
                max = (FFTSample)tmp;                                   \
                s = FFMIN((FFTSample)scalar_max,                        \
                          (FFTSample)fabsf(max));                       \
                                                                        \
                for (i = 1; i < timewarp->channels; i++) {                \
                    tmp = *(const scalar_type *)src;                    \
                    src += sizeof(scalar_type);                         \
                                                                        \
                    ti = (FFTSample)tmp;                                \
                    si = FFMIN((FFTSample)scalar_max,                   \
                               (FFTSample)fabsf(ti));                   \
                                                                        \
                    if (s < si) {                                       \
                        s   = si;                                       \
                        max = ti;                                       \
                    }                                                   \
                }                                                       \
                                                                        \
                *xdat = max;                                            \
            }                                                           \
        }                                                               \
    } while (0)

/**
 * Initialize complex data buffer of a given audio fragment
 * with down-mixed mono data of appropriate scalar type.
 */
static void yae_downmix(TimewarpContext *timewarp, AudioFragment *frag)
{
    // shortcuts:
    const uint8_t *src = frag->data;

    // init complex data buffer used for FFT and Correlation:
    memset(frag->xdat, 0, sizeof(FFTComplex) * timewarp->window);

    if (timewarp->format == AV_SAMPLE_FMT_U8) {
        yae_init_xdat(uint8_t, 127);
    } else if (timewarp->format == AV_SAMPLE_FMT_S16) {
        yae_init_xdat(int16_t, 32767);
    } else if (timewarp->format == AV_SAMPLE_FMT_S32) {
        yae_init_xdat(int, 2147483647);
    } else if (timewarp->format == AV_SAMPLE_FMT_FLT) {
        yae_init_xdat(float, 1);
    } else if (timewarp->format == AV_SAMPLE_FMT_DBL) {
        yae_init_xdat(double, 1);
    }
}

/**
 * Populate the internal data buffer on as-needed basis.
 *
 * @return
 *   0 if requested data was already available or was successfully loaded,
 *   AVERROR(EAGAIN) if more input data is required.
 */
static int yae_load_data(TimewarpContext *timewarp,
                         const uint8_t **src_ref,
                         const uint8_t *src_end,
                         int64_t stop_here)
{
    // shortcut:
    const uint8_t *src = *src_ref;
    const int read_size = stop_here - timewarp->position[0];

    if (stop_here <= timewarp->position[0]) {
        return 0;
    }

    // samples are not expected to be skipped:
    av_assert0(read_size <= timewarp->ring);

    while (timewarp->position[0] < stop_here && src < src_end) {
        int src_samples = (src_end - src) / timewarp->stride;

        // load data piece-wise, in order to avoid complicating the logic:
        int nsamples = FFMIN(read_size, src_samples);
        int na;
        int nb;

        nsamples = FFMIN(nsamples, timewarp->ring);
        na = FFMIN(nsamples, timewarp->ring - timewarp->tail);
        nb = FFMIN(nsamples - na, timewarp->ring);

        if (na) {
            uint8_t *a = timewarp->buffer + timewarp->tail * timewarp->stride;
            memcpy(a, src, na * timewarp->stride);

            src += na * timewarp->stride;
            timewarp->position[0] += na;

            timewarp->size = FFMIN(timewarp->size + na, timewarp->ring);
            timewarp->tail = (timewarp->tail + na) % timewarp->ring;
            timewarp->head =
                timewarp->size < timewarp->ring ?
                timewarp->tail - timewarp->size :
                timewarp->tail;
        }

        if (nb) {
            uint8_t *b = timewarp->buffer;
            memcpy(b, src, nb * timewarp->stride);

            src += nb * timewarp->stride;
            timewarp->position[0] += nb;

            timewarp->size = FFMIN(timewarp->size + nb, timewarp->ring);
            timewarp->tail = (timewarp->tail + nb) % timewarp->ring;
            timewarp->head =
                timewarp->size < timewarp->ring ?
                timewarp->tail - timewarp->size :
                timewarp->tail;
        }
    }

    // pass back the updated source buffer pointer:
    *src_ref = src;

    // sanity check:
    av_assert0(timewarp->position[0] <= stop_here);

    return timewarp->position[0] == stop_here ? 0 : AVERROR(EAGAIN);
}

/**
 * Populate current audio fragment data buffer.
 *
 * @return
 *   0 when the fragment is ready,
 *   AVERROR(EAGAIN) if more input data is required.
 */
static int yae_load_frag(TimewarpContext *timewarp,
                         const uint8_t **src_ref,
                         const uint8_t *src_end)
{
    // shortcuts:
    AudioFragment *frag = yae_curr_frag(timewarp);
    uint8_t *dst;
    int64_t missing, start, zeros;
    uint32_t nsamples;
    const uint8_t *a, *b;
    int i0, i1, n0, n1, na, nb;

    int64_t stop_here = frag->position[0] + timewarp->window;
    if (src_ref && yae_load_data(timewarp, src_ref, src_end, stop_here) != 0) {
        return AVERROR(EAGAIN);
    }

    // calculate the number of samples we don't have:
    missing =
        stop_here > timewarp->position[0] ?
        stop_here - timewarp->position[0] : 0;

    nsamples =
        missing < (int64_t)timewarp->window ?
        (uint32_t)(timewarp->window - missing) : 0;

    // setup the output buffer:
    frag->nsamples = nsamples;
    dst = frag->data;

    start = timewarp->position[0] - timewarp->size;
    zeros = 0;

    if (frag->position[0] < start) {
        // what we don't have we substitute with zeros:
        zeros = FFMIN(start - frag->position[0], (int64_t)nsamples);
        av_assert0(zeros != nsamples);

        memset(dst, 0, zeros * timewarp->stride);
        dst += zeros * timewarp->stride;
    }

    if (zeros == nsamples) {
        return 0;
    }

    // get the remaining data from the ring buffer:
    na = (timewarp->head < timewarp->tail ?
          timewarp->tail - timewarp->head :
          timewarp->ring - timewarp->head);

    nb = timewarp->head < timewarp->tail ? 0 : timewarp->tail;

    // sanity check:
    av_assert0(nsamples <= zeros + na + nb);

    a = timewarp->buffer + timewarp->head * timewarp->stride;
    b = timewarp->buffer;

    i0 = frag->position[0] + zeros - start;
    i1 = i0 < na ? 0 : i0 - na;

    n0 = i0 < na ? FFMIN(na - i0, (int)(nsamples - zeros)) : 0;
    n1 = nsamples - zeros - n0;

    if (n0) {
        memcpy(dst, a + i0 * timewarp->stride, n0 * timewarp->stride);
        dst += n0 * timewarp->stride;
    }

    if (n1) {
        memcpy(dst, b + i1 * timewarp->stride, n1 * timewarp->stride);
    }

    return 0;
}

/**
 * Prepare for loading next audio fragment.
 */
static void yae_advance_to_next_frag(TimewarpContext *timewarp)
{
    const double fragment_step = timewarp->tempo * (double)(timewarp->window / 2);

    const AudioFragment *prev;
    AudioFragment       *frag;

    timewarp->nfrag++;
    prev = yae_prev_frag(timewarp);
    frag = yae_curr_frag(timewarp);

    frag->position[0] = prev->position[0] + (int64_t)fragment_step;
    frag->position[1] = prev->position[1] + timewarp->window / 2;
    frag->nsamples    = 0;
}

/**
 * Calculate cross-correlation via rDFT.
 *
 * Multiply two vectors of complex numbers (result of real_to_complex rDFT)
 * and transform back via complex_to_real rDFT.
 */
static void yae_xcorr_via_rdft(FFTSample *xcorr,
                               RDFTContext *complex_to_real,
                               const FFTComplex *xa,
                               const FFTComplex *xb,
                               const int window)
{
    FFTComplex *xc = (FFTComplex *)xcorr;
    int i;

    // NOTE: first element requires special care -- Given Y = rDFT(X),
    // Im(Y[0]) and Im(Y[N/2]) are always zero, therefore av_rdft_calc
    // stores Re(Y[N/2]) in place of Im(Y[0]).

    xc->re = xa->re * xb->re;
    xc->im = xa->im * xb->im;
    xa++;
    xb++;
    xc++;

    for (i = 1; i < window; i++, xa++, xb++, xc++) {
        xc->re = (xa->re * xb->re + xa->im * xb->im);
        xc->im = (xa->im * xb->re - xa->re * xb->im);
    }

    // apply inverse rDFT:
    av_rdft_calc(complex_to_real, xcorr);
}

/**
 * Calculate alignment offset for given fragment
 * relative to the previous fragment.
 *
 * @return alignment offset of current fragment relative to previous.
 */
static int yae_align(AudioFragment *frag,
                     const AudioFragment *prev,
                     const int window,
                     const int delta_max,
                     const int drift,
                     FFTSample *correlation,
                     RDFTContext *complex_to_real)
{
    int       best_offset = -drift;
    FFTSample best_metric = -FLT_MAX;
    FFTSample *xcorr;

    int i0;
    int i1;
    int i;

    yae_xcorr_via_rdft(correlation,
                       complex_to_real,
                       (const FFTComplex *)prev->xdat,
                       (const FFTComplex *)frag->xdat,
                       window);

    // identify search window boundaries:
    i0 = FFMAX(window / 2 - delta_max - drift, 0);
    i0 = FFMIN(i0, window);

    i1 = FFMIN(window / 2 + delta_max - drift, window - window / 16);
    i1 = FFMAX(i1, 0);

    // identify cross-correlation peaks within search window:
    xcorr = correlation + i0;

    for (i = i0; i < i1; i++, xcorr++) {
        FFTSample metric = *xcorr;

        // normalize:
        FFTSample drifti = (FFTSample)(drift + i);
        metric *= drifti * (FFTSample)(i - i0) * (FFTSample)(i1 - i);

        if (metric > best_metric) {
            best_metric = metric;
            best_offset = i - window / 2;
        }
    }

    return best_offset;
}

/**
 * Adjust current fragment position for better alignment
 * with previous fragment.
 *
 * @return alignment correction.
 */
static int yae_adjust_position(TimewarpContext *timewarp)
{
    const AudioFragment *prev = yae_prev_frag(timewarp);
    AudioFragment       *frag = yae_curr_frag(timewarp);

    const int delta_max  = timewarp->window / 2;
    const int correction = yae_align(frag,
                                     prev,
                                     timewarp->window,
                                     delta_max,
                                     timewarp->drift,
                                     timewarp->correlation,
                                     timewarp->complex_to_real);

    if (correction) {
        // adjust fragment position:
        frag->position[0] -= correction;

        // clear so that the fragment can be reloaded:
        frag->nsamples = 0;

        // update cumulative correction drift counter:
        timewarp->drift += correction;
    }

    return correction;
}

/**
 * A helper macro for blending the overlap region of previous
 * and current audio fragment.
 */
#define yae_blend(scalar_type)                                          \
    do {                                                                \
        const scalar_type *aaa = (const scalar_type *)a;                \
        const scalar_type *bbb = (const scalar_type *)b;                \
                                                                        \
        scalar_type *out     = (scalar_type *)dst;                      \
        scalar_type *out_end = (scalar_type *)dst_end;                  \
        int64_t i;                                                      \
                                                                        \
        for (i = 0; i < overlap && out < out_end;                       \
             i++, timewarp->position[1]++, wa++, wb++) {                  \
            float w0 = *wa;                                             \
            float w1 = *wb;                                             \
            int j;                                                      \
                                                                        \
            for (j = 0; j < timewarp->channels;                           \
                 j++, aaa++, bbb++, out++) {                            \
                float t0 = (float)*aaa;                                 \
                float t1 = (float)*bbb;                                 \
                                                                        \
                *out =                                                  \
                    frag->position[0] + i < 0 ?                         \
                    *aaa :                                              \
                    (scalar_type)(t0 * w0 + t1 * w1);                   \
            }                                                           \
        }                                                               \
        dst = (uint8_t *)out;                                           \
    } while (0)

/**
 * Blend the overlap region of previous and current audio fragment
 * and output the results to the given destination buffer.
 *
 * @return
 *   0 if the overlap region was completely stored in the dst buffer,
 *   AVERROR(EAGAIN) if more destination buffer space is required.
 */
static int yae_overlap_add(TimewarpContext *timewarp,
                           uint8_t **dst_ref,
                           uint8_t *dst_end)
{
    // shortcuts:
    const AudioFragment *prev = yae_prev_frag(timewarp);
    const AudioFragment *frag = yae_curr_frag(timewarp);

    const int64_t start_here = FFMAX(timewarp->position[1],
                                     frag->position[1]);

    const int64_t stop_here = FFMIN(prev->position[1] + prev->nsamples,
                                    frag->position[1] + frag->nsamples);

    const int64_t overlap = stop_here - start_here;

    const int64_t ia = start_here - prev->position[1];
    const int64_t ib = start_here - frag->position[1];

    const float *wa = timewarp->hann + ia;
    const float *wb = timewarp->hann + ib;

    const uint8_t *a = prev->data + ia * timewarp->stride;
    const uint8_t *b = frag->data + ib * timewarp->stride;

    uint8_t *dst = *dst_ref;

    av_assert0(start_here <= stop_here &&
               frag->position[1] <= start_here &&
               overlap <= frag->nsamples);

    if (timewarp->format == AV_SAMPLE_FMT_U8) {
        yae_blend(uint8_t);
    } else if (timewarp->format == AV_SAMPLE_FMT_S16) {
        yae_blend(int16_t);
    } else if (timewarp->format == AV_SAMPLE_FMT_S32) {
        yae_blend(int);
    } else if (timewarp->format == AV_SAMPLE_FMT_FLT) {
        yae_blend(float);
    } else if (timewarp->format == AV_SAMPLE_FMT_DBL) {
        yae_blend(double);
    }

    // pass-back the updated destination buffer pointer:
    *dst_ref = dst;

    return timewarp->position[1] == stop_here ? 0 : AVERROR(EAGAIN);
}

/**
 * Feed as much data to the filter as it is able to consume
 * and receive as much processed data in the destination buffer
 * as it is able to produce or store.
 */
static void
yae_apply(TimewarpContext *timewarp,
          const uint8_t **src_ref,
          const uint8_t *src_end,
          uint8_t **dst_ref,
          uint8_t *dst_end)
{
    while (1) {
        if (timewarp->state == YAE_LOAD_FRAGMENT) {
            // load additional data for the current fragment:
            if (yae_load_frag(timewarp, src_ref, src_end) != 0) {
                break;
            }

            // down-mix to mono:
            yae_downmix(timewarp, yae_curr_frag(timewarp));

            // apply rDFT:
            av_rdft_calc(timewarp->real_to_complex, yae_curr_frag(timewarp)->xdat);

            // must load the second fragment before alignment can start:
            if (!timewarp->nfrag) {
                yae_advance_to_next_frag(timewarp);
                continue;
            }

            timewarp->state = YAE_ADJUST_POSITION;
        }

        if (timewarp->state == YAE_ADJUST_POSITION) {
            // adjust position for better alignment:
            if (yae_adjust_position(timewarp)) {
                // reload the fragment at the corrected position, so that the
                // Hann window blending would not require normalization:
                timewarp->state = YAE_RELOAD_FRAGMENT;
            } else {
                timewarp->state = YAE_OUTPUT_OVERLAP_ADD;
            }
        }

        if (timewarp->state == YAE_RELOAD_FRAGMENT) {
            // load additional data if necessary due to position adjustment:
            if (yae_load_frag(timewarp, src_ref, src_end) != 0) {
                break;
            }

            // down-mix to mono:
            yae_downmix(timewarp, yae_curr_frag(timewarp));

            // apply rDFT:
            av_rdft_calc(timewarp->real_to_complex, yae_curr_frag(timewarp)->xdat);

            timewarp->state = YAE_OUTPUT_OVERLAP_ADD;
        }

        if (timewarp->state == YAE_OUTPUT_OVERLAP_ADD) {
            // overlap-add and output the result:
            if (yae_overlap_add(timewarp, dst_ref, dst_end) != 0) {
                break;
            }

            // advance to the next fragment, repeat:
            yae_advance_to_next_frag(timewarp);
            timewarp->state = YAE_LOAD_FRAGMENT;
        }
    }
}

/**
 * Flush any buffered data from the filter.
 *
 * @return
 *   0 if all data was completely stored in the dst buffer,
 *   AVERROR(EAGAIN) if more destination buffer space is required.
 */
static int yae_flush(TimewarpContext *timewarp,
                     uint8_t **dst_ref,
                     uint8_t *dst_end)
{
    AudioFragment *frag = yae_curr_frag(timewarp);
    int64_t overlap_end;
    int64_t start_here;
    int64_t stop_here;
    int64_t offset;

    const uint8_t *src;
    uint8_t *dst;

    int src_size;
    int dst_size;
    int nbytes;

    timewarp->state = YAE_FLUSH_OUTPUT;

    if (timewarp->position[0] == frag->position[0] + frag->nsamples &&
        timewarp->position[1] == frag->position[1] + frag->nsamples) {
        // the current fragment is already flushed:
        return 0;
    }

    if (frag->position[0] + frag->nsamples < timewarp->position[0]) {
        // finish loading the current (possibly partial) fragment:
        yae_load_frag(timewarp, NULL, NULL);

        if (timewarp->nfrag) {
            // down-mix to mono:
            yae_downmix(timewarp, frag);

            // apply rDFT:
            av_rdft_calc(timewarp->real_to_complex, frag->xdat);

            // align current fragment to previous fragment:
            if (yae_adjust_position(timewarp)) {
                // reload the current fragment due to adjusted position:
                yae_load_frag(timewarp, NULL, NULL);
            }
        }
    }

    // flush the overlap region:
    overlap_end = frag->position[1] + FFMIN(timewarp->window / 2,
                                            frag->nsamples);

    while (timewarp->position[1] < overlap_end) {
        if (yae_overlap_add(timewarp, dst_ref, dst_end) != 0) {
            return AVERROR(EAGAIN);
        }
    }

    // flush the remaininder of the current fragment:
    start_here = FFMAX(timewarp->position[1], overlap_end);
    stop_here  = frag->position[1] + frag->nsamples;
    offset     = start_here - frag->position[1];
    av_assert0(start_here <= stop_here && frag->position[1] <= start_here);

    src = frag->data + offset * timewarp->stride;
    dst = (uint8_t *)*dst_ref;

    src_size = (int)(stop_here - start_here) * timewarp->stride;
    dst_size = dst_end - dst;
    nbytes = FFMIN(src_size, dst_size);

    memcpy(dst, src, nbytes);
    dst += nbytes;

    timewarp->position[1] += (nbytes / timewarp->stride);

    // pass-back the updated destination buffer pointer:
    *dst_ref = (uint8_t *)dst;

    return timewarp->position[1] == stop_here ? 0 : AVERROR(EAGAIN);
}

static av_cold int init(AVFilterContext *ctx, const char *args)
{
    TimewarpContext *timewarp = ctx->priv;
    int ret;

    // NOTE: this assumes that the caller has memset ctx->priv to 0:
    timewarp->format = AV_SAMPLE_FMT_NONE;
    timewarp->tempo  = 1.0;
    timewarp->state  = YAE_LOAD_FRAGMENT;

    timewarp->class = &timewarp_class;
    av_opt_set_defaults(timewarp);

    if ((ret = av_set_options_string(timewarp, args, "=", ":")) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error parsing options string '%s'.\n", args);
        return ret;
    }

    av_log(ctx, AV_LOG_INFO, "\n\nmetronomepipe = [%s]\n\n", timewarp->metronomepipe_str);

    //  Open metronome pipe for non-blocking input
    if ((timewarp->metronomepipe_fd = open(timewarp->metronomepipe_str, O_RDONLY | O_NONBLOCK)) == -1) {
        av_log(ctx, AV_LOG_ERROR, "Unable to open metronome pipe.\n");
        return AVERROR(EINVAL);
    }
    else {
        av_log(ctx, AV_LOG_INFO, "Opened metronome pipe.\n");
    }

    if ((timewarp->metronomepipe = fdopen(timewarp->metronomepipe_fd, "r")) == NULL) {
      av_log(ctx, AV_LOG_ERROR, "Unable to open metronome pipe file descriptor.\n");
      return AVERROR(EINVAL);
    }
    else {
        av_log(ctx, AV_LOG_INFO, "Opened metronome pipe file descriptor.\n");
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    TimewarpContext *timewarp = ctx->priv;
    yae_release_buffers(timewarp);
    fclose(timewarp->metronomepipe);
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterChannelLayouts *layouts = NULL;
    AVFilterFormats        *formats = NULL;

    // WSOLA necessitates an internal sliding window ring buffer
    // for incoming audio stream.
    //
    // Planar sample formats are too cumbersome to store in a ring buffer,
    // therefore planar sample formats are not supported.
    //
    enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_U8,
        AV_SAMPLE_FMT_S16,
        AV_SAMPLE_FMT_S32,
        AV_SAMPLE_FMT_FLT,
        AV_SAMPLE_FMT_DBL,
        AV_SAMPLE_FMT_NONE
    };

    layouts = ff_all_channel_layouts();
    if (!layouts) {
        return AVERROR(ENOMEM);
    }
    ff_set_common_channel_layouts(ctx, layouts);

    formats = ff_make_format_list(sample_fmts);
    if (!formats) {
        return AVERROR(ENOMEM);
    }
    ff_set_common_formats(ctx, formats);

    formats = ff_all_samplerates();
    if (!formats) {
        return AVERROR(ENOMEM);
    }
    ff_set_common_samplerates(ctx, formats);

    return 0;
}

static int config_props(AVFilterLink *inlink)
{
    AVFilterContext  *ctx = inlink->dst;
    TimewarpContext *timewarp = ctx->priv;

    enum AVSampleFormat format = inlink->format;
    int sample_rate = (int)inlink->sample_rate;
    int channels = av_get_channel_layout_nb_channels(inlink->channel_layout);

    return yae_reset(timewarp, format, sample_rate, channels);
}

static void push_samples(TimewarpContext *timewarp,
                         AVFilterLink *outlink,
                         int n_out)
{
    timewarp->dst_buffer->audio->sample_rate = outlink->sample_rate;
    timewarp->dst_buffer->audio->nb_samples  = n_out;

    // adjust the PTS:
    timewarp->dst_buffer->pts =
        av_rescale_q(timewarp->nsamples_out,
                     (AVRational){ 1, outlink->sample_rate },
                     outlink->time_base);

    ff_filter_samples(outlink, timewarp->dst_buffer);
    timewarp->dst_buffer = NULL;
    timewarp->dst        = NULL;
    timewarp->dst_end    = NULL;

    timewarp->nsamples_out += n_out;
}

static int filter_samples(AVFilterLink *inlink,
                           AVFilterBufferRef *src_buffer)
{
    AVFilterContext  *ctx = inlink->dst;
    TimewarpContext *timewarp = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    double new_tempo;

    int n_in = src_buffer->audio->nb_samples;
    int n_out = (int)(0.5 + ((double)n_in) / timewarp->tempo);

    const uint8_t *src = src_buffer->data[0];
    const uint8_t *src_end = src + n_in * timewarp->stride;

    //  Non-blockingly read tempo from pipe
    if (fread(&new_tempo, sizeof(double), 1, timewarp->metronomepipe) == 1) {
        av_log(ctx, AV_LOG_INFO, "New Tempo = %f\n", new_tempo);
	timewarp->tempo = new_tempo;
	n_out = (int)(0.5 + ((double)n_in) / timewarp->tempo);
    }
    else if (ferror(timewarp->metronomepipe) && errno != EAGAIN) {
        av_log(ctx, AV_LOG_ERROR, "Error reading from metronome pipe.\n");
    }


    while (src < src_end) {
        if (!timewarp->dst_buffer) {
            timewarp->dst_buffer = ff_get_audio_buffer(outlink,
                                                     AV_PERM_WRITE,
                                                     n_out);
            avfilter_copy_buffer_ref_props(timewarp->dst_buffer, src_buffer);

            timewarp->dst = timewarp->dst_buffer->data[0];
            timewarp->dst_end = timewarp->dst + n_out * timewarp->stride;
        }

        yae_apply(timewarp, &src, src_end, &timewarp->dst, timewarp->dst_end);

        if (timewarp->dst == timewarp->dst_end) {
            push_samples(timewarp, outlink, n_out);
            timewarp->request_fulfilled = 1;
        }
    }

    timewarp->nsamples_in += n_in;
    avfilter_unref_bufferp(&src_buffer);
    return 0;
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext  *ctx = outlink->src;
    TimewarpContext *timewarp = ctx->priv;
    int ret;

    timewarp->request_fulfilled = 0;
    do {
        ret = ff_request_frame(ctx->inputs[0]);
    }
    while (!timewarp->request_fulfilled && ret >= 0);

    if (ret == AVERROR_EOF) {
        // flush the filter:
        int n_max = timewarp->ring;
        int n_out;
        int err = AVERROR(EAGAIN);

        while (err == AVERROR(EAGAIN)) {
            if (!timewarp->dst_buffer) {
                timewarp->dst_buffer = ff_get_audio_buffer(outlink,
                                                         AV_PERM_WRITE,
                                                         n_max);

                timewarp->dst = timewarp->dst_buffer->data[0];
                timewarp->dst_end = timewarp->dst + n_max * timewarp->stride;
            }

            err = yae_flush(timewarp, &timewarp->dst, timewarp->dst_end);

            n_out = ((timewarp->dst - timewarp->dst_buffer->data[0]) /
                     timewarp->stride);

            if (n_out) {
                push_samples(timewarp, outlink, n_out);
            }
        }

        avfilter_unref_bufferp(&timewarp->dst_buffer);
        timewarp->dst     = NULL;
        timewarp->dst_end = NULL;

        return AVERROR_EOF;
    }

    return ret;
}

/*
static int process_command(AVFilterContext *ctx,
                           const char *cmd,
                           const char *arg,
                           char *res,
                           int res_len,
                           int flags)
{
    return !strcmp(cmd, "tempo") ? yae_set_tempo(ctx, arg) : AVERROR(ENOSYS);
}
*/

AVFilter avfilter_af_timewarp = {
    .name            = "timewarp",
    .description     = NULL_IF_CONFIG_SMALL("Adjust audio tempo dynamically based on commans from UNIX pipe"),
    .init            = init,
    .uninit          = uninit,
    .query_formats   = query_formats,
    //    .process_command = process_command,
    .priv_size       = sizeof(TimewarpContext),

    .inputs    = (const AVFilterPad[]) {
        { .name            = "default",
          .type            = AVMEDIA_TYPE_AUDIO,
          .filter_samples  = filter_samples,
          .config_props    = config_props,
          .min_perms       = AV_PERM_READ, },
        { .name = NULL}
    },

    .outputs   = (const AVFilterPad[]) {
        { .name            = "default",
          .request_frame   = request_frame,
          .type            = AVMEDIA_TYPE_AUDIO, },
        { .name = NULL}
    },
};
