/*
 * Copyright (c) 2011 Stefano Sabatini
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
 * filter for showing textual audio frame information
 */


#include <fcntl.h>       //  O_RDONLY, O_NONBLOCK


#include "libavutil/adler32.h"
#include "libavutil/audioconvert.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "audio.h"
#include "avfilter.h"
#include "internal.h"

typedef struct {
    const AVClass *class;

    unsigned int frame;

    float volume;
    int volume_i;
    
    FILE *metronomepipe;
    int metronomepipe_fd;
    char *metronomepipe_str;

} VolumewarpContext;

#define OFFSET(x) offsetof(VolumewarpContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
// #define F AV_OPT_FLAG_FILTERING_PARAM
#define F 0
static const AVOption volumewarp_options[] = {
    { "metronomepipe", "Name of FIFO paper used to receive metronome commands.", OFFSET(metronomepipe_str), AV_OPT_TYPE_STRING, .flags = A|F },
    { NULL },
};

static const AVClass volumewarp_class = {
  .class_name = "volumewarp filter",
  .item_name  = av_default_item_name,
  .option     = volumewarp_options,
  .version    = LIBAVUTIL_VERSION_INT,
};

static av_cold int init(AVFilterContext *ctx, const char *args)
{
    int ret;

    VolumewarpContext *s = ctx->priv;
    s->frame = 0;

    if (!args) {
        av_log(ctx, AV_LOG_ERROR, "No parameters supplied.\n");
        return AVERROR(EINVAL);
    }

    s->class = &volumewarp_class;
    av_opt_set_defaults(s);

    if ((ret = av_set_options_string(s, args, "=", ":")) < 0)
        return ret;

    av_log(ctx, AV_LOG_INFO, "\n\nmetronomepipe = [%s]\n\n", s->metronomepipe_str);


    //  Open metronome pipe for non-blocking input
    if ((s->metronomepipe_fd = open(s->metronomepipe_str, O_RDONLY | O_NONBLOCK)) == -1) {
        av_log(ctx, AV_LOG_ERROR, "Unable to open metronome pipe.\n");
        return AVERROR(EINVAL);
    }
    else {
        av_log(ctx, AV_LOG_INFO, "Opened metronome pipe.\n");
    }

    if ((s->metronomepipe = fdopen(s->metronomepipe_fd, "r")) == NULL) {
      av_log(ctx, AV_LOG_ERROR, "Unable to open metronome pipe file descriptor.\n");
      return AVERROR(EINVAL);
    }
    else {
        av_log(ctx, AV_LOG_INFO, "Opened metronome pipe file descriptor.\n");
    }


    //  Set volume
    s->volume = 0.25;
    s->volume_i = (int)(s->volume * 256 + 0.5);


    av_opt_free(s);
    return ret;
}

static void uninit(AVFilterContext *ctx)
{
    VolumewarpContext *vol = ctx->priv;

    fclose(vol->metronomepipe);
}

static int filter_samples(AVFilterLink *inlink, AVFilterBufferRef *insamples)
{
    AVFilterContext *ctx = inlink->dst;
    VolumewarpContext *vol = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    const int nb_samples = insamples->audio->nb_samples *
        av_get_channel_layout_nb_channels(insamples->audio->channel_layout);
    double new_volume;
    int i;


    //  Non-blockingly read volume from pipe
    if (fread(&new_volume, sizeof(double), 1, vol->metronomepipe) == 1) {
        vol->volume = new_volume;
        vol->volume_i = (int)(vol->volume * 256 + 0.5);
        av_log(ctx, AV_LOG_INFO, "Volume = %f (%d)\n", vol->volume, vol->volume_i);
    }
    else if (ferror(vol->metronomepipe) && errno != EAGAIN) {
        av_log(ctx, AV_LOG_ERROR, "Error reading from metronome pipe.\n");
    }


    //  Code below copied from af_volume.c
    if (vol->volume_i != 256) {
        switch (insamples->format) {
        case AV_SAMPLE_FMT_U8:
        {
            uint8_t *p = (void *)insamples->data[0];
            for (i = 0; i < nb_samples; i++) {
                int v = (((*p - 128) * vol->volume_i + 128) >> 8) + 128;
                *p++ = av_clip_uint8(v);
            }
            break;
        }
        case AV_SAMPLE_FMT_S16:
        {
            int16_t *p = (void *)insamples->data[0];
            for (i = 0; i < nb_samples; i++) {
                int v = ((int64_t)*p * vol->volume_i + 128) >> 8;
                *p++ = av_clip_int16(v);
            }
            break;
        }
        case AV_SAMPLE_FMT_S32:
        {
            int32_t *p = (void *)insamples->data[0];
            for (i = 0; i < nb_samples; i++) {
                int64_t v = (((int64_t)*p * vol->volume_i + 128) >> 8);
                *p++ = av_clipl_int32(v);
            }
            break;
        }
        case AV_SAMPLE_FMT_FLT:
        {
            float *p = (void *)insamples->data[0];
            float scale = (float)vol->volume;
            for (i = 0; i < nb_samples; i++) {
                *p++ *= scale;
            }
            break;
        }
        case AV_SAMPLE_FMT_DBL:
        {
            double *p = (void *)insamples->data[0];
            for (i = 0; i < nb_samples; i++) {
                *p *= vol->volume;
                p++;
            }
            break;
        }
        }
    }
    return ff_filter_samples(outlink, insamples);
}

AVFilter avfilter_af_volumewarp = {
    .name        = "volumewarp",
    .description = NULL_IF_CONFIG_SMALL("Change input volume dynamically based on commands from UNIX pipe"),

    .priv_size = sizeof(VolumewarpContext),
    .init      = init,
    .uninit    = uninit,

    .inputs    = (const AVFilterPad[]) {{ .name       = "default",
                                    .type             = AVMEDIA_TYPE_AUDIO,
                                    .filter_samples   = filter_samples,
                                    .min_perms        = AV_PERM_READ|AV_PERM_WRITE, },
                                  { .name = NULL}},

    .outputs   = (const AVFilterPad[]) {{ .name       = "default",
                                    .type             = AVMEDIA_TYPE_AUDIO },
                                  { .name = NULL}},
};
