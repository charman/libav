/*
 * avconv filter configuration
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avconv.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/avfiltergraph.h"

#include "libavutil/audioconvert.h"
#include "libavutil/avassert.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixfmt.h"
#include "libavutil/samplefmt.h"

/* Define a function for building a string containing a list of
 * allowed formats. */
#define DEF_CHOOSE_FORMAT(type, var, supported_list, none, get_name, separator)\
static char *choose_ ## var ## s(OutputStream *ost)                            \
{                                                                              \
    if (ost->st->codec->var != none) {                                         \
        get_name(ost->st->codec->var);                                         \
        return av_strdup(name);                                                \
    } else if (ost->enc->supported_list) {                                     \
        const type *p;                                                         \
        AVIOContext *s = NULL;                                                 \
        uint8_t *ret;                                                          \
        int len;                                                               \
                                                                               \
        if (avio_open_dyn_buf(&s) < 0)                                         \
            exit(1);                                                           \
                                                                               \
        for (p = ost->enc->supported_list; *p != none; p++) {                  \
            get_name(*p);                                                      \
            avio_printf(s, "%s" separator, name);                              \
        }                                                                      \
        len = avio_close_dyn_buf(s, &ret);                                     \
        ret[len - 1] = 0;                                                      \
        return ret;                                                            \
    } else                                                                     \
        return NULL;                                                           \
}

DEF_CHOOSE_FORMAT(enum AVPixelFormat, pix_fmt, pix_fmts, AV_PIX_FMT_NONE,
                  GET_PIX_FMT_NAME, ":")

DEF_CHOOSE_FORMAT(enum AVSampleFormat, sample_fmt, sample_fmts,
                  AV_SAMPLE_FMT_NONE, GET_SAMPLE_FMT_NAME, ",")

DEF_CHOOSE_FORMAT(int, sample_rate, supported_samplerates, 0,
                  GET_SAMPLE_RATE_NAME, ",")

DEF_CHOOSE_FORMAT(uint64_t, channel_layout, channel_layouts, 0,
                  GET_CH_LAYOUT_NAME, ",")

FilterGraph *init_simple_filtergraph(InputStream *ist, OutputStream *ost)
{
    FilterGraph *fg = av_mallocz(sizeof(*fg));

    if (!fg)
        exit(1);
    fg->index = nb_filtergraphs;

    fg->outputs = grow_array(fg->outputs, sizeof(*fg->outputs), &fg->nb_outputs,
                             fg->nb_outputs + 1);
    if (!(fg->outputs[0] = av_mallocz(sizeof(*fg->outputs[0]))))
        exit(1);
    fg->outputs[0]->ost   = ost;
    fg->outputs[0]->graph = fg;

    ost->filter = fg->outputs[0];

    fg->inputs = grow_array(fg->inputs, sizeof(*fg->inputs), &fg->nb_inputs,
                            fg->nb_inputs + 1);
    if (!(fg->inputs[0] = av_mallocz(sizeof(*fg->inputs[0]))))
        exit(1);
    fg->inputs[0]->ist   = ist;
    fg->inputs[0]->graph = fg;

    ist->filters = grow_array(ist->filters, sizeof(*ist->filters),
                              &ist->nb_filters, ist->nb_filters + 1);
    ist->filters[ist->nb_filters - 1] = fg->inputs[0];

    filtergraphs = grow_array(filtergraphs, sizeof(*filtergraphs),
                              &nb_filtergraphs, nb_filtergraphs + 1);
    filtergraphs[nb_filtergraphs - 1] = fg;

    return fg;
}

static void init_input_filter(FilterGraph *fg, AVFilterInOut *in)
{
    InputStream *ist = NULL;
    enum AVMediaType type = avfilter_pad_get_type(in->filter_ctx->input_pads, in->pad_idx);
    int i;

    // TODO: support other filter types
    if (type != AVMEDIA_TYPE_VIDEO && type != AVMEDIA_TYPE_AUDIO) {
        av_log(NULL, AV_LOG_FATAL, "Only video and audio filters supported "
               "currently.\n");
        exit(1);
    }

    if (in->name) {
        AVFormatContext *s;
        AVStream       *st = NULL;
        char *p;
        int file_idx = strtol(in->name, &p, 0);

        if (file_idx < 0 || file_idx >= nb_input_files) {
            av_log(NULL, AV_LOG_FATAL, "Invalid file index %d in filtegraph description %s.\n",
                   file_idx, fg->graph_desc);
            exit(1);
        }
        s = input_files[file_idx]->ctx;

        for (i = 0; i < s->nb_streams; i++) {
            if (s->streams[i]->codec->codec_type != type)
                continue;
            if (check_stream_specifier(s, s->streams[i], *p == ':' ? p + 1 : p) == 1) {
                st = s->streams[i];
                break;
            }
        }
        if (!st) {
            av_log(NULL, AV_LOG_FATAL, "Stream specifier '%s' in filtergraph description %s "
                   "matches no streams.\n", p, fg->graph_desc);
            exit(1);
        }
        ist = input_streams[input_files[file_idx]->ist_index + st->index];
    } else {
        /* find the first unused stream of corresponding type */
        for (i = 0; i < nb_input_streams; i++) {
            ist = input_streams[i];
            if (ist->st->codec->codec_type == type && ist->discard)
                break;
        }
        if (i == nb_input_streams) {
            av_log(NULL, AV_LOG_FATAL, "Cannot find a matching stream for "
                   "unlabeled input pad %d on filter %s", in->pad_idx,
                   in->filter_ctx->name);
            exit(1);
        }
    }
    av_assert0(ist);

    ist->discard         = 0;
    ist->decoding_needed = 1;
    ist->st->discard = AVDISCARD_NONE;

    fg->inputs = grow_array(fg->inputs, sizeof(*fg->inputs),
                            &fg->nb_inputs, fg->nb_inputs + 1);
    if (!(fg->inputs[fg->nb_inputs - 1] = av_mallocz(sizeof(*fg->inputs[0]))))
        exit(1);
    fg->inputs[fg->nb_inputs - 1]->ist   = ist;
    fg->inputs[fg->nb_inputs - 1]->graph = fg;

    ist->filters = grow_array(ist->filters, sizeof(*ist->filters),
                              &ist->nb_filters, ist->nb_filters + 1);
    ist->filters[ist->nb_filters - 1] = fg->inputs[fg->nb_inputs - 1];
}

static int configure_output_video_filter(FilterGraph *fg, OutputFilter *ofilter, AVFilterInOut *out)
{
    char *pix_fmts;
    OutputStream *ost = ofilter->ost;
    AVCodecContext *codec = ost->st->codec;
    AVFilterContext *last_filter = out->filter_ctx;
    int pad_idx = out->pad_idx;
    int ret;
    char name[255];

    snprintf(name, sizeof(name), "output stream %d:%d", ost->file_index, ost->index);
    ret = avfilter_graph_create_filter(&ofilter->filter,
                                       avfilter_get_by_name("buffersink"),
                                       name, NULL, NULL, fg->graph);
    if (ret < 0)
        return ret;

    if (codec->width || codec->height) {
        char args[255];
        AVFilterContext *filter;

        snprintf(args, sizeof(args), "%d:%d:flags=0x%X",
                 codec->width,
                 codec->height,
                 (unsigned)ost->sws_flags);
        snprintf(name, sizeof(name), "scaler for output stream %d:%d",
                 ost->file_index, ost->index);
        if ((ret = avfilter_graph_create_filter(&filter, avfilter_get_by_name("scale"),
                                                name, args, NULL, fg->graph)) < 0)
            return ret;
        if ((ret = avfilter_link(last_filter, pad_idx, filter, 0)) < 0)
            return ret;

        last_filter = filter;
        pad_idx = 0;
    }

    if ((pix_fmts = choose_pix_fmts(ost))) {
        AVFilterContext *filter;
        snprintf(name, sizeof(name), "pixel format for output stream %d:%d",
                 ost->file_index, ost->index);
        if ((ret = avfilter_graph_create_filter(&filter,
                                                avfilter_get_by_name("format"),
                                                "format", pix_fmts, NULL,
                                                fg->graph)) < 0)
            return ret;
        if ((ret = avfilter_link(last_filter, pad_idx, filter, 0)) < 0)
            return ret;

        last_filter = filter;
        pad_idx     = 0;
        av_freep(&pix_fmts);
    }

    if (ost->frame_rate.num) {
        AVFilterContext *fps;
        char args[255];

        snprintf(args, sizeof(args), "fps=%d/%d", ost->frame_rate.num,
                 ost->frame_rate.den);
        snprintf(name, sizeof(name), "fps for output stream %d:%d",
                 ost->file_index, ost->index);
        ret = avfilter_graph_create_filter(&fps, avfilter_get_by_name("fps"),
                                           name, args, NULL, fg->graph);
        if (ret < 0)
            return ret;

        ret = avfilter_link(last_filter, pad_idx, fps, 0);
        if (ret < 0)
            return ret;
        last_filter = fps;
        pad_idx = 0;
    }

    if ((ret = avfilter_link(last_filter, pad_idx, ofilter->filter, 0)) < 0)
        return ret;

    return 0;
}

static int configure_output_audio_filter(FilterGraph *fg, OutputFilter *ofilter, AVFilterInOut *out)
{
    OutputStream *ost = ofilter->ost;
    AVCodecContext *codec  = ost->st->codec;
    AVFilterContext *last_filter = out->filter_ctx;
    int pad_idx = out->pad_idx;
    char *sample_fmts, *sample_rates, *channel_layouts;
    char name[255];
    int ret;


    snprintf(name, sizeof(name), "output stream %d:%d", ost->file_index, ost->index);
    ret = avfilter_graph_create_filter(&ofilter->filter,
                                       avfilter_get_by_name("abuffersink"),
                                       name, NULL, NULL, fg->graph);
    if (ret < 0)
        return ret;

    if (codec->channels && !codec->channel_layout)
        codec->channel_layout = av_get_default_channel_layout(codec->channels);

    sample_fmts     = choose_sample_fmts(ost);
    sample_rates    = choose_sample_rates(ost);
    channel_layouts = choose_channel_layouts(ost);
    if (sample_fmts || sample_rates || channel_layouts) {
        AVFilterContext *format;
        char args[256];
        int len = 0;

        if (sample_fmts)
            len += snprintf(args + len, sizeof(args) - len, "sample_fmts=%s:",
                            sample_fmts);
        if (sample_rates)
            len += snprintf(args + len, sizeof(args) - len, "sample_rates=%s:",
                            sample_rates);
        if (channel_layouts)
            len += snprintf(args + len, sizeof(args) - len, "channel_layouts=%s:",
                            channel_layouts);
        args[len - 1] = 0;

        av_freep(&sample_fmts);
        av_freep(&sample_rates);
        av_freep(&channel_layouts);

        snprintf(name, sizeof(name), "audio format for output stream %d:%d",
                 ost->file_index, ost->index);
        ret = avfilter_graph_create_filter(&format,
                                           avfilter_get_by_name("aformat"),
                                           name, args, NULL, fg->graph);
        if (ret < 0)
            return ret;

        ret = avfilter_link(last_filter, pad_idx, format, 0);
        if (ret < 0)
            return ret;

        last_filter = format;
        pad_idx = 0;
    }

    if ((ret = avfilter_link(last_filter, pad_idx, ofilter->filter, 0)) < 0)
        return ret;

    return 0;
}

#define DESCRIBE_FILTER_LINK(f, inout, in)                         \
{                                                                  \
    AVFilterContext *ctx = inout->filter_ctx;                      \
    AVFilterPad *pads = in ? ctx->input_pads  : ctx->output_pads;  \
    int       nb_pads = in ? ctx->input_count : ctx->output_count; \
    AVIOContext *pb;                                               \
                                                                   \
    if (avio_open_dyn_buf(&pb) < 0)                                \
        exit(1);                                                   \
                                                                   \
    avio_printf(pb, "%s", ctx->filter->name);                      \
    if (nb_pads > 1)                                               \
        avio_printf(pb, ":%s", avfilter_pad_get_name(pads, inout->pad_idx));\
    avio_w8(pb, 0);                                                \
    avio_close_dyn_buf(pb, &f->name);                              \
}

int configure_output_filter(FilterGraph *fg, OutputFilter *ofilter, AVFilterInOut *out)
{
    av_freep(&ofilter->name);
    DESCRIBE_FILTER_LINK(ofilter, out, 0);

    switch (avfilter_pad_get_type(out->filter_ctx->output_pads, out->pad_idx)) {
    case AVMEDIA_TYPE_VIDEO: return configure_output_video_filter(fg, ofilter, out);
    case AVMEDIA_TYPE_AUDIO: return configure_output_audio_filter(fg, ofilter, out);
    default: av_assert0(0);
    }
}

static int configure_input_video_filter(FilterGraph *fg, InputFilter *ifilter,
                                        AVFilterInOut *in)
{
    AVFilterContext *first_filter = in->filter_ctx;
    AVFilter *filter = avfilter_get_by_name("buffer");
    InputStream *ist = ifilter->ist;
    AVRational tb = ist->framerate.num ? av_inv_q(ist->framerate) :
                                         ist->st->time_base;
    AVRational sar;
    char args[255], name[255];
    int pad_idx = in->pad_idx;
    int ret;

    sar = ist->st->sample_aspect_ratio.num ?
          ist->st->sample_aspect_ratio :
          ist->st->codec->sample_aspect_ratio;
    snprintf(args, sizeof(args), "%d:%d:%d:%d:%d:%d:%d", ist->st->codec->width,
             ist->st->codec->height, ist->st->codec->pix_fmt,
             tb.num, tb.den, sar.num, sar.den);
    snprintf(name, sizeof(name), "graph %d input from stream %d:%d", fg->index,
             ist->file_index, ist->st->index);

    if ((ret = avfilter_graph_create_filter(&ifilter->filter, filter, name,
                                            args, NULL, fg->graph)) < 0)
        return ret;

    if (ist->framerate.num) {
        AVFilterContext *setpts;

        snprintf(name, sizeof(name), "force CFR for input from stream %d:%d",
                 ist->file_index, ist->st->index);
        if ((ret = avfilter_graph_create_filter(&setpts,
                                                avfilter_get_by_name("setpts"),
                                                name, "N", NULL,
                                                fg->graph)) < 0)
            return ret;

        if ((ret = avfilter_link(setpts, 0, first_filter, pad_idx)) < 0)
            return ret;

        first_filter = setpts;
        pad_idx = 0;
    }

    if ((ret = avfilter_link(ifilter->filter, 0, first_filter, pad_idx)) < 0)
        return ret;
    return 0;
}

static int configure_input_audio_filter(FilterGraph *fg, InputFilter *ifilter,
                                        AVFilterInOut *in)
{
    AVFilterContext *first_filter = in->filter_ctx;
    AVFilter *filter = avfilter_get_by_name("abuffer");
    InputStream *ist = ifilter->ist;
    int pad_idx = in->pad_idx;
    char args[255], name[255];
    int ret;

    snprintf(args, sizeof(args), "time_base=%d/%d:sample_rate=%d:sample_fmt=%s"
             ":channel_layout=0x%"PRIx64,
             1, ist->st->codec->sample_rate,
             ist->st->codec->sample_rate,
             av_get_sample_fmt_name(ist->st->codec->sample_fmt),
             ist->st->codec->channel_layout);
    snprintf(name, sizeof(name), "graph %d input from stream %d:%d", fg->index,
             ist->file_index, ist->st->index);

    if ((ret = avfilter_graph_create_filter(&ifilter->filter, filter,
                                            name, args, NULL,
                                            fg->graph)) < 0)
        return ret;

    if (audio_sync_method > 0) {
        AVFilterContext *async;
        char args[256];
        int  len = 0;

        av_log(NULL, AV_LOG_WARNING, "-async has been deprecated. Used the "
               "asyncts audio filter instead.\n");

        if (audio_sync_method > 1)
            len += snprintf(args + len, sizeof(args) - len, "compensate=1:"
                            "max_comp=%d:", audio_sync_method);
        snprintf(args + len, sizeof(args) - len, "min_delta=%f",
                 audio_drift_threshold);

        snprintf(name, sizeof(name), "graph %d audio sync for input stream %d:%d",
                 fg->index, ist->file_index, ist->st->index);
        ret = avfilter_graph_create_filter(&async,
                                           avfilter_get_by_name("asyncts"),
                                           name, args, NULL, fg->graph);
        if (ret < 0)
            return ret;

        ret = avfilter_link(async, 0, first_filter, pad_idx);
        if (ret < 0)
            return ret;

        first_filter = async;
        pad_idx = 0;
    }
    if ((ret = avfilter_link(ifilter->filter, 0, first_filter, pad_idx)) < 0)
        return ret;

    return 0;
}

static int configure_input_filter(FilterGraph *fg, InputFilter *ifilter,
                                  AVFilterInOut *in)
{
    av_freep(&ifilter->name);
    DESCRIBE_FILTER_LINK(ifilter, in, 1);

    switch (avfilter_pad_get_type(in->filter_ctx->input_pads, in->pad_idx)) {
    case AVMEDIA_TYPE_VIDEO: return configure_input_video_filter(fg, ifilter, in);
    case AVMEDIA_TYPE_AUDIO: return configure_input_audio_filter(fg, ifilter, in);
    default: av_assert0(0);
    }
}

int configure_filtergraph(FilterGraph *fg)
{
    AVFilterInOut *inputs, *outputs, *cur;
    int ret, i, init = !fg->graph, simple = !fg->graph_desc;
    const char *graph_desc = simple ? fg->outputs[0]->ost->avfilter :
                                      fg->graph_desc;

    avfilter_graph_free(&fg->graph);
    if (!(fg->graph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);

    if (simple) {
        OutputStream *ost = fg->outputs[0]->ost;
        char args[255];
        snprintf(args, sizeof(args), "flags=0x%X", (unsigned)ost->sws_flags);
        fg->graph->scale_sws_opts = av_strdup(args);
    }

    if ((ret = avfilter_graph_parse2(fg->graph, graph_desc, &inputs, &outputs)) < 0)
        return ret;

    if (simple && (!inputs || inputs->next || !outputs || outputs->next)) {
        av_log(NULL, AV_LOG_ERROR, "Simple filtergraph '%s' does not have "
               "exactly one input and output.\n", graph_desc);
        return AVERROR(EINVAL);
    }

    for (cur = inputs; !simple && init && cur; cur = cur->next)
        init_input_filter(fg, cur);

    for (cur = inputs, i = 0; cur; cur = cur->next, i++)
        if ((ret = configure_input_filter(fg, fg->inputs[i], cur)) < 0)
            return ret;
    avfilter_inout_free(&inputs);

    if (!init || simple) {
        /* we already know the mappings between lavfi outputs and output streams,
         * so we can finish the setup */
        for (cur = outputs, i = 0; cur; cur = cur->next, i++)
            configure_output_filter(fg, fg->outputs[i], cur);
        avfilter_inout_free(&outputs);

        if ((ret = avfilter_graph_config(fg->graph, NULL)) < 0)
            return ret;
    } else {
        /* wait until output mappings are processed */
        for (cur = outputs; cur;) {
            fg->outputs = grow_array(fg->outputs, sizeof(*fg->outputs),
                                     &fg->nb_outputs, fg->nb_outputs + 1);
            if (!(fg->outputs[fg->nb_outputs - 1] = av_mallocz(sizeof(*fg->outputs[0]))))
                exit(1);
            fg->outputs[fg->nb_outputs - 1]->graph   = fg;
            fg->outputs[fg->nb_outputs - 1]->out_tmp = cur;
            cur = cur->next;
            fg->outputs[fg->nb_outputs - 1]->out_tmp->next = NULL;
        }
    }

    return 0;
}

int ist_in_filtergraph(FilterGraph *fg, InputStream *ist)
{
    int i;
    for (i = 0; i < fg->nb_inputs; i++)
        if (fg->inputs[i]->ist == ist)
            return 1;
    return 0;
}

