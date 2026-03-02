#include "VideoStreamer.h"
#include <gst/gst.h>
#include <gst/rtp/rtp.h>    // helpers for working with RTP buffers
#include <iostream>
#include <pthread.h>
#include <cstdint>
#include <chrono>
#include <cstring>

static GstPadProbeReturn add_timestamp_extension_probe(GstPad *, GstPadProbeInfo *info, gpointer)
{
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer)
        return GST_PAD_PROBE_OK;

    buffer = gst_buffer_make_writable(buffer);
    GST_PAD_PROBE_INFO_DATA(info) = buffer;

    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (!gst_rtp_buffer_map(buffer, GST_MAP_READWRITE, &rtp))
        return GST_PAD_PROBE_OK;

    const uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // RFC5285 one-byte header extension carrying 8-byte timestamp payload.
    // Layout in extension payload area:
    //   byte 0: 4-bit id (1), 4-bit len-1 (7 => 8 bytes)
    //   byte 1..8: timestamp (big-endian)
    //   byte 9..11: padding
    gst_rtp_buffer_set_extension(&rtp, TRUE);
    if (gst_rtp_buffer_set_extension_data(&rtp, 0xBEDE, 3)) { // 3 words = 12 bytes
        guint16 profile = 0;
        gpointer data = nullptr;
        guint wordlen = 0;
        if (gst_rtp_buffer_get_extension_data(&rtp, &profile, &data, &wordlen) &&
            profile == 0xBEDE && wordlen * 4 >= 12) {
            guint8 *ext = static_cast<guint8 *>(data);
            ext[0] = static_cast<guint8>((1u << 4) | 7u); // id=1, len=8 => 7
            guint64 be_ts = GUINT64_TO_BE(static_cast<guint64>(ts_us));
            std::memcpy(ext + 1, &be_ts, sizeof(be_ts));
            std::memset(ext + 9, 0, 3);
        }
    }

    // telemetry: show timestamp and dump first bytes on every packet
    {
        const guint16 seq = gst_rtp_buffer_get_seq(&rtp);
        std::cout << "[video] ext ts_us=" << ts_us << " seq=" << seq << " raw=";
        GstMapInfo fullmap;
        if (gst_buffer_map(buffer, &fullmap, GST_MAP_READ)) {
            for (gsize i = 0; i < 24 && i < fullmap.size; ++i)
                printf("%02x", fullmap.data[i]);
            std::cout << std::endl;
            gst_buffer_unmap(buffer, &fullmap);
        } else {
            std::cout << "<map-fail>" << std::endl;
        }
    }

    gst_rtp_buffer_unmap(&rtp);
    return GST_PAD_PROBE_OK;
}

VideoStreamer::VideoStreamer(const std::string &dstIp, int dstPort)
{
    // build pipeline description string (note: do not use shell quoting or -e)
    // give the payloader a fixed name so we can look it up reliably later
    pipelineDesc_ = "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=30/1 ! "
                    "queue max-size-buffers=1 leaky=downstream ! "
                    "v4l2h264enc extra-controls=\"controls,video_bitrate=20000000,repeat_sequence_header=1,iframe-period=30\" ! "
                    "video/x-h264,level=(string)4 ! rtph264pay name=pay0 config-interval=1 pt=96 ssrc=305419896 seqnum-offset=1000 timestamp-offset=2000 ! "
                    "udpsink host=" + dstIp + " port=" + std::to_string(dstPort) + " sync=false";

    std::cout << "[video] pipeline desc: " << pipelineDesc_ << std::endl;
    gst_init(nullptr, nullptr);
    // thread not started here; caller must call start()
}

VideoStreamer::~VideoStreamer()
{
    std::cout << "[video] destructor called\n";
    stop();
}

void VideoStreamer::start()
{
    if (running_)
        return;
    std::cout << "[video] starting thread\n";
    running_ = true;
    worker_ = std::thread(&VideoStreamer::threadFunc, this);
}

void VideoStreamer::stop()
{
    if (!running_)
        return;
    std::cout << "[video] stopping thread\n";
    running_ = false;

    // wake up the bus loop by setting pipeline to NULL; msg will appear
    if (pipeline_) {
        gst_element_send_event((GstElement*)pipeline_, gst_event_new_eos());
        gst_element_set_state((GstElement*)pipeline_, GST_STATE_NULL);
    }

    if (worker_.joinable())
        worker_.join();

    if (pipeline_) {
        std::cout << "[video] tearing down pipeline\n";
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

void VideoStreamer::threadFunc()
{
    pthread_setname_np(pthread_self(), "video");
    GError *error = nullptr;
    std::cout << "[video] threadFunc creating pipeline\n";
    pipeline_ = gst_parse_launch(pipelineDesc_.c_str(), &error);
    if (!pipeline_) {
        std::cerr << "[video] Failed to create GStreamer pipeline: "
                  << (error ? error->message : "unknown") << std::endl;
        if (error) g_error_free(error);
        return;
    }

    // Attach probe to payloader src pad to inject microsecond timestamp into
    // RTP header extension (RFC5285 one-byte, id=1, len=8).
    {
        GstElement *pay = gst_bin_get_by_name(GST_BIN(pipeline_), "pay0");
        if (pay) {
            GstPad *srcpad = gst_element_get_static_pad(pay, "src");
            if (srcpad) {
                gst_pad_add_probe(srcpad, GST_PAD_PROBE_TYPE_BUFFER,
                                  add_timestamp_extension_probe, nullptr, nullptr);
                std::cout << "[video] enabled RTP timestamp extension injection (id=1)\n";
                gst_object_unref(srcpad);
            } else {
                std::cerr << "[video] warning: pay0 has no src pad\n";
            }
            gst_object_unref(pay);
        } else {
            std::cerr << "[video] warning: rtph264pay element not found\n";
        }
    }

    std::cout << "[video] setting pipeline PLAYING\n";
    gst_element_set_state((GstElement*)pipeline_, GST_STATE_PLAYING);

    // simple loop that watches the bus for EOS or errors or until stop()
    GstBus *bus = gst_element_get_bus((GstElement*)pipeline_);
    while (running_) {
        GstMessage *msg = gst_bus_timed_pop_filtered(bus,
                GST_MSECOND * 100,
                static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        if (msg) {
            GError *err;
            gchar *dbg;
            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &dbg);
                std::cerr << "[video] GStreamer error: " << err->message << std::endl;
                g_error_free(err);
                g_free(dbg);
                running_ = false;
                break;
            case GST_MESSAGE_EOS:
                running_ = false;
                break;
            default:
                break;
            }
            gst_message_unref(msg);
        }
        // timed pop allows the loop to wake every 100ms and check running_
    }
    gst_object_unref(bus);
}
