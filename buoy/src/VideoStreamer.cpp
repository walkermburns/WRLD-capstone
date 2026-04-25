#include "VideoStreamer.h"
#include <gst/gst.h>
#include <iostream>
#include <pthread.h>
#include <cstdint>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <cctype>
#include <sstream>

namespace {
std::mutex g_au_ts_mutex;
std::deque<uint64_t> g_au_ts_queue;
std::mutex g_frame_idx_mutex;
uint32_t g_next_frame_idx = 0;
int g_meta_sock = -1;
sockaddr_in g_meta_addr{};

#pragma pack(push, 1)
struct VideoTimestampPacket {
    uint32_t frame_idx_be;
    uint64_t timestamp_us_be;
};
#pragma pack(pop)

static uint64_t host_now_us()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static uint64_t htonll_u64(uint64_t x)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return (static_cast<uint64_t>(htonl(static_cast<uint32_t>(x & 0xFFFFFFFFULL))) << 32) |
           htonl(static_cast<uint32_t>(x >> 32));
#else
    return x;
#endif
}

constexpr const char *kCam0Name = "/base/soc/i2c0mux/i2c@0/imx708@1a";
constexpr const char *kCam1Name = "/base/soc/i2c0mux/i2c@1/imx708@1a";

static std::string to_lower_copy(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

static int camera_name_to_index(const std::string &cameraName)
{
    if (cameraName.find("i2c@0") != std::string::npos ||
        cameraName == kCam0Name) {
        return 0;
    }
    if (cameraName.find("i2c@1") != std::string::npos ||
        cameraName == kCam1Name) {
        return 1;
    }
    return -1;
}

static std::string camera_selector_to_name(const std::string &selector, int defaultIndex)
{
    if (selector.empty()) {
        return defaultIndex == 0 ? kCam0Name : kCam1Name;
    }

    const std::string lowered = to_lower_copy(selector);
    if (lowered == "0" || lowered == "cam0" || lowered == "camera0" ||
        lowered == "i2c@0") {
        return kCam0Name;
    }
    if (lowered == "1" || lowered == "cam1" || lowered == "camera1" ||
        lowered == "i2c@1") {
        return kCam1Name;
    }

    if (selector.find("i2c@0") != std::string::npos) {
        return kCam0Name;
    }
    if (selector.find("i2c@1") != std::string::npos) {
        return kCam1Name;
    }

    return selector;
}

// Latch one sender timestamp per encoded H264 access unit entering rtph264pay.
// Also send one UDP metadata packet immediately here.
static GstPadProbeReturn latch_access_unit_timestamp_probe(GstPad *, GstPadProbeInfo *info, gpointer)
{
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer)
        return GST_PAD_PROBE_OK;

    const uint64_t ts_us = host_now_us();

    {
        std::lock_guard<std::mutex> lock(g_au_ts_mutex);
        g_au_ts_queue.push_back(ts_us);
        while (g_au_ts_queue.size() > 64) {
            g_au_ts_queue.pop_front();
        }
    }

    uint32_t frame_idx = 0;
    {
        std::lock_guard<std::mutex> lock(g_frame_idx_mutex);
        frame_idx = g_next_frame_idx++;
    }

    ssize_t sent_bytes = -1;
    if (g_meta_sock >= 0) {
        VideoTimestampPacket pkt{};
        pkt.frame_idx_be = htonl(frame_idx);
        pkt.timestamp_us_be = htonll_u64(ts_us);

        sent_bytes = sendto(g_meta_sock,
                            &pkt,
                            sizeof(pkt),
                            0,
                            reinterpret_cast<const sockaddr *>(&g_meta_addr),
                            sizeof(g_meta_addr));
    }

    static uint64_t dbg_counter = 0;
    if ((++dbg_counter % 30) == 0) {
        size_t qsize = 0;
        {
            std::lock_guard<std::mutex> lock(g_au_ts_mutex);
            qsize = g_au_ts_queue.size();
        }

        g_print("[video meta tx] frame_idx=%u ts_us=%" G_GUINT64_FORMAT
                " sent=%zd qsize=%zu\n",
                frame_idx,
                ts_us,
                sent_bytes,
                qsize);
    }

    return GST_PAD_PROBE_OK;
}
} // namespace

VideoStreamer::VideoStreamer(const std::string &dstIp,
                             int dstPort,
                             int dstPort2,
                             const std::string &cameraSelector,
                             const std::string &camera2Selector)
{
    const std::string camPrimary = camera_selector_to_name(cameraSelector, 0);
    const int primaryIndex = camera_name_to_index(camPrimary);
    const int secondaryIndex = primaryIndex == 0 ? 1 : 0;
    const std::string camSecondary = camera_selector_to_name(camera2Selector, secondaryIndex);

    std::ostringstream pipeline;
    pipeline << "libcamerasrc camera-name=\"" << camPrimary << "\" "
             << "! video/x-raw,width=1920,height=1080,framerate=30/1 "
             << "! queue max-size-buffers=1 leaky=downstream "
             << "! v4l2h264enc extra-controls=\"controls,video_bitrate=20000000,repeat_sequence_header=1,iframe-period=30\" "
             << "! video/x-h264,level=(string)4 "
             << "! rtph264pay name=pay0 config-interval=1 pt=96 ssrc=305419896 seqnum-offset=1000 timestamp-offset=2000 "
             << "! udpsink name=usink host=" << dstIp << " port=" << dstPort << " sync=false";

    if (dstPort2 > 0) {
        pipeline << " "
                 << "libcamerasrc camera-name=\"" << camSecondary << "\" "
                 << "! video/x-raw,width=1280,height=720,framerate=30/1,format=I420 "
                 << "! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
                 << "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 key-int-max=30 bframes=0 cabac=false ref=1 sliced-threads=true threads=2 byte-stream=true "
                 << "! video/x-h264,profile=baseline "
                 << "! rtph264pay name=pay1 config-interval=1 pt=96 ssrc=305419897 seqnum-offset=3000 timestamp-offset=4000 "
                 << "! udpsink name=usink2 host=" << dstIp << " port=" << dstPort2 << " sync=false";
    }

    pipelineDesc_ = pipeline.str();

    std::cout << "[video] pipeline desc: " << pipelineDesc_ << std::endl;
    std::cout << "[video] stream1 camera-name=" << camPrimary << std::endl;
    if (dstPort2 > 0) {
        std::cout << "[video] stream2 enabled on port " << dstPort2
                  << " using camera-name=" << camSecondary << std::endl;
    }
    gst_init(nullptr, nullptr);

    const int metaPort = dstPort + 1;
    g_meta_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_meta_sock >= 0) {
        std::memset(&g_meta_addr, 0, sizeof(g_meta_addr));
        g_meta_addr.sin_family = AF_INET;
        g_meta_addr.sin_port = htons(metaPort);
        inet_pton(AF_INET, dstIp.c_str(), &g_meta_addr.sin_addr);
        std::cout << "[video] metadata UDP target " << dstIp << ":" << metaPort << std::endl;
    } else {
        std::cerr << "[video] warning: failed to create metadata UDP socket\n";
    }
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

    if (g_meta_sock >= 0) {
        close(g_meta_sock);
        g_meta_sock = -1;
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

    {
        GstElement *pay = gst_bin_get_by_name(GST_BIN(pipeline_), "pay0");
        if (pay) {
            GstPad *sinkpad = gst_element_get_static_pad(pay, "sink");
            if (sinkpad) {
                gst_pad_add_probe(sinkpad,
                                  GST_PAD_PROBE_TYPE_BUFFER,
                                  latch_access_unit_timestamp_probe,
                                  nullptr,
                                  nullptr);
                std::cout << "[video] enabled AU timestamp latching + UDP metadata send on pay0 sink\n";
                gst_object_unref(sinkpad);
            } else {
                std::cerr << "[video] warning: pay0 has no sink pad\n";
            }

            gst_object_unref(pay);
        } else {
            std::cerr << "[video] warning: rtph264pay element not found\n";
        }
    }

    std::cout << "[video] pay0 sink probe attached" << std::endl;
    std::cout << "[video] setting pipeline PLAYING\n";
    gst_element_set_state((GstElement*)pipeline_, GST_STATE_PLAYING);

    GstBus *bus = gst_element_get_bus((GstElement*)pipeline_);
    while (running_) {
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus,
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
    }
    gst_object_unref(bus);
}