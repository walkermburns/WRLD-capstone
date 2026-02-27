#include "VideoStreamer.h"
#include <gst/gst.h>
#include <iostream>

VideoStreamer::VideoStreamer(const std::string &dstIp, int dstPort)
{
    // build pipeline description string (note: do not use shell quoting or -e)
    pipelineDesc_ = "libcamerasrc ! video/x-raw,width=1920,height=1080,framerate=30/1 ! "
                    "queue max-size-buffers=1 leaky=downstream ! "
                    "v4l2h264enc extra-controls=\"controls,video_bitrate=20000000,repeat_sequence_header=1,iframe-period=30\" ! "
                    "video/x-h264,level=(string)4 ! rtph264pay config-interval=1 pt=96 ! "
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
    GError *error = nullptr;
    std::cout << "[video] threadFunc creating pipeline\n";
    pipeline_ = gst_parse_launch(pipelineDesc_.c_str(), &error);
    if (!pipeline_) {
        std::cerr << "[video] Failed to create GStreamer pipeline: "
                  << (error ? error->message : "unknown") << std::endl;
        if (error) g_error_free(error);
        return;
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
