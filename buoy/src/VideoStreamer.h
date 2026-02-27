#pragma once

#include <string>
#include <thread>
#include <atomic>

// Thin wrapper around a GStreamer pipeline that captures from the
// Raspberry Pi camera and streams h264 over UDP.  The pipeline is built
// in the constructor and executed in a background thread; the caller
// can destroy the object to stop streaming.

class VideoStreamer {
public:
    VideoStreamer(const std::string &dstIp, int dstPort);
    ~VideoStreamer();

    // non-copyable, movable only for simplicity
    VideoStreamer(const VideoStreamer &) = delete;
    VideoStreamer &operator=(const VideoStreamer &) = delete;
    VideoStreamer(VideoStreamer &&) = delete;
    VideoStreamer &operator=(VideoStreamer &&) = delete;

    // start/stop are implicit, but these may be useful if you want to
    // control timing explicitly.
    void start();
    void stop();

private:
    void threadFunc();

    std::atomic<bool> running_{false};
    std::thread worker_;
    std::string pipelineDesc_;
    void *pipeline_ = nullptr; // GstElement*, keep void* here to avoid gst header in header
};
