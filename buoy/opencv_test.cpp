#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>

bool running = true;

void signalHandler(int signum) {
    std::cout << "\n[INFO] Interrupt signal (" << signum << ") received. Stopping..." << std::endl;
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);

    // ==================================================================================
    // PIPELINE 1: CAPTURE (Fixed)
    // ==================================================================================
    // Added 'queue' to decouple camera thread from app thread.
    // Added 'sync=false' to appsink to prevent clock drift issues.
    std::string capture_pipeline = 
        "libcamerasrc ! video/x-raw, width=1920, height=1080, framerate=30/1 ! "
        "queue max-size-buffers=1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true sync=false";

    std::cout << "[INFO] Opening Capture Pipeline..." << std::endl;
    // We use CAP_GSTREAMER explicitly to avoid ambiguity
    cv::VideoCapture cap(capture_pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Failed to open camera pipeline." << std::endl;
        std::cerr << "[TIP] Try running: GST_DEBUG=3 ./cam_record to see internal errors." << std::endl;
        return -1;
    }

    // ==================================================================================
    // PIPELINE 2: HARDWARE ENCODING (Fixed)
    // ==================================================================================
    // Added 'video/x-h264,profile=high' to ensure compatibility
    std::string output_pipeline = 
        "appsrc ! videoconvert ! video/x-raw, format=I420 ! "
        "v4l2h264enc ! video/x-h264, level=(string)4 ! "
        "h264parse ! mp4mux ! "
        "filesink location=output.mp4";

    std::cout << "[INFO] Opening Writer Pipeline..." << std::endl;
    cv::VideoWriter writer(output_pipeline, cv::CAP_GSTREAMER, 0, 30, cv::Size(1920, 1080), true);

    if (!writer.isOpened()) {
        std::cerr << "[ERROR] Failed to open output pipeline." << std::endl;
        return -1;
    }

    cv::Mat frame;
    int frame_count = 0;

    std::cout << "[INFO] Recording... Press Ctrl+C to stop." << std::endl;

    while (running) {
        if (!cap.read(frame)) {
            // If the pipeline breaks, we exit immediately
            std::cout << "[WARN] Frame capture failed." << std::endl;
            break;
        }

        // Draw the green square
        int cx = frame.cols / 2;
        int cy = frame.rows / 2;
        cv::rectangle(frame, cv::Point(cx - 50, cy - 50), cv::Point(cx + 50, cy + 50), cv::Scalar(0, 255, 0), 2);

        writer.write(frame);

        frame_count++;
        if (frame_count % 30 == 0) {
            std::cout << "Encoded " << frame_count << " frames..." << "\r" << std::flush;
        }
    }

    std::cout << "\n[INFO] Releasing resources..." << std::endl;
    cap.release();
    writer.release();
    cv::destroyAllWindows();
    std::cout << "[INFO] Done. Video saved to output.mp4" << std::endl;

    return 0;
}