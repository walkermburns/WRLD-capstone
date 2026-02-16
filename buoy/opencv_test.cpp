#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>

bool keepRunning = true;
void signalHandler(int s) { keepRunning = false; }

int main() {
    signal(SIGINT, signalHandler);

    // 1. Capture Raw Frames via V4L2
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) return -1;

    // Set format to YUYV (Raw) to ensure no hardware H264 decoding is happening
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 2. Prepare Software Encoder
    // 'X264' is a software-based H.264 implementation
    int codec = cv::VideoWriter::fourcc('X', '2', '6', '4');
    cv::VideoWriter writer("output.mp4", codec, 30, cv::Size(640, 480));

    cv::Mat frame;
    while (keepRunning) {
        if (!cap.read(frame)) break;

        // Draw a rectangle (The processing step)
        cv::rectangle(frame, cv::Point(150, 150), cv::Point(450, 350), cv::Scalar(0, 255, 0), 2);

        // Save frame using CPU-based encoding
        writer.write(frame);
    }

    std::cout << "Releasing resources and saving file..." << std::endl;
    cap.release();
    writer.release();
    return 0;
}