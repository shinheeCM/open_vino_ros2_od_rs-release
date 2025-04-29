#include "inference.h"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char **argv) {
    // Check if the correct number of arguments is provided
    if (argc != 3) {
        std::cerr << "usage: " << argv[0] << " <model_path> <video_path or camera_id>" << std::endl;
        return 1;
    }
    
    // Get the model path and video source (could be video file or camera ID)
    const std::string model_path = argv[1];
    const std::string source = argv[2];

    // OpenCV VideoCapture object
    cv::VideoCapture cap;

    // Try to interpret the source as a camera ID (integer)
    if (isdigit(source[0])) {
        int camera_id = std::stoi(source);
        cap.open(camera_id);
    } else {
        cap.open(source);
    }

    // Check if the video source is opened successfully
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Unable to open video source: " << source << std::endl;
        return 1;
    }

    // Define the confidence and NMS thresholds
    const float confidence_threshold = 0.5;
    const float NMS_threshold = 0.5;

    // Initialize YOLO Inference
    yolo::Inference inference(model_path, cv::Size(640, 640), confidence_threshold, NMS_threshold);

    cv::Mat frame;
    while (true) {
        // Read a new frame from the video/camera
        cap >> frame;

        // Check if the frame is valid
        if (frame.empty()) {
            std::cout << "End of video stream or no frames received." << std::endl;
            break;
        }

        // Run inference on the frame
        inference.RunInference(frame);

        // Display the frame with detections
        cv::imshow("YOLO Detection", frame);

        // Break if 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
