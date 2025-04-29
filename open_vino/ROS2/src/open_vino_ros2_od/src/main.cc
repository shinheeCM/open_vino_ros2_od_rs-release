#include "open_vino_ros2_od/inference.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

class YoloVideoInferenceNode : public rclcpp::Node {
public:
    YoloVideoInferenceNode() : Node("yolo_video_inference_node") {
        // Declare parameters for model path and video source (file or camera ID)
        this->declare_parameter<std::string>("model_path", "/home/dj/ObjectDetection/open_vino/yolov8n_openvino_model/yolov8n.xml");
        this->declare_parameter<std::string>("video_source", "0");

        // Get the parameters
        std::string model_path = this->get_parameter("model_path").as_string();
        std::string video_source = this->get_parameter("video_source").as_string();

        // Validate if the parameters are set
        if (model_path.empty() || video_source.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameters 'model_path' and 'video_source' must be set.");
            return;
        }

        // OpenCV VideoCapture object
        cv::VideoCapture cap;

        // Try to interpret the source as a camera ID (integer) or video file
        if (isdigit(video_source[0])) {
            int camera_id = std::stoi(video_source);
            cap.open(camera_id);
        } else {
            cap.open(video_source);
        }

        // Check if the video source is opened successfully
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open video source: %s", video_source.c_str());
            return;
        }

        // Define thresholds for confidence and NMS
        const float confidence_threshold = 0.5;
        const float NMS_threshold = 0.5;

        // Initialize YOLO Inference
        yolo::Inference inference(model_path, cv::Size(640, 640), confidence_threshold, NMS_threshold);

        // Process video frames
        cv::Mat frame;
        while (rclcpp::ok()) {
            // Read a new frame from the video or camera
            cap >> frame;

            // Check if the frame is valid
            if (frame.empty()) {
                RCLCPP_INFO(this->get_logger(), "End of video stream or no frames received.");
                break;
            }

            // Run inference on the frame
            inference.RunInference(frame);

            // Display the frame with detections
            cv::imshow("YOLO Detection", frame);

            // Break if 'q' is pressed (in the OpenCV window)
            if (cv::waitKey(1) == 'q') {
                rclcpp::shutdown();
                // break;
            }

            // Handle ROS 2 callbacks
            rclcpp::spin_some(this->get_node_base_interface());
        }

        cap.release();
        cv::destroyAllWindows();
    }
};

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<YoloVideoInferenceNode>();

    // Spin the node to keep it alive
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
