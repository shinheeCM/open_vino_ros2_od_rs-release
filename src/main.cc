#include "open_vino_ros2_od_rs/inference.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>  // Include RealSense SDK

class YoloVideoInferenceNode : public rclcpp::Node {
public:
    YoloVideoInferenceNode() : Node("yolo_video_inference_node") {
        this->declare_parameter<std::string>("model_path", "/home/dj/ObjectDetection/open_vino/yolov8n_openvino_model/yolov8n.xml");
        // this->declare_parameter<std::vector<std::string>>("class_names", {
        //     "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        //     "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        //     "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        //     "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        //     "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        //     "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        //     "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
        //     "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        //     "scissors", "teddy bear", "hair drier", "toothbrush"
        // });

        // this->get_parameter("class_names", yolo::Inference.classes_);

        std::vector<std::string> class_names;
        this->declare_parameter("class_names", std::vector<std::string>{
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", 
            "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", 
            "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", 
            "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", 
            "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", 
            "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", 
            "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", 
            "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", 
            "scissors", "teddy bear", "hair drier", "toothbrush"
        });
        this->get_parameter("class_names", class_names);

        std::string model_path = this->get_parameter("model_path").as_string();

        if (model_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'model_path' must be set.");
            return;
        }

        // Define thresholds for confidence and NMS
        const float confidence_threshold = 0.5;
        const float NMS_threshold = 0.5;

        // Initialize YOLO Inference
        yolo::Inference inference(model_path, cv::Size(640, 640), confidence_threshold, NMS_threshold, class_names);

        // Initialize RealSense pipeline
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        // cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);


        try {
            pipe.start(cfg);
        } catch (const rs2::error & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start RealSense pipeline: %s", e.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RealSense camera started successfully.");

        while (rclcpp::ok()) {
            // Wait for a new frame set
            rs2::frameset frames = pipe.wait_for_frames();

            // Get the color frame
            rs2::video_frame color_frame = frames.get_color_frame();

            if (!color_frame) {
                RCLCPP_WARN(this->get_logger(), "No color frame received.");
                continue;
            }

            // Convert RealSense frame to OpenCV Mat
            cv::Mat frame(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            // Run inference
            inference.RunInference(frame);

            // Display the frame
            cv::imshow("YOLO Detection - RealSense", frame);

            if (cv::waitKey(1) == 'q') {
                rclcpp::shutdown();
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }

        pipe.stop();
        cv::destroyAllWindows();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YoloVideoInferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
