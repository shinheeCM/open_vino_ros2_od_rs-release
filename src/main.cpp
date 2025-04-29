#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "object_detection/inference.h"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>

// colcon build --packages-select object_detection --symlink-install
// ros2 launch object_detection object_detection_launch.py

using namespace std;
using namespace cv;
rs2::pipeline pipe_;
class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode() : Node("object_detection_node") {
        // Camera stream
        // cap.open(0);  // Open default camera

        // ROS publishers
        // image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detection/image", 10);
        // objects_pub_ = this->create_publisher<std_msgs::msg::String>("detection/objects", 10);

        this->declare_parameter<std::string>("coco_yaml_path", "coco.yaml");
        this->declare_parameter<std::string>("onnx_model_path", "yolov8n.onnx");

        std::string coco_path = this->get_parameter("coco_yaml_path").as_string();
        std::string onnx_path = this->get_parameter("onnx_model_path").as_string();

        // Initialize YOLOv8
        yoloDetector_ = new YOLO_V8;
        ReadCocoYaml(yoloDetector_, coco_path);
        
        DL_INIT_PARAM params;
        params.rectConfidenceThreshold = 0.1;
        params.iouThreshold = 0.5;
        params.modelPath = onnx_path;
        params.imgSize = {640, 640};
        params.cudaEnable = false;
        params.modelType = YOLO_DETECT_V8;
        yoloDetector_->CreateSession(params);

        // Create a RealSense pipeline
        rs2::config cfg;
        rs2::colorizer color_map;

        // Enable the color and depth streams
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        // Start the pipeline
        pipe_.start(cfg);
        
        // Timer for processing frames at 30Hz
        timer_ = this->create_wall_timer(
            chrono::milliseconds(33), 
            std::bind(&ObjectDetectionNode::process_frame, this)
        );
    }


private:
    int ReadCocoYaml(YOLO_V8*& p, const std::string& yaml_path) {
        std::ifstream file(yaml_path);
        if (!file.is_open()) {
            std::cerr << "❌ Failed to open coco.yaml" << std::endl;
            return 1;
        }

        std::string line;
        std::vector<std::string> lines;
        while (std::getline(file, line)) {
            lines.push_back(line);
        }

        std::size_t start = 0, end = lines.size();
        for (std::size_t i = 0; i < lines.size(); i++) {
            if (lines[i].find("names:") != std::string::npos) {
                start = i + 1;
            } else if (start > 0 && lines[i].find(':') == std::string::npos) {
                end = i;
                break;
            }
        }

        std::vector<std::string> names;
        for (std::size_t i = start; i < end; i++) {
            std::stringstream ss(lines[i]);
            std::string index, name;
            std::getline(ss, index, ':');
            std::getline(ss, name);
            name.erase(0, name.find_first_not_of(" \t")); // trim
            names.push_back(name);
        }

        p->classes = names;
        return 0;
    }
    void process_frame() {
        // cv::Mat frame;
        // cap >> frame;

        rs2::frameset frames = pipe_.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        // Convert RealSense frame to OpenCV matrix
        // Mat color_image(Size(color_frame.as<rs2::video_frame>().get_width(),
        //                      color_frame.as<rs2::video_frame>().get_height()),
        //                 CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        Mat color_image(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);


        if (color_image.empty()) return;

        std::vector<DL_RESULT> results;
        yoloDetector_->RunSession(color_image, results);

        // Publish detection image
        cv::Mat display_frame = color_image.clone();
        for (auto& result : results) {
            cv::RNG rng(cv::getTickCount());
            cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

            // Draw bounding box
            cv::rectangle(display_frame, result.box, color, 2);

            // Prepare label with confidence
            float confidence = floor(100 * result.confidence) / 100;
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2) << confidence;
            std::string label = yoloDetector_->classes[result.classId] + " " + oss.str();

            // Draw label background and text
            cv::rectangle(display_frame, 
                          cv::Point(result.box.x, result.box.y - 25),
                          cv::Point(result.box.x + label.length() * 15, result.box.y),
                          color, cv::FILLED);
            cv::putText(display_frame, label, 
                        cv::Point(result.box.x, result.box.y - 5), 
                        cv::FONT_HERSHEY_SIMPLEX, 
                        0.75, cv::Scalar(0, 0, 0), 2);
        }

        // Convert OpenCV frame to ROS Image message
        sensor_msgs::msg::Image::SharedPtr ros_image = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", display_frame
        ).toImageMsg();

        // Publish the image
        // image_pub_->publish(*ros_image);

        // Create object detection result string
        std::ostringstream objects_ss;
        for (const auto& result : results) {
            objects_ss << yoloDetector_->classes[result.classId] << " ";
        }

        imshow("RealSense Color Image", display_frame);
        // Wait for the user to press 'q' to exit
        if (waitKey(1) == 'q') {
            pipe_.stop();
            rclcpp::shutdown();
        }
        // Publish the objects detected
        // auto objects_msg = std_msgs::msg::String();
        // objects_msg.data = objects_ss.str();
        // objects_pub_->publish(objects_msg);
    }

    cv::VideoCapture cap;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objects_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    YOLO_V8* yoloDetector_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    // pipe_.stop();
    rclcpp::shutdown();
    return 0;
}
