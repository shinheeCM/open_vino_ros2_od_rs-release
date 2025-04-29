#include <iostream>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "inference.h"



int ReadCocoYaml(YOLO_V8*& p) {
    std::ifstream file("coco.yaml");
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


void RunDetector(YOLO_V8*& p) {
        int camIndex = 0;
        cv::VideoCapture cap;
        cap.open(camIndex);

        // std::cout << "🎥 Running detection on " << (isWebcam ? "webcam" : "video") << " stream..." << std::endl;

        while (true) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) break;

            std::vector<DL_RESULT> res;
            p->RunSession(frame, res);

            for (auto& re : res) {
                cv::RNG rng(cv::getTickCount());
                cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

                cv::rectangle(frame, re.box, color, 2);
                float confidence = floor(100 * re.confidence) / 100;
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << confidence;
                std::string label = p->classes[re.classId] + " " + oss.str();

                cv::rectangle(frame, 
                              cv::Point(re.box.x, re.box.y - 25),
                              cv::Point(re.box.x + label.length() * 15, re.box.y),
                              color, cv::FILLED);

                cv::putText(frame, label, 
                            cv::Point(re.box.x, re.box.y - 5), 
                            cv::FONT_HERSHEY_SIMPLEX, 
                            0.75, cv::Scalar(0, 0, 0), 2);
            }

            cv::imshow("YOLOv8 Detection", frame);
            if (cv::waitKey(1) == 27) break;  // ESC to exit
        }

        cap.release();
        cv::destroyAllWindows();
        return;
  
}



void DetectTest() {
    YOLO_V8* yoloDetector = new YOLO_V8;
    ReadCocoYaml(yoloDetector);

    DL_INIT_PARAM params;
    params.rectConfidenceThreshold = 0.1;
    params.iouThreshold = 0.5;
    params.modelPath = "yolov8n.onnx";  // Make sure this model exists in your directory
    params.imgSize = { 640, 640 };

#ifdef USE_CUDA
    params.cudaEnable = true;
    params.modelType = YOLO_DETECT_V8;
#else
    params.cudaEnable = false;
    params.modelType = YOLO_DETECT_V8;
#endif

    yoloDetector->CreateSession(params);
    RunDetector(yoloDetector);
}



int main(int argc, char** argv) {
    DetectTest();
    return 0;
}