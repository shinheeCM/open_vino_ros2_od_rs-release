#!/bin/bash
wget https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8n.pt

# Valid Formats are supported: 'torchscript', 'onnx', 'openvino', 
    # 'engine', 'coreml', 'saved_model', 
    # 'pb', 'tflite', 'edgetpu', 
    # 'tfjs', 'paddle', 'ncnn'

# python3 model_export.py <input_model_path> <output_format>
python3 model_export.py /home/dj/ObjectDetection/open_vino/yolov8n.pt onnx
python3 model_export.py /home/dj/ObjectDetection/open_vino/yolov8n.pt openvino