# YOLOv8 ONNX Runtime C++ Example
### CLI

### build && run
```
    download and unzip it in current directory <!-- https://github.com/microsoft/onnxruntime/releases/download/v1.15.1/onnxruntime-linux-x64-gpu-1.15.1.tgz -->

    ./build.sh
    ./run.sh 
```

```bash
# Export the model using the command line
yolo export model=yolov8n.pt format=onnx opset=12 simplify=True dynamic=False imgsz=640
```

**Create Build Directory:**

    ```bash
    mkdir build && cd build
    ```

**Configure with CMake:**
    ```
        cmake ..
        make

    ```
