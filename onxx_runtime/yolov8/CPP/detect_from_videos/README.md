# YOLOv8 ONNX Runtime C++ Example
### CLI

### build && run
```
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
