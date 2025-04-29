cmake_minimum_required(VERSION 3.5)

# Set the project name and version
set(PROJECT_NAME object_detection)
project(${PROJECT_NAME} VERSION 0.0.1 LANGUAGES CXX)

# Ensure using C++17 for filesystem support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# -------------- ONNXRUNTIME ------------------#
# Set ONNXRUNTIME_VERSION and path
set(ONNXRUNTIME_VERSION 1.15.1)
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}")

# Include ONNX Runtime headers
include_directories(${ONNXRUNTIME_ROOT}/include)
include_directories(
  include
)

# Set the project sources
set(PROJECT_SOURCES
        src/main.cpp
        src/inference.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# Link OpenCV, ONNX Runtime, and ROS 2 dependencies
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    ${realsense2_LIBRARY}
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
)

# Copy necessary files (e.g., coco.yaml and yolov8n.onnx) into the build directory
configure_file(coco.yaml ${CMAKE_CURRENT_BINARY_DIR}/coco.yaml COPYONLY)
configure_file(yolov8n.onnx ${CMAKE_CURRENT_BINARY_DIR}/yolov8n.onnx COPYONLY)

# Create images directory in the build folder
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/images
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# ROS 2 ament package definition
ament_package()
