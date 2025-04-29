cmake_minimum_required(VERSION 3.12)
project(open_vino_ros2_od)

# Set C++17 standard for ROS 2 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Optional: Avoids using gnu++17, enforces standard C++17

# Find dependencies (ROS 2, OpenCV, and OpenVINO)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)

# Include directories for OpenCV and OpenVINO
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    /opt/intel/openvino_2025/runtime/include
)

# Define the executable
add_executable(${PROJECT_NAME}
    src/main.cc
    src/inference.cc
)

# Link libraries (OpenCV and OpenVINO)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    /opt/intel/openvino_2025/runtime/lib/intel64/libopenvino.so
)

# Add dependencies for ROS 2
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Install the executable
install(TARGETS ${PROJECT_NAME} 
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


# ROS 2 ament package definition
ament_package()