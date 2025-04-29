cmake_minimum_required(VERSION 3.12)
project(yolov8_openvino_example)

# set(CMAKE_CXX_STANDARD 14)
# Set C++17 instead of C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Optional: Avoids using gnu++17, enforces standard c++17


find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	/opt/intel/openvino_2025/runtime/include
)

add_executable(detect 
	main.cc
	inference.cc
)


# add_executable(detect_live
# 	main_live.cc
# 	inference.cc
# )


target_link_libraries(detect
	${OpenCV_LIBS}
	/opt/intel/openvino_2025/runtime/lib/intel64/libopenvino.so
)
