#!/bin/bash

colcon build --packages-select object_detection open_vino_ros2_od open_vino_ros2_od_rs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# colcon build --packages-select open_vino_ros2_od_rs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# Intel real sense ONNX runtime
# ros2 launch object_detection object_detection_launch.py

# colcon build --packages-select open_vino_ros2_od --symlink-install
# webcam
# ros2 launch open_vino_ros2_od object_detection_open_vino_launch.py

# Intel real sense OPenVINO
ros2 launch open_vino_ros2_od_rs rs_object_detection_open_vino_launch.py

# bloom-release --ros-distro humble open_vino_ros2_od_rs
# catkin_prepare_release
# bloom-release --new-track --rosdistro humble --track humble open_vino_ros2_od