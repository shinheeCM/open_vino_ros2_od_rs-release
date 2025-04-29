from launch import LaunchDescription
from launch_ros.actions import Node
# ros2 run open_vino_ros2_od_rs open_vino_ros2_od_rs
# colcon build --packages-select open_vino_ros2_od_rs --symlink-install
# ros2 launch open_vino_ros2_od_rs rs_object_detection_open_vino_launch.py
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='open_vino_ros2_od_rs',
            executable='open_vino_ros2_od_rs',
            name='open_vino_ros2_od_rs',
            parameters=[
                {'model_path': '/home/dj/ObjectDetection/open_vino/yolov8n_openvino_model/pallet_mix_april_25_253/weights/pallet_mix_april_25_253_openvino_model/pallet_mix_april_25_253.xml'},
                {'class_names': [
                    "pallet", "_", "_"
                ]}
            ],
            output='screen'
        )
    ])
