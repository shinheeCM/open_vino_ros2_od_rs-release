from launch import LaunchDescription
from launch_ros.actions import Node
# ros2 run open_vino_ros2_od open_vino_ros2_od
# colcon build --packages-select open_vino_ros2_od --symlink-install
# ros2 launch open_vino_ros2_od object_detection_open_vino_launch.py

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='open_vino_ros2_od',
            executable='open_vino_ros2_od',
            name='open_vino_ros2_od',
            parameters=[
                {'model_path': '/home/dj/ObjectDetection/open_vino/yolov8n_openvino_model/yolov8n.xml'},
                {'video_source': '0'}
            ],
            output='screen'
        )
    ])
