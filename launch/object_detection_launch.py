from launch import LaunchDescription
from launch_ros.actions import Node
# colcon build --packages-select object_detection --symlink-install
# ros2 launch object_detection object_detection_launch.py
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            parameters=[
                {'coco_yaml_path': '/home/dj/robotics/ObjectDetection/onxx_runtime/yolov8/ROS2/src/object_detection/coco.yaml'},
                {'onnx_model_path': '/home/dj/robotics/ObjectDetection/onxx_runtime/yolov8/ROS2/src/object_detection/yolov8n.onnx'}
            ],
            output='screen'
        )
    ])
