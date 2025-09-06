from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[
                        ('rgb/camera_info', '/camera_depth_sensor/depth/camera_info'),
                        ('rgb/image_rect_color', '/camera_depth_sensor/image_raw'),
                        ('depth_registered/image_rect', '/camera_depth_sensor/depth/image_raw'),
                        ('points', '/camera_depth_sensor/my_points')],
                ),
            ],
            output='screen',
        ),
    ])
