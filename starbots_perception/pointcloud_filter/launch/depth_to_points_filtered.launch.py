from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():

    return LaunchDescription([

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_world_to_d415',
            arguments=[ # '-0.425', '-0.375', '0.352', '-0.285', '0.240', '0.482', '0.405'
                '-0.455', '-0.245', '0.420',          # Translation: X, Y, Z
                '-0.108', '0.105', '0.200', '0.195',  # Rotation: Quaternion (xyzw)
                'world', 'D415_link'
            ],
            output='screen',
        ),
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
                        ('rgb/camera_info', '/D415/aligned_depth_to_color/camera_info'),
                        ('rgb/image_rect_color', '/D415/color/image_raw'),
                        ('depth_registered/image_rect', '/D415/aligned_depth_to_color/image_raw'),
                        ('points', '/D415/barista_points')],
                    # plugin='depth_image_proc::PointCloudXyzNode',
                    # remappings=[
                    #     ('camera_info', '/D415/depth/camera_info'),
                    #     ('image_rect', '/D415/depth/image_rect_raw'),
                    #     ('points', '/D415/barista_points')],
                ),
                launch_ros.descriptions.ComposableNode(
                    package = 'pointcloud_filter',
                    executable = 'pointcloud_filter_node',
                    name = 'pointcloud_filter',
                ),
            ],
            output='screen',
        ),
    ])
