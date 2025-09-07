from launch import LaunchDescription
import launch_ros.actions
import launch_ros.descriptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def generate_launch_description():

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=QoSProfile.HistoryPolicy.KEEP_LAST,
        depth=10
    )

    return LaunchDescription([

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_world_to_d415',
            arguments=[ #test
                '-0.450', '-0.300', '0.200',          # Translation: X, Y, Z
                '-0.354', '0.354', '0.612', '0.612',  # Rotation: Quaternion (xyzw)
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
                    parameters=[{'qos_profile': qos_profile}],
                    remappings=[
                        ('rgb/camera_info', '/D415/aligned_depth_to_color/camera_info'),
                        ('rgb/image_rect_color', '/D415/color/image_raw'),
                        ('depth_registered/image_rect', '/D415/aligned_depth_to_color/image_raw'),
                        ('points', '/D415/barista_points')],
                ),
                # launch_ros.descriptions.ComposableNode(
                #     package = 'pointcloud_filter',
                #     executable = 'pointcloud_filter_node',
                #     name = 'pointcloud_filter',
                # )
            ],
            output='screen',
        ),
    ])
