import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    lidarslam_params = os.path.join(get_package_share_directory('lidarslam'), 'param', 'lidarslam.yaml')

    lidarslam = Node(
        package='lidarslam',
        executable='lidarslam',
        parameters=[lidarslam_params],
        remappings=[
            ('input_cloud','/lidar/objects'),
            # ('input_cloud','velodyne_points'),
            ('current_pose','slam/pose'),
            ('map','slam/map'),
            ('path','slam/path_tracked'),
            ('map_array','slam/map_array'),
            ('modified_map','slam/modified_map'),
            ('modified_map_array','slam/modified_map_array'),
            ('modified_path','slam/modified_path'),
        ],
        output='screen'
    )

    # slam_container = ComposableNodeContainer(
    #     name="slam_container",
    #     namespace="",
    #     package="rclcpp_components",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         # composed node for faster msg transfer
    #         ComposableNode(
    #             package='scanmatcher',
    #             name='scan_matcher',
    #             plugin='graphslam::ScanMatcherComponent',
    #             parameters=[lidarslam_params],
    #             remappings=[
    #                 ('input_cloud','velodyne_points'),
    #                 ('current_pose','slam/pose'),
    #                 ('map','slam/map'),
    #                 ('map_array','slam/map_array'),
    #                 ('path','slam/path_tracked'),
    #             ],
    #             extra_arguments=[{"use_intra_process_comms": True}],
    #         ),
    #         ComposableNode(
    #             package='graph_based_slam',
    #             name='graph_based_slam',
    #             plugin='graphslam::GraphBasedSlamComponent',
    #             parameters=[lidarslam_params],
    #             remappings=[
    #                 ('map_array','slam/map_array'),
    #                 ('modified_map','slam/modified_map'),
    #                 ('modified_map_array','slam/modified_map_array'),
    #                 ('modified_path','slam/modified_path'),
    #             ],
    #             extra_arguments=[{"use_intra_process_comms": True}],
    #         ),
    #     ],
    #     output="both",
    # )

    return LaunchDescription(
        [
            lidarslam
        ]
    )