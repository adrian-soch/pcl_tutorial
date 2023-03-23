from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_tutorial',
            executable='minimal_pcl_tutorial',
            name='minimal_example_node',
            # prefix='valgrind --leak-check=yes ',      # Uncomment to run valgrind 
            output='screen',
            parameters=[
                {"cloud_topic": "/points"},
                {"world_frame": "laser_data_frame"},
                {"camera_frame": "laser_data_frame"},
                {"voxel_leaf_size": 0.25},          # Size in meters of voxel grid filter
                {"x_filter_min": 1.0},              # Dimensions in meters to crop point cloud
                {"x_filter_max": 120.0},
                {"y_filter_min": -25.0},
                {"y_filter_max": 10.0},
                {"z_filter_min": -15.0},
                {"z_filter_max": 15.0},
                {"plane_max_iterations": 100},      # RANSAC max iteration value
                {"plane_distance_threshold": 0.4},  # in meters
                {"cluster_tolerance": 1.5},         # in meters
                {"cluster_min_size": 3},            # number of points
                {"cluster_max_size": 2000}
            ]
        )
     ])
