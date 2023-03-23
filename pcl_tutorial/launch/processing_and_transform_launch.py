from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    '''
    Create a static transformer to translate the lidar data to another frame.

    [x y z yaw pitch roll frame_id child_frame_id]
    '''
    s_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0.2', '0', 'map', 'laser_data_frame']
    )

    perception_node = Node(
        package='pcl_tutorial',
        executable='pcl_tutorial',
        name='point_cloud_processing_node',
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
    
    # Items above will only be launched if they are present in this list
    return LaunchDescription([
        s_transform,
        perception_node
   ])