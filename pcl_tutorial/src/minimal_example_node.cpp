/**
 * @file minimal_example_node.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief Minimal working example of PCL and ROS 2
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>


/**
 * @brief Creates an object that will subscribe to a pointCloud2 topic and
 *        performs various basic tasks.
 *
 */
class MinimalPointCloudProcessor : public rclcpp::Node
{
public:
    MinimalPointCloudProcessor()
        : Node("minimal_point_cloud_processor", rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
    {
        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);
        crop_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("crop_cluster", 1);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param, voxel_leaf_size_param,
          x_filter_min_param, x_filter_max_param, y_filter_min_param, y_filter_max_param, z_filter_min_param,
          z_filter_max_param;

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/points"));
        this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("voxel_leaf_size", voxel_leaf_size_param, rclcpp::Parameter("", 0.25));
        this->get_parameter_or("x_filter_min", x_filter_min_param, rclcpp::Parameter("", 1.0));
        this->get_parameter_or("x_filter_max", x_filter_max_param, rclcpp::Parameter("", 120.0));
        this->get_parameter_or("y_filter_min", y_filter_min_param, rclcpp::Parameter("", -25.0));
        this->get_parameter_or("y_filter_max", y_filter_max_param, rclcpp::Parameter("", 10.0));
        this->get_parameter_or("z_filter_min", z_filter_min_param, rclcpp::Parameter("", -1.0));
        this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 8.0));

        cloud_topic     = cloud_topic_param.as_string();
        world_frame     = world_frame_param.as_string();
        camera_frame    = camera_frame_param.as_string();
        voxel_leaf_size = float(voxel_leaf_size_param.as_double());
        x_filter_min    = x_filter_min_param.as_double();
        x_filter_max    = x_filter_max_param.as_double();
        y_filter_min    = y_filter_min_param.as_double();
        y_filter_max    = y_filter_max_param.as_double();
        z_filter_min    = z_filter_min_param.as_double();
        z_filter_max    = z_filter_max_param.as_double();

        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

        cloud_subscriber_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 1, std::bind(&MinimalPointCloudProcessor::cloud_callback, this, std::placeholders::_1));

        /*
         * SET UP TF. Optional for transforming between coordinate frames
         *          You need to create a static tranform publisher to use this
         */
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:

    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr crop_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;

    float voxel_leaf_size;
    float x_filter_min, x_filter_max;
    float y_filter_min, y_filter_max;
    float z_filter_min, z_filter_max;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    /**
     * @brief Callback when PointCloud2 message is received. Processes the data and
     *      publishes intermediate results. Make sure this callback
     *      executes faster than the rate the message arrive.
     *
     * @param recent_cloud
     *
     * @result Published intermediate PointCloud2 results
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        // Use for timing callback execution time
        auto start = std::chrono::high_resolution_clock::now();

        // Transform for pointcloud in world frame
        geometry_msgs::msg::TransformStamped stransform;

        try {
            stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                tf2::TimePointZero, tf2::durationFromSec(3));
        }
        catch(const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

        // Convert ROS message to PCL type
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);

        /* ========================================
         * VOXEL GRID
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_filter.filter(*cloud_voxel_filtered);

        /* ========================================
         * CROPBOX
         * ========================================*/
        pcl::PointCloud<pcl::PointXYZI> xyz_filtered_cloud;
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud_voxel_filtered);
        Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
        Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
        crop.setMin(min_point);
        crop.setMax(max_point);
        crop.filter(xyz_filtered_cloud);

        /* ========================================
         * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
         * ========================================*/
        this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);
        this->publishPointCloud(crop_pub_, xyz_filtered_cloud);

        // Get duration and log to console
        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    } // cloud_callback

    /**
     * @brief Publishes a point cloud given a PCL cloud and a ROS publisher handle
     *
     * @param publisher
     * @param point_cloud
     */
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
      pcl::PointCloud<pcl::PointXYZI>                                                  point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp    = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }
}; // end MinimalPointCloudProcessor class

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
