/**
 * @file point_cloud_processing_node.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief Creates and begins an instance of the PointCloudProcessor class
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "pcl_tutorial/point_cloud_processing.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pcl_tutorial::PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
