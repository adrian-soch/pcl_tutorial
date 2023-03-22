/**
 * @file point_cloud_processing.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @version 0.1
 * @date 2023-03-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pcl_tutorial/point_cloud_processing.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_tutorial
{

void PointCloudProcessor::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
{
    // Use for timing callback execution time
    auto start = std::chrono::high_resolution_clock::now();

    // Transform for pointcloud in world frame
    geometry_msgs::msg::TransformStamped stransform;
    try {
        stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                    tf2::TimePointZero, tf2::durationFromSec(3));
    }
    catch (const tf2::TransformException &ex) {
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
        * STATISTICAL OUTLIER REMOVAL
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_input_cloud(new pcl::PointCloud<pcl::PointXYZI>(xyz_filtered_cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(sor_input_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*sor_cloud_filtered);
    
    /* ========================================
        * PLANE SEGEMENTATION
        * ========================================*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(plane_max_iter);
    seg.setDistanceThreshold(plane_dist_thresh);

    // Segment the largest planar component from the cropped cloud
    seg.setInputCloud(sor_cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(sor_cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);

    /* ========================================
        * EUCLIDEAN CLUSTER EXTRACTION
        * ========================================*/
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    *cloud_filtered = *cloud_f;
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tol);
    ec.setMinClusterSize(cluster_min_size);
    ec.setMaxClusterSize(cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<vision_msgs::msg::Detection3D> bboxes;
    
    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Put each cluster into a vector
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->points.push_back((*cloud_filtered)[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);

        // Init and fill bboxes
        vision_msgs::msg::Detection3D d3d;
        vision_msgs::msg::BoundingBox3D bb = PointCloudProcessor::getOrientedBoudingBox(*cloud_cluster);

        // Perform classification here to obtain object ID
        d3d.id = "id";
        d3d.header.stamp = recent_cloud->header.stamp;
        d3d.bbox = bb;
        
        bboxes.push_back(d3d);
    }

    /* ========================================
        * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
        * ========================================*/
    this->publishPointCloud(voxel_grid_pub_, *cloud_voxel_filtered);
    this->publishPointCloud(plane_pub_, *cloud_f);
    this->publishPointCloud(euclidean_pub_, *clusters[0]);
    this->publishPointCloud(stat_pub_, *sor_cloud_filtered);

    this->publish3DBBoxOBB(marker_array_pub_, bboxes);

    // Get duration and log to console
    auto stop = std::chrono::high_resolution_clock::now();
    auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
}

template <typename PointT>
vision_msgs::msg::BoundingBox3D PointCloudProcessor::getOrientedBoudingBox(const pcl::PointCloud<PointT> &cloud_cluster)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud_cluster, centroid);
    PointT min_pt, max_pt;
    Eigen::Vector3f center;
    pcl::getMinMax3D(cloud_cluster, min_pt, max_pt); 
    center = (max_pt.getVector3fMap() + min_pt.getVector3fMap())/2;
    
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cloud_cluster, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    // Eigen::Vector3f eigenValuesPCA  = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    Eigen::Matrix3f eigenVectorsPCA1;
    eigenVectorsPCA1.col(0) = eigenVectorsPCA.col(2);
    eigenVectorsPCA1.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsPCA1.col(2) = eigenVectorsPCA.col(0);
    eigenVectorsPCA = eigenVectorsPCA1;

    Eigen::Vector3f ea = (eigenVectorsPCA).eulerAngles(2, 1, 0); //yaw pitch roll
    Eigen::AngleAxisf keep_Z_Rot(ea[0], Eigen::Vector3f::UnitZ());
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(center);
    transform.rotate(keep_Z_Rot); // radians
    
    typename pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(cloud_cluster, *transformedCloud, transform.inverse());

    PointT min_pt_T, max_pt_T;
    pcl::getMinMax3D(*transformedCloud, min_pt_T, max_pt_T);
    Eigen::Vector3f center_new = (max_pt_T.getVector3fMap() + min_pt_T.getVector3fMap()) / 2;
    Eigen::Vector3f box_dim;
    box_dim = max_pt_T.getVector3fMap() - min_pt_T.getVector3fMap();
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translate(center_new);
    // Eigen::Affine3f transform3 = transform * transform2;

    const Eigen::Quaternionf bboxQ(keep_Z_Rot);

    PointT size;
    size.getArray3fMap() = max_pt_T.getArray3fMap() - min_pt_T.getArray3fMap();

    vision_msgs::msg::BoundingBox3D bb;
    bb.center.position.x = centroid[X];
    bb.center.position.y = centroid[Y];
    bb.center.position.z = centroid[Z];
    bb.center.orientation.x = bboxQ.x();
    bb.center.orientation.y = bboxQ.y();
    bb.center.orientation.z = bboxQ.z();
    bb.center.orientation.w = bboxQ.w();
    bb.size.x = size.x;
    bb.size.y = size.y;
    bb.size.z = size.z;

    return bb;
}

void PointCloudProcessor::publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
        pcl::PointCloud<pcl::PointXYZI> point_cloud)
{
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(point_cloud, *pc2_cloud);
    pc2_cloud->header.frame_id = world_frame;
    pc2_cloud->header.stamp = this->get_clock()->now();
    publisher->publish(*pc2_cloud);
}

void PointCloudProcessor::publish3DBBoxOBB(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
    const std::vector<vision_msgs::msg::Detection3D>& bboxes)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int idx = 0;
    for(auto c : bboxes) {
        visualization_msgs::msg::Marker marker;
        marker.id = idx++;
        marker.header.frame_id = world_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = c.bbox.center; // Sets position and orientation
        marker.scale = c.bbox.size;  // Set w,l,h
        
        marker.color.r = 0.0;
        marker.color.b = 0.0;
        marker.color.g = 0.7;
        marker.color.a = 0.4;   // Set alpha so we can see underlying points

        marker_array.markers.push_back(marker);
    }
    publisher->publish(marker_array);
} 

} // end namespace pcl_tutorial
