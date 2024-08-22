#pragma once
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include<pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

void ComputeEigenVectorPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector4f& pcaCentroid, Eigen::Matrix3f& eigenVectorsPCA);
Eigen::Matrix4f PCARegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr& P_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& X_cloud);
void  visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& regist);
