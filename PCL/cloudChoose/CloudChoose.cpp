#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    std::vector< int > indices;
    if (event.getPointsIndices(indices) == -1)
        return;

    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);

    std::stringstream ss;
    std::string cloudName;
    ss << num++;
    ss >> cloudName;
    cloudName += "_cloudName";

    viewer->addPointCloud(clicked_points_3d, red, cloudName);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

void main()
{
    if (pcl::io::loadPCDFile("bunny.pcd", *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << std::endl;
        return;
    }

    viewer->addPointCloud(cloud, "bunny");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
}