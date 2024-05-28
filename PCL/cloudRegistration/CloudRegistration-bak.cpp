#include "icp.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
/*˵����
 *1���������Ǵ�pcl��ICPԴ���н�ѡ�����ĳ��򣬳���ʵ���ڡ�icp.h���ļ���
 *2��Ϊ�˼�������Դ�����ʡ���˺ܶ�����ת�����жϵĲ��֣����Ǳ�����PCL ICP�㷨��Ҫ���㺯����ʵ�ַ�ʽ���Ը�ϸ�ڸ���Ȥ��ͬѧ�������Ķ�Դ��
 *3��������ͬʱ�ṩ��PCL��ICP�Ľӿڣ��ԶԱȱ�������ֲû������
 *4�������̲ο�PCL ICP�ٷ����̣�http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
 *5�������⻶ӭ��ϵ��pant333@163.com
 */

int main() {

    /*---------------------------��������ICPƥ��ĵ�������----------------------*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    //����source����
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    //��ӡsource��������
    for (size_t i = 0; i < cloud_in->points.size(); ++i) std::cout << "    " <<
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    //    ����target���ƣ�target����Ϊ��source������ÿ�����x����+0.7���ɣ�û����ת������x����ƽ��0.7��Ϊ��ֵ,ͬ��y��ֵΪ-0.3
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
        cloud_out->points[i].y = cloud_in->points[i].y - 0.3f;
    }
    //��ӡtarget����
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    /*----------------------ʹ�ñ����򾫼��ICP����-------------------------------*/
    ICP<pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "my icp has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    /*-------------------PCL����ICPʵ�ַ���------------------*/
    std::cout << "--------------------------------" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_pcl;
    icp_pcl.setInputSource(cloud_in);
    icp_pcl.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final_pcl;
    icp_pcl.align(Final_pcl);
    std::cout << "pcl icp has converged:" << icp_pcl.hasConverged() << " score: " <<
        icp_pcl.getFitnessScore() << std::endl;
    std::cout << icp_pcl.getFinalTransformation() << std::endl;

    return 0;
}