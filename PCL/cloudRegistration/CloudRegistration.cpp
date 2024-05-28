#include <iostream>  
#include <string>  
#include <pcl/io/ply_io.h>  //ply�ļ���ȡͷ�ļ�
#include <pcl/point_types.h>  
#include <pcl/registration/icp.h> //��׼����ͷ�ļ� 
#include <pcl/visualization/pcl_visualizer.h>  //���ӻ�����ͷ�ļ�
#include <pcl/console/time.h>   // TicToc  //ͷ�ļ���ʱ
#include <pcl/io/pcd_io.h>//pcd�ļ���ȡ
#include <pcl/filters/voxel_grid.h>//���ؽ������˲�

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool next_iteration = false;//����flag

//��ӡ������
void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.5f %6.5f %6.5f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.5f %6.5f %6.5f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.5f %6.5f %6.5f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.5f, %6.5f, %6.5f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}
//���̻ص�����
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	//������¿ո����next_iteration=true
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

int main()
{
	//������Ҫ�õ��ĵ��ƣ�����ģ�ת���ģ�ICP�������������ƣ� 
	PointCloudT::Ptr cloud_in_tgt(new PointCloudT);  // Original point cloud  ��Ŀ����ƣ�
	PointCloudT::Ptr cloud_toberegistration(new PointCloudT);  // Transformed point cloud	����׼�ĵ���
	PointCloudT::Ptr cloud_icp_in(new PointCloudT);   //filtered and icp input point cloud ��Ԥ�������icp�㷨��Ŀ�����
	PointCloudT::Ptr cloud_icp_toberegistration(new PointCloudT);  //filtered and icp output point cloud   ��Ԥ�������icp�㷨�Ĵ���׼����

	//ʱ��=time.tok-time.tic ���ڼ�ʱ
	pcl::console::TicToc time;
	time.tic();
	std::string filename1 = "pig_view1.pcd";//�����ļ���
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename1, *cloud_in_tgt) == -1)//*�򿪵����ļ�	
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");//�������ԭ�򣺶�ȡ�ļ�ʧ��
		return(-1);//�������
	}
	//�����ȡ���Ƶĵ�������ƶ�ȡʱ��
	std::cout << "\nLoaded file " << filename1 << " (" << cloud_in_tgt->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	/*����ȡ�ĵ��Ƹ��ƣ����ṩ��׼����Ĳ����Լ�������ӻ����ڵ���ʾ
	* cloud_icp_in����Ϊicp�㷨��Ŀ����Ʋ�������
	* cloud_toberegistration��icp�㷨��Դ���ƣ�����׼���ƣ�
	*/
	*cloud_icp_in = *cloud_in_tgt;

	std::string filename2 = "pig_view2.pcd";//�����ļ���
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename2, *cloud_toberegistration) == -1)//*�򿪵����ļ�	
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");//�������ԭ�򣺶�ȡ�ļ�ʧ��
		return(-1);//�������
	}

	//���ؽ������˲�
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;//�����˲�������
	voxel_grid.setLeafSize(0.01, 0.01, 0.01);//�������ش�С
	voxel_grid.setInputCloud(cloud_in_tgt);//������������ĵ���
	voxel_grid.filter(*cloud_icp_in);//�����������cloud_icp_in
	std::cout << "down size *cloud_in_tgt to" << cloud_icp_in->size() << endl;
	//ͬ����һ�����ƽ�����
	voxel_grid.setInputCloud(cloud_toberegistration);
	voxel_grid.filter(*cloud_icp_toberegistration);
	std::cout << "down size *cloud_toberegistration to" << cloud_icp_toberegistration->size() << endl;

	// ������תƽ�Ƶ�ת������
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)  
	double theta = M_PI / 4;  // The angle of rotation in radians 
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	//Z��ƽ��0.4��
	 //A translation on Z axis (0.4 meters)  
	transformation_matrix(2, 3) = 0.0;
	//��ӡ����ת����R��ƽ��T
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);
	std::cout << std::endl << std::endl << std::endl;

	//�þ���transformation_matrix�����ƽ��пռ�任���õ��ĵ��ƺ�Ŀ����Ƽ�����˿ռ����תƽ�ƹ�ϵ������ʹ��icp�㷨������׼��ԭ
	pcl::transformPointCloud(*cloud_toberegistration, *cloud_toberegistration, transformation_matrix);
	pcl::transformPointCloud(*cloud_icp_toberegistration, *cloud_icp_toberegistration, transformation_matrix);

	// Visualization ���ӻ� 
	pcl::visualization::PCLVisualizer viewer("ICP demo");//���ڱ���
	// Create two vertically separated viewports  
	int v1(0);//����4��С����
	int v2(1);
	int v3(2);
	int v4(3);
	//����С���ڵ�λ�÷ֲ�
	viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
	viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);
	// The color we will be using  
	float bckgr_gray_level = 0.0;  // Black  ��
	float txt_gray_lvl = 1.0 - bckgr_gray_level;//�ı���ɫ�뱳���෴

	viewer.addCoordinateSystem(1.0);//���������ᣬ1.0��������Ŀ��Ӵ�С
	// Original point cloud is white  Ŀ���������Ϊ��ɫ
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in_tgt, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	//��3�����������Ŀ����ƣ�Ŀ������ڿ��ӻ��Ĺ������ǲ��ᶯ��
	viewer.addPointCloud(cloud_in_tgt, cloud_in_color_h, "cloud_in_v1", v1);//��ʾcloud_in_tgt��һ�εĵ���
	viewer.addPointCloud(cloud_in_tgt, cloud_in_color_h, "cloud_in_v2", v2);
	viewer.addPointCloud(cloud_icp_in, cloud_in_color_h, "cloud_in_v3", v3);

	//Transformed point cloud is green  ��Ŀ�������תƽ�ƺ���δ�������ĵ�������Ϊ��ɫ�����ڴ���1��
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_toberegistration, 20, 180, 20);
	viewer.addPointCloud(cloud_toberegistration, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red  ����תƽ�ƺ�δ�������ĵ�������Ϊ��ɫ�����ڴ���2�����潫���ŵ������̸���
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_toberegistration, 180, 20, 20);
	viewer.addPointCloud(cloud_toberegistration, cloud_icp_color_h, "cloud_icp_v2", v2);

	// //ICP aligned point cloud is red  ��תƽ�Ʋ��������ĵ�������Ϊ��ɫ�����ڴ���3������icp�����룬�����ŵ������̸���λ��
	viewer.addPointCloud(cloud_icp_toberegistration, cloud_icp_color_h, "cloud_icp_v3", v3);

	//Adding text descriptions in each viewport  ���������Ϣ
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	/*icp��������iterations����Ϊ1ʱ���㷨ִ��һ�ε������㷨�ڲ�����ִ��һ�ε������ж���ǰ���������Ƿ���ڵ����趨����*/
	int iterations = 1;  // Default number of ICP iterations  icp������������Ϊ1
	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();//��ʾ�㷨�Ѿ������˶��ٴ�
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
	viewer.addText("Carlos Lee 202011", 10, 80, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "author", v2);

	//Set background color  ���ñ�����ɫ���������趨bckgr_gray_level=0.0�����Ա���ȫ��
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v3);

	//Set camera position and orientation  ���ÿ��ӻ����ڵĳ�ʼ�ӽ�
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024);  // Visualiser window size  �����������ӻ����ڴ�С  

	//Register keyboard callback :  //���̻ص���������Ӧ��������
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// The Iterative Closest Point algorithm  icp�㷨��ʼ
	time.tic();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);//����������������iterations�������Ѿ�����Ϊ0����ֻʹ��һ��icp�㷨
	icp.setInputSource(cloud_icp_toberegistration);//����Ŀ����ƣ���ԭ��ȡ��Ŀ�����ֻ������������ĵ���
	icp.setInputTarget(cloud_icp_in);//�������׼���ƣ���ԭ��ȡ��Ŀ����ƾ���תƽ�Ʋ���������ĵ���
	icp.align(*cloud_icp_toberegistration);//��1��icp�㷨��׼��ĵ���

	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();//��ȡ��ǰ������תƽ�ƾ���
	std::cout << endl;
	//ʹ�õõ��ı任��δ�������ĵ��ƽ��б任���Դﵽ��ʾЧ��
	pcl::transformPointCloud(*cloud_toberegistration, *cloud_toberegistration, icp_trans);
	// We set this variable to 1 for the next time we will call .align () function  ���õ�������Ϊ1����һ��ʹ�ü��̽��������һ�ε���
	icp.setMaximumIterations(1);
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())//�㷨�Ƿ���������
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;//�㷨��mse��������
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		std::cout << endl << endl << endl;
	}
	else
	{
		//���������������ֹͣ
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}
	//��2��3���ڸ����ϴξ�icp�㷨��ĵ��ƣ�����ط�ע�⿴��֮ǰ��addpointcloud��ӵ��ƣ�������updatePointCloud�Ǹ���֮ǰ��Ӻõĵ���
	viewer.updatePointCloud(cloud_toberegistration, cloud_icp_color_h, "cloud_icp_v2");
	viewer.updatePointCloud(cloud_icp_toberegistration, cloud_icp_color_h, "cloud_icp_v3");

	//Display the visualiser  
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		// The user pressed "space" :  �����̰��¿ո����next_iteration��Ϊtrue����������ļ��̻ص��������룩��if��������ִ��
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm  
			time.tic();
			icp.align(*cloud_icp_toberegistration);//����һ����׼��ĵ�����Ϊ���룬������һ����׼
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			icp_trans = icp.getFinalTransformation();
			std::cout << "trans matrix:\n " << /*icp_transdisplay**/icp_trans << endl;

			std::cout << endl;
			//ʹ�ô����ı任�Թ��˼�δ���˵�������ƽ��б任
			pcl::transformPointCloud(*cloud_toberegistration, *cloud_toberegistration, icp_trans);

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.  
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				std::cout << endl << endl << endl;
				//������ʾ��Ϣ�����
				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_toberegistration, cloud_icp_color_h, "cloud_icp_v2");
				viewer.updatePointCloud(cloud_icp_toberegistration, cloud_icp_color_h, "cloud_icp_v3");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				system("pause");
				return (-1);
			}
		}
		next_iteration = false;//next_iteration��Ϊfalse�����ٴΰ��¿ո��ʱnext_iteration=true�ٽ�����׼
	}
	system("pause");
	return (0);
}

