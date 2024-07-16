#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>

#pragma pack(push, 8)

typedef pcl::PointCloud<pcl::PointXYZ> PclCloud_Point;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PclCloudPtr_Point;
/**
* @brief 点云预对齐类。
*/
class PCL_InitialAlign
{
public:
	// 消息信息
	enum class eExtraPcl_Message
	{
		eMessageOK = 0,						// 执行成功
		eMessageBadData,					// 点云数据错误或无效
	};

	// 预对齐类型
	enum class eExtraPcl_InitialAlignType
	{
		eFast = 0,							// 快速预对齐
		eRobust								// 鲁棒预对齐
	};

private:
	eExtraPcl_Message m_eMessage; // 记录消息ID

public:
	// 构造/析构函数
	PCL_InitialAlign();

	~PCL_InitialAlign();

#pragma region Routine Parameters

	// 设置/获取 预对齐类型
	void SetInitialAlignType(const eExtraPcl_InitialAlignType& eInitialAlignType);

	eExtraPcl_InitialAlignType GetInitialAlignType() const
	{
		return m_eInitialAlignType;
	}

#pragma endregion

#pragma region Related to execution and results
	//*****************************************************
	// Function:    Execute
	// FullName:    PCL_InitialAlign::Execute
	// Description: 执行点云预对齐算法
	// Parameters:
	//	[out]		   meshIn : 输入STL     
	//	[in]		 pCloudIn : 输入点云                                                                                                                                                          
	// Return value:
	//					 bool : 成功/失败
	// Remarks:
	//*****************************************************
	template <typename PointT>
	bool Execute(const pcl::PolygonMesh& meshIn, const std::shared_ptr<pcl::PointCloud<PointT>>& pCloudIn);

	//*****************************************************
	// Function:    Execute
	// FullName:    PCL_InitialAlign::Execute
	// Description: 获取转换矩阵
	// Parameters:                                                                                                                                                          
	// Return value:
	//		  Eigen::Matrix4d : 转换矩阵
	// Remarks:
	//*****************************************************
	Eigen::Matrix4d GetTransform() const;

#pragma endregion

private:
	// 设置变量
	eExtraPcl_InitialAlignType m_eInitialAlignType;        // 预对齐类型
	Eigen::Matrix4d			   m_transform;				   // 转换矩阵
	int						   m_nThreadNum;			   // 线程数
};

typedef PCL_InitialAlign::eExtraPcl_Message    em3DCloudSampleMessage;
typedef PCL_InitialAlign::eExtraPcl_InitialAlignType em3DCloudInitialAlignType;
#pragma pack(pop)

