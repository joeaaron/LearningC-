#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#pragma pack(push, 8)

const int PCL_SLICE_POINT_NUM = 60000;

typedef struct LineSegment
{
	LineSegment() {}
	LineSegment(Eigen::Vector3d point, Eigen::Vector3d color)
	{
		m_point = point;
		m_color = color;
	}

	LineSegment& operator=(const LineSegment& rhs)
	{
		if (this == &rhs) return *this;
		m_point = rhs.m_point;
		m_color = rhs.m_color;

		return *this;
	}

	Eigen::Vector3d m_point;					// 线段端点的坐标
	Eigen::Vector3d m_color;					// 线段端点的颜色
}LineSeg;

typedef struct SliceCloudBuf
{
	struct
	{
		LineSegment lineSeg0;
		LineSegment lineSeg1;

		Eigen::Vector3d vec0;			// 切面端点0
		Eigen::Vector3d vec1;			// 切面端点1

		int nLineA, nLineB;
	}slc[PCL_SLICE_POINT_NUM];			// 切片点云

	std::vector<float> pCloud;			// 存放点云
	long   lCloudNum;					// 点云计数
	int	   nCnt;						// 切片点云计数
}SliceBuf;

typedef struct TransformData
{
	TransformData(Eigen::Vector3d rot, Eigen::Vector3d move)
		: m_rot(rot)
		, m_move(move)
	{
		// 初始化旋转矩阵和位移向量
		m_mtxR = Eigen::Matrix4d::Identity(); // 设置为单位矩阵
		m_vecT.head<3>() = m_move;
	}

	Eigen::Vector3d m_rot;
	Eigen::Vector3d m_move;

	Eigen::Matrix4d m_mtxR;
	Eigen::Vector4d m_vecT;
}TransData;

/**
* @brief 点云断面类。
*/
class PCL_Slice
{
public:
	// 消息信息
	enum class eExtraPcl_Message
	{
		eMessageOK = 0,						// 执行成功
		eMessageBadData,					// 点云数据错误或无效
	};

private:
	eExtraPcl_Message m_eMessage; // 记录消息ID

public:
	// 构造/析构函数
	PCL_Slice();

	~PCL_Slice();

#pragma region Routine Parameters

	// 设置/获取 切片位置
	void SetSlicePos(double dSlicePos);

	double GetSlicePos() const
	{
		return m_dSlicePos;
	}

	// 设置/获取 旋转
	void SetTransData(TransData transData);

#pragma endregion

#pragma region Related to execution and results
	//*****************************************************
	// Function:    Execute
	// FullName:    PCL_Sample::Execute
	// Description: 执行点云切片算法
	// Parameters:
	//	[out]		pCloudOut : 输出点云
	//	[in]		     pbuf : 输入点云                                                                                                                                                          
	// Return value:
	//					 bool : 成功/失败
	// Remarks:
	//*****************************************************
	bool Execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudOut, SliceBuf* pBuf);

#pragma endregion

private:
	// 设置变量
	double				  m_dSlicePos;				 // 切片位置
	double				  m_dInterval;				 // 点间距
	int					  m_nThreadNum;				 // 线程数
	TransData			  m_transData;
};

typedef PCL_Slice::eExtraPcl_Message    em3DCloudSliceMessage;

#pragma pack(pop)

