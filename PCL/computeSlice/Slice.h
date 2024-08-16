#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#pragma pack(push, 8)

const int PCL_SLICE_POINT_NUM = 60000;

typedef struct LineSegment
{
	LineSegment() {}
	LineSegment(double dx, double dy, double dz, double dr, double dg, double db)
	{
		x = dx;
		y = dy;
		z = dz;
		r = dr;
		g = dg;
		b = db;
	}

	LineSegment& operator=(const LineSegment& rhs)
	{
		if (this == &rhs) return *this;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		r = rhs.r;
		g = rhs.g;
		b = rhs.b;

		return *this;
	}

	double x = 0, y = 0, z = 0;					// 线段端点的坐标
	double r = 0, g = 0, b = 0;					// 线段端点的颜色
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
	double rotX = 0.0, rotY = 0.0, rotZ = 0.0;
	double moveX = 0.0, moveY = 0.0, moveZ = 0.0;

	Eigen::Matrix4d mtxR;
	Eigen::Vector4d vecT;

	// 构造函数
	TransformData(double rX, double rY, double rZ, double mX, double mY, double mZ)
		: rotX(rX), rotY(rY), rotZ(rZ), moveX(mX), moveY(mY), moveZ(mZ)
	{
		// 初始化旋转矩阵和位移向量
		mtxR = Eigen::Matrix4d::Identity(); // 设置为单位矩阵
		vecT << moveX, moveY, moveZ, 1.0;
	}
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

