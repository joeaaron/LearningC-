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

	double x = 0, y = 0, z = 0;					// �߶ζ˵������
	double r = 0, g = 0, b = 0;					// �߶ζ˵����ɫ
}LineSeg;

typedef struct SliceCloudBuf
{
	struct
	{
		LineSegment lineSeg0;			// �߶ζ˵�0
		LineSegment lineSeg1;			// �߶ζ˵�1

		Eigen::Vector3d vec0;			// ����˵�0
		Eigen::Vector3d vec1;			// ����˵�1

		int nLineA, nLineB;
	}slc[PCL_SLICE_POINT_NUM];			// ��Ƭ����

	std::vector<float> vCloud;			// ��ŵ���
	long   lCloudNum;					// ���Ƽ���
	int	   nCnt;						// ��Ƭ���Ƽ���
}SliceBuf;

typedef struct TransformData
{
	double rotX, rotY, rotZ;
	double moveX, moveY, moveZ;

	Eigen::Matrix4d mtxR;
	Eigen::Vector4d vecT;
}TransData;

/**
* @brief ���ƶ����ࡣ
*/
class PCL_Slice
{
public:
	// ��Ϣ��Ϣ
	enum class eExtraPcl_Message
	{
		eMessageOK = 0,						// ִ�гɹ�
		eMessageBadData,					// �������ݴ������Ч
	};

	// ��׼�᷽��
	enum class eExtraPcl_AxisDir
	{
		eAxixX = 0,							// X��
		eAxixY,								// Y��
		eAxixZ								// Z��
	};

private:
	eExtraPcl_Message m_eMessage; // ��¼��ϢID

public:
	// ����/��������
	PCL_Slice();

	~PCL_Slice();

#pragma region Routine Parameters

	// ����/��ȡ ��Ƭλ��
	void SetSlicePos(double dSlicePos);

	double GetSlicePos() const
	{
		return m_dSlicePos;
	}

	// ����/��ȡ ��׼�᷽��
	void SetAxisDir(eExtraPcl_AxisDir eAxisDir);

	eExtraPcl_AxisDir GetAxisDir() const
	{
		return m_eAxisDir;
	}
#pragma endregion

#pragma region Related to execution and results
	//*****************************************************
	// Function:    Execute
	// FullName:    PCL_Sample::Execute
	// Description: ִ�е�����Ƭ�㷨
	// Parameters:
	//	[out]		pCloudOut : �������
	//	[in]		     pbuf : �������                                                                                                                                                          
	// Return value:
	//					 bool : �ɹ�/ʧ��
	// Remarks:
	//*****************************************************
	bool Execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudOut, SliceBuf* pBuf);

#pragma endregion

private:
	// ���ñ���
	double				  m_dSlicePos;				 // ��Ƭλ��
	double				  m_dInterval;				 // ����
	eExtraPcl_AxisDir	  m_eAxisDir;				 // ��׼�᷽��
	int					  m_nThreadNum;				 // �߳���
};

typedef PCL_Slice::eExtraPcl_Message    em3DCloudSliceMessage;
typedef PCL_Slice::eExtraPcl_AxisDir    em3DCloudSliceAxisDir;

#pragma pack(pop)

