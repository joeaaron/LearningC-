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

	Eigen::Vector3d m_point;					// �߶ζ˵������
	Eigen::Vector3d m_color;					// �߶ζ˵����ɫ
}LineSeg;

typedef struct SliceCloudBuf
{
	struct
	{
		LineSegment lineSeg0;
		LineSegment lineSeg1;

		Eigen::Vector3d vec0;			// ����˵�0
		Eigen::Vector3d vec1;			// ����˵�1

		int nLineA, nLineB;
	}slc[PCL_SLICE_POINT_NUM];			// ��Ƭ����

	std::vector<float> pCloud;			// ��ŵ���
	long   lCloudNum;					// ���Ƽ���
	int	   nCnt;						// ��Ƭ���Ƽ���
}SliceBuf;

typedef struct TransformData
{
	TransformData(Eigen::Vector3d rot, Eigen::Vector3d move)
		: m_rot(rot)
		, m_move(move)
	{
		// ��ʼ����ת�����λ������
		m_mtxR = Eigen::Matrix4d::Identity(); // ����Ϊ��λ����
		m_vecT.head<3>() = m_move;
	}

	Eigen::Vector3d m_rot;
	Eigen::Vector3d m_move;

	Eigen::Matrix4d m_mtxR;
	Eigen::Vector4d m_vecT;
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

	// ����/��ȡ ��ת
	void SetTransData(TransData transData);

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
	int					  m_nThreadNum;				 // �߳���
	TransData			  m_transData;
};

typedef PCL_Slice::eExtraPcl_Message    em3DCloudSliceMessage;

#pragma pack(pop)

