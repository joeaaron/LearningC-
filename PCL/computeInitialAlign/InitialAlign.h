#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>

#pragma pack(push, 8)

typedef pcl::PointCloud<pcl::PointXYZ> PclCloud_Point;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PclCloudPtr_Point;
/**
* @brief ����Ԥ�����ࡣ
*/
class PCL_InitialAlign
{
public:
	// ��Ϣ��Ϣ
	enum class eExtraPcl_Message
	{
		eMessageOK = 0,						// ִ�гɹ�
		eMessageBadData,					// �������ݴ������Ч
	};

	// Ԥ��������
	enum class eExtraPcl_InitialAlignType
	{
		eFast = 0,							// ����Ԥ����
		eRobust								// ³��Ԥ����
	};

private:
	eExtraPcl_Message m_eMessage; // ��¼��ϢID

public:
	// ����/��������
	PCL_InitialAlign();

	~PCL_InitialAlign();

#pragma region Routine Parameters

	// ����/��ȡ Ԥ��������
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
	// Description: ִ�е���Ԥ�����㷨
	// Parameters:
	//	[out]		   meshIn : ����STL     
	//	[in]		 pCloudIn : �������                                                                                                                                                          
	// Return value:
	//					 bool : �ɹ�/ʧ��
	// Remarks:
	//*****************************************************
	template <typename PointT>
	bool Execute(const pcl::PolygonMesh& meshIn, const std::shared_ptr<pcl::PointCloud<PointT>>& pCloudIn);

	//*****************************************************
	// Function:    Execute
	// FullName:    PCL_InitialAlign::Execute
	// Description: ��ȡת������
	// Parameters:                                                                                                                                                          
	// Return value:
	//		  Eigen::Matrix4d : ת������
	// Remarks:
	//*****************************************************
	Eigen::Matrix4d GetTransform() const;

#pragma endregion

private:
	// ���ñ���
	eExtraPcl_InitialAlignType m_eInitialAlignType;        // Ԥ��������
	Eigen::Matrix4d			   m_transform;				   // ת������
	int						   m_nThreadNum;			   // �߳���
};

typedef PCL_InitialAlign::eExtraPcl_Message    em3DCloudSampleMessage;
typedef PCL_InitialAlign::eExtraPcl_InitialAlignType em3DCloudInitialAlignType;
#pragma pack(pop)

