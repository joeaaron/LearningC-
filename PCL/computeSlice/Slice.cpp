#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

const int PNT_STRIDE = 12;
const int PNT_MAX = 600000;
const int ROT_MATRIX_SIZE = 3;
const int TRANSLATION_SIZE = 3;

namespace
{
	pcl::PointXYZ EigenToPcl(const Eigen::Vector3d& eigenPoint)
	{
		pcl::PointXYZ pclPoint;
		pclPoint.x = static_cast<float>(eigenPoint.x());
		pclPoint.y = static_cast<float>(eigenPoint.y());
		pclPoint.z = static_cast<float>(eigenPoint.z());
		return pclPoint;
	}

	/*
	 * @brief �Ե������ݽ�����ת��ƽ��
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf �������ݵĽṹ��ָ��
	 */
	void RotateCloud(SliceBuf* pBuf, TransData& transData)
	{
		if (pBuf == nullptr || pBuf->pCloud.empty())
		{
			return;
		}

		long lCloudNum = pBuf->lCloudNum;
		auto& pCloud = pBuf->pCloud;

		// �ڷ���תƽ��
		double sinX = sin(transData.m_rot[0]);  	// �Ƕ�ת����
		double cosX = cos(transData.m_rot[0]);

		double sinY = sin(transData.m_rot[1]);  	// �Ƕ�ת����
		double cosY = cos(transData.m_rot[1]);

		double sinZ = sin(transData.m_rot[2]);		// �Ƕ�ת����
		double cosZ = cos(transData.m_rot[2]);

		// ��X�����ת����
		Eigen::Matrix4d R1, R2, R3;
		R1 << 1, 0, 0, 0,
			0, cosX, -sinX, 0,
			0, sinX, cosX, 0,
			0, 0, 0, 1;

		// ��Y�����ת����
		R2 << cosY, 0, sinY, 0,
			0, 1, 0, 0,
			-sinY, 0, cosY, 0,
			0, 0, 0, 1;

		// ��Z�����ת����
		R3 << cosZ, -sinZ, 0, 0,
			sinZ, cosZ, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		// ���� R4 = R2 * R1, R5 = R3 * R4
		Eigen::Matrix4d R4 = R2 * R1;
		Eigen::Matrix4d R5 = R3 * R4;
	
		// ƽ������
		Eigen::Vector4d T5(0, transData.m_move[0], transData.m_move[1], transData.m_move[2]);

		// ��ת��ƽ�Ƶ���
		for (int k = 1; k < lCloudNum; k++)
		{
			for (int i = 0; i < 4; i++)
			{
				Eigen::Vector4d p;
				for (int j = 1; j < 4; j++)
				{
					// ȡ��������3����
					p[j] = pCloud[k * PNT_STRIDE + i * 3 + j];
				}

				// ������תƽ�ƺ������ p2 = R5 * p1 + T5
				Eigen::Vector4d p1;
				p1.head<3>() = p.tail<3>();
				Eigen::Vector4d transPt = R5 * p1;
				if (i > 0)
				{
					transPt += T5;
				}

				Eigen::Vector4d result;
				result.tail<3>() = transPt.head<3>();

				for (int j = 1; j < 4; j++) {
					pCloud[k * PNT_STRIDE + i * 3 + j] = result[j];
				}
			}
		}

		// �������
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 1; k <= lCloudNum; ++k)
		{
			for (int i = 0; i < 4; i++)
			{
				cloud->points.emplace_back(pCloud[k * PNT_STRIDE + i * 3 + 0], pCloud[k * PNT_STRIDE + i * 3 + 1], pCloud[k * PNT_STRIDE + i * 3 + 2]);
			}
		}
		cloud->width = cloud->points.size();
		cloud->height = 1;
		cloud->is_dense = true;

		/*pcl::PCDWriter writer;
		writer.write("rotate2.pcd", *cloud, false);*/

		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setWindowName(u8"��Ƭ����");

		//viewer->addPointCloud(cloud, "Slice");
		//viewer->setRepresentationToSurfaceForAllActors();
		//viewer->initCameraParameters();
		//viewer->resetCamera();
		//viewer->spin();

		// ��¼��ת�����ƽ������
		transData.m_mtxR = R5;
		transData.m_vecT = T5;
	}


	/*
	 * @brief ������XY��Ľ���
	 *
	 * ���ݸ�����2���߶Σ����������߶Ρ�
	 *
	 * @param lineSeg1		�߶�1
	 * @param lineSeg2		�߶�2
	 * @param dSlicePos	    ��Ƭλ��
	 * @return ��������
	 */
	void CalLineSeg(LineSeg& lineSegOut, const LineSeg& lineSeg1, const LineSeg& lineSeg2, double dSlicePos)
	{
		double dCoef1 = dSlicePos - lineSeg1.m_point.z();
		double dCoef2 = lineSeg2.m_point.z() - lineSeg1.m_point.z();

		if (fabs(dCoef2) < DBL_EPSILON) return;

		lineSegOut.m_point.x() = dCoef1 * (lineSeg2.m_point.x() - lineSeg1.m_point.x()) / dCoef2 + lineSeg1.m_point.x();
		lineSegOut.m_point.y() = dCoef1 * (lineSeg2.m_point.y() - lineSeg1.m_point.y()) / dCoef2 + lineSeg1.m_point.y();
		//lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
		//lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
		//lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;
	}

	/*
	* @brief �ж������߶��Ƿ������ͬ�������˵�
	*
	* @param lineSeg1		�߶�1
	* @param lineSeg1		�߶�2
	* @param slcLineSeg0	��Ƭ�߶�1
	* @param slcLineSeg1	��Ƭ�߶�2
	*/
	bool IsSegmentEqual(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
		const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
	{
		return (lineSeg1.m_point.x() == slcLineSeg0.m_point.x() && lineSeg2.m_point.x() == slcLineSeg1.m_point.x() && lineSeg1.m_point.y() == slcLineSeg0.m_point.y() && lineSeg2.m_point.y() == slcLineSeg1.m_point.y())
			|| (lineSeg2.m_point.x() == slcLineSeg0.m_point.x() && lineSeg1.m_point.x() == slcLineSeg1.m_point.x() && lineSeg2.m_point.y() == slcLineSeg0.m_point.y() && lineSeg1.m_point.y() == slcLineSeg1.m_point.y());
	}

	/*
	 * @brief �Ե������ݽ�����ת��ƽ��
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf			�������ݵĽṹ��ָ��
	 * @param dSlicePos	    ��Ƭλ��
	 */
	void ComputeSlc(SliceBuf* pBuf, double dSlicePos)
	{
		if (pBuf == nullptr || pBuf->pCloud.empty())
		{
			return;
		}

		auto pCloud = pBuf->pCloud;
		long lCloudNum = pBuf->lCloudNum;
		int nCnt = 0;

		LineSeg lineSeg1, lineSeg2, lineSeg3;

		for (int k = 1; k <= lCloudNum && nCnt + 1 < PNT_MAX; k++)
		{
			// ��ȡ��ǰ����������������
			double xn = pCloud[k * PNT_STRIDE + 1];
			double yn = pCloud[k * PNT_STRIDE + 2];
			double zn = pCloud[k * PNT_STRIDE + 3];

			lineSeg1.m_point.x() = pCloud[k * PNT_STRIDE + 4]; lineSeg1.m_point.y()= pCloud[k * PNT_STRIDE + 5]; lineSeg1.m_point.z() = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.m_point.x() = pCloud[k * PNT_STRIDE + 7]; lineSeg2.m_point.y()= pCloud[k * PNT_STRIDE + 8]; lineSeg2.m_point.z() = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.m_point.x() = pCloud[k * PNT_STRIDE + 10]; lineSeg3.m_point.y() = pCloud[k * PNT_STRIDE + 11]; lineSeg3.m_point.z() = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			//����λ�����ٽ��ĵ�����
			if (lineSeg1.m_point.z() == dSlicePos && lineSeg2.m_point.z() == dSlicePos && lineSeg3.m_point.z() == dSlicePos) { continue; }
			bool flagRelated = false;
			int dt = 0;

			// ����������ڵ㶼��dSlicePosƽ���ͬ�ࣨ�������ڻ�С��dSlicePos�������һ������Ƿ���һ������������dSlicePosƽ���ϡ�
			// ����ǣ�����Щ�߶δ洢��pBuf->slc�С�
			if ((lineSeg1.m_point.z() >= dSlicePos && lineSeg2.m_point.z() >= dSlicePos && lineSeg3.m_point.z() >= dSlicePos)
				|| (lineSeg1.m_point.z() <= dSlicePos && lineSeg2.m_point.z() <= dSlicePos && lineSeg3.m_point.z() <= dSlicePos))
			{
				if (lineSeg1.m_point.z() == dSlicePos && lineSeg2.m_point.z() == dSlicePos && lineSeg3.m_point.z() > dSlicePos)
				{
					nCnt++;  dt = 0;   flagRelated = true;
					pBuf->slc[nCnt].lineSeg0 = lineSeg1;
					dt++;
					pBuf->slc[nCnt].lineSeg1 = lineSeg2;
					dt++;

				}
				if (lineSeg1.m_point.z() == dSlicePos && lineSeg3.m_point.z() == dSlicePos && lineSeg2.m_point.z() > dSlicePos)
				{
					nCnt++;  dt = 0;   flagRelated = true;
					pBuf->slc[nCnt].lineSeg0 = lineSeg1;
					dt++;
					pBuf->slc[nCnt].lineSeg1 = lineSeg3;
					dt++;
				}
				if (lineSeg3.m_point.z() == dSlicePos && lineSeg2.m_point.z() == dSlicePos && lineSeg1.m_point.z() > dSlicePos)
				{
					nCnt++;  dt = 0;   flagRelated = true;
					pBuf->slc[nCnt].lineSeg0 = lineSeg3;
					dt++;
					pBuf->slc[nCnt].lineSeg1 = lineSeg2;
					dt++;
				}
				pBuf->slc[nCnt].nLineA = 2;
			}
			// ����������ڵ㲻��dSlicePosƽ���ͬ�࣬�������dSlicePosƽ���ཻ���߶Σ�������洢��pBuf->slc�С�
			else
			{
				//�Ե���бȶԺ��ɾ��
				nCnt++;  dt = 0;   flagRelated = true;
				LineSegment lineSegOut;

				if ((lineSeg1.m_point.z() - dSlicePos) * (lineSeg2.m_point.z() - dSlicePos) < 0)
				{
					CalLineSeg(lineSegOut, lineSeg1, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg1.m_point.z() - dSlicePos) * (lineSeg3.m_point.z() - dSlicePos) < 0)
				{
					CalLineSeg(lineSegOut, lineSeg1, lineSeg3, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg3.m_point.z() - dSlicePos) * (lineSeg2.m_point.z() - dSlicePos) < 0)
				{
					CalLineSeg(lineSegOut, lineSeg3, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if (lineSeg1.m_point.z() == dSlicePos && (lineSeg3.m_point.z() - dSlicePos) * (lineSeg2.m_point.z() - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg1; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg1; }

					dt++;
				}
				if (lineSeg2.m_point.z() == dSlicePos && (lineSeg1.m_point.z() - dSlicePos) * (lineSeg3.m_point.z() - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg2; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg2; }

					dt++;
				}
				if (lineSeg3.m_point.z() == dSlicePos && (lineSeg1.m_point.z() - dSlicePos) * (lineSeg2.m_point.z() - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg3; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg3; }

					dt++;
				}

				pBuf->slc[nCnt].nLineA = 1;
			}

			//������Ƭ��λ�ã�����ÿ����dSlicePosƽ���ཻ���߶Σ�ȷ��������Ƭƽ���ϵķ��򣨻��ڵ�ǰ����߶εķ��򣩡�
			if (flagRelated)
			{
				lineSeg1 = pBuf->slc[nCnt].lineSeg0;
				lineSeg2 = pBuf->slc[nCnt].lineSeg1;
				lineSeg3.m_point.x() = lineSeg2.m_point.x() - lineSeg1.m_point.x();
				lineSeg3.m_point.y() = lineSeg2.m_point.y() - lineSeg1.m_point.y();

				if (xn * lineSeg3.m_point.y() - yn * lineSeg3.m_point.x() < 0)
				{
					pBuf->slc[nCnt].nLineB = -1;
				}
				else
				{
					pBuf->slc[nCnt].nLineB = 1;
				}
			}
		}

		//ɾ��buf.line_m[cnt]=2;�ظ��߶Σ������ϲ࣬�����߶�ɾ����
		//���ߵ�������������ƽ��ģ�1���£�2ͬ�࣬3һ��һƽ��4��ƽ    
		//����ѡz>z9�������� 
		for (int k = 1; k <= nCnt - 1; k++)
		{
			if (pBuf->slc[k].nLineA == 2)
			{
				bool flagSameSide = false;
				lineSeg1 = pBuf->slc[k].lineSeg0;
				lineSeg2 = pBuf->slc[k].lineSeg1;

				for (int i = k + 1; i <= nCnt; i++)
				{
					if (IsSegmentEqual(lineSeg1, lineSeg2, pBuf->slc[i].lineSeg0, pBuf->slc[i].lineSeg1))
					{
						flagSameSide = true;

						for (int j = i; j <= nCnt - 1; j++)
						{
							pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
						}
						--nCnt; // ���ټ���
					}
				}
				//ͬ��������߶ζ�ɾ��
				if (flagSameSide)
				{
					for (int j = k; j <= nCnt - 1; j++)
					{
						pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
					}
					--nCnt;
				}
			}
		}

		// ����pbuf->cnt�Է�ӳ�洢��pbuf->slc�е��߶�����
		pBuf->nCnt = nCnt;
	}

	/*
	 * @brief ��ת��Ƭ����
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf			��Ƭ�������ݵĽṹ��ָ��
	 * @param dSlicePos	    ��Ƭλ��
	 */
	void RotateSlc(SliceBuf* pbuf, double dSlicePos, const TransData& transData)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// ��ת����
		for (int i = 0; i < ROT_MATRIX_SIZE; i++)
		{
			for (int j = 0; j < ROT_MATRIX_SIZE; j++)
			{
				R(i, j) = transData.m_mtxR(i, j);
			}
			T(i) = transData.m_vecT(i);
		}

		Eigen::Matrix3d R_inv = R.transpose();
		Eigen::Vector3d T_inv = -R_inv*T;

		// �����������Ը����ĵ������ת��ƽ��
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R_inv * (p - T_inv) + T_inv;
			};

		// ����Ƭ�˵�0��1������ת��ƽ��
		for (int k = 0; k < pbuf->nCnt; k++)
		{
			rotateAndTranslate(pbuf->slc[k].lineSeg0.m_point.x(), pbuf->slc[k].lineSeg0.m_point.y(), pbuf->slc[k].vec0);
			rotateAndTranslate(pbuf->slc[k].lineSeg1.m_point.x(), pbuf->slc[k].lineSeg1.m_point.y(), pbuf->slc[k].vec1);
		}
	}

}

PCL_Slice::PCL_Slice()
	: m_eMessage(eExtraPcl_Message::eMessageOK)
	, m_dSlicePos(6.0)
	, m_dInterval(0.03)
	, m_transData(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0))
{
	m_nThreadNum = static_cast<int>(omp_get_num_procs() * 0.75);
}

PCL_Slice::~PCL_Slice()
{
}

void PCL_Slice::SetSlicePos(double dSlicePos)
{
	m_dSlicePos = dSlicePos;
}

void PCL_Slice::SetTransData(TransData transData)
{
	m_transData = transData;
}

bool PCL_Slice::Execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudOut, SliceBuf* pBuf)
{
	if (pBuf == nullptr || pBuf->pCloud.empty())
	{
		m_eMessage = em3DCloudSliceMessage::eMessageBadData;
		return false;
	}

	// ��ת����
 	RotateCloud(pBuf, m_transData);

	// ������Ƭ
	ComputeSlc(pBuf, m_dSlicePos);

	// ��ת��Ƭ����
	RotateSlc(pBuf, m_dSlicePos, m_transData);

	// �������
	pCloudOut = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (int i = 0; i <= pBuf->nCnt; ++i)
	{
		//�������Ƿ��㹻С
		double distance = (pBuf->slc[i].vec0 - pBuf->slc[i].vec1).norm();
		if (distance > m_dInterval)
		{
			int nNum = distance / m_dInterval;
			for (int j = 0; j <= nNum; ++j)
			{
				Eigen::Vector3d vec = pBuf->slc[i].vec0 + (pBuf->slc[i].vec1 - pBuf->slc[i].vec0) * j / (nNum + 1.0);
				pcl::PointXYZ pt = EigenToPcl(vec);
				pCloudOut->emplace_back(pt);
			}
		}
		pcl::PointXYZ pt0 = EigenToPcl(pBuf->slc[i].vec0);
		pcl::PointXYZ pt1 = EigenToPcl(pBuf->slc[i].vec1);
		pCloudOut->emplace_back(pt0);
		pCloudOut->emplace_back(pt1);
	}
	pCloudOut->width = pCloudOut->points.size();
	pCloudOut->height = 1;
	pCloudOut->is_dense = true;

	return pCloudOut != nullptr;
}

