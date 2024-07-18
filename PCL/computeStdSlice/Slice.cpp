#include "Slice.h"

const int PNT_STRIDE = 12;
const int PNT_MAX = 600000;
const int ROT_MATRIX_SIZE = 3;
const int TRANSLATION_SIZE = 3;
const int AXIS_X = 0;
const int AXIS_Y = 1;
const int AXIS_Z = 2;

TransData g_transData;

namespace
{
	/*
	 * @brief �Ե������ݽ�����ת��ƽ��
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf �������ݵĽṹ��ָ��
	 */
	void RotateCloud(SliceBuf* pBuf)
	{
		if (pBuf == nullptr || pBuf->vCloud.empty())
		{
			return;
		}

		long lCloudNum = pBuf->lCloudNum;
		auto pCloud = pBuf->vCloud;

		// �ڷ���תƽ��
		double sinX = sin(g_transData.rotX * M_PI / 180);
		double cosX = cos(g_transData.rotX * M_PI / 180);

		// ������ת���� R1, R2, R3
		Eigen::Matrix3d R1, R2, R3;
		R1 << 1, 0, 0,
			0, cosX, -sinX,
			0, sinX, cosX;

		R2 << cosX, 0, sinX,
			0, 1, 0,
			-sinX, 0, cosX;

		R3 << cosX, -sinX, 0,
			sinX, cosX, 0,
			0, 0, 1;

		// ���� R4 = R2 * R1, R5 = R3 * R4
		Eigen::Matrix3d R4 = R2 * R1;
		Eigen::Matrix3d R5 = R3 * R4;

		// ƽ������
		Eigen::Vector3d T5(g_transData.moveX, g_transData.moveY, g_transData.moveZ);

		// ��ת��ƽ�Ƶ���
		for (int k = 0; k < lCloudNum; k++)
		{
			for (int i = 0; i < 4; i++)
			{
				Eigen::Vector3d p;
				for (int j = 0; j < 3; j++)
				{
					p[j] = pCloud[k * PNT_STRIDE + i * 3 + j];
				}

				// ������תƽ�ƺ������ p2 = R5 * p1 + T5
				Eigen::Vector3d transPt = R5 * p;
				if (i > 0)
				{
					transPt += T5;
				}

				for (int j = 0; j < 3; j++) {
					pCloud[k * PNT_STRIDE + i * 3 + j] = transPt[j];
				}
			}
		}

		// ��¼��ת�����ƽ������
		Eigen::Matrix4d mat;
		mat << 0, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		g_transData.mtxR = mat;

		Eigen::Vector4d vec;
		vec.head<3>() = T5;
		vec[3] = 0;
		g_transData.vecT = vec;
	}

	/*
	 * @brief �����߶εĶ˵��������ɫ
	 *
	 * ���ݸ�����2���߶Σ����������߶Ρ�
	 *
	 * @param lineSeg1		�߶�1
	 * @param lineSeg2		�߶�2
	 * @param dSlicePos	    ��Ƭλ��
	 * @param nDir			��׼�᷽��
	 * @return ��������
	 */
	void CalLineSeg(LineSeg& lineSegOut, const LineSeg& lineSeg1, const LineSeg& lineSeg2, double dSlicePos, int nDir)
	{
		double dCoef1 = 0.0;
		double dCoef2 = 0.0;

		if (AXIS_Z == nDir)
		{
			dCoef1 = dSlicePos - lineSeg1.z;
			dCoef2 = lineSeg2.z - lineSeg1.z;

			if (fabs(dCoef2) < DBL_EPSILON) return;

			lineSegOut.x = dCoef1 * (lineSeg2.x - lineSeg1.x) / dCoef2 + lineSeg1.x;
			lineSegOut.y = dCoef1 * (lineSeg2.y - lineSeg1.y) / dCoef2 + lineSeg1.y;
		}
		else if (AXIS_X == nDir)
		{
			dCoef1 = dSlicePos - lineSeg1.x;
			dCoef2 = lineSeg2.x - lineSeg1.x;

			if (fabs(dCoef2) < DBL_EPSILON) return;

			lineSegOut.z = dCoef1 * (lineSeg2.z - lineSeg1.z) / dCoef2 + lineSeg1.z;
			lineSegOut.y = dCoef1 * (lineSeg2.y - lineSeg1.y) / dCoef2 + lineSeg1.y;
		}
		else
		{
			dCoef1 = dSlicePos - lineSeg1.y;
			dCoef2 = lineSeg2.y - lineSeg1.y;

			if (fabs(dCoef2) < DBL_EPSILON) return;

			lineSegOut.z = dCoef1 * (lineSeg2.z - lineSeg1.z) / dCoef2 + lineSeg1.z;
			lineSegOut.x = dCoef1 * (lineSeg2.x - lineSeg1.x) / dCoef2 + lineSeg1.x;

		}

		lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
		lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
		lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;
	}


	/*
	* @brief �ж������߶��Ƿ������ͬ�������˵�
	*
	* @param lineSeg1		�߶�1
	* @param lineSeg1		�߶�2
	* @param slcLineSeg0	��Ƭ�߶�1
	* @param slcLineSeg1	��Ƭ�߶�2
	* @param nDir			��׼�᷽��
	*/
	bool IsSegmentEqual(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
		const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1, int nDir)
	{
		if (AXIS_Z == nDir)
		{
			return (lineSeg1.x == slcLineSeg0.x && lineSeg2.x == slcLineSeg1.x && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
				|| (lineSeg2.x == slcLineSeg0.x && lineSeg1.x == slcLineSeg1.x && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
		}
		else if (AXIS_X == nDir)
		{
			return (lineSeg1.z == slcLineSeg0.z && lineSeg2.z == slcLineSeg1.z && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
				|| (lineSeg2.z == slcLineSeg0.z && lineSeg1.z == slcLineSeg1.z && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
		}
		else
		{
			return (lineSeg1.x == slcLineSeg0.x && lineSeg2.x == slcLineSeg1.x && lineSeg1.z == slcLineSeg0.z && lineSeg2.z == slcLineSeg1.z)
				|| (lineSeg2.x == slcLineSeg0.x && lineSeg1.x == slcLineSeg1.x && lineSeg2.z == slcLineSeg0.z && lineSeg1.z == slcLineSeg1.z);
		}
	}

	/*
	 * @brief ��ת��Ƭ����
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf			��Ƭ�������ݵĽṹ��ָ��
	 * @param dSlicePos	    ��Ƭλ��
	 * @param nDir			��׼�᷽��
	 */
	void RotateSlc(SliceBuf* pbuf, double dSlicePos, int nDir)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// ��ת����
		for (int i = 0; i < ROT_MATRIX_SIZE; i++)
		{
			for (int j = 0; j < ROT_MATRIX_SIZE; j++)
			{
				R(i, j) = g_transData.mtxR(i + 1, j + 1);
			}
			T(i) = g_transData.vecT(i + 1);
		}

		// �����������Ը����ĵ������ת��ƽ��
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R * (p - T) + T;
			};

		// ����Ƭ�˵�0��1������ת��ƽ��
		for (int k = 0; k < pbuf->nCnt; k++)
		{
			if (AXIS_X == nDir)
			{
				rotateAndTranslate(pbuf->slc[k].lineSeg0.z, pbuf->slc[k].lineSeg0.y, pbuf->slc[k].vec0);
				rotateAndTranslate(pbuf->slc[k].lineSeg1.z, pbuf->slc[k].lineSeg1.y, pbuf->slc[k].vec1);
			}
			else if (AXIS_Y == nDir)
			{
				rotateAndTranslate(pbuf->slc[k].lineSeg0.x, pbuf->slc[k].lineSeg0.z, pbuf->slc[k].vec0);
				rotateAndTranslate(pbuf->slc[k].lineSeg1.x, pbuf->slc[k].lineSeg1.z, pbuf->slc[k].vec1);
			}
			else
			{
				rotateAndTranslate(pbuf->slc[k].lineSeg0.x, pbuf->slc[k].lineSeg0.y, pbuf->slc[k].vec0);
				rotateAndTranslate(pbuf->slc[k].lineSeg1.x, pbuf->slc[k].lineSeg1.y, pbuf->slc[k].vec1);
			}

		}
	}

	bool GetSlc(SliceBuf* pBuf, int& nCnt, bool& flagRelated, const LineSeg& lineSeg1, const LineSeg& lineSeg2, const LineSeg& lineSeg3, double dSlicePos, int nDir)
	{
		int dt = 0;
		double dPos1 = 0.0, dPos2 = 0.0, dPos3 = 0.0;

		if (AXIS_Z == nDir)
		{
			dPos1 = lineSeg1.z;
			dPos2 = lineSeg2.z;
			dPos3 = lineSeg3.z;
		}
		else if (AXIS_X == nDir)
		{
			dPos1 = lineSeg1.x;
			dPos2 = lineSeg2.x;
			dPos3 = lineSeg3.x;
		}
		else
		{
			dPos1 = lineSeg1.y;
			dPos2 = lineSeg2.y;
			dPos3 = lineSeg3.y;
		}
		//����λ�����ٽ��ĵ�����
		if (dPos1 == dSlicePos && dPos2 == dSlicePos && dPos3 == dSlicePos) { return false; }

		// ����������ڵ㶼��dSlicePosƽ���ͬ�ࣨ�������ڻ�С��dSlicePos�������һ������Ƿ���һ������������dSlicePosƽ���ϡ�
		// ����ǣ�����Щ�߶δ洢��pBuf->slc�С�
		if ((dPos1 >= dSlicePos && dPos2 >= dSlicePos && dPos3 >= dSlicePos)
			|| (dPos1 <= dSlicePos && dPos2 <= dSlicePos && dPos3 <= dSlicePos))
		{
			if (dPos1 == dSlicePos && dPos2 == dSlicePos && dPos3 > dSlicePos)
			{
				nCnt++;  dt = 0;   flagRelated = true;
				pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				dt++;
				pBuf->slc[nCnt].lineSeg1 = lineSeg2;
				dt++;

			}
			if (dPos1 == dSlicePos && dPos3 == dSlicePos && dPos2 > dSlicePos)
			{
				nCnt++;  dt = 0;   flagRelated = true;
				pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				dt++;
				pBuf->slc[nCnt].lineSeg1 = lineSeg3;
				dt++;
			}
			if (dPos3 == dSlicePos && dPos2 == dSlicePos && dPos1 > dSlicePos)
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

			if ((dPos1 - dSlicePos) * (dPos2 - dSlicePos) < 0)
			{
				CalLineSeg(lineSegOut, lineSeg1, lineSeg2, dSlicePos, nDir);

				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if ((dPos1 - dSlicePos) * (dPos3 - dSlicePos) < 0)
			{
				CalLineSeg(lineSegOut, lineSeg1, lineSeg3, dSlicePos, nDir);

				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if ((dPos3 - dSlicePos) * (dPos2 - dSlicePos) < 0)
			{
				CalLineSeg(lineSegOut, lineSeg3, lineSeg2, dSlicePos, nDir);

				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if (dPos1 == dSlicePos && (dPos3 - dSlicePos) * (dPos2 - dSlicePos) < 0)
			{
				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg1; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg1; }

				dt++;
			}
			if (dPos2 == dSlicePos && (dPos1 - dSlicePos) * (dPos3 - dSlicePos) < 0)
			{
				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg2; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg2; }

				dt++;
			}
			if (dPos3 == dSlicePos && (dPos1 - dSlicePos) * (dPos2 - dSlicePos) < 0)
			{
				if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg3; }
				if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg3; }

				dt++;
			}

			pBuf->slc[nCnt].nLineA = 1;
		}

		return true;
	}
	/*
	 * @brief �Ե������ݽ�����ת��ƽ��
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf			�������ݵĽṹ��ָ��
	 * @param dSlicePos	    ��Ƭλ��
	 * @param nDir			��׼�᷽��
	 */
	void ComputeSlc(SliceBuf* pBuf, double dSlicePos, int nDir)
	{
		if (pBuf == nullptr || pBuf->vCloud.empty())
		{
			return;
		}

		auto pCloud = pBuf->vCloud;
		long lCloudNum = pBuf->lCloudNum;
		int nCnt = 0;

		LineSeg lineSeg1, lineSeg2, lineSeg3;

		for (int k = 1; k <= lCloudNum && nCnt + 1 < PNT_MAX; k++)
		{
			// ��ȡ��ǰ����������������
			double xn = pCloud[k * PNT_STRIDE + 1];
			double yn = pCloud[k * PNT_STRIDE + 2];
			double zn = pCloud[k * PNT_STRIDE + 3];

			lineSeg1.x = pCloud[k * PNT_STRIDE + 4]; lineSeg1.y = pCloud[k * PNT_STRIDE + 5]; lineSeg1.z = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.x = pCloud[k * PNT_STRIDE + 7]; lineSeg2.y = pCloud[k * PNT_STRIDE + 8]; lineSeg2.z = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.x = pCloud[k * PNT_STRIDE + 10]; lineSeg3.y = pCloud[k * PNT_STRIDE + 11]; lineSeg3.z = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			bool flagRelated = false;

			if (!GetSlc(pBuf, nCnt, flagRelated, lineSeg1, lineSeg2, lineSeg3, dSlicePos, nDir)) continue;


			//������Ƭ��λ�ã�����ÿ����dSlicePosƽ���ཻ���߶Σ�ȷ��������Ƭƽ���ϵķ��򣨻��ڵ�ǰ����߶εķ��򣩡�
			if (flagRelated)
			{
				lineSeg1 = pBuf->slc[nCnt].lineSeg0;
				lineSeg2 = pBuf->slc[nCnt].lineSeg1;
				lineSeg3.x = lineSeg2.x - lineSeg1.x;
				lineSeg3.y = lineSeg2.y - lineSeg1.y;

				if (xn * lineSeg3.y - yn * lineSeg3.x < 0)
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
					if (IsSegmentEqual(lineSeg1, lineSeg2, pBuf->slc[i].lineSeg0, pBuf->slc[i].lineSeg1, nDir))
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

		// ��ת��Ƭ����
		RotateSlc(pBuf, dSlicePos, nDir);
	}

	pcl::PointXYZ EigenToPcl(const Eigen::Vector3d& eigenPoint)
	{
		pcl::PointXYZ pclPoint;
		pclPoint.x = static_cast<float>(eigenPoint.x());
		pclPoint.y = static_cast<float>(eigenPoint.y());
		pclPoint.z = static_cast<float>(eigenPoint.z());
		return pclPoint;
	}
}

PCL_Slice::PCL_Slice()
	: m_eMessage(eExtraPcl_Message::eMessageOK)
	, m_dSlicePos(6.0)
	, m_dInterval(0.03)
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

void PCL_Slice::SetAxisDir(eExtraPcl_AxisDir eAxisDir)
{
	m_eAxisDir = eAxisDir;
}

bool PCL_Slice::Execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudOut, SliceBuf* pBuf)
{
	if (pBuf == nullptr || pBuf->vCloud.empty())
	{
		m_eMessage = em3DCloudSliceMessage::eMessageBadData;
		return false;
	}

	// ��ת����
	RotateCloud(pBuf);

	// ������Ƭ
	ComputeSlc(pBuf, m_dSlicePos, (int)m_eAxisDir);


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

