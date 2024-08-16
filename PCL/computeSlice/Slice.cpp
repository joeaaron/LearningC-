#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

const int PNT_STRIDE = 12;
const int PNT_MAX = 600000;
const int ROT_MATRIX_SIZE = 3;
const int TRANSLATION_SIZE = 3;

//TransData transData;

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
		double sinX = sin(transData.rotX);  // * M_PI / 180);		// �Ƕ�ת����
		double cosX = cos(transData.rotX);  // * M_PI / 180);

		double sinY = sin(transData.rotY);  // * M_PI / 180);		// �Ƕ�ת����
		double cosY = cos(transData.rotY);  // * M_PI / 180);

		double sinZ = sin(transData.rotZ); // * M_PI / 180);	    // �Ƕ�ת����
		double cosZ = cos(transData.rotZ); // * M_PI / 180);

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
		Eigen::Vector4d T5(0, transData.moveX, transData.moveY, transData.moveZ);

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
		transData.mtxR = R5;
		transData.vecT = T5;
	}

	/*
	 * @brief ������YZ��Ľ���
	 *
	 * ���ݸ�����2���߶Σ����������߶Ρ�
	 *
	 * @param lineSeg1		�߶�1
	 * @param lineSeg2		�߶�2
	 * @param dSlicePos	    ��Ƭλ��
	 * @return ��������
	 */
	void CalLineSegX(LineSeg& lineSegOut, const LineSeg& lineSeg1, const LineSeg& lineSeg2, double dSlicePos)
	{
		double dCoef1 = dSlicePos - lineSeg1.x;
		double dCoef2 = lineSeg2.x - lineSeg1.x;

		if (fabs(dCoef2) < DBL_EPSILON) return;

		lineSegOut.z = dCoef1 * (lineSeg2.z - lineSeg1.z) / dCoef2 + lineSeg1.z;
		lineSegOut.y = dCoef1 * (lineSeg2.y - lineSeg1.y) / dCoef2 + lineSeg1.y;
	/*	lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
		lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
		lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;*/
	}

	void CalLineSegY(LineSeg& lineSegOut, const LineSeg& lineSeg1, const LineSeg& lineSeg2, double dSlicePos)
	{
		double dCoef1 = dSlicePos - lineSeg1.y;
		double dCoef2 = lineSeg2.y - lineSeg1.y;

		if (fabs(dCoef2) < DBL_EPSILON) return;

		lineSegOut.z = dCoef1 * (lineSeg2.z - lineSeg1.z) / dCoef2 + lineSeg1.z;
		lineSegOut.x = dCoef1 * (lineSeg2.x - lineSeg1.x) / dCoef2 + lineSeg1.x;
		/*	lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
			lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
			lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;*/
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
	void CalLineSegZ(LineSeg& lineSegOut, const LineSeg& lineSeg1, const LineSeg& lineSeg2, double dSlicePos)
	{
		double dCoef1 = dSlicePos - lineSeg1.z;
		double dCoef2 = lineSeg2.z - lineSeg1.z;

		if (fabs(dCoef2) < DBL_EPSILON) return;

		lineSegOut.x = dCoef1 * (lineSeg2.x - lineSeg1.x) / dCoef2 + lineSeg1.x;
		lineSegOut.y = dCoef1 * (lineSeg2.y - lineSeg1.y) / dCoef2 + lineSeg1.y;
		//lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
		//lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
		//lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;
	}

	bool IsSegmentEqualX(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
		const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
	{
		return (lineSeg1.z == slcLineSeg0.z && lineSeg2.z == slcLineSeg1.z && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
			|| (lineSeg2.z == slcLineSeg0.z && lineSeg1.z == slcLineSeg1.z && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
	}

	/*
	* @brief �ж������߶��Ƿ������ͬ�������˵�
	*
	* @param lineSeg1		�߶�1
	* @param lineSeg1		�߶�2
	* @param slcLineSeg0	��Ƭ�߶�1
	* @param slcLineSeg1	��Ƭ�߶�2
	*/
	bool IsSegmentEqualZ(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
		const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
	{
		return (lineSeg1.x == slcLineSeg0.x && lineSeg2.x == slcLineSeg1.x && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
			|| (lineSeg2.x == slcLineSeg0.x && lineSeg1.x == slcLineSeg1.x && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
	}

	void ComputeSlcX(SliceBuf* pBuf, double dSlicePos)
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

			lineSeg1.x = pCloud[k * PNT_STRIDE + 4]; lineSeg1.y = pCloud[k * PNT_STRIDE + 5]; lineSeg1.z = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.x = pCloud[k * PNT_STRIDE + 7]; lineSeg2.y = pCloud[k * PNT_STRIDE + 8]; lineSeg2.z = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.x = pCloud[k * PNT_STRIDE + 10]; lineSeg3.y = pCloud[k * PNT_STRIDE + 11]; lineSeg3.z = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			//����λ�����ٽ��ĵ�����
			if (lineSeg1.x == dSlicePos && lineSeg2.x == dSlicePos && lineSeg3.x == dSlicePos) { continue; }
			bool flagRelated = false;
			int dt = 0;

			// ����������ڵ㶼��dSlicePosƽ���ͬ�ࣨ�������ڻ�С��dSlicePos�������һ������Ƿ���һ������������dSlicePosƽ���ϡ�
			// ����ǣ�����Щ�߶δ洢��pBuf->slc�С�
			if ((lineSeg1.x >= dSlicePos && lineSeg2.x >= dSlicePos && lineSeg3.x >= dSlicePos)
				|| (lineSeg1.x <= dSlicePos && lineSeg2.x <= dSlicePos && lineSeg3.x <= dSlicePos))
			{
				//if (lineSeg1.x == dSlicePos && lineSeg2.x == dSlicePos && lineSeg3.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg2;
				//	dt++;

				//}
				//if (lineSeg1.x == dSlicePos && lineSeg3.x == dSlicePos && lineSeg2.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg3;
				//	dt++;
				//}
				//if (lineSeg3.x == dSlicePos && lineSeg2.x == dSlicePos && lineSeg1.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg3;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg2;
				//	dt++;
				//}
				//pBuf->slc[nCnt].nLineA = 2;
			}
			// ����������ڵ㲻��dSlicePosƽ���ͬ�࣬�������dSlicePosƽ���ཻ���߶Σ�������洢��pBuf->slc�С�
			else
			{
				//�Ե���бȶԺ��ɾ��
				nCnt++;  dt = 0;   flagRelated = true;
				LineSegment lineSegOut;

				if ((lineSeg1.x - dSlicePos) * (lineSeg2.x - dSlicePos) < 0)
				{
					CalLineSegX(lineSegOut, lineSeg1, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg1.x - dSlicePos) * (lineSeg3.x - dSlicePos) < 0)
				{
					CalLineSegX(lineSegOut, lineSeg1, lineSeg3, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg3.x - dSlicePos) * (lineSeg2.x - dSlicePos) < 0)
				{
					CalLineSegX(lineSegOut, lineSeg3, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if (lineSeg1.x == dSlicePos && (lineSeg3.x - dSlicePos) * (lineSeg2.x - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg1; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg1; }

					dt++;
				}
				if (lineSeg2.x == dSlicePos && (lineSeg1.x - dSlicePos) * (lineSeg3.x - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg2; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg2; }

					dt++;
				}
				if (lineSeg3.x == dSlicePos && (lineSeg1.x - dSlicePos) * (lineSeg2.x - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg3; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg3; }

					dt++;
				}

				pBuf->slc[nCnt].nLineA = 1;
			}

			//������Ƭ��λ�ã�����ÿ����dSlicePosƽ���ཻ���߶Σ�ȷ��������Ƭƽ���ϵķ��򣨻��ڵ�ǰ����߶εķ��򣩡�
			//if (flagRelated)
			//{
			//	lineSeg1 = pBuf->slc[nCnt].lineSeg0;
			//	lineSeg2 = pBuf->slc[nCnt].lineSeg1;
			//	lineSeg3.z = lineSeg2.z - lineSeg1.z;
			//	lineSeg3.y = lineSeg2.y - lineSeg1.y;

			//	if (zn * lineSeg3.y - yn * lineSeg3.z < 0)
			//	{
			//		pBuf->slc[nCnt].nLineB = -1;
			//	}
			//	else
			//	{
			//		pBuf->slc[nCnt].nLineB = 1;
			//	}
			//}
		}

		//ɾ��buf.line_m[cnt]=2;�ظ��߶Σ������ϲ࣬�����߶�ɾ����
		//���ߵ�������������ƽ��ģ�1���£�2ͬ�࣬3һ��һƽ��4��ƽ    
		//����ѡz>z9�������� 
		//for (int k = 1; k <= nCnt - 1; k++)
		//{
		//	if (pBuf->slc[k].nLineA == 2)
		//	{
		//		bool flagSameSide = false;
		//		lineSeg1 = pBuf->slc[k].lineSeg0;
		//		lineSeg2 = pBuf->slc[k].lineSeg1;

		//		for (int i = k + 1; i <= nCnt; i++)
		//		{
		//			if (IsSegmentEqualX(lineSeg1, lineSeg2, pBuf->slc[i].lineSeg0, pBuf->slc[i].lineSeg1))
		//			{
		//				flagSameSide = true;

		//				for (int j = i; j <= nCnt - 1; j++)
		//				{
		//					pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
		//				}
		//				--nCnt; // ���ټ���
		//			}
		//		}
		//		//ͬ��������߶ζ�ɾ��
		//		if (flagSameSide)
		//		{
		//			for (int j = k; j <= nCnt - 1; j++)
		//			{
		//				pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
		//			}
		//			--nCnt;
		//		}
		//	}
		//}

		// ����pbuf->cnt�Է�ӳ�洢��pbuf->slc�е��߶�����
		pBuf->nCnt = nCnt;
	}

	void ComputeSlcY(SliceBuf* pBuf, double dSlicePos)
	{
		if (pBuf == nullptr || pBuf->pCloud.empty())
		{
			return;
		}

		auto pCloud = pBuf->pCloud;
		long lCloudNum = pBuf->lCloudNum;
		int nCnt = 0;

		LineSeg lineSeg1, lineSeg2, lineSeg3;

		for (int k = 1; k <= lCloudNum; k++)
		{
			if (nCnt + 1 >= PNT_MAX)
			{
				break;
			}
			// ��ȡ��ǰ����������������
			double xn = pCloud[k * PNT_STRIDE + 1];
			double yn = pCloud[k * PNT_STRIDE + 2];
			double zn = pCloud[k * PNT_STRIDE + 3];

			lineSeg1.x = pCloud[k * PNT_STRIDE + 4]; lineSeg1.y = pCloud[k * PNT_STRIDE + 5]; lineSeg1.z = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.x = pCloud[k * PNT_STRIDE + 7]; lineSeg2.y = pCloud[k * PNT_STRIDE + 8]; lineSeg2.z = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.x = pCloud[k * PNT_STRIDE + 10]; lineSeg3.y = pCloud[k * PNT_STRIDE + 11]; lineSeg3.z = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			//����λ�����ٽ��ĵ�����
			if (lineSeg1.y == dSlicePos && lineSeg2.y == dSlicePos && lineSeg3.y == dSlicePos) { continue; }
			bool flagRelated = false;
			int dt = 0;

			// ����������ڵ㶼��dSlicePosƽ���ͬ�ࣨ�������ڻ�С��dSlicePos�������һ������Ƿ���һ������������dSlicePosƽ���ϡ�
			// ����ǣ�����Щ�߶δ洢��pBuf->slc�С�
			if ((lineSeg1.y >= dSlicePos && lineSeg2.y >= dSlicePos && lineSeg3.y >= dSlicePos)
				|| (lineSeg1.y <= dSlicePos && lineSeg2.y <= dSlicePos && lineSeg3.y <= dSlicePos))
			{
				//if (lineSeg1.x == dSlicePos && lineSeg2.x == dSlicePos && lineSeg3.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg2;
				//	dt++;

				//}
				//if (lineSeg1.x == dSlicePos && lineSeg3.x == dSlicePos && lineSeg2.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg1;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg3;
				//	dt++;
				//}
				//if (lineSeg3.x == dSlicePos && lineSeg2.x == dSlicePos && lineSeg1.x > dSlicePos)
				//{
				//	nCnt++;  dt = 0;   flagRelated = true;
				//	pBuf->slc[nCnt].lineSeg0 = lineSeg3;
				//	dt++;
				//	pBuf->slc[nCnt].lineSeg1 = lineSeg2;
				//	dt++;
				//}
				//pBuf->slc[nCnt].nLineA = 2;
			}
			// ����������ڵ㲻��dSlicePosƽ���ͬ�࣬�������dSlicePosƽ���ཻ���߶Σ�������洢��pBuf->slc�С�
			else
			{
				//�Ե���бȶԺ��ɾ��
				nCnt++;  dt = 0;   flagRelated = true;
				LineSegment lineSegOut;

				if ((lineSeg1.y - dSlicePos) * (lineSeg2.y - dSlicePos) < 0)
				{
					CalLineSegY(lineSegOut, lineSeg1, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg1.y - dSlicePos) * (lineSeg3.y - dSlicePos) < 0)
				{
					CalLineSegY(lineSegOut, lineSeg1, lineSeg3, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg3.y - dSlicePos) * (lineSeg2.y - dSlicePos) < 0)
				{
					CalLineSegY(lineSegOut, lineSeg3, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if (lineSeg1.y == dSlicePos && (lineSeg3.y - dSlicePos) * (lineSeg2.y - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg1; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg1; }

					dt++;
				}
				if (lineSeg2.y == dSlicePos && (lineSeg1.y - dSlicePos) * (lineSeg3.y - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg2; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg2; }

					dt++;
				}
				if (lineSeg3.y == dSlicePos && (lineSeg1.y - dSlicePos) * (lineSeg2.y - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg3; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg3; }

					dt++;
				}

				pBuf->slc[nCnt].nLineA = 1;
			}

			//������Ƭ��λ�ã�����ÿ����dSlicePosƽ���ཻ���߶Σ�ȷ��������Ƭƽ���ϵķ��򣨻��ڵ�ǰ����߶εķ��򣩡�
			//if (flagRelated)
			//{
			//	lineSeg1 = pBuf->slc[nCnt].lineSeg0;
			//	lineSeg2 = pBuf->slc[nCnt].lineSeg1;
			//	lineSeg3.z = lineSeg2.z - lineSeg1.z;
			//	lineSeg3.y = lineSeg2.y - lineSeg1.y;

			//	if (zn * lineSeg3.y - yn * lineSeg3.z < 0)
			//	{
			//		pBuf->slc[nCnt].nLineB = -1;
			//	}
			//	else
			//	{
			//		pBuf->slc[nCnt].nLineB = 1;
			//	}
			//}
		}

		//ɾ��buf.line_m[cnt]=2;�ظ��߶Σ������ϲ࣬�����߶�ɾ����
		//���ߵ�������������ƽ��ģ�1���£�2ͬ�࣬3һ��һƽ��4��ƽ    
		//����ѡz>z9�������� 
		//for (int k = 1; k <= nCnt - 1; k++)
		//{
		//	if (pBuf->slc[k].nLineA == 2)
		//	{
		//		bool flagSameSide = false;
		//		lineSeg1 = pBuf->slc[k].lineSeg0;
		//		lineSeg2 = pBuf->slc[k].lineSeg1;

		//		for (int i = k + 1; i <= nCnt; i++)
		//		{
		//			if (IsSegmentEqualX(lineSeg1, lineSeg2, pBuf->slc[i].lineSeg0, pBuf->slc[i].lineSeg1))
		//			{
		//				flagSameSide = true;

		//				for (int j = i; j <= nCnt - 1; j++)
		//				{
		//					pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
		//				}
		//				--nCnt; // ���ټ���
		//			}
		//		}
		//		//ͬ��������߶ζ�ɾ��
		//		if (flagSameSide)
		//		{
		//			for (int j = k; j <= nCnt - 1; j++)
		//			{
		//				pBuf->slc[j] = pBuf->slc[j + 1]; // ��ǰ�ƶ��߶�
		//			}
		//			--nCnt;
		//		}
		//	}
		//}

		// ����pbuf->cnt�Է�ӳ�洢��pbuf->slc�е��߶�����
		pBuf->nCnt = nCnt;
	}

	/*
	 * @brief �Ե������ݽ�����ת��ƽ��
	 *
	 * ���ݸ�������ת�ǶȺ�ƽ�������Ե������ݽ�����ת��ƽ�ơ�
	 *
	 * @param pBuf			�������ݵĽṹ��ָ��
	 * @param dSlicePos	    ��Ƭλ��
	 */
	void ComputeSlcZ(SliceBuf* pBuf, double dSlicePos)
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

			lineSeg1.x = pCloud[k * PNT_STRIDE + 4]; lineSeg1.y = pCloud[k * PNT_STRIDE + 5]; lineSeg1.z = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.x = pCloud[k * PNT_STRIDE + 7]; lineSeg2.y = pCloud[k * PNT_STRIDE + 8]; lineSeg2.z = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.x = pCloud[k * PNT_STRIDE + 10]; lineSeg3.y = pCloud[k * PNT_STRIDE + 11]; lineSeg3.z = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			//����λ�����ٽ��ĵ�����
			if (lineSeg1.z == dSlicePos && lineSeg2.z == dSlicePos && lineSeg3.z == dSlicePos) { continue; }
			bool flagRelated = false;
			int dt = 0;

			// ����������ڵ㶼��dSlicePosƽ���ͬ�ࣨ�������ڻ�С��dSlicePos�������һ������Ƿ���һ������������dSlicePosƽ���ϡ�
			// ����ǣ�����Щ�߶δ洢��pBuf->slc�С�
			if ((lineSeg1.z >= dSlicePos && lineSeg2.z >= dSlicePos && lineSeg3.z >= dSlicePos)
				|| (lineSeg1.z <= dSlicePos && lineSeg2.z <= dSlicePos && lineSeg3.z <= dSlicePos))
			{
				if (lineSeg1.z == dSlicePos && lineSeg2.z == dSlicePos && lineSeg3.z > dSlicePos)
				{
					nCnt++;  dt = 0;   flagRelated = true;
					pBuf->slc[nCnt].lineSeg0 = lineSeg1;
					dt++;
					pBuf->slc[nCnt].lineSeg1 = lineSeg2;
					dt++;

				}
				if (lineSeg1.z == dSlicePos && lineSeg3.z == dSlicePos && lineSeg2.z > dSlicePos)
				{
					nCnt++;  dt = 0;   flagRelated = true;
					pBuf->slc[nCnt].lineSeg0 = lineSeg1;
					dt++;
					pBuf->slc[nCnt].lineSeg1 = lineSeg3;
					dt++;
				}
				if (lineSeg3.z == dSlicePos && lineSeg2.z == dSlicePos && lineSeg1.z > dSlicePos)
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

				if ((lineSeg1.z - dSlicePos) * (lineSeg2.z - dSlicePos) < 0)
				{
					CalLineSegZ(lineSegOut, lineSeg1, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg1.z - dSlicePos) * (lineSeg3.z - dSlicePos) < 0)
				{
					CalLineSegZ(lineSegOut, lineSeg1, lineSeg3, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if ((lineSeg3.z - dSlicePos) * (lineSeg2.z - dSlicePos) < 0)
				{
					CalLineSegZ(lineSegOut, lineSeg3, lineSeg2, dSlicePos);

					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSegOut; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSegOut; }

					dt++;
				}
				if (lineSeg1.z == dSlicePos && (lineSeg3.z - dSlicePos) * (lineSeg2.z - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg1; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg1; }

					dt++;
				}
				if (lineSeg2.z == dSlicePos && (lineSeg1.z - dSlicePos) * (lineSeg3.z - dSlicePos) < 0)
				{
					if (dt == 0) { pBuf->slc[nCnt].lineSeg0 = lineSeg2; }
					if (dt == 1) { pBuf->slc[nCnt].lineSeg1 = lineSeg2; }

					dt++;
				}
				if (lineSeg3.z == dSlicePos && (lineSeg1.z - dSlicePos) * (lineSeg2.z - dSlicePos) < 0)
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
					if (IsSegmentEqualZ(lineSeg1, lineSeg2, pBuf->slc[i].lineSeg0, pBuf->slc[i].lineSeg1))
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
	void RotateSlcZ(SliceBuf* pbuf, double dSlicePos, const TransData& transData)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// ��ת����
		for (int i = 0; i < ROT_MATRIX_SIZE; i++)
		{
			for (int j = 0; j < ROT_MATRIX_SIZE; j++)
			{
				R(i, j) = transData.mtxR(i, j);
			}
			T(i) = transData.vecT(i);
		}

		// �����������Ը����ĵ������ת��ƽ��
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R * (p - T) + T;
			};

		// ����Ƭ�˵�0��1������ת��ƽ��
		for (int k = 0; k < pbuf->nCnt; k++)
		{
			rotateAndTranslate(pbuf->slc[k].lineSeg0.x, pbuf->slc[k].lineSeg0.y, pbuf->slc[k].vec0);
			rotateAndTranslate(pbuf->slc[k].lineSeg1.x, pbuf->slc[k].lineSeg1.y, pbuf->slc[k].vec1);
		}
	}

	void RotateSlcX(SliceBuf* pbuf, double dSlicePos, const TransData& transData)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// ��ת����
		for (int i = 0; i < ROT_MATRIX_SIZE; i++)
		{
			for (int j = 0; j < ROT_MATRIX_SIZE; j++)
			{
				R(i, j) = transData.mtxR(i + 1, j + 1);
			}
			T(i) = transData.vecT(i + 1);
		}

		// �����������Ը����ĵ������ת��ƽ��
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R * (p - T) + T;
			};

		// ����Ƭ�˵�0��1������ת��ƽ��
		for (int k = 0; k < pbuf->nCnt; k++)
		{
			rotateAndTranslate(pbuf->slc[k].lineSeg0.z, pbuf->slc[k].lineSeg0.y, pbuf->slc[k].vec0);
			rotateAndTranslate(pbuf->slc[k].lineSeg1.z, pbuf->slc[k].lineSeg1.y, pbuf->slc[k].vec1);
		}
	}

	void RotateSlcY(SliceBuf* pbuf, double dSlicePos, const TransData& transData)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// ��ת����
		for (int i = 0; i < ROT_MATRIX_SIZE; i++)
		{
			for (int j = 0; j < ROT_MATRIX_SIZE; j++)
			{
				R(i, j) = transData.mtxR(i + 1, j + 1);
			}
			T(i) = transData.vecT(i + 1);
		}

		// �����������Ը����ĵ������ת��ƽ��
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R * (p - T) + T;
			};

		// ����Ƭ�˵�0��1������ת��ƽ��
		for (int k = 0; k < pbuf->nCnt; k++)
		{
			rotateAndTranslate(pbuf->slc[k].lineSeg0.x, pbuf->slc[k].lineSeg0.z, pbuf->slc[k].vec0);
			rotateAndTranslate(pbuf->slc[k].lineSeg1.x, pbuf->slc[k].lineSeg1.z, pbuf->slc[k].vec1);
		}
	}


}

PCL_Slice::PCL_Slice()
	: m_eMessage(eExtraPcl_Message::eMessageOK)
	, m_dSlicePos(6.0)
	, m_dInterval(0.03)
	, m_transData(0, 0, 0, 0, 0, 0)
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
	ComputeSlcZ(pBuf, m_dSlicePos);

	// ��ת��Ƭ����
	RotateSlcZ(pBuf, m_dSlicePos, m_transData);

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

