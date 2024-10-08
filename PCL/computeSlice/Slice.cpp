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
	 * @brief 对点云数据进行旋转和平移
	 *
	 * 根据给定的旋转角度和平移量，对点云数据进行旋转和平移。
	 *
	 * @param pBuf 点云数据的结构体指针
	 */
	void RotateCloud(SliceBuf* pBuf, TransData& transData)
	{
		if (pBuf == nullptr || pBuf->pCloud.empty())
		{
			return;
		}

		long lCloudNum = pBuf->lCloudNum;
		auto& pCloud = pBuf->pCloud;

		// 摆放旋转平移
		double sinX = sin(transData.m_rot[0]);  	// 角度转弧度
		double cosX = cos(transData.m_rot[0]);

		double sinY = sin(transData.m_rot[1]);  	// 角度转弧度
		double cosY = cos(transData.m_rot[1]);

		double sinZ = sin(transData.m_rot[2]);		// 角度转弧度
		double cosZ = cos(transData.m_rot[2]);

		// 绕X轴的旋转矩阵
		Eigen::Matrix4d R1, R2, R3;
		R1 << 1, 0, 0, 0,
			0, cosX, -sinX, 0,
			0, sinX, cosX, 0,
			0, 0, 0, 1;

		// 绕Y轴的旋转矩阵
		R2 << cosY, 0, sinY, 0,
			0, 1, 0, 0,
			-sinY, 0, cosY, 0,
			0, 0, 0, 1;

		// 绕Z轴的旋转矩阵
		R3 << cosZ, -sinZ, 0, 0,
			sinZ, cosZ, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		transData.m_mtxR = R3 * R2 * R1;

		// 平移向量
		Eigen::Vector4d T(0, transData.m_move[0], transData.m_move[1], transData.m_move[2]);
		transData.m_vecT = T;

		// 旋转和平移点云
#pragma omp parallel for collapse(2)
		for (int k = 1; k < lCloudNum; k++)
		{
			for (int i = 0; i < 4; i++)
			{
				Eigen::Vector4d p;
				for (int j = 1; j < 4; j++)
				{
					// 取三角面上3个点
					p[j] = pCloud[k * PNT_STRIDE + i * 3 + j];
				}

				// 点云旋转平移后的坐标 p2 = R5 * p1 + T5
				Eigen::Vector4d p1;
				p1.head<3>() = p.tail<3>();
				Eigen::Vector4d transPt = transData.m_mtxR * p1;
				if (i > 0)
				{
					transPt += T;
				}

				Eigen::Vector4d result;
				result.tail<3>() = transPt.head<3>();

				for (int j = 1; j < 4; j++) {
					pCloud[k * PNT_STRIDE + i * 3 + j] = result[j];
				}
			}
		}

		// 输出点云
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
		//viewer->setWindowName(u8"切片点云");

		//viewer->addPointCloud(cloud, "Slice");
		//viewer->setRepresentationToSurfaceForAllActors();
		//viewer->initCameraParameters();
		//viewer->resetCamera();
		//viewer->spin();
		
	}


	/*
	 * @brief 计算与XY面的交点
	 *
	 * 根据给定的2个线段，计算最终线段。
	 *
	 * @param lineSeg1		线段1
	 * @param lineSeg2		线段2
	 * @param dSlicePos	    切片位置
	 * @return 计算结果。
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
	* @brief 判断两个线段是否具有相同的两个端点
	*
	* @param lineSeg1		线段1
	* @param lineSeg1		线段2
	* @param slcLineSeg0	切片线段1
	* @param slcLineSeg1	切片线段2
	*/
	bool IsSegmentEqual(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
		const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
	{
		return (lineSeg1.m_point.x() == slcLineSeg0.m_point.x() && lineSeg2.m_point.x() == slcLineSeg1.m_point.x() && lineSeg1.m_point.y() == slcLineSeg0.m_point.y() && lineSeg2.m_point.y() == slcLineSeg1.m_point.y())
			|| (lineSeg2.m_point.x() == slcLineSeg0.m_point.x() && lineSeg1.m_point.x() == slcLineSeg1.m_point.x() && lineSeg2.m_point.y() == slcLineSeg0.m_point.y() && lineSeg1.m_point.y() == slcLineSeg1.m_point.y());
	}

	/*
	 * @brief 对点云数据进行旋转和平移
	 *
	 * 根据给定的旋转角度和平移量，对点云数据进行旋转和平移。
	 *
	 * @param pBuf			点云数据的结构体指针
	 * @param dSlicePos	    切片位置
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
			// 提取当前点和相邻两点的坐标
			double xn = pCloud[k * PNT_STRIDE + 1];
			double yn = pCloud[k * PNT_STRIDE + 2];
			double zn = pCloud[k * PNT_STRIDE + 3];

			lineSeg1.m_point.x() = pCloud[k * PNT_STRIDE + 4]; lineSeg1.m_point.y()= pCloud[k * PNT_STRIDE + 5]; lineSeg1.m_point.z() = pCloud[k * PNT_STRIDE + 6];
			lineSeg2.m_point.x() = pCloud[k * PNT_STRIDE + 7]; lineSeg2.m_point.y()= pCloud[k * PNT_STRIDE + 8]; lineSeg2.m_point.z() = pCloud[k * PNT_STRIDE + 9];
			lineSeg3.m_point.x() = pCloud[k * PNT_STRIDE + 10]; lineSeg3.m_point.y() = pCloud[k * PNT_STRIDE + 11]; lineSeg3.m_point.z() = pCloud[k * PNT_STRIDE + PNT_STRIDE];

			//查找位置相临近的点坐标
			if (lineSeg1.m_point.z() == dSlicePos && lineSeg2.m_point.z() == dSlicePos && lineSeg3.m_point.z() == dSlicePos) { continue; }
			bool flagRelated = false;
			int dt = 0;

			// 如果三个相邻点都在dSlicePos平面的同侧（即都大于或都小于dSlicePos），则进一步检查是否有一个或两个点在dSlicePos平面上。
			// 如果是，则将这些线段存储在pBuf->slc中。
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
			// 如果三个相邻点不在dSlicePos平面的同侧，则计算与dSlicePos平面相交的线段，并将其存储在pBuf->slc中。
			else
			{
				//对点进行比对后的删除
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

			//计算切片点位置：对于每个与dSlicePos平面相交的线段，确定其在切片平面上的方向（基于当前点和线段的方向）。
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

		//删除buf.line_m[cnt]=2;重复线段（都在上侧，两段线都删除）
		//切线的两三角形在切平面的：1上下，2同侧，3一上一平，4两平    
		//上下选z>z9的三角形 
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
							pBuf->slc[j] = pBuf->slc[j + 1]; // 向前移动线段
						}
						--nCnt; // 减少计数
					}
				}
				//同侧的两个线段都删除
				if (flagSameSide)
				{
					for (int j = k; j <= nCnt - 1; j++)
					{
						pBuf->slc[j] = pBuf->slc[j + 1]; // 向前移动线段
					}
					--nCnt;
				}
			}
		}

		// 更新pbuf->cnt以反映存储在pbuf->slc中的线段数量
		pBuf->nCnt = nCnt;
	}

	/*
	 * @brief 旋转切片点云
	 *
	 * 根据给定的旋转角度和平移量，对点云数据进行旋转和平移。
	 *
	 * @param pBuf			切片点云数据的结构体指针
	 * @param dSlicePos	    切片位置
	 */
	void RotateSlc(SliceBuf* pbuf, double dSlicePos, const TransData& transData)
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;

		// 旋转矩阵
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

		// 辅助函数，对给定的点进行旋转和平移
		auto rotateAndTranslate = [&](double x, double y, Eigen::Vector3d& newPt) {
			Eigen::Vector3d p(x, y, dSlicePos);
			newPt = R_inv * (p - T_inv) + T_inv;
			};

		// 对切片端点0和1进行旋转和平移
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

	// 旋转点云
 	RotateCloud(pBuf, m_transData);

	// 计算切片
	ComputeSlc(pBuf, m_dSlicePos);

	// 旋转切片点云
	RotateSlc(pBuf, m_dSlicePos, m_transData);

	// 输出点云
	pCloudOut = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	for (int i = 0; i <= pBuf->nCnt; ++i)
	{
		//检测距离是否足够小
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

