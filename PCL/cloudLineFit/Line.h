#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen\src\Core\Matrix.h>

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\io\io.h>

template<int32_t Dim, typename Real, typename PointT>
class LineFit
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	LineFit() {}
	~LineFit() {}

	bool setFittingPoints(std::vector<PointT, Eigen::aligned_allocator<PointT>>& points)
	{
		if (points.size() < 3)
		{
			return false;
		}
		mInitialPoints.resize(points.size());
		for (int i = 0; i < points.size(); ++i)
		{
			for (int j = 0; j < Dim; ++j)
				mInitialPoints[i][j] = points[i].data[j];
		}
		return true;
	}

	bool computeLineParam(Eigen::Vector<Real, 2 * Dim>& param)
	{
		bool isTrue = computeSuitableLsBaseVectors();
		if (!isTrue)
		{
			return false;
		}
		//param.resize(2 * Dim);
		for (int i = 0; i < Dim; ++i)
		{
			param(i) = mCenterPoint[i];
			param(i + Dim) = mLsVector[0][i];
		}

		return true;
	}

	bool getCenterPoint(Eigen::Vector<Real, Dim>& point)
	{
		point = getCenterPoint();
		return true;
	}

private:
	Eigen::Vector<Real, Dim> getCenterPoint() //获取重心点
	{
		int tCount = mInitialPoints.size();
		Eigen::Vector<Real, Dim> tPoint;
		tPoint.setZero();
		for (size_t i = 0; i < tCount; i++)
		{
			tPoint = tPoint + mInitialPoints[i] / tCount;
		}
		this->mCenterPoint = tPoint;

		return this->mCenterPoint;
	}

	Eigen::Matrix<Real, Dim, Dim> computeCovarianceMatrix()  //获取领域点的协方差矩阵
	{
		Eigen::Vector<Real, Dim> tCenterPoint = getCenterPoint();
		int count = mInitialPoints.size();
		Eigen::Vector<Real, Dim> tData;
		tData.setZero();
		Eigen::Matrix<Real, Dim, Dim> tCovMat = Eigen::Matrix<Real, Dim, Dim>::Zero();
		for (size_t i = 0; i < count; i++)
		{
			tData = mInitialPoints[i] - tCenterPoint;
			tCovMat += tData * tData.transpose() / count;
		}

		return tCovMat;
	}

	bool computeEigenValuesAndVectors(Eigen::Matrix<Real, Dim, Dim> covMat
		, Eigen::Matrix<Real, Dim, Dim>& eigVectors, Eigen::Vector<Real, Dim>& eigValues)
	{
		Eigen::JacobiSVD<Eigen::Matrix<Real, Dim, Dim>> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
		eigValues = svd.singularValues();
		eigVectors = svd.matrixV();
		//Eigen::Vector<Real, Dim> n = vec.col(Dim - 1);

		return true;
	}

	bool computeSuitableLsBaseVectors() //计算最小二乘基向量
	{
		if (mInitialPoints.size() > 3)
		{
			Eigen::Matrix<Real, Dim, Dim> covMat = computeCovarianceMatrix();
			Eigen::Matrix<Real, Dim, Dim> eigVectors;
			Eigen::Vector<Real, Dim> eigValues;
			computeEigenValuesAndVectors(covMat, eigVectors, eigValues);

			mLsVector.resize(Dim);
			for (int i = 0; i < Dim; ++i)
				mLsVector[i] = eigVectors.col(i);
			mCenterPoint = getCenterPoint();
		}
		else
		{
			return false;
		}

		if (mLsVector[Dim - 1].norm() < 0.00000001)
		{
			return false;
		}

		return true;
	}

private:
	std::vector<Eigen::Vector<Real, Dim>> mInitialPoints;
	std::vector<Eigen::Vector<Real, Dim>> mLsVector;
	Eigen::Vector<Real, Dim> mCenterPoint;
};
