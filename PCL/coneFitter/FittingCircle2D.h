#ifndef FITTIINGCIRCLE2D_H_H
#define FITTIINGCIRCLE2D_H_H
#include "FittingBase.h"

namespace Gauss 
{
	double F(Fitting::Circle2D circle, const Point &p);
	double GetError(Fitting::Circle2D circle, const std::vector<Eigen::Vector3d>& points);

	class FittingCircle2D: public Fitting::FittingBase  
	{
		Fitting::Circle2D circle;
		Fitting::Matrix Jacobi(const std::vector<Eigen::Vector3d>& points);

		/* afterHook
		* ��������´�
		*/
		void afterHook(const Eigen::VectorXd& xp);


		/* ��ȡ d����
		*/
		Eigen::VectorXd getDArray(const std::vector<Eigen::Vector3d>& points);

		// GetInitFit
		bool GetInitFit(const std::vector<Eigen::Vector3d>& points);
		
		/* F
		* ���뺯��
		*/
		double F(const Eigen::Vector3d& p);

		/* ��ȡ ��С���˲в�
		*/
		double  GetError(const std::vector<Eigen::Vector3d>& points);

		/* ��ȡ ���
		*/
		void Copy(void* ele);
	};
}


#endif // !FITTIINGCIRCLE2D_H_H