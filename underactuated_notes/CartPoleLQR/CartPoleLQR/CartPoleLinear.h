#include "Eigen/Dense"
#include "SystemParams.h"

#pragma once

class CartPoleLinear
{
private:
	Eigen::Matrix4d _a;
	Eigen::Matrix<double, 4, 1> _b;
	Eigen::Matrix<double, 2, 4> _c;
	Eigen::Matrix<double, 2, 1> _d;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CartPoleLinear(const SystemParams& system_params);

	Eigen::Matrix4d AMatrix() { return _a; }
	Eigen::Matrix<double, 4, 1> BMatrix() { return _b; }
	Eigen::Matrix<double, 2, 4> CMatrix() { return _c; }
	Eigen::Matrix<double, 2, 1> DMatrix() { return _d; }
};

