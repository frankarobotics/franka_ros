// Author: Enrico Corvaglia
// https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_controllers/include/utils/pseudo_inversion.h
// File provided under public domain
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose
// between damped and not)
// returns the pseudo inverted matrix M_pinv_

#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace franka_example_controllers {

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd sing_vals_ = svd.singularValues();
  for (unsigned int i = 0; i < sing_vals_.size(); ++i)
    sing_vals_[i] = sing_vals_[i] / (sing_vals_[i] * sing_vals_[i] + lambda_ * lambda_);

  M_pinv_.noalias() =
      Eigen::MatrixXd(svd.matrixV() * (sing_vals_.asDiagonal() * svd.matrixU().transpose()));
}

}  // namespace franka_example_controllers
