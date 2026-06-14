// author: Horibe Takamasa

#ifndef BUILD_RICCATI_SOLVER_H
#define BUILD_RICCATI_SOLVER_H

#include <Eigen/Dense>

inline bool solveRiccatiArimotoPotter(const Eigen::MatrixXd &A,
                               const Eigen::MatrixXd &B,
                               const Eigen::MatrixXd &Q,
                               const Eigen::MatrixXd &R, Eigen::MatrixXd &P) {

  const long dim_x = A.rows();

  // set Hamilton matrix
  Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
  Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

  // calc eigenvalues and eigenvectors
  const Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

  // check eigen values
  // std::cout << "eigen values：\n" << Eigs.eigenvalues() << std::endl;
  // std::cout << "eigen vectors：\n" << Eigs.eigenvectors() << std::endl;

  // extract stable eigenvectors into 'eigvec'
  Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
  int j = 0;
  for (int i = 0; i < 2 * dim_x; ++i) {
    if (Eigs.eigenvalues()[i].real() < 0.) {
      eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
      ++j;
    }
  }

  // calc P with stable eigen vector matrix
  const Eigen::MatrixXcd Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
  const Eigen::MatrixXcd Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
  P = (Vs_2 * Vs_1.inverse()).real();

  return true;
}
#endif //BUILD_RICCATI_SOLVER_H
