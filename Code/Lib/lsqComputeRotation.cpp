/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "lsqComputeRotation.h"

namespace lsq {

  /// Private method to check whether two numbers are approximately equal.
  bool SVDMethod::m_are_almost_equal(double a, double b) {
    return fabs(a - b) < m_precision;
  }

  /// Method to compute the rotation matrix which connect the two set of 3D points using SVD.
  Eigen::Matrix3d SVDMethod::find_rotation(Eigen::Matrix3d H_matrix) {

    Eigen::Matrix3d U_matrix;
    Eigen::Matrix3d V_matrix;
    Eigen::Vector3d singular_values;

    Eigen::Matrix3d rotation_matrix;
    double determinant_rotation;

    Eigen::JacobiSVD<Eigen::Matrix3d> singular_value_decomposition(H_matrix,
                                                                   Eigen::FullPivHouseholderQRPreconditioner |
                                                                   Eigen::ComputeFullU |
                                                                   Eigen::ComputeFullV);

    U_matrix = singular_value_decomposition.matrixU();
    V_matrix = singular_value_decomposition.matrixV();
    singular_values = singular_value_decomposition.singularValues();

    rotation_matrix = V_matrix * U_matrix.transpose();
    determinant_rotation = rotation_matrix.determinant();

    if ( m_are_almost_equal(determinant_rotation, 1.) ) {
      return rotation_matrix;
    }
    else if ( m_are_almost_equal(determinant_rotation, -1.) && m_are_almost_equal(singular_values(2), 0.) ) {
      V_matrix.col(2) = -V_matrix.col(2);
      rotation_matrix = V_matrix * U_matrix.transpose();
      return rotation_matrix;
    }
    else {
      std::string message = std::string("SVD method cannot find the rotation.");
      throw std::length_error(message);
    }

  }

  /// Method to compute the rotation matrix which connect the two set of 3D points using quaternions.
  Eigen::Matrix3d QuaternionMethod::find_rotation(Eigen::Matrix3d H_matrix) {}

  /// Method to build the 4x4 matrix N out of the 3x3 matrix H.
  Eigen::Matrix4d QuaternionMethod::m_build_matrix_N(Eigen::Matrix3d H_matrix) {}

  /// Method to build the rotation matrix out of a quaternion (4D vector).
  Eigen::Matrix3d QuaternionMethod::m_build_rotation(Eigen::Vector4d quaternion) {}
  

} // end namespace