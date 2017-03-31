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
  Eigen::Matrix4d QuaternionMethod::m_build_matrix_N(Eigen::Matrix3d H_matrix) {

    Eigen::Matrix4d N_matrix = Eigen::Matrix4d::Zero(4, 4);

    N_matrix(0,0) = H_matrix(0,0) + H_matrix(1,1) + H_matrix(2,2);
    N_matrix(0,1) = N_matrix(1,0) = H_matrix(1,2) - H_matrix(2,1);
    N_matrix(0,2) = N_matrix(2,0) = H_matrix(2,0) - H_matrix(0,2);
    N_matrix(0,3) = N_matrix(3,0) = H_matrix(0,1) - H_matrix(1,0);

    N_matrix(1,1) = H_matrix(0,0) - H_matrix(1,1) - H_matrix(2,2);
    N_matrix(1,2) = N_matrix(2,1) = H_matrix(0,1) + H_matrix(1,0);
    N_matrix(1,3) = N_matrix(3,1) = H_matrix(2,0) + H_matrix(0,2);

    N_matrix(2,2) = -H_matrix(0,0) + H_matrix(1,1) - H_matrix(2,2);
    N_matrix(2,3) = N_matrix(3,2) = H_matrix(1,2) + H_matrix(2,1);

    N_matrix(3,3) = -H_matrix(0,0) - H_matrix(1,1) + H_matrix(2,2);

    return N_matrix;

  }

  /// Method to build the rotation matrix out of a quaternion (4D vector).
  Eigen::Matrix3d QuaternionMethod::m_build_rotation(Eigen::Vector4d quaternion) {

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero(3, 3);

    rotation(0,0) = pow(quaternion(0),2.) + pow(quaternion(1),2.) - pow(quaternion(2),2.) - pow(quaternion(3),2.);
    rotation(0,1) = 2 * ( quaternion(1) * quaternion(2) - quaternion(0) * quaternion(3) );
    rotation(0,2) = 2 * ( quaternion(1) * quaternion(3) + quaternion(0) * quaternion(2) );

    rotation(1,0) = 2 * ( quaternion(1) * quaternion(2) + quaternion(0) * quaternion(3) );
    rotation(1,1) = pow(quaternion(0),2.) - pow(quaternion(1),2.) + pow(quaternion(2),2.) - pow(quaternion(3),2.);
    rotation(1,2) = 2 * ( quaternion(2) * quaternion(3) - quaternion(0) * quaternion(1) );

    rotation(2,0) = 2 * ( quaternion(1) * quaternion(3) - quaternion(0) * quaternion(2) );
    rotation(2,1) = 2 * ( quaternion(2) * quaternion(3) + quaternion(0) * quaternion(1) );
    rotation(2,2) = pow(quaternion(0),2.) - pow(quaternion(1),2.) - pow(quaternion(2),2.) + pow(quaternion(3),2.);

    return rotation;

  }
  

} // end namespace