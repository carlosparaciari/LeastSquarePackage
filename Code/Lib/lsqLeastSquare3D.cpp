/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "lsqLeastSquare3D.h"

namespace lsq {

  /// Method to add a 3D point into the first vector.
  void LeastSquare3D::add_point_first_vector(const Eigen::Array3d & point) {

  	m_first_point_vector.push_back(point);

  }

  /// Method to pop back the last 3D point of the first vector.
  Eigen::Array3d LeastSquare3D::pop_point_first_vector() {

    if( m_first_point_vector.empty() ) {
      throw std::length_error(m_lenght_error_message);
    }
    else {
      Eigen::Array3d last_element;
      last_element = m_first_point_vector.back();
      m_first_point_vector.pop_back();
      return last_element;
    }

  }

  /// Method to add a 3D point into the second vector.
  void LeastSquare3D::add_point_second_vector(const Eigen::Array3d & point) {

  	m_second_point_vector.push_back(point);

  }

  /// Method to pop back the last 3D point of the second vector.
  Eigen::Array3d LeastSquare3D::pop_point_second_vector() {

    if( m_second_point_vector.empty() ) {
      throw std::length_error(m_lenght_error_message);
    }
    else {
      Eigen::Array3d last_element;
      last_element = m_second_point_vector.back();
      m_second_point_vector.pop_back();
      return last_element;
    }

  }


} // end namespace