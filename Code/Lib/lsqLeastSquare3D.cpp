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

  /// The class constructor.
  LeastSquare3D::LeastSquare3D() {
  	m_first_centroid = {0.,0.,0.};
  	m_second_centroid = {0.,0.,0.};
  }

  /// Method to add a 3D point into the first vector.
  void LeastSquare3D::add_point_first_vector(const Eigen::Array3d & point) {

  	m_first_point_vector.push_back(point);

  }

  /// Method to add a 3D point into the second vector.
  void LeastSquare3D::add_point_second_vector(const Eigen::Array3d & point) {

  	m_second_point_vector.push_back(point);

  }

  /// Private method to pop back the last 3D point of a vector.
  Eigen::Array3d LeastSquare3D::m_pop_point_vector(std::vector<Eigen::Array3d>* point_vector) {

  	if( point_vector->empty() ) {
  	  std::string message = std::string("Cannot remove any element.");
      throw std::length_error(message);
    }
    else {
      Eigen::Array3d last_element;
      last_element = point_vector->back();
      point_vector->pop_back();
      return last_element;
    }

  }

  /// Method to pop back the last 3D point of the first vector.
  Eigen::Array3d LeastSquare3D::pop_point_first_vector() {

    try {
      return m_pop_point_vector(&m_first_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Fisrt vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to pop back the last 3D point of the second vector.
  Eigen::Array3d LeastSquare3D::pop_point_second_vector() {

    try {
      return m_pop_point_vector(&m_second_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Second vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to compute the centroid of the a vector of 3D points.
  Eigen::Array3d LeastSquare3D::m_compute_centroid(const std::vector<Eigen::Array3d>* point_vector) {

    if( point_vector->empty() ) {
  	  std::string message = std::string("Cannot compute the centroid.");
      throw std::length_error(message);
    }
    else {
      Eigen::Array3d centroid = {0.,0.,0.};
      auto vector_iterator = point_vector->begin();
      for ( ; vector_iterator != point_vector->end(); ++vector_iterator) {
      	centroid += *vector_iterator;
      }
      centroid = centroid/double(point_vector->size());
      return centroid;
    }

  }

  /// Method to compute the centroid of the first vector of 3D points.
  void LeastSquare3D::centroid_first_vector() {

    try {
      m_first_centroid = m_compute_centroid(&m_first_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("First vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to compute the centroid of the second vector of 3D points.
  void LeastSquare3D::centroid_second_vector() {

  	try {
      m_second_centroid = m_compute_centroid(&m_second_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Second vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to get the centroid of the first vector.
  Eigen::Array3d LeastSquare3D::get_centroid_first_vector() {

  	return m_first_centroid;

  }

  /// Method to get the centroid of the second vector.
  Eigen::Array3d LeastSquare3D::get_centroid_second_vector() {

  	return m_second_centroid;

  }

} // end namespace