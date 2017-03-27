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
  	m_first_centroid = Eigen::Vector3d::Zero(3);
  	m_second_centroid = Eigen::Vector3d::Zero(3);
  	m_H_matrix = Eigen::Matrix3d::Zero(3, 3);
  	m_rotation_matrix = Eigen::Matrix3d::Zero(3, 3);
  	m_translation_vector = Eigen::Vector3d::Zero(3);
  }

  /// Method to add a 3D point into the first vector.
  void LeastSquare3D::add_point_first_vector(Eigen::Vector3d & point) {

  	m_first_point_vector.push_back(point);

  }

  /// Method to add a 3D point into the second vector.
  void LeastSquare3D::add_point_second_vector(Eigen::Vector3d & point) {

  	m_second_point_vector.push_back(point);

  }

  /// Private method to pop back the last 3D point of a vector.
  Eigen::Vector3d LeastSquare3D::m_pop_point_vector(std::vector<Eigen::Vector3d> & point_vector) {

  	if( point_vector.empty() ) {
  	  std::string message = std::string("Cannot remove any element.");
      throw std::length_error(message);
    }
    else {
      Eigen::Vector3d last_element;
      last_element = point_vector.back();
      point_vector.pop_back();
      return last_element;
    }

  }

  /// Method to pop back the last 3D point of the first vector.
  Eigen::Vector3d LeastSquare3D::pop_point_first_vector() {

    try {
      return m_pop_point_vector(m_first_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Fisrt vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to pop back the last 3D point of the second vector.
  Eigen::Vector3d LeastSquare3D::pop_point_second_vector() {

    try {
      return m_pop_point_vector(m_second_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Second vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to compute the centroid of the a vector of 3D points.
  Eigen::Vector3d LeastSquare3D::m_compute_centroid(const std::vector<Eigen::Vector3d> & point_vector) {

    if( point_vector.empty() ) {
  	  std::string message = std::string("Cannot compute the centroid.");
      throw std::length_error(message);
    }
    else {
      Eigen::Vector3d centroid = {0.,0.,0.};
      auto vector_iterator = point_vector.begin();
      for ( ; vector_iterator != point_vector.end(); ++vector_iterator) {
      	centroid += *vector_iterator;
      }
      centroid = centroid/double(point_vector.size());
      return centroid;
    }

  }

  /// Method to compute the centroid of the first vector of 3D points.
  void LeastSquare3D::centroid_first_vector() {

    try {
      m_first_centroid = m_compute_centroid(m_first_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("First vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to compute the centroid of the second vector of 3D points.
  void LeastSquare3D::centroid_second_vector() {

  	try {
      m_second_centroid = m_compute_centroid(m_second_point_vector);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Second vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to get the centroid of the first vector.
  Eigen::Vector3d LeastSquare3D::get_centroid_first_vector() {

  	return m_first_centroid;

  }

  /// Method to get the centroid of the second vector.
  Eigen::Vector3d LeastSquare3D::get_centroid_second_vector() {

  	return m_second_centroid;

  }

  /// Private method to update a vector around its centroid.
  void LeastSquare3D::m_update_points_around_centroid(std::vector<Eigen::Vector3d> & point_vector, const Eigen::Vector3d & point) {

    if( point_vector.empty() ) {
  	  std::string message = std::string("Cannot update the set of points around the centroid.");
      throw std::length_error(message);
    }
    else {
      for ( int i = 0; i < point_vector.size(); ++i)
        point_vector[i] -= point;
    }

  }

  /// Method to update the first vector of points around its centroid.
  void LeastSquare3D::update_first_points_around_centroid() {

  	try {
      m_update_points_around_centroid(m_first_point_vector, m_first_centroid);
    }
    catch (std::length_error &error) {
      std::string message = std::string("First vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to update the second vector of points around its centroid.
  void LeastSquare3D::update_second_points_around_centroid() {

  	try {
      m_update_points_around_centroid(m_second_point_vector, m_second_centroid);
    }
    catch (std::length_error &error) {
      std::string message = std::string("Second vector is empty. ") + error.what();
      throw std::length_error(message);
    }

  }

  /// Method to compute an auxiliar matrix needed for computing the rotation.
  void LeastSquare3D::compute_H_matrix() {

  	if( m_first_point_vector.empty() ) {
  	  std::string message = std::string("First vector is empty. Cannot compute the matrix H.");
      throw std::length_error(message);
    }

    if( m_second_point_vector.empty() ) {
  	  std::string message = std::string("Second vector is empty. Cannot compute the matrix H.");
      throw std::length_error(message);
    }

    int end_element;

    if( m_first_point_vector.size() > m_second_point_vector.size() ){
      end_element = m_second_point_vector.size();
    }
    else {
      end_element = m_first_point_vector.size();
    }

    m_H_matrix = Eigen::Matrix3d::Zero(3, 3);

    for( int i = 0 ; i < end_element; ++i )
      m_H_matrix += (m_first_point_vector[i]) * (m_second_point_vector[i]).transpose();

  }

  /// Method to get the auxiliary matrix H.
  Eigen::Matrix3d LeastSquare3D::get_H_matrix() {

    return m_H_matrix;

  }

  /// Method to set the algorithm to compute the rotation.
  void LeastSquare3D::set_rotation_strategy(std::unique_ptr<ComputeRotation> computing_algorithm) {

  	m_method_for_rotation = std::move(computing_algorithm);

  }

  /// Method to check whether the algorithm to compute the rotation has been setted.
  bool LeastSquare3D::is_rotation_strategy() {

  	if (m_method_for_rotation)
  	  return true;
  	else
  	  return false;

  }

  /// Method to compute the rotation matrix connecting the first set of points to the second.
  void LeastSquare3D::compute_rotation_matrix() {}

  /// Method to compute the translation vector connecting the first set of points to the second.
  void LeastSquare3D::compute_translation_vector() {

  	m_translation_vector = m_second_centroid - m_rotation_matrix * m_first_centroid;
  }

  /// Method to get the rotation matrix connecting the two set of points.
  Eigen::Matrix3d LeastSquare3D::get_rotation_matrix() {

    return m_rotation_matrix;

  }

  /// Method to get the translation vector connecting the two set of points.
  Eigen::Vector3d LeastSquare3D::get_translation_vector() {

  	return m_translation_vector;

  }

} // end namespace
