/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef lsqLeastSquare3D_h
#define lsqLeastSquare3D_h

#include "lsqWin32ExportHeader.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <exception>
#include <memory>
#include "lsqComputeRotation.h"

/**
* \file lsqLeastSquare3D.h
* \brief Class for extimating the rotation and translation connecting two sets of 3D points.
* \ingroup utilities
*/
namespace lsq
{

/**
* \brief The class LeastSquare3D provides methods to compute the rotation matrix and the translation vector connecting two set of 3D points.
*
* The class provides method for computing the transformation connecting two sets of 3D points. Specifically, LeastSquare3D can compute the
* rotation and translation which map the first set of points into the other. These quantities are evaluated using the least squares method,
* and two algorithms to compute the rotation are implemented in this package.
*/
LEASTSQUARESPACKAGE_WINEXPORT class LeastSquare3D {

  public:

	  /// The class constructor.
  	LeastSquare3D();

  	/// The class destructor.
  	~LeastSquare3D() {};

  	/// Method to add a 3D point into the first vector.
    /**
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa add_point_second_vector(), pop_point_first_vector().
    *
    * Add a point at the end of the vector m_first_point_vector.
    */
  	void add_point_first_vector(Eigen::Vector3d & point);

    /// Method to pop back the last 3D point of the first vector.
    /**
    * \return the last point in the first vector.
    * \sa m_pop_point_vector(), pop_point_second_vector().
    *
    * Erase the last element of m_first_point_vector, and return it.
    */
    Eigen::Vector3d pop_point_first_vector();

  	/// Method to add a 3D point into the second vector.
    /**
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa add_point_first_vector(), m_add_point_to_vector().
    *
    * Add a point at the end of the vector m_second_point_vector.
    */
  	void add_point_second_vector(Eigen::Vector3d & point);

    /// Method to pop back the last 3D point of the second vector.
    /**
    * \return the last point in the second vector.
    * \sa m_pop_point_vector(), pop_point_first_vector().
    *
    * Erase the last element of m_second_point_vector, and return it.
    */
    Eigen::Vector3d pop_point_second_vector();

    /// Method to check if the two vectors have the same number of elements.
    /**
    * \return a boolean, true if m_first_point_vector has same size of m_second_point_vector.
    */
    bool same_number_of_points();

  	/// Method to compute the centroid of the first vector of 3D points.
    /**
    * \sa m_compute_centroid(), centroid_second_vector().
    *
    * The centroid of the first set of points \f$\{ p_i \}_{i=1}^{N}\f$ is computed as
    * \f$p = \frac{1}{N} \sum_{i=1}^{N} p_i\f$.
    */
  	void centroid_first_vector();

    /// Method to get the centroid of the first vector.
    /**
    * \return the the centroid of the first vector.
    * \sa centroid_first_vector().
    *
    * Returns the centroid of the first vector if it has been evaluated, or an empty 3D array otherwise.
    */
    Eigen::Vector3d get_centroid_first_vector();

  	/// Method to compute the centroid of the second vector of 3D points.
    /**
    * \sa m_compute_centroid(), centroid_first_vector().
    *
    * The centroid of the second set of points \f$\{ p'_i \}_{i=1}^{N}\f$ is computed as
    * \f$p' = \frac{1}{N} \sum_{i=1}^{N} p'_i\f$.
    */
  	void centroid_second_vector();

  	/// Method to get the centroid of the second vector.
    /**
    * \return the the centroid of the second vector.
    * \sa centroid_second_vector().
    *
    * Returns the centroid of the second vector if it has been evaluated, or an empty 3D array otherwise.
    */
    Eigen::Vector3d get_centroid_second_vector();

  	/// Method to update the first vector of points around its centroid.
    /**
    * \sa m_update_points_around_centroid().
    *
    * The first set of points get updated from \f$\{ p_i \}_{i=1}^{N}\f$ into \f$\{ p_i - p \}_{i=1}^{N}\f$,
    * where \f$p\f$ is the centroid of the set.
    */
  	void update_first_points_around_centroid();

  	/// Method to update the second vector of points around its centroid.
    /**
    * \sa m_update_points_around_centroid().
    *
    * The second set of points get updated from \f$\{ p'_i \}_{i=1}^{N}\f$ into \f$\{ p'_i - p' \}_{i=1}^{N}\f$,
    * where \f$p'\f$ is the centroid of the set.
    */
  	void update_second_points_around_centroid();

  	/// Method to compute an auxiliary matrix needed for computing the rotation.
    /**
    * The method uses both the first and second set of points to compute this matrix.
    */
  	void compute_H_matrix();

  	/// Method to get the auxiliary matrix H.
    /**
    * \return the matrix H.
    * \sa compute_H_matrix().
    */
  	Eigen::Matrix3d get_H_matrix();

  	/// Method to set the algorithm to compute the rotation.
    /**
    * \param computing_algorithm an instance of the class ComputeRotation, specifies the strategy to use
	  *
		* Set the algorithm used by the class to compute the rotation connecting the two points.
		* The algorithm can change, and the strategy pattern is used to provide two distinct algorithms.
    */
  	void set_rotation_strategy(std::unique_ptr<ComputeRotation> computing_algorithm);

  	/// Method to check whether the algorithm to compute the rotation has been setted.
    /**
    * \return boolean value
    *
		* Return true if the strategy has been selected. false otherwhise.
    */
  	bool is_rotation_strategy();

  	/// Method to compute the rotation matrix connecting the first set of points to the second.
    /**
    * \sa set_rotation_strategy().
    * The algorithm used to compute the rotation matrix depends on which strategy we set.
    */
  	void compute_rotation_matrix();

  	/// Method to compute the translation vector connecting the first set of points to the second.
    /**
    * Once the rotation has been found, the translation vector connecting the two set of vectors is
    * easily obtained by subtracting the two set of points and averaging.
    */  	
  	void compute_translation_vector();

  	/// Method to get the rotation matrix connecting the two set of points.
    /**
    * \return the rotation matrix.
    * \sa compute_rotation_matrix().
    */
  	Eigen::Matrix3d get_rotation_matrix();

  	/// Method to get the translation vector connecting the two set of points.
    /**
    * \return the translation vector.
    * \sa compute_translation_vector().
    */
  	Eigen::Vector3d get_translation_vector();

  private:

  	std::vector<Eigen::Vector3d> m_first_point_vector; /**< Vector containing the first set of 3D points. */
  	std::vector<Eigen::Vector3d> m_second_point_vector; /**< Vector containing the second set of 3D points. */

  	std::unique_ptr<ComputeRotation> m_method_for_rotation; /**< Instance of the strategy class defining the algorithm to compute the rotation matrix. */

  	Eigen::Vector3d m_first_centroid; /**< Centroid of the first set of points. */
  	Eigen::Vector3d m_second_centroid; /**< Centroid of the second set of points. */

  	Eigen::Matrix3d m_H_matrix; /**< Auxiliary matrix. We need it for the algorithms computing the rotation matrix */

  	Eigen::Matrix3d m_rotation_matrix; /**< Rotation matrix connecting the two sets of points. */
  	Eigen::Vector3d m_translation_vector; /**< Translation vector connecting the two sets of points. */

  	/// Private method to pop back the last 3D point of a vector.
    /**
    * \param point_vector a vector of 3D points.
    * \return the last point in the point_vector.
    * \sa pop_point_first_vector(), pop_point_second_vector().
    *
    * Erase the last element of point_vector, and return it.
    */
    Eigen::Vector3d m_pop_point_vector(std::vector<Eigen::Vector3d> & point_vector);

  	/// Private method to compute the centroid of the a vector of 3D points.
    /**
    * \param point_vector a vector of 3D points.
    * \param centroid a 3D point where we save the value of the centroid.
    * \sa centroid_first_vector(), centroid_second_vector().
    *
    * The centroid of the a set of points is computed.
    */
  	Eigen::Vector3d m_compute_centroid(const std::vector<Eigen::Vector3d> & point_vector);

  	/// Private method to update a vector around its centroid.
    /**
    * \param point_vector a vector of 3D points.
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa update_first_points_around_centroid(), update_second_points_around_centroid().
    *
    * Update a set of 3D points so that their centroid is zero.
    */
  	void m_update_points_around_centroid(std::vector<Eigen::Vector3d> & point_vector, const Eigen::Vector3d & point);

};

} // end namespace

#endif
