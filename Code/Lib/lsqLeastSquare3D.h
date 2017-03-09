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
  	LeastSquare3D(){};

  	/// The class destructor.
  	~LeastSquare3D(){};

  	/// Method to add a 3D point into the first vector.
    /**
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa add_point_second_vector(), m_add_point_to_vector().
    *
    * Add a point at the end of the vector m_first_point_vector.
    */
  	void add_point_first_vector(const Eigen::Array3d point);

  	/// Method to add a 3D point into the second vector.
    /**
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa add_point_first_vector(), m_add_point_to_vector().
    *
    * Add a point at the end of the vector m_second_point_vector.
    */
  	void add_point_second_vector(const Eigen::Array3d point);

  	/// Method to set the algorithm to compute the rotation.
    /**
    * \param computing_algorithm an instance of the class ComputeRotation, specifies the strategy to use
	*
	* Set the algorithm used by the class to compute the rotation connecting the two points.
	* The algorithm can change, and the strategy pattern is used to provide two distinct algorithms.
    */
  	void set_rotation_strategy(ComputeRotation* computing_algorithm);

  	/// Method to compute the centroid of the first vector of 3D points.
    /**
    * \sa m_compute_centroid(), centroid_second_vector().
    *
    * The centroid of the first set of points \f$\{ \text{p}_i \}_{i=1}^{N}\f$ is computed as
    * \f$\text{p} = \frac{1}{N} \sum_{i=1}^{N} \text{p}_i\f$.
    */
  	void centroid_first_vector();

  	/// Method to compute the centroid of the second vector of 3D points.
    /**
    * \sa m_compute_centroid(), centroid_first_vector().
    *
    * The centroid of the second set of points \f$\{ \text{p}'_i \}_{i=1}^{N}\f$ is computed as
    * \f$\text{p}' = \frac{1}{N} \sum_{i=1}^{N} \text{p}'_i\f$.
    */
  	void centroid_second_vector();

  	/// Method to update the first vector of points around its centroid.
    /**
    * \sa m_update_points_around_centroid().
    *
    * The first set of points get updated from \f$\{ \text{p}_i \}_{i=1}^{N}\f$ into \f$\{ \text{p}_i - \text{p} \}_{i=1}^{N}\f$,
    * where \f$\text{p}\f$ is the centroid of the set.
    */
  	void update_first_points_around_centroid();

  	/// Method to update the second vector of points around its centroid.
    /**
    * \sa m_update_points_around_centroid().
    *
    * The second set of points get updated from \f$\{ \text{p}'_i \}_{i=1}^{N}\f$ into \f$\{ \text{p}'_i - \text{p}' \}_{i=1}^{N}\f$,
    * where \f$\text{p}'\f$ is the centroid of the set.
    */
  	void update_second_points_around_centroid();

  	/// Method to compute an auxiliar matrix needed for computing the rotation.
    /**
    * The method uses both the first and second set of points to compute this matrix.
    */
  	void compute_H_matrix();

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
  	Eigen::Array3d get_translation_vector();

  private:

  	std::vector<Eigen::Array3d> m_first_point_vector; /**< Vector containing the first set of 3D points. */
  	std::vector<Eigen::Array3d> m_second_point_vector; /**< Vector containing the second set of 3D points. */

  	ComputeRotation* m_method_for_rotation; /**< Instance of the strategy class defining the algorithm to compute the rotation matrix. */

  	Eigen::Array3d m_first_centroid; /**< Centroid of the first set of points. */
  	Eigen::Array3d m_second_centroid; /**< Centroid of the second set of points. */

  	Eigen::Matrix3d m_H_matrix; /**< Auxiliary matrix. We need it for the algorithms computing the rotation matrix */

  	Eigen::Matrix3d m_rotation_matrix; /**< Rotation matrix connecting the two sets of points. */
  	Eigen::Array3d m_translation_vector; /**< Translation vector connecting the two sets of points. */

  	/// Method to add a 3D point into a vector.
    /**
    * \param point_vector a vector of 3D points.
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa add_point_first_vector(), add_point_second_vector().
    *
    * Add a point at the end of the vector point_vector.
    */
  	void m_add_point_to_vector(std::vector<Eigen::Array3d> point_vector, const Eigen::Array3d point);

  	/// Method to compute the centroid of the a vector of 3D points.
    /**
    * \param point_vector a vector of 3D points.
    * \sa centroid_first_vector(), centroid_second_vector().
    *
    * The centroid of the a set of points is computed.
    */
  	void m_compute_centroid(const std::vector<Eigen::Array3d> point_vector);

  	/// Method to update a vector around its centroid.
    /**
    * \param point_vector a vector of 3D points.
    * \param point a 3D point defined by coordinates x,y,z.
    * \sa update_first_points_around_centroid(), update_second_points_around_centroid().
    *
    * Update a set of 3D points so that their centroid is zero.
    */
  	void m_update_points_around_centroid(std::vector<Eigen::Array3d> point_vector, const Eigen::Array3d point);

};

} // end namespace

#endif
