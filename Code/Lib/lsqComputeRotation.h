/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef lsqComputeRotation_h
#define lsqComputeRotation_h

#include "lsqWin32ExportHeader.h"
#include <cmath>
#include <string>
#include <exception>
#include <Eigen/Dense>
#include <Eigen/SVD>

/**
* \file lsqComputeRotation.h
* \brief Various Utilities.
* \ingroup utilities
*/
namespace lsq
{

/**
* \brief The class ComputeRotation is the interface we use for our strategy pattern.
* 
* This abstract class is the interface we use in order to compute the rotation connecting two sets of points.
* The strategies we implement to compute the rotation inherit from this class.
*/
LEASTSQUARESPACKAGE_WINEXPORT class ComputeRotation {

  public:

  	/// Abstract method to compute the rotation matrix which connect the two set of 3D points.
    /**
    * \param H_matrix an auxiliary matrix computed by the LeastSquare3D class.
    * \return The rotation matrix.
    * \sa SVDMethod, QuaternionMethod.
    *
    */
    virtual Eigen::Matrix3d find_rotation(Eigen::Matrix3d H_matrix) =0;

};

/**
* \brief The class SVDMethod implements the method suggested by K.S. Arun et al.
* 
* This class compute the rotation matrix using the singular value decomposition of the auxiliary matrix H.
* This method is described in K.S. Arun et al., IEEE PAMI-9, 698-700 (1987).
*/
LEASTSQUARESPACKAGE_WINEXPORT class SVDMethod : public ComputeRotation {

  public:

  	/// Method to compute the rotation matrix which connect the two set of 3D points.
    /**
    * \param H_matrix an auxiliary matrix computed by the LeastSquare3D class.
    * \return The rotation matrix.
    * \sa QuaternionMethod.
    *
    * This method compute the rotation by computing the SVD of the matrix H.
    * Can throw an exception if the matrix cannot be found.
    */
    Eigen::Matrix3d find_rotation(Eigen::Matrix3d H_matrix);

  private:

  	const double m_precision = 1.e-5; /**< constant precision up to which two doubles are equal. */

    /// Private method to check whether two numbers are approximately equal.
  	bool m_are_almost_equal(double a, double b);

};

/**
* \brief The class QuaternionMethod implements the method suggested by Horn.
* 
* This class compute the rotation matrix using the quaternion representation of the rotation, starting from the auxiliary matrix H.
* This method is described in B.K.P. Horn, J. Opt. Soc. Am. A 4, 629-642 (1987).
*/
LEASTSQUARESPACKAGE_WINEXPORT class QuaternionMethod : public ComputeRotation {

  public:

    Eigen::Matrix3d find_rotation(Eigen::Matrix3d H_matrix) {};

};

} // end namespace

#endif
