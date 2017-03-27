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

  /// Method to compute the rotation matrix which connect the two set of 3D points using SVD.
  Eigen::Matrix3d SVDMethod::find_rotation(Eigen::Matrix3d H_matrix) {};

} // end namespace