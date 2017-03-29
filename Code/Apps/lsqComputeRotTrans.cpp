/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <boost/program_options.hpp>
#include "lsqLeastSquare3D.h"
#include <iostream>
#include <cstdlib>

/**
* \file lsqComputeRotTrans.cpp
* \brief End-user application for computing the rotation and translation connecting two sets of 3D points.
* \ingroup applications
*
* This command line application can read from two separate files, each of them containing the coordinates of many 3D points,
* and compute the rotation and translation connecting the two sets. The user can choose between two different methods to
* compute the rotation, and the methods can be selected with a specific option (see help). The two methods are based, respectively,
* on the singular value decomposition of a matrix obtained from the sets of points (K.S. Arun et al., IEEE PAMI-9, 698-700, 1987),
* and on the quaternionic representation of matrices (B.K.P. Horn, J. Opt. Soc. Am. A 4, 629-642, 1987). Rotation and translation
* are saved in file "rotation.dat" and "translation.dat" respectively.
*/
int main(int argc, char** argv) {

}