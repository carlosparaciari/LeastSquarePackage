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
* This class is the interface we use in order to compute the rotation connecting two sets of points.
* The strategies we implement to compute the rotation inherit from this class.
*/
LEASTSQUARESPACKAGE_WINEXPORT class ComputeRotation {

	ComputeRotation(){};
	~ComputeRotation(){};

};

} // end namespace

#endif
