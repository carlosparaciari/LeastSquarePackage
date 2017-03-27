#/*============================================================================
#
#  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

set(LEASTSQUARESPACKAGE_APPS
  lsqMyFirstApp
  lsqComputeRotTrans
)
foreach(app ${LEASTSQUARESPACKAGE_APPS})
  add_executable(${app} ${app}.cpp)
  target_link_libraries(${app} leastsquarespackage ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()
