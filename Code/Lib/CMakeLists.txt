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

set(LEASTSQUARESPACKAGE_SRCS
  lsqBasicTypes.cpp
  lsqMyFunctions.cpp
  lsqLeastSquare3D.cpp
)

add_library(leastsquarespackage ${LEASTSQUARESPACKAGE_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DLEASTSQUARESPACKAGE_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(leastsquarespackage PRIVATE ${ALL_THIRD_PARTY_LIBRARIES})
