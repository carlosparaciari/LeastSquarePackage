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

set(DATA_DIR ${CMAKE_SOURCE_DIR}/Testing/Data)
set(TMP_DIR ${CMAKE_BINARY_DIR}/Testing/Temporary)

set(TEST_CASES
  lsqReadFileTest
  lsqLeastSquare3DTest
  lsqComputeRotationTest
)

foreach(_test_case ${TEST_CASES})
  add_executable(${_test_case} ${_test_case}.cpp lsqCatchMain.cpp)
  target_link_libraries(${_test_case} ${ALL_LIBRARIES})
endforeach()

# Adding tests is done outside the above loop,
# as each test will need different arguments.

##################################################################################################
# Dont forget its:  add_test(<test name (globally unique) > <exe name> <argument1> <argument2>
##################################################################################################

add_test(Read
  ${EXECUTABLE_OUTPUT_PATH}/lsqReadFileTest
  ${DATA_DIR}/first_set_points_test.txt
  ${DATA_DIR}/second_set_points_test.txt
  ${DATA_DIR}/rotation_test.txt
  ${DATA_DIR}/translation_test.txt)
add_test(LS3D ${EXECUTABLE_OUTPUT_PATH}/lsqLeastSquare3DTest)
add_test(Rotation ${EXECUTABLE_OUTPUT_PATH}/lsqComputeRotationTest)
