/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "lsqCatchMain.h"
#include "lsqComputeRotation.h"
#include <Eigen/Dense>
#include <iostream>

TEST_CASE( "Compute the rotation with the SVD methods", "[set_strategy]" ) {

  std::unique_ptr<lsq::ComputeRotation> algorithm( new lsq::SVDMethod() );

  Eigen::Matrix3d input_H_matrix = Eigen::Matrix3d::Zero(3, 3);
  Eigen::Matrix3d expected_rotation_matrix = Eigen::Matrix3d::Zero(3, 3);
  Eigen::Matrix3d obtained_rotation_matrix;

  SECTION( "The method fails because the rotation has det = -1 and the singular values are non-negative." ) {
    input_H_matrix(0,2) = -0.2;
    input_H_matrix(1,1) = -0.5;
    input_H_matrix(2,0) = 1.;

    REQUIRE_THROWS( obtained_rotation_matrix = algorithm->find_rotation(input_H_matrix) );
  }

  SECTION( "The method succeeds in finding the rotation, but has to modify the rotation to avoid a reflection." ) {
    input_H_matrix(1,1) = -0.3;
    input_H_matrix(2,0) = -1.;

    expected_rotation_matrix(0,2) = -1.;
    expected_rotation_matrix(1,1) = -1.;
    expected_rotation_matrix(2,0) = -1.;

    obtained_rotation_matrix = algorithm->find_rotation(input_H_matrix);

    REQUIRE( expected_rotation_matrix == obtained_rotation_matrix );

  }

  SECTION( "The method succeeds in finding the rotation." ) {
    input_H_matrix(1,1) = -0.3;
    input_H_matrix(2,0) = 1.;

    expected_rotation_matrix(0,2) = 1.;
    expected_rotation_matrix(1,1) = -1.;
    expected_rotation_matrix(2,0) = 1.;

    obtained_rotation_matrix = algorithm->find_rotation(input_H_matrix);

    REQUIRE( expected_rotation_matrix == obtained_rotation_matrix );

  }

}
