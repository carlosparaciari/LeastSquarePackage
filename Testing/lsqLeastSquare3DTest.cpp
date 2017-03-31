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
#include "lsqLeastSquare3D.h"
#include "lsqComputeRotation.h"
#include <Eigen/Dense>
#include <iostream>

TEST_CASE( "Throw exception when we pop an element from empty array", "[add_remove_points]" ) {

  lsq::LeastSquare3D add_example;
  Eigen::Vector3d point_3D;

  SECTION( "Pop an element from the first vector" ) {
    REQUIRE_THROWS( point_3D = add_example.pop_point_first_vector() );
  }
  SECTION( "Pop an element from the second vector" ) {
    REQUIRE_THROWS( point_3D = add_example.pop_point_second_vector() );
  }

}

TEST_CASE( "Add 3D point to first and second set", "[add_remove_points]" ) {

  lsq::LeastSquare3D add_example;
  Eigen::Vector3d point_3D_in = {1,2,3};
  Eigen::Vector3d point_3D_out;

  SECTION( "Add point to first vector and pop it out" ) {
    add_example.add_point_first_vector(point_3D_in);
    point_3D_out = add_example.pop_point_first_vector();
    REQUIRE( point_3D_out == point_3D_in );
  }

  SECTION( "Add point to second vector and pop it out" ) {
    add_example.add_point_second_vector(point_3D_in);
    point_3D_out = add_example.pop_point_second_vector();
    REQUIRE( point_3D_out == point_3D_in );
  }

  SECTION( "Check if the two vectors have same size when they are empty" ) {
    REQUIRE( add_example.same_number_of_points() == true );
  }

  SECTION( "Check if the two vectors have same size when it is true" ) {
    add_example.add_point_first_vector(point_3D_in);
    add_example.add_point_second_vector(point_3D_in);
    REQUIRE( add_example.same_number_of_points() == true );
  }

  SECTION( "Check if the two vectors have same size when it is false" ) {
    add_example.add_point_first_vector(point_3D_in);
    add_example.add_point_second_vector(point_3D_in);
    add_example.add_point_second_vector(point_3D_in);
    REQUIRE( add_example.same_number_of_points() == false );
  }

}

TEST_CASE( "Compute centroid for the sets of points and update the set", "[centroids]" ) {

  lsq::LeastSquare3D centroid_example;
  Eigen::Vector3d point_3D_in;

  point_3D_in = {1.,0.,0.};
  centroid_example.add_point_first_vector(point_3D_in);
  point_3D_in = {0.,1.,0.};
  centroid_example.add_point_first_vector(point_3D_in);
  point_3D_in = {0.,0.,1.};
  centroid_example.add_point_first_vector(point_3D_in);

  Eigen::Vector3d expected_centroid = {0.,0.,0.};
  Eigen::Vector3d obtained_centroid;

  SECTION( "Get first centroid without having computed it" ) {
    obtained_centroid = centroid_example.get_centroid_first_vector();
    REQUIRE( expected_centroid == obtained_centroid );
  }

  SECTION( "Get second centroid without having computed it" ) {
    obtained_centroid = centroid_example.get_centroid_second_vector();
    REQUIRE( expected_centroid == obtained_centroid );
  }

  expected_centroid = {1./3., 1./3., 1./3.};

  SECTION( "Compute first centroid and check it" ) {
    centroid_example.centroid_first_vector();
    obtained_centroid = centroid_example.get_centroid_first_vector();
    REQUIRE( expected_centroid == obtained_centroid );
  }

  SECTION( "Compute again the first centroid and check that it does not change" ) {
    centroid_example.centroid_first_vector();
    obtained_centroid = centroid_example.get_centroid_first_vector();
    REQUIRE( expected_centroid == obtained_centroid );
  }

  SECTION( "Compute second centroid and get an error (the second vector is empty)" ) {
    REQUIRE_THROWS( centroid_example.centroid_second_vector() );
  }
  
  Eigen::Vector3d expected_point;
  Eigen::Vector3d obtained_point;
  std::vector<Eigen::Vector3d> expected_updated_first_vector;

  expected_updated_first_vector.push_back( Eigen::Vector3d( {0.-1./3., 0.-1./3., 1.-1./3.} ) );
  expected_updated_first_vector.push_back( Eigen::Vector3d( {0.-1./3., 1.-1./3., 0.-1./3.} ) );
  expected_updated_first_vector.push_back( Eigen::Vector3d( {1.-1./3., 0.-1./3., 0.-1./3.} ) );

  SECTION( "Update the first set of points around the centroid" ) {
    centroid_example.centroid_first_vector();
    centroid_example.update_first_points_around_centroid();
    for(int i = 0 ; i < 3 ; ++i) {
      obtained_point = centroid_example.pop_point_first_vector();
      expected_point = expected_updated_first_vector[i];
      REQUIRE( expected_point == obtained_point );
    }
  }

  SECTION( "Updating the second set of points around the centroid gives an error since it is empty" ) {
    REQUIRE_THROWS( centroid_example.update_second_points_around_centroid() );
  } 

}

TEST_CASE( "Compute the H matrix for the problem", "[H_matrix]" ) {

  lsq::LeastSquare3D H_matrix_example;

  Eigen::Vector3d point_one;
  Eigen::Vector3d point_two;
  Eigen::Matrix3d obtained_matrix;
  Eigen::Matrix3d expected_matrix = Eigen::Matrix3d::Zero(3, 3);

  SECTION( "Return the H matrix without compute it." ) {
    obtained_matrix = H_matrix_example.get_H_matrix();
    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "No element in both sets of points." ) {
    REQUIRE_THROWS( H_matrix_example.compute_H_matrix() );
  }

  SECTION( "No element in first sets of points." ) {
    point_two = {1.,0.,0.};
    H_matrix_example.add_point_second_vector(point_two);
    REQUIRE_THROWS( H_matrix_example.compute_H_matrix() );
  }

  SECTION( "No element in second sets of points." ) {
    point_one = {1.,0.,0.};
    H_matrix_example.add_point_first_vector(point_one);
    REQUIRE_THROWS( H_matrix_example.compute_H_matrix() );
  }

  SECTION( "One element in each set of points: xx case" ) {
    point_one = {1.,0.,0.};
    point_two = {1.,0.,0.};
    expected_matrix(0,0) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: xy case" ) {
    point_one = {1.,0.,0.};
    point_two = {0.,1.,0.};
    expected_matrix(0,1) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: xz case" ) {
    point_one = {1.,0.,0.};
    point_two = {0.,0.,1.};
    expected_matrix(0,2) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  } 
  
  SECTION( "One element in each set of points: yx case" ) {
    point_one = {0.,1.,0.};
    point_two = {1.,0.,0.};
    expected_matrix(1,0) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: yy case" ) {
    point_one = {0.,1.,0.};
    point_two = {0.,1.,0.};
    expected_matrix(1,1) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: yz case" ) {
    point_one = {0.,1.,0.};
    point_two = {0.,0.,1.};
    expected_matrix(1,2) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zx case" ) {
    point_one = {0.,0.,1.};
    point_two = {1.,0.,0.};
    expected_matrix(2,0) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zy case" ) {
    point_one = {0.,0.,1.};
    point_two = {0.,1.,0.};
    expected_matrix(2,1) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zz case" ) {
    point_one = {0.,0.,1.};
    point_two = {0.,0.,1.};
    expected_matrix(2,2) = 1;

    H_matrix_example.add_point_first_vector(point_one);
    H_matrix_example.add_point_second_vector(point_two);
    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "Two element in first set, one in the second." ) {
    point_one = {1.,0.,0.};
    H_matrix_example.add_point_first_vector(point_one);
    point_two = {1.,0.,0.};
    H_matrix_example.add_point_second_vector(point_two);
    point_one = {0.,1.,0.};
    H_matrix_example.add_point_first_vector(point_one);

    expected_matrix(0,0) = 1;

    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "Two element in first set, two in the second." ) {
    point_one = {1.,0.,0.};
    H_matrix_example.add_point_first_vector(point_one);
    point_two = {1.,0.,0.};
    H_matrix_example.add_point_second_vector(point_two);

    expected_matrix(0,0) = 1;

    point_one = {0.,1.,0.};
    H_matrix_example.add_point_first_vector(point_one);
    point_two = {0.,0.,1.};
    H_matrix_example.add_point_second_vector(point_two);

    expected_matrix(1,2) = 1;

    H_matrix_example.compute_H_matrix();
    obtained_matrix = H_matrix_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

}

TEST_CASE( "Set the algorithm we use for computing the rotation", "[set_strategy]" ) {

  lsq::LeastSquare3D strategy_example;

  std::unique_ptr<lsq::ComputeRotation> algorithm( new lsq::SVDMethod() );

  SECTION( "Check that the pointer algorithm is not empty." ) {
    if( !algorithm ) {
      REQUIRE(false);
    }
    REQUIRE(true);
  }

  SECTION( "Check that the pointer to the selected algorithm in the class is empty." ) {
    if( strategy_example.is_rotation_strategy() ) {
      REQUIRE(false);
    }
    REQUIRE(true);
  }

  strategy_example.set_rotation_strategy( std::move(algorithm) );

  SECTION( "Check that the pointer algorithm is empty." ) {
    if( algorithm ) {
      REQUIRE(false);
    }
    REQUIRE(true);
  }

  SECTION( "Check that the pointer to the selected algorithm in the class is not empty." ) {
    if( !strategy_example.is_rotation_strategy() ) {
      REQUIRE(false);
    }
    REQUIRE(true);
  }

}

TEST_CASE( "Compute the rotation matrix for the problem", "[R_matrix]" ) {

  lsq::LeastSquare3D rotation_example;

  Eigen::Matrix3d obtained_matrix;
  Eigen::Matrix3d expected_matrix = Eigen::Matrix3d::Zero(3, 3);

  SECTION( "Return the rotation without computing it." ) {
    obtained_matrix = rotation_example.get_rotation_matrix();
    REQUIRE( expected_matrix == obtained_matrix );
  }

  std::unique_ptr<lsq::ComputeRotation> algorithm( new lsq::SVDMethod() );
  rotation_example.set_rotation_strategy( std::move(algorithm) );
  
  SECTION( "Compute the rotation with the SVDMethod, without first computing H." ) {
    rotation_example.compute_rotation_matrix();
    obtained_matrix = rotation_example.get_rotation_matrix();

    expected_matrix = Eigen::Matrix3d::Identity(3, 3);

    REQUIRE( expected_matrix == obtained_matrix );
  }

  Eigen::Vector3d point_3D;

  point_3D = {1.,0.,0.};
  rotation_example.add_point_first_vector(point_3D);
  point_3D = {0.,1.,0.};
  rotation_example.add_point_first_vector(point_3D);
  point_3D = {0.,0.,1.};
  rotation_example.add_point_first_vector(point_3D);

  point_3D = {1.,0.,0.};
  rotation_example.add_point_second_vector(point_3D);
  point_3D = {0.,0.,-1.};
  rotation_example.add_point_second_vector(point_3D);
  point_3D = {0.,1.,0.};
  rotation_example.add_point_second_vector(point_3D);

  SECTION( "Properly compute the rotation with the SVDMethod." ) {
    rotation_example.centroid_first_vector();
    rotation_example.update_first_points_around_centroid();
    rotation_example.centroid_second_vector();
    rotation_example.update_second_points_around_centroid();
    rotation_example.compute_H_matrix();
    rotation_example.compute_rotation_matrix();

    obtained_matrix = rotation_example.get_rotation_matrix();

    expected_matrix(0,0) = 1.;
    expected_matrix(2,1) = -1.;
    expected_matrix(1,2) = 1.;

    REQUIRE( obtained_matrix.isApprox(expected_matrix,1.e-10) );
  }

}

TEST_CASE( "Compute the translation vector for the problem", "[translation_vector]" ) {

  lsq::LeastSquare3D translation_example;

  Eigen::Vector3d point_one;
  Eigen::Vector3d point_two;

  Eigen::Vector3d obtained_vector;
  Eigen::Vector3d expected_vector = Eigen::Vector3d::Zero(3);

  SECTION( "Return the translation without computing it." ) {
    obtained_vector = translation_example.get_translation_vector();
    REQUIRE( expected_vector == obtained_vector );
  }

  SECTION( "Compute the translation vector without computing centroids and rotation." ) {
    translation_example.compute_translation_vector();
    obtained_vector = translation_example.get_translation_vector();
    REQUIRE( expected_vector == obtained_vector );
  }

  SECTION( "Compute the translation vector while computing the centroids (no rotation though)." ) {
    point_one = {1.,0.,0.};
    translation_example.add_point_first_vector(point_one);
    translation_example.centroid_first_vector();  /* This first centroid should be useless since rotation is zero matrix. */

    point_two = {0.,1.,0.};
    translation_example.add_point_second_vector(point_two);
    translation_example.centroid_second_vector();

    translation_example.compute_translation_vector();
    obtained_vector = translation_example.get_translation_vector();
    expected_vector = {0.,1.,0.};
    REQUIRE( expected_vector == obtained_vector );
  }

}
