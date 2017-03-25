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
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

TEST_CASE( "Number of passed arguments is received", "[init]" ) {

  int expectedNumberOfArgs = 5;
  if (lsq::argc != expectedNumberOfArgs)
  {
    std::cerr << "Usage: mpMyFirstCatchTest fileName.txt" << std::endl;
    REQUIRE( lsq::argc == expectedNumberOfArgs );
  }
  REQUIRE(true);

}

TEST_CASE( "Data files for test are open", "[init]" ) {

  std::fstream filein;

  for( int i = 1 ; i < lsq::argc ; ++i)
  {
    filein.open (lsq::argv[i]);

    if (!filein.is_open())
    {
      REQUIRE( filein.is_open() );
    }
    filein.close();
    REQUIRE(true);
  }

}

TEST_CASE( "First set of points is read", "[init]" ) {

  double x_value, y_value, z_value;
  int i = 0;
  double points[3][3] = {  
    {1., 0., 0.} ,
    {0., 1., 0.} ,
    {0., 0., 1.}
  };

  std::fstream filein;
  filein.open (lsq::argv[1]);

  while(!filein.eof())
  {
    filein >> x_value >> y_value >> z_value;
    REQUIRE(points[i][0] == x_value);
    REQUIRE(points[i][1] == y_value);
    REQUIRE(points[i][2] == z_value);
    ++i;
  }

  filein.close();

}

TEST_CASE( "Second set of points is read", "[init]" ) {

  double x_value, y_value, z_value;
  int i = 0;
  double points[3][3] = {  
    {0.967371, 1.19473, 0.337903} ,
    {-0.162097, 1.96737, 0.694726} ,
    {0.194726, 0.837903, 1.46737}
  };

  std::fstream filein;
  filein.open (lsq::argv[2]);

  while(!filein.eof())
  {
    filein >> x_value >> y_value >> z_value;
    REQUIRE(points[i][0] == x_value);
    REQUIRE(points[i][1] == y_value);
    REQUIRE(points[i][2] == z_value);
    ++i;
  }

  filein.close();

}

TEST_CASE( "Rotation matrix is read", "[init]" ) {

  double first_elem, second_elem, third_elem;
  int i = 0;
  double rotation[3][3] = {  
    {0.967371, -0.162097, 0.194726} ,
    {0.194726, 0.967371, -0.162097} ,
    {-0.162097, 0.194726, 0.967371}
  };

  std::fstream filein;
  filein.open (lsq::argv[3]);

  while(!filein.eof())
  {
    filein >> first_elem >> second_elem >> third_elem;
    REQUIRE(rotation[i][0] == first_elem);
    REQUIRE(rotation[i][1] == second_elem);
    REQUIRE(rotation[i][2] == third_elem);
    ++i;
  }

  filein.close();

}

TEST_CASE( "Translation vector is read", "[init]" ) {

  double x_value, y_value, z_value;
  double translation[3] = {0., 1., 0.5};

  std::fstream filein;
  filein.open (lsq::argv[4]);

  filein >> x_value >> y_value >> z_value;
  REQUIRE(translation[0] == x_value);
  REQUIRE(translation[1] == y_value);
  REQUIRE(translation[2] == z_value);

  filein.close();

}

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

}

TEST_CASE( "Compute centroid for the sets of points and update the set", "[centroids]" ) {

  lsq::LeastSquare3D centroid_example;
  Eigen::Vector3d point_3D_in;
  double x_value, y_value, z_value;

  std::fstream filein;
  filein.open (lsq::argv[1]);

  while(!filein.eof())
  {
    filein >> x_value >> y_value >> z_value;
    point_3D_in = {x_value, y_value, z_value};
    centroid_example.add_point_first_vector(point_3D_in);
  }

  filein.close();

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

  lsq::LeastSquare3D centroid_example;

  Eigen::Vector3d point_one;
  Eigen::Vector3d point_two;
  Eigen::Matrix3d obtained_matrix;
  Eigen::Matrix3d expected_matrix = Eigen::Matrix3d::Zero(3, 3);

  SECTION( "Return the H matrix without compute it." ) {
    obtained_matrix = centroid_example.get_H_matrix();
    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "No element in both sets of points." ) {
    REQUIRE_THROWS( centroid_example.compute_H_matrix() );
  }

  SECTION( "No element in first sets of points." ) {
    point_two = {1.,0.,0.};
    centroid_example.add_point_second_vector(point_two);
    REQUIRE_THROWS( centroid_example.compute_H_matrix() );
  }

  SECTION( "No element in second sets of points." ) {
    point_one = {1.,0.,0.};
    centroid_example.add_point_first_vector(point_one);
    REQUIRE_THROWS( centroid_example.compute_H_matrix() );
  }

  SECTION( "One element in each set of points: xx case" ) {
    point_one = {1.,0.,0.};
    point_two = {1.,0.,0.};
    expected_matrix(0,0) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: xy case" ) {
    point_one = {1.,0.,0.};
    point_two = {0.,1.,0.};
    expected_matrix(0,1) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: xz case" ) {
    point_one = {1.,0.,0.};
    point_two = {0.,0.,1.};
    expected_matrix(0,2) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  } 
  
  SECTION( "One element in each set of points: yx case" ) {
    point_one = {0.,1.,0.};
    point_two = {1.,0.,0.};
    expected_matrix(1,0) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: yy case" ) {
    point_one = {0.,1.,0.};
    point_two = {0.,1.,0.};
    expected_matrix(1,1) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: yz case" ) {
    point_one = {0.,1.,0.};
    point_two = {0.,0.,1.};
    expected_matrix(1,2) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zx case" ) {
    point_one = {0.,0.,1.};
    point_two = {1.,0.,0.};
    expected_matrix(2,0) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zy case" ) {
    point_one = {0.,0.,1.};
    point_two = {0.,1.,0.};
    expected_matrix(2,1) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "One element in each set of points: zz case" ) {
    point_one = {0.,0.,1.};
    point_two = {0.,0.,1.};
    expected_matrix(2,2) = 1;

    centroid_example.add_point_first_vector(point_one);
    centroid_example.add_point_second_vector(point_two);
    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "Two element in first set, one in the second." ) {
    point_one = {1.,0.,0.};
    centroid_example.add_point_first_vector(point_one);
    point_two = {1.,0.,0.};
    centroid_example.add_point_second_vector(point_two);
    point_one = {0.,1.,0.};
    centroid_example.add_point_first_vector(point_one);

    expected_matrix(0,0) = 1;

    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

  SECTION( "Two element in first set, two in the second." ) {
    point_one = {1.,0.,0.};
    centroid_example.add_point_first_vector(point_one);
    point_two = {1.,0.,0.};
    centroid_example.add_point_second_vector(point_two);

    expected_matrix(0,0) = 1;

    point_one = {0.,1.,0.};
    centroid_example.add_point_first_vector(point_one);
    point_two = {0.,0.,1.};
    centroid_example.add_point_second_vector(point_two);

    expected_matrix(1,2) = 1;

    centroid_example.compute_H_matrix();
    obtained_matrix = centroid_example.get_H_matrix();

    REQUIRE( expected_matrix == obtained_matrix );
  }

}
