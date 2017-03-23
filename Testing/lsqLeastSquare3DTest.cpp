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
  Eigen::Array3d point_3D;

  SECTION( "Pop an element from the first vector" ) {
    REQUIRE_THROWS( point_3D = add_example.pop_point_first_vector() );
  }
  SECTION( "Pop an element from the second vector" ) {
    REQUIRE_THROWS( point_3D = add_example.pop_point_second_vector() );
  }

}

TEST_CASE( "Add 3D point to first and second set", "[add_remove_points]" ) {

  lsq::LeastSquare3D add_example;
  Eigen::Array3d point_3D_in = {1,2,3};
  Eigen::Array3d point_3D_out;

  SECTION( "Add point to first vector and pop it out" ) {
    add_example.add_point_first_vector(point_3D_in);
    point_3D_out = add_example.pop_point_first_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( point_3D_out(i) == point_3D_in(i) );
  }

  SECTION( "Add point to second vector and pop it out" ) {
    add_example.add_point_second_vector(point_3D_in);
    point_3D_out = add_example.pop_point_second_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( point_3D_out(i) == point_3D_in(i) );
  }

}

TEST_CASE( "Compute centroid for the sets of points and update the set", "[centroids]" ) {

  lsq::LeastSquare3D centroid_example;
  Eigen::Array3d point_3D_in;
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

  Eigen::Array3d expected_centroid = {0.,0.,0.};
  Eigen::Array3d obtained_centroid;

  SECTION( "Get first centroid without having computed it" ) {
    obtained_centroid = centroid_example.get_centroid_first_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( expected_centroid(i) == obtained_centroid(i) );
  }

  SECTION( "Get second centroid without having computed it" ) {
    obtained_centroid = centroid_example.get_centroid_second_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( expected_centroid(i) == obtained_centroid(i) );
  }

  expected_centroid = {1./3., 1./3., 1./3.};

  SECTION( "Compute first centroid and check it" ) {
    centroid_example.centroid_first_vector();
    obtained_centroid = centroid_example.get_centroid_first_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( expected_centroid(i) == obtained_centroid(i) );
  }

  SECTION( "Compute again the first centroid and check that it does not change" ) {
    centroid_example.centroid_first_vector();
    obtained_centroid = centroid_example.get_centroid_first_vector();
    for( int i = 0 ; i < 3 ; ++i )
      REQUIRE( expected_centroid(i) == obtained_centroid(i) );
  }

  SECTION( "Compute second centroid and get an error (the second vector is empty)" ) {
    REQUIRE_THROWS( centroid_example.centroid_second_vector() );
  }
  
  Eigen::Array3d expected_point;
  Eigen::Array3d obtained_point;
  std::vector<Eigen::Array3d> expected_updated_first_vector;

  expected_updated_first_vector.push_back( Eigen::Array3d( {0.-1./3., 0.-1./3., 1.-1./3.} ) );
  expected_updated_first_vector.push_back( Eigen::Array3d( {0.-1./3., 1.-1./3., 0.-1./3.} ) );
  expected_updated_first_vector.push_back( Eigen::Array3d( {1.-1./3., 0.-1./3., 0.-1./3.} ) );

  SECTION( "Update the first set of points around the centroid" ) {
    centroid_example.centroid_first_vector();
    centroid_example.update_first_points_around_centroid();
    for(int i = 0 ; i < 3 ; ++i) {
      obtained_point = centroid_example.pop_point_first_vector();
      expected_point = expected_updated_first_vector[i];
      for( int j = 0 ; j < 3 ; ++j )
        REQUIRE( expected_point(j) == obtained_point(j) );
    }
  }

  SECTION( "Updating the second set of points around the centroid gives an error since it is empty" ) {
    REQUIRE_THROWS( centroid_example.update_second_points_around_centroid() );
  } 

}
