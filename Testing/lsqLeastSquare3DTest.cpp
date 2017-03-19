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