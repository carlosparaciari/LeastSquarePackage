/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <lsqMyFunctions.h>
#include <iostream>

#ifdef BUILD_gflags
#include "gflags/gflags.h"
#endif

#ifdef BUILD_glog
#include <glog/logging.h>
#endif

#ifdef BUILD_Eigen
#include <Eigen/Dense>
#endif

#ifdef BUILD_OpenCV
#include <cv.h>
#endif

/**
 * \brief Demo file to check that #includes and library linkage is correct.
 */
int main(int argc, char** argv)
{

#ifdef BUILD_glog
  google::InitGoogleLogging(argv[0]);
#endif

#ifdef BUILD_gflags
  gflags::SetVersionString("1.0.0");
#endif

#ifdef BUILD_Eigen
  Eigen::MatrixXd m(2,2);
  std::cout << "Printing 2x2 matrix ..." << m << std::endl;
#endif

#ifdef BUILD_OpenCV
  cv::Matx44d matrix = cv::Matx44d::eye();
  std::cout << "Printing 4x4 matrix ..." << matrix << std::endl;
#endif

  std::cout << "Calculating ... " << lsq::MyFirstFunction(1) << std::endl;
  return 0;
}
