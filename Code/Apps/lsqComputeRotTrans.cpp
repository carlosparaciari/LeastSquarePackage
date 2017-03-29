/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <boost/program_options.hpp>
#include "lsqLeastSquare3D.h"
#include <iostream>
#include <algorithm>
#include <iterator>

// A helper function to simplify the main part.
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    return os;
}

/**
* \file lsqComputeRotTrans.cpp
* \brief End-user application for computing the rotation and translation connecting two sets of 3D points.
* \ingroup applications
*
* This command line application can read from two separate files, each of them containing the coordinates of many 3D points,
* and compute the rotation and translation connecting the two sets. The user can choose between two different methods to
* compute the rotation, and the methods can be selected with a specific option (see help). The two methods are based, respectively,
* on the singular value decomposition of a matrix obtained from the sets of points (K.S. Arun et al., IEEE PAMI-9, 698-700, 1987),
* and on the quaternionic representation of matrices (B.K.P. Horn, J. Opt. Soc. Am. A 4, 629-642, 1987). Rotation and translation
* are saved in file "rotation.dat" and "translation.dat" respectively.
*/
int main(int argc, char* argv[]) {

  try {

    boost::program_options::options_description desc("Allowed options");
    std::string algorithm_strategy;

    desc.add_options()
      ("help,h", "produce help message")
      ("version,v", "return the version of the application")
      ("method,m", boost::program_options::value< std::string >(&algorithm_strategy)->default_value("svd"),
        "set the algorithm to use for computing the rotation, either svd or quat")
      ("input-files", boost::program_options::value< std::vector<std::string> >(),
        "input files with the coordinates of the two sets of 3D points");

    boost::program_options::positional_options_description p;
    p.add("input-files", -1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
      std::cout << "Usage: options_description [options]" << std::endl;
      std::cout << desc;  // The helper function is used by the help to print the list of commands.
      return 0;
    }

    if (vm.count("version")) {
      std::cout << "lsqComputeRotTrans version 0.0.1" << std::endl;
      return 0;
    }

    if (vm.count("input-files")) {
      auto input_filenames = vm["input-files"].as<std::vector<std::string>>();
      if (input_filenames.size() != 2) {
        std::cerr << "Provide two files with the sets of coordinates of the 3D points." << std::endl;
        return 1;
      }
      std::cout << "Input files are: " << input_filenames << std::endl;
    }
    else {
      std::cerr << "No input files are provided. Provide two files with the sets of coordinates of the 3D points." << std::endl;
      return 1;
    }

    if ( algorithm_strategy == std::string("svd") ) {
      std::cout << "The chosen algorithm is svd." << std::endl;
    }
    else if ( algorithm_strategy == std::string("quat") ) {
      std::cout << "The chosen algorithm is quat." << std::endl;
    }
    else {
      std::cerr << "The chosen method is not available. Refer to the --help for the available methods." << std::endl;
      return 1;
    }

    std::cout << "Here it goes the implementation of the application." << std::endl;

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
