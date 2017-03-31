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
#include <fstream>
#include <algorithm>
#include <iterator>

// A helper function to simplify the help command.
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

    std::string algorithm_strategy;
    std::string output_filename;

    // Allowed options, shown in the help.
    boost::program_options::options_description desc("Options");

    desc.add_options()
      ("help,h", "produce help message")
      ("version,v", "return the version of the application")
      ("method,m", boost::program_options::value< std::string >(&algorithm_strategy)->default_value("svd"),
        "set the algorithm for computing the rotation, either svd or quat")
      ("output,o", boost::program_options::value< std::string >(&output_filename)->default_value("output.dat"),
        "set the name of the output file where rotation and translation are saved")
    ;

    // Hidden options, are allowed on command line, but are not shown to the user.
    boost::program_options::options_description hidden("Hidden options");

    hidden.add_options()
      ("input-files", boost::program_options::value< std::vector<std::string> >(),
        "input files with the coordinates of the two sets of 3D points")
    ;

    boost::program_options::options_description cmdline_options;
    cmdline_options.add(desc).add(hidden);

    boost::program_options::positional_options_description p;
    p.add("input-files", -1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(cmdline_options).positional(p).run(), vm);
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

    lsq::LeastSquare3D reference_frames;

    if (vm.count("input-files")) {

      auto input_filenames = vm["input-files"].as<std::vector<std::string>>();

      if (input_filenames.size() != 2) {
        std::cerr << "Provide two files with the sets of coordinates of the 3D points." << std::endl;
        return 1;
      }

      // ---------------------------- Read the coordinates and fill the two sets of 3D points ---------------------------
      std::ifstream filein;
      double x_value, y_value, z_value;
      Eigen::Vector3d point;

      for( int i = 0 ; i < 2 ;  ++i ) {

        filein.open( input_filenames[i] );

        if (!filein.is_open()) {
          std::cerr << "Error opening the input file " << input_filenames[i] << "." << std::endl;
          return 1;
        }

        std::cout << "Input file " << input_filenames[i] << " is read and points are saved..." << std::endl;

        while(!filein.eof()) {
          filein >> x_value >> y_value >> z_value;
          point = {x_value, y_value, z_value};
          if ( i == 0 )
            reference_frames.add_point_first_vector(point);
          else
            reference_frames.add_point_second_vector(point);
        }

        filein.close();
      }

      if ( !reference_frames.same_number_of_points() ) {
        std::cout << "warning: The two sets of points have different size. The additional points will be neglected." << std::endl;
      }

    }
    else {
      std::cerr << "No input files are provided. Provide two files with the sets of coordinates of the 3D points." << std::endl;
      return 1;
    }

    //  ----------------------------------- Set the algorithm to compute the rotation -----------------------------------
    std::unique_ptr<lsq::ComputeRotation> algorithm;

    if ( algorithm_strategy == std::string("svd") ) {
      std::cout << "The chosen algorithm is svd." << std::endl;
      algorithm = std::unique_ptr<lsq::ComputeRotation>( new lsq::SVDMethod() );
    }
    else if ( algorithm_strategy == std::string("quat") ) {
      std::cout << "The chosen algorithm is quat." << std::endl;
      algorithm = std::unique_ptr<lsq::ComputeRotation>( new lsq::QuaternionMethod() );
    }
    else {
      std::cerr << "The chosen method is not available. Refer to the --help for the available methods." << std::endl;
      return 1;
    }

    reference_frames.set_rotation_strategy( std::move(algorithm) );

    // ---------------------- Compute the centroids and the update the sets of points around them -----------------------
    reference_frames.centroid_first_vector();
    reference_frames.centroid_second_vector();
    reference_frames.update_first_points_around_centroid();
    reference_frames.update_second_points_around_centroid();

    // ---------------------------------------------- Compute the H matrix ----------------------------------------------
    reference_frames.compute_H_matrix();

    // ---------------------------------------- Compute rotation and translation ----------------------------------------
    reference_frames.compute_rotation_matrix();
    reference_frames.compute_translation_vector();

    // ----------------------------------------- Save rotation and translation -----------------------------------------
    Eigen::Matrix3d rotation = reference_frames.get_rotation_matrix();
    Eigen::Vector3d translation = reference_frames.get_translation_vector();

    std::ofstream fileout;

    fileout.open( output_filename );

    if (!fileout.is_open()) {
      std::cerr << "Error opening the output file " << output_filename << "." << std::endl;
      return 1;
    }

    std::cout << "Rotation and translation are saved in the output file " << output_filename << "." << std::endl;

    fileout << "The rotation matrix is:" << std::endl;
    fileout << rotation << std::endl;
    fileout << "The translation vector is:" << std::endl;
    fileout << translation << std::endl;

    fileout.close();

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
