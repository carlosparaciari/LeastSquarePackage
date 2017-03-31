LeastSquarePackage
------------------

This package provides tools for computing the least-square rotation and translation
connecting two sets of 3D points. Two different algorithms are provided for finding
the rotation, one based on singular value decomposition - K.S. Arun et al., IEEE PAMI-9, 698-700 (1987),
and the other based on the quaternionic representation of rotations - B.K.P. Horn, J. Opt. Soc. Am. A 4, 629-642 (1987). 

Credit goes to [MattClarkson](https://github.com/MattClarkson) for the CMake template used for this package. 

Note: The package was built as an exercise and is no longer maintained.

Overview
--------

The features of the package are:
 1. Methods to compute the centroid of sets of 3D points.
 2. Two different algorithms to compute the rotation.
 3. The algorithms are implemented using the strategy pattern, so you can add more and choose the one you like at runtime.
 4. Unit tests with Catch (run ```make test``` to run the tests)
 5. Use CMake to build the package (also possible to use SuperBuild option to download dependencies).
 6. Doxygen creates the documentation.
 7. Use Eigen 3.3.2 and Boost 1.63.0.
 8. Command-line application which reads from two files (sets of points) and returns the rotation and translation.


Tested On
-----------------------------

 * Windows - Windows 10, g++ 5.3.0 (MinGW - MSYS), CMake 3.7.1
 * Linux - soon


Build Instructions
-----------------------------

This package comes with the possibility of downloading several libraries.
In particular, it is possible to download and build Eigen and Boost, which
are needed for the package to work. To download and build dependencies, use
CMake to configure:

  * BUILD_SUPERBUILD:BOOL=ON

You can also avoid to download and build these dependencies, in case they
are already present in your host machine. In that case, set:

  * BUILD_SUPERBUILD:BOOL=OFF

In any case, the following options should be set as follow:

  * BUILD_Eigen:BOOL=ON
  * BUILD_Boost:BOOL=ON

To switch between static/dynamic linking, use CMake to set:

  * BUILD_SHARED_LIBS:BOOL=ON|OFF

To switch between Debug and Release mode, use CMake to set:

  * CMAKE_BUILD_TYPE:STRING=Debug|Release

Note: Only Debug and Release are supported. 

It might be possible that your host system have a version of Boost that
is different from the one assumed here (1.63.0). In that case, you will
have to modify ./CMakeLists.txt file (replace 1.63 with your version
number in line 244).

Notice that, if you set the SuperBuild option on, the Boost version will
be 1.56. You will not have to change the version by hand, as CMake will do
it for you.


Windows Users
-------------

If you build the project with shared libraries (BUILD_SHARED_LIBS:BOOL=ON)
then when you run executables, you should look for the batch file
StartVS_Debug.bat or StartVS_Release.bat in the LEASTSQUARESPACKAGE-build folder.
This sets the path before launching Visual Studio, so that dynamically
loaded libraries are found at run time.

Note: The package was never built with Visual Studio, so other problems might appear.


Command-line application
------------------------

This package comes with a handy command-line application, lsqComputeRotTrans.
This application reads from two different files (with the coordinates of 3D
points in it), and outputs the rotation and translation connecting the points.
You can set the method you want to use, and specify the output file where the
rotation and translation will be saved.
