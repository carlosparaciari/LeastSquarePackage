/*=============================================================================

  LeastSquaresPackage: A software package for estimating the rotation and translation of rigid bodies.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef lsqWin32ExportHeader_h
#define lsqWin32ExportHeader_h

/**
* \file lsqWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(LEASTSQUARESPACKAGE_STATIC)
  #ifdef LEASTSQUARESPACKAGE_WINDOWS_EXPORT
    #define LEASTSQUARESPACKAGE_WINEXPORT __declspec(dllexport)
  #else
    #define LEASTSQUARESPACKAGE_WINEXPORT __declspec(dllimport)
  #endif  /* LEASTSQUARESPACKAGE_WINEXPORT */
#else
/* linux/mac needs nothing */
  #define LEASTSQUARESPACKAGE_WINEXPORT
#endif

#endif
