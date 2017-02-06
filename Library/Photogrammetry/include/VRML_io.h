
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



/*!
 * \file
 * \brief Declarations of the functions for output in VRML format.
 *
 */
/*!
 * \ingroup Photogrammetry
 * \brief Declarations of the functions for output in VRML format.
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _VRML_io_h_
#define _VRML_io_h_

/*
--------------------------------------------------------------------------------
This file contains the declarations of the functions for output in VRML format.
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "Position3D.h"

/// Open a VRML 1.0 file and write the header
extern FILE   *VRML_Open(const char *);

/// Open a VRML 2.0 file and write the header
extern FILE   *VRML2_Open(const char *);

/// Write the trailer of a VRML 1.0 file and close it
extern void   VRML_Close(FILE *);

/// Close the VRML 2.0 file
extern void   VRML2_Close(FILE *);

/// Write the beginning of a point list to a VRML 1.0 file
extern void   VRML_Write_Begin_Of_PointList(FILE *);

/// Write the end of a point list to a VRML 1.0 file
extern void   VRML_Write_End_Of_PointList(FILE *);

/// Write the colour to a VRML 1.0 file
extern void   VRML_Set_Colour(FILE *, float, float, float);

/// Convert a scalar to a colour
extern void   VRML_Scalar_To_Colour(float, float *, float *, float *);

/// Write a cylinder to a VRML 1.0 file
extern void   VRML_Write_Cylinder(FILE *, Position3D &, Position3D &, double);

#endif /* _VRML_io_h_ */  /* Don't add after this point */
