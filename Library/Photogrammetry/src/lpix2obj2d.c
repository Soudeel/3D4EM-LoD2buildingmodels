
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



 /*
  * $Log$
  * Revision 1.3  2010/07/09 08:40:06  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:41:22  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:19  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:01  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:30  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:58  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.1.1.1  2003/04/09 11:07:37  rabbani
  * basic photogrammetric classes and functions (orientation, in/output, matrices, ...
  *
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:53  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for pix2obj2d
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lpix2obj2d
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "Database.h"
/* -library_includes_end */

/****************************************************************
* 
*  Routine Name: lpix2obj2d - This function transforms image coordinates to planimetric object coordinates given the grid parameters defining the relation between the two coordinate systems.
* 
*       Purpose: This function transforms a set of image coordinates to planimetric object
coordinates.
The relation between the 2D object coordinate ksystem (often used in orthophotos
or rectified imagery) and the image coordinate system is specified by the
object coordinates of the left upper corner of the left upper image pixel and
the pixel size. These three parameters are obtained from the grid definition
structure.
*         Input: .IP "grid" 15
structure containing specification of the 2D object coordinate system
.IP "imgpt" 15
structure containing the pixel coordinates of a point with their variances
*        Output: .IP "objpt2d" 15
structure containing the 2D object coordinates of a point with their variances
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: George Vosselman
*          Date: Aug 18, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
void Image_To_Object_2D(Grid *grid, ImgPt *imgpt, ObjPt2D *objpt2d)
/* -library_def_end */

/* -library_code */
{
  objpt2d->num = imgpt->num;

  objpt2d->x = grid->x + (imgpt->c + 0.5) * grid->pixelsize;
  objpt2d->y = grid->y - (imgpt->r + 0.5) * grid->pixelsize;

  objpt2d->v_x   = imgpt->v_c * grid->pixelsize * grid->pixelsize;
  objpt2d->v_y   = imgpt->v_r * grid->pixelsize * grid->pixelsize;
  objpt2d->cv_xy = imgpt->cv_rc * grid->pixelsize * grid->pixelsize;
}
/* -library_code_end */
