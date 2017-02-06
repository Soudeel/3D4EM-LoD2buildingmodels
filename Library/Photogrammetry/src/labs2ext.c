
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
  * Revision 1.3  2010/07/09 08:40:05  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:41:21  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:13  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:00  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:28  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:57  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.1.1.1  2003/04/09 11:07:37  rabbani
  * basic photogrammetric classes and functions (orientation, in/output, matrices, ...
  *
  * Revision 1.1.1.1  2003/03/07 13:17:29  pfeifer
  * Orientation of images
  *
  * Revision 1.1.1.1  2003/03/03 14:11:29  rabbani
  * Routines for image orientation
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for abs2ext
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	labs2ext
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */


#include "internals.h"

/* -library_includes */
#include "Database.h"
/****************************************************************
* 
*  Routine Name: labs2ext - Conversion of a relative and absolute orientation files of a model to the two exterior orientations of the images.
* 
*       Purpose: Library Routine for abs2ext
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: M.G. Vosselman
*          Date: Aug 18, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int labs2ext(Exterior *rel1, Exterior *rel2, Absolute *abs,
             Exterior *ext1, Exterior *ext2)
/* -library_def_end */

/* -library_code */
{
  double modelpt[3], objectpt[3];
  void matmult(double *, double *, double *, int, int, int);

/* Rotation matrices */

  matmult((double *) abs->rot, (double *) rel1->rot, (double *) ext1->rot,
          3, 3, 3);
  matmult((double *) abs->rot, (double *) rel2->rot, (double *) ext2->rot,
          3, 3, 3);

/* Projection centre of the left image */

  modelpt[0] = rel1->x * abs->lambda;
  modelpt[1] = rel1->y * abs->lambda;
  modelpt[2] = rel1->z * abs->lambda;
  matmult((double *) abs->rot, (double *) modelpt, (double *) objectpt,
          3, 3, 1);
  ext1->x = objectpt[0] + abs->x;
  ext1->y = objectpt[1] + abs->y;
  ext1->z = objectpt[2] + abs->z;

/* Projection centre of the right image */

  modelpt[0] = rel2->x * abs->lambda;
  modelpt[1] = rel2->y * abs->lambda;
  modelpt[2] = rel2->z * abs->lambda;
  matmult((double *) abs->rot, (double *) modelpt, (double *) objectpt,
          3, 3, 1);
  ext2->x = objectpt[0] + abs->x;
  ext2->y = objectpt[1] + abs->y;
  ext2->z = objectpt[2] + abs->z;

  return(1);

}
/* -library_code_end */
