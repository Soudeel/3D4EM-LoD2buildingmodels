
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
  * Revision 1.2  2010/06/15 14:41:22  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:16  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:00  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:29  WINDOWSNT\heuel
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
   >>>> 	Library Routine for ext2abs
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lext2abs
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include <math.h>
#include "Database.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lext2abs - Conversion of two exterior orientation files to the relative and absolute orientation of a model.
* 
*       Purpose: Library Routine for ext2abs
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
int lext2abs(Exterior *ext1, Exterior *ext2, Exterior *rel1, Exterior *rel2,
             Absolute *abs)
/* -library_def_end */

/* -library_code */
{
  double scale2, modelpt[3], objectpt[3], tmp[3][3], tmp2;
  int    i, j;
  
  void mattrans(double *, int, int, double *);
  void matmult(double *, double *, double *, int, int, int);
  
/* Absolute orientation paramters */

  /* Length of the modelbase in the object space (= scale factor) */

  tmp2 = (ext1->z - ext2->z);
  scale2 = tmp2 * tmp2;
  tmp2 = (ext1->y - ext2->y);
  scale2 += tmp2 * tmp2;
  tmp2 = (ext1->x - ext2->x);
  scale2 += tmp2 * tmp2;
/*
  scale2 = (ext1->x - ext2->x) * (ext1->x - ext2->x) +
           (ext1->y - ext2->y) * (ext1->y - ext2->y) +
           (ext1->z - ext2->z) * (ext1->z - ext2->z);
*/
  abs->lambda = sqrt(scale2);

  /* Rotation matrix */

  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      abs->rot[i][j] = ext1->rot[i][j];

  /* Offset */

  abs->x = ext1->x;
  abs->y = ext1->y;
  abs->z = ext1->z;

/* Relative orientation parameters */

  /* Rotation matrix of left image (= identity matrix) */

  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) {
      rel1->rot[i][j] = 0.0;
      if (i == j) rel1->rot[i][j] = 1.0;
    }
  }

  /* Rotation matrix of right image */

  mattrans((double *) ext1->rot, 3, 3, (double *) tmp);
  matmult((double *) tmp, (double *) ext2->rot, (double *) rel2->rot, 3, 3, 3);

  /* Projection centre of the left image */

  rel1->x = 0.0;
  rel1->y = 0.0;
  rel1->z = 0.0;

  /* Projection centre of the right image */

  objectpt[0] = (ext2->x - ext1->x) / abs->lambda;
  objectpt[1] = (ext2->y - ext1->y) / abs->lambda;
  objectpt[2] = (ext2->z - ext1->z) / abs->lambda;
  mattrans((double *) abs->rot, 3, 3, (double *) tmp);
  matmult((double *) tmp, (double *) objectpt, (double *) modelpt, 3, 3, 1);
  rel2->x = modelpt[0];
  rel2->y = modelpt[1];
  rel2->z = modelpt[2];

  return(1);
}
/* -library_code_end */
