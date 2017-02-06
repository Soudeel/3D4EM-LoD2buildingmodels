
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
  * Revision 1.1  2006/04/06 14:10:15  WINDOWSNT\vosselman
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
  * Revision 1.1.1.1  2003/03/07 12:59:10  pfeifer
  * Standard library: points, matrices, triangulation, ...
  *
  * Revision 1.1.1.1  2003/03/03 14:03:27  rabbani
  * Standard library for FRS
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for difvec
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	ldifvec
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */


#include "internals.h"

/* -library_includes */
#include <stdlib.h>
#include "Database.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: ldifvec - Derivation of difference vectors between two sets of image points.
* 
*       Purpose: Derivation of difference vectors between two sets of image points.
Difference vectors are computed for points with identical point numbers.
The vector is pointing from the reference data to the (measured) data
to be evaluated. The scale of this vector can be controled by an
exaggeration factor.
*         Input: .IP "imgpts1" 15
structure containing the image points of the reference set
.IP "imgpts2" 15
structure containing the image points of the (measured) set to be
evaluated
.IP "vs" 15
double scale factor for difference vectors
*        Output: .IP "vecs" 15
ImgLines structure containing the derived difference vectors
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: George Vosselman
*          Date: Jul 26, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
void ldifvec(ImgPts *imgpts1, ImgPts *imgpts2, ImgLines *vecs, double vs)
/* -library_def_end */

/* -library_code */
{
  ImgLine *vec;
  int     i1, i2, found;
  ImgPt   *imgpt1, *imgpt2;

  vec = vecs->lines;

  for (i1=0, imgpt1=imgpts1->pts;
       i1<imgpts1->num_pts;
       i1++, imgpt1++) {
    found = 0;
    for (i2=0, imgpt2=imgpts2->pts;
         i2<imgpts2->num_pts && !found;
         i2++, imgpt2++) {
      if (imgpt1->num == imgpt2->num) {
        vecs->num_lines++;
        vec->num          = imgpt1->num;
        vec->pts          = (ImgPt *) malloc(2 * sizeof(ImgPt));
        vec->num_pts      = 2;
        vec->label        = 0;
        vec->pts[0]       = *imgpt1;
        vec->pts[0].num   = imgpt1->num * 10;
        vec->pts[1].r     = imgpt1->r + (imgpt2->r - imgpt1->r) * vs;
        vec->pts[1].c     = imgpt1->c + (imgpt2->c - imgpt1->c) * vs;
        vec->pts[1].v_r   = vs * vs * imgpt2->v_r +
                            (1 - vs) * (1 - vs) * imgpt1->v_r;
        vec->pts[1].v_c   = vs * vs * imgpt2->v_c +
                            (1 - vs) * (1 - vs) * imgpt1->v_c;
        vec->pts[1].cv_rc = vs * vs * imgpt2->cv_rc +
                            (1 - vs) * (1 - vs) * imgpt1->cv_rc;
        vec->pts[1].num   = imgpt1->num * 10 + 1;
        vec++;
      }
    }
  }
}
/* -library_code_end */
