
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
  * Revision 1.1  2006/04/06 14:10:18  WINDOWSNT\vosselman
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
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:52  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for model2obj
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lmodel2obj
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include <stdlib.h>
#include "Database.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lmodel2obj - *
* 
*       Purpose: Library Routine for model2obj
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: R.Th. Ursem
*          Date: Jul 28, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lmodel2obj(ModelPts *model, Absolute *absolute, ObjPts **object)
/* -library_def_end */

/* -library_code */
{
   int i,j;
   double variance[3][3], objvar[3][3], temp[3][3];
   
   void mattrans(double *, int, int, double *);
   void matmult(double *, double *, double *, int, int, int);

   /*------ Allocate space ------*/
   *object = (ObjPts *)calloc(1, sizeof(ObjPts));
   (*object)->pts = (ObjPt *)calloc(model->num_pts, sizeof(ObjPt));
   (*object)->num_pts = model->num_pts;
   
   for (i = 0; i < model->num_pts; i++)
   {
      (*object)->pts[i].num = model->pts[i].num;
      
      /*------------------- Object coordinates ----------------------*/
      
      (*object)->pts[i].x = absolute->lambda * 
                                     (absolute->rot[0][0] * model->pts[i].x  + 
                                      absolute->rot[0][1] * model->pts[i].y  +
                                      absolute->rot[0][2] * model->pts[i].z) +
                            absolute->x;

      (*object)->pts[i].y = absolute->lambda * 
                                     (absolute->rot[1][0] * model->pts[i].x  + 
                                      absolute->rot[1][1] * model->pts[i].y  +
                                      absolute->rot[1][2] * model->pts[i].z) +
                            absolute->y;

      (*object)->pts[i].z = absolute->lambda * 
                                     (absolute->rot[2][0] * model->pts[i].x  + 
                                      absolute->rot[2][1] * model->pts[i].y  +
                                      absolute->rot[2][2] * model->pts[i].z) +
                            absolute->z;
                            
      /*------------------- variances ----------------------*/
      
      variance[0][0] = model->pts[i].v_x;
      variance[1][1] = model->pts[i].v_y;
      variance[2][2] = model->pts[i].v_z;
      variance[0][1] = variance[1][0] = model->pts[i].cv_xy;
      variance[0][2] = variance[2][0] = model->pts[i].cv_xz;
      variance[2][1] = variance[1][2] = model->pts[i].cv_yz;
      
      matmult((double *) absolute->rot, (double *) variance, (double *) objvar,
              3, 3, 3);
      mattrans((double *) absolute->rot, 3, 3, (double *) temp);
      matmult((double *) objvar, (double *) temp, (double *) variance, 3, 3, 3);
      
      (*object)->pts[i].v_x = absolute->lambda * absolute->lambda * variance[0][0];
      (*object)->pts[i].v_y = absolute->lambda * absolute->lambda * variance[1][1];
      (*object)->pts[i].v_z = absolute->lambda * absolute->lambda * variance[2][2];
                            
      /*------------------- covariances ----------------------*/
      
      (*object)->pts[i].cv_xy = absolute->lambda * absolute->lambda * variance[1][0];
      (*object)->pts[i].cv_xz = absolute->lambda * absolute->lambda * variance[2][0];
      (*object)->pts[i].cv_yz = absolute->lambda * absolute->lambda * variance[2][1];
   }
   return(1);
}
/* -library_code_end */
