
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
  * Revision 1.1  2006/04/06 14:10:14  WINDOWSNT\vosselman
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
  * Revision 1.2  2003/05/15 13:50:45  pfeifer
  * introduced standard headers (eg <vector> instead of <vector.h>),
  * improved include syntax
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
   >>>> 	Library Routine for absor
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	labsor
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */


#include "internals.h"

/* -library_includes */
#include <math.h>
#include "Database.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: labsor - This program determines the seven parameters of the absolute orientation,  using at least three FULL control points and a corresponding number of object points.
* 
*       Purpose: Library Routine for absor
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: Ir. R.Th.Ursem
*          Date: Jul 28, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int labsor(CtrlPts *ctrlpts, ModelPts *modelpts, Absolute *absolute, int show)
/* -library_def_end */

/* -library_code */
{
   /*------ Declaration of functions ------*/
   double GetScaleFactor(CtrlPts *, ModelPts *);
   int Approx_Absor_Values(CtrlPts *, ModelPts *, double, int, int, double *,
                           double *, double *, double *, double *, double *); 
   int Exact_Absor_Values(CtrlPts *, ModelPts *, double *, int, int, double *,
                          double *, double *, double *, double *, double *);
   int CheckCorrespondingPoints(CtrlPts *, ModelPts *);
   
   /*------ Variable list ------*/
   double *lambda, *q1, *q2, *q3, *dX, *dY, *dZ;
   int num_pts;
   
   /*------ Start of library function ------*/
   num_pts = CheckCorrespondingPoints(ctrlpts, modelpts);
   if (num_pts < 3)
   {
      printf("labsor: At least three FULL control points are needed\n");
      return( 0 );
   }
   
   absolute->lambda = GetScaleFactor(ctrlpts, modelpts);
   lambda = &(absolute->lambda);
   
   q1 = &(absolute->a);
   q2 = &(absolute->b);
   q3 = &(absolute->c);
   dX = &(absolute->x);
   dY = &(absolute->y);
   dZ = &(absolute->z);

   if (!Approx_Absor_Values(ctrlpts, modelpts, *lambda, num_pts, show,
                            q1, q2, q3, dX, dY, dZ))
   {
      printf("labsor: ERROR when calculating approximate values.\n");
      return 0;
   }
   
   if (!Exact_Absor_Values(ctrlpts, modelpts, lambda, num_pts, show,
                           q1, q2, q3, dX, dY, dZ))
   {
      printf("labsor: ERROR when calculating exact values.\n");
      return 0;
   }
   
   Rot_From_Quat((Exterior *) absolute);
   Angles_From_Rot((Exterior *) absolute);
   Print_Absolute(absolute);
   
   return ( 1 );
}

int CheckCorrespondingPoints(CtrlPts *ctrlpts, ModelPts *modelpts)
{
   int i,j, num_pts;

   num_pts = 0;
   printf("\nCorresponding points (FULL control points):\n");
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < modelpts->num_pts; j++)
         if (ctrlpts->pts[i].num == modelpts->pts[j].num &&
             ctrlpts->pts[i].status == FULL_CTRL )
            {
               if (num_pts == 100) printf(" and more...");
               else if (num_pts < 100) printf(" %d", ctrlpts->pts[i].num);
               num_pts++;
            }
            
   printf("\nTotal: %d points.\n", num_pts);
   
   return num_pts;
}

double GetScaleFactor(CtrlPts *ctrlpts, ModelPts *modelpts)
{
   int i,j,k;
   int num1, num2;
   double distance, max_dist, GetDistance(CtrlPt *, CtrlPt *);
   
   max_dist = 0;
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < modelpts->num_pts; j++)
         if (ctrlpts->pts[i].num == modelpts->pts[j].num &&
             ctrlpts->pts[i].status == FULL_CTRL)
         {
            for (k = i+1; k < ctrlpts->num_pts; k++)
               if (ctrlpts->pts[k].num != ctrlpts->pts[i].num)
               {
                  distance = GetDistance(&(ctrlpts->pts[i]),
                                         &(ctrlpts->pts[k]));
                  if (distance > max_dist)
                  {
                     max_dist = distance;
                     num1 = ctrlpts->pts[i].num;
                     num2 = ctrlpts->pts[k].num;
                  }
               }
         }
   
   printf("\nMaximum distance in object system: %lf ", max_dist);
   printf("(between points %d and %d).\n", num1, num2);
   
   for (i = 0; i < modelpts->num_pts; i++)
      if (modelpts->pts[i].num == num1)
      {
         for (j = 0; j < modelpts->num_pts; j++)
            if (modelpts->pts[j].num == num2)
               distance = GetDistance((CtrlPt *) &(modelpts->pts[i]),
                                      (CtrlPt *) &(modelpts->pts[j]));
      }
   
   printf("Corresponding distance in model system: %lf\n", distance);
   printf("Scale factor: %lf\n", max_dist / distance);
   
   return max_dist / distance;
}

double GetDistance(CtrlPt *pt1, CtrlPt *pt2)
{
   double xdiff, ydiff, zdiff;
   
   xdiff = (pt1->x - pt2->x);
   xdiff *= xdiff;

   ydiff = (pt1->y - pt2->y);
   ydiff *= ydiff;

   zdiff = (pt1->z - pt2->z);
   zdiff *= zdiff;
   
   return sqrt(xdiff + ydiff + zdiff);
}
/* -library_code_end */
