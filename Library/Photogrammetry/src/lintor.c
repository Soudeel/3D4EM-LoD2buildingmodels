
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
  * Revision 1.1  2006/04/06 14:10:17  WINDOWSNT\vosselman
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
  * Revision 1.1.1.1  2003/03/03 14:11:30  rabbani
  * Routines for image orientation
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for intor
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lintor
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include <math.h>
#include "Database.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lintor - This program determines the parameters of the Helmert transformation between the pixel coordinate system and the camera system using the coordinates of the fiducial marks in both systems.
* 
*       Purpose: This program determines the parameters of the Helmert transformation between
the pixel coordinate system and the camera system using the coordinates of
the fiducial marks in both systems.
-short_lib_description_end

-man1_long_description
This program determines the parameters of the Helmert transformation between
the pixel coordinate system of a digital image and the camera coordinate system of the scanned photograph. A least squares adjustment is performed using the
coordinates of the fiducial marks in both coordinate systems.

The relation between the pixel coordinate system and the photo coordinate system
is given by:

      ( x )              ( s_c   0  )   (  c - c0  )
      (   )  =  R_alpha  (          )   (          )
      ( y )              (  0   s_r )   (  r0 - r  )

with photo coordinates (x,y), pixel coordinates (r,c), rotation matrix
(R_alpha), pixel spacings (s_r, s_c), and origin of the photo coordinate system
in the pixel coordinate ksystem (r0, c0).

The parameters of the interior orientation of the camera with respect to the
photo coordinate ksystem (as they can be obtained from the camera calibration
protocol) have to be specified by the user. This information is used to
calculate the interior orientation parameters of the digitized image.
*         Input: .IP "imgpts" 15
structure containing the pixel coordinates of the fiducial marks.
.IP "photopts" 15
structure containing the photo coordinates of the fiducial marks.
.IP "interior" 15
structure containing the interior orientation parameters as provided by the
user.
.IP "xpp" 15
float x-coordinate of principal point in photo coordinate system.
.IP "ypp" 15
float y-coordinate of principal point in photo coordinate system.
*        Output: .IP "interior" 15
structure containing the interior orientation parameters as provided by the
user, updated with the principal point in the pixel coordinate system,
the pixel spacings and the rotation of the scanned photograph.
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: The rotation between the pixel and photo coordinate system should be around
0 degrees.
*    Written By: George Vosselman
*          Date: Aug 18, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lintor(ImgPts *imgpts, CamPts *photopts, Interior *interior, 
           float xpp, float ypp)
/* -library_def_end */

/* -library_code */
{
   /*------ Declaration of functions ------*/
   int LS_Interior(ImgPts *, CamPts *, int, double *, double *, double *,
                   double *, double *);
   
   /*------ Variable list ------*/
   double rmin, rmax, cmin, cmax, xmin, xmax, ymin, ymax,
          r0, c0, rs, cs, rot, xrot, yrot;
   int    i, j, num_pts;
   
   /*------ Start of library function ------*/

   /* Get the number of corresponding points and the coordinate ranges */

   num_pts = 0;
   printf("Using fiducial marks: ");
   /* modification 18-11-96 by F. van den Heuvel:
      Initialization of rmin, cmin, xmin and ymin. */
   rmin = rmax = imgpts->pts[0].r;
   cmin = cmax = imgpts->pts[0].c;
   xmin = xmax = photopts->pts[0].x;
   ymin = ymax = photopts->pts[0].y;
   /* modification end */
   for (i = 0; i < imgpts->num_pts; i++)
      for (j = 0; j < photopts->num_pts; j++)
         if (imgpts->pts[i].num == photopts->pts[j].num)
            {
               printf(" %d", imgpts->pts[i].num);
               num_pts++;
               if (imgpts->pts[i].r < rmin || num_pts == 0) rmin = imgpts->pts[i].r;
               if (imgpts->pts[i].r > rmax || num_pts == 0) rmax = imgpts->pts[i].r;
               if (imgpts->pts[i].c < cmin || num_pts == 0) cmin = imgpts->pts[i].c;
               if (imgpts->pts[i].c > cmax || num_pts == 0) cmax = imgpts->pts[i].c;
               if (photopts->pts[j].x < xmin || num_pts == 0) xmin = photopts->pts[j].x;
               if (photopts->pts[j].x > xmax || num_pts == 0) xmax = photopts->pts[j].x;
               if (photopts->pts[j].y < ymin || num_pts == 0) ymin = photopts->pts[j].y;
               if (photopts->pts[j].y > ymax || num_pts == 0) ymax = photopts->pts[j].y;
            }
   printf("\n");
   if (num_pts < 3)
   {
      printf("lintor: At least three fiducial marks are needed\n");
      return( 0 );
   }
   
   /* Establish approximate values for the Helmert transformation */

   r0 = (rmax + rmin) / 2.0;             /* Row origin in photo system        */
   c0 = (cmax + cmin) / 2.0;             /* Column origin in photo system     */
   rs  = (ymax - ymin) / (rmax - rmin);  /* Pixel spacing in row direction    */
   cs  = (xmax - xmin) / (cmax - cmin);  /* Pixel spacing in column direction */
   rot = 0.0;                          /* Rotation between coordinate systems */

   if (!LS_Interior( imgpts, photopts, num_pts, &r0, &c0, &rs, &cs, &rot))
   {
      printf("lintor: ERROR when calculating exact values.\n");
      return 0;
   }
   
   /* Correct for principal point in camera coordinate system and
      put values into the structure. */

   xrot = xpp * cos(rot) - ypp * sin(rot);
   yrot = xpp * sin(rot) + ypp * cos(rot);
   interior->rh = r0 - yrot / rs;
   interior->ch = c0 + xrot / cs;
   interior->spacing_r = rs;
   interior->spacing_c = cs;
   interior->rot       = rot;

   Print_Interior(interior);
   
   return ( 1 );
}
/* -library_code_end */
