
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
  * Revision 1.1.1.1  2003/03/03 14:12:52  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for orthoimage
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lorthoimage
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include "Database.h"
#include "viff.h"
/* -library_includes_end */

/****************************************************************
* 
*  Routine Name: lorthoimage - Derivation of an orthophoto from a DEM, an image and the orientation parameters.
* 
*       Purpose: .LP
This routine calculates an orthophoto of an image. As input it requires
the original image, the interior and exterior orientation parameters and
the digital elevation model of the area.
.LP
The elevations of the orthophoto pixels are determined by bilinear
interpolation in the DEM. The method for interpolation of the grey values
can be specified as one of: nearest neighbour, bilinear, or bicubic.
*         Input: .IP "image" 15
input image in a VIFF structure.
.IP "dem" 15
elevation data in a VIFF structure.
.IP "ortho" 15
empty orthophoto of correct size in a VIFF structure.
.IP "interior" 15
interior orientation data
.IP "exterior" 15
exterior orientation data
.IP "area" 15
method used for area selection
.IP "interpolation"
method for grey value interpolation
*        Output: .IP "ortho" 15
orthophoto in a VIFF structure.
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: The original image should by of type BYTE, the image containing the elevation
data should be of type FLOAT.
*    Written By: George Vosselman
*          Date: Jul 27, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lorthoimage(xvimage *image, xvimage *dem, xvimage *ortho,
                Interior *interior, Exterior *exterior,
                int area, int interpolation)
/* -library_def_end */

/* -library_code */
{
  unsigned char *orthopixel;
  int           ir, ic, ok, first_outside_dem=1, first_outside_image=1;
  float         *demdata, height;
  CamPt         campt;
  ObjPt         objpt;
  double        rdem, cdem, rimage, cimage;

  int f_resample(float *, double, double, int, int, float *, int);
  int uc_resample(unsigned char *, double, double, int, int,
                  unsigned char *, int);
  int lobj2cam(ObjPt *, Interior *, Exterior *, CamPt *);
  void Metric_To_Pix(Interior *, double, double, double *, double *);
                        
  objpt.y = ortho->fspare2 + (ortho->col_size - 0.5) * ortho->pixsizy;

  for (ir = 0, orthopixel = (unsigned char *) ortho->imagedata;
       ir < ortho->col_size;
       ir++, objpt.y -= ortho->pixsizy) {
    rdem    = dem->col_size - 1.0 - (objpt.y - dem->fspare2) / dem->pixsizy;
    objpt.x = ortho->fspare1 + ortho->pixsizx / 2.0;

    for (ic = 0;
         ic < ortho->row_size;
         ic++, objpt.x += ortho->pixsizx, orthopixel++) {
      cdem = (objpt.x - dem->fspare1) / dem->pixsizx;
      ok   = f_resample((float *) dem->imagedata, rdem, cdem,
                        dem->col_size, dem->row_size,
                        &height, 2);

/* Orthophoto pixel outside DEM */

      if (!ok) {
        if (first_outside_dem) {
          if (area != 2) {
            printf("Warning: Orthophoto pixel(s) outside DEM range!\n");
            printf("         Pixels outside DEM are assigned grey value 255\n");
          }
          first_outside_dem = 0;
        }
        *orthopixel = 255;
      }

/* Project terrain point into the image and interpolate grey value */

      else {
        objpt.z = (double) height;
        lobj2cam(&objpt, interior, exterior, &campt);
        Metric_To_Pix(interior, campt.x, campt.y, &rimage, &cimage);
        ok = uc_resample((unsigned char *) image->imagedata, rimage, cimage,
                         image->col_size, image->row_size,
                         orthopixel, interpolation);
        if (!ok) {
          if (first_outside_image) {
            if (area != 1) {
              printf("Warning: Orthophoto pixel(s) outside image range!\n");
              printf("         Pixels outside image are assigned grey value 0\n");
            }
            first_outside_image = 0;
          }
          *orthopixel = 0;
        }
      }
    }
  }
  return( 1 );
}
/* -library_code_end */
