
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



#include "viff.h"
#include "Database.h"

/*
 *----------------------------------------------------------------------------
 * Name    : get_ortho_size.c
 * Author  : George Vosselman
 * Version : 1.0
 * Date    : 04-09-95
 * Function: Calculation of the orthophoto size based on
 *           1 - the size of the image projected into the terrain,
 *           2 - the size of the supplied DEM, or
 *           3 - the boundaries specified by the user.
 *----------------------------------------------------------------------------
 */

int get_ortho_size(xvimage *image, xvimage *dem, xvimage *ortho,
                   Interior *interior, Exterior *exterior,
                   int toggle,
                   float xmin, float xmax, float ymin, float ymax,
                   float pixelsize)

/* NOTE: xvimage elements fspare1 and fspare2 are used to store the float
 *       coordinates of the lower left corner of the lower left pixel. The
 *       elements startx and starty can not be used for this purpose since
 *       they are integer.
 */

{
  int    ir, ic, i;
  double r[4], c[4], x, y, X, Y, Xmin, Xmax, Ymin, Ymax, Zmin;
  float  *Zptr;

  void Pix_To_Metric(Interior *, double, double, double *, double *);
  
  ortho->pixsizx =
  ortho->pixsizy = pixelsize;

  switch (toggle) {

/* Entire image */

    case (1) : /* Get the lowest DEM point */
               Zmin = 1.0e30;
               for (ir=0, Zptr = (float *) dem->imagedata; ir < dem->col_size-1; ir++) {
                 for (ic=0; ic < dem->row_size-1; ic++, Zptr++) {
                   if (*Zptr < Zmin) Zmin = *Zptr;
                 }
               }

               /* Project the image corners onto the plane of the
                * lowest DEM point.
                */

               r[0] = r[1] = 0.0;
               r[2] = r[3] = (double) (image->col_size - 1);
               c[0] = c[2] = 0.0;
               c[1] = c[3] = (double) (image->row_size - 1);
               for (i=0; i<4; i++) {
                 Pix_To_Metric(interior, r[i], c[i], &x, &y);
                 X = (Zmin - exterior->z) *
                     (exterior->rot[0][0] * x +
                      exterior->rot[0][1] * y -
                      exterior->rot[0][2] * interior->cc) /
                     (exterior->rot[2][0] * x +
                      exterior->rot[2][1] * y -
                      exterior->rot[2][2] * interior->cc) +
                     exterior->x;
                 Y = (Zmin - exterior->z) *
                     (exterior->rot[1][0] * x +
                      exterior->rot[1][1] * y -
                      exterior->rot[1][2] * interior->cc) /
                     (exterior->rot[2][0] * x +
                      exterior->rot[2][1] * y -
                      exterior->rot[2][2] * interior->cc) +
                     exterior->y;
                 if (i == 0) {
                   Xmin = Xmax = X;
                   Ymin = Ymax = Y;
                 }
                 else {
                   if (X < Xmin) Xmin = X;
                   if (X > Xmax) Xmax = X;
                   if (Y < Ymin) Ymin = Y;
                   if (Y > Ymax) Ymax = Y;
                 }
               }

               /* Leave a border of 5 pixels around the derived rectangle. */

               ortho->fspare1  = Xmin - 5.0 * pixelsize;
               ortho->fspare2  = Ymin - 5.0 * pixelsize;
               ortho->row_size = (int) ((Xmax - Xmin + 10.0) / pixelsize + 0.5);
               ortho->col_size = (int) ((Ymax - Ymin + 10.0) / pixelsize + 0.5);
               break;

/* DEM range */

    case (2) : ortho->fspare1  = dem->fspare1 - dem->pixsizx / 2.0;
               ortho->fspare2  = dem->fspare2 - dem->pixsizy / 2.0;
               ortho->row_size = (int) (((float) dem->row_size * dem->pixsizx / pixelsize) + 0.5);
               ortho->col_size = (int) (((float) dem->col_size * dem->pixsizy / pixelsize) + 0.5);
               break;

/* User specified range */

    case (3) : ortho->fspare1  = xmin;
               ortho->fspare2  = ymin;
               ortho->row_size = (int) ((xmax - xmin) / pixelsize + 0.5);
               ortho->col_size = (int) ((ymax - ymin) / pixelsize + 0.5);
               break;
  }

  printf("Size of orthophoto: %d rows x %d columns.\n",
         ortho->col_size, ortho->row_size);
  printf("Pixelsize: %f\n",pixelsize);
  printf("Centre of lower left pixel at (%f,%f)\n",
         ortho->fspare1 + pixelsize/2.0, ortho->fspare2 + pixelsize/2.0);
  printf("Lower left corner of that pixel at (%f,%f)\n",
         ortho->fspare1, ortho->fspare2);
  printf("Upper right corner of upper right pixel at (%f,%f)\n",
         ortho->fspare1 + pixelsize * ortho->row_size,
         ortho->fspare2 + pixelsize * ortho->col_size);

  return(1);
}
