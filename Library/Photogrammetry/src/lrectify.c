
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
  * Revision 1.2  2003/11/05 15:58:40  fotsoft
  * Added include file <math.h>. Without this "ceil" did not work properly.
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
   >>>> 	Library Routine for rectify
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lrectify
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */


#include "internals.h"

/* -library_includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Database.h"
#include "viff.h"
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lrectify - Rectification of a viff image. This routine accepts a number of control point coordinates (row,column) of the original and the resampled image and the number of rows and columns of the resampled image. After choosing one of the three interpolation methods (nearest neighbour, bilinear interpolation or cubic convolu- tion), it calculates the parameters of the projective trans- formation and applies the resampling routine. Output is the resampled viff image.
* 
*       Purpose: Structuring of the four subroutines parcal.c, neigh.c,
biline.c and cucon.c, which calculate the parameters for
the projective transformation and resample the image with
nearest neighbour or bilinear interpolation or cubic
convolution (as desired).
Formulars used for the interpolation:
 - Nearest neighbour:
.br
  r = (int) (rt + 0.5)
  c = (int) (ct + 0.5)
.br
 - Bilinear interpolation:
.br
  g1 = (rb - rt) * g(ra,ca) + (rt - ra) * g(rb,ca)
  g2 = (rb - rt) * g(ra,cb) + (rt - ra) * g(rb,cb)
   g = (cb - ct) * g1       + (ct - ca) * g2
.br
 - Cubic convolution: (S = sum)
.br 
  g = S(-1 <= u <= 2) p(u-rt+r2) * 
      S(-1 <= v <= 2) p(v-ct+c2) * g(r2+u,c2+v)
  
  p(z) =  |z|**3 - 2*|z|**2 + 1           for  0<=|z|<1 
  p(z) = -|z|**3 + 5*|z|**2 - 8*|z| + 4   for  1<=|z|<2
  p(z) =  0                               for  2<=|z|
.br
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: Evelyn Koeller
Modifications by R.Th. Ursem
*          Date: May 18, 2002
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lrectify(ImgPts *org, CtrlPts *ctrl, xvimage *viff_in, int resam, int area,
             double x_min, double x_max, double y_min, double y_max, 
             int border_color, double pix_size, xvimage **viff_out,
             char *par_file)
/*-----------------------------------------------------------------------------
 * Input parameters:
 *   resam             - desired resampling method:
 *                          1 = nearest neighbour
 *                          2 = bilinear interpolation
 *                          3 = cubic convolution
 *   border_color      - grey value to be assigned to pixels inside viff_out but
 *                       outside viff_in
 *   viff_in           - original grey value image
 *   org               - image points measured in original image
 *   ctrl              - control points in metric system
 *
 * Output parameters:
 *   viff_out          - resampled image
 *   par_file          - file with transformation parameters
 *
 *-----------------------------------------------------------------------------*/
/* -library_def_end */

/* -library_code */
{  
   int        size;
   int        error, i, j;
   double     x2pix[8], pix2x[8];
   int        valid_points = 0;
   double     *x_org, *y_org, *x_res, *y_res, *var;
   double     img_r[4], img_c[4], obj_x[4], obj_y[4]; /* Area corners */
   double     img_r2[4], img_c2[4], var2[8];
   AdjustInfo info;
   char       tmpstr[80];
   FILE       *fd;

   void    Apply_Trans();
   xvimage *createimage(unsigned long, unsigned long, unsigned long,
                        unsigned long, unsigned long, char *,
                        unsigned long, unsigned long, unsigned long,
                        unsigned long, unsigned long, unsigned long);
   int Cal_Trans_Pars(double *, double *, double *, double *,
                   double *, int, AdjustInfo *, double par_out[8]);
   int Robust_Cal_Trans_Pars(double *, double *, double *, double *, double *,
                          int, AdjustInfo *, double par[8], int, double, int);                
   void Apply_Trans(double *, double *, double, double, double *); 
   int neigh(xvimage *, xvimage *, double, double, double, double, double,
             double [8], int);
   int biline(xvimage *, xvimage *, double, double, double, double, double,
              double [8], int);
   int cucon(xvimage *, xvimage *, double, double, double, double, double,
             double [8], int);
   int Put_ProjTrans2D(const char *par_file, const double *);
                     
   Print_ImgPts(org);
   Print_CtrlPts(ctrl);
   
   size = ctrl->num_pts;
   x_org = (double *)calloc(size, sizeof(double));
   y_org = (double *)calloc(size, sizeof(double));
   x_res = (double *)calloc(size, sizeof(double));
   y_res = (double *)calloc(size, sizeof(double));
   var   = (double *)calloc(2*size, sizeof(double));
   info.numbers = (int *)calloc(size, sizeof(int));
   
   for (i = 0; i < org->num_pts; i++)
      for (j = 0; j < ctrl->num_pts; j++)
         if ((org->pts[i].num == ctrl->pts[j].num) &&
             (ctrl->pts[j].status == PLANI_CTRL ||
              ctrl->pts[j].status == FULL_CTRL))
         {
            x_org[valid_points]   = org->pts[i].r;
            y_org[valid_points]   = org->pts[i].c;
            
            if (org->pts[i].v_r == 0 || org->pts[i].v_c == 0)
               printf("WARNING: Variance factor of point %d equals 0, replaced by 0.5\n", org->pts[i].num);
            var[2*valid_points]   = (org->pts[i].v_r == 0) ? 0.5 : org->pts[i].v_r;
            var[2*valid_points+1] = (org->pts[i].v_c == 0) ? 0.5 : org->pts[i].v_c;
            
            x_res[valid_points]   = ctrl->pts[j].x;
            y_res[valid_points]   = ctrl->pts[j].y;
            
            info.numbers[valid_points] = org->pts[i].num;
            valid_points++;
         }
         
   /*------ Filling of the info structure for adjust ------*/
   info.output = stdout;
   info.n_o_p_p = 2;
   info.var_num_obs = 0;
   
   strcpy(tmpstr, "pixels");
   info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.units, tmpstr);
   
   strcpy(tmpstr, "row and column coordinate");
   info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.order_string, tmpstr);
   
   info.order_char = (char *)malloc( 2 * sizeof(char) );
   info.order_char[0] = 'r';
   info.order_char[1] = 'c';
   
   /* Calculate transformation parameters from metric to pixel system */

   error = Robust_Cal_Trans_Pars(x_org, y_org, x_res, y_res, var, valid_points,
                          &info, x2pix, 0, 10000.0, 3); /* Set to NOT robust */
   if (error != 0) return( False );

   if (area == 1 || par_file) {

      /* Calculate the inverse transformation */

      error = Cal_Trans_Pars(x_res, y_res, x_org, y_org, var, valid_points,
                             NULL, pix2x);
      if (error != 0) return( False );

   }

   if (area == 1) { /* Entire photograph */
      /* Set the image corners */

      img_r[0] = -0.5;
      img_r[1] = -0.5;
      img_r[2] = (double) viff_in->col_size - 0.5;
      img_r[3] = (double) viff_in->col_size - 0.5;
      img_c[0] = -0.5;
      img_c[1] = (double) viff_in->row_size - 0.5;
      img_c[2] = -0.5;
      img_c[3] = (double) viff_in->row_size - 0.5;

      /* Determine approximate locations of image corners in terrain */

      for (i=0; i<4; i++)
        Apply_Trans(obj_x+i, obj_y+i, img_r[i], img_c[i], pix2x);

      /* Determine the exact corresponding locations in the image of these
       * points.
       */

      for (i=0; i<4; i++)
        Apply_Trans(img_r2+i, img_c2+i, obj_x[i], obj_y[i], x2pix);

      /* Calculate the exact inverse transformations with these points */

      for (i=0; i<8; i++) var2[i] = 1.0;
      error = Cal_Trans_Pars(obj_x, obj_y, img_r2, img_c2, var2, 4,
                             NULL, pix2x);
      if (error != 0) return( False );

      /* Determine the exact terrain position of the image corners */

      for (i=0; i<4; i++)
        Apply_Trans(obj_x+i, obj_y+i, img_r[i], img_c[i], pix2x);

      /* Determine the bounding box */

      y_min = x_min =  1.0e20;
      x_max = y_max = -1.0e20;
      for (i = 0; i < 4; i++) {
         if (obj_x[i] < x_min) x_min = obj_x[i];
         if (obj_y[i] < y_min) y_min = obj_y[i];
         if (obj_x[i] > x_max) x_max = obj_x[i];
         if (obj_y[i] > y_max) y_max = obj_y[i];
      }
   }

   printf("Area to rectify:\n");
   printf("(%5.1lf,%5.1lf)+------+(%5.1lf,%5.1lf)\n",
          x_min, y_max, x_max, y_max);
   for (i=0; i<3; i++) printf("             |      |\n");
   printf("(%5.1lf,%5.1lf)+------+(%5.1lf,%5.1lf)\n\n",
          x_min, y_min, x_max, y_min);
   printf("New image size = %d x %d pixels\n", 
          (int) ceil((y_max - y_min) / pix_size), 
          (int) ceil((x_max - x_min) / pix_size));

   *(viff_out) = createimage((int) ceil((y_max - y_min) / pix_size), 
                             (int) ceil((x_max - x_min) / pix_size),
                             VFF_TYP_1_BYTE, 1, 1, 
                             "Created by rectify", 0, 0,
                             VFF_MS_NONE, VFF_MAPTYP_NONE, 
                             VFF_LOC_IMPLICIT, 0);
   if ((*viff_out) == NULL) {
      printf("Could not allocate enough memory for output image\n");
      exit(0);
   }

   /* Calling the desired resampling routine */
   printf("Resampling\n");
   switch (resam) { 
        case 1:
           neigh (viff_in, (*viff_out), x_min, x_max, 
                  y_min, y_max, pix_size, x2pix, border_color);
           break;
        case 2:
           biline(viff_in, (*viff_out), x_min, x_max, 
                  y_min, y_max, pix_size, x2pix, border_color);
           break;
        case 3:
           cucon (viff_in, (*viff_out), x_min, x_max, 
                  y_min, y_max, pix_size, x2pix, border_color);
           break;
        default:
           printf("Error by calling resampling routine\n");
           break;
    }

    if (par_file) Put_ProjTrans2D(par_file, pix2x);

return(True);
}
   
void Apply_Trans(double *x_org, double *y_org, double x_res, double y_res,
                 double *par)
{
   double denom = (x_res * par[6] + y_res * par[7] + 1);
   *x_org = (x_res * par[0] + y_res * par[1] + par[2]) / denom;
   *y_org = (x_res * par[3] + y_res * par[4] + par[5]) / denom;
}
/* -library_code_end */
