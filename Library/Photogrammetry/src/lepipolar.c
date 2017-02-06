
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
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:53  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for epipolar
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lepipolar
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Database.h"
#include "viff.h"
   
#ifndef MIN
#define min(a,b) ((a) > (b) ? (b) : (a))
#endif

struct _min_max {
  double x,y;
};
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lepipolar - This library function creates epipolar images from non-epipolar images
* 
*       Purpose: Library Routine for epipolar
*         Input: .br
  
.br
xvimage        *img_1 - First image
.br
Exterior       *ext_1 - Exterior orientation elements
.br
Interior       *int_1 - Interior orientation elements
.br
  
.br
xvimage        *img_2 - Second image
.br
Exterior       *ext_2 - Exterior orientation elements
.br
Interior       *int_2 - Interior orientation elements
*        Output: .br
  
.br
.in +0.5c
.ul 2
The output images should not be allocated, as memory will be allocated
for them by the library routine.
.in -0.5c
.br
xvimage        *nor_1 - Epipolar version of first image
.br
xvimage        *nor_2 - Epipolar version of second image
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: Ir. R.Th. Ursem
*          Date: Jul 28, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
int lepipolar(xvimage *img_1, Exterior *ext_1, Interior *int_1,
              xvimage *img_2, Exterior *ext_2, Interior *int_2,
              xvimage **nor_1, xvimage **nor_2)
/* -library_def_end */

/* -library_code */
{
  Exterior base;             /* Base transformation elements             */
  double Rn1[3][3];          /* Normalized rotation matrix  Rn = Rb . R^ */
  double Rn2[3][3];          /* Normalized rotation matrix  for camera 2 */
  double Rt[3][3];
  double size_1c, size_1r;   /* Size of the epipolar image (image 1)     */
  double size_2c, size_2r;   /* Size of the epipolar image (image 2)     */
  int i, j, error;           /* Loop counters                            */
  double x_org[4], y_org[4],
         x_res[4], y_res[4];
  double var[8];
  double c1[8], c2[8];
  double min_y, max_y;
  double xh, yh;
  double spacing;
  int r, c;
  AdjustInfo info;
  char tmpstr[80];
  
  unsigned char *data;       /* Pointer to the image data                */
  struct _min_max org1[4], org2[4], epi1[4], epi2[4], max1, max2, min1, min2;
  
  void mattrans(double *, int, int, double *);
  void matmult(double *, double *, double *, int, int, int);
  int Calculate_Epipolar_Size(Interior *, xvimage *, double Rn[3][3],
                              struct _min_max *, struct _min_max *);
  int Cal_Trans_Pars(double *, double *, double *, double *,
                   double *, int, AdjustInfo *, double par_out[8]);
  int Resample_Image(xvimage *, xvimage **, Interior *, double par[8], 
                   double, double, double, double, int, int);
  int Calculate_Principal_Point(Interior *, double, double);
                                                
/*------ Calculation of the difference in position of the two cameras ------*/
  base.status = ALL_VALID;

  base.x = ext_2->x - ext_1->x;
  base.y = ext_2->y - ext_1->y;
  base.z = ext_2->z - ext_1->z;
  
  if (base.x < 0.000000001 && base.y < 0.000000001)
    {
    fprintf( stderr, "Both images were taken from the same position, with only\n");
    fprintf( stderr, "a difference in the height. This situation can not be processed\n");
    fprintf( stderr, "by this program.\n");
    return( False );
    }
  
/*------ Calculate the rotation angles from true vertical to epipolar image -----*/
  base.k[0] = atan( base.y / base.x );
  base.f[0] = -1.0 * atan( base.z / (sqrt( base.x*base.x + base.y*base.y )) );
  
  if (ext_1->status & INVALID_ANGLES)
    {
    fprintf( stderr, "\tThe status byte of exterior orientation structure indicates\n");
    fprintf( stderr, "\tthat the calculated rotation angles of camera 1 are invalid.\n");
    fprintf( stderr, "\tThe calculation of epipolar images can not be completed.\n");
    return( False );
    }
  
  if (ext_2->status & INVALID_ANGLES)
    {
    fprintf( stderr, "\tThe status byte of exterior orientation structure indicates\n");
    fprintf( stderr, "\tthat the calculated rotation angles of camera 2 are invalid.\n");
    fprintf( stderr, "\tThe calculation of epipolar images can not be completed.\n");
    return( False );
    }
    
  base.w[0] = ( ext_1->w[0] + ext_2->w[0] ) / 2.0;
  
  base.w[1] = base.k[1] = base.f[1] = -1.0;

/*------ Calculate the rotation matrix, based on the above calculated angles ------*/
  Rot_From_Angles(&base);
  Quat_From_Rot(&base);
  
  printf("------Basic transformation parameters:\n");
  Print_Exterior(&base);
  
  mattrans((double *) base.rot, 3, 3, (double *) Rt);

/*------ Calculate the transformation matrices from pixel to epipolar image ------*/
  matmult((double *) Rt, (double *) ext_1->rot, (double *) Rn1, 3, 3, 3);
  matmult((double *) Rt, (double *) ext_2->rot, (double *) Rn2, 3, 3, 3);

/*                                  _\\|//_ 
                                    ( .-. )
---------------------------------o00--(_)--00o------------------------------------
      With this result we can calculate the position of any pixel in the 
      epipolar image by applying a projective transformation, and determine the
      grey value of that pixel by bilinear interpolation in the original image.
      
      The next thing to do is to determine the size of the epipolar images by
      transforming the four corner points of each image, using the previously
      derived transformation parameters. After the transformation of the four
      corner points, the size of the epipolar images can be determined by deter-
      mining the minimum and maximum values of the row and column coordinates.
      
      From the size of the original image and the size of the epipolar image a
      stepsize for the resampling can be calculated.
----------------------------------------------------------------------------------
*/
  spacing = min( int_2->spacing_r, min( int_2->spacing_c, min(int_1->spacing_c, int_1->spacing_r)));
  int_1->spacing_c = spacing;
  int_1->spacing_r = spacing;
  int_2->spacing_c = spacing;
  int_2->spacing_r = spacing;
  
  Calculate_Epipolar_Size(int_1, img_1, Rn1, (struct _min_max *) &org1,
                          (struct _min_max *) &epi1);
  Calculate_Epipolar_Size(int_2, img_2, Rn2, (struct _min_max *) &org2,
                          (struct _min_max *) &epi2);
  
  /*------ Determine minima and maxima of all coordinates ------*/
  min1.x = min2.x = min1.y = min2.y =  1000;
  max1.x = max2.x = max1.y = max2.y = -1000;
  
  for (i = 0; i < 4; i++)
  {
     if (epi1[i].x > max1.x) max1.x = epi1[i].x;
     if (epi2[i].x > max2.x) max2.x = epi2[i].x;
     
     if (epi1[i].y > max1.y) max1.y = epi1[i].y;
     if (epi2[i].y > max2.y) max2.y = epi2[i].y;
     
     if (epi1[i].x < min1.x) min1.x = epi1[i].x;
     if (epi2[i].x < min2.x) min2.x = epi2[i].x;
     
     if (epi1[i].y < min1.y) min1.y = epi1[i].y;
     if (epi2[i].y < min2.y) min2.y = epi2[i].y;
  }
  
  /*------ With the determined minima and maxima, calculate the size in pix ------*/
  size_1c = (max1.x - min1.x) / int_1->spacing_c;
  size_2c = (max2.x - min2.x) / int_2->spacing_c;
  
  max_y = (max1.y > max2.y) ? max1.y : max2.y;
  min_y = (min1.y < min2.y) ? min1.y : min2.y;
  size_1r = size_2r = (max_y - min_y) / int_1->spacing_r;
  
  /*------ Inform the user what we have accomplished so far ------*/
  printf("\nDetermined image sizes:\n");
  printf("\timage 1 -> %1.0lf x %1.0lf pixels\n", size_1r, size_1c);
  printf("\timage 2 -> %1.0lf x %1.0lf pixels\n", size_2r, size_2c);
  printf("\n");
  
/*                                  _\\|//_ 
                                    ( .-. )
---------------------------------o00--(_)--00o------------------------------------
      Calculate the transformation parameters between the epipolar image and 
      the orignal image. With these parameters, for every pixel in the epipolar 
      image the corresponding point in the original image can be determined.
      The determination of the parameters is done with a least squares adjustment
      given the four corner points of both the epipolar and the original image
----------------------------------------------------------------------------------
*/

  for (i = 0; i < 8; i++) var[i] = 1.0;
  
  info.numbers = (int *)calloc( 4, sizeof(int) );
  info.output = stdout;
  info.n_o_p_p = 2;
  info.var_num_obs = 0;
  
  for (i = 0; i < 4; i++)
  {
     x_org[i] = org1[i].x;
     y_org[i] = org1[i].y;
     x_res[i] = epi1[i].x;
     y_res[i] = epi1[i].y;
     info.numbers[i] = i + 1;
  }
  
  strcpy(tmpstr, "millimeters");
  info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
  strcpy(info.units, tmpstr);
  
  strcpy(tmpstr, "x and y coordinates");
  info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
  strcpy(info.order_string, tmpstr);
  
  info.order_char = (char *)calloc(2, sizeof(char) );
  info.order_char[0] = 'x';
  info.order_char[1] = 'y';
  
  error = Cal_Trans_Pars(x_org, y_org, x_res, y_res, var, 4, &info, c1);
  if (error != 0) return( False );

  for (i = 0; i < 4; i++)
  {
     x_org[i] = org2[i].x;
     y_org[i] = org2[i].y;
     x_res[i] = epi2[i].x;
     y_res[i] = epi2[i].y;
  }
  error = Cal_Trans_Pars(x_org, y_org, x_res, y_res, var, 4, &info, c2);
  if (error != 0) return( False );
    
/*                                  _\\|//_ 
                                    ( .-. )
---------------------------------o00--(_)--00o------------------------------------
      Now we know what the new size of the image is, we can determine the value 
      of every pixel in the epipolar image by bilinear interpolation in the 
      original pixel image.
----------------------------------------------------------------------------------
*/
  
  printf("Creating epipolar version of first image\n");
  Resample_Image(img_1, nor_1, int_1, c1, min1.x, max1.x, min_y, max_y, 
                 (int) ceil(size_1r), (int) ceil(size_1c)+1);
  
  printf("Creating epipolar version of second image\n\n");
  Resample_Image(img_2, nor_2, int_2, c2, min2.x, max2.x, min_y, max_y, 
                 (int) ceil(size_2r), (int) ceil(size_2c)+1);

  /*------ calculate the new location of the principal point ------*/
  Calculate_Principal_Point(int_1, min1.x, max_y);
  Calculate_Principal_Point(int_2, min2.x, max_y);
  
  return( True );
}

/*                           _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Calculate_Epipolar_Size
  Author        : R.Th. Ursem
  Creation date : 8 - Nov 1994
  
  Input  : Interior orientation data, the image and rotation matrix
  Output : Size of the epipolar image
  
---------------------------------------------------------------------
*/
int Calculate_Epipolar_Size(Interior *interior, xvimage *image, double Rn[3][3],
                            struct _min_max *org, struct _min_max *epi)
{
  double t_x, t_y;       /* Temporary storage for coordinates   */
  int i;
  
  void Pix_To_Metric(Interior *, double, double, double *, double *);
  
/*------ Convert the pixel coordinates of the original image to mm ------*/
  org[0].x = org[0].y = org[1].x = org[2].y = 0.0;
  org[3].x = org[2].x = (double) image->col_size;
  org[3].y = org[1].y = (double) image->row_size;
  
  Pix_To_Metric(interior, org[0].x, org[0].y, &(epi[0].x), &(epi[0].y) );
  Pix_To_Metric(interior, org[1].x, org[1].y, &(epi[1].x), &(epi[1].y) );
  Pix_To_Metric(interior, org[2].x, org[2].y, &(epi[2].x), &(epi[2].y) );
  Pix_To_Metric(interior, org[3].x, org[3].y, &(epi[3].x), &(epi[3].y) );
  
/*------ Then apply a transformation to the epipolar image (in mm) ------*/
  for (i = 0; i < 4; i++)
    {
    t_x  = (Rn[0][0] * epi[i].x + Rn[0][1] * epi[i].y - Rn[0][2] * interior->cc);
    t_x /= (Rn[2][0] * epi[i].x + Rn[2][1] * epi[i].y - Rn[2][2] * interior->cc);
    t_x *= (-1.0 * interior->cc);
    
    t_y  = (Rn[1][0] * epi[i].x + Rn[1][1] * epi[i].y - Rn[1][2] * interior->cc);
    t_y /= (Rn[2][0] * epi[i].x + Rn[2][1] * epi[i].y - Rn[2][2] * interior->cc);
    t_y *= (-1.0 * interior->cc);

    org[i].x = epi[i].x; org[i].y = epi[i].y;
    epi[i].x = t_x;      epi[i].y = t_y;
    }
  
/*------ Determine minimum and maximum of the coordinates ------*/

  printf("\n\t\tOriginal\t\t      Epipolar\n");
  for (i = 0; i < 4; i++)
     printf("%15.3lf%15.3lf%15.3lf%15.3lf\n", org[i].x, org[i].y, epi[i].x, epi[i].y);

  return( True );
}

/*                           _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Calculate_Principal_Point
  Author        : R.Th. Ursem
  Creation date : 20 - Mrt 1994
  
  Input  : Interior orientation data and rotation matrix
  Output : Interior orientation data
  
---------------------------------------------------------------------
*/
int Calculate_Principal_Point(Interior *interior, double min_x, double max_y)
  {
  interior->ch = fabs(min_x) / interior->spacing_c;
  interior->rh = fabs(max_y) / interior->spacing_r;

  interior->k1 = interior->k2 = interior->k3 =
     interior->p1 = interior->p2 = 0.0;
  
  printf("New position of principle point (%1.1lf,%1.1lf)\n", interior->rh, interior->ch);
  
  return( True );
  }

/*                           _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Resample_Image
  Author        : R.Th. Ursem
  Creation date : 8 - Nov 1994
  
---------------------------------------------------------------------
*/
int Resample_Image(xvimage *img, xvimage **nor, Interior *in, double par[8], 
                   double min_x, double max_x, double min_y, double max_y,
                   int size_r, int size_c)
{
   unsigned char *data;   /* Pointer to data of epipolar image   */
   double x_epi,  y_epi;  /* Coordinates in epipolar image (mm)  */
   int    r,      c;      /* Coordinates in epipolar image (pix) */
   double x_real, y_real; /* Coordinates in original image (mm)  */
   double r_real, c_real; /* Coordinates in original image (pix) */

   double  resample(unsigned char *, double, double, int);
   xvimage *createimage(unsigned long, unsigned long, unsigned long,
                        unsigned long, unsigned long, char *,
                        unsigned long, unsigned long, unsigned long,
                        unsigned long, unsigned long, unsigned long);
   void Metric_To_Pix(Interior *, double, double, double *, double *);
                           
   /*------ Allocate space for the new image ------*/
   *nor = createimage(size_r, size_c, VFF_TYP_1_BYTE, 1, 1, 
                      "Created by epipolar", 0, 0,
                      VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
   data = (unsigned char *)(*nor)->imagedata;
   
   for (y_epi = max_y, r = 0; y_epi > min_y; y_epi -= in->spacing_r, r++) {
      for (x_epi = min_x, c = 0; x_epi < max_x; x_epi += in->spacing_c, c++) {

         /*------ Transform epipolar coordinates to original image ------*/

         x_real  = (par[0] * x_epi + par[1] * y_epi + par[2]);
         x_real /= (par[6] * x_epi + par[7] * y_epi + 1.0);
         
         y_real  = (par[3] * x_epi + par[4] * y_epi + par[5]);
         y_real /= (par[6] * x_epi + par[7] * y_epi + 1.0);
         
         /*------ Transform the position in mm to pixels ------*/

         Metric_To_Pix(in, x_real, y_real, &r_real, &c_real);
         
         if (r_real < 0             || c_real < 0 || 
             r_real > img->col_size || c_real > img->row_size)
            /*------ Point is outside the original image ------*/
            *(data + r * (*nor)->row_size + c) = 0;
         else
            /*------ Point is inside the original image, resample ------*/
            *(data + r * (*nor)->row_size + c) = 
               (unsigned char) (resample((unsigned char *) img->imagedata,
                                         r_real, c_real, img->row_size) + 0.5);
      }
   }
   return(1);
}
/* -library_code_end */
