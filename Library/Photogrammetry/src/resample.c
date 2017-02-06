
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



#include <stdio.h>
#include <stdlib.h>
#include "viff.h"

/* Quick and dirty bilinear resampling of a byte image without boundary
 * checking.
 */

double resample(unsigned char *p_g, double r, double c, int r_size)
{
   unsigned char *p1;
   int     i_r, i_c;
   double  dr, dc, g1, g2, temp1, temp2, retval;
   
   i_r     = (int) r;
   dr      = r - (double) i_r;
   i_c     = (int) c;
   dc      = c  - (double) i_c;
   p1      = p_g + (i_c + i_r * r_size);
   g1      = (double) *p1;
   g2      = (double) *(p1+1);
   temp1   = g1 + dc * (g2 - g1);
   p1     += r_size;
   g1      = (double) *p1;
   g2      = (double) *(p1+1);
   temp2   = g1 + dc * (g2 - g1);
   
   retval =  temp1 + dr * (temp2 - temp1); 
   return ( retval ); 
}

/*
----------------------------------------------------------------------------------
  This function is used to calculate the weight of a pixel in the
  cubic convolution interpolation.
  
  p(z) =  | z |**3 - 2 * | z |**2 + 1               for  0 <= | z | < 1 
  p(z) = -| z |**3 + 5 * | z |**2 - 8 * | z | + 4   for  1 <= | z | < 2
  p(z) =  0                                         for  2 <= | z |
----------------------------------------------------------------------------------
*/
double Polynom(double z)
{
  if (z >= 0.0) {
    if (z < 1.0)             return(   z * z * z  - 2.0 * z * z           + 1.0 );
    if (z >= 1.0 && z < 2.0) return( -(z * z * z) + 5.0 * z * z - 8.0 * z + 4.0 );
    /* else if (z > 2.0) */  return(                                        0.0 );
  }
    
  if (z > -1.0)              return( -(z * z * z) - 2.0 * z * z           + 1.0 );
  if (z <= -1.0 && z > -2.0) return(   z * z * z  + 5.0 * z * z + 8.0 * z + 4.0 );
  /* else if (z < -2.0) */   return(                                        0.0 );
}

/* Resampling of a byte image */

int uc_resample (unsigned char *image, double r, double c, int nrow, int ncol,
                 unsigned char *pixel, int method)
{
  unsigned char *ptr, *ptrr;
  int           ir, ic, i, irb, icb;
  double        dr, dc, temp1, temp2, sum, polr[4], polc[4], Polynom(double);
   
  switch (method) {

/* Nearest neighbour interpolation */

    case (1) : ir = (int) (r + 0.5);
               ic = (int) (c + 0.5);
               if (r < -0.5 || ir > nrow-1 || c < -0.5 || ic > ncol-1) return(0);
               ptr = image + ir * ncol + ic;
               *pixel = *ptr;
               return(1);

/* Bilinear interpolation */

    case (2) : ir = (int) r;
               ic = (int) c;
               if (r < 0.0 || ir > nrow-2 || c < 0.0 || ic > ncol-2) return(0);
               dr    = r - (double) ir;
               dc    = c - (double) ic;
               ptr   = image + ir * ncol + ic;
               temp1 = (double) *ptr + dc * ((double) *(ptr + 1) - (double) *ptr);
               ptr  += ncol;
               temp2 = (double) *ptr + dc * ((double) *(ptr + 1) - (double) *ptr);
               *pixel = (int) (temp1 + dr * (temp2 - temp1) + 0.5); 
               return(1);

/* Bicubic interpolation */

    case (3) : irb = (int) r - 1;
               icb = (int) c - 1;
               if (r < 1.0 || irb > nrow-4 || c < 1.0 || icb > ncol-4) return(0);
               for (i = 0; i < 4; i++) {
                 polr[i] = Polynom((double) (irb + i) - r);
                 polc[i] = Polynom((double) (icb + i) - c);
               }
               sum = 0.0;
               for (ir = 0, ptrr = image + irb * ncol + icb;
                    ir < 4;
                    ir++, ptrr += ncol) {
                 for (ic = 0, ptr = ptrr;
                      ic < 4; 
                      ic++, ptr++) {
                   sum += polr[ir] * polc[ic] * (double) *ptr;
                 }
               }
               if (sum < 0.0) *pixel = 0;
               else if (sum < 256.0) *pixel = (int) sum;
               else *pixel = 255;
               return(1);

   default   : fprintf(stderr, "Invalid interpolation method: %d\n",method);
               fprintf(stderr, "1 - Nearest neighbour\n");
               fprintf(stderr, "2 - Bilinear\n");
               fprintf(stderr, "3 - Bicubic\n");
               exit(0);
  }
  return(0);
}

/* Resampling of a float image */

int f_resample(float *image, double r, double c, int nrow, int ncol,
               float *pixel, int method)
{
  float  *ptr, *ptrr;
  int    ir, ic, i, irb, icb;
  double dr, dc, temp1, temp2, sum, polr[4], polc[4], Polynom(double);
   
  switch (method) {

/* Nearest neighbour interpolation */

    case (1) : ir = (int) (r + 0.5);
               ic = (int) (c + 0.5);
               if (r < -0.5 || ir > nrow-1 || c < -0.5 || ic > ncol-1) return(0);
               ptr = image + ir * ncol + ic;
               *pixel = *ptr;
               return(1);

/* Bilinear interpolation */

    case (2) : ir = (int) r;
               ic = (int) c;
               if (r < 0.0 || ir > nrow-2 || c < 0.0 || ic > ncol-2) return(0);
               dr    = r - ir;
               dc    = c - ic;
               ptr   = image + ir * ncol + ic;
               temp1 = (double) *ptr + dc * ((double) (*(ptr + 1) - *ptr));
               temp2 = (double) *(ptr + ncol) +
                       dc * ((double) (*(ptr + ncol + 1) - *(ptr + ncol)));
               *pixel = (float) (temp1 + dr * (temp2 - temp1)); 
               return(1);

/* Bicubic interpolation */

    case (3) : irb = (int) r - 1;
               icb = (int) c - 1;
               if (r < 1.0 || irb > nrow-4 || c < 1.0 || icb > ncol-4) return(0);
               for (i = 0; i < 4; i++) {
                 polr[i] = Polynom((double) (irb + i) - r);
                 polc[i] = Polynom((double) (icb + i) - c);
               }
               sum = 0.0;
               for (ir = 0, ptrr = image + irb * ncol + icb;
                    ir < 4;
                    ir++, ptrr += ncol) {
                 for (ic = 0, ptr = ptrr;
                      ic < 4; 
                      ic++, ptr++) {
                   sum += polr[ir] * polc[ic] * (double) *ptr;
                 }
               }
               *pixel = (float) sum;
               return(1);

   default   : fprintf(stderr, "Invalid interpolation method: %d\n", method);
               fprintf(stderr, "1 - Nearest neighbour\n");
               fprintf(stderr, "2 - Bilinear\n");
               fprintf(stderr, "3 - Bicubic\n");
               exit(0);
  }
  return(0);
}

/* Resampling of a colour band of a colour compressed byte image */

int colour_resample (xvimage *image, double r, double c, unsigned char *red,
                     unsigned char *green, unsigned char *blue, int method)
{
  int           nrow, ncol;
  unsigned char *ptr, *ptrr, grey, grey2, col[3];
  int           ir, ic, i, irb, icb, offset;
  double        dr, dc, temp1, temp2, sum, polr[4], polc[4], Polynom(double);

  unsigned char *maps = (unsigned char *) image->maps;

  nrow = image->col_size;
  ncol = image->row_size;
   
  switch (method) {

/* Nearest neighbour interpolation */

    case (1) : ir = (int) (r + 0.5);
               ic = (int) (c + 0.5);
               if (ir < 0 || ir > nrow-1 || ic < 0 || ic > ncol-1) return(0);
               ptr = (unsigned char *) image->imagedata + ir * ncol + ic;
               *red   = maps[*ptr];
               *green = maps[*ptr + image->map_col_size];
               *blue  = maps[*ptr + 2*image->map_col_size];
               return(1);

/* Bilinear interpolation */

    case (2) : ir = (int) r;
               ic = (int) c;
               if (r < 0.0 || ir > nrow-2 || c < 0.0 || ic > ncol-2) return(0);
               dr    = r - (double) ir;
               dc    = c - (double) ic;
               ptr   = (unsigned char *) image->imagedata + ir * ncol + ic;
               for (i=0, offset=0; i<3; i++, offset+= image->map_col_size) {
                 grey  = maps[*ptr + offset];
                 grey2 = maps[*(ptr+1) + offset];
                 temp1 = (double) grey + dc * ((double) grey2 - (double) grey);
                 ptr  += ncol;
                 grey  = maps[*ptr + offset];
                 grey2 = maps[*(ptr+1) + offset];
                 temp2 = (double) grey + dc * ((double) grey2 - (double) grey);
                 ptr  -= ncol;
                 col[i] = (unsigned char) (temp1 + dr * (temp2 - temp1) + 0.5); 
               }
               *red   = col[0];
               *green = col[1];
               *blue  = col[2];
               return(1);

/* Bicubic interpolation */

    case (3) : irb = (int) r - 1;
               icb = (int) c - 1;
               if (r < 1.0 || irb > nrow-4 || c < 1.0 || icb > ncol-4) return(0);
               for (i = 0; i < 4; i++) {
                 polr[i] = Polynom((double) (irb + i) - r);
                 polc[i] = Polynom((double) (icb + i) - c);
               }
               sum = 0.0;
               for (i=0, offset=0; i<3; i++, offset+=image->map_col_size) {
                 for (ir = 0,
		      ptrr = (unsigned char *) image->imagedata + irb*ncol +icb;
                      ir < 4;
                      ir++, ptrr += ncol) {
                   for (ic = 0, ptr = ptrr;
                        ic < 4; 
                        ic++, ptr++) {
                     grey = maps[*ptr + offset];
                     sum += polr[ir] * polc[ic] * (double) grey;
                   }
                 }
                 if (sum < 0.0) col[i] = 0;
                 else if (sum < 255.0) col[i] = (unsigned char) sum;
                 else col[i] = 255;
               }
               *red   = col[0];
               *green = col[1];
               *blue  = col[2];
               return(1);

   default   : fprintf(stderr, "Invalid interpolation method: %d\n",method);
               fprintf(stderr, "1 - Nearest neighbour\n");
               fprintf(stderr, "2 - Bilinear\n");
               fprintf(stderr, "3 - Bicubic\n");
               exit(0);
  }
  return(0);
}
