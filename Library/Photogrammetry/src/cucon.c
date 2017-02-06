
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

void cucon(xvimage *viff_in, xvimage *viff_out, double min_x, double max_x,
           double min_y, double max_y, double pix_size, double par[8],
           int maxgv)
   /*----------------------------------------------------------------------------   
   *  resampling of image image_data to image im2 with cubic convolution and 
   *  a projective transformation
   *----------------------------------------------------------------------------
   *  Input parameters:
   *
   *  viff_in      - original image 
   *  min_x, max_x, min_y, max_y - area that is covered.
   *  pix_size     - size of the pixel
   *  par          - parameters of the projective transformation from im2
   *                 to image_data as calculated by routine Cal_Trans_Pars
   *  maxgv        - grey value to be assigned to pixels inside viff_out but
   *                 outside viff_in
   *  
   *  Output parameters:
   *  
   *  viff_out     - resampled image
   *----------------------------------------------------------------------------*/
   
{   
   int greyvalues[4][4];    /* greyvalues of the surrounding 16 pixels            */
   int *grey_pointer;       /* pointer to the greyvalues array                    */
   int    i_r, i_c;         /* integer coordinates of above mentioned point       */
   double dr,  dc;          /* fractional part of f_r and f_c                     */
   int    i, j, u, v;       /* loop counters                                      */
   double temp;             /* value of the interpolated pixel                    */
   int    r_size;           /* number of columns in the original image            */
   unsigned char *p1;       /* Pointer to upper left of the 16 surrounding pixels */
   unsigned char *pixel;   
   double fr, fc, x, y, devid;   
   int r, c;

   double Polynom(double);
   
   pixel = (unsigned char *) viff_out->imagedata;
   for (y = max_y, r = 0; y > min_y; y -= pix_size, r++)
   { 
      for (x = min_x, c = 0; x < max_x; x += pix_size, c++)
      { 
	 devid =  par[6] * x + par[7] * y + 1.0;
	 fc    = (par[3] * x + par[4] * y + par[5]) / devid;
	 fr    = (par[0] * x + par[1] * y + par[2]) / devid;
	 i_r   = (int) fr;
	 i_c   = (int) fc;
	 
	 if (i_r - 1 < 0 || i_r > viff_in->col_size - 2 || 
	     i_c - 1 < 0 || i_c > viff_in->row_size - 2)
	    *(pixel + r * viff_out->row_size + c) = maxgv;
	 else
	 {
	    dr    = fr - (float) i_r;
	    dc    = fc - (float) i_c;
	    
	    r_size = viff_in->row_size;
	    p1     = (unsigned char *)viff_in->imagedata + i_c + i_r * r_size ;
	    
	    /*------ Fill the array greyvalues with the color values of the 16 nearest pixels ------*/
	    grey_pointer = (int *)greyvalues;    /* so that we can access the elements of the matrix faster */
	    for (i = -1; i < 3; i++)
	       for (j = -1; j < 3; j++, grey_pointer++)
		  *(grey_pointer) = (int) *(p1 + i * r_size + j);
	    
	    temp = 0.0;                   /* Store the start of the greyvalues array in grey_pointer */
	    grey_pointer = (int *)greyvalues;    /* so that we can access the elements of the matrix faster */
	    for (u = -1; u < 3; u++)
	       for (v = -1; v < 3; v++, grey_pointer++)
		  temp += Polynom((double) u - dr) * Polynom((double) v - dc) * (double) *(grey_pointer);
	    
	    if (temp > 255) temp = 255;
	    if (temp < 0)   temp =   0;
	    
	    *(pixel + r * viff_out->row_size + c) = (unsigned char)temp;
	 }
      }
   }
}
