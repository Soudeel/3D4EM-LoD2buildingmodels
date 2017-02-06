
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

int neigh(xvimage *viff_in, xvimage *viff_out, double min_x, double max_x,
          double min_y, double max_y, double pix_size, double par[8], int maxgv)
   
   /*------------------------------------------------------------------------
   *  resampling of image image_data to image im2 with the nearest neighbour 
   *  procedure and a projective transformation
   *------------------------------------------------------------------------
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
   *-------------------------------------------------------------------------
   */
{  
   unsigned char *image_data = (unsigned char *)viff_in->imagedata;
   int r, c, rt, ct, in_size, out_size;
   double devid; 
   unsigned char *pixel;
   double x, y;
   
   /* resampling */
   
   pixel = (unsigned char *)viff_out->imagedata;
   in_size = viff_in->row_size;
   out_size = viff_out->row_size;
   for (y = max_y, r = 0; y > min_y; y -= pix_size, r++)
   { 
      for (x = min_x, c = 0; x < max_x; x += pix_size, c++)
      { 
	 devid =  par[6] * x + par[7] * y + 1.0;
	 ct    = (int) ((par[3] * x + par[4] * y + par[5]) / devid + 0.5);
	 rt    = (int) ((par[0] * x + par[1] * y + par[2]) / devid + 0.5);
	 
	 if (rt < 0 || rt > viff_in->col_size || ct < 0 || ct > viff_in->row_size) 
	    *(pixel + r * out_size + c) = maxgv ;
	 else
	    *(pixel + r * out_size + c) = *(image_data + ct + rt * in_size);
      }
   }
   return(0) ;
}
