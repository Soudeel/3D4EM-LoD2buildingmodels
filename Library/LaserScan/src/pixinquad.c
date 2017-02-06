
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
#include <math.h>

void Find_Pixel_In_Quadrant(int iq, unsigned char *image, int nrows, int ncols,
                            int rgap, int cgap, int maxdist,
			                int *rpix, int *cpix, int *pixvalue, 
                            double *pixdist)
{
  int           i, dist, ir, ic, r1, c1, r2, c2, rinc1, cinc1, rinc2, cinc2;
  unsigned char *pixel;
  double        founddist;

  dist = 1;  *pixvalue = 0; *pixdist = maxdist * 2;

/* Start with the nearest pixels */

  while (dist <= maxdist && *pixvalue == 0) {

/* Set area bounds to scan */

    switch (iq) {
      case 0:
        r1 = rgap - dist;      r2 = rgap - dist;
        c1 = cgap - dist;      c2 = cgap;
        rinc1 = 1;             rinc2 = 0;
        cinc1 = 0;             cinc2 = -1;
	break;

      case 1:
        r1 = rgap + dist;      r2 = rgap;
        c1 = cgap - dist;      c2 = cgap - dist;
        rinc1 = 0;             rinc2 = 1;
        cinc1 = 1;             cinc2 = 0;
	break;

      case 2:
        r1 = rgap + dist;      r2 = rgap + dist;
        c1 = cgap + dist;      c2 = cgap;
        rinc1 = -1;            rinc2 = 0;
        cinc1 = 0;             cinc2 = 1;
	break;

      case 3:
        r1 = rgap - dist;      r2 = rgap;
        c1 = cgap + dist;      c2 = cgap + dist;
        rinc1 = 0;             rinc2 = -1;
        cinc1 = -1;            cinc2 = 0;
	break;
    }

    for (i=0, ir=r1, ic=c1;
	 i<dist;
	 i++, ir+=rinc1, ic+=cinc1) {
      if (ir >= 0 && ic >= 0 && ir < nrows && ic < ncols) {
        pixel = image + ir * ncols + ic;
	if (*pixel) {
	  founddist = (ir - rgap) * (ir - rgap) + (ic - cgap) * (ic - cgap);
	  founddist = sqrt(founddist);
	  if (founddist < *pixdist) {
	    *pixdist = founddist;
	    *pixvalue = *pixel;
	    *rpix = ir;
	    *cpix = ic;
          }
        }
      }
    }

    for (i=0, ir=r2, ic=c2;
	 i<dist;
	 i++, ir+=rinc2, ic+=cinc2) {
      if (ir >= 0 && ic >= 0 && ir < nrows && ic < ncols) {
        pixel = image + ir * ncols + ic;
	if (*pixel) {
	  founddist = (ir - rgap) * (ir - rgap) + (ic - cgap) * (ic - cgap);
	  founddist = sqrt(founddist);
	  if (founddist < *pixdist) {
	    *pixdist = founddist;
	    *pixvalue = *pixel;
	    *rpix = ir;
	    *cpix = ic;
          }
        }
      }
    }

    dist++;

  }
}


// Float image version

void Find_Pixel_In_Quadrant_Float(int iq, float *image, float min_value,
                                  int nrows, int ncols,
                                  int rgap, int cgap, int maxdist,
			                      int *rpix, int *cpix, float *pixvalue, 
                                  double *pixdist)
{
  int    i, dist, ir, ic, r1, c1, r2, c2, rinc1, cinc1, rinc2, cinc2;
  float  *pixel;
  double founddist;

  dist = 1;  *pixvalue = min_value; *pixdist = maxdist * 2;

/* Start with the nearest pixels */

  while (dist <= maxdist && *pixvalue == min_value) {

/* Set area bounds to scan */

    switch (iq) {
      case 0:
        r1 = rgap - dist;      r2 = rgap - dist;
        c1 = cgap - dist;      c2 = cgap;
        rinc1 = 1;             rinc2 = 0;
        cinc1 = 0;             cinc2 = -1;
	    break;

      case 1:
        r1 = rgap + dist;      r2 = rgap;
        c1 = cgap - dist;      c2 = cgap - dist;
        rinc1 = 0;             rinc2 = 1;
        cinc1 = 1;             cinc2 = 0;
	    break;

      case 2:
        r1 = rgap + dist;      r2 = rgap + dist;
        c1 = cgap + dist;      c2 = cgap;
        rinc1 = -1;            rinc2 = 0;
        cinc1 = 0;             cinc2 = 1;
	    break;

      case 3:
        r1 = rgap - dist;      r2 = rgap;
        c1 = cgap + dist;      c2 = cgap + dist;
        rinc1 = 0;             rinc2 = -1;
        cinc1 = -1;            cinc2 = 0;
	    break;
    }

    for (i=0, ir=r1, ic=c1;
	 i<dist;
	 i++, ir+=rinc1, ic+=cinc1) {
      if (ir >= 0 && ic >= 0 && ir < nrows && ic < ncols) {
        pixel = image + ir * ncols + ic;
	if (*pixel != min_value) {
	  founddist = (ir - rgap) * (ir - rgap) + (ic - cgap) * (ic - cgap);
	  founddist = sqrt(founddist);
	  if (founddist < *pixdist) {
	    *pixdist = founddist;
	    *pixvalue = *pixel;
	    *rpix = ir;
	    *cpix = ic;
          }
        }
      }
    }

    for (i=0, ir=r2, ic=c2;
	 i<dist;
	 i++, ir+=rinc2, ic+=cinc2) {
      if (ir >= 0 && ic >= 0 && ir < nrows && ic < ncols) {
        pixel = image + ir * ncols + ic;
	if (*pixel != min_value) {
	  founddist = (ir - rgap) * (ir - rgap) + (ic - cgap) * (ic - cgap);
	  founddist = sqrt(founddist);
	  if (founddist < *pixdist) {
	    *pixdist = founddist;
	    *pixvalue = *pixel;
	    *rpix = ir;
	    *cpix = ic;
          }
        }
      }
    }

    dist++;

  }
}
