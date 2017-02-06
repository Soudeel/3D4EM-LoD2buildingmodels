
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
  * Revision 1.3  2010/07/09 08:40:01  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:47:16  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:07:36  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:35:52  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:06  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:37  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.2  2003/05/06 12:38:12  pfeifer
  * references to Matrix changed to vMatrix
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for fillgaps
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lfillgaps
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */


#include "internals.h"
#include <math.h>

/* -library_includes */
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lfillgaps - In a raster image with gaps (pixels with value 0) this function calculates values for these pixels by linear interpolation between three pixels with values in three different quadrants around the gap.
* 
*       Purpose: Library Routine for fillgaps
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: George Vosselman
*          Date: May 04, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/
/* -library_def */
void Fill_Gaps_In_Raster(unsigned char *gaps, unsigned char *filled, int nrows,
                         int ncols, int maxdist, int extrapolate)
/* -library_def_end */

/* -library_code */
{  
  unsigned char *gaps_pixel, *filled_pixel;
  int           i, ir, ic, iq, rpos[4], cpos[4], value[4], numpixel, sum,
                iq0, iq1, iq2, imax, gapvalue, warning_extra, warning_collinear;
  double        dist[4], maxd, normal[3], dnorm, distorg, vmin, vmax;

  void Find_Pixel_In_Quadrant(int, unsigned char *, int, int, int, int, int,
	  		                  int *, int *, int *, double *);
                            
  warning_extra = warning_collinear = 0;

  for (ir=0, gaps_pixel=gaps, filled_pixel=filled;
       ir<nrows;
       ir++) {
    for (ic=0;
         ic<ncols;
         ic++, gaps_pixel++, filled_pixel++) {

/* Copy filled values */

      if (*gaps_pixel) {
        *filled_pixel = *gaps_pixel;
      }

/* Interpolate gaps */

      else {

/* Find nearest pixels in four quadrants */

        numpixel = 0;
        for (iq=0; iq<4; iq++) {
          Find_Pixel_In_Quadrant(iq, gaps, nrows, ncols, ir, ic, maxdist,
                                 &(rpos[iq]), &(cpos[iq]), 
                                 &(value[iq]), &(dist[iq])); 
          if (value[iq]) numpixel++;
        }

        // No interpolation if less than three pixels found
        if (numpixel < 3) {
          if (extrapolate && numpixel > 0) {
          	// Take average of all found values
          	for (iq=0, sum=0; iq<4; iq++)
          	  if (value[iq]) sum += value[iq];
          	*filled_pixel = sum / numpixel;
          }
          else *filled_pixel = 0;
        }

        else {

/* In case of pixels in all quadrants, eliminate the most distant pixel */

          if (numpixel == 4) {
            maxd = 0;
            for (iq=0; iq<4; iq++) {
              if (dist[iq] > maxd) {
                maxd = dist[iq];  imax = iq;
              }
            }
            value[imax] = 0;
          }

/* Set indices for interpolation */

          if (value[0]) {
            iq0 = 0;
            if (value[1]) {
              iq1 = 1;
              if (value[2]) {
                iq2 = 2;
              }
              else iq2 = 3;
            }
            else {
              iq1 = 2;
              iq2 = 3;
            }
          }
          else {
            iq0 = 1;
            iq1 = 2;
            iq2 = 3;
          }

/* Calculate surface through the three points */

          rpos[iq1]  -= rpos[iq0];
          cpos[iq1]  -= cpos[iq0];
          value[iq1] -= value[iq0];
          rpos[iq2]  -= rpos[iq0];
          cpos[iq2]  -= cpos[iq0];
          value[iq2] -= value[iq0];
          normal[0] = cpos[iq1] * value[iq2] - cpos[iq2] * value[iq1];
          normal[1] = value[iq1] * rpos[iq2] - value[iq2] * rpos[iq1];
          normal[2] = rpos[iq1] * cpos[iq2]  - rpos[iq2] * cpos[iq1];
          for (i=0, dnorm=0; i<3; i++) dnorm += normal[i] * normal[i];
          dnorm = sqrt(dnorm);
          for (i=0; i<3; i++) normal[i] /= dnorm;
          distorg = normal[0] * rpos[iq0] +
                    normal[1] * cpos[iq0] +
                    normal[2] * value[iq0];

/* Interpolate the gap if the points are not on a line */

          distorg -= ir * normal[0] + ic * normal[1];
          if (fabs(normal[2]) > 0.00001) {
            gapvalue = (int) (distorg / normal[2]);
            if (gapvalue < 1) gapvalue = 1;
            else if (gapvalue > 255) gapvalue = 255;

/* Set the new value if this is not an extreme extrapolation */

	        vmin = vmax = value[iq0];
	        if (value[iq1] < vmin) vmin = value[iq1];
	        if (value[iq2] < vmin) vmin = value[iq2];
	        if (value[iq1] > vmax) vmax = value[iq1];
	        if (value[iq2] > vmax) vmax = value[iq2];
	        vmin *= 0.5;  vmax *= 1.5;
	        if (gapvalue >= vmin && gapvalue <= vmax) {
	    	  if (gapvalue <= 0) *filled_pixel = 1;
              else *filled_pixel = gapvalue;
	        }
	        else {
              if (!warning_extra) {
                printf("Reiterate gap filling to avoid extrapolation.\n");
                warning_extra = 1;
              }
              *filled_pixel = 0;
	        }
          }
          else {
            if (!warning_collinear) {
              printf("Reiterate gap filling since selected points were collinear.\n");
              warning_collinear = 1;
            }
            *filled_pixel = 0;
          }
        }
      }
    }
  }
}

// Float version

void Fill_Gaps_In_Raster_Float(float *gaps, float *filled, int nrows,
                               int ncols, int maxdist, int extrapolate)
{  
  float   *gaps_pixel, *filled_pixel, gapvalue, min_value, value[4], sum;
  int     i, ir, ic, iq, rpos[4], cpos[4], numpixel,
          iq0, iq1, iq2, imax, warning_extra, warning_collinear;
  double  dist[4], maxd, normal[3], dnorm, distorg, vmin, vmax;

  void Find_Pixel_In_Quadrant_Float(int, float *, float, int, int, int, int, int,
	  		                        int *, int *, float *, double *);
                            
  warning_extra = warning_collinear = 0;

  // Determine minimum pixel value
  min_value = 1e10;
  for (ir=0, gaps_pixel=gaps; ir<nrows; ir++) {
  	for (ic=0; ic<ncols; ic++, gaps_pixel++) {
  	  if (*gaps_pixel < min_value) min_value = *gaps_pixel;
  	}
  }

  for (ir=0, gaps_pixel=gaps, filled_pixel=filled; ir<nrows; ir++) {
    for (ic=0; ic<ncols; ic++, gaps_pixel++, filled_pixel++) {

      // Copy filled values
      if (*gaps_pixel != min_value) {
        *filled_pixel = *gaps_pixel;
      }

      // Fill gaps
      else {

        // Find nearest pixels in four quadrants
        numpixel = 0;
        for (iq=0; iq<4; iq++) {
          Find_Pixel_In_Quadrant_Float(iq, gaps, min_value,
		                               nrows, ncols, ir, ic, maxdist,
                                       &(rpos[iq]), &(cpos[iq]), 
                                       &(value[iq]), &(dist[iq])); 
          if (value[iq] != min_value) numpixel++;
        }

        // No interpolation if less than three pixels found
        if (numpixel < 3) {
          if (extrapolate && numpixel > 0) {
          	// Take average of all found values
          	for (iq=0, sum=0.0; iq<4; iq++)
          	  if (value[iq] != min_value) sum += value[iq];
          	*filled_pixel = sum / numpixel;
          }
          else *filled_pixel = min_value;
        }

        // Interpolate
        else {

          // In case of pixels in all quadrants, eliminate the most distant pixel
          if (numpixel == 4) {
            maxd = 0;
            for (iq=0; iq<4; iq++) {
              if (dist[iq] > maxd) {
                maxd = dist[iq];  imax = iq;
              }
            }
            value[imax] = min_value;
          }

          // Set indices for interpolation
          if (value[0] != min_value) {
            iq0 = 0;
            if (value[1] != min_value) {
              iq1 = 1;
              if (value[2] != min_value) {
                iq2 = 2;
              }
              else iq2 = 3;
            }
            else {
              iq1 = 2;
              iq2 = 3;
            }
          }
          else {
            iq0 = 1;
            iq1 = 2;
            iq2 = 3;
          }

          // Calculate surface through the three points
          rpos[iq1]  -= rpos[iq0];
          cpos[iq1]  -= cpos[iq0];
          value[iq1] -= value[iq0];
          rpos[iq2]  -= rpos[iq0];
          cpos[iq2]  -= cpos[iq0];
          value[iq2] -= value[iq0];
          normal[0] = cpos[iq1] * value[iq2] - cpos[iq2] * value[iq1];
          normal[1] = value[iq1] * rpos[iq2] - value[iq2] * rpos[iq1];
          normal[2] = rpos[iq1] * cpos[iq2]  - rpos[iq2] * cpos[iq1];
          for (i=0, dnorm=0; i<3; i++) dnorm += normal[i] * normal[i];
          dnorm = sqrt(dnorm);
          for (i=0; i<3; i++) normal[i] /= dnorm;
          distorg = normal[0] * rpos[iq0] +
                    normal[1] * cpos[iq0] +
                    normal[2] * value[iq0];

          // Interpolate the gap if the points are not on a line
          distorg -= ir * normal[0] + ic * normal[1];
          if (fabs(normal[2]) > 0.00001) {
            gapvalue = (int) (distorg / normal[2]);

            // Set the new value if this is not an extreme extrapolation
	        vmin = vmax = value[iq0];
	        if (value[iq1] < vmin) vmin = value[iq1];
	        if (value[iq2] < vmin) vmin = value[iq2];
	        if (value[iq1] > vmax) vmax = value[iq1];
	        if (value[iq2] > vmax) vmax = value[iq2];
	        vmin *= 0.5;  vmax *= 1.5;
	        if (gapvalue >= vmin && gapvalue <= vmax) {
              *filled_pixel = fmax(gapvalue, min_value+0.01);
	        }
	        else {
              if (!warning_extra) {
                printf("Reiterate gap filling to avoid extrapolation.\n");
                warning_extra = 1;
              }
              *filled_pixel = min_value;
	        }
          }
          else {
            if (!warning_collinear) {
              printf("Reiterate gap filling since selected points were collinear.\n");
              warning_collinear = 1;
            }
            *filled_pixel = min_value;
          }
        }
      }
    }
  }
}
