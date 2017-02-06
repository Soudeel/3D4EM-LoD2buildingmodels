
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
--------------------------------------------------------------------------------
 Collection of functions for class PixelPosition2D

 Initial creation
 Author : Ildiko Suveg
 Date   : 03-03-1999

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "PixelPosition.h"
#include "InteriorOrientation.h"

extern "C" int Metric_To_Pix(Interior *, double, double, double *, double *);

/*
--------------------------------------------------------------------------------
           Transformation of camera coordinates to pixel coordinates
--------------------------------------------------------------------------------
*/


PixelPosition::PixelPosition(const Position2D *pos, const InteriorOrientation *intor)
  : Position2D()
{
  Interior *into = (Interior*)malloc(sizeof(Interior));
  intor->Cpp2C(&into);	

  Metric_To_Pix(into, pos->x[0], pos->x[1], &x[0], &x[1]); 
  
  free(into);
}   


