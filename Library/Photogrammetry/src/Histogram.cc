
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
 Collection of functions for class Histogram

 Histogram::Histogram(int, double, double)    Constructor
 void Histogram::Add(double)                  Add value to histogram

 Initial creation
 Author : George Vosselman
 Date   : 09-06-1999

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

#include <stdio.h>
#include <stdlib.h>
#include "Histogram.h"

/*
--------------------------------------------------------------------------------
                                Constructor
--------------------------------------------------------------------------------
*/

Histogram::Histogram(int num_bins, double minv, double maxv) : Image()
{
  Initialise();
  image = NewImage(1, num_bins, VFF_TYP_4_BYTE);
  Clear();
  Set1DLocationData(minv, maxv);
}

/*
--------------------------------------------------------------------------------
                        Add a value to the histogram
--------------------------------------------------------------------------------
*/

void Histogram::Add(double value)
{
  int    bin, *freq;
  double minv, maxv;

/* Determine the bin */

  Get1DLocationData(&minv, &maxv);
  bin = (int) ((value - minv) * NumColumns() / (maxv - minv));
  if (bin < 0) bin = 0;
  if (bin >= NumColumns()) bin = NumColumns() - 1;

/* Increase the frequency of the bin */

  freq = IntPixel(0, bin);
  (*freq)++;
}
