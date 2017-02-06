
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
 Additional collection of functions for class LaserSubUnit

 void LaserSubUnit::RemoveFilteredPoints()      Delete filtered points
 void LaserSubUnit::SelectFilteredPoints        Select the filtered points
   (LaserSubUnit & const)

 Initial creation
 Author : George Vosselman
 Date   : 30-11-1999

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
#include <math.h>
#include "LaserSubUnit.h"

/*
--------------------------------------------------------------------------------
                      Remove or select the filtered points
--------------------------------------------------------------------------------
*/

void LaserSubUnit::RemoveFilteredPoints()
{
  LaserSubUnit::iterator point, goodpoint;
  DataBoundsLaser        tile_bounds;

/* In case of tile wise filtering, do not select points in tile border */

  if (DataOrganisation() & TileWise) {
    tile_bounds = TileBounds();
    for (point=begin(), goodpoint=begin();
         point!=end();
         point++) {
      if (!point->Filtered()) {
        if (tile_bounds.InsideXY(&*point)) {
          *goodpoint = *point;
          goodpoint++;
        }
      }
    }
    erase(goodpoint, end());
  }
  else LaserPoints::RemoveFilteredPoints();
}

void LaserSubUnit::SelectFilteredPoints(const LaserSubUnit &allpoints)
{
  LaserSubUnit::const_iterator point;
  DataBoundsLaser        tile_bounds;

/* In case of tile wise filtering, do not select points in tile border */

  if (DataOrganisation() & TileWise) {
    if (!empty()) erase(begin(), end());
    tile_bounds = TileBounds();
    for (point=allpoints.begin(); point!=allpoints.end(); point++) {
      if (point->Filtered() && tile_bounds.InsideXY(&*point)) push_back(*point);
    }
  }
  else LaserPoints::SelectFilteredPoints(allpoints);
}
