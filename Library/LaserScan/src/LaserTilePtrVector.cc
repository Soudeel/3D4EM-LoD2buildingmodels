
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
 Collection of functions for class LaserTilePtrVector

 Initial creation
 Author : George Vosselman
 Date   : 29-03-1999

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

#include "LaserTilePtrVector.h"

/*
--------------------------------------------------------------------------------
                           Copy assignment
--------------------------------------------------------------------------------
*/

LaserTilePtrVector LaserTilePtrVector::operator = (const LaserTilePtrVector &v)
{
  if (this == &v) return *this;  // Check for self assignment
  if (!empty()) erase(begin(), end());
  if (!v.empty()) insert(begin(), v.begin(), v.end());
  return(*this);
}

/*
--------------------------------------------------------------------------------
                    Check if the vector contains a tile
--------------------------------------------------------------------------------
*/
bool LaserTilePtrVector::Contains(LaserSubUnit *tile) const
{
  LaserTilePtrVector::const_iterator t;
  
  for (t=begin(); t!=end(); t++)
    if (*t == tile) return true;
  return false;
}
