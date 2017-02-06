
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
 Collection of functions for class Lines2D          

 Lines2D & Lines2D::operator =(const Lines2D &)    Copy assignment
 
 Initial creation
 Author : George Vosselman
 Date   : 24-02-2001

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
#include "Lines2D.h"

/*
--------------------------------------------------------------------------------
                                 Copy assignment
--------------------------------------------------------------------------------
*/

Lines2D &Lines2D::operator = (const Lines2D &lines)
{
  // Check for self assignment
  if (this == &lines) return *this;
  if (!empty()) erase(begin(), end());
  if (!lines.empty()) insert(begin(), lines.begin(), lines.end());
  return(*this);
}
