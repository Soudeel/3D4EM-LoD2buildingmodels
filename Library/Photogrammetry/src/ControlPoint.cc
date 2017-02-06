
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
 Collection of functions for class ControlPoint
 
 ControlPoint& operator=(const ControlPoint& point)     Copy assignemnt    
 void ControlPoint::Cpp2C(CtrlPt **)      Conversion of C++ class to C structure
 void ControlPoint::C2Cpp(CtrlPt *)       Conversion of C structure to C++ class

 Initial creation
 Author : Ildi Suveg
 Date   : 24-11-1998

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

#include "ControlPoint.h"

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

ControlPoint& ControlPoint::operator=(const ControlPoint& point)
{
  // Check for self assignment
  if (this == &point) return *this;
  x[0]   = point.x[0];
  x[1]   = point.x[1];
  x[2]   = point.x[2];
  num    = point.num;
  status = point.status;
  return *this;
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ControlPoint::Cpp2C(CtrlPt **ctrlptptr) const
{
  CtrlPt *ctrlpt;

/* Allocate space if this has not been done yet */

  ctrlpt = *ctrlptptr;
  if (ctrlpt == NULL) {
    ctrlpt = (CtrlPt*) malloc(sizeof(CtrlPt));
    *ctrlptptr = ctrlpt;
  }

/* Copy the data from the C++ to the C ctrl */

  ctrlpt->x     = x[0];
  ctrlpt->y     = x[1];
  ctrlpt->z     = x[2];
  ctrlpt->num   = num;
  ctrlpt->status= status;
}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ControlPoint::C2Cpp(CtrlPt *ctrlpt)
{
  x[0]   = ctrlpt->x;
  x[1]   = ctrlpt->y;
  x[2]   = ctrlpt->z;
  num    = ctrlpt->num;
  status = ctrlpt->status;
}
