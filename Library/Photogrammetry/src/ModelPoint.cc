
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
 Collection of functions for class ModelPoint

 ModelPoint& operator=(const ModelPoint& point)     Copy assignemnt    
 void ModelPoint::Cpp2C(ModelPt **)      Conversion of C++ class to C structure
 void ModelPoint::C2Cpp(ModelPt *)       Conversion of C structure to C++ class

 Initial creation
 Author : Ildi Suveg
 Date   : 23-11-1998

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

#include <stdlib.h>
#include "ModelPoint.h"

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

ModelPoint& ModelPoint::operator=(const ModelPoint& point)
{
  // Check for self assignment
  if (this == &point) return *this;
  x[0]   = point.x[0];
  x[1]   = point.x[1];
  x[2]   = point.x[2];
  num    = point.num;
  var[0] = point.var[0];
  var[1] = point.var[1];
  var[2] = point.var[2];
  var[3] = point.var[3];
  var[4] = point.var[4];
  var[5] = point.var[5];
  return *this;
}


/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ModelPoint::Cpp2C(ModelPt **modelptptr) const
{
  ModelPt *modelpt;

/* Allocate space if this has not been done yet */

  modelpt = *modelptptr;
  if (modelpt == NULL) {
    modelpt = (ModelPt*) malloc(sizeof(ModelPt));
    *modelptptr = modelpt;
  }

/* Copy the data from the C++ to the C Model */

  modelpt->x     = x[0];
  modelpt->y     = x[1];
  modelpt->z     = x[2];
  modelpt->num   = num;
  modelpt->v_x   = var[0];
  modelpt->v_y   = var[1];
  modelpt->v_z   = var[2];
  modelpt->cv_xy = var[3];
  modelpt->cv_xz = var[4];
  modelpt->cv_yz = var[5];

}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ModelPoint::C2Cpp(ModelPt *modelpt)
{
  x[0]   = modelpt->x;
  x[1]   = modelpt->y;
  x[2]   = modelpt->z;
  num    = modelpt->num;
  var[0] = modelpt->v_x;
  var[1] = modelpt->v_y;
  var[2] = modelpt->v_z;
  var[3] = modelpt->cv_xy;
  var[4] = modelpt->cv_xz;
  var[5] = modelpt->cv_yz;
}
