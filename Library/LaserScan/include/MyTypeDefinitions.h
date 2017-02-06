
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Defines data types for different types.
*
*--------------------------------------------------------------------*/
#ifndef ______MY___TYPES_H___
#define ______MY___TYPES_H___
#include <vector>
#include <set>
#include "LaserPoints.h"
#include "Vector3D.h"
typedef long long int int64;

typedef std::vector<int> IndicesVector;
typedef std::set<int> IndicesSet;
typedef std::vector<IndicesVector> SegmentsVector;
typedef std::vector<Vector3D> CoordinateVector;
typedef std::vector<Vector3D> NormalVector;
typedef std::vector<Vector3D> Vector3Ds;

typedef Vector3D Color3D;
typedef std::vector<Color3D> ColorVector;
#endif //______MY___TYPES_H___

