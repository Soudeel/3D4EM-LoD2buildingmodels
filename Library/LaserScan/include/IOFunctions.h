
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



#ifndef _IO_FUNCTIONS____H____
#define _IO_FUNCTIONS____H____ 
/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : Nov 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Functions for outputting different types using iostreams.
*
*--------------------------------------------------------------------*/

#include <iostream>
#include <cstdio>
#include "Vector3D.h"
#include "Rotation3D.h"


class Vector3D;
std::ostream& operator<<(std::ostream&, Vector3D const&);

class Rotation3D;
void Print(Rotation3D const&, char*, FILE* pFile=stderr);
#endif
