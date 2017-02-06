
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



#include "Quaternion.h"
#include "Database4Cpp.h"

Quaternion::Quaternion(const ExteriorOrientation &extor)
{
   Exterior *ext;
   ext = NULL;
   extor.Cpp2C(&ext);
   Quat_From_Rot(ext);
   q[0] = ext->a;
   q[1] = ext->b;
   q[2] = ext->c;
   free(ext);
}
    
Quaternion::Quaternion(const AbsoluteOrientation &absor)
{
   Absolute *abss;
   abss = NULL;
   absor.Cpp2C(&abss);
   Quat_From_Rot((Exterior*)abss);
   q[0] = abss->a;
   q[1] = abss->b;
   q[2] = abss->c;
   free(abss);
}
