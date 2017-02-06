
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



#include <vector>
#include "LaserPatch.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

#ifndef _LaserPatches_h_
#define _LaserPatches_h_

class LaserPatches : public std::vector <LaserPatch>
{
      public:
             ///default constructor
             LaserPatches() {};
         
             ~LaserPatches() {};
             ///construct from objectpoints and linetopologies
             LaserPatches(const ObjectPoints &, const LineTopologies &);
             LaserPatches(int new_size)
	         {
		      resize(new_size);
	          } 
	        void Release();  
	          
	     
                          
                    
};
#endif /* _LaserPatches_h_ */
