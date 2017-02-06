
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



#include "Plane.h"
#include "Vector3D.h"
#include "Positions3D.h"
#include "LaserPatches.h"


LaserPatches::LaserPatches(const ObjectPoints &objpts, const LineTopologies &tops)
 {
              LaserPatch p;
              for(int i=0;i<tops.size();i++)
              {
              p=LaserPatch(objpts,tops[i]);
              push_back(p);
              }  
}

/*
--------------------------------------------------------------------------------
                           Delete all points
--------------------------------------------------------------------------------
*/
void LaserPatches::Release()
{
  LaserPatches newset = LaserPatches();

  
  
  // Delete all points
  if (!empty()) erase(begin(), end());

  /* Although all points are deleted, the memory space is still allocated by
   * the vector. In order to force deallocation we swap the vector with an new
   * empty vector. Note that this swap only affects the vector<LaserPoint>
   * even though LaserPoints also inherits from LaserPointsInfo and
   * LaserDataFiles. All meta data of "this" therefore remains unchanged.
   */
  swap(newset);
  
 
}


/*Pieces &Pieces::operator = (const Pieces &ps)
{
  Plane p;
  if (!empty()) erase(begin(), end());
  if (!ps.empty()) insert(begin(), ps.begin(), ps.end());
  for(int i=0;i<size();i++)
  {
  p= ps[i].getPlane();      
  ((*this)[i]).setPlane(p);        
  }
  return(*this);
}
*/

