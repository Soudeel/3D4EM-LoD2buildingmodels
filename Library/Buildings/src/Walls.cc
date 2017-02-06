
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
 Additional collection of functions for class LaserPoints

 void LaserPoints::ConstructWalls               Construct the walls of
   (ObjectPoints &, LineTopologies &)           a building

 Initial creation
 Author : George Vosselman
 Date   : 29-03-2001

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
#include "LaserPoints.h" // local
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                       Declaration of C functions
--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                       Determine line segment near face
--------------------------------------------------------------------------------
*/

void LaserPoints::ConstructWalls(ObjectPoints &corners,
                                 LineTopologies &topology,
                                 int wall_label, double ground_height)
{
  int                       highest_corner_number, i, num_corners,
                            next_wall_number;
  ObjectPoints::iterator    point;
  ObjectPoint               ground_point;
  PointNumberList::iterator node1, node2;
  LineTopology              wall;
  LineTopologies::iterator  face;
  LineTopologies            walls;

// No walls in case of no corners, or no roof lines

  if (corners.empty() || topology.empty()) return;

// Retrieve the highest point number of a roof corner

  point = corners.begin();
  highest_corner_number = point->Number();
  for (point++; point!=corners.end(); point++)
    if (point->Number() > highest_corner_number)
      highest_corner_number = point->Number();

// Retrieve the next wall number

  for (face=topology.begin(), next_wall_number=0; face!=topology.end(); face++)
    if (face->Number() >= next_wall_number)
      next_wall_number = face->Number() + 1;

// Make all polygons counter clock wise

  topology.MakeCounterClockWise(corners);

// Copy all points and project the copies onto the ground plane

  num_corners = corners.size();
  corners.reserve(2 * num_corners);
  for (i=0, point=corners.begin(); i<num_corners; i++, point++) {
    ground_point = *point;
    ground_point.Z() = ground_height;
    ground_point.Number() += highest_corner_number + 1;
    corners.push_back(ground_point);
  }

// Construct the topology of the walls

  wall.Label() = wall_label;
  for (face=topology.begin(); face!=topology.end(); face++) {
    node1 = face->begin();
    for (node2=node1+1; node2!=face->end(); node2++, next_wall_number++) {
      wall.Number() = next_wall_number;
      wall.push_back(*node1);
      wall.push_back(*node2);
      wall.push_back(PointNumber(node2->Number() + highest_corner_number + 1));
      wall.push_back(PointNumber(node1->Number() + highest_corner_number + 1));
      wall.push_back(*node1);
      walls.push_back(wall);
      wall.erase(wall.begin(), wall.end());
      node1 = node2;
    }
  }

// Add the walls to the roof faces

  topology.insert(topology.end(), walls.begin(), walls.end());
  walls.erase(walls.begin(), walls.end());
}
