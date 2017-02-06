
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

 void LaserPoints::IntersectionEdgesInPartitions   Intersection edges inside
   (PointNumberLists &, Planes &,                  the partitions
    ObjectPoints2D &, LineTopologies &,
    double, double, double, double)

 Initial creation
 Author : George Vosselman
 Date   : 25-04-2001

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
#include "Position2D.h"  // local
#include "LaserPoints.h" // local
#include "Planes.h"
#include "LineSegments2D.h"
#include "ObjectPoints2D.h"
#include "PointNumberLists.h"
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

LineSegments2D LaserPoints::IntersectionEdgesInPartitions
        (PointNumberLists &faces, Planes &planes,
         ObjectPoints2D &partition_corners, LineTopologies &partitions,
         double min_dist_intersection_partition,
         double min_angle_intersection_partition,
         double max_dist_point_to_intersection,
         double max_angle_horizontal)
{
  PointNumberLists::iterator face1, face2;
  Planes::iterator           plane1, plane2;
  Position3D                 begin_point, end_point;
  LineSegment2D              intersection_edge;
  LineSegments2D             partition_edges, intersection_edges;
  double                     check_dist;

/* Convert the partitions to line segments */

  partition_edges = LineSegments2D(partition_corners, partitions);

/* Determine the intersection lines */

  for (face1=faces.begin(), plane1=planes.begin();
       face1!=faces.end(); face1++, plane1++) {
    for (face2=face1+1, plane2=plane1+1;
         face2!=faces.end(); face2++, plane2++) {
      if (IntersectFaces(face1->PointNumberListReference(),
                         face2->PointNumberListReference(),
                         plane1->PlaneReference(), plane2->PlaneReference(),
                         max_dist_point_to_intersection,
                         begin_point, end_point)) {

/* Create the 2D line segment */

        intersection_edge = LineSegment2D(begin_point.Position2DOnly(),
                                          end_point.Position2DOnly(),
                                          plane1->Number(), plane2->Number());

/* Check the vicinity of a partition edge */

        if (plane1->IsHorizontal(max_angle_horizontal) ||
            plane2->IsHorizontal(max_angle_horizontal))
          check_dist = min_dist_intersection_partition; // Avoid small segments
        else
          check_dist = 0.01; // To avoid splitting at the same position
        if (!partition_edges.Collinear(intersection_edge.Line2DReference(),
                                       min_angle_intersection_partition,
                                       check_dist))
          intersection_edges.push_back(intersection_edge);
      }
    }
  }
  return(intersection_edges);
}
