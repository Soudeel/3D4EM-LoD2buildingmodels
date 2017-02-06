
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

 LineSegments2D LaserPoints::HeightJumpsInPartition  Height jumps in a partition
   (ObjectPoints2D &, LineTopology &,
    LineTopologies &, TINEdges &,
    double, double, double, double)

 Initial creation
 Author : George Vosselman
 Date   : 22-04-2001

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
#include "digphot_arch.h"
#include <limits.h>
#include "Position2D.h"  // local
#include "LaserPoints.h" // local
#include "LineSegments2D.h"
#include "ObjectPoints2D.h"
#include "LineTopologies.h"
#include "PointNumberLists.h"
#include "Planes.h"
#include "stdmath.h"

#ifndef MAXDOUBLE
#define MAXDOUBLE 1e30
#endif

/*
--------------------------------------------------------------------------------
                       Declaration of C functions
--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                       Determine height jumps in face
--------------------------------------------------------------------------------
*/

LineSegments2D LaserPoints::HeightJumpsInPartition(
                ObjectPoints2D &partition_corners, LineTopology &partition,
                LineSegments2D &intersection_edges,
                LineTopologies &contours, Planes &planes, TINEdges &edges,
                double min_dist_height_jump_partition,
                double min_angle_height_jump_partition,
                double max_dist_point_to_height_jump,
                double min_jump_height)
{
  LaserPoints::iterator      point;
  int                        *lookup_label, i, face_index, label, tmp,
                             num_labels;
  PointNumberLists           faces;
  PointNumberLists::iterator face;
  PointNumberList            nb_nodes;
  PointNumberList::iterator  node, nb_node;
  LineTopologies::iterator   contour, contour2;
  LineSegments2D             partition_edges, height_jump_edges;
  LineSegments2D::iterator   partition_edge;
  LineSegment2D              height_jump_edge;
  Line2D                     line;
  double                     largest_dist, dist, s, s_min, s_max,
                             s_min2, s_max2;
  Planes::iterator           plane;

/* Create a look up table for the labels */

  contour = contours.end() - 1;
  num_labels = contour->Number() + 1;  // Retrieve number of used labels
  lookup_label = (int *) malloc(num_labels * sizeof(int));
  for (i=0; i<num_labels; i++) lookup_label[i] = -1;
  for (contour=contours.begin(), i=0; contour!=contours.end(); contour++, i++)
    lookup_label[contour->Number()] = i;

/* Determine the segments of the partition border */

  partition_edges = LineSegments2D(partition_corners, partition);

/* Relabel the points inside the partition, and store the point numbers
 * for each face.
 */

  for (i=0; i<contours.size(); i++) faces.push_back(PointNumberList());
  for (point=begin(), i=0; point!=end(); point++, i++) {
    if (point->Label() < num_labels) { // point with a face label
      if (point->InsidePolygon(partition_corners, // and inside partition
                               partition.PointNumberListReference())) {
        faces[lookup_label[point->Label()]].push_back(PointNumber(i));
        point->Label(point->Label() + num_labels);
      }
    }
  }

/* Bound each face with by lines parallel to partition segments. */

  for (face=faces.begin(), face_index=0;
       face!=faces.end(); face++, face_index++) {
    if (!face->empty()) {
      for (partition_edge=partition_edges.begin();
           partition_edge!=partition_edges.end(); partition_edge++) {
        largest_dist = 0;
        for (node=face->begin(); node!=face->end(); node++) {
          point = begin() + node->Number();
          dist = partition_edge->Line2D::DistanceToPointSigned
                                                      (point->Position2DOnly());
          if (fabs(dist) > fabs(largest_dist)) largest_dist = dist;
        }
        line = Line2D(partition_edge->Getsinphi(), partition_edge->Getcosphi(),
                      partition_edge->GetDisto() + largest_dist);

/* Neglect this line if there is a nearby partition segment or
 * a nearby intersection line.
 */

        if (!partition_edges.Collinear(line, min_angle_height_jump_partition,
                                       min_dist_height_jump_partition)) {
          if (!intersection_edges.Collinear(line,
                                            min_angle_height_jump_partition,
                                            min_dist_height_jump_partition)) {

/* Support for a height jump is defined by the presence of contour points of
 * this face and contour points of other lower faces. First determine the
 * presence of contour points of this face.
 */

            s_max = -MAXDOUBLE;  s_min = MAXDOUBLE;
            contour = contours.begin() + face_index;
            for (node=contour->begin(); node!=contour->end(); node++) {
              point = begin() + node->Number();
              if (point->Label() >= num_labels) { // point in partition
                if (line.DistanceToPoint(point->Position2DOnly()) <
                    max_dist_point_to_height_jump) {
                  s = line.Scalar(point->Position2DOnly());
                  if (s < s_min) s_min = s;
                  if (s > s_max) s_max = s;
                }
              }
            }

/* Determine the support by other faces. First check nearby point of other
 * contours, then also check adjacent points of other faces (needed in case
 * of occluded areas and in case of holes.
 *
 * Modification: nearby lower points do not need to be part of a face.
 */

            if (s_max >= s_min) { // Some support from current face
              plane = planes.begin() + face_index;
              s_max2 = -MAXDOUBLE;  s_min2 = MAXDOUBLE;
              for (contour2=contours.begin();
                   contour2!=contours.end(); contour2++) {
                if (contour2 != contour) { // all other contour faces
                  for (node=contour2->begin(); node!=contour2->end(); node++) {
                    point = begin() + node->Number();
                    if (point->Label() >= num_labels) { // point in partition
                      if (point->Z() + min_jump_height < 
                          plane->Z_At(point->X(), point->Y(), &tmp) &&
                          line.DistanceToPoint(point->Position2DOnly()) <
                          max_dist_point_to_height_jump) {
                        s = line.Scalar(point->Position2DOnly());
                        if (s < s_min2) s_min2 = s;
                        if (s > s_max2) s_max2 = s;
                      }
                    }
                  }
                }
              }
              label = contour->Number() + num_labels;
              for (node=contour->begin(); node!=contour->end(); node++) {
                nb_nodes = edges[node->Number()];
                for (nb_node=nb_nodes.begin();
                     nb_node!=nb_nodes.end(); nb_node++) {
                  point = begin() + nb_node->Number();
                  if (point->Label() >= num_labels && // point in partition
// modification       point->Label() < 2 * num_labels && // labeled
                      point->Label() != label) { // in other face
                    if (point->Z() + min_jump_height <
                        plane->Z_At(point->X(), point->Y(),&tmp)) {
                      s = line.Scalar(point->Position2DOnly());
                      if (s < s_min2) s_min2 = s;
                      if (s > s_max2) s_max2 = s;
                    }
                  }
                }
              }
          
/* Store the segment if there is common support */

              if (s_max2 >= s_min2) { // Some support from other faces
                if (s_min2 > s_min) s_min = s_min2;
                if (s_max2 < s_max) s_max = s_max2;
                if (s_max >= s_min) // Some common support
                  height_jump_edges.push_back(LineSegment2D(line, s_min,s_max));
              }
            }
          }
        }
      }
    }
  }

/* Set the labels back to their original values */

  for (face=faces.begin(), contour=contours.begin();
       face!=faces.end();
       face++, contour++)
    Label(face->PointNumberListReference(), contour->Number());

/* Return the height jump edges */

  return(height_jump_edges);
}
