
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

 void LaserPoints::AlignFacesToPartitions       Align face bounds to partition
   (PointNumberLists &, Planes &,               bounds, intersection edges,
    ObjectPoints2D &, LineTopologies &,         and height jump edges
    ObjectPoints &, LineTopologies &,
    TINEdges &, double, double, double,
    double, double, double, double
    double, double, double)
 bool LaserPoints::SubPartition                 Try to subdivide partition based
   (ObjectPoints2D &, LineTopologies &, int,    on intersection edges and
    LineSegments2D &, LineTopologies &,         height jump edges
    Planes &, TINEdges &, LineSegments2D &,
    double, double, double,
    double, double, double)
 void LaserPoints::ConstructRoofFace            Construct the roof face in a
   (ObjectPoints2D &, LineTopology &,           partition that can not be
    Planes &, ObjectPoints &,                   split any further
    LineTopologies &)
{

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
#include "Position2D.h"
#include "LineTopologies.h"
#include "LaserPoints.h"
#include "LineSegments2D.h"
#include "ObjectPoints2D.h"
#include "PointNumberLists.h"
#include "stdmath.h"
#include "VRML_io.h"

#ifndef MAXDOUBLE
#define MAXDOUBLE 1e30
#endif

/*
--------------------------------------------------------------------------------
                       Declaration of external functions
--------------------------------------------------------------------------------
*/

extern void PartitionSizes(ObjectPoints2D &, ObjectPoints2D &,
                           LineTopologies &, double &, double &);

/*
--------------------------------------------------------------------------------
              Merge the ground plan with the detected planar faces
--------------------------------------------------------------------------------
*/

void LaserPoints::AlignFacesToPartitions(PointNumberLists &faces,
                                         Planes &planes,
                                         ObjectPoints2D &partition_corners,
                                         LineTopologies &partitions,
                                         ObjectPoints &roofface_corners,
                                         LineTopologies &roofface_topology,
                                         TINEdges &edges,
                                         double min_dist_intersection_partition,
                                        double min_angle_intersection_partition,
                                         double max_dist_point_to_intersection,
                                         double min_dist_height_jump_partition,
                                         double min_angle_height_jump_partition,
                                         double max_dist_point_to_height_jump,
                                         double min_jump_height,
                                         double min_partition_size,
                                         double min_partition_percentage,
                                         double max_angle_horizontal,
                                         ObjectPoints2D &intersection_points,
                                         LineTopologies &intersection_lines,
                                         ObjectPoints2D &jump_edge_points,
                                         LineTopologies &jump_edges)
{
  PointNumberLists::iterator face;
  Planes::iterator           plane;
  LineSegments2D             intersection_segments;
  LineTopologies             contours;
  LineTopologies::iterator   partition;
  int                        partition_index;

  LineSegments2D             all_edges, height_jump_edges;

/* Determine the intersection lines */

  intersection_segments =
    IntersectionEdgesInPartitions(faces, planes,
                                  partition_corners, partitions,
                                  min_dist_intersection_partition,
                                  min_angle_intersection_partition,
                                  max_dist_point_to_intersection,
                                  max_angle_horizontal);

  intersection_segments.PointsWithTopology(intersection_points,
                                           intersection_lines, 0);
//  intersection_points.Write("intersect.objpts2d");
//  intersection_lines.Write("intersect.top");

/* Derive the contours of the faces */

  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end(); face++, plane++) 
    contours.push_back(DeriveContour(plane->Number(),
                                     face->PointNumberListReference(), edges));

/* Try the further split the partitions, using the intersection lines and
 * height jump lines found in the partitions.
 */

  for (partition_index=0;
       partition_index<partitions.size();
       partition_index++) {
    if (SubPartition(partition_corners, partitions, partition_index,
                     intersection_segments, contours, planes, edges,
                     height_jump_edges, min_dist_height_jump_partition,
                     min_angle_height_jump_partition,
                     max_dist_point_to_height_jump,
                     min_jump_height,
                     min_partition_size, min_partition_percentage))
      partition_index--;
    if (!height_jump_edges.empty())
      all_edges.insert(all_edges.end(), height_jump_edges.begin(),
                       height_jump_edges.end());
  }
  all_edges.PointsWithTopology(jump_edge_points, jump_edges, 0);
  
//  jump_edge_points.Write("jump.objpts2d");
//  jump_edges.Write("jump.top");
//  partition_corners.Write("refined.objpts2d");
//  partitions.Write("refined.top");

/* Assign the partitions to a plane and construct the face bounds */

  for (partition=partitions.begin(); partition!=partitions.end(); partition++) {
    ConstructRoofFace(partition_corners, partition->LineTopologyReference(),
                      planes, roofface_corners, roofface_topology);
  }

/* Output of intermediate results:
 * - intersection edges
 * - height jump edges
 * - partitioning results
 * - roof faces
 */

//  roofface_corners.Write("roof.objpts");
//  roofface_topology.Write("roof.top");

/* Merge adjacent faces in the same plane */

  printf("Merging of polygons enabled.\n");
  roofface_corners.RemoveDoublePoints(roofface_topology, 0.01);
  roofface_topology.InsertNodes(roofface_corners);
  roofface_topology.MergePolygons();
  roofface_topology.RemoveCollinearNodes(roofface_corners);
  roofface_topology.ReNumber(roofface_corners);
//  roofface_corners.Write("roofm.objpts");
//  roofface_topology.Write("roofm.top");
}

/*
--------------------------------------------------------------------------------
 Try to split the partition based on intersection edges and height jumps edges
--------------------------------------------------------------------------------
*/

bool LaserPoints::SubPartition(ObjectPoints2D &partition_corners,
                               LineTopologies &partitions, int partition_index,
                               LineSegments2D &intersection_segments,
                               LineTopologies &contours,
                               Planes &planes, TINEdges &edges,
                               LineSegments2D &height_jump_edges,
                               double min_dist_height_jump_partition,
                               double min_angle_height_jump_partition,
                               double max_dist_point_to_height_jump,
                               double min_jump_height,
                               double min_partition_size,
                               double min_partition_percentage)
{
  LineTopologies::iterator partition;
  LineTopologies           new_partitions;
  LineSegments2D           partition_edges;
  LineSegments2D::iterator edge, best_edge;
  double                   len_present, len_missing, best_support, size1,
                           size2, support;
  ObjectPoints2D           new_corners;

  double degree = 4.0 * atan(1.0) / 180.0;

  int debug = 0;

/* Locate the height jump edges */

  partition = partitions.begin() + partition_index;
  height_jump_edges = HeightJumpsInPartition(partition_corners,
                                             partition->LineTopologyReference(),
                                             intersection_segments, contours,
                                             planes, edges,
                                             min_dist_height_jump_partition,
                                             min_angle_height_jump_partition,
                                             max_dist_point_to_height_jump,
                                             min_jump_height);

/* Convert partition to line segments */

  partition_edges = LineSegments2D(partition_corners,
                                   partition->LineTopologyReference());

/* Find the height jump edge with the best support for this partition */

  best_support = 0;
//  best_edge    = NULL;
  if (debug) printf("Partition %d\n", partition_index);
  for (edge=height_jump_edges.begin(); edge!=height_jump_edges.end(); edge++) {
    if (partition_edges.SegmentInPartition(edge->LineSegment2DRef(),
                                           len_present, len_missing)) {
      partition_edges.CheckPartitionSizes(partition_corners,
                                          partition->LineTopologyReference(),
                                          edge->Line2DReference(),
                                          size1, size2);
      support = MIN(size1, size2) * len_present / (len_present + len_missing);
      if (debug) printf("jump: %5.2f %5.2f %5.2f    %5.2f %5.2f  %5.2f\n",
                        len_present, len_missing, len_present + len_missing,
                        size1, size2, support);
      if ((support > min_partition_size ||
           MIN(size1, size2) / (size1 + size2) > min_partition_percentage) &&
           support > best_support) {
        best_support = support;
        best_edge    = edge;
      }
    }
  }

/* Find the intersection line with the best support for this partition.
 * Check again for collinearity with partition edges since partition edges
 * may have arised from new partitions that have not yet been tested for
 * collinearity.
 *
 * Modification: a good intersection edge is preferred over a better height
 * jump edge.
 */

//best_support /= 10.0;
  for (edge=intersection_segments.begin();
       edge!=intersection_segments.end(); edge++) {
    if (partition_edges.SegmentInPartition(edge->LineSegment2DRef(),
                                           len_present, len_missing)) {
      if (!partition_edges.Collinear(edge->Line2DReference(), 0.01 * degree,
                                     0.01)) {
        partition_edges.CheckPartitionSizes(partition_corners,
                                            partition->LineTopologyReference(),
                                            edge->Line2DReference(),
                                            size1, size2);
        support = MIN(size1, size2) * len_present / (len_present + len_missing);
        if (size1 > 1e10 || size2 > 1e10) {
          printf("Warning: Something's wrong in CheckPartitionSizes()\n");
          printf("         Partitioning rejected!\n");
          support = -1.0;
          size1 = 0.0;
          size2 = 0.1;
        }
        if (debug) printf("intr: %5.2f %5.2f %5.2f    %5.2f %5.2f  %5.2f\n",
                          len_present, len_missing, len_present + len_missing,
                          size1, size2, support);
        if ((support > min_partition_size ||
             MIN(size1, size2) / (size1 + size2) > min_partition_percentage) &&
             support > best_support) {
          best_support = support;
          best_edge    = edge;
        }
      }
    }
  }
 
/* If there is an edge with support, split the partition */

  if (best_support > 0.0) {
//if (best_edge) {
    partition_edges.SplitPartition(partition_corners,
                                   partition->LineTopologyReference(),
                                   best_edge->Line2DReference(),
                                   new_corners, new_partitions);
    PartitionSizes(partition_corners, new_corners, new_partitions,
                   size1, size2);
    printf("Partition %d split in two parts. Sizes are %5.2f and %5.2f\n",
           partition->Number(), size1, size2);
    if (!new_corners.empty())
      partition_corners.insert(partition_corners.end(),
                               new_corners.begin(), new_corners.end());

/* Replace current partition by first part and add second part */

    new_partitions[0].Number() = partition->Number();
    *partition = new_partitions[0];
    partition = partitions.end() - 1;
    new_partitions[1].Number() = partition->Number() + 1;
    partitions.push_back(new_partitions[1]);
    return(1);
  }
  return(0);
}

/*
--------------------------------------------------------------------------------
               Construct the roof face in a single partition
--------------------------------------------------------------------------------
*/

bool LaserPoints::ConstructRoofFace(ObjectPoints2D &partition_corners,
                                    LineTopology &partition,
                                    Planes &planes,
                                    ObjectPoints &roofface_corners,
                                    LineTopologies &roofface_topology)
{
  int                       num_labels, *face_count, num_unlabeled_points,
                            best_label, label, next_corner_number,
                            next_face_number, success, best_plane_found;
  double                    *plane_fit, dist, best_fit;
  Planes::iterator          plane, best_plane;
  LaserPoints::iterator     point;
  ObjectPoints::iterator    corner;
  ObjectPoint               new_corner;
  LineTopologies::iterator  face;
  LineTopology              new_face;
  ObjectPoint2D             *corner2d;
  PointNumberList::iterator node;

/* Determine the best fitting plane in the partition */

  plane = planes.end() - 1;
  num_labels = plane->Number() + 1;
  face_count = (int *) calloc(num_labels, sizeof(int));
  plane_fit  = (double *) calloc(num_labels, sizeof(double));
  num_unlabeled_points = 0;
  for (point=begin(); point!=end(); point++) {
    if (point->InsidePolygon(partition_corners, partition)) {
      if (point->Label() >= num_labels) num_unlabeled_points++;
      else {
        face_count[point->Label()]++;
        for (plane=planes.begin(); plane!=planes.end(); plane++) {
          dist = plane->Z_At(point->X(), point->Y(), &success) -
                 point->Z();
          plane_fit[plane->Number()] += dist * dist;
        }
      }
    }
  }
  for (label=0, best_fit=MAXDOUBLE; label<num_labels; label++) {
    if (face_count[label] > 0 && plane_fit[label] < best_fit) {
      best_fit   = plane_fit[label];
      best_label = label;
    }
  }
  if (!best_fit == MAXDOUBLE) return(0); /* No labeled point in partition */

/* Locate the corresponding plane */

  best_plane_found = 0;
  for (plane=planes.begin(); plane!=planes.end() && !best_plane_found; plane++)
    if (plane->Number() == best_label) {
      best_plane = plane;
      best_plane_found = 1;
    }
  if (!best_plane_found) {
    printf("Error: best plane not found in ConstructRoofFace\n");
    return(0);
  }

/* Determine the next corner and face numbers */

  for (corner=roofface_corners.begin(), next_corner_number=0;
       corner!=roofface_corners.end(); corner++)
    if (corner->Number() >= next_corner_number)
      next_corner_number = corner->Number() + 1;
  for (face=roofface_topology.begin(), next_face_number=0;
       face!=roofface_topology.end(); face++)
    if (face->Number() >= next_face_number)
      next_face_number = face->Number() + 1;

/* Construct the roof face */

  for (node=partition.begin(); node!=partition.end(); node++) {

/* Construct new corner and add it to the new face */

    if (node != partition.end() - 1) {
      corner2d = partition_corners.GetPoint(*node);
      new_corner.Number() = next_corner_number;
      new_corner.X() = corner2d->X();
      new_corner.Y() = corner2d->Y();
      new_corner.Z() = best_plane->Z_At(new_corner.X(), new_corner.Y(),
                                        &success);
      roofface_corners.push_back(new_corner);
      new_face.push_back(new_corner.NumberRef());
      next_corner_number++;
    }
    else // Close the polygon
      new_face.push_back(new_face.begin()->NumberRef());
 }
 new_face.Label()  = best_label;
 new_face.Number() = next_face_number;
 roofface_topology.push_back(new_face);
 return(1);
}
