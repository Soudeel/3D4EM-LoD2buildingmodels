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
 Functions for operations on partitions

 double PartitionSize                            Size of a convex polygon
   (ObjectPoints2D &, LineTopology &)
 void PartitionSizes                             Size of two convex polygons
   (ObjectPoints2D &, ObjectPoints2D &,
    LineTopologies &, double &, double &)
 void LineSegments2D::SplitPartition             Split a partition by a segment
   (ObjectPoints2D &, LineTopology &, 
    Line2D &,
    ObjectPoints2D &, LineTopologies &)
 void LineSegments2D::CheckPartitionSizes        Check partition sizes without
   (ObjectPoints2D &, LineTopology &,            actually partitioning
    Line2D &, double &, double &)

 Initial creation
 Author : George Vosselman
 Date   : 26-04-2001

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
#include "Plane.h"       // local
#include "Position3D.h"
#include "Line2D.h"
#include "Line3D.h"
#include "ObjectPoints2D.h"
#include "ObjectPoints.h"
#include "PointNumberLists.h"
#include "Positions2D.h"
#include "LineSegments2D.h"
#include "stdmath.h"

#ifndef MAXDOUBLE
#define MAXDOUBLE 1e30
#endif

/*
--------------------------------------------------------------------------------
                        Determine the size of a partition
--------------------------------------------------------------------------------
*/

double PartitionSize(ObjectPoints2D &corners, LineTopology &partition)
{
  ObjectPoint2D             *p1, *p2, *p3;
  double                    size;
  PointNumberList::iterator node;

  if (partition.size() < 3) return(0);
  node = partition.begin();
  p1 = corners.GetPoint(node->NumberRef());
  node++;
  p2 = corners.GetPoint(node->NumberRef());
  for (node++, size=0; node!=partition.end(); node++) {
    p3 = corners.GetPoint(node->NumberRef());
    size += fabs(0.5 * (
                 p1->X() * p2->Y() + p2->X() * p3->Y() + p3->X() * p1->Y() -
                 p1->Y() * p2->X() - p2->Y() * p3->X() - p3->Y() * p1->X()));
    p2 = p3;
  }
  return(size);
}

/*
--------------------------------------------------------------------------------
                      Determine the sizes of two partitions
--------------------------------------------------------------------------------
*/

void PartitionSizes(ObjectPoints2D &partition_corners,
                    ObjectPoints2D &new_corners,
                    LineTopologies &new_partitions,
                    double &size1, double &size2)
{

/* Temporarily add the new corners to the list of corners */

  if (!new_corners.empty())
    partition_corners.insert(partition_corners.end(),
                             new_corners.begin(), new_corners.end());
    
/* Determine sizes */

  size1 = PartitionSize(partition_corners, new_partitions[0]);
  size2 = PartitionSize(partition_corners, new_partitions[1]);

/* Remove additional points from corner list */

  if (!new_corners.empty())
    partition_corners.erase(partition_corners.end() - new_corners.size(), 
                            partition_corners.end());
}

/*
--------------------------------------------------------------------------------
                            Split the partition
--------------------------------------------------------------------------------
*/

void LineSegments2D::SplitPartition(ObjectPoints2D &partition_corners,
                                    LineTopology &partition,
                                    Line2D &intersection_line,
                                    ObjectPoints2D &new_corners,
                                    LineTopologies &new_partitions)
{
  int                       i, point_index, point_index1, point_index2,
                            edge_index, edge_index1, edge_index2;
  PointNumber               pn, point_number1, point_number2;
  PointNumberList::iterator node;
  LineTopology              new_partition;
  Position2D                pos1, pos2;
  double                    dist, min_dist1, min_dist2;
  LineSegment2D             intersection;
  LineSegments2D::iterator  edge;
  ObjectPoint2D             *point, new_point;
  ObjectPoints2D::iterator  last_point;

  int debug = 0;

/* Intersect the line with the partition */

  if (!IntersectPartition(intersection_line, intersection,
                                          0.01)) {
    printf("Error: no intersection found in SplitPartition\n");
    exit(0);
  }

/* Determine if the begin or end point is near a corner */

  pos1 = intersection.BeginPoint();
  pos2 = intersection.EndPoint();
  point_index1 = point_index2 = -1;
  for (point_index=0, node=partition.begin();
       node!=partition.end();
       point_index++, node++) {
    point = partition_corners.GetPoint(*node);
    if (point_index1 < 0) 
      if (pos1.Distance(point->Position2DRef()) < 0.1) {
        point_index1 = point_index;
        point_number1 = *node;
      }
    if (point_index2 < 0) 
      if (pos2.Distance(point->Position2DRef()) < 0.1) {
        point_index2 = point_index;
        point_number2 = *node;
      }
  }

/* Add new partition corners */

  last_point = partition_corners.end() - 1;
  if (!new_corners.empty()) 
    new_corners.erase(new_corners.begin(), new_corners.end());
  if (point_index1 < 0) {
    new_point.Position2DRef() = pos1;
    new_point.Number() = last_point->Number() + 1;
    new_corners.push_back(new_point);
    point_number1 = new_point.NumberRef();
    last_point = new_corners.begin();
  }
  if (point_index2 < 0) {
    new_point.Position2DRef() = pos2;
    new_point.Number() = last_point->Number() + 1;
    new_corners.push_back(new_point);
    point_number2 = new_point.NumberRef();
  }

/* Determine the nearest segment if there is no nearest point */

  edge_index1 = edge_index2 = -1;
  min_dist1 = min_dist2 = MAXDOUBLE;
  for (edge_index=0, edge=begin(); edge!=end(); edge_index++, edge++) {
    if (point_index1 < 0) {
      dist = edge->Distance(pos1);
      if (dist < min_dist1) {
        min_dist1 = dist;
        edge_index1 = edge_index;
      }
    }
    if (point_index2 < 0) {
      dist = edge->Distance(pos2);
      if (dist < min_dist2) {
        min_dist2 = dist;
        edge_index2 = edge_index;
      }
    }
  }

  if (debug) printf("edge1 %d edge2 %d point1 %d point2 %d\n",
                    edge_index1, edge_index2, point_index1, point_index2);

/* Case 1: Two new nodes inserted in an edge */

  if (edge_index1 >= 0 && edge_index2 >= 0) {
    if (edge_index1 > edge_index2) {
      i=edge_index1; edge_index1=edge_index2; edge_index2=i;
      pn=point_number1; point_number1=point_number2; point_number2=pn;
    }

    new_partition.erase(new_partition.begin(), new_partition.end());
    for (i=0; i<=edge_index1; i++) new_partition.push_back(partition[i]);
    new_partition.push_back(point_number1);
    new_partition.push_back(point_number2);
    for (i=edge_index2+1; i<partition.size(); i++)
      new_partition.push_back(partition[i]);
    new_partitions.push_back(new_partition);

    new_partition.erase(new_partition.begin(), new_partition.end());
    new_partition.push_back(point_number1);
    for (i=edge_index1+1; i<=edge_index2; i++)
      new_partition.push_back(partition[i]);
    new_partition.push_back(point_number2);
    new_partition.push_back(point_number1);
    new_partitions.push_back(new_partition);
  }

/* Case 2: Both nodes of the intersection line are old partition corners */

  else if (point_index1 >= 0 && point_index2 >= 0) {
    if (point_index1 > point_index2) {
      i=point_index1; point_index1=point_index2; point_index2=i;
    }

    new_partition.erase(new_partition.begin(), new_partition.end());
    for (i=0; i<=point_index1; i++) new_partition.push_back(partition[i]);
    for (i=point_index2; i<partition.size(); i++)
      new_partition.push_back(partition[i]);
    new_partitions.push_back(new_partition);

    new_partition.erase(new_partition.begin(), new_partition.end());
    for (i=point_index1; i<=point_index2; i++)
      new_partition.push_back(partition[i]);
    new_partition.push_back(partition[point_index1]);
    new_partitions.push_back(new_partition);
  }

/* Otherwise, one new and one old partition corner. First check which one
 * comes first. Then distiguish two other cases.
 */

  else {
    if (edge_index1 >= 0) {
      if (point_index2 <= edge_index1) {
        point_index1  = point_index2;  point_index2 = -1;
        pn=point_number1; point_number1=point_number2; point_number2=pn;
        edge_index2   = edge_index1;   edge_index1  = -1;
      }
    }
    else {   // edge_index2 >= 0
      if (point_index1 > edge_index2) {
        point_index2  = point_index1;  point_index1 = -1;
        pn=point_number1; point_number1=point_number2; point_number2=pn;
        edge_index1   = edge_index2;   edge_index2  = -1;
      }
    }

/* Case 3: First node of intersection line is an old partition corner */

    if (point_index1 >= 0) {
      new_partition.erase(new_partition.begin(), new_partition.end());
      for (i=0; i<=point_index1; i++) new_partition.push_back(partition[i]);
      new_partition.push_back(point_number2);
      for (i=edge_index2+1; i<partition.size(); i++)
        new_partition.push_back(partition[i]);
      new_partitions.push_back(new_partition);

      new_partition.erase(new_partition.begin(), new_partition.end());
      for (i=point_index1; i<=edge_index2; i++)
        new_partition.push_back(partition[i]);
      new_partition.push_back(point_number2);
      new_partition.push_back(partition[point_index1]);
      new_partitions.push_back(new_partition);
    }

/* Case 4: Second node of intersection line is an old partition corner */

    else {
      new_partition.erase(new_partition.begin(), new_partition.end());
      for (i=0; i<=edge_index1; i++) new_partition.push_back(partition[i]);
      new_partition.push_back(point_number1);
      for (i=point_index2; i<partition.size(); i++)
        new_partition.push_back(partition[i]);
      new_partitions.push_back(new_partition);

      new_partition.erase(new_partition.begin(), new_partition.end());
      new_partition.push_back(point_number1);
      for (i=edge_index1+1; i<=point_index2; i++)
        new_partition.push_back(partition[i]);
      new_partition.push_back(point_number1);
      new_partitions.push_back(new_partition);
    }
  }
}

/*
--------------------------------------------------------------------------------
                                Check partition sizes
--------------------------------------------------------------------------------
*/

void LineSegments2D::CheckPartitionSizes(ObjectPoints2D &partition_corners,
                                         LineTopology &partition,
                                         Line2D &intersection_line,
                                         double &size1, double &size2)
{
  ObjectPoints2D new_corners;
  LineTopologies new_partitions;

/* Just split the partitions */

  SplitPartition(partition_corners, partition, intersection_line,
                 new_corners, new_partitions);

/* Get the sizes */

  PartitionSizes(partition_corners, new_corners, new_partitions,
                 size1, size2);
}
