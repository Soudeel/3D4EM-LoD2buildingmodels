
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
 Functions for class LaserPoints, originally used for building reconstruction
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Position2D.h"
#include "LineTopologies.h"
#include "Plane.h"
#include "DataBoundsLaser.h"
#include <Matrix3.h> 
#include "LaserPoints.h"
#include "PointNumberLists.h"
#include "Adjustment.h"
#include "Rotation2D.h"
#include "stdmath.h"

#include "digphot_arch.h"
#ifdef windows
#define HUGE 1e30
#endif

using namespace std;

/*
--------------------------------------------------------------------------------
                       Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void Update_Normal_Eq(double *, double, double, double *, double *,
                                 int);
extern "C" void Solve_Normal_Eq(double *, double *, int);

/*
--------------------------------------------------------------------------------
                     Fit a plane to a subset of points
--------------------------------------------------------------------------------
*/

/* Fit a plane to the points with a certain attribute value */

Plane LaserPoints::FitPlane(int value1, int value2, 
		            const LaserPointTag attribute) const
{
  LaserPoints::const_iterator point;
  Plane                       plane;
  int                         value;

  for (point=begin(); point!=end(); point++) {
    value = point->Attribute(attribute);
    if (value == value1 || value == value2) {
      plane.AddPoint(*point, false);
    }
  }
  plane.Recalculate();
  plane.Number() = value1;
 
  return(plane);
}

/* Fit a plane to the points with a certain long segment number */

Plane LaserPoints::FitPlane(long long int number) const
{
  LaserPoints::const_iterator point;
  Plane                       plane;

  for (point=begin(); point!=end(); point++) {
    if (point->LongSegmentNumber() == number) {
      plane.AddPoint(*point, false);
    }
  }
  plane.Recalculate();
 
  return(plane);
}

/* Fit a plane to the points in the point number list */

Plane LaserPoints::FitPlane(const PointNumberList &nodes,
                            int plane_number) const
{
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;
  double                          *a, *ata, *aty, norm;
  Plane                           plane;

  a    = (double *) calloc(3, sizeof(double));
  ata  = (double *) calloc(9, sizeof(double));
  aty  = (double *) calloc(3, sizeof(double));
  a[2] = 1.0;

  for (node=nodes.begin(); node!=nodes.end(); node++) {
    point = begin() + node->Number();
    a[0] = -point->X();
    a[1] = -point->Y();
    Update_Normal_Eq(a, point->Z(), 1.0, ata, aty, 3);
  }
  Solve_Normal_Eq(ata, aty, 3);
  Vector3D normal = Vector3D(aty[0], aty[1], 1.0);
  norm   = normal.Length();
  plane.Normal() = normal.Normalize();
  plane.Distance() = aty[2] / norm;
  plane.Number() = plane_number;
  return(plane);
}

/* Fit a line to a sequence of points */

Line3D LaserPoints::FitLine(LaserPoints::const_iterator first_point,
                            LaserPoints::const_iterator last_point) const
{
  LaserPoints::const_iterator     point;
  Line3D                          line;
  Plane                           plane;

  for (point=first_point; point!=last_point+1; point++)
    line.AddPoint(point->Position3DRef(), false);
  line.Recalculate();
  
  return line;
}

/*
--------------------------------------------------------------------------------
            Label all points within some distance of the plane
--------------------------------------------------------------------------------
*/

int LaserPoints::Label(const Plane &plane, double max_dist, int old_label1,
                       int old_label2, int near_label, int far_label,
                       const LaserPointTag label_tag)
{
  LaserPoints::iterator point;
  int                   num_pts=0, old_label;
  
  for (point=begin(); point!=end(); point++) {
    old_label = point->Attribute(label_tag);
    if (old_label == old_label1 || old_label == old_label2) {
      if (fabs(point->Distance(plane)) <= max_dist) {
        point->Attribute(label_tag) = near_label;
        num_pts++;
      }
      else point->Attribute(label_tag) = far_label;
    }
  }
  return(num_pts);
}

/*
--------------------------------------------------------------------------------
                      (Re-)Set all labels of a point set
--------------------------------------------------------------------------------
*/

void LaserPoints::Label(const PointNumberList &node_list, int label,
                        const LaserPointTag label_tag )
{
  PointNumberList::const_iterator node;
  LaserPoints::iterator           point;

  for (node=node_list.begin(); node!=node_list.end(); node++) {
    point = begin() + node->Number();
    point->Attribute(label_tag) = label;
  }
}

/*
--------------------------------------------------------------------------------
                   Label all points inside the polygon
--------------------------------------------------------------------------------
*/
void LaserPoints::LabelInside(const ObjectPoints2D &objpts,
                              const PointNumberList &nodes, int label,
                              const LaserPointTag label_tag)
{
  LaserPoints::iterator point;

  for (point=begin(); point!=end(); point++)
    if (point->InsidePolygon(objpts, nodes))
      point->Attribute(label_tag) = label;
}

/*
--------------------------------------------------------------------------------
                         Initialise the Hough space
--------------------------------------------------------------------------------
*/

void LaserPoints::InitialiseHoughSpace(HoughSpace &houghspace, int label,
                                       double slope_max,
                                       double slope_bin, double dist_bin,
                                       int equation,
                                       const LaserPointTag label_tag)
{
  DataBoundsLaser labelbounds;
  double          dist_min, dist_max, slope_min, slope_min2, slope_max2;
  int             dist_dim, slope_dim, slope_dim2;

  // Determine the bounds based on all points or points with a specific label
  if (label == -1) labelbounds = DeriveDataBounds(1);
  else labelbounds = DeriveDataBoundsTaggedPoints(label, label_tag);

  // Determine the extent of the coordinates and the Hough space parameters
  switch (equation) {
    case 0: // v1 X + v2 Y + v3 = Z
      dist_min  = -labelbounds.ZRange()/2.0 -
                  labelbounds.XRange() * slope_max / 2.0 -
                  labelbounds.YRange() * slope_max / 2.0;
      dist_max  = labelbounds.ZRange()/2.0 +
                  labelbounds.XRange() * slope_max / 2.0 +
                  labelbounds.YRange() * slope_max / 2.0;
      slope_min  = -slope_max;
      slope_min2 = slope_min;
      slope_max2 = slope_max;
      break;
    case 1: // sin v1 sin v2 X + sin v1 cos v2 Y + cos v1 = v3
      dist_max = (labelbounds.Maximum() - labelbounds.Minimum()).Length() / 2.0;
      dist_min = -dist_max;
      dist_dim  = (int) ((dist_max - dist_min) / dist_bin + 0.01);
      slope_min  = 0.0;
      slope_min2 = 0.0;
      slope_max2 = 8 * atan(1.0); // 2 pi
      break;
    default:
      printf("Error: Invalid equation type %d in LaserPoints::InitialiseHoughSpace\n", equation);
      exit(0);
  }

  // Determine the size of the Hough space
  dist_dim   = (int) ((dist_max - dist_min) / dist_bin + 0.01);
  if (dist_dim == 0) {dist_dim = 1; dist_max = dist_min + dist_bin / 2.0;}
  slope_dim  = (int) ((slope_max - slope_min) / slope_bin + 0.01);
  slope_dim2 = (int) ((slope_max2 - slope_min2) / slope_bin + 0.01);

  // Initialise the Hough space
  houghspace.Initialise(slope_dim, slope_dim2, dist_dim,
                        slope_min, slope_min2, dist_min,
                        slope_max, slope_max2, dist_max, equation);

  // Set the coordinate offsets
  houghspace.SetOffsets(
     (labelbounds.Minimum().X() + labelbounds.Maximum().X())/2.0,
     (labelbounds.Minimum().Y() + labelbounds.Maximum().Y())/2.0,
     (labelbounds.Minimum().Z() + labelbounds.Maximum().Z())/2.0);
}

/*
--------------------------------------------------------------------------------
                         Add all points to the Hough space
--------------------------------------------------------------------------------
*/

void LaserPoints::IncrementHoughSpace(HoughSpace &space) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    space.AddPoint(point->X(), point->Y(), point->Z());
}

void LaserPoints::IncrementHoughSpace(HoughSpace &space, int label,
                                      const LaserPointTag label_tag) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    if (point->Attribute(label_tag) == label)
      space.AddPoint(point->X(), point->Y(), point->Z());
}

/*
--------------------------------------------------------------------------------
                    Derive the bounds of the points with some label
--------------------------------------------------------------------------------
*/

DataBoundsLaser LaserPoints::DeriveDataBoundsTaggedPoints
  (int value, const LaserPointTag tag) const
{
  LaserPoints::const_iterator point;
  DataBoundsLaser             bounds;

  bounds.Initialise();
  for (point=begin(); point!=end(); point++)
    if (point->Attribute(tag) == value)
      bounds.Update(point->LaserPointRef());

  return(bounds);
}

/*
--------------------------------------------------------------------------------
                    Check if the points are in a plane
--------------------------------------------------------------------------------
*/

int LaserPoints::PointsInPlane(const PointNumberList &pointlist,
                               const Plane &plane,
                               double max_dist, double min_perc) const
{
  PointNumberList::const_iterator node;
  int                             num_near = 0;
  LaserPoints::const_iterator     point;

  for (node=pointlist.begin(); node!=pointlist.end(); node++) {
    point = begin() + node->Number();
    if (ABS(point->Distance(plane)) <= max_dist) num_near++;
  }
  return(num_near >= pointlist.size() * min_perc / 100.0);
}

/*
--------------------------------------------------------------------------------
                  Determine the centre of a point list
--------------------------------------------------------------------------------
*/

ObjectPoint LaserPoints::Centroid(const PointNumberList &nodes,
                                  int point_number) const
{
  PointNumberList::const_iterator node;
  LaserPoints::const_iterator     point;
  ObjectPoint                     centre;

  centre.X() = 0;
  centre.Y() = 0;
  centre.Z() = 0;
  for (node=nodes.begin(); node!=nodes.end(); node++) {
    point = begin() + node->Number();
    centre.X() += point->X();
    centre.Y() += point->Y();
    centre.Z() += point->Z();
  }
  centre.X() /= nodes.size();
  centre.Y() /= nodes.size();
  centre.Z() /= nodes.size();
  centre.Number() = point_number;
  return(centre);
}

/*
--------------------------------------------------------------------------------
                       Intersect two faces
--------------------------------------------------------------------------------
*/
int LaserPoints::IntersectFaces(const PointNumberList &face1,
                                const PointNumberList &face2,
                                const Plane &plane1, const Plane &plane2,
                                double max_dist,
                                Position3D &point1, Position3D &point2) const
{
  Line3D line;
  double begin1, end1, begin2, end2;
  bool report_error=false;

  // Intersect the planes
  if (!Intersect2Planes(plane1, plane2, line)) {
    if (report_error) printf("LaserPoint::IntersectFaces error: Planes do not intersect\n");
    return(0);
  }

  // Check the range of support for both faces
  if (!FaceNearLine(face1, line, max_dist, begin1, end1)) {
    if (report_error) printf("LaserPoint::IntersectFaces error: No points of face 1 near line\n");
    return(0);
  }
  if (!FaceNearLine(face2, line, max_dist, begin2, end2)) {
    if (report_error) printf("LaserPoint::IntersectFaces error: No points of face 2 near line\n");
    return(0);
  }

  // Determine the overlap in the scalar range
  if (begin1 > end2 || begin2 > end1) {
    if (report_error) printf("LaserPoint::IntersectFaces error: Line segments do not overlap\n");
    return(0);
  }
  begin1 = MAX(begin1, begin2);
  end1   = MIN(end1, end2);

  // Derive the points
  point1 = line.Position(begin1);
  point2 = line.Position(end1);
  return(1);
}

/*
--------------------------------------------------------------------------------
                       Determine line segment near face
--------------------------------------------------------------------------------
*/

int LaserPoints::FaceNearLine(const PointNumberList &face, const Line3D &line,
                              double max_dist,
                              double &sbegin, double &send) const
{
  PointNumberList::const_iterator node;
  LaserPoints::const_iterator     point;
  int                             found_point;
  double                          scalar, dist, mindist;

  int debug=0;

  mindist = 100000000.0; // debugging
  found_point = 0;
  if (debug) printf("Points of face (size %d)\n", face.size());
  for (node=face.begin(); node!=face.end(); node++) {
    point = begin() + node->Number();

    if (debug) {
      dist = line.DistanceToPoint(point->Position3DRef());
      if (dist < mindist) mindist = dist;
    }

    if (line.DistanceToPoint(point->Position3DRef()) <= max_dist) {
      scalar = line.Scalar(point->Position3DRef());
      if (found_point) {
        if (scalar < sbegin) sbegin = scalar;
        if (scalar > send)     send = scalar;
      }
      else {
        sbegin = send = scalar;
      }
      found_point++;
    }
  }
  if (debug)
    printf("\n%d near points found over distance %6.2f, nearest is %6.2f\n",
           found_point, send - (sbegin), mindist);
  if (found_point == 1) return 0; // One point doesn't give a line segment
  return(found_point);
}

/*
--------------------------------------------------------------------------------
              Derive normal vector at the position of a laser point
--------------------------------------------------------------------------------
*/

Vector3D LaserPoints::NormalAtPoint(const PointNumber &node,
                                    const TINEdges &edges) const
{
  PointNumberList node_list;
  Plane           plane;

  node_list = edges[node.Number()];
  node_list.push_back(node);

  plane = FitPlane(node_list, 0);
  return(plane.Normal());
}

/*
--------------------------------------------------------------------------------
                    Select points within polygons
--------------------------------------------------------------------------------
*/

int LaserPoints::Select(LaserPoints &selection, const ObjectPoints &corners,
                        const LineTopologies &polygons, bool set_IsSelectedTag)
{
  LineTopologies::const_iterator polygon;
  LaserPoints::iterator          point;
  int                            old_size = selection.size();
  DataBoundsLaser                polygon_bounds;

  DeriveDataBounds(1); // Make sure we have laser data bounds
  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
    polygon_bounds = DataBoundsLaser(polygon->Bounds(corners));
    // Only check points if the bounds of the laser data and polygon overlap
    if (OverlapXY(bounds, polygon_bounds)) {
      for (point=begin(); point!=end(); point++) {
        if (point->InsidePolygon(corners, polygon->PointNumberListReference())) {
          selection.push_back(*point);
          if (set_IsSelectedTag) point->Select();
        }
      }
    }
  }
  return(selection.size() - old_size);
}

/*
--------------------------------------------------------------------------------
                         Select the nearest point
--------------------------------------------------------------------------------
*/

int LaserPoints::NearestPoint(const Position3D &pos, const TINEdges &edges,
                              bool planimetric) const
{
  LaserPoints::const_iterator point;
  double                      dist, best_dist;
  TINEdges::const_iterator    nodes;
  TINEdgeSet::const_iterator  node;
  PointNumber                 best_node;
  bool                        improved;
  
  if (empty()) return -1;
  // Take the first point as start for the search of the nearest point
  best_node = PointNumber(0);
  if (planimetric) best_dist = (begin()->vect2D() - pos.vect2D()).Length();
  else best_dist = (begin()->vect() - pos.vect()).Length();
  do {
    improved = false;
    // Loop over the neighbours of the nearest point so far
    nodes = edges.begin() + best_node.Number();
    for (node=nodes->begin(); node!=nodes->end(); node++) {
      point = begin() + node->Number();
      if (planimetric) dist = (point->vect2D() - pos.vect2D()).Length();
      else dist = (point->vect() - pos.vect()).Length();
      // Update the best node
      if (dist < best_dist) {
        improved = true;
        best_dist = dist;
        best_node = *node;
      }
    }
  // Continue searching for nearer points if there has been an improvement
  // in this iteration
  } while (improved);
  return best_node.Number();
}

/*
--------------------------------------------------------------------------------
                         Select the nearest point
--------------------------------------------------------------------------------
*/

double LaserPoints::Distance2NearestPoint(Position3D pos, int nearestpoint)
{
  double      dist;
  LaserPoints::const_iterator point;
  if (empty()) return -1;

  // Take the first point as start for the search of the nearest point

      point = begin() + nearestpoint;
      dist = (point->vect() - pos.vect()).Length();
   return dist;
}

/*
--------------------------------------------------------------------------------
              Select the next point of a specified pulse type
--------------------------------------------------------------------------------
*/

LaserPoints::const_iterator LaserPoints::NextPointOfPulseType(
       LaserPulseType type, LaserPoints::const_iterator start_point) const
{
  LaserPoints::const_iterator point=start_point;
  
  if (point == end()) return end();
  
  // If pulse type is available, select accordingly
  if (start_point->HasAttribute(PulseCountTag)) {
    while (!point->IsPulseType(type) && point != end()) point++;
    return point;
  }
  
  else {
    switch (type) {
      case AnyPulse:
      case FirstPulse:
      case LastPulse:
      case SinglePulse:
        return point;
      default:
        return end();
    }
  }    
}

/*
--------------------------------------------------------------------------------
                Select the nearest point with a specific label
--------------------------------------------------------------------------------
*/

int LaserPoints::NearestTaggedPoint(const Position2D &pos, int value,
                                    LaserPoints::const_iterator &nearest_point,
                                    const LaserPointTag tag) const
{
  LaserPoints::const_iterator point;
  double                      dist, min_dist=HUGE;
  int                         point_selected;
  
  point_selected = 0;
  for (point=begin(); point!=end(); point++) {
    if (point->Attribute(tag) == value) {
      dist = (point->vect2D() - pos.vect()).Length();
      if (dist < min_dist) {
        nearest_point = point;
        min_dist = dist;
        point_selected = 1;
      }
    }
  }
  return(point_selected);
}

/*
--------------------------------------------------------------------------------
         Collect all points within a neighbourhood with a specific label
--------------------------------------------------------------------------------
*/

PointNumberList LaserPoints::TaggedNeighbourhood(const PointNumber &centre,
                                                 int value, double range,
                                                 const TINEdges &edges,
                                                 const LaserPointTag tag,
                                                 bool planimetric,
                                                 bool direct_neighbours_only) const
{
  PointNumberList             neighbourhood;
  PointNumberList::iterator   node;
  LaserPoints::const_iterator point;

  // First select all points in the neighbourhood
  neighbourhood = Neighbourhood(centre, range, edges, planimetric,
                                direct_neighbours_only);

  // Remove all points with other attribute values
  for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) {
    point = begin() + node->Number();
    if (point->Attribute(tag) != value) {
      neighbourhood.erase(node);
      node--;
    }
  }

  return(neighbourhood);
}

/*
--------------------------------------------------------------------------------
            Determine the most frequent value of the specified attribute
--------------------------------------------------------------------------------
*/
int LaserPoints::MostFrequentAttributeValue(LaserPointTag tag, int &count) const
{
  map<int, int>               value_counters;
  map<int, int>::iterator     value_counter;
  LaserPoints::const_iterator point;
  int                         value;
  
  count = 0;
  if (empty()) return -1;
  
  // Collect the value counts
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      value = point->Attribute(tag);
      value_counter = value_counters.find(value);
      if (value_counter == value_counters.end())
        value_counters.insert(pair<int, int>(value, 1));
      else
        value_counter->second++;
    }
  }
  // Determine the highest count
  for (value_counter=value_counters.begin(), count=0;
       value_counter!=value_counters.end(); value_counter++) {
    if (value_counter->second > count) {
      value = value_counter->first;
      count = value_counter->second;
    }
  }
  if (count == 0) return -1;
  return value;
}

int LaserPoints::MostFrequentAttributeValue(LaserPointTag tag, 
                                            const PointNumberList &list,
											int &count,
											int minimum_value) const
{
  map<int, int>                   value_counters;
  map<int, int>::iterator         value_counter;
  LaserPoints::const_iterator     point;
  int                             value;
  PointNumberList::const_iterator node;
  
  count = 0;
  if (empty()) return -1;
  
  // Collect the value counts
  for (node=list.begin(); node!=list.end(); node++) {
  	point = begin() + node->Number();
    if (point->HasAttribute(tag)) {
      value = point->Attribute(tag);
      if (value < minimum_value) continue;
      value_counter = value_counters.find(value);
      if (value_counter == value_counters.end())
        value_counters.insert(pair<int, int>(value, 1));
      else
        value_counter->second++;
    }
  }
  // Determine the highest count
  for (value_counter=value_counters.begin(), count=0;
       value_counter!=value_counters.end(); value_counter++) {
    if (value_counter->second > count) {
      value = value_counter->first;
      count = value_counter->second;
    }
  }
  if (count == 0) return -1;
  return value;
}


long long int LaserPoints::MostFrequentLongAttributeValue(LaserPointTag tag, 
                                                          const PointNumberList &list,
											              int &count,
											              int minimum_value) const
{
  map<long long int, int>                   value_counters;
  map<long long int, int>::iterator         value_counter;
  LaserPoints::const_iterator               point;
  long long int                             value;
  PointNumberList::const_iterator           node;
  
  count = 0;
  if (empty()) return -1;
  
  // Collect the value counts
  for (node=list.begin(); node!=list.end(); node++) {
  	point = begin() + node->Number();
    if (point->HasAttribute(tag)) {
      value = point->LongAttribute(tag);
      if (value < minimum_value) continue;
      value_counter = value_counters.find(value);
      if (value_counter == value_counters.end())
        value_counters.insert(pair<long long int, int>(value, 1));
      else
        value_counter->second++;
    }
  }
  // Determine the highest count
  for (value_counter=value_counters.begin(), count=0;
       value_counter!=value_counters.end(); value_counter++) {
    if (value_counter->second > count) {
      value = value_counter->first;
      count = value_counter->second;
    }
  }
  if (count == 0) return -1;
  return value;
}

/*
--------------------------------------------------------------------------------
       Create a point number list of points with a specific attribute value
--------------------------------------------------------------------------------
*/
void LaserPoints::TaggedPointNumberList(PointNumberList &list,
                                        const LaserPointTag tag, 
                                        int value) const
{
  LaserPoints::const_iterator point;
  int number;
  
  if (list.size()) list.erase(list.begin(), list.end());
  for (point=begin(), number=0; point!=end(); point++, number++)
    if (point->Attribute(tag) == value) list.push_back(PointNumber(number));
}

PointNumberList & LaserPoints::TaggedPointNumberList(const LaserPointTag tag,
                                                     int value) const
{
  PointNumberList *list = new PointNumberList();
  LaserPoints::const_iterator point;
  int number;
  
  for (point=begin(), number=0; point!=end(); point++, number++)
    if (point->Attribute(tag) == value) list->push_back(PointNumber(number));

  return *list;
}

/*
--------------------------------------------------------------------------------
       Calculates the noise for the plane parameters for a set of laser points 
       with a specified tag value, given the laser point noise value.
--------------------------------------------------------------------------------
*/

Matrix3 LaserPoints::QualityPlane(int value1, double point_noise, 
		            const LaserPointTag attribute) const
{
  LaserPoints::const_iterator point;
  double                      *a, *ata, *aty, covariance, meanx, meany;
  Matrix3                     inverse, matata;
  int                         value, count;
  
  a    = (double *) calloc(3, sizeof(double));
  ata  = (double *) calloc(9, sizeof(double));
  aty  = (double *) calloc(3, sizeof(double));
  a[2] = 1.0;
  count = 0;
for (point=begin(); point!=end(); point++) {
    value = point->Attribute(attribute);
    if (value == value1) {
       meanx = meanx + point->X();
       meany = meany + point->Y();       
       count++;
       }
       }
meanx = meanx/count;
meany = meany/count;       

  for (point=begin(); point!=end(); point++) {
    value = point->Attribute(attribute);
    if (value == value1) {
//      a[0] = -point->X();
//      a[1] = -point->Y();
      //Update_Normal_Eq(a, point->Z(), 1.0, ata, aty, 3);
      ata[0] = ata[0] + ((point->X()-meanx)*(point->X()-meanx));
      ata[1] = ata[1] + ((point->X()-meanx)*(point->Y()-meany));      
      ata[2] = ata[2] + (-(point->X()-meanx));
      ata[3] = ata[3] + ((point->X()-meanx)*(point->Y()-meany));
      ata[4] = ata[4] + ((point->Y()-meany)*(point->Y()-meany));
      ata[5] = ata[5] + (-(point->Y()-meany));      
      ata[6] = ata[6] + (-(point->X()-meanx));      
      ata[7] = ata[7] + (-(point->Y()-meany));      
      ata[8] = ata[8] + 1;
    }
  }
  if (point_noise == 0) point_noise = 1;
  covariance = point_noise * point_noise;
  matata = Matrix3(ata[0]/covariance, ata[1]/covariance, ata[2]/covariance,
                   ata[3]/covariance, ata[4]/covariance, ata[5]/covariance, 
                   ata[6]/covariance, ata[7]/covariance, ata[8]/covariance);
  inverse = matata.Inverse();
  return(inverse);
}


/*
--------------------------------------------------------------------------------
     Detection of a 3D line in a point cloud using RANSAC
         max_dist_to_line Maximum distance of a point to the detected line
         min_num_hist Minimum number of points close to the line for successful detection
         max_num_tries Maximum number of RANSAC iterations
         num_hits Number of points near the line if at least min_num_hits
         select_tag If unequal UndefinedTag, only points with value
                    select_value of tag select_tag are used for the line detection
         select_value See select_tag
--------------------------------------------------------------------------------
*/
Line3D LaserPoints::RANSAC3DLine(double max_dist_to_line, int min_num_hits,
                                 int max_num_tries, int &num_hits,
                                 double max_dist_between_points,
                                 LineSegments3D &segments,
                                 LaserPointTag select_tag,
                                 int select_value) const
{
  Line3D                      line;
  LaserPoints::const_iterator point, point1, point2;
  int                         num_pts, pt_no, random_number1, 
                              random_number2, num_tries;
  long                        random_long;
  bool                        found, changed;
  double                      scalar;
  LineSegments3D::iterator    segment1, segment2;
  
  num_hits = 0;
  
  // Count number of points with the specified attribute value
  if (select_tag == UndefinedTag) 
    num_pts = size();
  else {
    for (point=begin(), num_pts=0; point!=end(); point++)
      if (point->HasAttribute(select_tag))
        if (point->Attribute(select_tag) == select_value) num_pts++;
  }
  if (num_pts < min_num_hits) return line; // Can never succeed

  srand(1); // Initialise random number generator

  // Generate hypotheses
  num_tries = 0;
  do {
    num_tries++;
    // Delete old line data
    line.Erase();
    // Get two random points
    random_long = rand();
    random_number1 = random_long - (random_long / num_pts) * num_pts;
    if (select_tag == UndefinedTag)
      point1 = begin() + random_number1;
    else {
      for (point=begin(), pt_no=0, found=false;
           point!=end() && !found; point++) {
        if (point->HasAttribute(select_tag)) {
          if (point->Attribute(select_tag) == select_value) {
            if (pt_no == random_number1) {
              found = true;
              point1 = point;
            }
            else pt_no++;
          }
        }
      }
    }
    random_number2 = random_number1;
    while (random_number2 == random_number1) {
      random_long = rand();
      random_number2 = random_long - (random_long / num_pts) * num_pts;
    }
    if (select_tag == UndefinedTag)
      point2 = begin() + random_number2;
    else {
      for (point=begin(), pt_no=0, found=false;
           point!=end() && !found; point++) {
        if (point->HasAttribute(select_tag)) {
          if (point->Attribute(select_tag) == select_value) {
            if (pt_no == random_number2) {
              found = true;
              point2 = point;
            }
            else pt_no++;
          }
        }
      }
    }
    // Construct the 3D line
    line = Line3D(point1->Position3DRef(), point2->Position3DRef());
    // Count number of points near the line and collect data for recalculation
    for (point=begin(), num_hits=0; point!=end(); point++) {
      found = true;
      if (select_tag != UndefinedTag) {
        if (point->HasAttribute(select_tag)) {
          if (point->Attribute(select_tag) != select_value) found = false;
        }
        else found = false;
      }
      if (found) {
        if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
          num_hits++;
          line.AddPoint(point->Position3DRef(), false);
        }
      }
    }
    if (num_hits > min_num_hits) {
      // Fit line to all selected points
      line.Recalculate();
    
      // Make a segment from every point near the line, store number of points in segment number
      segments.erase(segments.begin(), segments.end());
      for (point=begin(), num_hits=0; point!=end(); point++) {
        found = true;
        if (select_tag != UndefinedTag) {
          if (point->HasAttribute(select_tag)) {
            if (point->Attribute(select_tag) != select_value) found = false;
          }
          else found = false;
        }
        if (found) {
          if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
            num_hits++;
            scalar = line.Scalar(point->Position3DRef());
            segments.push_back(LineSegment3D(line, scalar, scalar));
            (segments.end()-1)->Number() = 1;
          }
        }
      }

      // Merge segments
      segments.SortOnStartScalar();
      do {
        changed = false;
        for (segment1=segments.begin(); segment1<segments.end()-1; segment1++) {
          segment2 = segment1 + 1;
          if (segment2->ScalarBegin() - segment1->ScalarEnd() <=
              max_dist_between_points) {
            changed = true;
            segment1->ScalarEnd() = segment2->ScalarEnd();
            segment1->Number() = segment1->Number() + segment2->Number();
            segments.erase(segment2);
            segment1--;
          }
        }
      } while (changed);
      
      // Sort segments after the number of points
      segments.SortOnNumber();
      if (segments.empty()) num_hits = 0;
      else num_hits = segments.begin()->Number();
    }
  } while (num_tries < max_num_tries && num_hits < min_num_hits);
  
  if (num_hits < min_num_hits) segments.erase(segments.begin(), segments.end());
  return line;
}

/*
--------------------------------------------------------------------------------
Detection of a 3D line in a point cloud using RANSAC
max_dist_to_line Maximum distance of a point to the detected line
min_num_hist Minimum number of points close to the line for successful detection
max_num_tries Maximum number of RANSAC iterations
num_hits Number of points near the line if at least min_num_hits
select_tag If unequal UndefinedTag, only points with value
select_value of tag select_tag are used for the line detection
select_value See select_tag
--------------------------------------------------------------------------------
*/
Line3D LaserPoints::RANSAC3DRailTrack(double max_dist_to_line,
									  double gauge_distance, int min_num_hits,
									  int max_num_tries, int &num_hits,
									  double max_dist_between_points,
									  LineSegments3D &segments,
									  LineSegments3D &second_segments,
									  LaserPointTag select_tag,
									  int select_value) const
{
	Line3D                      line, secondline;
	Position3D				  pos3, pos4;
	LaserPoints::const_iterator point, point1, point2;
	int                         num_pts, pt_no, random_number1, 
		random_number2, num_tries, num_hits_second_line;
	long                        random_long;
	bool                        found, changed;
	double                      scalar;
	LineSegments3D::iterator    segment1, segment2;

	num_hits = 0;
	num_hits_second_line = 0;
	// Count number of points with the specified attribute value
	if (select_tag == UndefinedTag) 
		num_pts = size();
	else {
		for (point=begin(), num_pts=0; point!=end(); point++)
			if (point->HasAttribute(select_tag))
				if (point->Attribute(select_tag) == select_value) num_pts++;
	}
	if (num_pts < min_num_hits) return line; // Can never succeed

	srand(1); // Initialise random number generator

	// Generate hypotheses
	num_tries = 0;
	do {
		num_tries++;
		// Delete old line data
		line.Erase();
		// Get two random points
		random_long = rand();
		random_number1 = random_long - (random_long / num_pts) * num_pts;
		if (select_tag == UndefinedTag)
			point1 = begin() + random_number1;
		else {
			for (point=begin(), pt_no=0, found=false;
				point!=end() && !found; point++) {
					if (point->HasAttribute(select_tag)) {
						if (point->Attribute(select_tag) == select_value) {
							if (pt_no == random_number1) {
								found = true;
								point1 = point;
							}
							else pt_no++;
						}
					}
			}
		}
		random_number2 = random_number1;
		while (random_number2 == random_number1) {
			random_long = rand();
			random_number2 = random_long - (random_long / num_pts) * num_pts;
		}
		if (select_tag == UndefinedTag)
			point2 = begin() + random_number2;
		else {
			for (point=begin(), pt_no=0, found=false;
				point!=end() && !found; point++) {
					if (point->HasAttribute(select_tag)) {
						if (point->Attribute(select_tag) == select_value) {
							if (pt_no == random_number2) {
								found = true;
								point2 = point;
							}
							else pt_no++;
						}
					}
			}
		}
		// Construct the 3D line
		line = Line3D(point1->Position3DRef(), point2->Position3DRef());
		// Count number of points near the line and collect data for recalculation
		for (point=begin(), num_hits=0, num_hits_second_line=0; point!=end(); point++) {
			found = true;
			if (select_tag != UndefinedTag) {
				if (point->HasAttribute(select_tag)) {
					if (point->Attribute(select_tag) != select_value) found = false;
				}
				else found = false;
			}
			if (found) {
				if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
					num_hits++;
					line.AddPoint(point->Position3DRef(), false);
				}
				if (line.DistanceToPoint(point->Position3DRef()) <= gauge_distance+max_dist_to_line&&
					line.DistanceToPoint(point->Position3DRef()) >= gauge_distance-max_dist_to_line) {
						if (num_hits_second_line>1 && secondline.DistanceToPoint(point->Position3DRef())<=max_dist_to_line){
							secondline.AddPoint(point->Position3DRef(), false);
							num_hits_second_line++;			
						}

						if (num_hits_second_line==1 && point->Position3DRef().Distance(pos3)>0.5){
							pos4 = point->Position3DRef();
							secondline = Line3D(pos3, pos4);

							num_hits_second_line++;
						}

						if (num_hits_second_line==0) {
							pos3 = point->Position3DRef();
							num_hits_second_line++;
						}
				}

			}
		}
		if (num_hits_second_line<min_num_hits) num_hits=0;

		if (num_hits > min_num_hits && num_hits_second_line > min_num_hits) {
			// Fit line to all selected points
			line.Recalculate();
			secondline.Recalculate();
			// Make a segment from every point near the line, store number of points in segment number
			segments.erase(segments.begin(), segments.end());
			for (point=begin(), num_hits=0, num_hits_second_line=0; point!=end(); point++) {
				found = true;
				if (select_tag != UndefinedTag) {
					if (point->HasAttribute(select_tag)) {
						if (point->Attribute(select_tag) != select_value) found = false;
					}
					else found = false;
				}
				if (found) {
					if (line.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
						num_hits++;
						scalar = line.Scalar(point->Position3DRef());
						segments.push_back(LineSegment3D(line, scalar, scalar));
						(segments.end()-1)->Number() = 1;
					}
					if (secondline.DistanceToPoint(point->Position3DRef()) <= max_dist_to_line) {
						num_hits_second_line++;
						scalar = secondline.Scalar(point->Position3DRef());
						second_segments.push_back(LineSegment3D(secondline, scalar, scalar));
						(second_segments.end()-1)->Number() = 1;
					}

				}
			}

			// Merge segments
			segments.SortOnStartScalar();
			do {
				changed = false;
				for (segment1=segments.begin(); segment1<segments.end()-1; segment1++) {
					segment2 = segment1 + 1;
					if (segment2->ScalarBegin() - segment1->ScalarEnd() <=
						max_dist_between_points) {
							changed = true;
							segment1->ScalarEnd() = segment2->ScalarEnd();
							segment1->Number() = segment1->Number() + segment2->Number();
							segments.erase(segment2);
							segment1--;
					}
				}
			} while (changed);
			second_segments.SortOnStartScalar();
			do {
				changed = false;
				for (segment1=second_segments.begin(); segment1<second_segments.end()-1; segment1++) {
					segment2 = segment1 + 1;
					if (segment2->ScalarBegin() - segment1->ScalarEnd() <=
						max_dist_between_points) {
							changed = true;
							segment1->ScalarEnd() = segment2->ScalarEnd();
							segment1->Number() = segment1->Number() + segment2->Number();
							second_segments.erase(segment2);
							segment1--;
					}
				}
			} while (changed);

			// Sort segments after the number of points
			segments.SortOnNumber();
			second_segments.SortOnNumber();

			if (segments.empty()) num_hits = 0;
			else num_hits = segments.begin()->Number();
			if (num_hits_second_line<min_num_hits) num_hits=0;
		}
	} while (num_tries < max_num_tries && num_hits < min_num_hits && num_hits_second_line < min_num_hits);

	if (num_hits < min_num_hits) {
		segments.erase(segments.begin(), segments.end());
		second_segments.erase(second_segments.begin(), second_segments.end());
	}
	return line;
}

/*
--------------------------------------------------------------------------------
Detection of 3D line segments in a point cloud using RANSAC, parallel to a given 2d line segment
max_dist_to_line Maximum distance of a point to the detected line
min_num_hist Minimum number of points close to the line for successful detection
max_num_tries Maximum number of RANSAC iterations
num_hits Number of points near the line if at least min_num_hits
select_tag If unequal UndefinedTag, only points with value
select_value of tag select_tag are used for the line detection
select_value See select_tag
--------------------------------------------------------------------------------
*/

#include "LineSegments2D.h"
LineSegments2D LaserPoints::RANSAC2DLineParallel(double max_dist_to_line, int min_num_hits,
												 int max_num_tries, int &num_hits,
												 double max_dist_between_points, double minimum_length,
												 LineSegment2D prefline,
												 LaserPointTag select_tag,
												 int select_value) 
{
	Line2D                      line;
	LaserPoints::iterator point, point1, point2;
	LaserPoints					temppoints, processpoints;
	int                         num_pts, pt_no, random_number1, 
		random_number2, num_tries, perc=0, iter;
	long                        random_long;
	bool                        found, changed;
	double                      scalar, diffx, diffy;
	LineSegments2D::iterator    segment1, segment2;
	LineSegments2D				segments, allsegments;
	Position2D				  beginpoint, endpoint, pos1, pos2;
	num_hits = 0;
	beginpoint = prefline.BeginPoint();
	endpoint = prefline.EndPoint();
	diffx = beginpoint.X() - endpoint.X();
	diffy = beginpoint.Y() - endpoint.Y();
	diffx=0.1*diffx/prefline.Length();
	diffy=0.1*diffy/prefline.Length();
	this->SetAttribute(IsProcessedTag, 0);
	processpoints = *this;
	processpoints.SetUnFiltered();
	// Count number of points with the specified attribute value
	if (select_tag == UndefinedTag) 
		num_pts = processpoints.size();
	else {
		for (point=processpoints.begin(), num_pts=0; point!=processpoints.end(); point++)
			if (point->HasAttribute(select_tag))
				if (point->Attribute(select_tag) == select_value) num_pts++;
	}
	if (num_pts < min_num_hits) return segments; // Can never succeed
	iter=0;
	
	do{
		iter++;
		num_pts = processpoints.size();
		segments.erase(segments.begin(), segments.end());
		srand(1); // Initialise random number generator

		// Generate hypotheses
		num_tries = 0;
		do {
			num_tries++;
			// Delete old line data
			//    line.Erase();
			// Get two random points
			random_long = rand();
			random_number1 = random_long - (random_long / num_pts) * num_pts;
			if (select_tag == UndefinedTag){
				point1 = processpoints.begin() + random_number1;
				/*   for (point=begin(), pt_no=0, found=false;
				point!=end() && !found; point++) {
				if (!point->Processed()) {
				if (pt_no == random_number1) {
				found = true;
				point1 = point;
				}
				else pt_no++;
				}
				}
				*/}
			else {
				for (point=processpoints.begin(), pt_no=0, found=false;
					point!=processpoints.end() && !found; point++) 
				{
						if (point->HasAttribute(select_tag) && !point->Processed()) {
							if (point->Attribute(select_tag) == select_value) {
								if (pt_no == random_number1) {
									found = true;
									point1 = point;
								}
								else pt_no++;
							}
						}
				}
			}
			// construct 2d line, using preferred line direction
			pos1=Position2D(point1->X(), point1->Y());
			pos2=Position2D(point1->X()+diffx, point1->Y()+diffy);
			temppoints.ErasePoints();
			line = Line2D(pos1, pos2);
			// Count number of points near the line and collect data for recalculation
			for (point=processpoints.begin(), num_hits=0; point!=processpoints.end(); point++) {
				found = true;
				pos1=Position2D(point->X(), point->Y());

				if (select_tag != UndefinedTag) {
					if (point->HasAttribute(select_tag)) {
						if (point->Attribute(select_tag) != select_value) found = false;
					}
					else found = false;
				}
				if (found) {
					if (line.DistanceToPoint(pos1) <= max_dist_to_line && !point->Processed()) {
						num_hits++;
						temppoints.push_back(*point);
						//         line.AddPoint(point->Position2DRef(), false);
					}
				}
			}
			if (num_hits > min_num_hits) {
				// Fit line to all selected points
				//   line.Recalculate();
				pos1=Position2D(temppoints.Mean()[0], temppoints.Mean()[1]);
				pos2=Position2D(temppoints.Mean()[0]+diffx, temppoints.Mean()[1]+diffy);
				line = Line2D(pos1, pos2);
				// Make a segment from every point near the line, store number of points in segment number
				segments.erase(segments.begin(), segments.end());
				for (point=processpoints.begin(), num_hits=0; point!=processpoints.end(); point++) {
					pos1=Position2D(point->X(), point->Y());
					found = true;
					if (select_tag != UndefinedTag) {
						if (point->HasAttribute(select_tag)) {
							if (point->Attribute(select_tag) != select_value) found = false;
						}
						else found = false;
					}
					if (found) {
						if (line.DistanceToPoint(pos1) <= max_dist_to_line && !point->Processed()) {
							num_hits++;
							scalar = line.Scalar(pos1);
							segments.push_back(LineSegment2D(line, scalar, scalar));
							(segments.end()-1)->Number() = 1;
							point->SetFiltered();
							point->SetProcessed();

							perc++;
						}
					}
				}

				// Merge segments
				segments.SortOnStartScalar();
				do {
					changed = false;

					for (int iSeg=0; iSeg<segments.size()-1; iSeg++) {
						segment1 = segments.begin()+iSeg;
						segment2 = segments.begin()+iSeg+1;

						if (segment2->ScalarBegin()-segment1->ScalarEnd()<=max_dist_between_points) {
							changed = true;
							segment1->ScalarEnd() = segment2->ScalarEnd();
							segment1->Number() = segment1->Number()+segment2->Number();
							segments.erase(segment2);
							iSeg--;
						}
					}

					/*for (segment1=segments.begin(); segment1<segments.end()-1; segment1++) {
						segment2 = segment1 + 1;
						if (segment2->ScalarBegin() - segment1->ScalarEnd() <=
							max_dist_between_points) {
								changed = true;
								segment1->ScalarEnd() = segment2->ScalarEnd();
								segment1->Number() = segment1->Number() + segment2->Number();
								segments.erase(segment2);
								segment1--;
						}
					}*/
				} while (changed);

				for (int iSeg=0; iSeg<segments.size(); iSeg++) {
					segment1 = segments.begin()+iSeg;
					if (segment1->Length()<minimum_length) {
						segments.erase(segment1);
						iSeg--;
					}
				}

				/*
				for (segment1=segments.begin(); segment1<segments.end(); segment1++) {
					if (segment1->Length()<minimum_length) {
						segments.erase(segment1);
						segment1--;
					}
				}*/

				// Sort segments after the number of points
				segments.SortOnNumber();
				if (segments.empty()) num_hits = 0;
				else num_hits = segments.begin()->Number();
			}
		} while (num_tries < max_num_tries && num_hits < min_num_hits);

		if (num_hits < min_num_hits) segments.erase(segments.begin(), segments.end());
		allsegments.insert(allsegments.end(), segments.begin(), segments.end());
		processpoints.RemoveFilteredPoints();
		//printf("size of points: %d, segments %d, iter %d\n", processpoints.size(), segments.size(), iter);
	} while (iter<10 && processpoints.size()>min_num_hits);
	return allsegments;
}
