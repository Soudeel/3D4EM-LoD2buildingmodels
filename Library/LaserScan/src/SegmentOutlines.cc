
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
#include "LaserPoints.h"
#include "PointNumberLists.h"
#include "Adjustment.h"
#include "Rotation2D.h"
#include "LineSegments2D.h"
#include "stdmath.h"
#include "VRML_io.h"



#include "digphot_arch.h"
#ifdef windows
#define HUGE 1e30
#endif

using namespace std;

/*
--------------------------------------------------------------------------------
                          Derive the enclosing polygon
--------------------------------------------------------------------------------
*/

bool LaserPoints::EnclosingPolygon(ObjectPoints &points, 
                                   LineTopology &enclosing_polygon,
                                   const SegmentationParameters &segmentation_pars,
                                   const OutliningParameters &outlining_pars,
                                   bool use_sloped_segments)
{
  TINEdges                    edges;
  LineTopology                laser_contour, polygon;
  LineTopologies              segment_contours, contour_segments, contour_edges,
                              polygons;
  LineTopologies::iterator    segment_contour, segment_contour2, contour_segment,
                              contour_segment2;
  PointNumberList             component, unused_points;
  LineTopology::iterator      node, node2;
  LaserPoints::iterator       laser_point, laser_point2;
  LaserPoint                  check_point;
  int                         next_number,i ,number, num_pts, label;
  ObjectPoint                 point;
  ObjectPoints::iterator      corner_point, corner_point2;
  vector <int>                segment_numbers;
  vector <int>::iterator      segment_number;
  PointNumberLists            segments;
  PointNumberLists::iterator  segment, segment2;
  Planes                      planes;
  Planes::iterator            plane, plane2;
  Position3D                  pos1, pos2;
  Position2D                  pos2D;
  LineSegments2D              directions;
  LineSegments2D::iterator    dir;
  LineSegment2D               direction, edge, edge2;
  double                      slope, *hough_dirs, max_dist, scalar, scalar_min,
                              scalar_max, min_dist, dist, scalar2, max_dist_hough;
  HoughSpace                  houghspace;
  Line2D                      line, outer_edge, bisector;
  bool                        done, done_constrained, has_segments;
  double                      degree = atan(1.0) / 45.0;
  FILE           *fp1,*fp2;
  LineTopologies *tops;
  ObjectPoints objpts;
// Preparation for enclosing polygon determination
  // Check for presence of TIN and data bounds
  if (tin == NULL) return false;
  if (!DataBounds().XYBoundsSet()) return false;

  objpts=ConstructObjectPoints () ;
 
  // Create the TIN edges
  edges.Derive(TINReference());
    
  tops=edges.EdgeTopologies () ;
  
  
  // Remove long edges
  RemoveLongEdges(edges, segmentation_pars.MaxDistanceInComponent(), true);
  
  tops=edges.EdgeTopologies () ;
 
  
  // Derive the contour
  for (int i=0; i<size(); i++) component.push_back(PointNumber(i));
  laser_contour = DeriveContour(1, component, 1.5, 1.0, edges);
  if (!laser_contour.IsClosed())
    laser_contour.push_back(*(laser_contour.begin()));

   tops->clear();
   tops->push_back(laser_contour);

  // Store points and topology
  if (points.empty()) next_number = 0;
  else next_number = (points.end() - 1)->Number() + 1;
  if (!polygon.empty()) polygon.erase(polygon.begin(), polygon.end());
  point.Z() = 0.0;
  
  // Segment the point cloud if no segmentation numbers are available and
  // sloped segments should be used for inferring preferred directions
  has_segments = HasAttribute(SegmentNumberTag);
  if (!has_segments && use_sloped_segments) SurfaceGrowing(segmentation_pars);

// Intersect adjacent segments and keep intersection lines if they are
// supported by the points of both segments

  if (use_sloped_segments) {
    // First determine which segment numbers are there
    segment_numbers = AttributeValues(SegmentNumberTag);
    // Determine the points of all segments
    for (segment_number=segment_numbers.begin();
         segment_number!=segment_numbers.end(); segment_number++) {
      segments.push_back(TaggedPointNumberList(SegmentNumberTag, *segment_number));
    }  
    // Determine the segment contours
    for (segment=segments.begin(), segment_number=segment_numbers.begin();
         segment!=segments.end(); segment++, segment_number++) {
      segment_contours.push_back(DeriveContour(*segment_number, *segment,
                                               edges, true, SegmentNumberTag));
    }

    // Fit planes to the segments
    for (segment=segments.begin(), segment_number=segment_numbers.begin();
         segment!=segments.end(); segment++, segment_number++) {
      planes.push_back(FitPlane(*segment, *segment_number));
    }
    // Intersect segments
    for (segment_contour=segment_contours.begin(), plane=planes.begin(),
         segment=segments.begin();
         segment_contour!=segment_contours.end();
         segment_contour++, plane++, segment++) {
      for (segment_contour2=segment_contour+1, plane2=plane+1, segment2=segment+1;
           segment_contour2!=segment_contours.end();
           segment_contour2++, plane2++, segment2++) {
        if (IntersectFaces(*segment_contour, *segment_contour2, *plane, *plane2,
                           outlining_pars.MaximumDistancePointIntersectionLine(),
                           pos1, pos2)) {
          direction = LineSegment2D(pos1.Position2DOnly(), pos2.Position2DOnly(),
                                    plane->Number(), plane2->Number());
          direction.ScalarBegin() = 0.0;
          direction.ScalarEnd() = 2.0 * min(segment->size(), segment2->size());
          directions.AddIfLongestSegment(direction,
                              outlining_pars.MinimumAnglePreferredDirections());
        }
      }
    }   
  
    // Derive proferred directions from the slopes of the detected planes,
    // but prefer directions of intersection lines above directions derived
    // from slopes
    pos1 = Position3D(0.0, 0.0, 0.0);
    pos2.Z() = 0.0;
    point.Z() = 0.0;
    for (plane=planes.begin(), segment=segments.begin();
         plane!=planes.end(); plane++, segment++) {
      pos2.X() = plane->Normal().X();
      pos2.Y() = plane->Normal().Y();
      slope =  pos2.Length() / fabs(plane->Normal().Z());
      if (slope > 0.2) {
        pos2.vect() *= ((double) segment->size()) / ((double) size());
        direction = LineSegment2D(pos1.Position2DOnly(), pos2.Position2DOnly(),
                                  plane->Number(), plane2->Number());
        direction.ScalarBegin() = 0.0;
        direction.ScalarEnd()   = (double) segment->size();
        directions.AddIfLongestSegment(direction, 
                                outlining_pars.MinimumAnglePreferredDirections());
      }
    }
  }
  
  // Determine sequences of points on linear segments
  for (node=laser_contour.begin(); node!=laser_contour.end()-1; node++) {
    laser_point = begin() + node->Number();
    laser_point->RemoveAttribute(LabelTag);
  }
  max_dist_hough = sqrt(bounds.XRange() * bounds.XRange() + 
                        bounds.YRange() * bounds.YRange()) / 2.0;
  done = done_constrained = false;
  label = 1;
  do {
    // In the first loop, perform the constrained Hough transform using
    // the directions infered from intersections and slopes
    if (directions.size() && !done_constrained) {
      done_constrained = true;
      // Put directions and perpendicular directions in a double array
      hough_dirs = (double *) malloc(2 * directions.size() * sizeof(double));
      for (dir=directions.begin(), i=0; dir!=directions.end(); dir++, i+=2) {
        hough_dirs[i] = dir->AngleOx();
        hough_dirs[i+1] = hough_dirs[i] + 2 * atan(1.0); // + 90 degrees
      }
      // Initialise Hough space
      houghspace.Initialise(2 * directions.size(), hough_dirs,
                            (int) (2.0 * max_dist_hough / 
                            outlining_pars.HoughBinSizeDistance()),
                            -max_dist_hough, max_dist_hough);
    }
    // In the second loop, or if there are no preferred directions
    // use an unconstrained Hough transform. After this loop, we're done
    else {
      done = true;
      // Initialise Hough space
      houghspace.Initialise((int) (180 * degree /
                                   outlining_pars.HoughBinSizeDirection()),
                            (int) (2 * max_dist_hough /
                                   outlining_pars.HoughBinSizeDistance()),
                            0.0, -max_dist_hough,
                            180 * degree, max_dist_hough);
    }
    // Set offsets
    houghspace.SetOffsets(bounds.MidPoint().X(), bounds.MidPoint().Y(), 0.0);
    // Put the contour points into the Hough space
    num_pts = 0;
    for (node=laser_contour.begin(); node!=laser_contour.end()-1; node++) {
      laser_point = begin() + node->Number();
      if (!laser_point->HasAttribute(LabelTag)) {
        num_pts++;
        houghspace.AddPoint(laser_point->Position2DOnly());
      }
    }
    // Mark the points of the best lines
    line = houghspace.BestLine(&num_pts, 0, 1);
    
 
    
    while (num_pts >= outlining_pars.MinimumNumberOfPointsInOutlineSegment()) {
      // Label the points on the next line of the Hough space
      for (node=laser_contour.begin(); node!=laser_contour.end()-1; node++) {
        laser_point = begin() + node->Number();

        if (line.DistanceToPoint(laser_point->Position2DOnly()) < 
            outlining_pars.MaximumDistancePointOutline() &&
            !laser_point->HasAttribute(LabelTag)) {
          laser_point->Label(label);
          // Remove new labeled points from the Hough space
          houghspace.RemovePoint(laser_point->Position2DOnly());
        }
      }
      // Extract the connected point sequences with of this line, allowing
      // gaps of a certain number of points and a certain size
      contour_segments = TaggedSequences(laser_contour, label, LabelTag, 
                               outlining_pars.MaximumPointGapInOutlineSegment(),
                               outlining_pars.MaximumGapSizeInOutlineSegment());
      
      // Select segments larger than a minimum size
      for (contour_segment=contour_segments.begin();
           contour_segment!=contour_segments.end(); contour_segment++) {
        if (contour_segment->size() >= 
            outlining_pars.MinimumNumberOfPointsInOutlineSegment()) {
          // If the first edge of a contour segment makes a large angle
          // with the line, remove that edge and put the point back into the
          // Hough space. From what is left also put the point back into the
          // Hough space.
          laser_point = begin() + contour_segment->begin()->Number();
          laser_point2 = begin() + (contour_segment->begin()+1)->Number();
          outer_edge = Line2D(laser_point->Position2DOnly(), 
                              laser_point2->Position2DOnly());
          houghspace.AddPoint(laser_point->Position2DOnly());
          laser_point->RemoveAttribute(LabelTag);
          if (Angle2Lines(line, outer_edge) > atan(1.0)) {
            contour_segment->erase(contour_segment->begin());
            houghspace.AddPoint(laser_point2->Position2DOnly());
            laser_point2->RemoveAttribute(LabelTag);
          }
          // Do the same for the last edge
          laser_point = begin() + (contour_segment->end()-1)->Number();
          laser_point2 = begin() + (contour_segment->end()-2)->Number();
          outer_edge = Line2D(laser_point->Position2DOnly(), 
                              laser_point2->Position2DOnly());
          houghspace.AddPoint(laser_point->Position2DOnly());
          laser_point->RemoveAttribute(LabelTag);
          if (Angle2Lines(line, outer_edge) > atan(1.0)) {
            contour_segment->erase(contour_segment->end()-1);
            houghspace.AddPoint(laser_point2->Position2DOnly());
            laser_point2->RemoveAttribute(LabelTag);
          }
          // Create a line segment
          laser_point = begin() + contour_segment->begin()->Number();
          pos2D = laser_point->Position2DOnly();
          scalar_min = scalar_max = line.Scalar(pos2D);
          min_dist = max_dist = line.DistanceToPointSigned(pos2D);
          for (node=contour_segment->begin()+1;
               node!=contour_segment->end(); node++) {
            laser_point = begin() + node->Number();
            pos2D = laser_point->Position2DOnly();
            scalar = line.Scalar(pos2D);
            if (scalar > scalar_max) scalar_max = scalar;
            else if (scalar < scalar_min) scalar_min = scalar;
            dist = line.DistanceToPointSigned(pos2D);
            if (dist > max_dist) max_dist = dist;
            else if (dist < min_dist) min_dist = dist;
          }
          // Correct the line distance to origin
          line.SetDistanceToOrigin(line.DistanceToOrigin() + max_dist);
          // Get the mid point of the line segment
          pos2D = line.Position((scalar_min + scalar_max) / 2.0);
          check_point.X() = pos2D.X();
          check_point.Y() = pos2D.Y();
          // Correct the distance if the wrong side was taken and flip the line
          if (check_point.InsidePolygon(*this, laser_contour)) {
            line.SetDistanceToOrigin(line.DistanceToOrigin() - 
                                     max_dist + min_dist);
            line.SwapNormal();
            scalar     = scalar_min;
            scalar_min = -scalar_max;
            scalar_max = -scalar;
            // Use point number as polygon number for sorting later-on
            polygon.Number() = laser_contour.FindPoint(*(contour_segment->end()-2));
          }
          else {
            // Use point number as polygon number for sorting later-on
            polygon.Number() = laser_contour.FindPoint(*(contour_segment->begin()+1));
          }
          // Get the end points of the segment
          pos2D = line.Position(scalar_min);
          point.X() = pos2D.X();
          point.Y() = pos2D.Y();
          point.Number() = next_number;
          points.push_back(point);
          polygon.Erase();
          polygon.push_back(PointNumber(next_number));
          next_number++;
          pos2D = line.Position(scalar_max);
          point.X() = pos2D.X();
          point.Y() = pos2D.Y();
          point.Number() = next_number;
          points.push_back(point);
          polygon.push_back(PointNumber(next_number));
          next_number++;
          polygons.push_back(polygon);
          contour_edges.push_back(polygon);
        }
      }
      // Extract the next line from the Hough space      
      label++;
      line = houghspace.BestLine(&num_pts, 0, 1);
    }
  } while (!done);
  
  
  // Check that we have at least three contour edges
  if (contour_edges.size() < 3) return false;
  
 
  // Put the contour segments in the right order by sorting them on the line number
  contour_edges.Sort();
  
  // Close gaps by intersecting contour segments and constructing perpendicular
  // connections
  enclosing_polygon.Erase();
  contour_segment = contour_edges.end() - 1;
  for (contour_segment2=contour_edges.begin();
       contour_segment2!=contour_edges.end();
       contour_segment=contour_segment2, contour_segment2++) {
    edge  = *(LineSegments2D(points, *contour_segment).begin());
    edge2 = *(LineSegments2D(points, *contour_segment2).begin());
    // Intersect lines if angle is larger than 45 degrees
    if (Angle2Lines(edge.Line2DReference(), edge2.Line2DReference()) >
        atan(1.0)) {
      Intersection2Lines(edge.Line2DReference(), edge2.Line2DReference(), pos2D);
      // Reset one end point to the new corner point
      corner_point = points.PointIterator(*(contour_segment->end()-1));
      corner_point->X() = pos2D.X();
      corner_point->Y() = pos2D.Y();
      enclosing_polygon.push_back(*(contour_segment->end()-1));
      // Store the point number of the other point for deletion later on
      unused_points.push_back(*(contour_segment2->begin()));
    }
    // Otherwise, construct a perpendicular connection line
    else {
      // Construct bisector
      bisector = Bisector(edge.Line2DReference(), edge2.Line2DReference());
      // Get scalars of projections of both points on bisector
      corner_point = points.PointIterator(*(contour_segment->end()-1));
      scalar = bisector.Scalar(corner_point->Position2DOnly());
      corner_point2 = points.PointIterator(*(contour_segment2->begin()));
      scalar2 = bisector.Scalar(corner_point2->Position2DOnly());
      // Get midpoint projection on bisector
      pos2D = bisector.Position((scalar+scalar2)/2.0);
      // Construct perpendicular line through this point
      line = bisector.PerpendicularLine(pos2D);
      // Intersect this line with the two segment edges to construct new
      // corner points
      Intersection2Lines(edge.Line2DReference(), line, pos2D);
      corner_point->X() = pos2D.X();
      corner_point->Y() = pos2D.Y();
      enclosing_polygon.push_back(corner_point->NumberRef());
      Intersection2Lines(edge2.Line2DReference(), line, pos2D);
      corner_point2->X() = pos2D.X();
      corner_point2->Y() = pos2D.Y();
      enclosing_polygon.push_back(corner_point2->NumberRef());
    }
  }
  // Close the polygon
  enclosing_polygon.push_back(*(enclosing_polygon.begin()));
  // Clear all old polygons
  polygons.Erase();
  // Delete unused points
  for (node=unused_points.begin(); node!=unused_points.end(); node++) {
    corner_point = points.PointIterator(*node);
    points.erase(corner_point);
  }
  
  return true;
}

/*
--------------------------------------------------------------------------------
                        Find the minimun enclosing rectangle
--------------------------------------------------------------------------------
*/
  
bool LaserPoints::EnclosingRectangle(double max_edge_dist, ObjectPoints &points, 
                                     LineTopology &polygon) const
{
  TINEdges               edges;
  LineTopology           laser_contour;
  PointNumberList        component, convex_corners;
  LineTopology::iterator node, node1, node2, prev_node, next_node, convex_node;
  LaserPoints::const_iterator laser_point, laser_point1, laser_point2;
  int                    next_number;
  Line2D                 line, bb_lines[4], perp_line;
  double                 dist, dist_min1, dist_max1,
			             dist_min2, dist_max2, area, best_area;
  Position2D             corners[4];
  ObjectPoint            point;
  ObjectPoints::iterator corner_point;
  int                    i;

// Preparation for minimum enclosing rectangle determination

  // Check for presence of TIN and data bounds
  if (tin == NULL) return false;
  if (!DataBounds().XYBoundsSet()) return false;
  
  // Create the TIN edges
  edges.Derive(TINReference());

  // Derive the convex hull
  for (int i=0; i<size(); i++) component.push_back(PointNumber(i));
  laser_contour = DeriveContour(1, component, edges, false);
  if (!laser_contour.IsClosed())
    laser_contour.push_back(*(laser_contour.begin()));

// Determine minimum enclosing rectangle

  // Determine the convex hull edge belonging to the minimum enclosing rectangle
  best_area = 1.0e30;
  node1 = laser_contour.begin();
  laser_point1 = begin() + node1->Number();
  for (node1++; node1!=laser_contour.end(); node1++) {
    // Construct a line for this convex hull edge
    laser_point2 = laser_point1;
    laser_point1 = begin() + node1->Number();
    line = Line2D(laser_point1->Position2DOnly(),
		  laser_point2->Position2DOnly());
    // Determine the sizes of the bounding rectangle
    for (node=laser_contour.begin(); node!=laser_contour.end()-1; node++) {
      laser_point = begin() + node->Number();
      dist = line.DistanceToPointSigned(laser_point->Position2DOnly());
      if (node == laser_contour.begin()) {
        dist_min1 = dist_max1 = dist;
      }
      else {
        if (dist < dist_min1) dist_min1 = dist;
        if (dist > dist_max1) dist_max1 = dist;
      }
    }
    perp_line = line.PerpendicularLine(laser_point->Position2DOnly());
    dist_min2 = dist_max2 = 0.0;
    for (node=laser_contour.begin(); node!=laser_contour.end()-1; node++) {
      laser_point = begin() + node->Number();
      dist = perp_line.DistanceToPointSigned(laser_point->Position2DOnly());
      if (dist < dist_min2) dist_min2 = dist;
      if (dist > dist_max2) dist_max2 = dist;
    }
    // Derive the bounding lines if this is the best area so far
    area = (dist_max1 - dist_min1) * (dist_max2 - dist_min2);
    if (area < best_area) {
      best_area = area;
      bb_lines[0] = line; 
      bb_lines[0].SetDistanceToOrigin(line.DistanceToOrigin() + dist_min1);
      bb_lines[1] = line;
      bb_lines[1].SetDistanceToOrigin(line.DistanceToOrigin() + dist_max1);
      bb_lines[2] = perp_line; 
      bb_lines[2].SetDistanceToOrigin(perp_line.DistanceToOrigin() + dist_min2);
      bb_lines[3] = perp_line;
      bb_lines[3].SetDistanceToOrigin(perp_line.DistanceToOrigin() + dist_max2);
    }
  }
    
  // Determine the corners of the bounding box
  Intersection2Lines(bb_lines[0], bb_lines[2], corners[0]);
  Intersection2Lines(bb_lines[0], bb_lines[3], corners[1]);
  Intersection2Lines(bb_lines[1], bb_lines[3], corners[2]);
  Intersection2Lines(bb_lines[1], bb_lines[2], corners[3]);

  // Store points and topology
  if (points.empty()) next_number = 0;
  else next_number = (points.end() - 1)->Number() + 1;
  if (!polygon.empty()) polygon.erase(polygon.begin(), polygon.end());
  for (int i=0; i<4; i++) {
    point.X() = corners[i].X();
    point.Y() = corners[i].Y();
    point.Number() = next_number + i;
    points.push_back(point);
    polygon.push_back(PointNumber(next_number + i));
  }
  polygon.push_back(PointNumber(next_number)); // Close polygon
  polygon.MakeCounterClockWise(points);
  
  return true;
}

/*
--------------------------------------------------------------------------------
                        Find the smallest enclosing circle
--------------------------------------------------------------------------------
*/

Circle2D LaserPoints::EnclosingCircle() const
{
  LaserPoints::const_iterator p1, p2, p3, ptest;
  Circle2D                    circle, best_circle;
  double                      min_radius, dist, max_dist;
  bool                        success;

  for (p1=begin(), max_dist=0.0; p1!=end(); p1++) {
    for (p2=p1+1; p2!=end(); p2++) {
      dist = (*p2 - *p1).Length();
      if (dist > max_dist) max_dist = dist;
    }
  }
  min_radius = max_dist / 2.00001;

  best_circle = Circle2D(Position2D(0.0, 0.0), 1e10);
  for (p1=begin(); p1!=end(); p1++) {
    for (p2=p1+1; p2!=end(); p2++) {
      for (p3=p2+1; p3!=end(); p3++) {
        circle = Circle2D(Position2D(p1->vect2D()), Position2D(p2->vect2D()),
                          Position2D(p3->vect2D()), success);
        if (success && circle.Radius() > min_radius &&
            circle.Radius() < best_circle.Radius()) {
          for (ptest=begin(); ptest!=end() && success; ptest++) {
            if (ptest == p1 || ptest == p2 || ptest == p3) continue;
            if (!circle.Inside(Position2D(ptest->vect2D()))) success = false;
          }
          if (success) best_circle = circle;
        }
      }
    }
  }

  return(best_circle);
}
