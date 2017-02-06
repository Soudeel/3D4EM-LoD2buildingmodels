
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
 Collection of functions for class LineSegments2D

 LineSegments2D::LineSegments2D                         Construct from a polygon
   (const ObjectPoints2D &, const LineTopology &)
 LineSegments2D::LineSegments2D                         Construct from polygons
   (const ObjectPoints2D &, const LineTopologies &)
 LineSegments2D::LineSegments2D                         Construct from a polygon
   (const ObjectPoints &, const LineTopology &)
 LineSegments2D::LineSegments2D                         Construct from polygons
   (const ObjectPoints &, const LineTopologies &)
 LineSegments2D &LineSegments2D::operator=              Copy assignment
   (const LineSegments2D &)
 bool LineSegments2D::Collinear(const Line2D &, double, Check on collinearity of
    double) const                                       line with a segment
 void LineSegments2D::PointsWithTopology                Convert to points with
   (ObjectPoints2D &, LineTopologies &)                 topology
 bool LineSegments2D::IntersectPartition                Intersect a convex
   (const Line2D &, LineSegment2D &, double) const      partition with a line
 bool LineSegments2D::SegmentInPartition                Determine part of
   (const LineSegment2D &, double &, double &) const    segment in partition

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>
#include "LineSegments2D.h"
#include "Positions2D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
		   	   Construct from polygons
--------------------------------------------------------------------------------
*/

LineSegments2D::LineSegments2D(const ObjectPoints2D &points,
                               const LineTopology &polygon)
  : std::vector<LineSegment2D>()
{
  const ObjectPoint2D          *begin_point, *end_point;
  LineTopology::const_iterator node;

  node        = polygon.begin();
  begin_point = points.GetPoint(node->NumberRef());
  for (node++; node!=polygon.end(); node++) {
    end_point = points.GetPoint(node->NumberRef());
    if (begin_point && end_point) {
      if (polygon.size() == 2) // Use same number and label
        push_back(LineSegment2D(begin_point->Position2DRef(),
                                end_point->Position2DRef(),
                                polygon.Number(), polygon.Label()));
      else // Initialise number and label with 0
        push_back(LineSegment2D(begin_point->Position2DRef(),
                                end_point->Position2DRef()));
    }
    begin_point = end_point;
  }
}

LineSegments2D::LineSegments2D(const ObjectPoints2D &points,
                               const LineTopologies &polygons)
  : std::vector<LineSegment2D>()
{
  LineTopologies::const_iterator polygon;
  LineSegments2D                 polygon_segments;

  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
    polygon_segments = LineSegments2D(points, polygon->LineTopologyReference());
    if (!polygon_segments.empty())
      insert(end(), polygon_segments.begin(), polygon_segments.end());
  }
}

/*
LineSegments2D::LineSegments2D(const ObjectPoints &points,
                               const LineTopology &polygon)
  : std::vector<LineSegment2D>()
{
  const ObjectPoint            *begin_point, *end_point;
  Position2D                   begin_pos, end_pos;
  LineTopology::const_iterator node;

  node        = polygon.begin();
  begin_point = points.GetPoint(node->NumberRef());
  for (node++; node!=polygon.end(); node++) {
    end_point = points.GetPoint(node->NumberRef());
    if (begin_point && end_point) {
      begin_pos.vect() = begin_point->vect();
      end_pos.vect()   = end_point->vect();
      if (polygon.size() == 2) // Use same number and label
        push_back(LineSegment2D(begin_pos, end_pos,
                                polygon.Number(), polygon.Label()));
      else // Initialise number and label with 0
        push_back(LineSegment2D(begin_pos, end_pos));
    }
    begin_point = end_point;
  }
}*/
//biao 
LineSegments2D::LineSegments2D(const ObjectPoints &points,
							   const LineTopology &polygon)
							   : std::vector<LineSegment2D>()
{
	const ObjectPoint            *begin_point, *end_point;
	Position2D                   begin_pos, end_pos;
	LineTopology::const_iterator node;
	if (polygon.size()<=1) return;

	node = polygon.begin();
	begin_point = points.GetPoint(node->NumberRef());
	//for (++node; node!=polygon.end(); node++) {
	for (unsigned int i=0; i<polygon.size()-1; i++) {
		begin_point = points.GetPoint(polygon[i].NumberRef());
		end_point = points.GetPoint(polygon[i+1].NumberRef());

		if (begin_point && end_point) {
			begin_pos.vect() = begin_point->vect();
			end_pos.vect()   = end_point->vect();
			if (polygon.size() == 2) // Use same number and label
				push_back(LineSegment2D(begin_pos, end_pos,
				polygon.Number(), polygon.Label()));
			else // Initialise number and label with 0
				push_back(LineSegment2D(begin_pos, end_pos));
		}
	}
}

LineSegments2D::LineSegments2D(const ObjectPoints &points,
                               const LineTopologies &polygons)
  : std::vector<LineSegment2D>()
{
  LineTopologies::const_iterator polygon;
  LineSegments2D                 polygon_segments;

  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
    polygon_segments = LineSegments2D(points, polygon->LineTopologyReference());
    if (!polygon_segments.empty())
      insert(end(), polygon_segments.begin(), polygon_segments.end());
  }
}

/*
--------------------------------------------------------------------------------
				Copy assignment
--------------------------------------------------------------------------------
*/

LineSegments2D &LineSegments2D::operator=(const LineSegments2D &segments)
{
  // Check for self assignment
  if (this == &segments) return *this;
  if (!empty()) erase(begin(), end());
  if (!segments.empty()) insert(begin(), segments.begin(), segments.end());
  return(*this);
}   

/*
--------------------------------------------------------------------------------
		Check on collinearity of line with one of the segments
--------------------------------------------------------------------------------
*/

bool LineSegments2D::Collinear(const Line2D &line, double err_angle,
                               double err_dist) const
{
  LineSegments2D::const_iterator segment;

  for (segment=begin(); segment!=end(); segment++)
    if (segment->Collinear(line, err_angle, err_dist)) return(1);
  return(0);
}

/*
--------------------------------------------------------------------------------
		Convert to 2D object points with topology
--------------------------------------------------------------------------------
*/

void LineSegments2D::PointsWithTopology(ObjectPoints2D &points,
                                        LineTopologies &top, int append) const
{
  LineSegments2D::const_iterator segment;

  if (!append) {
    if (!points.empty()) points.erase(points.begin(), points.end());
    if (!top.empty()) top.erase(top.begin(), top.end());
  }
  for (segment=begin(); segment!=end(); segment++)
    segment->PointsWithTopology(points, top, 1);
}

/*
--------------------------------------------------------------------------------
		Intersect a convex partition with a line
--------------------------------------------------------------------------------
*/

bool LineSegments2D::IntersectPartition(const Line2D &line,
                                        LineSegment2D &intersection,
                                        double err_dist) const
{
  LineSegments2D::const_iterator segment;
  Positions2D                int_points;
  Position2D                 int_point;
  Positions2D::iterator      pos1, pos2;
  double                     dist, max_dist;

/* Collect all intersection positions */

  for (segment=begin(); segment!=end(); segment++)
    if (segment->Intersect(line, int_point, err_dist))
      int_points.push_back(int_point);

/* No intersecting segment if less than two intersection points */

  if (int_points.size() < 2) return(0);

/* Select the two points that are furthest apart */

  max_dist = 0;
  for (pos1=int_points.begin(); pos1!=int_points.end(); pos1++) {
    for (pos2=pos1+1; pos2!=int_points.end(); pos2++) {
      dist = pos1->Distance(pos2->Position2DRef());
      if (dist > max_dist) {
        max_dist = dist;
        intersection = LineSegment2D(pos1->Position2DRef(),
                                     pos2->Position2DRef());
      }
    }
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
	   Determine if a part of the segment is inside the partition
--------------------------------------------------------------------------------
*/

bool LineSegments2D::SegmentInPartition(const LineSegment2D &segment,
                                        double &len_present,
                                        double &len_missing) const
{
  LineSegment2D intersection;
  double        s, s_min, s_max, len_total;

  if (!IntersectPartition(segment.Line2DReference(), intersection, 0.01))
    return(0);

  len_total = intersection.ScalarEnd() - intersection.ScalarBegin();
  s_min = intersection.Scalar(segment.BeginPoint());
  s_max = intersection.Scalar(segment.EndPoint());
  if (s_min > s_max) {s = s_min;  s_min = s_max;  s_max = s;}
  s_min = MAX(s_min, intersection.ScalarBegin());
  s_max = MIN(s_max, intersection.ScalarEnd());
  len_present = s_max - s_min;
  if (len_present <= 0) return(0);
  len_missing = len_total - len_present;
  return(1);
}

void LineSegments2D::AddIfLongestSegment(const LineSegment2D &segment,
                                         double min_angle)
{
  LineSegments2D::iterator old_segment;
  double                   angle, pi=4.0*atan(1.0);
  
  for (old_segment=begin(); old_segment!=end(); old_segment++) {
    angle = Angle2Lines(old_segment->Line2DReference(),
                        segment.Line2DReference());
    if (angle > pi / 4.0) angle = pi / 2 - angle;
    if (angle < min_angle) {
      if (old_segment->Length() < segment.Length())
        *old_segment = segment;
      return;
    } 
  }
  // No similar directions, just add segment
  push_back(segment);
}

int LargerStartScalarLineSegment2D(const void *ptr1, const void *ptr2)
{
	const LineSegment2D *segment1, *segment2;

	segment1 = (LineSegment2D *) ptr1;
	segment2 = (LineSegment2D *) ptr2;
	if (segment1->ScalarBegin() > segment2->ScalarBegin()) return 1;
	if (segment1->ScalarBegin() < segment2->ScalarBegin()) return -1;
	return 0;
}

void LineSegments2D::SortOnStartScalar()
{
	qsort((void *) &*begin(), (int) size(), sizeof(LineSegment2D),
		LargerStartScalarLineSegment2D);
}

/*
--------------------------------------------------------------------------------
Sort line segments on number
--------------------------------------------------------------------------------
*/
int LargerNumberLineSegment2D(const void *ptr1, const void *ptr2)
{
	const LineSegment2D *segment1, *segment2;

	segment1 = (LineSegment2D *) ptr1;
	segment2 = (LineSegment2D *) ptr2;
	if (segment1->Number() > segment2->Number()) return 1;
	if (segment1->Number() < segment2->Number()) return -1;
	return 0;
}

void LineSegments2D::SortOnNumber()
{
	if (this->empty()) return;
	qsort((void *) &*begin(), (int) size(), sizeof(LineSegment2D),
		LargerNumberLineSegment2D);
}
