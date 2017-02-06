
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
 Collection of functions for class LineSegments3D

 LineSegments3D::LineSegments3D                         Construct from a polygon
   (const ObjectPoints &, const LineTopology &)
 LineSegments3D::LineSegments3D                         Construct from polygons
   (const ObjectPoints &, const LineTopologies &)
 LineSegments3D &LineSegments3D::operator=              Copy assignment
   (const LineSegments3D &)
 bool LineSegments3D::Collinear(const Line3D &, double, Check on collinearity of
    double) const                                       line with a segment
 void LineSegments3D::PointsWithTopology                Convert to points with
   (ObjectPoints &, LineTopologies &)                 topology

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>
#include "LineSegments3D.h"
#include "Positions3D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
		   	   Construct from polygons
--------------------------------------------------------------------------------
*/

LineSegments3D::LineSegments3D(const ObjectPoints &points,
                               const LineTopology &polygon)
  : std::vector<LineSegment3D>()
{
  const ObjectPoint            *begin_point, *end_point;
  Position3D                   begin_pos, end_pos;
  LineTopology::const_iterator node;

  node        = polygon.begin();
  begin_point = points.GetPoint(node->NumberRef());
  for (node++; node!=polygon.end(); node++) {
    end_point = points.GetPoint(node->NumberRef());
    if (begin_point && end_point) {
      begin_pos.vect() = begin_point->vect();
      end_pos.vect()   = end_point->vect();
      if (polygon.size() == 2) // Use same number and label
        push_back(LineSegment3D(begin_pos, end_pos,
                                polygon.Number(), polygon.Label()));
      else // Initialise number and label with 0
        push_back(LineSegment3D(begin_pos, end_pos));
    }
    begin_point = end_point;
  }
}

LineSegments3D::LineSegments3D(const ObjectPoints &points,
                               const LineTopologies &polygons)
  : std::vector<LineSegment3D>()
{
  LineTopologies::const_iterator polygon;
  LineSegments3D                 polygon_segments;

  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
    polygon_segments = LineSegments3D(points, polygon->LineTopologyReference());
    if (!polygon_segments.empty())
      insert(end(), polygon_segments.begin(), polygon_segments.end());
  }
}

/*
--------------------------------------------------------------------------------
				Copy assignment
--------------------------------------------------------------------------------
*/

LineSegments3D &LineSegments3D::operator=(const LineSegments3D &segments)
{
  // Check for self assignment
  if (this == &segments) return *this;
  if (!empty()) erase(begin(), end());
  if (!segments.empty()) insert(begin(), segments.begin(), segments.end());
  return(*this);
}   

/*
--------------------------------------------------------------------------------
		Convert to 3D object points with topology
--------------------------------------------------------------------------------
*/

void LineSegments3D::PointsWithTopology(ObjectPoints &points,
                                        LineTopologies &top, int append) const
{
  LineSegments3D::const_iterator segment;

  if (!append) {
    if (!points.empty()) points.erase(points.begin(), points.end());
    if (!top.empty()) top.erase(top.begin(), top.end());
  }
  for (segment=begin(); segment!=end(); segment++)
    segment->PointsWithTopology(points, top, 1);
}

/*
--------------------------------------------------------------------------------
		Sort line segments on start scalar
--------------------------------------------------------------------------------
*/
int LargerStartScalarLineSegment3D(const void *ptr1, const void *ptr2)
{
  const LineSegment3D *segment1, *segment2;
  
  segment1 = (LineSegment3D *) ptr1;
  segment2 = (LineSegment3D *) ptr2;
  if (segment1->ScalarBegin() > segment2->ScalarBegin()) return 1;
  if (segment1->ScalarBegin() < segment2->ScalarBegin()) return -1;
  return 0;
}

void LineSegments3D::SortOnStartScalar()
{
  qsort((void *) &*begin(), (int) size(), sizeof(LineSegment3D),
        LargerStartScalarLineSegment3D);
}
/*
--------------------------------------------------------------------------------
		Sort line segments on number
--------------------------------------------------------------------------------
*/
int LargerNumberLineSegment3D(const void *ptr1, const void *ptr2)
{
  const LineSegment3D *segment1, *segment2;
  
  segment1 = (LineSegment3D *) ptr1;
  segment2 = (LineSegment3D *) ptr2;
  if (segment1->Number() > segment2->Number()) return 1;
  if (segment1->Number() < segment2->Number()) return -1;
  return 0;
}

void LineSegments3D::SortOnNumber()
{
  qsort((void *) &*begin(), (int) size(), sizeof(LineSegment3D),
        LargerNumberLineSegment3D);
}
