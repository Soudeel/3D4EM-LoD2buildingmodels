
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
 Collection of functions for class LineSegment2D

 LineSegment2D(Position2D &, Position2D &)       Construct from two positions
 LineSegment2D(Line2D &, double, double)         Construct from line and scalars
 LineSegment2D & LineSegment2D::operator=        Copy assignment
   (const LineSegment2D& segment)
 Position2D LineSegment2D::MiddlePoint() const   Construct the mid point
 double LineSegment2D::Distance                  Distance to position
   (const Position2D &) const  
 friend double DistanceLineSegment2DToLineSegment2D  Distance between two
   (const LineSegment2D &,                           line segments
    const LineSegment2D &)
 friend bool Intersect2LineSegments2D            Intersection of two segments
   (const LineSegment2D &, const LineSegment2D &,
    Position2D &, double)
 bool LineSegment2D::Intersect(const Line2D &,   Intersect segment with line
    Position2D &, double) const
 bool LineSegment2D::Collinear(const Line2D &,   Check collinearity with line
    double, double) const
 void LineSegment2D::PointsWithTopology          Convert to points with topology
    ObjectPoints2D &, LineTopologies &,
    int) const

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>
#include "LineSegment2D.h"
#ifndef PI
#  define PI       4.0 * atan(1.0)
#endif

/*
--------------------------------------------------------------------------------
                           Construct a segment
--------------------------------------------------------------------------------
*/

LineSegment2D::LineSegment2D(const Position2D &pos1, const Position2D &pos2)
  : Line2D(), LineNumber()
{	
  Line2DReference() = Line2D(pos1, pos2);
  scalar_begin = Scalar(pos1);
  scalar_end = Scalar(pos2);
  label = num = 0;
}

LineSegment2D::LineSegment2D(const Position2D &pos1, const Position2D &pos2,
                             int number, int lab)
  : Line2D(), LineNumber()
{	
  Line2DReference() = Line2D(pos1, pos2);
  scalar_begin = Scalar(pos1);
  scalar_end   = Scalar(pos2);
  num          = number;
  label        = lab;
}

LineSegment2D::LineSegment2D(const Line2D &line, double s_begin, double s_end)
  : Line2D(), LineNumber()
{	
  Line2DReference() = line;
  scalar_begin = s_begin;
  scalar_end   = s_end;
  label = num = 0;
}

/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

LineSegment2D & LineSegment2D::operator=(const LineSegment2D& segment)
{
  // Check for self assignment
  if (this == &segment) return *this;
  Line2DReference() = segment.Line2DReference();          	
  scalar_begin = segment.scalar_begin;
  scalar_end   = segment.scalar_end;
  num          = segment.num;
  label        = segment.label;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                            Construct the mid point
--------------------------------------------------------------------------------
*/

Position2D LineSegment2D::MiddlePoint() const
{
  return(Position((scalar_begin + scalar_end) / 2.0));
}

/*
--------------------------------------------------------------------------------
                               Distance functions
--------------------------------------------------------------------------------
*/

double LineSegment2D::Distance(const Position2D &pos) const
{
  double scalar;

  scalar = Scalar(pos);
  if (scalar < scalar_begin)
    return(pos.Distance(Position(scalar_begin)));
  else if (scalar > scalar_end) 
    return(pos.Distance(Position(scalar_end)));
  else
    return(DistanceToPoint(pos));
}

double LineSegment2D::Distance(const LineSegment2D &segment2) const
{
  Position2D intersection;
  double     scalar1, scalar2;
  double temp;
  LineSegment2D segment2_temp;
  
  
  double angle = acos(Direction().DotProduct(segment2.Direction()));
  
  if(angle>PI/2) //in case the parallel is reverse direction parallel
   segment2_temp=LineSegment2D(segment2.EndPoint(), segment2.BeginPoint());
  else 
   segment2_temp=segment2; 
   
  // Intersecting segments
  if (Intersection2Lines(Line2DReference(),
                         segment2_temp.Line2DReference(), intersection)) {
    scalar1 = Scalar(intersection);
    scalar2 = segment2_temp.Scalar(intersection);
    if (scalar1 < scalar_begin)
      return(segment2_temp.Distance(BeginPoint()));
    else if (scalar1 > scalar_end)
      return(segment2_temp.DistanceToPoint(EndPoint()));
    else {
      if (scalar2 < segment2_temp.scalar_begin)
        return(DistanceToPoint(segment2_temp.BeginPoint()));
      else if (scalar2 > segment2_temp.scalar_end)
        return(DistanceToPoint(segment2_temp.EndPoint()));
      else
        return(0.0);
    }
  }

  else { // Parallel segments
  
    if (scalar_begin > segment2_temp.scalar_end)
      return(BeginPoint().Distance(segment2_temp.EndPoint()));
    else if (scalar_end < segment2_temp.scalar_begin)
      return(EndPoint().Distance(segment2_temp.BeginPoint()));
    else
      return(DistanceToPoint(segment2_temp.BeginPoint()));
  }
}

/*
--------------------------------------------------------------------------------
                          Intersection of two segments
--------------------------------------------------------------------------------
*/

bool Intersect2LineSegments2D(const LineSegment2D &segment1,
                              const LineSegment2D &segment2,
                              Position2D &intersection, double max_dist)
{
  if (Intersection2Lines(segment1.Line2DReference(),
                         segment2.Line2DReference(), intersection))
    if (segment1.Distance(segment2) <= max_dist) return(1);
  return(0);
}

/*
--------------------------------------------------------------------------------
                        Intersection of segment with line
--------------------------------------------------------------------------------
*/

bool LineSegment2D::Intersect(const Line2D &line,
                              Position2D &intersection, double max_dist) const
{
  double scalar;

  if (Intersection2Lines(Line2DReference(), line, intersection)) {
    scalar = Scalar(intersection);
    if (scalar >= scalar_begin - max_dist && 
        scalar <= scalar_end + max_dist) return(1);
  }
  return(0);
}

/*
--------------------------------------------------------------------------------
                          Collinearity with line
--------------------------------------------------------------------------------
*/

bool LineSegment2D::Collinear(const Line2D &line, double err_angle,
                              double err_dist) const
{
  if (!ParallelLines(line, Line2DReference(), err_angle)) return(0);
  if (line.DistanceToPoint(BeginPoint()) > err_dist) return(0);
  if (line.DistanceToPoint(EndPoint()) > err_dist) return(0);
  return(1);
}

/*
--------------------------------------------------------------------------------
                      Convert to 2D object points with topology
--------------------------------------------------------------------------------
*/

void LineSegment2D::PointsWithTopology(ObjectPoints2D &points,
                                       LineTopologies &top, int append) const
{
  ObjectPoints2D::iterator point;
  ObjectPoint2D            new_point;
  LineTopologies::iterator line;
  LineTopology             new_line;
  int                      highest_number, number_used;

/* Clear old data */

  if (!append) {
    if (!points.empty()) points.erase(points.begin(), points.end());
    if (!top.empty()) top.erase(top.begin(), top.end());
  }

/* Retrieve highest point number */
  
  highest_number = -1;
  for (point=points.begin(); point!=points.end(); point++)
    if (point->Number() > highest_number) highest_number = point->Number();

/* Add the points */

  new_point.Position2DRef() = BeginPoint();
  new_point.Number()        = highest_number + 1;
  points.push_back(new_point);
  new_point.Position2DRef() = EndPoint();
  new_point.Number()        = highest_number + 2;
  points.push_back(new_point);
  new_line.push_back(PointNumber(highest_number+1));
  new_line.push_back(PointNumber(highest_number+2));

/* Check if the segment number has been used already */

  for (line=top.begin(), number_used=0; line!=top.end() && !number_used; line++)
    if (line->Number() == num) number_used = 1;

/* If the number is not used, use the segment number. Otherwise use the
 * highest line number plus one.
 */

  if (!number_used) new_line.Number() = num;
  else {
    for (line=top.begin(), highest_number=-1; line!=top.end(); line++)
      if (line->Number() > highest_number) highest_number = line->Number();
    new_line.Number() = highest_number + 1;
  }

/* Add the line */

  top.push_back(new_line);
}
