
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
 Collection of functions for class LineSegment3D

 LineSegment3D(Position3D &, Position3D &)       Construct from two positions
 LineSegment3D(Line3D &, double, double)         Construct from line and scalars
 LineSegment3D & LineSegment3D::operator=        Copy assignment
   (const LineSegment3D&)
 Position3D LineSegment3D::MiddlePoint() const   Construct the mid point
 double LineSegment3D::Distance                  Distance to position
   (const Position3D &) const  
 void LineSegment3D::PointsWithTopology          Convert to points with topology
    ObjectPoints &, LineTopologies &,
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
#include "LineSegment3D.h"


/*
--------------------------------------------------------------------------------
                           Construct a segment
--------------------------------------------------------------------------------
*/

LineSegment3D::LineSegment3D(const Position3D &pos1, const Position3D &pos2)
  : Line3D(), LineNumber()
{	
  Line3DReference() = Line3D(pos1, pos2);
  scalar_begin = Scalar(pos1);
  scalar_end   = Scalar(pos2);
}

LineSegment3D::LineSegment3D(const Position3D &pos1, const Position3D &pos2,
                             int number, int lab)
  : Line3D(), LineNumber()
{	
  Line3DReference() = Line3D(pos1, pos2);
  scalar_begin = Scalar(pos1);
  scalar_end   = Scalar(pos2);
  num          = number;
  label        = lab;
}

LineSegment3D::LineSegment3D(const Line3D &line, double s_begin, double s_end)
  : Line3D(), LineNumber()
{	
  InitialisePointers();
  Line3DReference() = line;
  scalar_begin = s_begin;
  scalar_end   = s_end;
  num = label = 0;
}

LineSegment3D::LineSegment3D(const Line3D &line, double s_begin, double s_end,
                             int number, int lab)
  : Line3D(), LineNumber()
{	
  Line3DReference() = line;
  scalar_begin = s_begin;
  scalar_end   = s_end;
  num          = number;
  label        = lab;
}
/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

LineSegment3D & LineSegment3D::operator=(const LineSegment3D& segment)
{
  // Check for self assignment
  if (this == &segment) return *this;
  Line3DReference() = segment.Line3DReference();          	
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

Position3D LineSegment3D::MiddlePoint() const
{
  return(Position((scalar_begin + scalar_end) / 2.0));
}

/*
--------------------------------------------------------------------------------
                               Distance functions
--------------------------------------------------------------------------------
*/

double LineSegment3D::Distance(const Position3D &pos) const
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

/*
--------------------------------------------------------------------------------
                      Convert to 3D object points with topology
--------------------------------------------------------------------------------
*/

void LineSegment3D::PointsWithTopology(ObjectPoints &points,
                                       LineTopologies &top, int append) const
{
  ObjectPoints::iterator   point;
  ObjectPoint              new_point;
  LineTopologies::iterator line;
  LineTopology             new_line;
  int                      highest_number;

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

  new_point.Position3DRef() = BeginPoint();
  new_point.Number()        = highest_number + 1;
  points.push_back(new_point);
  new_point.Position3DRef() = EndPoint();
  new_point.Number()        = highest_number + 2;
  points.push_back(new_point);
  new_line.push_back(PointNumber(highest_number+1));
  new_line.push_back(PointNumber(highest_number+2));

/* Retrieve the highest line number */

  highest_number = -1;
  for (line=top.begin(); line!=top.end(); line++)
    if (line->Number() > highest_number) highest_number = line->Number();

/* Add the line */

  new_line.Number() = highest_number + 1;
  top.push_back(new_line);
}

/*
--------------------------------------------------------------------------------
                    Reverse the line segment direction
--------------------------------------------------------------------------------
*/
void LineSegment3D::ReverseDirection()
{
  Position3D pos1, pos2;
  
  pos1 = EndPoint();
  pos2 = BeginPoint();
  Line3DReference() = Line3D(pos1, pos2);
  scalar_begin = Scalar(pos1);
  scalar_end   = Scalar(pos2);
}
