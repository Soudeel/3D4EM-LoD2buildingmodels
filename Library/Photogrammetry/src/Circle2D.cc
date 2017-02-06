
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
 Collection of functions for class Circle2D

 Circle2D(const Position2D &, const Position2D &,    Construct from three points
          const Position2D &)
 Circle2D & Circle2D::operator=(const Circle2D &)    Copy assignment
 double Circle2D::SignedDistance(Position2D &)       Distance of point to circle
 void Circle2D::Polygon(int, ObjectPoints &,         Convert to polygon
                        LineTopology &) const

 Initial creation
 Author : George Vosselman
 Date   : 04-01-2004

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
#include "Circle2D.h"
#include "LineSegment2D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
		         Construct from three positions
--------------------------------------------------------------------------------
*/
 
Circle2D::Circle2D(const Position2D &p1, const Position2D &p2,
                   const Position2D &p3, bool &success)
{
  Line2D line1, line2;
  LineSegment2D cord;

  success = false;
  cord = LineSegment2D(p1, p2);
  line1 = Line2D(cord.MiddlePoint(),
                 cord.MiddlePoint() + cord.Direction().Normal());
  cord = LineSegment2D(p1, p3);
  line2 = Line2D(cord.MiddlePoint(),
                 cord.MiddlePoint() + cord.Direction().Normal());
  if (Intersection2Lines(line1, line2, centre)) {
    success = true;
    radius = (p1 - centre).Length();
  }
  num = label = 0;
}

/*
--------------------------------------------------------------------------------
		   Construct from centre position and radius
--------------------------------------------------------------------------------
*/
 
Circle2D::Circle2D(const Position2D &p, double r)
{
  centre = p;
  radius = r;
  num = label = 0;
}

/*
--------------------------------------------------------------------------------
		                Copy assignment
--------------------------------------------------------------------------------
*/
 
Circle2D & Circle2D::operator=(const Circle2D &circle)
{
  // Check for self assignment
  if (this == &circle) return *this;
  num    = circle.num;
  centre = circle.centre;
  radius = circle.radius;
  label  = circle.label;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                    Signed distance of a point to the circle
--------------------------------------------------------------------------------
*/

double Circle2D::SignedDistance(const Position2D &pos) const
{
  double sqdist;

  sqdist = (pos - centre).SqLength();
  if (sqdist > 0.0) return(sqrt(sqdist) - radius);
  else return(-radius);
}

/*
--------------------------------------------------------------------------------
                       Conversion to a polygon
--------------------------------------------------------------------------------
*/

void Circle2D::Polygon(int num_nodes, ObjectPoints &points,
                       LineTopology &polygon) const
{
  double phi, pi = 4.0 * atan(1.0), phi_inc;
  int    i, pnr;

  if (points.size()) pnr = (points.end() - 1)->Number() + 1;
  else pnr = 0;
  if (polygon.size()) polygon.erase(polygon.begin(), polygon.end());
  phi_inc = 2 * pi / num_nodes;
  for (i=0, phi=0.0; i<num_nodes; i++, pnr++, phi+=phi_inc) {
    points.push_back(ObjectPoint(centre.X() + radius * cos(phi),
                                 centre.Y() + radius * sin(phi), 0.0, pnr,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    polygon.push_back(PointNumber(pnr));
  }
  polygon.push_back(*(polygon.begin()));
}
