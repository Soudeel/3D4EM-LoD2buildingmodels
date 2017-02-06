
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
 Collection of functions for class Line2D

--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "Line2D.h"
#include "digphot_arch.h"
#include "Line3D.h"
#include "DataBounds2D.h"

#ifdef linux
# include <assert.h>
#endif


#ifndef PI
#  define PI       4.0 * atan(1.0)
#endif
#define MID_DIST 0.5  // test which side of a line is int or ext. to a polygon
#define EPS1	 10   // max error for testing if a point is on a line
		      // it is used to check if a point is on an 
		      // epipolar line	
		      
#define EPS_PARALLEL_LINES	5 * PI / 180	// degrees      
#define EPS_COLLINEAR_LINES	1		// distance


/*
--------------------------------------------------------------------------------
                      Constructors
--------------------------------------------------------------------------------
*/

// Construct from given valuues
Line2D::Line2D(double sine, double cosine, double distance)
{
  double length = sine * sine + cosine * cosine;
  if (length != 0.0) {
    length = sqrt(length);
    sinphi = sine / length;
    cosphi = cosine / length;
  }
  d = distance;
}

// Construct from two 2D points
Line2D::Line2D(const Position2D &pt1, const Position2D &pt2)
{	
     double l = DistancePosition2DToPosition2D(pt1, pt2);
     sinphi = -(pt2.X() - pt1.X()) / l;
     cosphi = (pt2.Y() - pt1.Y()) / l;

     d = pt1.X() * cosphi + pt1.Y() * sinphi;     
}

// Construct from a set of 2D points
Line2D::Line2D(const Positions2D &pts)
{	
   double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
   Positions2D::const_iterator i;
   assert(pts.size() >= 2);
   for(i = pts.begin(); i < pts.end(); i++)
   {
     sum_x += i->GetX();
     sum_y += i->GetY();
     sum_x2 += i->GetX() * i->GetX();
     sum_xy += i->GetX() * i->GetY();
   }
   
   double tmp = pts.size() * sum_x2 - sum_x * sum_x;
   if (tmp != 0) {
     double b = (sum_y * sum_x2 - sum_x * sum_xy) / tmp; 
     double m = (pts.size() * sum_xy - sum_y * sum_x) / tmp;

     Position2D pt1(0, b);
     Position2D pt2(pts.begin()->X(), m * pts.begin()->X() + b);
     double l = DistancePosition2DToPosition2D(pt1, pt2);
     sinphi = -(pt2.X() - pt1.X()) / l;
     cosphi = (pt2.Y() - pt1.Y()) / l;

     d = pt1.X() * cosphi + pt1.Y() * sinphi;     
   }
   else { // Vertical line
     cosphi = 1.0;
     sinphi = 0.0;
     d      = sum_x / pts.size();
   }
}

// Construct from two 3D positions
Line2D::Line2D(const Position3D &pt1, const Position3D &pt2)
{
  double l = (pt1.vect2D() - pt2.vect2D()).Length();
  sinphi = -(pt2.X() - pt1.X()) / l;
  cosphi = (pt2.Y() - pt1.Y()) / l;
  d = pt1.X() * cosphi + pt1.Y() * sinphi;     
}

// Construct from a 3D line
Line2D::Line2D(const Line3D &line)
{
  *this = Line2D(line.Position(0.0), line.Position(1.0));
}

/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

Line2D& Line2D::operator=(const Line2D& lin)
{
   sinphi = lin.sinphi;
   cosphi = lin.cosphi;
   d = lin.d;       	
   return *this;
}


double Line2D::Y(double x) const
{
   if (sinphi == 0) // lin || Oy
     return d;
        
   return (d - x * cosphi) / sinphi;
}   


void Line2D::FindParallelLine(const Position2D &pos, Line2D &lin) const
{
   lin.sinphi = sinphi;
   lin.cosphi = cosphi;
   lin.d = pos.X() * cosphi + pos.Y() * sinphi;  
}


void Line2D::FindParallelLines(double dist, const Position2D &posm, 
	Line2D &lin1, Position2D &posm1, Line2D &lin2, Position2D &posm2) const
{
  lin1.sinphi = sinphi;
  lin2.sinphi = sinphi;
  lin1.cosphi = cosphi;
  lin2.cosphi = cosphi;
  lin1.d = d - dist;
  lin2.d = d + dist;
  posm1.X() = posm.X() - MID_DIST * cosphi;
  posm1.Y() = posm.Y() - MID_DIST * sinphi;
  posm2.X() = posm.X() + MID_DIST * cosphi;
  posm2.Y() = posm.Y() + MID_DIST * sinphi;  
}	 


Line2D Line2D::PerpendicularLine(const Position2D &pos) const
{
   Line2D lin;
   lin.sinphi = cosphi;
   lin.cosphi = -sinphi;
   lin.d = pos.X() * lin.cosphi + pos.Y() * lin.sinphi;
   return lin;
}   


void Line2D::Print() const
{
   printf("sinphi = %15.6f cosphi = %15.6f  d =%15.6f\n", sinphi, cosphi, d);
}


double Line2D::DistanceToPoint(const Position2D &pos) const
{
   return fabs(pos.X() * cosphi + pos.Y() * sinphi - d);
}
   

double Line2D::DistanceToPointSigned(const Position2D &pos) const
{
   return pos.X() * cosphi + pos.Y() * sinphi - d;
}


bool Line2D::PointOnLine(const Position2D &pos, double err_dist) const
{
   return (DistanceToPoint(pos) < err_dist); 
}  	 			


bool Line2D::PointsAtDistance(const Position2D &pt, double dist, 
        		Position2D &pt1, Position2D &pt2) const
{   
  pt1.X() = pt.X() - dist * sinphi;
  pt2.X() = pt.X() + dist * sinphi;
  pt1.Y() = pt.Y() + dist * cosphi;
  pt2.Y() = pt.Y() - dist * cosphi;
     
  return true;
}        		


double Line2D::AngleOx() const
{
   double angle = atan2(-cosphi, sinphi);    // GV
   if (angle < -PI/2) angle += PI;           // GV
   if (angle > PI/2)  angle -= PI;           // GV
   return angle;
//GV return atan(-cosphi/sinphi);
}


double Angle2Lines(const Line2D &lin1, const Line2D &lin2)
{
//GV double angle = fabs(lin1.AngleOx() - lin2.AngleOx());
   //if (angle > PI) angle = PI - angle;
   //if (angle > PI - angle) angle = PI - angle;
   double angle = acos(lin1.Direction().DotProduct(lin2.Direction()));  // GV
   if (angle > PI / 2) angle = PI - angle;                              // GV
   return angle;
}


bool ParallelLines(const Line2D &lin1, const Line2D &lin2, double err_angle)
{
   return (Angle2Lines(lin1, lin2) < err_angle);
}           	  
   	  

bool CollinearLines(const Line2D &lin1, const Line2D &lin2, 
	double err_angle, double err_dist)
{
   double a;                                                 // GV
   if (!ParallelLines(lin1, lin2, err_angle))
   	  return false;
   if (lin1.Direction().DotProduct(lin2.Direction()) > 0.0)  // GV
     a = fabs(lin1.d - lin2.d);
   else                                                      // GV
     a = fabs(lin1.d + lin2.d);                              // GV
   return (a < err_dist);
}           	  


bool Intersection2Lines(const Line2D &lin1, const Line2D &lin2, Position2D &pos)
{
   if (Angle2Lines(lin1, lin2) < EPS_PARALLEL_LINES) // parallel lines
        return 0;  
         
   pos.X() = (lin1.d * lin2.sinphi - lin2.d * lin1.sinphi) / 
   		(lin1.cosphi * lin2.sinphi - lin1.sinphi * lin2.cosphi);
   pos.Y() = lin1.Y(pos.X());
   return 1;
}


void Intersection2NonParallelLines(const Line2D &lin1, const Line2D &lin2, Position2D &pos)
{         
   pos.X() = (lin1.d * lin2.sinphi - lin2.d * lin1.sinphi) / 
   		(lin1.cosphi * lin2.sinphi - lin1.sinphi * lin2.cosphi);
   pos.Y() = lin1.Y(pos.X());
}



bool operator==(const Line2D &lin1, const Line2D &lin2)
{
   return (lin1.sinphi == lin2.sinphi && lin1.cosphi == lin2.cosphi 
   	&& lin1.d == lin2.d);
}   	

/*
-------------------------------------------------------------------------------
           Functions working with footpoints and directions
-------------------------------------------------------------------------------
*/

Position2D Line2D::FootPoint() const
{
  return(Position2D(d * cosphi, d * sinphi));
}

Vector2D Line2D::Direction() const
{
  return(Vector2D(-sinphi, cosphi));
}

Vector2D Line2D::Normal() const
{
  return(Vector2D(cosphi, sinphi));
}

Position2D Line2D::Project(const Position2D &point) const
{
  Position2D projection;
  double     distance;

  distance = DistanceToPointSigned(point);
  projection.X() = point.X() - distance * cosphi;
  projection.Y() = point.Y() - distance * sinphi;

  return(projection);
}

double Line2D::Scalar(const Position2D &point) const
{
  Position2D projection;

  projection = Project(point);
  if (fabs(Direction().X()) > fabs(Direction().Y()))
    return((projection.X() - FootPoint().X()) / Direction().X());
  else
    return((projection.Y() - FootPoint().Y()) / Direction().Y());
}

Position2D Line2D::Position(double scalar) const
{
  return Position2D(FootPoint().X() + scalar * Direction().X(),
                    FootPoint().Y() + scalar * Direction().Y());
}

void Line2D::SwapNormal()
{
  sinphi = -sinphi;
  cosphi = -cosphi;
  d      = -d;
}

// Construct the bisector of the smallest angle between two lines

Line2D & Bisector(const Line2D &line1, const Line2D &line2)
{
  Position2D intersection;
  Line2D     *bisector = new Line2D(), normal;
  double     dir1, dir2, dir_bisector, dir_tmp;
  
  // Check for parallel lines
  if (line1.cosphi * line2.sinphi - line1.sinphi * line2.cosphi == 0.0) {
    *bisector = line1;
    bisector->d = (line1.d + line2.d) / 2.0;
  }
  else {
    dir1 = line1.AngleOx();
    dir2 = line2.AngleOx();
    if (dir1 < dir2) {
      dir_tmp = dir1;
      dir1    = dir2;
      dir2    = dir_tmp;
    }
    if (dir2 - dir1 >= PI) dir2 -= PI;
    if (dir2 - dir1 >= PI/2.0) dir2 -= PI;
    dir_bisector = (dir1 + dir2) / 2.0;
    normal = Line2D(sin(dir_bisector), cos(dir_bisector), 0.0);
    Intersection2NonParallelLines(line1, line2, intersection);
    *bisector = normal.PerpendicularLine(intersection);
  }
  return *bisector;
}

// Determine the bounding box of the rotated rectangle

DataBounds2D & Line2D::BoundingBox(double scalar_min, double scalar_max,
                                   double max_dist) const
{
  Position2D   end_point, corner1, corner2;
  Line2D       line_perp;
  DataBounds2D *bounds = new DataBounds2D();
  
  end_point = Position(scalar_min);
  line_perp = PerpendicularLine(end_point);
  line_perp.PointsAtDistance(end_point, max_dist, corner1, corner2);
  bounds->Update(corner1);
  bounds->Update(corner2);
  end_point = Position(scalar_max);
  line_perp = PerpendicularLine(end_point);
  line_perp.PointsAtDistance(end_point, max_dist, corner1, corner2);
  bounds->Update(corner1);
  bounds->Update(corner2);
  return *bounds;
}
