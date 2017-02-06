
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



#include "Segment.h"
#include "Constants.h"

Position2D Segment::MiddlePoint() const
{
  double xm, ym;	
  xm = (pos1.X() + pos2.X()) / 2;
  ym = (pos1.Y() + pos2.Y()) / 2;
  return Position2D(xm, ym);
}  


bool Segment::PointOnSegment(const Position2D &pt, 
		bool online, double error) const
{
   if (online)
   {
     if (!PointOnLine(pt, error))
      return false;
   }

   double l = Length();  
   double d1 = Distance(pos1, pt); 
   double d2 = Distance(pos2, pt);
   if (Distance(pos1, pt) < l && Distance(pos2, pt) < l)
      return true;
   return false;
} 


Position2D Segment::PointAtDistance(const Position2D &pos, int dist) const
{
  double l1 = Distance(pos1, pos);
  double l2 = Distance(pos2, pos);
  
  Position2D pt1, pt2;
  if (l1 < l2)
  {
      pt1 = pos1;
      pt2 = pos2;
  }
  else
  {  
      pt1 = pos2;
      pt2 = pos1;
  }        
   
  if (sinphi == 0)
  {
     l1 = pt1.Y() - dist - pt2.Y();
     if (l1 < 0) l1 = -l1;
     
     l2 = pt1.Y() + dist - pt2.Y();
     if (l2 < 0) l2 = -l2;
     
     if (l1 < l2)
         return Position2D(pt1.X(), pt1.Y() - dist);
     else    
     	 return Position2D(pt1.X(), pt2.Y() + dist);
  }
  
  double m = - cosphi / sinphi;
  //double n = d / sinphi;
      
  double x1 = sqrt((dist) / ( 1 + m * m)) + pt1.X();
  double y1 = Y(x1);
  double x2 = - sqrt((dist) / ( 1 + m * m)) + pt1.X();
  double y2 = Y(x2);
  
  l1 = Distance(Position2D(x1, y1), pt2);
  l2 = Distance(Position2D(x2, y2), pt2);
  
  if (l1 > l2)
      return Position2D(x1, y1);
  else
      return Position2D(x2, y2);   
}



bool Intersection2Segments(const Segment &s1, const Segment &s2, 
		double err_dist, Position2D &pos, bool onseg)
{
   if (ParallelSegments(s1, s2, err_dist))
        return 0;  
         
   pos.X() = (s1.d * s2.sinphi - s2.d * s1.sinphi) / 
   		(s1.cosphi * s2.sinphi - s1.sinphi * s2.cosphi);
   pos.Y() = s1.Y(pos.X());

   if (!onseg)
     return 1;

   return (s1.PointOnSegment(pos) && s2.PointOnSegment(pos));
}



bool ParallelSegments(const Segment &s1, const Segment &s2, double err_dist)
{
   if (!ParallelLines(s1, s2, ERR_PARALLEL_LINES)) // perpendicular lines
   {
      return false;
   }

   Position2D pr1 = s2.Project(s1.pos1);
   Position2D pr2 = s2.Project(s1.pos2);

   double d1 = s1.DistanceToPointSigned(pr1);
   double d2 = s1.DistanceToPointSigned(pr2);
   return (fabs(d1 - d2) < err_dist);
}


bool CollinearSegments(const Segment &s1, const Segment &s2, double err_dist)
{
   if (!ParallelLines(s1, s2, ERR_PARALLEL_LINES)) // perpendicular lines
   {
      return false;
   }

   Position2D pr1 = s2.Project(s1.pos1);
   Position2D pr2 = s2.Project(s1.pos2);

   double d1 = s1.DistanceToPointSigned(pr1);
   double d2 = s1.DistanceToPointSigned(pr2);

   if (fabs(d1 - d2) > err_dist)	// not parallel
     return false;

   return (fabs(d1) < err_dist && fabs(d2) < err_dist);
}


double UnionCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_union)
{
   double d1 = Distance(s1.EndPoint(0), s2.EndPoint(0));
   double d2 = Distance(s1.EndPoint(0), s2.EndPoint(1));
   double d3 = Distance(s1.EndPoint(1), s2.EndPoint(0));
   double d4 = Distance(s1.EndPoint(1), s2.EndPoint(1));

   double max = d1;
   if (d2 > max) max = d2;
   if (d3 > max) max = d3;
   if (d4 > max) max = d4;

   if (d1 == max)
   {
      seg_union = Segment(s1.EndPoint(0), s2.EndPoint(0));
      return max;
   }
   if (d2 == max)
   {
      seg_union = Segment(s1.EndPoint(0), s2.EndPoint(1));
      return max;
   }
   if (d3 == max)
   {
      seg_union = Segment(s1.EndPoint(1), s2.EndPoint(0));
      return max;
   }
   //if (d4 == max)
   //{
      seg_union = Segment(s1.EndPoint(1), s2.EndPoint(1));
      return max;
   //}
}


bool IntersectionCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_intersect)
{
   double d1 = Distance(s1.pos1, s2.pos1);
   double d2 = Distance(s1.pos2, s2.pos1);
   double len = s1.Length();

   if (d1 < len && d2 < len)	// s2.pos1 on segment s1
   {
      d2 = Distance(s1.pos1, s2.pos2);
      len = s2.Length();
      if (d1 < len && d2 < len)
          seg_intersect = Segment(s2.pos1, s1.pos1);
      else     
          seg_intersect = Segment(s2.pos1, s1.pos2);
      return true;
   }

   d1 = Distance(s1.pos1, s2.pos2);
   d2 = Distance(s1.pos2, s2.pos2);

   if (d1 < len && d2 < len)	// s2.pos2 on segment s1
   {
      d2 = Distance(s1.pos1, s2.pos1);
      len = s2.Length();
      if (d1 < len && d2 < len)
          seg_intersect = Segment(s2.pos2, s1.pos1);
      else     
          seg_intersect = Segment(s2.pos2, s1.pos2);
      return true;
   }

   return false;    
}



bool GapCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_gap)
{
   double d1 = Distance(s1.pos1, s2.pos1);
   double d2 = Distance(s1.pos2, s2.pos1);
   double len = s1.Length();

   if (d1 < len && d2 < len)	// s2.pos1 on segment s1
      return false;    

   double d3 = Distance(s1.pos1, s2.pos2);
   double d4 = Distance(s1.pos2, s2.pos2);

   if (d3 < len && d4 < len)	// s2.pos2 on segment s1
      return false;    

   double d5 = Distance(s2.pos1, s1.pos1);
   double d6 = Distance(s2.pos2, s1.pos1);
   len = s2.Length();

   if (d5 < len && d6 < len)	// s1.pos1 on segment s2
      return false;    

   Position2D pt1, pt2;

   if (d1 < d3)
      pt1 = s2.pos1;
   else
      pt1 = s2.pos2;

   if (d1 < d2)
      pt2 = s1.pos1;
   else
      pt2 = s1.pos2;

   seg_gap = Segment(pt1, pt2);
   return true;
}


bool operator==(const Segment &s1, const Segment &s2)
{
   if (Distance(s1.pos1, s2.pos1) < 0.1 && Distance(s1.pos2, s2.pos2) < 0.1)
     return true;
   return false;
}



bool CollinearPoints(const Position2D &pos1, const Position2D &pos2, 
	const Position2D &pos3, double err_dist)
{
   Positions2D pts;
   pts.push_back(pos1);
   pts.push_back(pos2);
   pts.push_back(pos3);
   Line2D lin(pts);

   return (lin.DistanceToPoint(pos1) < err_dist && 
			lin.DistanceToPoint(pos2) < err_dist && 
			lin.DistanceToPoint(pos3) < err_dist);
}
