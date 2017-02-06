
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
 Collection of functions for class Segment2D and class Segments2D

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include <stdlib.h>
#include "Segment2D.h"
#include "Constants.h"
#include <string.h>


bool operator<(const Position2D &pt1, const Position2D &pt2);


/*
--------------------------------------------------------------------------------
                      Constructor: A Segment determined by 2 points
--------------------------------------------------------------------------------
*/

Segment2D::Segment2D(const Position2D &pt1, const Position2D &pt2)
{	
  if (pt2.X() < pt1.X())
  {
     pos1 = pt2;
     pos2 = pt1;
  }
  else
  {
     pos1 = pt1;
     pos2 = pt2;
  }
  lin() = Line2D(pos1, pos2);       
}

/*
--------------------------------------------------------------------------------
		Construct by converting a LineTopology structure
--------------------------------------------------------------------------------
*/

Segment2D::Segment2D(const LineTopology &lin_top, const ImagePoints &lin_pts)
{
  ImagePoint *pt1, *pt2;
  
  // to skip const lin_pts
  ImagePoints pts;
  pts.insert(pts.begin(), lin_pts.begin(), lin_pts.end());
  
  pt1 = pts.GetPoint(lin_top[0]);
  pt2 = pts.GetPoint(lin_top[1]);
  *this = Segment2D(pt1->vect(), pt2->vect());
}

/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

Segment2D& Segment2D::operator=(const Segment2D& seg)
{
   if (this == &seg) return *this;  // Check for self assignment
   lin() = seg.lin();          	
   pos1 = seg.pos1;
   pos2 = seg.pos2;
          	
   return *this;
}


/*
--------------------------------------------------------------------------------
			Convert to a LineTopology structure
--------------------------------------------------------------------------------
*/

void Segment2D::ConvertToLineTop(LineTopology &lin_top, ImagePoints &lin_pts) const
{
   Covariance2D covar;
   
   // add first point of the segment
   PointNumber ptnr(lin_pts.size());
   Vector2D v = pos1.vect();
   //lin_pts.push_back(ImagePoint(pos1.vect(), ptnr, covar));   
   lin_pts.push_back(ImagePoint(v, ptnr, covar));   
   lin_top.push_back(ptnr);
   
   // add second point of the segment
   ptnr = PointNumber(lin_pts.size());
   //lin_pts.push_back(ImagePoint(pos2.vect(), ptnr, covar));
   v = pos2.vect();
   lin_pts.push_back(ImagePoint(v, ptnr, covar));   
   lin_top.push_back(ptnr);
}


double Segment2D::Length() const
{
  float dx = pos2.X() - pos1.X();
  dx = dx * dx;
  float dy = pos1.Y() - pos2.Y();
  dy = dy * dy;
  float l = sqrt(dx + dy);
  return (double) l;
}


Position2D Segment2D::MiddlePoint() const
{
  double xm, ym;	
  xm = (pos1.X() + pos2.X()) / 2;
  ym = (pos1.Y() + pos2.Y()) / 2;
  return Position2D(xm, ym);
}  


bool Segment2D::PointOnSegment(const Position2D &pos) const
{
//  if (!PointOnLine(pos))
//  	return 0;

  if ((pos1.X() == pos.X() && pos1.Y() == pos.Y()) ||
    (pos2.X() == pos.X() && pos2.Y() == pos.Y()))
    return true;

  if (pos1.X() < pos.X() && pos.X() < pos2.X())
  {
        if (pos1.Y() < pos2.Y())
        {
          if (pos1.Y() < pos.Y() && pos.Y() < pos2.Y())     
               return 1;
          return 0;     
        }
        else
        {
          if (pos2.Y() < pos.Y() && pos.Y() < pos1.Y())     
               return 1;
          return 0;
        }                 
  }  	
  return 0;	  	
}        


bool Segment2D::PointProjectionOnSegment(const Position2D &pos) const 
{
   double d = DistanceToPoint(pos);
   double d1 = Distance(pos1, pos);
   double d2 = Distance(pos2, pos);
   double p;
   if (d1 > d2)
     p = sqrt(d1 * d1 - d * d);
   else
     p = sqrt(d2 * d2 - d * d);
   d = Distance(pos1, pos2);
   return (d > p);
}     

/*
//???????????????????
bool Segment2D::PointProjection(const Position2D &pos, Position2D &pr_pos) const
{
   Line2D lin(1/m, pos.Y() + pos.X()/m);
   Intersection2Lines(*this, lin, pr_pos);
   return PointOnSegment(pr_pos); 
}   
       	
//???????????????????
void Segment2D::SegmentProjection(const Segment2D &seg, Segment2D &pr_seg) const
{
  Position2D pr_pos1, pr_pos2;
  PointProjection(seg.pos1, pr_pos1);
  PointProjection(seg.pos2, pr_pos2);
  pr_seg = Segment2D(pr_pos1, pr_pos2);
}
*/


double Segment2D::MinDistToEndPoints(const Position2D &pos) const
{
  double l1 = Distance(pos1, pos);
  double l2 = Distance(pos2, pos);
  if (l1 < l2)
      return l1;
  else
      return l2;
}          


Position2D Segment2D::PointAtDistance(const Position2D &pos, int dist) const
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


void Segment2D::Print() const
{
   printf("Pos1: %ld  %ld  Pos2: %ld %ld\n", 
        (long)pos1.X(), (long)pos1.Y(), (long)pos2.X(), (long)pos2.Y());
//   printf("m = %ld, n = %ld\n", (long)m, (long)n);

}


/*
--------------------------------------------------------------------------------
                                Intersection of two segments
--------------------------------------------------------------------------------
*/

bool Intersect2Segments(const Segment2D &seg1, const Segment2D &seg2, 
		Position2D &pos)
{ 
   if (seg1.pos1 == seg2.pos1 || seg1.pos1 == seg2.pos2
   	|| seg1.pos2 == seg2.pos1 || seg1.pos2 == seg2.pos2)
   	return 0;	// ??? why?
   
   if (!Intersection2Lines(seg1, seg2, pos))
   	return 0;	

   if (seg1.PointOnSegment(pos) && seg2.PointOnSegment(pos))
   	return 1;
   
   return 0;	   
}


bool DirectionCollinearSegments(const Segment2D &seg1, 
   		const Segment2D &seg2, const Position2D &common_pos)
{
  Position2D pt1, pt2;
  if (Distance(seg1.pos1, common_pos) > Distance(seg1.pos2, common_pos))
       pt1 = seg1.pos1;
  else pt1 = seg1.pos2;
  if (Distance(seg2.pos1, common_pos) > Distance(seg2.pos2, common_pos))
       pt2 = seg2.pos1;
  else pt2 = seg2.pos2;
  
  double d = Distance(pt1, pt2);
  if (seg1.Length() > seg2.Length())
  { 
     if (d >= seg1.Length())
        return false;
     return true;
  }      
  else
  {   
     if (d >= seg2.Length())
        return false;
     return true;
  }     
}   		


bool DirectionCollinearSegments(const Position2D &pt1, const Segment2D &seg1, 
	const Position2D &pt2, const Segment2D &seg2) 
{
   double max = Distance(seg1.EndPoint(0), seg2.EndPoint(0));    
   double d1 = Distance(seg1.EndPoint(0), seg2.EndPoint(1));              
   if (d1 > max) max = d1;
   d1 = Distance(seg1.EndPoint(1), seg2.EndPoint(0));              
   if (d1 > max) max = d1;
   d1 = Distance(seg1.EndPoint(1), seg2.EndPoint(1));              
   if (d1 > max) max = d1;
   
   Position2D tmp_pt1, tmp_pt2;
   if (Distance(pt1, seg1.EndPoint(0)) < Distance(pt1, seg1.EndPoint(1)))
        tmp_pt1 = seg1.EndPoint(0);
   else 
        tmp_pt1 = seg1.EndPoint(1);
        
   if (Distance(pt2, seg2.EndPoint(0)) < Distance(pt2, seg2.EndPoint(1)))
        tmp_pt2 = seg2.EndPoint(0);
   else 
        tmp_pt2 = seg2.EndPoint(1);
        
   if (max == Distance(tmp_pt1, tmp_pt2))     
        return true;                     
   return false;
}


bool operator==(const Segment2D &seg1, const Segment2D &seg2)
{
   return (seg1.pos1 == seg2.pos1 && seg1.pos2 == seg2.pos2);
}   	


/* Intersection of 2 segments lying on the same line */
bool CommonSegmentPart(const Segment2D &s1, const Segment2D &s2, Segment2D &s)
{
  if (s1.pos1.X() > s2.pos1.X())
     return CommonSegmentPart(s2, s1, s);
     
  if (s1.pos2.X() < s2.pos1.X())
      return 0;
      
  if (s1.pos1.X() == s2.pos1.X())    
  {
     if (s1.pos2.X() < s2.pos2.X())
     	s = s1;
     else 
        s = s2;
     
     return 1;
  }
  
  if (s1.pos2.X() > s2.pos2.X())    
     s = s2;
  else    
  {
    s.pos1 = s2.pos1;
    s.pos2 = s1.pos2;
  }
  
  return 1;
}    
     	 

/*******************************************************************************
--------------------------------------------------------------------------------
                                Class Segments2D                  
--------------------------------------------------------------------------------
*******************************************************************************/

/*
--------------------------------------------------------------------------------
		Construct by converting a LineTopologies structure
--------------------------------------------------------------------------------
*/

Segments2D::Segments2D(const LineTopologies &lin_top, const ImagePoints &lin_pts)
{
/*  LineTopologies::const_iterator i;
    
  for( i = lin_top.begin(); i < lin_top.end(); i++)
  {
    if (i->size() == 2)
    {
       Segment2D s(*i, lin_pts);
       push_back(s);
    }   
  }*/
  
  ImagePoint *pt1, *pt2;
  
  // to skip const lin_pts
  ImagePoints pts;
  pts.insert(pts.begin(), lin_pts.begin(), lin_pts.end());
  
  LineTopologies::const_iterator i;
    
  for( i = lin_top.begin(); i < lin_top.end(); i++)
  {
    if (i->size() == 2)
    {   
      pt1 = pts.GetPoint(*(i->begin()));
      pt2 = pts.GetPoint(*(i->begin() + 1));
      if (pt1 != NULL && pt2 != NULL)
      {
        Segment2D s(pt1->vect(), pt2->vect());
        push_back(s);
      }  
    }
  }        
}


Segments2D::Segments2D(const LineTopology &lin_top, const ImagePoints &lin_pts)
{
  LineTopology::const_iterator i;
  ImagePoints pts;
  pts.insert(pts.begin(), lin_pts.begin(), lin_pts.end());
  ImagePoint *pt1, *pt2;
      
  for(i = lin_top.begin(); i < lin_top.end() - 1; i++)
  {  
     pt1 = pts.GetPoint(*i);
     pt2 = pts.GetPoint(*(i + 1));
     push_back(Segment2D(pt1->vect(), pt2->vect()));
  }
}


/*
--------------------------------------------------------------------------------
			Convert to a LineTopologies structure
--------------------------------------------------------------------------------
*/

void Segments2D::ConvertToLineTops(LineTopologies &lin_top, ImagePoints &lin_pts) const
{
  Segments2D::const_iterator i;
  LineTopology line;
  
  for(i = begin(); i < end(); i++)
  {
    line = LineTopology(lin_top.size());
    i->ConvertToLineTop(line, lin_pts);
    lin_top.push_back(line);
    line.erase(line.begin(), line.end());
  }
}   


/*
--------------------------------------------------------------------------------
				Copy assignment
--------------------------------------------------------------------------------
*/

Segments2D &Segments2D::operator=(const Segments2D &seg)
{
   if (!empty())
         erase(begin(), end());
   insert(begin(), seg.begin(), seg.end());
   return *this; 
}   

/*
--------------------------------------------------------------------------------
				Write into files
--------------------------------------------------------------------------------
*/

void Segments2D::Write(char *name) const
{
  LineTopologies lin_top;
  ImagePoints lin_pts;
  
  ConvertToLineTops(lin_top, lin_pts);
  
  // write topology
  char *name1 = new char[strlen(name) + 5];
  sprintf(name1, "%s.top", name);
  lin_top.Write(name1);
  delete [] name1;  
  
  // write points
  char *name2 = new char[strlen(name) + 5];
  sprintf(name2, "%s.pts", name);
  lin_pts.Write(name2);
  delete [] name2;  
}  

/////??????
/*
void Segments2D::ConnectCollinearSegments(const Segment2D &s, 
			const Segments2D &seg_list)
{
  Segments2D::const_iterator i;
  Segment2D ss;
  for(i = seg_list.begin(); i < seg_list.end(); i++)
  {
    if (CollinearSegments(s, *i, ERR_PAR_SEG, ERR_COL_SEG))   
      if ((s.PointOnSegment(i->EndPoint(0)) && 
            s.PointOnSegment(i->EndPoint(1))) ||
          (s.PointOnSegment(i->EndPoint(0)) &&
             (Distance(s.EndPoint(0), i->EndPoint(1)) < GAP ||
              Distance(s.EndPoint(1), i->EndPoint(1)) < GAP)) ||
          (s.PointOnSegment(i->EndPoint(1)) &&
             (Distance(s.EndPoint(0), i->EndPoint(0)) < GAP ||
              Distance(s.EndPoint(1), i->EndPoint(0)) < GAP)))              
    {
       AddSegment(*i);
    }
  }
}
*/

/// ????
/*
void Segments2D::ConnectCollinearSegments(Segments2D &seg_list)
{
  Segments2D::iterator i;
  Segment2D ss;
  Segment2D s = *seg_list.begin();
  push_back(s);
  
  for(i = seg_list.begin() + 1; i < seg_list.end(); i++)
  {
    if (CollinearSegments(s, *i, ERR_PAR_SEG, ERR_COL_SEG))   
    {
       AddSegment(*i);
       seg_list.erase(i);
       i--;
    }
  }
  seg_list.erase(seg_list.begin());
}
*/

double Segments2D::Length() const
{
   Segments2D::const_iterator i;
   double len = 0;
   
   for(i = begin(); i < end(); i++)
      len = len + i->Length();
   
   return len;
}      


void Segments2D::AddSegment(const Segment2D &s)
{
   if (empty())
   {
      push_back(s);
      return;   
   }
   
   Segments2D::iterator i;
   i = begin(); 
   while(i < end())
   {
     if (s.EndPoint(1) < i->EndPoint(0))
     {
         insert(i, s);
         return;
     }       
     if (s.EndPoint(0) < i->EndPoint(0))
     {         
         if (s.EndPoint(1) < i->EndPoint(1))
         {
            Segment2D ss(s.EndPoint(0), i->EndPoint(1));
            erase(i);            
            insert(i, ss);
            return;
         }
         else
         {
            erase(i);
            i--;
         }          
     }
     if (i->EndPoint(0) < s.EndPoint(0) &&
     	   s.EndPoint(1) < i->EndPoint(1))
     	   return;    
     i++;     
   }
   push_back(s);
}     


         

/*void Segments2D::AddSegment(const Segment2D &s)
{
   if (empty())
   {
      push_back(s);
      return;   
   }
   
   Segments2D::iterator i, j, min_iter;
   double d1, d2, min; 

   for(i = begin(); i < end(); i++)
   {
     d1 = Distance(s.EndPoint(0), i->EndPoint(1));
     if (d1 < GAP)	// insert after
     {
        // check next segment
        if (i < end())
        {
          j = i + 1;
          if (Distance(s.EndPoint(1), j->EndPoint(0)) < GAP)
             *i = Segment2D(i->EndPoint(0), j->EndPoint(1));     
          else
             *i = Segment2D(i->EndPoint(1), s.EndPoint(0));
        }
        else     
          *i = Segment2D(i->EndPoint(1), s.EndPoint(0));
        return;  
     }
     
     d2 = Distance(s.EndPoint(1), i->EndPoint(0));
     if (d2 < GAP)	// insert before
     {
        // check previous segment
        if (i != begin())
        {
          j = i - 1;
          if (Distance(s.EndPoint(0), j->EndPoint(1)) < GAP)
             *i = Segment2D(i->EndPoint(1), j->EndPoint(0));     
          else
             *i = Segment2D(i->EndPoint(0), s.EndPoint(1));
        }
        else     
          *i = Segment2D(i->EndPoint(0), s.EndPoint(1));
        return;  
     }     
     
     if (d1 < d2 && d1 < min) 
     {
             min = d1;
             min_iter = i;
     }   
     if (d2 < d1 && d2 < min) 
     {
             min = d2;
             min_iter = i - 1;
     }
   }  
   
   // insert after min_iter
   if (min_iter < end())
   {
      min_iter++;
      insert(min_iter, s);
   }
   else 
      push_back(s);         
}*/          
      

/// ???????
/*       			 
bool Segments2D::TestCollinearSegments(const Segment2D &s1, const Segment2D &s2, 
	double err_par, double err_col)
{
   Segments2D::const_iterator i;
   
   for(i = begin(); i < end(); i++)
   {
      if (!CollinearSegments(*i, s1, err_par, err_col))
        if (!CollinearSegments(*i, s2, err_par, err_col))
        return false;
   }
   
   return true;
}      
*/

/********************************************************************************/

// compare two collinear points
bool operator<(const Position2D &pt1, const Position2D &pt2)
{
   if (pt1.X() <= pt2.X())
      return true;
   return false;
}     
