
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
 Collection of functions for class House

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#ifdef __sgi
  #include <algo.h>
  #include <function.h>
#else
  #include <algorithm>
  #include <functional>
#endif
#include "House.h"
#include "HouseGraph.h"
#include "Constants.h"

// #define __DEBUG
#define DIST	1

#define min(a, b) 	(a < b) ? a : b

//static int no_new_corner = 0;


const ObjectPoint2D &ClosestPerpendicularPoint(const Segment2D &s, 
		const ObjectPoint2D &pos, const ObjectPoints2D &plan_pts);

// Binary predicate object type
class SameObjectPoints2D : public unary_function<ObjectPoint2D, bool>{
  PointNumber ptno;
  
public:
    explicit SameObjectPoints2D(const PointNumber &n)
       : ptno(n) {};
       
    bool operator()(const ObjectPoint2D &pos)
      {
         return pos.Number() == ptno.Number(); 
      }
};         


bool Member(std::vector<int> list, int n);

/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

House::House(int n, const LineTopology &lin, const ObjectPoints2D &pts)
	: FeatureNumber(n)
{
   LineTopology::const_iterator i;
   ObjectPoint2D *pt;
   for(i = lin.begin(); i < lin.end(); i++)
   {
     pt = pts.GetPoint(*i);
     assert(pt != NULL);
     push_back(*pt);
   }

   // if house close: start point == end point;
   if (begin()->vect() == (end() - 1)->vect() &&
		begin()->Number() != (end() - 1)->Number())
   {
      erase(end() - 1);
      push_back(*begin());
   }
}     
	 
House& House::operator=(const House& house)
{
   if (this == &house) return *this;  // Check for self assignment
   if (!empty()) 
     erase(begin(), end());
     
   num = house.num;	
   House::const_iterator i = house.begin();
   for(;i < house.end(); i++)
    	push_back(*i);
    	
   return *this;
}

/*
--------------------------------------------------------------------------------
                       Check if the house is closed
--------------------------------------------------------------------------------
*/

int House::IsComplet() const
{
/*   House::const_iterator i = begin();
   ObjectPoint2D pt1 = *i;	
   i = 	end() - 1;
   ObjectPoint2D pt2 = *i;	*/
   if (size() < 2)
     return false;
   if (begin()->Distance((end() - 1)->Position2DRef()) < MIN_DIST_MAP_POINTS)
	return true;
   return false;
}


/*
--------------------------------------------------------------------------------
                     Construct the LineTopology structure
--------------------------------------------------------------------------------
*/

void House::Convert2LineTop(LineTopology *line, ObjectPoints2D *objpts) const
{
   House::const_iterator i;
   PointNumber ptnr; 

   for(i = begin(); i < end(); i++)	
   {
	objpts->push_back(*i);
	ptnr = PointNumber(i->Number());
	line->push_back(ptnr);
   }	

/*   
   for(i = begin(); i < end() - 1; i++)	
   {
	objpts->push_back(*i);
	ptnr = PointNumber(i->Number());
	line->push_back(ptnr);
   }	
   
   // the end point = start point
   ptnr = PointNumber(begin()->Number());
   line->push_back(ptnr); */
}


bool House::IsHouseCorner(const ObjectPoint2D &pt) const
{
   return !(find_if(begin(), end(), SameObjectPoints2D(pt.NumberRef())) == end());
}


int House::GetHouseCorner(const Position2D &pt) const
{
   ObjectPoints2D::const_iterator i;   
   for(i = begin(); i < end() - 1; i++)
      if (Distance(*i, pt) < MIN_DIST_MAP_POINTS)
           return i - begin();
   return -1;    	
}

void House::MergeClosePoints(double dist)
{
   if (size() < 2)
      return;

   ObjectPoints2D::iterator i;
   
   for(i = begin(); i < end() - 2;)	
   {
      if (Distance(*i, *(i + 1)) < dist)   
      {
           i = erase(i + 1);
           i--;
      }     
      else 
         i++;      
   }   
   if (Distance(*begin(), *(end() - 2)) < dist)
        erase(end() - 2); 
}      


void House::DeleteCollinearPoints()
{
   if (size() < 2)
      return;

   ObjectPoints2D::iterator i;
   Line2D s;
   
   for(i = begin(); i < end() - 2;)	
   {
      //s = Line2D(*i, *(i + 2));
      //if (s.PointOnLine(*(i + 1), MIN_DIST_MAP_POINTS))
      if (CollinearPoints(*i, *(i + 1), *(i + 2), MIN_DIST_MAP_POINTS))
      {
          /* CHANGE 21.02.2001
          erase(i + 1);  */
//          printf("delete %d\n", (i + 1)->Number());
          i = erase(i + 1); i--;
      }
      else
       	  i++;
   }
   
 //  s = Line2D(*(begin() + 1), *(end() - 2));
//   if (s.PointOnLine(*begin(), MIN_DIST_MAP_POINTS))
   if (CollinearPoints(*(end() - 2), *begin(), *(begin() + 1), MIN_DIST_MAP_POINTS))
   {
        erase(end() - 1);
        erase(begin());
        push_back(*begin());
   }
}        


void House::Print() const
{
   ObjectPoints2D::const_iterator i;
   for(i = begin(); i < end(); i++)
      printf("%d  ", i->Number());
   printf("\n");
}
       	  

bool House::AreNeighbourCorners(const ObjectPoint2D &pt1, 
			const ObjectPoint2D &pt2) const
{
   ObjectPoints2D::const_iterator i;
   for(i = begin(); i < end() - 1; i++)
   {
      if (i->Number() == pt1.Number())
      {
         if ((i + 1)->Number() == pt2.Number())
            return true;
         if (i == begin())
         {
            if ((end() - 2)->Number() == pt2.Number())
               return true;
         }      
         else      
            if ((i - 1)->Number() == pt2.Number())
               return true;
         return false;
      }
   }
   return false;
}               

const ObjectPoint2D &House::GetHouseCorner(int corner_no) const
{ 
   if (corner_no >= size() - 1) 
         corner_no = corner_no - size() + 1;
   return *(begin() + corner_no); 
}


void House::GetHouseLine(int lin_no, Segment2D &seg) const
{
  seg = Segment2D(GetHouseCorner(lin_no), GetHouseCorner(lin_no + 1));
}  

/* Segment3D has no function EndPoint. The next House function is therefore commented out. 
 * It is not used in any digphot programme.
 *
 * George.
 */
/*
void House::GetMapLineSegments(int lin_no, Segments3D &seg) const
{
  Segments3D::iterator i;
  Segment2D map_seg;
  GetHouseLine(lin_no, map_seg);
  
  i = seg.begin(); 
  while(i < seg.end())
  {
     //Segment2D s = i->Segm2D();
     Vector2D v1 = i->EndPoint(0).vect2D();
     Vector2D v2 = i->EndPoint(1).vect2D();
     Segment2D s(v1, v2);
     //Segment2D s(i->EndPoint(0).vect2D(), i->EndPoint(1).vect2D());
     if (!ParallelLines(map_seg, s, ERR_PARALLEL_LINES))
        seg.erase(i);
     else 
        i++;   
  }              	
}
*/


bool House::PointInsideHouse(const Position2D &point) const
{
  int num_intersections = 0;
  double r0, c0, r1, c1, c_test;

  ObjectPoints2D::const_iterator i;
  /*PointNumber pt1, pt2; 
  pt1 = begin()->Number();
  pt2 = (end() - 1)->Number();  */
  
  // Check if the polygon is closed 
  if (!IsComplet())
  	  return(0);  

  // Get the coordinates of the first polygon point 
  r0 = begin()->X();
  c0 = begin()->Y();

  // Count the number of intersections of polygon edges with the line from the
  // point (r,c) to (r, inf).
  for(i = begin() + 1; i != end(); i++)
  {
    // Get the coordinates of the next polygon point
    r1 = i->X();
    c1 = i->Y();

    //Check whether the lines intersect 
    if ((r0 - point.X()) * (r1 - point.X()) <= 0 && r1 != point.X()) 
    {
      c_test = ((r1 - point.X()) * c0 + (point.X() - r0) * c1) / (r1 - r0);
      if (c_test > point.Y()) num_intersections++;
    }
    
    r0 = r1;
    c0 = c1;
  }

  if ((num_intersections/2)*2 == num_intersections) 
        return(0);
  else 
        return(1);
}


int House::GetHouseSegment(const Position2D &pt) const
{
   std::vector<Segment> map_seg;
   GetHouseSegments(map_seg);
 
   std::vector<Segment>::const_iterator s;   
   for(s = map_seg.begin(); s < map_seg.end(); s++)
       if (s->PointOnSegment(pt, 1, MIN_DIST_MAP_POINTS))
          return s - map_seg.begin();
   return -1;
}



int House::SegmentIntersectHouse(const Segment &seg) const 
{
   std::vector<Segment> map_seg;
   GetHouseSegments(map_seg);
 
   Position2D pos;
   std::vector<Segment>::const_iterator s;   
   for(s = map_seg.begin(); s < map_seg.end(); s++)
       if (!(*s == seg) && !(s->EndPoint(0) == seg.EndPoint(0)) && 
        	!(s->EndPoint(0) == seg.EndPoint(1)) &&
        	!(s->EndPoint(1) == seg.EndPoint(0)) &&
        	!(s->EndPoint(1) == seg.EndPoint(1)) &&
                Intersection2Segments(*s, seg, MIN_DIST_MAP_POINTS, pos))
          return s - map_seg.begin();
   return -1;
}


int House::SegmentIntersectHouseDiff(const Segment &seg) const 
{
   std::vector<Segment> map_seg;
   GetHouseSegments(map_seg);
 
   Position2D pos;
   std::vector<Segment>::const_iterator s;   
   for(s = map_seg.begin(); s < map_seg.end(); s++)
    if (!(*s == seg) && !(s->EndPoint(0) == seg.EndPoint(0)) && 
        	!(s->EndPoint(0) == seg.EndPoint(1)) &&
        	!(s->EndPoint(1) == seg.EndPoint(0)) &&
        	!(s->EndPoint(1) == seg.EndPoint(1)) && 
            !s->PointOnLine(seg.EndPoint(0), MIN_DIST_MAP_POINTS) &&
            !s->PointOnLine(seg.EndPoint(1), MIN_DIST_MAP_POINTS))
    {
/*      if (CollinearSegments(*s, seg, MIN_DIST_MAP_POINTS))
      {
          Segment ss;
          IntersectionCollinearSegments(*s, seg, ss);
          if ((ss.EndPoint(0) == s->EndPoint(0) && ss.EndPoint(1) == s->EndPoint(1)) ||
			  (ss.EndPoint(0) == s->EndPoint(1) && ss.EndPoint(1) == s->EndPoint(0)))
                return s - map_seg.begin();
      }
*/
      if (Intersection2Segments(*s, seg, MIN_DIST_MAP_POINTS, pos))
      {
         Segment ss1, ss2;
         if (s != map_seg.begin())
           ss1 = *(s - 1);
         else 
           ss1 = *(map_seg.end() - 1); 

         if (s != map_seg.end() - 1)
           ss2 = *(s + 1);
         else 
           ss2 = *(map_seg.begin()); 

         if (!CollinearSegments(seg, ss1, MIN_DIST_MAP_POINTS) && 
			!CollinearSegments(seg, ss2, MIN_DIST_MAP_POINTS))
            return s - map_seg.begin();
      }
    }
   return -1;
}



void House::ShiftHouse(double offsetx, double offsety)
{
   ObjectPoints2D::iterator i;   
   for(i = begin(); i < end(); i++)
   {
     i->X() = i->X() - offsetx;
     i->Y() = i->Y() - offsety;
   }
}


/************************************************************/
/*                 HOUSE PARTIONING                         */
/************************************************************/

void PrintRectangles(const std::vector<ObjectPoints2D> &rect_list);


void House::DivideHousePlan(std::vector<ObjectPoints2D> &rect_list, 
				PointNumber &ptno) const
{      
   if (size() <= 5)
   { 
     rect_list.push_back(*this);
     return;
   }
 

   HouseGraph graph;   
   std::vector<Segment> seg;
   ObjectPoints2D::const_iterator i;   
   int k = 0;
   std::vector<int>  col_seg_ind;

   // build the initial graph
   for(i = begin(); i < end() - 1; i++, k++)
   {
      seg.push_back(Segment(i->pos(), (i + 1)->pos()));
    
      HouseGraphNode *node = new HouseGraphNode;  
      node->pt = *i;
      //node.neighbours.clear();
      graph.push_back(node);
      if (i != begin())
      {
         graph[k]->neighbours.push_back(graph[k - 1]); 	 
	 graph[k - 1]->neighbours.push_back(graph[k]);
      } 	 
   }   
   graph[--k]->neighbours.push_back(graph[0]); 	 
   graph[0]->neighbours.push_back(graph[k]);   

   // search for collinear segments
   int k1 = 0, k2 = 0;
   std::vector<Segment>::const_iterator s1, s2;   
   for(s1 = seg.begin(); s1 < seg.end() - 1; s1++, k1++)
   {
     for(s2 = s1 + 2, k2 = k1 + 2; s2 < seg.end(); s2++, k2++)      
      if (CollinearSegments(*s1, *s2, MIN_DIST_MAP_POINTS))
      {
        Position2D posm1, posm2;
	    posm1 = s2->PointAtDistance(s1->EndPoint(0), 1);
	    posm2 = s1->PointAtDistance(s2->EndPoint(0), 1);
	    if (PointInsideHouse(posm1) && PointInsideHouse(posm2))
     { 
      int ind1 = k1;
	  int ind2 = k2;

col_seg_ind.insert(col_seg_ind.begin(), k1);
#ifdef __DEBUG
printf("col segs: %d  %d\n", k1, k2);
#endif

     double d1 = Distance(*(begin() + ind1), *(begin() + ind2));
	 double d2 = Distance(*(begin() + ind1 + 1), *(begin() + ind2));	
	 
	 if (d1 < d2) 
	 {
	      double d3 = Distance(*(begin() + ind1), *(begin() + ind2 + 1)); 
	      if (d1 < d3)
	      {
	       graph[ind1]->neighbours.push_back(graph[ind2]);
	       graph[ind2]->neighbours.push_back(graph[ind1]);
	      }
	      else
	      {
           if (ind2 == seg.size() - 1)
               ind2 = -1;
	       graph[ind1]->neighbours.push_back(graph[ind2 + 1]);
	       graph[ind2 + 1]->neighbours.push_back(graph[ind1]);
	      }
	 }   
	 else
	 {
	    double d3 = Distance(*(begin() + ind1 + 1), *(begin() + ind2 + 1)); 
	    if (d2 < d3)
	    {
	       graph[ind1 + 1]->neighbours.push_back(graph[ind2]);
	       graph[ind2]->neighbours.push_back(graph[ind1 + 1]);
	    }
	    else
	    {
           if (ind2 == seg.size() - 1)
                   ind2 = -1;
	       graph[ind1 + 1]->neighbours.push_back(graph[ind2 + 1]);
	       graph[ind2 + 1]->neighbours.push_back(graph[ind1 + 1]);
	    }
	 }  
       } // interior point
      }   		       		 
   }

/*
#ifdef __DEBUG         
   printf("house graph after determining collinear house segments\n");
   graph.Print();
   printf("\n");      
#endif
*/

   // check graph
   std::vector<HouseGraphNode*>::iterator gr;      
   for(gr = graph.begin(); gr < graph.end(); gr++)
   {
      if ((*gr)->neighbours.size() > 3)
      {
         std::vector<HouseGraphNode*>::iterator g1, g2;
         for(g1 = (*gr)->neighbours.begin(); g1 < (*gr)->neighbours.end() - 1; g1++)  
           for(g2 = g1 + 1; g2 < (*gr)->neighbours.end(); g2++)  
           {
              Segment2D s((*g1)->pt, (*g2)->pt);
              //if (CollinearPoints((*g1)->pt, (*g2)->pt, (*gr)->pt, 0.3))
              if (s.PointOnLine((*gr)->pt, 0.3) && !s.PointOnSegment((*gr)->pt))
              {
                  double d1 = Distance((*g1)->pt, (*gr)->pt);
                  double d2 = Distance((*g2)->pt, (*gr)->pt);
                  double d = s.Length();
                  if (d1 > d || d2 > d) 
                  {  
		     // point ouside of segment
                     if (d1 < d2)
                     {
			// delete g2 from neighbour list
                        std::vector<HouseGraphNode*>::iterator gg2;
                        for(gg2 = (*g2)->neighbours.begin() + 2; 
				gg2 < (*g2)->neighbours.end(); gg2++) 
                            if(*gg2 == *gr)
                            {
                               (*g2)->neighbours.erase(gg2);
                               break;
                            }
                        g2 = (*gr)->neighbours.erase(g2); 
                        g2--;
                     }
                     else   
                     {
			// delete g1 from neighbour list
                        std::vector<HouseGraphNode*>::iterator gg1;
                        for(gg1 = (*g1)->neighbours.begin() + 2; 
				gg1 < (*g1)->neighbours.end(); gg1++) 
                            if(*gg1 == *gr)
                            {
                               (*g1)->neighbours.erase(gg1);
                               break;
                            }
                        g1 = (*gr)->neighbours.erase(g1); 
                        g1--;
			break;
                     }
                   }
               }
            }
       }
   }  


#ifdef __DEBUG         
   printf("house graph after determining collinear house segments\n");
   graph.Print();
   printf("\n");      
#endif


   // find intersection points between the segments of the ground plan      
   std::vector<Segment> seg_new, seg_new1;
   Position2D pos;   
   for(s1 = seg.begin(), k1 = 0; s1 < seg.end(); s1++, k1++)
      for(s2 = seg.begin(), k2 = 0 ; s2 < seg.end(); s2++, k2++)      
       if (!(s1 == s2) && !(s1->EndPoint(0) == s2->EndPoint(0)) && 
        	!(s1->EndPoint(0) == s2->EndPoint(1)) &&
        	!(s1->EndPoint(1) == s2->EndPoint(0)) &&
        	!(s1->EndPoint(1) == s2->EndPoint(1)) &&
                Intersection2Segments(*s1, *s2, MIN_DIST_MAP_POINTS, pos, 0) &&    
                s1->PointOnSegment(pos))
       {       

     Segment ss1, ss2;
     if (s1 != seg.begin())
        ss1 = *(s1 - 1);
     else 
        ss1 = *(seg.end() - 1); 
     if (s1 != seg.end() - 1)
        ss2 = *(s1 + 1);
     else 
        ss2 = *(seg.begin()); 

     if (!CollinearSegments(*s2, ss1, MIN_DIST_MAP_POINTS) && 
			!CollinearSegments(*s2, ss2, MIN_DIST_MAP_POINTS))
     {    
	  Position2D posm;
      if (Distance(pos, s2->EndPoint(0)) < DIST)
      {
            ss1 = Segment(pos, s2->EndPoint(0));
            posm = ss1.MiddlePoint();
      }
      else
      {
        if (Distance(pos, s2->EndPoint(1)) < DIST)
        {
            ss1 = Segment(pos, s2->EndPoint(1));
            posm = ss1.MiddlePoint();
        }
        else  
	       posm = s2->PointAtDistance(pos, DIST);	  
      }

	  if (PointInsideHouse(posm))
	  {
	    // add the intersection point to the neighbours of 
	    // the closest endpoint of s2
        bool do_insert = true, new_node = true;
	    std::vector<HouseGraphNode*>::iterator j;
	    int incr = s2 - seg.begin();
        ObjectPoint2D pt_close, pt(pos, ptno, Covariance2D());        
	    HouseGraphNode *node;
        if (Distance(pos, s1->EndPoint(0)) > MIN_DIST_MAP_POINTS && 
                Distance(pos, s1->EndPoint(1)) > MIN_DIST_MAP_POINTS)         
        {
          node = new HouseGraphNode; 
	      node->pt = pt;
        }
        else
        {
          new_node = false; 
          for(j = graph.begin() + k1; j < graph.end(); j++)
            if (Distance((*j)->pt, pos) < MIN_DIST_MAP_POINTS)
            {
					node = *j;
//printf("d=%6.6f, nd=%d\n", Distance((*j)->pt, pos), (*j)->pt.Number());
//printf("%12.6f  %12.6f\n", pos.X(), pos.Y());
                    break;
            }   
        }
	    
	    if (Distance(pos, *(begin() + incr)) < 
	    			Distance(pos, *(begin() + incr + 1)))	    
               pt_close = *(begin() + incr);
	    else   
        {
	       pt_close = *(begin() + incr + 1);
           if (incr + 1 == seg.size())
                      incr = 0;
        }          

        Segment ss(pt_close, pt);	       

/*
if (ptno.Number() == 4)
{
   
Position2D ppos;
printf("bool = %d, %d,  %d  %6.6f  %6.6f  %d\n", Intersection2Segments(*(seg.begin() + 8), ss, MIN_DIST_MAP_POINTS, ppos), 
	Intersection2Segments(*(seg.begin() + 9), ss, MIN_DIST_MAP_POINTS, ppos), 
     Intersection2Segments(*(seg.begin() + 10), ss, MIN_DIST_MAP_POINTS, ppos),
      (seg.begin() + 10)->DistanceToPoint(pt_close), (seg.begin() + 10)->DistanceToPoint(pt), 
      SegmentIntersectHouseDiff(ss));   
}
*/

        if (SegmentIntersectHouseDiff(ss) != -1)
            do_insert = false;
        else   
        {
            for(j = graph.begin() + incr; j < graph.end(); j++)
            {
               if ((*j)->pt.Number() == pt_close.Number())
	           {
                  std::vector<HouseGraphNode*>::iterator n;
		          for(n = (*j)->neighbours.begin(); n < (*j)->neighbours.end(); n++)
                  {
                      if (ss.PointOnSegment((*n)->pt, 1, MIN_DIST_MAP_POINTS))
                      {
                          do_insert = false;
                          break;
                      }
                  }     
                  if (do_insert)
                  { 
                     (*j)->neighbours.push_back(node);    
		             node->neighbours.push_back(*j);
                  } 
                  else
		            break;
	            }
	        }
         }	  	    

        if (do_insert && !new_node) 
        {
#ifdef __DEBUG
              printf("seg_new1: %d, %d, k1=%d, k2=%d\n", node->pt.Number(), pt_close.Number(), k1, k2);
#endif  
           seg_new1.push_back(Segment(node->pt, pt_close));   	 
        } 

        if (do_insert && new_node)
        {
           ptno++;
           seg_new.push_back(Segment(pt, pt_close));   

           // add the point to the points of the ground plan 
           // between the endpoints of s1
           bool found = false;
           incr = s1 - seg.begin();
           double d0 = Distance(pos, *(begin() + incr));            
	   
           for(j = graph.begin() + incr; j < graph.end(); j++)
           {
             if (found)
             {     
                double d = Distance((*j)->pt, *(begin() + incr));
                if (d0 < d)
                {
		   // add point between j-1 and j
		   std::vector<HouseGraphNode*>::iterator n;
		   for(n = (*j)->neighbours.begin(); n < (*j)->neighbours.end(); n++)
		     if ((*n)->pt.Number() == (*(j - 1))->pt.Number())
		     {
		       (*j)->neighbours.erase(n);
		       (*j)->neighbours.push_back(node);
		       break;
		     }
		   for(n = (*(j - 1))->neighbours.begin(); n < (*(j - 1))->neighbours.end(); n++)
		     if ((*n)->pt.Number() == (*j)->pt.Number())
		     {
		       (*(j - 1))->neighbours.erase(n);
		       (*(j - 1))->neighbours.push_back(node);
		       break;
		     }          
		   node->neighbours.push_back(*j);
		   node->neighbours.push_back(*(j - 1));       		   
                   graph.insert(j, node);             
                   break;
                }
             }
             
             if ((*j)->pt.Number() == (begin() + incr)->Number())             
                found = true;                         
                
             if (j + 1 < graph.end() && 
             	    (*(j + 1))->pt.Number() == (begin() + incr + 1)->Number())   
             {      
	           // add point between j and j + 1
	           std::vector<HouseGraphNode*>::iterator n;
		       for(n = (*j)->neighbours.begin(); n < (*j)->neighbours.end(); n++)
		         if ((*n)->pt.Number() == (*(j + 1))->pt.Number())
		         {
		            (*j)->neighbours.erase(n);
		            (*j)->neighbours.push_back(node);
		            break;
		         }
		for(n = (*(j + 1))->neighbours.begin(); n < (*(j + 1))->neighbours.end(); n++)
		   if ((*n)->pt.Number() == (*j)->pt.Number())
		   {
		      (*(j + 1))->neighbours.erase(n);
		      (*(j + 1))->neighbours.push_back(node);
		      break;
		   }          
		node->neighbours.push_back(*j);
		node->neighbours.push_back(*(j + 1));       
                graph.insert(j + 1, node);             
                break;             
             }   
             
             if (j + 1 == graph.end() && 
             	    (*(graph.begin()))->pt.Number() == (begin() + incr + 1)->Number())   
             {      
	        // add point between j and 0
	        std::vector<HouseGraphNode*>::iterator n;
		for(n = (*j)->neighbours.begin(); n < (*j)->neighbours.end(); n++)
		   if ((*n)->pt.Number() == (*(graph.begin()))->pt.Number())
		   {
		      (*j)->neighbours.erase(n);
		      (*j)->neighbours.push_back(node);
		      break;
		   }
		for(n = (*(graph.begin()))->neighbours.begin(); 
		    n < (*(graph.begin()))->neighbours.end(); n++)
		   if ((*n)->pt.Number() == (*j)->pt.Number())
		   {
		      (*(graph.begin()))->neighbours.erase(n);
		      (*(graph.begin()))->neighbours.push_back(node);
		      break;
		   }          
		node->neighbours.push_back(*j);
		node->neighbours.push_back(*(graph.begin()));       
//                //graph.insert(graph.begin(), node);             
                graph.push_back(node);      
                break;             
             }                
          }
         }
        }
       }
      }	   


#ifdef __DEBUG         
   printf("house graph after intersecting house segments\n");
   graph.Print();
   printf("\n");      
#endif

/*
//delete col segs
    std::vector<int>::const_iterator ii;
    for(ii = col_seg_ind.begin(); ii < col_seg_ind.end(); ii++)
      seg.erase(seg.begin() + *ii);
*/

//seg_new.insert(seg_new.end(), seg_new1.begin(), seg_new1.end());

   // look for internal intersection points	between interior segments    
   // and ground plan segments	  
   int kk1 = 0, kk2 = 0;
   std::vector<Segment>::const_iterator s11;
   for(s11 = seg_new.begin(); s11 < seg_new.end(); s11++, kk1++)
      for(s2 = seg.begin(), kk2 = 0; s2 < seg.end(); s2++, kk2++)      
       if ( !Member(col_seg_ind, kk2) &&
            !(s11->EndPoint(0) == s2->EndPoint(0)) && 
        	!(s11->EndPoint(0) == s2->EndPoint(1)) &&
        	!(s11->EndPoint(1) == s2->EndPoint(0)) &&
        	!(s11->EndPoint(1) == s2->EndPoint(1)) &&
                Intersection2Segments(*s11, *s2, MIN_DIST_MAP_POINTS, pos, 0) &&
                (Distance(pos, s11->EndPoint(0)) > MIN_DIST_MAP_POINTS) && 
                (Distance(pos, s11->EndPoint(1)) > MIN_DIST_MAP_POINTS) &&       
                s11->PointOnSegment(pos) &&
//				(graph.GraphPoint(pos, MIN_DIST_MAP_POINTS) == -1) &&
                PointInsideHouse(pos))    
      {   
		int c = graph.GraphPoint(pos, MIN_DIST_MAP_POINTS);
        if (c != -1)
        {
           Segment ss(s2->EndPoint(0), pos);	       
          if (SegmentIntersectHouseDiff(ss) == -1)
          { 
 	       HouseGraphNode *node = *(graph.begin() + c); 
#ifdef __DEBUG
           printf("kk1=%d, kk2=%d  c=%d, no=%d\n", kk1, kk2, c, node->pt.Number());
#endif
	       graph.InsertGraphNode(node, s11->EndPoint(0), s11->EndPoint(1));
	       graph.InsertGraphNode(node, s2->EndPoint(0), s2->EndPoint(1));

          }
        }

else
{
     bool do_insert1 = true;
     for(s1 = seg_new1.begin(); s1 < seg_new1.end(); s1++)
        if ((s1->EndPoint(0) == s11->EndPoint(1) || 
				s1->EndPoint(1) == s11->EndPoint(1)) &&
                s2->PointOnLine(s1->EndPoint(0), MIN_DIST_MAP_POINTS) &&
                s2->PointOnLine(s1->EndPoint(1), MIN_DIST_MAP_POINTS))
     {
         do_insert1 = false;
#ifdef __DEBUG
              printf("kk1=%d, kk2=%d\n", kk1, kk2);
#endif
         break;
     }


if (do_insert1)
{
     int c = GetHouseCorner(s11->EndPoint(1));
     if (c == -1)
       printf("Error: It should be a house corner: %d, %d\n", kk1, kk2);
     assert(c != -1);

     Segment ss1, ss2;
     if (c != 0)
        ss1 = *(seg.begin() + c - 1);
     else 
        ss1 = *(seg.end() - 1); 
     ss2 = *(seg.begin() + c);

     if (!CollinearSegments(*s2, ss1, MIN_DIST_MAP_POINTS) && 
			!CollinearSegments(*s2, ss2, MIN_DIST_MAP_POINTS))
{
    bool do_insert = false;
    c = GetHouseSegment(s11->EndPoint(0));
     if (c != -1)
     {
       ss1 = *(seg.begin() + c);
       if (!(s2->PointOnLine(ss1.EndPoint(0), 0.5) && 	!s2->EndPoint(ss1.EndPoint(0)) 
       		&& ParallelLines(ss1, *s2, ERR_PARALLEL_LINES))) 
         do_insert = true;
     }
     else
     {
       c = GetHouseCorner(s11->EndPoint(1));
       if (c == -1)
         printf("Error: The point should be either a house corner or on a house segment: %d, %d\n", kk1, kk2);
       assert(c != -1);   

       if (c != 0)
        ss1 = *(seg.begin() + c - 1);
       else 
        ss1 = *(seg.end() - 1); 
       ss2 = *(seg.begin() + c);

     if (!CollinearSegments(*s2, ss1, MIN_DIST_MAP_POINTS) && 
			!CollinearSegments(*s2, ss2, MIN_DIST_MAP_POINTS))
          do_insert = true; 
     }
     if (do_insert)
     {
         Position2D posm;
//	     posm = s2->PointAtDistance(s11->EndPoint(0), DIST);
	     posm = s2->PointAtDistance(pos, DIST);
         if (PointInsideHouse(posm))
         { 
            Segment ss;
            if (Distance(s2->EndPoint(0), pos) < Distance(s2->EndPoint(1), pos)) 
                ss = Segment(s2->EndPoint(0), pos);	       
            else 
                ss = Segment(s2->EndPoint(1), pos);	       
            if (SegmentIntersectHouseDiff(ss) == -1)
            {     
#ifdef __DEBUG
              printf("kk1=%d, kk2=%d  no=%d\n", kk1, kk2, ptno.Number());
#endif
//if (ptno.Number() == 39)  // || ptno.Number() == 30 || ptno.Number() == 31 || ptno.Number() == 32)
//   printf("dfhsjd\n");
              ObjectPoint2D pt(pos, ptno, Covariance2D());
              ptno.Number() = ptno.Number() + 1;
	   HouseGraphNode *node = new HouseGraphNode; 
	   node->pt = pt;
	 	 
	   graph.InsertGraphNode(node, s11->EndPoint(0), s11->EndPoint(1));
	   graph.InsertGraphNode(node, s2->EndPoint(0), s2->EndPoint(1));
	   graph.push_back(node);
// graph.PrintNode(10);
            }
         }
      } 
}}}}



   // check graph: a node can have the same neighbour twice
   for(gr = graph.begin(); gr < graph.end(); gr++)
   {
      if ((*gr)->neighbours.size() > 2)
      {
         std::vector<HouseGraphNode*>::iterator g1, g2;
         for(g1 = (*gr)->neighbours.begin(); g1 < (*gr)->neighbours.end() - 1; g1++)  
         {
           g2 = g1 + 1; 
           while(g2 < (*gr)->neighbours.end())  
           {
              if ((*g1)->pt.Number() == (*g2)->pt.Number())
                  g2 = (*gr)->neighbours.erase(g2);
              else
                  g2++; 
           }
         }
      }
   }  



#ifdef __DEBUG         
   printf("house graph after intersecting int segments with house segments\n");
   graph.Print();
   printf("\n");      
   printf("sz=%d\n\n", seg_new1.size());
#endif

/*
int b1 = Intersection2Segments(*(seg_new1.begin() + 0), *(seg_new.begin() + 13), MIN_DIST_MAP_POINTS, pos);
int b2 = graph.GraphPoint(pos, MIN_DIST_MAP_POINTS);
printf("%d, %d,  %6.6f,  %6.6f\n", b1, b2, pos.X(), pos.Y());
*/

   // look for internal intersection points	between interior segments    
   // and extended ground plan segments	  
   kk1 = 0, kk2 = 0;
   for(s11 = seg_new1.begin(); s11 < seg_new1.end(); s11++, kk1++)
      for(s2 = seg.begin(), kk2 = 0; s2 < seg.end(); s2++, kk2++)      
       if (!(s11->EndPoint(0) == s2->EndPoint(0)) && 
        	!(s11->EndPoint(0) == s2->EndPoint(1)) &&
        	!(s11->EndPoint(1) == s2->EndPoint(0)) &&
        	!(s11->EndPoint(1) == s2->EndPoint(1)) &&
                Intersection2Segments(*s11, *s2, MIN_DIST_MAP_POINTS, pos, 0) &&
                (Distance(pos, s11->EndPoint(0)) > MIN_DIST_MAP_POINTS) && 
                (Distance(pos, s11->EndPoint(1)) > MIN_DIST_MAP_POINTS) &&       
                s11->PointOnSegment(pos)) 
      {        
		int c = graph.GraphPoint(pos, MIN_DIST_MAP_POINTS);
        if (c != -1)
        {
 	       HouseGraphNode *node = *(graph.begin() + c); 
#ifdef __DEBUG
           printf("kk1=%d, kk2=%d  c=%d, no=%d\n", kk1, kk2, c, node->pt.Number());
#endif
//printf("gvhjsgdgfj\n");
	       graph.InsertGraphNode(node, s11->EndPoint(0), s11->EndPoint(1));
//           graph.Print(); 
//printf("gvhjsgdgfj\n");
	       graph.InsertGraphNode(node, s2->EndPoint(0), s2->EndPoint(1));
         }
      } 


#ifdef __DEBUG
   // print & write agjacency graph  
   std::vector<HouseGraphNode*>::const_iterator g;      
   ObjectPoints2D pts;
   for(g = graph.begin(); g < graph.end(); g++)
   {
      pts.push_back((*g)->pt);
      printf("Node: %d,  neighbours:", (*g)->pt.Number());
      std::vector<HouseGraphNode*>::const_iterator n;
      for(n = (*g)->neighbours.begin(); n < (*g)->neighbours.end(); n++)
         printf("%d, ", (*n)->pt.Number());
      printf("\n");
   }   	 
  pts.Write("qqq.pts");
#endif
 
///return;
  
  // find rectangles 
  graph.FindRects(rect_list);
//  PrintRectangles(rect_list); 
}      


void PrintRectangles(const std::vector<ObjectPoints2D> &rect_list)
{
   std::vector<ObjectPoints2D>::const_iterator i;
   ObjectPoints2D::const_iterator j;
   
   for(i = rect_list.begin(); i < rect_list.end(); i++)
   {
      printf("Rect %d:  ", i - rect_list.begin());
      for(j = i->begin(); j < i->end(); j++)
         printf("%d  ", j->Number());
      printf("\n");
   }
}         


void House::GetHouseSegments(std::vector<Segment> &map_seg) const
{
   ObjectPoints2D::const_iterator i;      
   for(i = begin(); i < end() - 1; i++)
      map_seg.push_back(Segment(i->pos(), (i + 1)->pos()));
}

/*
// collect colliniar lines and order them based on their length
void House::GetCollinearSegments(std::vector<Segment> &collin_lins) const
{
   // compute initial map segments
   std::vector<Segment> map_seg;
   GetHouseSegments(map_seg);
 
   // search for collinear segments
   std::vector<Segment>::const_iterator s1, s2;   
   int k1 = 0, k2;
   for(s1 = map_seg.begin(); s1 < map_seg.end() - 1; s1++, k1++)
     for(s2 = s1 + 2, k2 = k1 + 2; s2 < map_seg.end(); s2++, k2++)      
      if (s2->PointOnLine(s1->EndPoint(0), 0.5) && 
		!s2->EndPoint(s1->EndPoint(0)) 
       		&& ParallelLines(*s1, *s2, ERR_PARALLEL_LINES)) 
      {
        Position2D posm1, posm2;
        Segment2D ss1(s1->EndPoint(0), s1->EndPoint(1));
	posm1 = ss1.PointAtDistance(s2->EndPoint(0), 1);
        Segment2D ss2(s2->EndPoint(0), s2->EndPoint(1));
	posm2 = ss2.PointAtDistance(s1->EndPoint(0), 1);
	if (PointInsideHouse(posm1) && PointInsideHouse(posm2))
        {
           Segment ss;
           double len = UnionCollinearSegments(*s1, *s2, ss);

           // check if this collinear line is not already in the list
           std::vector<Segment>::iterator l;
           bool found = false;
           int pos_insert = -1;
           for(l = collin_lins.begin(); l < collin_lins.end(); l++)
           {
              if (l->PointOnLine(s1->EndPoint(0), 0.5) &&
	           l->PointOnLine(s1->EndPoint(1), 0.5))
              {           
                  l = collin_lins.erase(l);
  		  // insert in order
                  bool ins = false;
                  for(; l >= collin_lins.begin(); l--)
		     if (l->Length() > len)
                     {
                        collin_lins.insert(l, ss);
                        ins = true;
                        break;
                     } 
                  if (!ins)
                      collin_lins.insert(collin_lins.begin(), ss);
                  found = true;
                  break;
              }
              else
              {
                 if (pos_insert == -1 && l->Length() < len)
                      pos_insert = l - collin_lins.begin();
              }
           }
           if (!found)
           {
             printf("collin_seg %d,  %d\n", k1, k2);
             if (pos_insert != -1)
		collin_lins.insert(collin_lins.begin() + pos_insert, ss);
             else 
                collin_lins.push_back(ss);
           }
        } 
      }
   printf("no collin lines = %d\n", collin_lins.size()); 
}
*/

bool House::CollectCollinearSegments(const Segment &s, 
		std::vector<Segment> &collin_lins) const
{
   // compute initial map segments
   std::vector<Segment> map_seg;
   GetHouseSegments(map_seg);
 
   // search for collinear segments
   std::vector<Segment>::const_iterator s1;   
   int k1 = 0;
   for(s1 = map_seg.begin(); s1 < map_seg.end(); s1++, k1++)
      if (!(*s1 == s) && s.PointOnLine(s1->EndPoint(0), 0.5) && 
		!s.EndPoint(s1->EndPoint(0)) 
       		&& ParallelLines(*s1, s, ERR_PARALLEL_LINES)) 
      {
          collin_lins.push_back(*s1);
      }
   return (!collin_lins.empty()); 
}


bool Member(std::vector<int> list, int n)
{
   std::vector<int>::const_iterator i;
   for(i = list.begin(); i < list.end(); i++)
      if (*i == n)
        return true;
   return false;
}
