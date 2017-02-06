
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



#include "HouseGraph.h"
#include "Constants.h"
#include "Segment2D.h"
#include "Segment.h"

bool MemberSubList(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list);
bool MemberList(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list);
int MemberList3(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list);
bool Member(const ObjectPoint2D &node, const ObjectPoints2D &rect);


// #define __DEBUG

HouseGraph::~HouseGraph()
{
   std::vector<HouseGraphNode*>::iterator i;
   for(i = begin(); i < end(); i++)
   {
      (*i)->neighbours.clear();
      delete *i;
   }  
   clear();
}


void HouseGraph::FindRects(std::vector<ObjectPoints2D> &rect_list) const
{
  HouseGraph::const_iterator g;
  ObjectPoints2D visited_nodes;
  for(g = begin(); g < end(); g++)
  {
     ObjectPoints2D rect;
     rect.push_back((*g)->pt);
     visited_nodes.push_back((*g)->pt);
     FindRect(*g, rect, rect_list, visited_nodes);
  }
}  


void HouseGraph::FindRect(const HouseGraphNode* node,  
	ObjectPoints2D &rect, std::vector<ObjectPoints2D> &rect_list,
        const ObjectPoints2D &visited_nodes) const
{
    if (rect.size() > 2 && Neighbours(*rect.begin(), *(rect.end() - 1)))
    {
        if (!MemberList(rect, rect_list))
        {
//          printf(" add rect %d %d %d %d\n", rect[0].Number(), rect[1].Number(),
//		rect[2].Number(), rect[3].Number());     
          rect_list.push_back(rect);
        } 
   }
   
   std::vector<HouseGraphNode*>::const_iterator i;
   for(i = node->neighbours.begin(); i < node->neighbours.end(); i++)
   {
      if (!Member((*i)->pt, rect) && !Member((*i)->pt, visited_nodes))
      {
         bool badd = true;  
         if (rect.size() > 1)
         {
            rect.push_back(*rect.begin());          
            ObjectPoints2D::const_iterator pt;
            for(pt = rect.begin(); pt < rect.end() - 1; pt++)
            { 
               Line2D lin(*pt, *(pt + 1));
               if (lin.PointOnLine((*i)->pt, 0.1))  
               {
                  badd = false;
                  break;
               }
            }
            rect.erase(rect.end() - 1);
         }
         if (badd)
         {    
           ObjectPoints2D r1;
           r1.insert(r1.begin(), rect.begin(), rect.end());
           r1.push_back((*i)->pt);
           FindRect(*i, r1, rect_list, visited_nodes);
         }
      }
   }
}



/*
void HouseGraph::FindRect(const HouseGraphNode* node,  
	ObjectPoints2D &rect, std::vector<ObjectPoints2D> &rect_list) const
{
    int ind;
      
    if (rect.size() > 2 && Neighbours(*rect.begin(), *(rect.end() - 1)))
    {
      if((ind = MemberList3(rect, rect_list)) != -1)
      {
         if ((rect_list.begin() + ind)->size() > rect.size())
         {
                rect_list.erase(rect_list.begin() + ind);
                rect_list.push_back(rect);
         }       
      }
      else
      {
        printf(" add rect %d %d %d %d\n", rect[0].Number(), rect[1].Number(),
		rect[2].Number(), rect[3].Number());     
        rect_list.push_back(rect);
      }
      return;
   }
   
   std::vector<HouseGraphNode*>::const_iterator i;
   for(i = node->neighbours.begin(); i < node->neighbours.end(); i++)
   {
      if (!Member((*i)->pt, rect))
      {
         ObjectPoints2D r1;
         r1.insert(r1.begin(), rect.begin(), rect.end());
         r1.push_back((*i)->pt);
         if (r1.size() > 2 && ((ind = MemberList3(r1, rect_list)) != -1))
         {
            if ((rect_list.begin() + ind)->size() > r1.size())
              FindRect(*i, r1, rect_list);
         }
         else     
           FindRect(*i, r1, rect_list);
      }
   }
}*/



bool HouseGraph::Neighbours(const ObjectPoint2D &n1, 
				const ObjectPoint2D &n2) const
{       
  std::vector<HouseGraphNode*>::const_iterator g;
  for(g = begin(); g < end(); g++)
  {
     if ((*g)->pt.Number() == n1.Number())
     {
       std::vector<HouseGraphNode*>::const_iterator i;
       for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
       {
         if ((*i)->pt.Number() == n2.Number())
           return true;
       }
       return false;
     }
     if ((*g)->pt.Number() == n2.Number())
     {
       std::vector<HouseGraphNode*>::const_iterator i;
       for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
       {
         if ((*i)->pt.Number() == n1.Number())
           return true;
       }
       return false;
     }  
  }
  return false;
}



int HouseGraph::GraphPoint(const Position2D &pos, double err_dist) const
{
  std::vector<HouseGraphNode*>::const_iterator g;
  for(g = begin(); g < end(); g++)
  {
     if (Distance((*g)->pt, pos) < err_dist)
        return g - begin();
  }
  return -1;
}        


void HouseGraph::InsertGraphNode(HouseGraphNode* node, 
	const Position2D &pos1, const Position2D &pos2)
{
   Segment s(pos1, pos2);
   if (s.PointOnSegment(node->pt))
   {
      // look if there is an arc in the graph between pos1 and pos2
      bool found = false;
      std::vector<HouseGraphNode*>::iterator g;
      for(g = begin(); g < end(); g++)
      {
        if (Distance((*g)->pt, pos1) < MIN_DIST_MAP_POINTS / 2)
        {
          std::vector<HouseGraphNode*>::iterator i;
          for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
          {
             if (Distance((*i)->pt, pos2) < MIN_DIST_MAP_POINTS / 2)
	     {
	        DeleteNeighbour(*i, pos1);
	        (*i)->neighbours.push_back(node);
	        node->neighbours.push_back(*i);
	        (*g)->neighbours.erase(i);
	        (*g)->neighbours.push_back(node);
	        node->neighbours.push_back(*g);
	        found = true;
	        break;
	     }
	  }      	             
          if (!found)
          {
             for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
             {
                if ((*i)->pt.Number() == node->pt.Number())
           {
#ifdef __DEBUG
               printf("no insert\n");
#endif

                    return;
}
                if ((s.PointOnLine((*i)->pt, MIN_DIST_MAP_POINTS) && 
                	(Distance((*i)->pt, pos2) < Distance(pos1, pos2))))
                {
                    InsertGraphNode(node, (*i)->pt, pos2);
		            return;
                }
             }
             InsertGraphNode(node, pos2, (*i)->pt);  
             return;
          }
          else
            break;
        }
      }        
   }       
   else
   {                		        
     Position2D pos_close;
     if (Distance(pos1, node->pt) < Distance(pos2, node->pt))
      pos_close = pos1;
     else
      pos_close = pos2;
  
     std::vector<HouseGraphNode*>::iterator g;
     double d1, d2;
     for(g = begin(); g < end(); g++)
     {
       if (Distance((*g)->pt, pos_close) < MIN_DIST_MAP_POINTS / 2)
       {
         std::vector<HouseGraphNode*>::iterator i;
         for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
         {
           if ((*i)->pt.Number() == node->pt.Number())
           {
#ifdef __DEBUG
               printf("no insert\n");
#endif
                        return;
           }           


           Segment ss((*i)->pt, pos_close);
/*
if (node->pt.Number() == 26 && (*i)->pt.Number() == 1204)
{
  printf("%d,  %d, %d   %d,  %6.6f\n", (*g)->pt.Number(), (*i)->pt.Number(),
		ss.PointOnLine((*i)->pt, MIN_DIST_MAP_POINTS), 
        ss.PointOnSegment(node->pt), ss.DistanceToPoint(node->pt));
  ss.PointOnSegment(node->pt);
} */

           if (ss.PointOnLine(node->pt, MIN_DIST_MAP_POINTS) && 
           	(ss.PointOnSegment(node->pt) || 
           	 Distance(pos_close, node->pt) > Distance((*i)->pt, node->pt)))
           {     
             InsertGraphNode(node, pos_close, (*i)->pt);
             return;
           }
         }  


         Segment ss(node->pt, pos_close);
         for(i = (*g)->neighbours.begin(); i < (*g)->neighbours.end(); i++)
         {

if (node->pt.Number() == 26 && (*i)->pt.Number() == 1204)
{
  printf("%d,  %d, %d   %d,  %6.6f\n", (*g)->pt.Number(), (*i)->pt.Number(),
		ss.PointOnLine((*i)->pt, MIN_DIST_MAP_POINTS), 
        ss.PointOnSegment(node->pt), ss.DistanceToPoint(node->pt));
  ss.PointOnSegment(node->pt);
} 

           if (ss.PointOnLine((*i)->pt, MIN_DIST_MAP_POINTS) && 
           	 Distance(pos_close, node->pt) > Distance((*i)->pt, node->pt))
           {     
             InsertGraphNode(node, pos_close, (*i)->pt);
             return;
           }
         }  

         printf("Error!!!!!!: An arc should exist ");
         PrintNode(**g);
       }  
     }   
  }   
} 

/*
void HouseGraph::AddNeighbour(HouseGraphNode* node1, HouseGraphNode* node2)     
{
   std::vector<HouseGraphNode*>::iterator i;
   for(i = node1->neighbours.begin(); i < node1->neighbours.end(); i++)
   {
      if(*i == node2)
        return;
      Segment s(node1->pt, (*i)->pt));
      if (s.PointOnLine(node2->pt, MIN_DIST_MAP_POINTS))
      {
         if (s.PointOnSegment(node2->pt)
         {
           (*i)->neighbours.erase(node1);
           (*i)->neighbours.push_back(node2);    
           node2->neighbours.push_back(i);    
           node1->neighbours.erase(i);
           node1->neighbours.push_back(node2);    
           node2->neighbours.push_back(node1);    
           return;
         }
         else
           AddNeighbour(node2, node1);
      }  
   }       
}
*/
 

void HouseGraph::DeleteNeighbour(HouseGraphNode* node, const Position2D &pos)        
{
   std::vector<HouseGraphNode*>::iterator i;
   for(i = node->neighbours.begin(); i < node->neighbours.end(); i++)
   {
      if(Distance((*i)->pt, pos) < MIN_DIST_MAP_POINTS)
      {
         node->neighbours.erase(i);
         return;
      }
   }       
}
	

void HouseGraph::PrintNode(const HouseGraphNode &node) const
{
   printf("Node %d: ", node.pt.Number());
   std::vector<HouseGraphNode*>::const_iterator i;
   for(i = node.neighbours.begin(); i < node.neighbours.end(); i++)
   {
      printf("%d, ", (*i)->pt.Number());
   }
   printf("\n");     
}


void HouseGraph::PrintNode(const PointNumber &no) const
{
   std::vector<HouseGraphNode*>::const_iterator g;      
   for(g = begin(); g < end(); g++)
   {
      if ((*g)->pt == no)
      {  
         PrintNode(**g);
         break;
      }
   }
}




void HouseGraph::Print() const
{
   std::vector<HouseGraphNode*>::const_iterator g;      
   for(g = begin(); g < end(); g++)
   {
      printf("Node: %d,  neighbours:", (*g)->pt.Number());
      std::vector<HouseGraphNode*>::const_iterator n;
      for(n = (*g)->neighbours.begin(); n < (*g)->neighbours.end(); n++)
         printf("%d, ", (*n)->pt.Number());
      printf("\n");
   }   	 
}


bool SameRectangles(const ObjectPoints2D &rect1, const ObjectPoints2D &rect2)
{
   if (rect1.size() != rect2.size())
      return false;

   ObjectPoints2D::const_iterator j;  
   for(j = rect2.begin(); j < rect2.end(); j++)
         if (!Member(*j, rect1))
           return false;
   return true;
}


bool MemberList(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list)
{   
   if (rect_list.empty())
      return false;
      
   std::vector<ObjectPoints2D>::const_iterator i;
   for(i = rect_list.begin(); i < rect_list.end(); i++)
   {
      if (SameRectangles(*i, rect))
           return true;
   }
   
   return false;                
}        


int MemberList3(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list)
{   
   if (rect_list.empty())
      return -1;
      
   std::vector<ObjectPoints2D>::const_iterator i;
   ObjectPoints2D::const_iterator j;
   int no = 0;
   
   for(i = rect_list.begin(); i < rect_list.end(); i++)
   {
      no = 0;
      for(j = rect.begin(); j < rect.end(); j++)
      {
         if (Member(*j, *i))
         {
           no++;
           if (no >= 3)
             return i - rect_list.begin();
         }  
      }   
   }
   
   return -1;                
}        



bool MemberSubList(const ObjectPoints2D &rect, const std::vector<ObjectPoints2D> &rect_list)
{
   std::vector<ObjectPoints2D>::const_iterator i;
   ObjectPoints2D::const_iterator j;
   bool found = true;
   
   for(i = rect_list.begin(); i < rect_list.end(); i++)
   {
      found = true;
      for(j = rect.begin(); j < rect.end(); j++)
      {
         if (!Member(*j, *i))
         {
           found = false;
           break;
         }  
      }   
      if (found)
        return true;
   }
   return false;                
}        


bool Member(const ObjectPoint2D &node, const ObjectPoints2D &rect)
{
   ObjectPoints2D::const_iterator j;
   for(j = rect.begin(); j < rect.end(); j++)
      if(j->Number() == node.Number())
        return true;
   return false;
}        


