
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



#include "PartitioningRules.h"

#define ERR_POS		0.3

int PartitioningRules::no_new_corner = 0;

void PartitioningRules::ConditionRule1()
{
   // look for collinear segments
   std::vector<Segment>::const_iterator s1, s2, s3;   
   int k1 = 0, k2;
   for(s1 = map_seg.begin(); s1 < map_seg.end() - 1; s1++, k1++)
     for(s2 = s1 + 2, k2 = k1 + 2; s2 < map_seg.end(); s2++, k2++)      
      if (s2->PointOnLine(s1->EndPoint(0), 0.5) && 
		    !s2->EndPoint(s1->EndPoint(0)) 
       		&& ParallelLines(*s1, *s2, ERR_PARALLEL_LINES)) 
      {
        Segment ss;
        GapCollinearSegments(*s1, *s2, ss); 
        if (SegmentIntersectHouse(ss) == -1)
        { 
           Position2D posm;
	       posm = ss.MiddlePoint();
           if (PointInsideHouse(posm))
           { 
              // check if there is no other collin seg between s1 and s2
              bool ok = true;
              for(s3 = s1 + 2; s3 < map_seg.end(); s3++)
              {
                 Segment ss3;
                 if (s3 != s2 && ss.PointOnLine(s3->EndPoint(0), 0.5) 
         	    && ParallelLines(ss, *s3, ERR_PARALLEL_LINES) 
		    && IntersectionCollinearSegments(ss, *s3, ss3))
                 {
                    ok = false;
                    break;
                 }
              }    
             if (ok)
             {
               printf("rule1:  %d, %d\n", k1, k2);
               rules1.push_back(Rule(1, k1, k2));               
             } 
           }
         }
      }
}


void PartitioningRules::ConditionRule2()
{
   // look for internal intersection points	      
   int kk1 = 0, kk2 = 0, k1, k2;
   Position2D pos1, pos2;
   std::vector<Segment>::const_iterator s1, s2;
   for(s1 = map_seg.begin(); s1 < map_seg.end() - 1; s1++, kk1++)
   {
      double min1 = 1000000, min2 = 1000000;
      for(s2 = s1 + 4, kk2 = kk1 + 4; s2 < map_seg.end(); s2++, kk2++)      
       if (Intersection2Lines(*s1, *s2, pos1) &&
                Distance(s1->EndPoint(0), pos1) > ERR_POS && 
                Distance(s1->EndPoint(1), pos1) > ERR_POS &&
        	Distance(s2->EndPoint(0), pos1) > ERR_POS && 
                Distance(s2->EndPoint(1), pos1) > ERR_POS &&
                !s1->PointOnSegment(pos1) && 
		!s2->PointOnSegment(pos1) &&              
                PointInsideHouse(pos1))    
      {        
 if (kk1 == 6 && kk2 == 15)
 {
    double a1 =  Distance(s1->EndPoint(1), pos1);
    double a2 =  Distance(s2->EndPoint(0), pos1);    
    printf("a1=%f, a2=%f\n", a1, a2);
    printf("pos1: %f, %f\n", pos1.X(), pos1.Y());
    printf("pos90: %f, %f\n", s1->EndPoint(1).X(), s1->EndPoint(1).Y());
  }      
        Segment ss1;
        double d1 = Distance(s1->EndPoint(0), pos1);
        double d = Distance(s1->EndPoint(1), pos1);
        if (d1 < d)
           ss1 = Segment(pos1, s1->EndPoint(0));
        else
        {
           d1 = d;
           ss1 = Segment(pos1, s1->EndPoint(1));
        }
        if ((d1 < min1 || fabs(d1 - min1) < ERR_POS) && (SegmentIntersectHouse(ss1) == -1))
        { 
          Segment ss2;
          double d2 = Distance(s2->EndPoint(0), pos1);
          d = Distance(s2->EndPoint(1), pos1);
          if (d1 < d)
             ss2 = Segment(pos1, s2->EndPoint(0));
          else
          {
             d2 = d;
             ss2 = Segment(pos1, s2->EndPoint(1));
          }
         if ((d2 < min2 || fabs(d2 - min2) < ERR_POS) && SegmentIntersectHouse(ss1) == -1)
         { 
         //Position2D posm;
	 //posm = s1->PointAtDistance(pos1, 1);
         //if (PointInsideHouse(posm))
         //{ 
           if (Intersection2Lines(*(s1 + 1), *(s2 - 1), pos2) &&
                Distance(s1->EndPoint(0), pos2) > ERR_POS && 
                Distance(s1->EndPoint(1), pos2) > ERR_POS &&
        	Distance(s2->EndPoint(0), pos2) > ERR_POS && 
                Distance(s2->EndPoint(1), pos2) > ERR_POS &&
                !(s1 + 1)->PointOnSegment(pos2) && 
		!(s2 - 1)->PointOnSegment(pos2) &&               
                PointInsideHouse(pos2))    
            {
              Segment ss3, ss4;
              if (Distance((s1 + 1)->EndPoint(0), pos2) < 
				Distance((s1 + 1)->EndPoint(1), pos2))
                 ss3 = Segment(pos2, (s1 + 1)->EndPoint(0));
              else
                 ss3 = Segment(pos2, (s1 + 1)->EndPoint(1));

              if (Distance((s2 - 1)->EndPoint(0), pos2) < 
				Distance((s2 - 1)->EndPoint(1), pos2))
                 ss4 = Segment(pos2, (s2 - 1)->EndPoint(0));
              else
                 ss4 = Segment(pos2, (s2 - 1)->EndPoint(1));

	      //posm = (s1 + 1)->PointAtDistance(pos2, 1);
              //if (PointInsideHouse(posm))
              if (SegmentIntersectHouse(ss3) == -1 && SegmentIntersectHouse(ss4) == -1) 
              {
                 std::vector<Segment> collin_segs1, collin_segs2;
                 std::vector<Segment>::const_iterator l;
                 bool ok = true; 
                 if (CollectCollinearSegments(*s1, collin_segs1))
                 {
                    // select the closest segment to pos1; 
	            for(l = collin_segs1.begin(); l < collin_segs1.end(); l++)
                       if (ss1.PointOnSegment(l->EndPoint(0)))
                       {
                           ok = false;
                           break;
                       }   
                 }
                 if (ok && CollectCollinearSegments(*s2, collin_segs2))
                 {
                    // select the closest segment to pos1; 
	            for(l = collin_segs2.begin(); l < collin_segs2.end(); l++)
                       if (ss2.PointOnSegment(l->EndPoint(0)))
                       {
                           ok = false;
                           break;
                       }   
                 }
                 if (ok)
                 {    
                   min1 = d1; min2 = d2;
                   k1 = kk1; k2 = kk2;
                 }
               }
            }
         }}
      } 
    if (min2 != 1000000)
    {
        double d1 = max(Distance(s1->EndPoint(0), pos1), 
			Distance(s1->EndPoint(1), pos1));
        double d2 = max(Distance(s2->EndPoint(0), pos1), 
			Distance(s2->EndPoint(1), pos1));
        if (k1 > 0)
           d2 = max(d2, (s1 - 1)->Length());
        double area1 = d1 * d2;
        d1 = max(Distance((s1 + 1)->EndPoint(0), pos2), 
			Distance((s1 + 1)->EndPoint(1), pos2));
        d1 = max(d1, (s2 - 1)->Length());
        d2 = max(Distance((s2 - 1)->EndPoint(0), pos2), 
			Distance((s2 - 1)->EndPoint(1), pos2));
        double area2 = d1 * d2;

          printf("rule2  %d, %d\n", k1, k2);
        if (area1 > area2)
	{    
          rules2.push_back(Rule(2, k1, k2));
//          printf("rule2  %d, %d\n", k1, k2);
	}
	else  
	{    
          rules2.push_back(Rule(2, k1 + 1, k2 - 1));
  //        printf("rule2  %d, %d\n", k1 + 1, k2 - 1);
	}
    }
  }     
}    


// deletes the conflicting rules from the list of rules
void PartitioningRules::CheckConflicts()
{
  std::vector<Rule>::iterator i1, i2;
  i1 = rules1.begin(); 
  while(i1 < rules1.end())
  {
    bool del = false;
    if (rules2.empty())
      return;
    i2 = rules2.begin(); 
    while(i2 < rules2.end() && !del)
    {
      if (i1->ind_seg1 == i2->ind_seg1 || i1->ind_seg2 == i2->ind_seg2 || 
	  i1->ind_seg1 == i2->ind_seg1 + 1 || i1->ind_seg2 == i2->ind_seg2 - 1) 
      {	  
          printf("Conflict:");
	  i1->Print();
	  i2->Print();
          Conflict c;
	  c.rule1 = *i1;
	  c.rule2 = *i2;
	  conflicts.push_back(c);
	  i2 = rules2.erase(i2);
	  del = true;
      }
    }  
    if (del)
      i1 = rules1.erase(i1);
  }
}


void PartitioningRules::PartitioningRule1(std::vector<House> &house_parts, 
			 std::vector<CollinSegs> &collin_segs)
{
   if (size() < 9 || collin_segs.empty())
   {
       house_parts.push_back(*this);
       return;
   }
   
   if (map_seg.empty())
      GetHouseSegments(map_seg);

   Segment ss1 = collin_segs.begin()->seg1;
   Segment ss2 = collin_segs.begin()->seg2;
   collin_segs.erase(collin_segs.begin());
  
   // look for s1
   std::vector<Segment>::const_iterator s1;   
   int k1 = -1, k2 = -1;
   bool intersect_ss1 = false, intersect_ss2 = false;
   ObjectPoint2D pt1, pt2;
   for(s1 = map_seg.begin(); s1 < map_seg.end(); s1++)
   {
      double dd = Distance(s1->EndPoint(0), ss1.EndPoint(0));
      int b = s1->PointOnSegment(ss1.EndPoint(1), 1, ERR_POS);

      if ((*s1 == ss1) ||
         (Distance(s1->EndPoint(0), ss1.EndPoint(0)) < ERR_POS &&
            s1->PointOnSegment(ss1.EndPoint(1), 1, ERR_POS)) ||
         (Distance(s1->EndPoint(1), ss1.EndPoint(1)) < ERR_POS &&
            s1->PointOnSegment(ss1.EndPoint(0), 1, ERR_POS)) ||
         (s1->PointOnSegment(ss1.EndPoint(0), 1, ERR_POS) && 
           s1->PointOnSegment(ss1.EndPoint(1), 1, ERR_POS))) 
      {
          k1 = s1 - map_seg.begin();
          break;
      }
   }

   if (k1 == -1 && (k1 = SegmentIntersectHouse(ss1)) != -1)
   {
      intersect_ss1 = true;
      Position2D pos;
      Intersection2Lines(ss1, map_seg[k1], pos);
      pt1 = ObjectPoint2D(pos, PointNumber(no_new_corner++), Covariance2D());
   }

   if (k1 == -1)
   {
      // s1 was not found
      PartitioningRule1(house_parts, collin_segs);
      return;
   } 

   for(s1 = map_seg.begin(); s1 < map_seg.end(); s1++)
   {
      if ((*s1 == ss2) || 
         (Distance(s1->EndPoint(0), ss2.EndPoint(0)) < 0.2 &&
            s1->PointOnSegment(ss2.EndPoint(1), 1, 0.2)) ||
         (Distance(s1->EndPoint(1), ss2.EndPoint(1)) < 0.2 &&
            s1->PointOnSegment(ss2.EndPoint(0), 1, 0.2)) ||
         (s1->PointOnSegment(ss2.EndPoint(0), 1, 0.2) && 
           s1->PointOnSegment(ss2.EndPoint(1), 1, 0.2))) 
      {
          k2 = s1 - map_seg.begin();
          break;
      }
   }   

   if (k2 == -1 && (k2 = SegmentIntersectHouse(ss2)) != -1)
   {
      intersect_ss2 = true;
      Position2D pos;
      Intersection2Lines(ss2, map_seg[k2], pos);
      pt2 = ObjectPoint2D(pos, PointNumber(no_new_corner++), Covariance2D());
   }

   if (k2 == -1)
   {
      PartitioningRule1(house_parts, collin_segs);
      return;
   } 

   if (k1 > k2)
   {
      int tmp = k1; k1 = k2; k2 = tmp;
      bool b = intersect_ss1; intersect_ss1 = intersect_ss2; 
      intersect_ss2 = b;
      ObjectPoint2D pt = pt1; pt1 = pt2; pt2 = pt;
   }   
                  
   printf("rule1  %d, %d\n", k1, k2);
   PartitioningRules::const_iterator i;
   int k = 0; 
   PartitioningRules house1, house2;
   for(i = begin(); i < end() - 1; i++, k++)
   {
       if (k <= k1 || k > k2)
                  house1.push_back(*i);
       if (k > k1 && k <= k2)
                  house2.push_back(*i);
       if (k == k1 && intersect_ss1)
       {
          house1.push_back(pt1);
          house2.push_back(pt1); 
       } 
       if (k == k2 && intersect_ss2)
       {
          house1.push_back(pt2);
          house2.push_back(pt2); 
       } 
   }              
   house1.push_back(*house1.begin());
   house2.push_back(*house2.begin());
   house1.Print();
   house2.Print();
   std::vector<CollinSegs> tmp_collin_segs;
   tmp_collin_segs.insert(tmp_collin_segs.begin(), collin_segs.begin(), collin_segs.end());
   house1.PartitioningRule1(house_parts, collin_segs);
   house2.PartitioningRule1(house_parts, tmp_collin_segs);       
}



void PartitioningRules::ApplyRules12(std::vector<House> &house_parts)
{
   if (size() < 8)
   {
      house_parts.push_back(*this);
      return;
   }
       
   ConditionRule1();      
   std::vector<House> house_parts1;   
   if (!rules1.empty())
   {
      std::vector<CollinSegs>  collin_segs;
      std::vector<Rule>::const_iterator i;
      for(i = rules1.begin(); i < rules1.end(); i++)
         collin_segs.push_back(
	 	CollinSegs(map_seg[i->ind_seg1], map_seg[i->ind_seg2]));	 
      PartitioningRule1(house_parts1, collin_segs);
   }   
   
   std::vector<House>::const_iterator h;
   printf("________________________________\n");
   for(h = house_parts1.begin(); h < house_parts1.end(); h++)
         h->Print(); 
 
   for(h = house_parts1.begin(); h < house_parts1.end(); h++)
   {
       PartitioningRules pr(*h);
       pr.PartitioningRule2(house_parts);
   }    
   printf("________________________________\n");
   for(h = house_parts.begin(); h < house_parts.end(); h++)
         h->Print(); 

}     



void PartitioningRules::PartitioningRule1(std::vector<House> &house_parts)
{
   if (size() < 8)
   {
      house_parts.push_back(*this);
      return;
   }
       
   ConditionRule1();      
   if (!rules1.empty())
   {
      std::vector<CollinSegs>  collin_segs;
      std::vector<Rule>::const_iterator i;
      for(i = rules1.begin(); i < rules1.end(); i++)
         collin_segs.push_back(
	 	CollinSegs(map_seg[i->ind_seg1], map_seg[i->ind_seg2]));	 
      PartitioningRule1(house_parts, collin_segs);
   }   
   
   std::vector<House>::const_iterator h;
   printf("________________________________\n");
   for(h = house_parts.begin(); h < house_parts.end(); h++)
         h->Print();  
}     



void PartitioningRules::PartitioningRule2(std::vector<House> &house_parts) 
{
   if (size() < 9)
   {
       house_parts.push_back(*this);
       return;
   }
   
   if (map_seg.empty())
      GetHouseSegments(map_seg);
  
   // look for internal intersection points	      
   int kk1 = 0, kk2 = 0, k1, k2;
   Position2D pos1, pos2;
   std::vector<Segment>::const_iterator s1, s2;
   for(s1 = map_seg.begin(); s1 < map_seg.end() - 1; s1++, kk1++)
   {
      double min1 = 1000000, min2 = 1000000;
      for(s2 = s1 + 4, kk2 = kk1 + 4; s2 < map_seg.end(); s2++, kk2++)      
       if (Intersection2Lines(*s1, *s2, pos1) &&
                Distance(s1->EndPoint(0), pos1) > ERR_POS && 
                Distance(s1->EndPoint(1), pos1) > ERR_POS &&
        	Distance(s2->EndPoint(0), pos1) > ERR_POS && 
                Distance(s2->EndPoint(1), pos1) > ERR_POS &&
                !s1->PointOnSegment(pos1) && 
		!s2->PointOnSegment(pos1) &&              
                PointInsideHouse(pos1))    
      {        
        Segment ss1;
        double d1 = Distance(s1->EndPoint(0), pos1);
        double d = Distance(s1->EndPoint(1), pos1);
        if (d1 < d)
           ss1 = Segment(pos1, s1->EndPoint(0));
        else
        {
           d1 = d;
           ss1 = Segment(pos1, s1->EndPoint(1));
        }
        if ((d1 < min1 || fabs(d1 - min1) < ERR_POS) && (SegmentIntersectHouse(ss1) == -1))
        { 
          Segment ss2;
          double d2 = Distance(s2->EndPoint(0), pos1);
          d = Distance(s2->EndPoint(1), pos1);
          if (d1 < d)
             ss2 = Segment(pos1, s2->EndPoint(0));
          else
          {
             d2 = d;
             ss2 = Segment(pos1, s2->EndPoint(1));
          }
         if ((d2 < min2 || fabs(d2 - min2) < ERR_POS) && SegmentIntersectHouse(ss1) == -1)
         { 
         //Position2D posm;
	 //posm = s1->PointAtDistance(pos1, 1);
         //if (PointInsideHouse(posm))
         //{ 
           if (Intersection2Lines(*(s1 + 1), *(s2 - 1), pos2) &&
                Distance(s1->EndPoint(0), pos2) > ERR_POS && 
                Distance(s1->EndPoint(1), pos2) > ERR_POS &&
        	Distance(s2->EndPoint(0), pos2) > ERR_POS && 
                Distance(s2->EndPoint(1), pos2) > ERR_POS &&
                !(s1 + 1)->PointOnSegment(pos2) && 
		!(s2 - 1)->PointOnSegment(pos2) &&               
                PointInsideHouse(pos2))    
            {
              Segment ss3, ss4;
              if (Distance((s1 + 1)->EndPoint(0), pos2) < 
				Distance((s1 + 1)->EndPoint(1), pos2))
                 ss3 = Segment(pos2, (s1 + 1)->EndPoint(0));
              else
                 ss3 = Segment(pos2, (s1 + 1)->EndPoint(1));

              if (Distance((s2 - 1)->EndPoint(0), pos2) < 
				Distance((s2 - 1)->EndPoint(1), pos2))
                 ss4 = Segment(pos2, (s2 - 1)->EndPoint(0));
              else
                 ss4 = Segment(pos2, (s2 - 1)->EndPoint(1));

	      //posm = (s1 + 1)->PointAtDistance(pos2, 1);
              //if (PointInsideHouse(posm))
              if (SegmentIntersectHouse(ss3) == -1 && SegmentIntersectHouse(ss4) == -1) 
              {
                 std::vector<Segment> collin_segs1, collin_segs2;
                 std::vector<Segment>::const_iterator l;
                 bool ok = true; 
                 if (CollectCollinearSegments(*s1, collin_segs1))
                 {
                    // select the closest segment to pos1; 
	            for(l = collin_segs1.begin(); l < collin_segs1.end(); l++)
                       if (ss1.PointOnSegment(l->EndPoint(0)))
                       {
                           ok = false;
                           break;
                       }   
                 }
                 if (ok && CollectCollinearSegments(*s2, collin_segs2))
                 {
                    // select the closest segment to pos1; 
	            for(l = collin_segs2.begin(); l < collin_segs2.end(); l++)
                       if (ss2.PointOnSegment(l->EndPoint(0)))
                       {
                           ok = false;
                           break;
                       }   
                 }
                 if (ok)
                 {    
                   min1 = d1; min2 = d2;
                   k1 = kk1; k2 = kk2;
                 }
               }
            }
           //}
         }}
      } 
    if (min2 != 1000000)
    {
        double d1 = max(Distance(s1->EndPoint(0), pos1), 
			Distance(s1->EndPoint(1), pos1));
        double d2 = max(Distance(s2->EndPoint(0), pos1), 
			Distance(s2->EndPoint(1), pos1));
        if (k1 > 0)
           d2 = max(d2, (s1 - 1)->Length());
        double area1 = d1 * d2;
        d1 = max(Distance((s1 + 1)->EndPoint(0), pos2), 
			Distance((s1 + 1)->EndPoint(1), pos2));
        d1 = max(d1, (s2 - 1)->Length());
        d2 = max(Distance((s2 - 1)->EndPoint(0), pos2), 
			Distance((s2 - 1)->EndPoint(1), pos2));
        double area2 = d1 * d2;

        printf("rule2  %d, %d\n", k1, k2);
        PartitioningRules::const_iterator i;
        int k = 0; 
        PartitioningRules house1, house2;	

        if (area1 > area2)
	{
          for(i = begin(); i < end() - 1; i++, k++)
          {
            if (k <= k1 || k > k2)
              house1.push_back(*i);
            if (k == k1)
                 house1.push_back(ObjectPoint2D(pos1, 
			PointNumber(no_new_corner), Covariance2D()));  
            if (k > k1 && k <= k2)
               house2.push_back(*i);
            if (k == k2)
                house2.push_back(ObjectPoint2D(pos1, 
			PointNumber(no_new_corner++), Covariance2D()));  
          }
        }
        else 
	{
          for(i = begin(); i < end() - 1; i++, k++)
          {

            if (k <= k1 + 1 || k >= k2)
              house1.push_back(*i);
            if (k == k1 + 1)
                 house1.push_back(ObjectPoint2D(pos2, 
			PointNumber(no_new_corner), Covariance2D()));  
            if (k >= k1 + 1 && k < k2)
               house2.push_back(*i);
            if (k == k2)
                house2.push_back(ObjectPoint2D(pos2, 
			PointNumber(no_new_corner++), Covariance2D()));  
          }
        }
       house1.push_back(*house1.begin());
       house2.push_back(*house2.begin());
       house1.Print();
       house2.Print();
       house1.PartitioningRule2(house_parts);
       house2.PartitioningRule2(house_parts);       
       //break; 
       return; 
    }             
    } // for s1
    house_parts.push_back(*this); 
}



void PartitioningRules::Print() const
{
   std::vector<Rule>::const_iterator i;
   for(i = rules1.begin(); i < rules1.end(); i++)
      i->Print();
   for(i = rules2.begin(); i < rules2.end(); i++)
      i->Print();      
}      
