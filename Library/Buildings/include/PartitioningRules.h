
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



#ifndef _PartitioningRules_h_
#define _PartitioningRules_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  PartitioningRules

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/
#ifdef __sgi
#include <vector.h>
#else
#include <vector>
#endif

#include "ObjectPoints2D.h"
#include "LineTopology.h"

#include "House.h"
#include "Segment.h"


//struct CollinSegs;


struct Rule{
   int no_rule;
   int ind_seg1;
   int ind_seg2;   
   
   Rule(int no = 0, int ind1 = -1, int ind2 = -1)
   	: no_rule(no), ind_seg1(ind1), ind_seg2(ind2) {};
	
   void Print() const
     { printf("Rule %d:  seg1 = %d, seg2 = %d\n", no_rule, ind_seg1, ind_seg2);}	
};


struct Conflict{
   Rule rule1;
   Rule rule2;
};   


struct CollinSegs
{
    Segment seg1;
    Segment seg2;

    CollinSegs(const Segment &s1, const Segment &s2)
		: seg1(s1), seg2(s2) {};
};


/*
--------------------------------------------------------------------------------
                          PartitioningRules
--------------------------------------------------------------------------------
*/


class PartitioningRules : public House
{
   std::vector<Segment> 	map_seg;   // list of house edges
   
   std::vector<Rule>	rules1;	   
   std::vector<Rule>	rules2;	   
   
   std::vector<Conflict>	conflicts;
   
   static int no_new_corner;
 
public:
        /// Default constructor   
	PartitioningRules() {}; 
	  
	/// Constructor  
	PartitioningRules(const House &h)
	   : House(h) 
	   { GetHouseSegments(map_seg); }  
	
	/// Destructor
	~PartitioningRules() {};

	/// Collect house segments which satisfy rule1	
	void ConditionRule1();
	
	/// Collect house segments which satisfy rule2		
	void ConditionRule2();
	
	void CheckConflicts();
	
	/// partition house based on partioning rules
	void ApplyRules12(std::vector<House> &house_parts);
			
    void Print() const;

	void PartitioningRule1(std::vector<House> &house_parts);

	void PartitioningRule1(std::vector<House> &house_parts, 
			std::vector<CollinSegs> &collin_segs);

	void PartitioningRule2(std::vector<House> &house_parts);
};





#endif /* _House_h_ */   /* Do NOT add anything after this line */
