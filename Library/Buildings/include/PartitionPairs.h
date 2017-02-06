
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



#ifndef _PartitionPairs_h
#define _PartitionPairs_h

#include "Partition.h"


struct RectPair
{
   Partition p1;
   Partition p2;
   Partition p12;
   
   PartitionScheme parts;
   
   RectPair(const Partition &pp1, const Partition &pp2, const Partition &pp)
      : p1(pp1), p2(pp2), p12(pp) {} ;      
};

   
class VectorRectPairs : public vector<RectPair>
{
   
public:
   VectorRectPairs() {};
   
   /// generate all possible complex partitions
   void GenerateAllRectangles(const PartitionScheme &rect_list);
   
   /// generate complex partitions by merging partitions from list1 with
   /// partitions from list2
   /// returns false if no complex partition can be generated		
   bool GenerateRectangles(const PartitionScheme &rect_list1, 
   	const PartitionScheme &rect_list2, PartitionScheme &rect_list);   		   

   /// merge two partitions; returns true for success	
   bool MergeRectangles(const Partition &rect1, const Partition &rect2, 
		Partition &rect);			

   /// test if there is a partition in the list of composed partitions	  
   bool Member(const Partition &rect) const;
   
   /// test whether is a composed partition formed from the given 2 partitions	
   bool Member(const Partition &rect1, const Partition &rect2, Partition &rect) const;		   
	  
   /// returns the partition pair which generated the composed partition
   /// works only with composed partitions: Number >= 100
   const RectPair& FindPartition(const Partition &rect) const;
   
   /// returns the primitive partitions of a complex partition		
   void GetSubPartitions(const Partition &rect, PartitionScheme &sub_parts) const;   
	
   /// returns a list of complex partitions which contains the given partition	
   void GetMergedPartitions(const Partition &rect, PartitionScheme &mp_list) const;

   /// check if the first partition is a subpartition of the second one
   bool CheckSubPartitions(const Partition &rect1, const Partition &rect2) const;   
   
   /// check if the two partitions have a common subpart
   bool CommonSubPartition(const Partition &rect1, const Partition &rect2) const;
   
   void Delete(const Partition &p);
   
   /// print
   void Print() const;
};     

#endif   	
