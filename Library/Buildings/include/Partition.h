
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



#ifndef _Partition_h
#define _Partition_h

#include "House.h"


class Partition : public ObjectPoints2D, public FeatureNumber
{   
   
public:

   /// Default constructor	
   Partition()
        {} ;
   
   /// Constructor
   Partition(const ObjectPoints2D &pts, FeatureNumber n)
   	: ObjectPoints2D(pts), FeatureNumber(n) {};   	
   	
   /// Copy constructor
   Partition(const Partition &p)
        { *this = p; }

   /// Destructor	
   ~Partition() 
   	{};
   	
   /// Asignment operator	
   Partition& operator=(const Partition &p);	
   
   /// Returns a constant reference to the object
   const Partition& PartitionRef() const
      { return *this; }
      
   /// Returns a reference to the object   
   Partition& PartitionRef()
      { return *this; }
   
   /// Test if a point belongs to the partition   
   int Member(const PointNumber &node) const;  
         
   /// Rotate the partition such that the given corner becomes the first one   
   /// Returns true if the corner is found, false otherwise 
   bool Rotate(const PointNumber &pt);
   
   /// Revert the points  
   void Reverse();       
   	   	   	 
   /// Print	   	   	              	
   void Print() const;	  
   
   /// Equality operator
   friend bool operator==(const Partition &p1, const Partition &p2);   		
};



class PartitionScheme : public vector<Partition>
{

public:
   
   /// Default constructor
   PartitionScheme() 
      {};
   
   /// Destructor
   ~PartitionScheme() 
      {};
   
   /// Asignment operator	
   PartitionScheme& operator=(const PartitionScheme& ps);   
   
   /// Returns a constant reference to the object   
   const PartitionScheme& PartitionSchemeRef() const
      { return *this; }
   
   /// Returns a reference to the object   
   PartitionScheme& PartitionSchemeRef()
      { return *this; }
   
   /// Test if a partition is included in the list of partitions      
   bool Member(const Partition &p) const;   
   
   /// subtract 2 partition schemes
   PartitionScheme& operator-=(const PartitionScheme& ps);   
    
   /// Print            
   void Print() const;   

   /// Equality operator   
   friend bool operator==(const PartitionScheme &p1, const PartitionScheme &p2);   		   
};


class VectorRectPairs;

class PartitionSchemes : public vector<PartitionScheme>
{
  int no_max;
  
public:
   /// Default constructor
   PartitionSchemes(int no = 0) 
      { no_max = no; }
   
   /// Constructor
   PartitionSchemes(const House &house);	  

   /// Destructor
   ~PartitionSchemes() 
      {};
   
   /// Asignment operator	   
   PartitionSchemes& operator=(const PartitionSchemes& ps);   

   /// Returns a constant reference to the object   
   const PartitionSchemes& PartitionSchemesRef() const
      { return *this; }

   /// Returns a reference to the object         
   PartitionSchemes& PartitionSchemesRef()
      { return *this; }                  
   
   /// Add partition scheme to the list only if it isn't contained in the list
   void Add(const PartitionScheme &ps);     
   
   /// Sort based on the length of the ps
   void SortMDL();
   
   /// Print
   void Print() const;   
      
   void MergePartitions(const vector<ObjectPoints2D> &rect_list); 
   void MergePartitionsRec(const PartitionScheme &rect_list, 
   		PartitionSchemes &ps_list, const VectorRectPairs &rect_pairs);    		   
};

#endif   	
    
