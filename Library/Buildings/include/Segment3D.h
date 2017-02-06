
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



#ifndef _Segment3D_h_
#define _Segment3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Segment3D  
  
  Segments3D
  
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
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "Line3D.h"
#include "Segment2D.h"
#include "Constants.h"

/*
--------------------------------------------------------------------------------
                                    Segment3D
--------------------------------------------------------------------------------
*/


class Segment3D : public Line3D
{

protected:

    Position3D pos1, pos2;
		
public:
	/// Default constructor 
	Segment3D()		
	  {} ;
			  
	/// Constructor - A segment determined by 2 points			  
	Segment3D(const Position3D &pos1, const Position3D &pos2);				
				        
	/// Construct by converting a LineTopology structure	
	Segment3D(const LineTopology &lin_top, const ObjectPoints &lin_pts);
		
	/// Copy assignament				        				
	Segment3D& operator=(const Segment3D&);    

	/// Destructor	  
	~Segment3D() 
	   {};			

	/// Constant reference to the object	
	const Segment3D &segm() const 
	   { return *this; }
			    
	/// Reference to the object		     		
        Segment3D &segm() 
           { return *this; }

	/// Convert to a LineTopology structure	             
        void ConvertToLineTop(LineTopology &lin_top, ObjectPoints &lin_pts) const;   
        		
	Segment2D Segm2D() const;
					     
        double Length() const;
                
        Position3D MiddlePoint() const;
        
        double AngleOxy() const; 
        
	const Position3D &EndPoint(int) const;
	Position3D &EndPoint(int);
	
	/// Find the distance of a point to the segment 
        double DistanceFromPoint(const Position3D &pos) const; 
        
        /// test whether the point is lying on the segment
        bool PointOnSegment(const Position3D &pos, double error) const; 
        
        void Print() const;

	/// test equality of two segments
	friend bool operator==(const Segment3D &seg1, const Segment3D &seg2); 	
        			     
        /// Angle of two segments			     
   	friend double Angle2Segments(const Segment3D &seg1, const Segment3D &seg2);	
   	
   	/// Intersection of 2 segments lying on the same line
   	friend bool CommonSegmentPart(const Segment3D &seg1, 
   			const Segment3D &seg2, Segment3D &seg);			        	
   	       	       
   	friend int CompareStartingPoints(const Segment3D &seg1, const Segment3D &seg2);
   		       /* Return value: -1 if s1 < s2
   		       		         0 if s1 = s2
   		       		         1 if s1 > s2                       */ 			              	       
   		       		                
	/// Test if two segments are parallel
        friend bool ParallelSegments(const Segment3D &s1, 
        		const Segment3D &s2, double error = ERR_PARALLEL_SEG);
        
        /// Test if two segments are collinear
        friend bool CollinearSegments(const Segment3D &s1, 
        		const Segment3D &s2, double error);
};


//typedef vector<Segment3D> Segments3D;
class Segments3D : public std::vector<Segment3D>
{
public:
	/// Default constructor 
	Segments3D() 
	  {};

	/// Construct by converting a LineTopologies structure	
	Segments3D(const LineTopologies &lin_top, const ObjectPoints &lin_pts);
	
	/// Destructor
	~Segments3D()
	  {
	     if (!empty())
	        erase(begin(), end());
	  }

	/// Convert to a LineTopologies structure	  
	void ConvertToLineTops(LineTopologies &lin_top, ObjectPoints &lin_pts) const;
	
	/// Write into files - a line topologies file and an object point file
	void Write(char *file_name) const;
};	             


#endif /* _Segment3D_h_ */   /* Do NOT add anything after this Segment */
