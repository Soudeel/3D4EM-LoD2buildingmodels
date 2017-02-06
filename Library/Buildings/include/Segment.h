
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



#ifndef _Segment_h_
#define _Segment_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  Segment
      
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Line2D.h"


bool CollinearPoints(const Position2D &pos1, const Position2D &pos2, 
	const Position2D &pos3, double err_dist);


/*
--------------------------------------------------------------------------------
                                    Segment
--------------------------------------------------------------------------------
*/

class Segment : public Line2D
{
protected:
   
   Position2D 	pos1, pos2;
   
public:
	/// Default constructor             
	Segment()	   
	  {};
	  
	/// Constructor  
	Segment(const Position2D &pt1, const Position2D &pt2)
		: Line2D(pt1, pt2)
		{ pos1 = pt1; pos2 = pt2; }
	
	/// Destructor
	~Segment()               
	  {};
	   
	/// Copy assignment          				
	Segment& operator=(const Segment &s)
	  { lin() = s.lin();  pos1 = s.pos1; pos2 = s.pos2; 
            return *this; 
          } 
	
	/// Get reference
	Segment& SegRef()
	  { return *this; }
	
	/// Get constant reference
	const Segment& SegRef() const
	  { return *this; }   

	/// Return endpoint
	const Position2D& EndPoint(int i) const 
	  { return ((i == 0) ? pos1 : pos2); }  

        /// Check if the specified point is one of the end points of the seg 
	bool EndPoint(const Position2D &pt) const
	  { return pt == pos1 || pt == pos2; }
	  
	/// Return length
	double Length() const
	  { return Distance(pos1, pos2); }  

	/// Position of the middle point 
	Position2D MiddlePoint() const;

        /// check if a point is on the segment
        /// if online == 1 test also if the point is on the line
        bool PointOnSegment(const Position2D &pt, 
			bool online = 0, double error = 0.1) const;

        /// Det the position of the point at the given distance from the closest 
        /// endpoint to 'pos'
        Position2D PointAtDistance(const Position2D &pos, int dist) const; 

    /// Test if 2 segments intersect each other
    /// if onseg == 1 test also if the intersection point is on both segments
    friend bool Intersection2Segments(const Segment &s1, const Segment &s2, 
					double err_dist, Position2D &pos, bool onseg = 1);

	/// Test if 2 segments are parallel
    friend bool ParallelSegments(const Segment &s1, const Segment &s2, 
					double err_dist);

	/// Test if 2 segments are collinear
    friend bool CollinearSegments(const Segment &s1, const Segment &s2, 
					double err_dist);
       
	/// Find union of two collinear segments 
	/// return the length of the union
	friend double UnionCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_union);

	/// Find intersection of two collinear segments 
	friend bool IntersectionCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_intersect);

	/// Find gap between two collinear segments 
    /// returns true if there is gap 
	friend bool GapCollinearSegments(const Segment &s1, const Segment &s2,
			Segment &seg_gap);

    friend bool operator==(const Segment &s1, const Segment &s2);
};


#endif /* _Roof_h_ */   /* Do NOT add anything after this line */
