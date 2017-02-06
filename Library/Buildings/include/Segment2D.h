
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



#ifndef _Segment2D_h_
#define _Segment2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Segment2D  
  
  Segments2D
  
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
#include "ImagePoints.h"
#include "Line2D.h"
#include "Constants.h"

/*
--------------------------------------------------------------------------------
                                    Segment2D
--------------------------------------------------------------------------------
*/


class Segment2D : public Line2D
{

    Position2D pos1, pos2;
		
public:
	/// Default constructor
	Segment2D()		
	  {};
			  
	/// Constructor - a segment determined by 2 points			  
	Segment2D(const Position2D &pos1, const Position2D &pos2);
	
	/// Construct by converting a LineTopology structure	
	Segment2D(const LineTopology &lin_top, const ImagePoints &lin_pts);
		
	/// Copy assignment  			
	Segment2D& operator=(const Segment2D&);  
	  
	/// Destructor  
	~Segment2D() 
	  {};     
	  
	const Segment2D &segm() const
	  { return *this; }  
	
	/// Convert to a LineTopology structure	             
        void ConvertToLineTop(LineTopology &lin_top, ImagePoints &lin_pts) const;   
		
	/// Length of the segment			     
        double Length() const;
                
        /// Position of the middle point 
        Position2D MiddlePoint() const;
        
        /// Check if the specified point is one of the end points of the seg 
	bool EndPoint(const Position2D &pos) const
	  { return pos == pos1 || pos == pos2; }
	
	/// Return one of the endpoints
	const Position2D &EndPoint(int ind) const
	  { return ind == 0 ? pos1 : pos2; }
	
	/// Check if a point is lying on the segment (the point is on the line)
        bool PointOnSegment(const Position2D &pos) const; 
        
	/// Check if a the projection of a point is lying on the segment
        bool PointProjectionOnSegment(const Position2D &pos) const; 
        
        // ???????
        /// Get the projection point of a point on the segment
        /// Return true if the projection point is on the segment
        bool PointProjection(const Position2D &pos, Position2D &prj_pos) const;    
                
        // ?????        
        // Project a segment onto the segment
        void SegmentProjection(const Segment2D &s, Segment2D &prj_seg) const;   	
        
        /// Get the min dist between a point and the endpoints of the segment
        double MinDistToEndPoints(const Position2D &pos) const;
        
        // ??????
        /// Det the position of the point at the given distance from the closest 
        /// endpoint to 'pos'
        Position2D PointAtDistance(const Position2D &pos, int dist) const; 
        
        void Print() const;

	friend bool operator==(const Segment2D &seg1, const Segment2D &seg2);
        
        /// Intersection point of two segments 			     
  	friend bool Intersect2Segments(const Segment2D &seg1, const Segment2D &seg2, 
  			Position2D &pos);	
   	
   	/// Intersection of 2 segments lying on the same line 
   	friend bool CommonSegmentPart(const Segment2D &seg1, 
   			const Segment2D &seg2, Segment2D &seg);			        	

	/// Test if two collinear segments having a common point have the same direction
	friend bool DirectionCollinearSegments(const Segment2D &seg1, 
   			const Segment2D &seg2, const Position2D &pos);   			

	/// Test if two collinear segments have the same direction
	friend bool DirectionCollinearSegments(const Position2D &pt1, 
	                const Segment2D &seg1, const Position2D &pt2, 
   			const Segment2D &seg2);     			
};


class Segments2D : public std::vector<Segment2D>
{
public:
	/// Default constructor 
	Segments2D() 
	  {};

	/// Construct by converting a LineTopologies structure	
	Segments2D(const LineTopologies &lin_top, const ImagePoints &lin_pts);
	
	/// Construct by converting a LineTopology structure	
	Segments2D(const LineTopology &lin_top, const ImagePoints &lin_pts);
	
	/// Destructor
	~Segments2D()
	  {};
	  
	/// Copy assignment   
	Segments2D& operator=(const Segments2D &seg);   			

	/// Convert to a LineTopologies structure	  
	void ConvertToLineTops(LineTopologies &lin_top, ImagePoints &lin_pts) const;
	
	double Length() const;

	// ???	
	// create a list of segments from list which are collinear to s
	void ConnectCollinearSegments(const Segment2D &s, const Segments2D &list);

	// ???
	// create a list of segments from list which are collinear to the 
	// first segment of the list
	// list is updated - the segments which are added to the new list are
	// deleted
	void ConnectCollinearSegments(Segments2D &list);
	
	// add collinear segments in sequential order
	void AddSegment(const Segment2D &s);
	
	/// ????
	// test if the segments from the list are collinear to one of 
	// the two given segments
	bool TestCollinearSegments(const Segment2D &s1, const Segment2D &s2,
		double err_par, double err_col);
	
	/// Write into files - a line topologies file and an object point file
	void Write(char *file_name) const;
};	             



#endif /* _Segment2D_h_ */   /* Do NOT add anything after this Segment */
