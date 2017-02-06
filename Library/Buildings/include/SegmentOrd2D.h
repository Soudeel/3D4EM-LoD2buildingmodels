
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



#ifndef _SegmentOrd2D_h_
#define _SegmentOrd2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  SegmentOrd2D  
  
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
#include <vector.h>
#endif
#include "Segment2D.h"

/*
--------------------------------------------------------------------------------
                                    SegmentOrd2D
--------------------------------------------------------------------------------
*/


class SegmentOrd2D : public Segment2D
{
		
public:
	/// Default constructor
	SegmentOrd2D()		
	  {};
			  
	/// Constructor - a segment determined by 2 points			
	SegmentOrd2D(const Position2D &pos1, const Position2D &pos2);
	
	/// Construct by converting a LineTopology structure	
	SegmentOrd2D(const LineTopology &lin_top, const ImagePoints &lin_pts);
		
	/// Copy assignment  			
	SegmentOrd2D& operator=(const SegmentOrd2D&);  
	  
	/// Destructor  
	~SegmentOrd2D() 
	  {};     
	  
	const SegmentOrd2D &segm() const
	  { return *this; }  
	      
        /// Get the min dist between a point and the endpoints of the segment
        double MinDistToEndPoints(const Position2D &pos) const;
        
        // ??????
        /// Det the position of the point at the given distance from the closest 
        /// endpoint to 'pos'
        Position2D PointAtDistance(const Position2D &pos, int dist) const; 
        
};


class SegmentsOrd2D : public std::vector<SegmentOrd2D>
{
public:
	/// Default constructor 
	SegmentsOrd2D() 
	  {};

	/// Construct by converting a LineTopologies structure	
	SegmentsOrd2D(const LineTopologies &lin_top, const ImagePoints &lin_pts);
	
	/// Construct by converting a LineTopology structure	
	SegmentsOrd2D(const LineTopology &lin_top, const ImagePoints &lin_pts);
	
	/// Destructor
	~SegmentsOrd2D()
	  {};
	  
	/// Copy assignment   
	SegmentsOrd2D& operator=(const SegmentsOrd2D &seg);   			

	/// Convert to a LineTopologies structure	  
	void ConvertToLineTops(LineTopologies &lin_top, ImagePoints &lin_pts) const;
	
	double Length() const;
	
	/// Write into files - a line topologies file and an object point file
	void Write(char *file_name) const;
};	             



#endif /* _SegmentOrd2D_h_ */   /* Do NOT add anything after this line */
