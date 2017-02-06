
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



/*!
 * \file
 * \brief Line segment in a two-dimensional coordinate system
 *
 */
/*!
 * \class LineSegment2D
 * \ingroup Photogrammetry
 * \brief Line segment in a two-dimensional coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><t
r><td>
 *
 * \author        \G_Vosselman
 * \date          08-May-2003 (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef _LineSegment2D_h_
#define _LineSegment2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineSegment2D   - A segment of a 2D line
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position2D.h"
#include "Line2D.h"
#include "ObjectPoints2D.h"
#include "LineTopologies.h"

/*
--------------------------------------------------------------------------------
                                    LineSegment2D
--------------------------------------------------------------------------------
*/

class LineSegment2D : public Line2D, public LineNumber
{

    /// Scalar of begin point of line segment
    double scalar_begin;
    /// Scalar of end point of line segment
    double scalar_end;
    /// Label of the segment
    int label;
		
public:
	/// Default constructor
	LineSegment2D()	: Line2D(), LineNumber() {}
			  
	/// Construct a segment from two positions
	LineSegment2D(const Position2D &pos1, const Position2D &pos2);
	
	/// Construct a segment from two positions, a number and a label
	LineSegment2D(const Position2D &pos1, const Position2D &pos2,
                  int nr, int lab);
	
	/// Construct a segment from a line and begin and end scalar
	LineSegment2D(const Line2D &line, double s_begin, double s_end);
	
	/// Copy assignment  			
	LineSegment2D& operator=(const LineSegment2D &line_segment);  
	  
	/// Default destructor  
	~LineSegment2D() {};     
	  
        /// Return readable reference
	const LineSegment2D & LineSegment2DRef() const
	  { return *this; }  
	
        /// Return writable reference
	LineSegment2D & LineSegment2DRef()
	  { return *this; }  
	
        /// Set label
        void Label(int new_label)
          { label = new_label; }

        /// Return label
        int Label() const
          { return label; }

	/// Length of the segment			     
        double Length() const
          { return(scalar_end - scalar_begin); }
                
        /// Scalar of begin position
        double ScalarBegin() const
          { return(scalar_begin); }

        /// Scalar of end position
        double ScalarEnd() const
          { return(scalar_end); }

        /// Scalar of begin position (writable)
        double & ScalarBegin()
          { return(scalar_begin); }

        /// Scalar of end position (writable)
        double & ScalarEnd()
          { return(scalar_end); }

        /// Begin position of segment
        Position2D BeginPoint() const
          { return(Position(scalar_begin)); }

        /// End position of segment
        Position2D EndPoint() const
          { return(Position(scalar_end)); }

        /// Position of the middle point 
        Position2D MiddlePoint() const;

        /// Distance of position to segment
        double Distance(const Position2D &pos) const;

        /// Distance to another segment
        double Distance(const LineSegment2D &segm) const;

        /// Distance of between two segments
        friend double DistanceLineSegment2DToLineSegment2D
           (const LineSegment2D &segm1, const LineSegment2D &segm2)
           {return(segm1.Distance(segm2));}

        /// Intersection of two segments
        /** @param segm1 First line segment
            @param segm2 Second line segment
            @param inter_pos The calculated position of the intersection point
            @param max_dist Maximum distance between the segments (tolerance)
            @return 1 if the segments intersect, 0 otherwise.
        */
        friend bool Intersect2LineSegments2D(const LineSegment2D &segm1,
                                             const LineSegment2D &segm2,
                                             Position2D &inter_pos,
                                             double max_dist);

        /// Intersection of segment with line
        /** @param line Line to be intersected with this segment
            @param inter_pos The calculated position of the intersection point
            @param max_dist Maximum distance between the line and segment
                   (tolerance)
            @return 1 if the segment intersects the line, 0 otherwise.
        */
        bool Intersect(const Line2D &line, Position2D &inter_pos,
                       double max_dist) const;

        /// Check on collinearity with a line
        /** A segment is considered collinear with a line if the angle between
            the segment and the line is below a threshold and the end points
            of the segment are within a specified distance to the line.
            @param line A two-dimensional line
            @param err_angle Tolerance for difference in direction
            @param err_dist Tolerance for distance of end points to line.
            @return 1 if the segment is collinear with the line, 0 otherwise.
        */
        bool Collinear(const Line2D &line, double err_angle,
                       double err_dist) const;

        /// Convert to 2D object points with topology
        /** 
            @param objpts Object points created from the end points of the
                          segment
            @param top Topology of the segment (an open polygon with two nodes).
            @param append If true, add the topology of the segment to the
                          topology vector. Otherwise, first empty the vector.
        */
        void PointsWithTopology(ObjectPoints2D &objpts, LineTopologies &top,
                                int append) const;
};
#endif /* _LineSegment2D_h_ */   /* Do NOT add anything after this line */
