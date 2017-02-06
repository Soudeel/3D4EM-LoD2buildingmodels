
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
 * \brief Line segment in a three-dimensional coordinate system
 *
 */
/*!
 * \class LineSegment3D
 * \ingroup Photogrammetry
 * \brief Line segment in a three-dimensional coordinate system
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

#ifndef _LineSegment3D_h_
#define _LineSegment3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineSegment3D   - A segment of a 3D line
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position3D.h"
#include "Line3D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"

/*
--------------------------------------------------------------------------------
                                    LineSegment3D
--------------------------------------------------------------------------------
*/

class LineSegment3D : public Line3D, public LineNumber
{

    /// Scalar of begin point of line segment
    double scalar_begin;
    /// Scalar of end point of line segment
    double scalar_end;
    /// Label of the segment
    int label;

public:
	/// Default constructor
	LineSegment3D()	: Line3D(), LineNumber() {}
			  
	/// Construct a segment from two positions
	LineSegment3D(const Position3D &pos1, const Position3D &pos2);
	
	/// Construct a segment from two positions, a number and a label
	LineSegment3D(const Position3D &pos1, const Position3D &pos2,
                  int number, int label);
	
	/// Construct a segment from a line and begin and end scalar
	LineSegment3D(const Line3D &line, double s_begin, double s_end);
	
	/// Construct a segment from a line, begin and end scalar, number and label
	LineSegment3D(const Line3D &line, double s_begin, double s_end,
                  int number, int label);
	
	/// Copy constructor
	LineSegment3D(const LineSegment3D &segment) : Line3D(), LineNumber()
	  { *this = segment; }
	  
	/// Copy assignment  			
	LineSegment3D& operator=(const LineSegment3D &segment);  
	  
	/// Default destructor  
	~LineSegment3D() {};     
	  
        /// Return readable reference
	const LineSegment3D & LineSegment3DRef() const
	  { return *this; }  
	
        /// Return writable reference
	LineSegment3D & LineSegment3DRef()
	  { return *this; }  
	
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
    double &ScalarBegin()
      { return(scalar_begin); }

    /// Scalar of end position (writable)
    double &ScalarEnd()
      { return(scalar_end); }

    /// Begin position of segment
    Position3D BeginPoint() const
      { return(Position(scalar_begin)); }

    /// End position of segment
    Position3D EndPoint() const
      { return(Position(scalar_end)); }

    /// Position of the middle point 
    Position3D MiddlePoint() const;

    /// Distance of position to segment
    double Distance(const Position3D &pos) const;

    /// Convert to 3D object points with topology
    /** @param objpts Object points created from the end points of the
                      segment
        @param top Topology of the segment (an open polygon with two nodes).
        @param append If true, add the topology of the segment to the
                      topology vector. Otherwise, first empty the vector.
    */
    void PointsWithTopology(ObjectPoints &objpts, LineTopologies &top,
                            int append) const;
                            
    /// Reverse the segment direction
    void ReverseDirection();
};
#endif /* _LineSegment3D_h_ */   /* Do NOT add anything after this line */
