
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
 * \brief A vector of line segments in a three-dimensional coordinate system
 *
 */
/*!
 * \class LineSegments3D
 * \ingroup Photogrammetry
 * \brief A vector of line segments in a three-dimensional coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><t
r><td>
 *
 * \author        \G_Vosselman
 * \date          21-Oct-2008 (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef _LineSegments3D_h_
#define _LineSegments3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineSegments3D - a vector of 3D line segments
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <vector>
#include "LineSegment3D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"

/*
--------------------------------------------------------------------------------
                                    Segmenst3D
--------------------------------------------------------------------------------
*/

class LineSegments3D : public std::vector<LineSegment3D>
{
  public:
    /// Default constructor 
    LineSegments3D() : std::vector<LineSegment3D>() {}

    /// Copy constructor
    LineSegments3D(const LineSegments3D &segments)
      : std::vector<LineSegment3D>()
      { *this = segments; }
      
	/// Destructor
	~LineSegments3D()
	  {};
	  
	/// Copy assignment   
	LineSegments3D& operator=(const LineSegments3D &);

    /// Construct from a 3D polygon
    LineSegments3D(const ObjectPoints &objpts, const LineTopology &top);

    /// Construct from 3D polygons
    LineSegments3D(const ObjectPoints &objpts, const LineTopologies &top);

    /// Convert segments to points with topology
    /**
        @param objpts Object points created from the end points of the
                      segment
        @param top Topology of the segments (an open polygon with two nodes)
        @param append If true, add the topology of the segment to the
                      topology vector. Otherwise, first empty the vector.
    */
    void PointsWithTopology(ObjectPoints &objpts, LineTopologies &top,
                            int append) const;
    
    /// Sort line segments on start scalar
    /** This function is only useful for collinear line segments */
    void SortOnStartScalar();

    /// Sort line segments on number
    void SortOnNumber();
};	             
#endif /* _LineSegments3D_h_ */   /* Do NOT add anything after this line */
