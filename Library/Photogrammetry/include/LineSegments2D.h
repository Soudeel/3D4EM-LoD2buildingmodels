
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
 * \brief A vector of line segments in a two-dimensional coordinate system
 *
 */
/*!
 * \class LineSegments2D
 * \ingroup Photogrammetry
 * \brief A vector of line segments in a two-dimensional coordinate system
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

#ifndef _LineSegments2D_h_
#define _LineSegments2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LineSegments2D - a vector of 2D line segments
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <vector>
#include "LineSegment2D.h"
#include "ObjectPoints2D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"

/*
--------------------------------------------------------------------------------
                                    Segmenst2D
--------------------------------------------------------------------------------
*/

class LineSegments2D : public std::vector<LineSegment2D>
{
public:
	/// Default constructor 
	LineSegments2D() : std::vector<LineSegment2D>() {}

	/// Destructor
	~LineSegments2D()
	  {};
	  
	/// Copy assignment   
	LineSegments2D& operator=(const LineSegments2D &);

        /// Construct from a 2D polygon
        LineSegments2D(const ObjectPoints2D &objpts, const LineTopology &top);

        /// Construct from 2D polygons
        LineSegments2D(const ObjectPoints2D &objpts, const LineTopologies &top);

        /// Construct from a 3D polygon
        LineSegments2D(const ObjectPoints &objpts, const LineTopology &top);

        /// Construct from 3D polygons
        LineSegments2D(const ObjectPoints &objpts, const LineTopologies &top);

        /// Check on collinearity of one of the segments with a line
        /** @param line A two-dimensional line
            @param err_angle Allowed difference in direction
            @param err_dist  Allowed distance of end points to the line
            @return 1 if one of the segments is collinear to the line,
                    0 otherwise.
        */
        bool Collinear(const Line2D &, double, double) const;

        /// Convert segments to points with topology
        /**
            @param objpts Object points created from the end points of the
                          segment
            @param top Topology of the segments (an open polygon with two nodes)
            @param append If true, add the topology of the segment to the
                          topology vector. Otherwise, first empty the vector.
        */
        void PointsWithTopology(ObjectPoints2D &objpts, LineTopologies &top,
                                int append) const;

        /// Intersect a convex polygon by a line
        /** This function was specific to a building reconstruction programme.
            This LineSegments2D is a set of line segments that form a convex
            polygon. This polygon is intersected by a line. If successful,
            the resulting intersecting line segment is determined.
            @param line Line to intersect the convex polygon
            @param inter_segm The resulting line segment that intersects the
                              polygon
            @param max_dist Tolerance for the computation of intersections
            @return 1 if the line intersects the polygon, 0 if not.
        */
        bool IntersectPartition(const Line2D &line, LineSegment2D &inter_segm,
                                double max_dist) const;

        /// Part of segment inside polygon
        /** This function was specific to a building reconstruction programme.
            This LineSegments2D is a set of line segments that form a convex
            polygon. For the specified line segment (segm) it is determined
            over which length this line segment is inside the polygon. It is
            also determined to what extent the line segment does not cover the
            line segment representing the intersection between the line (of the
            line segment) and the specified line segment (segm).
            @param segm A 2D line segment
            @param len_present Length of line segment inside the polygon
            @param len_missing Difference in length with intersection segment
            @return 0 if the segment is completely outside the polygon, 
                    1 otherwise.
        */
        bool SegmentInPartition(const LineSegment2D &segm, double &len_present,
                                double &len_missing) const;

        /// Split a partition by a line
        /** This function was specific to a building reconstruction programme. 
            This LineSegments2D is a set of line segments that form a convex
            polygon. The corner points and topology of the polygon are also
            specified. The polygon is intersected with a two dimensional line
            and split into two parts. The calculated intersection points are put
            into a separate vector (new_corners) and the topology of the
            two resulting polygons is put into a separate topology vector 
            (new_partitions)
            @param corners Corner points of the polygon
            @param polygon Topology of the polygon
            @param line Line to intersect with the polygon
            @param new_corners Calculated intersection points
            @param new_polygons Topology of the two new polygons
        */
        void SplitPartition(ObjectPoints2D &corners, LineTopology &polygon,
                            Line2D &line, ObjectPoints2D &new_corners,
                            LineTopologies &new_polygons);

        /// Get the partition sizes without actually partitioning
        /** This function was specific to a building reconstruction programme. 
            This LineSegments2D is a set of line segments that form a convex
            polygon. The corner points and topology of the polygon are also
            specified. Without actually splitting a polygon (like by
            SplitPartition) this function calculates the sizes of both parts
            that would result from a polygon split.
            @param corners Corner points of the polygon
            @param polygon Topology of the polygon
            @param line Line to intersect with the polygon
            @param size1 Size of the first polygon part
            @param size2 Size of the second polygon part
        */
        void CheckPartitionSizes(ObjectPoints2D &corners, LineTopology &polygon,
                                 Line2D &line, double &size1, double &size2);
        
        /// Add a segment if it is the longest of its direction
        /** A segment is added to this if there is no longer segment with a
            similar direction. If there is a shorter segment with a similar
            direction, this segment is replaced. Directions are calculated
            modulo 90 degrees.
            @param segment   Segment to be added
            @param min_angle Minimum angle between two segments in this
                             (in radians)
        */
        void AddIfLongestSegment(const LineSegment2D &segment,
                                 double min_angle);
        void SortOnStartScalar();
		void SortOnNumber();
};	             
#endif /* _LineSegments2D_h_ */   /* Do NOT add anything after this line */
