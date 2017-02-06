
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
 * \brief Interface to Class ObjectPoint2D - A point in a two-dimensional object coordinate system
 *
 */
/*!
 * \class ObjectPoint2D
 * \ingroup Photogrammetry
 * \brief Interface to Class ObjectPoint2D - A point in a two-dimensional object coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ObjectPoint2D_h_
#define _ObjectPoint2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ObjectPoint2D  - A point in a two-dimensional object coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point2D.h"
#include "Database4Cpp.h"

class ObjectPoint;
class ImagePoint;
class ImageGrid;
class LineTopology;
class ObjectPoints2D;
class ObjectPoints;

//------------------------------------------------------------------------------
/// A two-dimensional object point
//------------------------------------------------------------------------------

class ObjectPoint2D : public Point2D {

friend class ObjectPoint;

  public:

    /// Default constructor
    ObjectPoint2D() : Point2D() {}

    /// Construct with specified values
    /** @param x     X-coordinate
        @param y     Y-coordinate
        @param num   Model point number
        @param v_x   Variance of the X-coordinate
        @param v_y   Variance of the Y-coordinate
        @param cv_xy Co-variance of the X- and Y-coordinate
    */
    ObjectPoint2D(double x, double y, int num, 
		  double v_x, double v_y, double cv_xy)
	: Point2D(x, y, num, v_x, v_y, cv_xy) {}
	
    /// Construct from a vector, point number and a covariance matrix
    /** @param pos Coordinates of the model point
        @param num Point number of the model point
        @param cov Co-variance matrix of the model point
    */
    ObjectPoint2D(const Vector2D &pos, const PointNumber &num,
                const Covariance2D &cov)
        : Point2D(pos, num, cov) {}

    /// Construct by converting a C object
    ObjectPoint2D(ObjPt2D *objpt) : Point2D()
      { C2Cpp(objpt); }
      
    /// Construct from an image point and an image grid definition
    /** The row-column coordinates of the image point are converted to the
        X- en Y-coordinates of the object point through the image grid
        definition.
        @param imgpt Pointer to an image point
        @param grid  Pointer to an image grid definition
    */
    ObjectPoint2D(const ImagePoint *imgpt, const ImageGrid *grid);

    /// Copy constructor
    ObjectPoint2D(const ObjectPoint2D &pt) : Point2D()
      { *this = pt; } 
  
    /// Make a 2D version of a 3D object point
    ObjectPoint2D(const ObjectPoint &pt);

    /// Default destructor
    ~ObjectPoint2D() {};

    /// Return the writable reference
    ObjectPoint2D & ObjectPoint2DReference()
      {return(*this);}

    /// Return the readable reference
    const ObjectPoint2D & ObjectPoint2DReference() const
      {return(*this);}

    /// Return the pointer
    ObjectPoint2D * ObjectPoint2DPtr()
      {return(this);}

    /// Return the pointer for reading only
    const ObjectPoint2D * ObjectPoint2DPtr() const
      {return(this);}

    /// Copy assigment
    ObjectPoint2D & operator = (const ObjectPoint2D & point);  
    
    /// Set the X and Y coordinates
    void SetXY(double X, double Y)
    	{ x[0] = X; x[1] = Y; } 
      
    /// Conversion from C to C++ object
    void C2Cpp(ObjPt2D *);

    /// Conversion from C++ to C object
    void Cpp2C(ObjPt2D **) const;
    
    /// Write a point record to a file
    void Write(FILE *) const;

    /// Intersect two line segments
    /** The two line segments need to be specified by one common list of 2D
        object points and two line topologies. Of each line topology the first
        two nodes are assumed to represent the line segments to be intersected.
        Further nodes of the line topologies are ignored. The X- and
        Y-coordinates of the intersection point are stored in "this".
        @param objpts List of 2D object points
        @param segm1  First line segment
        @param segm2  Second line segment
        @return 0 - failure (node number of line topology not found in the
                list of object points, or two parallel lines), 1 - success.
    */
    int IntersectLines(const ObjectPoints2D &objpts, const LineTopology &segm1,
                       const LineTopology &segm2);

    /// Check if the point is inside a polygon of 2D object points
    /** @param objpts   A vector of object points including the points
                        of the polygon
        @param polygon Node numbers of the polygon. This polygon needs to be
                       closed, i.e. the first and last node number need to be
                       the same.
        @return 0 - point is outside the polygon,
                1 - point is inside the polygon
    */
    int InsidePolygon(const ObjectPoints2D &objpts,
                      const LineTopology &polygon) const;

    /// Check if the point is inside a polygon of 3D object points
    /** @param objpts   A vector of object points including the points
                        of the polygon
        @param polygon Node numbers of the polygon. This polygon needs to be
                       closed, i.e. the first and last node number need to be
                       the same.
        @return 0 - point is outside the polygon,
                1 - point is inside the polygon
    */
    int InsidePolygon(const ObjectPoints &objpts,
                      const LineTopology &polygon) const;
};
#endif /* _ObjectPoint2D_h_ */   /* Do NOT add anything after this line */
