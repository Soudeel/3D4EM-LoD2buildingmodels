
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
 * \brief Interface to Class ObjectPoint - A point in an object coordinate system
 *
 */
/*!
 * \class ObjectPoint
 * \ingroup Photogrammetry
 * \brief Interface to Class ObjectPoint - A point in an object coordinate system
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

#ifndef _ObjectPoint_h_
#define _ObjectPoint_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ObjectPoint  - A point in an object coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point3D.h"
#include "Database4Cpp.h"

class ObjectPoint2D;
class ControlPoint;
class ImagePoint;
class ImageGrid;
class DataGrid;


//------------------------------------------------------------------------------
/// Three-dimensional object point
//------------------------------------------------------------------------------

class ObjectPoint : public Point3D {

  public:

    /// Default constructor
    ObjectPoint() : Point3D() {}

    /// Construct with specified values
    /** @param x     X-coordinate
        @param y     Y-coordinate
        @param z     Z-coordinate
        @param num   Model point number
        @param v_x   Variance of the X-coordinate
        @param v_y   Variance of the Y-coordinate
        @param v_z   Variance of the Z-coordinate
        @param cv_xy Co-variance of the X- and Y-coordinate
        @param cv_xz Co-variance of the X- and Z-coordinate
        @param cv_yz Co-variance of the Y- and Z-coordinate
    */
    ObjectPoint(double x, double y, double z, int num, 
	        double v_x, double v_y, double v_z,
	        double cv_xy, double cv_xz, double cv_yz)
	: Point3D(x, y, z, num, v_x, v_y, v_z, cv_xy, cv_xz, cv_yz) {}
	
    /// Construct by converting a C object
    ObjectPoint(ObjPt *objpt) : Point3D()
      { C2Cpp(objpt); }

    /// Construct from a vector, point number and a covariance matrix
    /** @param pos Coordinates of the model point
        @param num Point number of the model point
        @param cov Co-variance matrix of the model point
    */
    ObjectPoint(const Vector3D &pos, const PointNumber &num,
                const Covariance3D &cov)
    	: Point3D(pos, num, cov) {}

    /// Construct from a 2D object point
    /** @param objpt2d Two-dimensional object point
        @param z       Height of the three-dimensional object point to be
                       constructed. Default height is 0.
    */
    ObjectPoint(const ObjectPoint2D *objpt2d, double z = 0);   
    
    /// Construct from a control point
    ObjectPoint(const ControlPoint *);
    
    /// Construct from an image point and an image grid
    /** The row-column coordinates of the image point are converted to the
        X- en Y-coordinates of the object point through the image grid
        definition. The Z-coordinate is specified as an argument.
        @param imgpt Pointer to an image point
        @param grid  Pointer to an image grid definition
        @param z     Z-coordinate of the object point to be constructed
    */
    ObjectPoint(const ImagePoint *imgpt, const ImageGrid *grid, double z);
        	
    /// Construct from an image point and a data grid
    /** The row-column coordinates of the image point are converted to the
        X- en Y-coordinates of the object point through the data grid
        definition. The Z-coordinate is determined by converting the image
        grey value using the data grid definition.
        @param imgpt Pointer to an image point
        @param grid  Pointer to an data grid definition
        @param grey  Grey value at the specified image point location
    */
    ObjectPoint(const ImagePoint *imgpt, const DataGrid *grid, double grey);

    /// Copy constructor
    ObjectPoint(const ObjectPoint &pt) : Point3D()
      { *this = pt; } 
        	
    /// Default destructor
    ~ObjectPoint() {};
     
    /// Copy assignment
    ObjectPoint & operator = (const ObjectPoint & point);
      
    /// Conversion from C to C++ object
    void C2Cpp(ObjPt *);

    /// Conversion from C++ to C object
    void Cpp2C(ObjPt **) const;

    /// Return the readable reference
    const ObjectPoint &ObjectPointRef() const
      { return *this; }

    /// Return the writable reference
    ObjectPoint &ObjectPointRef()
      { return *this; }

    /// Return the pointer
    ObjectPoint * ObjectPointPtr()
      { return this; }

};
#endif /* _ObjectPoint_h_ */   /* Do NOT add anything after this line */
