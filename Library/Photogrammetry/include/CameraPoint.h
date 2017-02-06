
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
 * \brief Interface to Class CameraPoint - A point in a camera coordinate system
 *
 */
/*!
 * \class CameraPoint
 * \ingroup Photogrammetry
 * \brief Interface to Class CameraPoint - A point in a camera coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li The point is described by a point number, a vector with its two coordinates and
 * the variances and covariances of the coordinates. All parameters are inherited
 * from the Point2D class. The unit for the coordinates and (co-)variances is
 * assumed to be millimeter.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _CameraPoint_h_
#define _CameraPoint_h_

/*
--------------------------------------------------------------------------------
  CameraPoint  - A point in a camera coordinate system
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point2D.h"
#include "Database4Cpp.h"
#include "Line3D.h"

class ObjectPoint;
class ControlPoint;
class ImagePoint;
class InteriorOrientation;
class ExteriorOrientation;

//------------------------------------------------------------------------------
//                      Point in a camera coordinate system
/*
The point is described by a point number, a vector with its two coordinates and
the variances and covariances of the coordinates. All parameters are inherited
from the Point2D class. The unit for the coordinates and (co-)variances is
assumed to be millimeter.
*/
//------------------------------------------------------------------------------

class CameraPoint : public Point2D {

  public:

    /// Default constructor
    CameraPoint() : Point2D() {}

    /// Construct camera point from coordinates and variances
    /** @param x     x-coordinate
        @param y     y-coordinate
        @param n     Point number
        @param v_x   Variance of x-coordinate
        @param v_y   Variance of y-coordinate
        @param cv_xy Covariance of x- and y-coordinate
    */
    CameraPoint(double x, double y, int n, 
		double v_x, double v_y, double cv_xy)
	: Point2D(x, y, n, v_x, v_y, cv_xy) {}

    /// Construct camera point from a vector, point number and covariance matrix
    /** @param vec Vector containing the x- and y-coordinate
        @param n   Point number
        @param v   Point variance
    */
    CameraPoint(const Vector2D &vec, const PointNumber &n,
                const Covariance2D &v)
    	: Point2D(vec, n, v) {}

    /// Construct by converting a C object
    CameraPoint(CamPt *campt) : Point2D()
      {C2Cpp(campt);}

   /// Construct by projecting an object point into the camera coordinate system
    /** The collinearity equations are used for the projection
        @param objpt Object point to be projected into the camera coordinate
                     system
        @param intor Interior orientation
        @param extor Exterior orientation
    */
    CameraPoint(const ObjectPoint *objpt, const InteriorOrientation *intor,
                const ExteriorOrientation *extor);
    
    
   /// Construct by projecting a control point into the camera coordinate system
    /** The collinearity equations are used for the projection
        @param ctrlpt Control point to be projected into the camera coordinate
                      system
        @param intor  Interior orientation
        @param extor  Exterior orientation
    */
    CameraPoint(const ControlPoint *ctrlpt, const InteriorOrientation *intor,
                const ExteriorOrientation *extor);

    /// Construct by transformation of an image point into the camera coordinate system
    /** This transformation converts the pixel coordinates to coordinates in mm
        and corrects for the lens distortion.
        @param imgpt Image point (coordinates in pixels)
        @param intor Interior orientation
    */
    CameraPoint(const ImagePoint *imgpt, const InteriorOrientation *intor);

    /// Copy constructor	
    CameraPoint(const CameraPoint &campt)
       { *this = campt; }

    /// Destructor
    ~CameraPoint() {};

    /// Copy assignment
    CameraPoint& operator=(const CameraPoint& point);

    /// Returns a constant reference
    const CameraPoint& CameraPointRef() const
      	{ return *this; }
      
    /// Returns a non-constant reference
    CameraPoint& CameraPointRef()
	{ return *this; }

    /// Conversion from C to C++ object
    void C2Cpp(CamPt *campt);
 
    /// Conversion from C++ to C object
    void Cpp2C(CamPt **campt) const;
    
    /// Projection of a control point into the camera coordinate system
    /** The collinearity equations are used for the projection
         @param ctrlpt Control point to be projected into the camera coordinate
                       system
         @param intor  Interior orientation
         @param extor  Exterior orientation
    */
    void Control2CameraPoint(const ControlPoint *ctrlpt,
                             const InteriorOrientation *intor, 
    		             const ExteriorOrientation *extor);
    		
    /// Projection of an object point into the camera coordinate system
    /** The collinearity equations are used for the projection
        @param objpt Object point to be projected into the camera coordinate
                     system
        @param intor Interior orientation
        @param extor Exterior orientation
    */
    void Object2CameraPoint(const ObjectPoint *objpt,
                            const InteriorOrientation *intor, 
    		            const ExteriorOrientation *extor);

    /// Transformation of an image point into the camera coordinate system
    /** This transformation converts the pixel coordinates to coordinates in mm
        and corrects for the lens distortion.
    @param imgpt Image point (coordinates in pixels)
    @param intor Interior orientation
    */
    void Image2CameraPoint(const ImagePoint *imgpt,
                           const InteriorOrientation *intor);
    
    /// Back projection of a camera point to object coordinate given height
    /** The collinearity equations are used for the back projection
        @param intor Interior orientation
        @param extor Exterior orientation
        @param height Height(Z) vale of the point in object coordinate system
    */
                   
    ObjectPoint Camera2ObjectPoint(const InteriorOrientation *intor,
                                   const ExteriorOrientation *extor,
                                   const double height);
    
    /// 3D line of a camera point to object coordinate system
    /** The collinearity equations are used for the back projection
        @param intor Interior orientation
        @param extor Exterior orientation
    */
                   
    Line3D CameraPoint2ObjectLine(const InteriorOrientation *intor,
                                   const ExteriorOrientation *extor);
    
    /// Back projection of a camera point to object coordinate given plane
    /** The collinearity equations are used for the back projection
        @param intor Interior orientation
        @param extor Exterior orientation
        @param plane Plane containing the objet point
    */
    ObjectPoint* Camera2ObjectPoint(const InteriorOrientation *into,
                                   const ExteriorOrientation *extor, 
                                   const Plane *plane);
    
};
#endif /* _CameraPoint_h_ */   /* Do NOT add anything after this line */
