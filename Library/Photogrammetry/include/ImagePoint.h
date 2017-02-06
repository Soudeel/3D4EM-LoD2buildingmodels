
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
 * \brief Interface to Class ImagePoint - A stochastic point in a digital image
 *
 */
/*!
 * \class ImagePoint
 * \ingroup Photogrammetry
 * \brief Interface to Class ImagePoint - A stochastic point in a digital image
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A stochastic point in a digital image - This point is defined by a pixel position (row, column), a point number and
 *  a point covariance matrix.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ImagePoint_h_
#define _ImagePoint_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImagePoint  - A stochastic point in a digital image

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "ImagePointPlain.h"
#include "Covariance2D.h"
#include "Database4Cpp.h"

class CameraPoint;
class InteriorOrientation;

//------------------------------------------------------------------------------
/// A stochastic point in a digital image
/** This point is defined by a pixel position (row, column), a point number and
    a point covariance matrix.
*/
//------------------------------------------------------------------------------

class ImagePoint : public ImagePointPlain, public Covariance2D {

  public:

    /// Default constructor
    ImagePoint() : ImagePointPlain(), Covariance2D() {}

    /// Construct by converting a C object
    ImagePoint(ImgPt *imgpt) : ImagePointPlain(), Covariance2D()
      {C2Cpp(imgpt);};

    /// Constructor from specified values
    /** @param number   Image point number
        @param row      Row coordinate
        @param column   Column coordinate
        @param variance Variance of the row and column coordinate.
                        The covariance is set to 0.
    */
    ImagePoint(int number, double row, double column, double variance)
      : ImagePointPlain(), Covariance2D()
      {num = number; x[0] = row;  x[1] = column;
       var[0] = var[1] = variance; var[2] = 0.0;};
       
    /// Constructor from a 2D vector, a point number and a 2D covariance matrix
    ImagePoint(const Vector2D &vec, const PointNumber &n,
               const Covariance2D &var)
    	: ImagePointPlain(vec, n), 
    	  Covariance2D(var) {};

    /// Constructor transforming camera points to the image coordinate system
    /** This transformation converts the mm coordinates to coordinates in
        pixels and adds the lens distortion.
        @param campt Camera point
        @param intor Interior orientation
    */
    ImagePoint(const CameraPoint *campt, const InteriorOrientation *intor);

    /// Copy constructor	
    ImagePoint(const ImagePoint &pt) : ImagePointPlain(), Covariance2D()
       { *this = pt; }

    /// Destructor
    ~ImagePoint() {};

    /// Copy assignment
    ImagePoint & operator = (const ImagePoint & point);

    /// Conversion from C to C++ object
    void C2Cpp(ImgPt *);
 
    /// Conversion from C++ to C object
    void Cpp2C(ImgPt **) const;

    /// Transform a camera point into an image point
    /** This transformation converts the mm coordinates to coordinates in
        pixels and adds the lens distortion.
        @param campt Camera point
        @param intor Interior orientation
    */
    void Camera2ImagePoint(const CameraPoint *campt,
                           const InteriorOrientation *intor);

    /// Transform an image point into a camera point
    /** This transformation converts the pixel coordinates to coordinates in
        mm and corrects for the lens distortion.
        @param intor Interior orientation
        @return      A pointer to a camera point
    */
    CameraPoint *Image2CameraPoint(const InteriorOrientation *intor) const;
};

#endif /* _ImagePoint_h_ */   /* Do NOT add anything after this line */
