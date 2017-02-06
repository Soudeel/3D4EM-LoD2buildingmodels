
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
 * \brief Interface to Class ObjectPoints2D - A set of stochastic two-dimensional object points
 *
 */
/*!
 * \class ObjectPoints2D
 * \ingroup Photogrammetry
 * \brief Interface to Class ObjectPoints2D - A set of stochastic two-dimensional object points
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

#ifndef _ObjectPoints2D_h_
#define _ObjectPoints2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ObjectPoints2D  - A set of stochastic two-dimensional object points

--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "ObjectPoint2D.h"
#include "ImagePoints.h"
#include "ImageGrid.h"
#include "Database4Cpp.h"


//------------------------------------------------------------------------------
/// A vector of two-dimensional object points
//------------------------------------------------------------------------------

class ObjectPoints2D : public VectorPoint<ObjectPoint2D>
{
  public:

    /// Default constructor
    ObjectPoints2D() : VectorPoint<ObjectPoint2D>() {}

    /// Construct by reading a file
    ObjectPoints2D(const char *filename) : VectorPoint<ObjectPoint2D>()
    	{ Read(filename); }	     	

    /// Construct by converting a C object
    ObjectPoints2D(ObjPts2D *objpts) : VectorPoint<ObjectPoint2D>()
	{ C2Cpp(objpts); }

    /// Construct from image points and an image grid definition
    /** The row-column coordinates of the image points are converted to the
        X- en Y-coordinates of the object points through the image grid
        definition.
        @param imgpts Image points
        @param grid   Image grid definition
    */
    ObjectPoints2D(const ImagePoints &imgpts, const ImageGrid &grid);

    /// Default destructor
    ~ObjectPoints2D() {};

    /// Copy assignment
    ObjectPoints2D & operator = (const ObjectPoints2D &);

    /// Return the readable reference
    const ObjectPoints2D &RefObjectPoints2D() const
        { return *this; } 

    /// Return the writable reference
    ObjectPoints2D &RefObjectPoints2D()
        { return *this; } 

    /// Convert from C to C++ object
    void C2Cpp(ObjPts2D *);

    /// Convert from C++ to C object
    void Cpp2C(ObjPts2D **) const;

    /// Read object points from a file
    /** @param filename Name of the file with 2D object points
        @return 0 - failure, 1 - success
    */
    int Read(const char *filename);

    /// Write object points to a file
    /** @param filename Name of the file for the 2D object points
        @return 0 - failure, 1 - success
    */
    int Write(const char *filename) const;

    /// Print the object points to stdout
    void Print() const;

    /// Write the header of a point file
    /** The file is opened by this function and left open. Usage unknown.
        @param filename Name of the 2D point file to be created
        @return 0 - failure, 1 - success.
    */
    int WriteHeader(const char *filename) const;
};
#endif /* _ObjectPoints2D_h_ */   /* Do NOT add anything after this line */
