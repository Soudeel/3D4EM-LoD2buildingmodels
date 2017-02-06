
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
 * \brief Interface to Class ModelPoint - A point in a model coordinate system
 *
 */
/*!
 * \class ModelPoint
 * \ingroup Photogrammetry
 * \brief Interface to Class ModelPoint - A point in a model coordinate system
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

#ifndef _ModelPoint_h_
#define _ModelPoint_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ModelPoint  - A point in a model coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point3D.h"
#include "Database4Cpp.h"

//------------------------------------------------------------------------------
/// Point in a model coordinate system
//------------------------------------------------------------------------------

class ModelPoint : public Point3D {

  public:

    /// Default constructor
    ModelPoint() : Point3D() {}

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
    ModelPoint(double x, double y, double z, int num, 
	    double v_x, double v_y, double v_z,
	    double cv_xy, double cv_xz, double cv_yz)
	: Point3D(x, y, z, num, v_x, v_y, v_z, cv_xy, cv_xz, cv_yz) {}
	
    /// Construct by converting a C object
    ModelPoint(ModelPt *modelpt) : Point3D()
      { C2Cpp(modelpt); }

    /// Construct from a vector, point number and a covariance matrix
    /** @param pos Coordinates of the model point
        @param num Point number of the model point
        @param cov Co-variance matrix of the model point
    */
    ModelPoint(const Vector3D &pos, const PointNumber &num,
               const Covariance3D &cov)
    	: Point3D(pos, num, cov) {}

    /// Default destructor
    ~ModelPoint() {}
      
    /// Copy assignment
    ModelPoint & operator = (const ModelPoint & point);
      
    /// Conversion from C to C++ object
    void C2Cpp(ModelPt *);

    /// Conversion from C++ to C object
    void Cpp2C(ModelPt **) const;
};
#endif /* _ModelPoint_h_ */   /* Do NOT add anything after this line */
