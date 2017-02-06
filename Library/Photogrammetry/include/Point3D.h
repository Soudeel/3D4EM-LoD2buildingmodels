
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
 * \brief Interface to Class Point3D - A stochastic point in a three-dimensional coordinate system
 *
 */
/*!
 * \class Point3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Point3D - A stochastic point in a three-dimensional coordinate system
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

#ifndef _Point3D_h_
#define _Point3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Point3D  - A stochastic point in a three-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point3DPlain.h"
#include "Covariance3D.h"

//------------------------------------------------------------------------------
/// A stochastic point in a three-dimensional coordinate system
//------------------------------------------------------------------------------

class Point3D : public Point3DPlain, public Covariance3D {

  public:

    /// Default constructor
    Point3D() : Point3DPlain(), Covariance3D() {}
      
    /// Construct from coordinates, a point number and (co-)variances
    Point3D(double x, double y, double z, int num, 
	    double v_x, double v_y, double v_z,
	    double cv_xy, double cv_xz, double cv_yz)
	: Point3DPlain(x, y, z, num), 
	  Covariance3D(v_x, v_y, v_z, cv_xy, cv_xz, cv_yz) {}

    /// Construct from a non-stochastic point and a covariance matrix
    Point3D(const Point3DPlain &point, const Covariance3D &cv)
    	: Point3DPlain(point),
    	  Covariance3D(cv) {}
   
    /// Construct from a vector, a point number and a covariance matrix
    Point3D(const Vector3D &vec, const PointNumber &n, const Covariance3D &cv)
    	: Point3DPlain(vec, n),
    	  Covariance3D(cv)
    	  {}
    	 	  
    /// Default destructor
    ~Point3D() {};
};
#endif /* _Point3D_h_ */   /* Do NOT add anything after this line */
