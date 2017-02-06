
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
 * \brief Interface to Class Point2D - A stochastic point in a two-dimensional coordinate system
 *
 */
/*!
 * \class Point2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Point2D - A stochastic point in a two-dimensional coordinate system
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

#ifndef _Point2D_h_
#define _Point2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Point2D  - A stochastic point in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point2DPlain.h"
#include "Covariance2D.h"

//------------------------------------------------------------------------------
/// A stochastic point in a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Point2D : public Point2DPlain, public Covariance2D {

  public:

    /// Default constructor
    Point2D() : Point2DPlain(), Covariance2D() {}

    /// Construct from coordinates, a point number and (co-)variances
    Point2D(double x, double y, int num, double v_x, double v_y, double cv_xy) 
	: Point2DPlain(x, y, num), 
	  Covariance2D(v_x, v_y, cv_xy) {} ;
	 
    /// Construct from a vector, a point number and a covariance matrix
    Point2D(const Vector2D &vec, const PointNumber &n, const Covariance2D &var)
    	: Point2DPlain(vec, n), 
    	  Covariance2D(var) {};

    /// Default destructor
    ~Point2D() {};
};
#endif /* _Point2D_h_ */   /* Do NOT add anything after this line */
