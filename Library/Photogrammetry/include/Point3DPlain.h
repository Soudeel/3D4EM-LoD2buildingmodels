
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
 * \brief Interface to Class Point3DPlain - A non-stochastic point in a three-dimensional coordinate system
 *
 */
/*!
 * \class Point3DPlain
 * \ingroup Photogrammetry
 * \brief Interface to Class Point3DPlain - A non-stochastic point in a three-dimensional coordinate system
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

#ifndef _Point3DPlain_h_
#define _Point3DPlain_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 Point3DPlain - A non-stochastic point in a three-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position3D.h"
#include "PointNumber.h"

//------------------------------------------------------------------------------
/// A non-stochastic point in a three-dimensional coordinate system
//------------------------------------------------------------------------------

class Point3DPlain : public PointNumber, public Position3D {

  public:

    /// Default constructor
    Point3DPlain() : PointNumber(), Position3D() {}

    /// Construct from coordinates and a point number
    Point3DPlain(double x, double y, double z, int n)	
	: PointNumber(n), Position3D(x, y, z) {} 

    /// Construct from a coordinate vector and a point number
    Point3DPlain(const Vector3D &v, const PointNumber &n)
    	: PointNumber(n), Position3D(v) {}

    /// Copy constructor
    Point3DPlain(const Point3DPlain *point)
      : PointNumber(), Position3D()
    	{ x[0] = point->x[0]; x[1] = point->x[1];
    	  x[2] = point->x[2]; num = point->num;
    	}  

    /// Default destructor
    ~Point3DPlain()
      {};
};
#endif /* _Point3DPlain_h_ */   /* Do NOT add anything after this line */
