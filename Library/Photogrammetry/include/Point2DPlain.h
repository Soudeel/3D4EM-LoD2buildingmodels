
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
 * \brief Interface to Class Point2DPlain - A non-stochastic point in a two-dimensional coordinate system
 *
 */
/*!
 * \class Point2DPlain
 * \ingroup Photogrammetry
 * \brief Interface to Class Point2DPlain - A non-stochastic point in a two-dimensional coordinate system
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

#ifndef _Point2DPlain_h_
#define _Point2DPlain_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Point2DPlain  - A non-stochastic point in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position2D.h"
#include "PointNumber.h"

//------------------------------------------------------------------------------
/// A non-stochastic point in a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Point2DPlain : public PointNumber, public Position2D {

  public:

    /// Default constructor
    Point2DPlain() : PointNumber(), Position2D() {}

    /// Construct from coordinates and a point number
    Point2DPlain(double x, double y, int n)
      : PointNumber(n), Position2D(x, y) {}; 
	
    /// Construct from a vector and a point number
    Point2DPlain(const Vector2D &v, const PointNumber &n)
      : PointNumber(n), Position2D(v) {};	

    /// Default destructor
    ~Point2DPlain() {};
};
#endif /* _Point2DPlain_h_ */   /* Do NOT add anything after this line */
