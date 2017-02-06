
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
 * \brief Interface to Class ImagePointPlain - A non-stochastic point in a digital image
 *
 */
/*!
 * \class ImagePointPlain
 * \ingroup Photogrammetry
 * \brief Interface to Class ImagePointPlain - A non-stochastic point in a digital image
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A non-stochastic point in a digital image
 * This point is defined by a pixel position (row, column) and a point number
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ImagePointPlain_h_
#define _ImagePointPlain_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImagePointPlain  - A non-stochastic point in a digital image

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "PixelPosition.h"
#include "PointNumber.h"

//------------------------------------------------------------------------------
/// A non-stochastic point in a digital image
/** This point is defined by a pixel position (row, column) and a point number
*/
//------------------------------------------------------------------------------

class ImagePointPlain : public PixelPosition, public PointNumber {

  public:

    /// Default constructor
    ImagePointPlain() : PixelPosition(), PointNumber() {}

    /// Construct from specified values
    ImagePointPlain(double r, double c, int num)
      : PixelPosition(r, c), PointNumber(num) {}
	  
    /// Construct from a 2D vector and a point number
    ImagePointPlain(const Vector2D &v, const PointNumber &n)
      : PixelPosition(v), PointNumber(n) {}	
	  
    /// Default destructor
    ~ImagePointPlain() {};
};
#endif /* _ImagePointPlain_h_ */   /* Do NOT add anything after this line */
