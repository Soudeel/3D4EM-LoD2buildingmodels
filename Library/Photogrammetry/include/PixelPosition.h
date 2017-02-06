
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
 * \brief Interface to Class PixelPosition - A position in a digital image
 *
 */
/*!
 * \class PixelPosition
 * \ingroup Photogrammetry
 * \brief Interface to Class PixelPosition - A position in a digital image
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A position in a digital image
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _PixelPosition_h_
#define _PixelPosition_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  PixelPosition  - A position in a digital image

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Position2D.h"
class InteriorOrientation;

//------------------------------------------------------------------------------
/// A position in a digital image
//------------------------------------------------------------------------------

class PixelPosition : public Position2D {

  public:

    /// Default constructor
    PixelPosition() : Position2D() {}

    /// Construct from a row and column coordinate
    PixelPosition(double r, double c) : Position2D(r, c) {}
	
    /// Construct from a two-dimensional vector 
    PixelPosition(const Vector2D &vec) : Position2D(vec) {}
    	
    /// Construct by transforming a position in a camera coordinate system into the image coordinate system
    /** @param campos Position in a camera coordinate system
        @param intor  Interior orientation of the camera
    */
    PixelPosition(const Position2D *campos, const InteriorOrientation *intor);

    /// Copy constructor
    PixelPosition(const PixelPosition &pos) : Position2D()
        { vect() = pos.vect(); }   
	
    /// Destructor
    ~PixelPosition() {};

    /// Copy assignament
    PixelPosition& operator=(const PixelPosition &pos)
        { vect() = pos.vect(); return *this; }
      
    /// Return the readable row number
    double Row() const
      { return x[0]; }	

    /// Return the readable column number
    double Col() const
      { return x[1]; }	

    /// Return the readable column number
    double Column() const
      { return x[1]; }	

    /// Return the writable row number
    double &Row()
      { return x[0]; }      

    /// Return the writable column number
    double &Col()
      { return x[1]; }

    /// Return the writable column number
    double &Column()
      { return x[1]; }
};
#endif /* _PixelPosition_h_ */   /* Do NOT add anything after this line */
