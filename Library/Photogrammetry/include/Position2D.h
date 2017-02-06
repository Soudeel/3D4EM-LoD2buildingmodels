
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
 * \brief Interface to Class Position2D - A position in a two-dimensional coordinate system
 *
 */
/*!
 * \class Position2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Position2D - A position in a two-dimensional coordinate system
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

#ifndef _Position2D_h_
#define _Position2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Position2D                - A position in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "Vector2D.h"
#include "ExteriorOrientation.h"


class PixelPosition;
class InteriorOrientation;
class ObjectPoints;
class LineTopology;
class Position3D;
class Positions2D;
class Plane;

//------------------------------------------------------------------------------
/// A position in a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Position2D : public Vector2D {

friend class PixelPosition;

  public:

    /// Default constructor
    Position2D() : Vector2D() {}

    /// Construct from two coordinates
    Position2D(double x, double y) : Vector2D(x, y) {}
	
    /// Construct from a coordinate vector
    Position2D(const Vector2D &vec) : Vector2D(vec) {};
    	
    /// Construct by reading from a string
    Position2D(char *string);

    /// Construct by transforming image positions into a camera system
    /** This transformation converts the pixel coordinates to coordinates in mm
        and corrects for the lens distortion.
        @param pixel Image position (coordinates in pixels)
        @param intor Interior orientation
    */
    Position2D(const PixelPosition *pixel, const InteriorOrientation *intor);	

    /// Copy constructor
    Position2D(const Position2D &pos) : Vector2D()
      { *this = pos; }	

    /// Destructor
    ~Position2D() {};

    /// Copy assignment
    Position2D & operator = (const Position2D &p)
      { x[0] = p.x[0]; x[1] = p.x[1];
        return(*this); } 
    
    // Not documented. Usage of const Position2DRef() const should be preferred.
    const Position2D & pos() const
        { return *this; } 
	
    /// Return the readable reference
    const Position2D & Position2DRef() const
        { return *this; } 
	
    // Not documented. Usage of Position2DRef() should be preferred.
    Position2D &pos() 
	{ return *this; }
	    	
    /// Return the writable reference
    Position2D & Position2DRef() 
	{ return *this; }
	    	
    /// Read position from a string
    void Read(char *string);

    /// Write position to a file
    /** The format used is "  %15.6f  %15.6f\n".
        @param fd File descriptor
    */
    void Write(FILE *fd) const;

    /// Print position to stdout
    void Print() const;

    /// Return the X-coordinate
    double GetX() const
      { return x[0];}

    /// Return the Y-coordinate
    double GetY() const
      { return x[1];}

     ///Swap X and Y
     void SwapXY()
      { double help=x[0];x[0]=x[1]; x[1]=help;}

    /// Distance between two positions
    double Distance(const Position2D &pos) const;
    
      ///Projection of a image position back to a plane in 3D space. 
      /**
         @param myplane a Plane in laser space
        @param ext Exterior paramters
        @param focal focal lense
        @param image_w image width, to transform from center origin image to left_top origin image
        */
   
    Position3D To3D(Plane myplane, ExteriorOrientation ext, double focal, int image_w, int image_h)  ;
    
    /// Distance between two positions
    friend double DistancePosition2DToPosition2D
      (const Position2D &pos1, const Position2D &pos2)
      {return(pos1.Distance(pos2));}

    /// Distance between two positions
    friend double Distance(const Position2D &pos1, const Position2D &pos2)
      { return (pos1.Distance(pos2)); }
      
  
};
#endif /* _Position2D_h_ */   /* Do NOT add anything after this line */
