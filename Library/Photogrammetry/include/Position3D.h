
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
 * \brief Interface to Class Position3D - A position in a three-dimensional coordinate system
 *
 */
/*!
 * \class Position3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Position3D - A position in a three-dimensional coordinate system
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

#ifndef _Position3D_h_
#define _Position3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Position3D              - A position in a three-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "Vector3D.h"
#include "Position2D.h"
#include "ExteriorOrientation.h"

class Plane;

//------------------------------------------------------------------------------
/// A position in a three-dimensional coordinate system
//------------------------------------------------------------------------------

class Position3D : public Vector3D {

  public:

    /// Default constructor
    Position3D() : Vector3D() {}

    /// Construct from three coordinates
    Position3D(double x, double y, double z) : Vector3D(x, y, z) {}

    /// Construct from a coordinate vector
    Position3D(const Vector3D &v) : Vector3D(v) {};

    /// Construct by reading from a string
    Position3D(char *string);

    /// Copy constructor
    Position3D(const Position3D &pos) : Vector3D()
      { *this = pos; }	

    /// Destructor
    ~Position3D() {};

    /// Copy assignment
    Position3D & operator = (const Position3D &p)
      { x[0] = p.x[0]; x[1] = p.x[1]; x[2] = p.x[2]; 
        return(*this); } 

    // Not documented. Usage of const &Position3DRef() const is preferred.
    const Position3D &pos() const
      { return *this; }

    /// Return the readable reference
    const Position3D &Position3DRef() const
      { return *this; }

    // Not documented. Usage of &Position3DRef() is preferred.
    Position3D &pos()
      { return *this; }

    /// Return the writable reference
    Position3D &Position3DRef()
      { return *this; }

    /// Return a 2D position
    Position2D Position2DOnly() const
      { return(Position2D(X(), Y())); }

    /// Read the position from a string
    void Read(char *string);

    /// Write the position to a file
    /** The coordinates are writtin in the format "  %15.6f  %15.6f  %15.6f\n".
        @param fd File descriptor
    */
    void Write(FILE *fd) const;

     /// Return the X-coordinate
    double GetX() const
    	{ return x[0];}
    	  
     /// Return the Y-coordinate
    double GetY() const
    	{ return x[1];}

     /// Return the Z-coordinate
    double GetZ() const
    	{ return x[2];}
    	  
    /// Set the X-coordinate
    void SetX(double x1)
    	{ x[0] = x1;}
    	  
    /// Set the Y-coordinate
    void SetY(double y)
    	{ x[1] = y;}

    /// Set the Z-coordinate
    void SetZ(double z)
    	{ x[2] = z;}

    /// Distance between two positions
    double Distance(const Position3D &) const;

	/// 2D distance between two positions
	double Distance2D(const Position3D &pos2) const;


    /// Distance between two positions
    friend double DistancePosition3DToPosition3D
      (const Position3D &pos1, const Position3D &pos2)
      {return(pos1.Distance(pos2));}

    /// Distance between two positions
    friend double Distance(const Position3D &pos1, const Position3D &pos2)
      {return(pos1.Distance(pos2));}

    /// Distance of a 3D position to a plane
    double Distance(const Plane &plane) const;

    /// Write a position to a VRML file
    /** @param fd File descriptor of VRML file */
    void VRML_Write(FILE *fd) const;

    /// Write a position as a cross to a VRML file
    /** @param fd   File descriptor of VRML file
        @param size Length of the 8 legs of the cross
    */
    void VRML_Write_Cross(FILE *fd, double size) const;

    /// Write a position as a sphere to a VRML file
    /** @param fd File descriptor of VRML file
        @param radius Radius of the sphere
    */
    void VRML_Write_Sphere(FILE *fd, double radius) const;
    
    ///Calculate the image position based on Exterior Orientation
    Position2D ToPixel(ExteriorOrientation ext,double focal) const;

    /// Check if the location is the same
    bool SameLocation(const Position3D &pos) const;
};
#endif /* _Position3D_h_ */   /* Do NOT add anything after this line */
