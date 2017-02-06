
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
 * \brief Interface to Class Rotation2D - A rotation matrix in a two-dimensional coordinate system
 *
 */
/*!
 * \class Rotation2D
 * \ingroup LinearAlgebra
 * \brief Interface to Class Rotation2D - A rotation matrix in a two-dimensional coordinate system
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


#ifndef _Rotation2D_h_
#define _Rotation2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Rotation2D         - A rotation matrix in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
/// A rotation matrix for a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Rotation2D {

  protected:

    /// The elements of the rotation matrix
    double r[2][2];

  public:

    /// Default constructor
    Rotation2D() {};

    /// Default destructor
    ~Rotation2D() {};

    /// Trace of rotation matrix.
    double Trace() const { return r[0][0] + r[1][1]; }

    /// Element access, read only.
    double R(int row, int column) const { return r[row][column]; }

    /// Element access, writable
    double &R(int row, int column) { return r[row][column]; }
};
#endif /* _Rotation2D_h_ */   /* Do NOT add anything after this line */
