
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
 * \brief Interface to Class Positions3D - A set of positions in a three-dimensional coordinate system
 *
 */
/*!
 * \class Positions3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Positions3D - A set of positions in a three-dimensional coordinate system
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

#ifndef _Positions3D_h_
#define _Positions3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Positions3D  - A set of positions in a three-dimensional coordinate system

--------------------------------------------------------------------------------
*/

#include <vector>
#include "Position3D.h"


//------------------------------------------------------------------------------
/// A vector of positions in a three-dimensional coordinate system
//------------------------------------------------------------------------------

class Positions3D: public std::vector <Position3D>
{
  public:

    /// Default constructor
    Positions3D() : std::vector <Position3D>() {}

    /// Construct by reading from a file
    /** @param filename Name of the file with 3D positions
        @param success  Status of the reading function. 0 - failure, 1 - success
    */
    Positions3D(char *filename, int *success) : std::vector <Position3D>()
      {*success = Read(filename);}

    /// Default destructor
    ~Positions3D() {};

    /// Return the writable reference
    Positions3D &Positions3DRef()
      {return(*this);}

    /// Return the readable reference
    const Positions3D &Positions3DRef() const
      {return(*this);}

    /// Read positions from a file
    /** @param filename Name of the file with 3D positions
        @return Success status. 0 - failure, 1 - success.
    */
    int Read(char *filename);

    /// Write positions to a file
    /** @param filename Name of the file for the 3D positions
        @return Success status. 0 - failure, 1 - success.
    */
    int Write(char *filename) const;
};
#endif /* _Positions3D_h_ */   /* Do NOT add anything after this line */
