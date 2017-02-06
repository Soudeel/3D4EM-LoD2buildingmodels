
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
 * \brief Interface to Class Positions2D - A set of positions in a two-dimensional coordinate system
 *
 */
/*!
 * \class Positions2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Positions2D - A set of positions in a two-dimensional coordinate system
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

#ifndef _Positions2D_h_
#define _Positions2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Positions2D  - A set of positions in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

#include <vector>
#include "Position2D.h"

//------------------------------------------------------------------------------
/// A vector of positions in a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Positions2D: public std::vector <Position2D>
{
  public:

    /// Default constructor
    Positions2D() : std::vector <Position2D>() {}

    /// Construct by reading from a file
    /** @param filename Name of the file with the 2D points
        @param success  Return status of the read function. 0 - failure, 
                        1 - success.
    */
    Positions2D(char *filename, int *success) : std::vector <Position2D>()
      {*success = Read(filename);}

    /// Default destructor
    ~Positions2D() {};

    /// Read positions from a file
    /** @param filename Name of the file with the 2D points
        @return Success status, 0 - failure, 1 - success
    */
    int Read(char *filename);

    /// Write positions to a file
    /** @param filename Name of the file for the 2D points
        @return Success status, 0 - failure, 1 - success
    */
    int Write(char *filename) const;

    /// Remove points with nearly the same coordinates
    /** @param distance Distance within which points should be merged. */
    void RemoveDoublePoints(double distance);
};
#endif /* _Positions2D_h_ */   /* Do NOT add anything after this line */
