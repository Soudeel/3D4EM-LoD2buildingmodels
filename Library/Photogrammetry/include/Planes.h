
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
 * \brief Interface to Class Planes - A vector of planes
 *
 */
/*!
 * \class Planes
 * \ingroup Photogrammetry
 * \brief Interface to Class Planes - A vector of planes
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

#ifndef _Planes_h_
#define _Planes_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Planes            - A vector of planes

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <vector>
#include "Plane.h"

//------------------------------------------------------------------------------
/// A vector of planes
//------------------------------------------------------------------------------

class Planes : public std::vector<Plane> {

  public:
    /// Default constructor
    Planes() : std::vector<Plane>() {}
    
    /// Construct by reading file
    Planes(char *filename, int *success)
      { *success = Read(filename); }

    /// Default destructor
    ~Planes() {Erase();}

    /// Copy assignment
    Planes & operator = (const Planes &);
    
    /// Erase all planes
    void Erase();

    /// Read planes from a file
    /** @param filename Name of the file for the planes
        @return Success status. 0 - failure, 1 - success.
    */
    int Read(char *filename);
    
    /// Write planes to a file
    /** @param filename Name of the file for the planes
        @return Success status. 0 - failure, 1 - success.
    */
    int Write(char *filename) const;
};
#endif /* _Planes_h_ */   /* Do NOT add anything after this line */
