
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
 * \brief Interface to Class PixelPositions - A set of positions in a digital image
 *
 */
/*!
 * \class PixelPositions
 * \ingroup Photogrammetry
 * \brief Interface to Class PixelPositions - A set of positions in a digital image
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A vector of positions in a digital image
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _PixelPositions_h_
#define _PixelPositions_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  PixelPositions  - A set of positions in a digital image

--------------------------------------------------------------------------------
*/

#include <vector>
#include "PixelPosition.h"

//------------------------------------------------------------------------------
/// A vector of positions in a digital image
//------------------------------------------------------------------------------

class PixelPositions : public std::vector<PixelPosition>
{
  public:

    /// Default constructor
    PixelPositions() : std::vector<PixelPosition>() {}

    /// Default destructor
    ~PixelPositions() {};
};
#endif /* _PixelPositions_h_ */   /* Do NOT add anything after this line */
