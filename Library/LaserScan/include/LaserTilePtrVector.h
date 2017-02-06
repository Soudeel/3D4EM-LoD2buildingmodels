
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
 * \brief Interface to Class LaserTilePtrVector
 *
 */
/*!
 * \class LaserTilePtrVector
 * \ingroup LDataOrg
 * \brief Interface to Class LaserTilePtrVector
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Vector of pointers to laser tiles (class LaserSubUnit)
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _LaserTilePtrVector_h_
#define _LaserTilePtrVector_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserTilePtrVector             - Vector of pointers to laser tiles

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserSubUnit.h"

//------------------------------------------------------------------------------
///                  Vector of pointers to laser tiles
//------------------------------------------------------------------------------

class LaserTilePtrVector : public std::vector<LaserSubUnit *>
{
  public:

    /// Default constructor
    LaserTilePtrVector() {};

    /// Default destructor
    ~LaserTilePtrVector() {};

    /// Return the reference
    LaserTilePtrVector & LaserTilePtrVectorRef()
      {return(*this);}

    /// Return the const reference
    const LaserTilePtrVector & LaserTilePtrVectorRef() const
      {return(*this);}

    /// Copy assignment
    LaserTilePtrVector operator = (const LaserTilePtrVector &tileptrvector);
    
    /// Check if the vector contains a tile pointer
    bool Contains(LaserSubUnit *tile) const;
    
    /// Erase the tile pointers in the vector
    void Erase()
      {if (!empty()) erase(begin(), end());}
};

#endif /* _LaserTilePtrVector_h_ */  /* Don't add after this point */
