
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
 * \brief Interface to Class PointNumberLists - A vector of point number lists
 *
 */
/*!
 * \class PointNumberLists
 * \ingroup Photogrammetry
 * \brief Interface to Class PointNumberLists - A vector of point number lists
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

#ifndef _PointNumberLists_h_
#define _PointNumberLists_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  PointNumberLists  - A vector of point number lists

--------------------------------------------------------------------------------
*/


#include "PointNumberList.h"


//------------------------------------------------------------------------------
///                 A set of vectors with point numbers
//------------------------------------------------------------------------------

class PointNumberLists : public std::vector<PointNumberList>
{
  public:

    /// Default constructor
    PointNumberLists() : std::vector<PointNumberList>() {}

    /// Default destructor
    ~PointNumberLists() {Erase();}

    /// Copy assignment
    PointNumberLists & operator = (const PointNumberLists &);
    
    /// Erase all lists
    void Erase();
};
#endif /* _PointNumberLists_h_ */   /* Do NOT add anything after this line */
