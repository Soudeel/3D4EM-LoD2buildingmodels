
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
 * \brief Interface to Class Lines2D - A vector of 2D lines
 *
 */
/*!
 * \class Lines2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Lines2D - A vector of 2D lines
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

#ifndef _Lines2D_h_
#define _Lines2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Lines2D            - A vector of 2D lines

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Line2D.h"

//------------------------------------------------------------------------------
/// A vector of two-dimensional lines
//------------------------------------------------------------------------------

class Lines2D : public std::vector<Line2D> {

  public:
    /// Default constructor
    Lines2D() : std::vector<Line2D>() {}

    /// Default destructor
    ~Lines2D()
      {if (!empty()) erase(begin(), end());}

    /// Copy assignment
    Lines2D & operator = (const Lines2D &);
};

#endif /* _Lines2D_h_ */   /* Do NOT add anything after this line */
