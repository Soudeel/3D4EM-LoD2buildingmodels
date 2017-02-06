
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
 * \brief Interface to Class Points2D - A set of stochastic two-dimensional points
 *
 */
/*!
 * \class Points2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Points2D - A set of stochastic two-dimensional points
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

#ifndef _Points2D_h_
#define _Points2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Points2D  - A set of stochastic two-dimensional points

--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "Point2D.h"
#include "Database4Cpp.h"


//------------------------------------------------------------------------------
/// A vector of stochastic two-dimensional points
//------------------------------------------------------------------------------

class Points2D : public VectorPoint<Point2D>
{
  public:

    /// Default constructor
    Points2D() : VectorPoint<Point2D>() {}

    /// Default destructor
    ~Points2D() {}
};
#endif /* _Points2D_h_ */   /* Do NOT add anything after this line */
