
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
 * \brief Interface to Class ControlPoints - A set of stochastic three-dimensional control points
 *
 */
/*!
 * \class ControlPoints
 * \ingroup Photogrammetry
 * \brief Interface to Class ControlPoints - A set of stochastic three-dimensional control points
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


#ifndef _ControlPoints_h_
#define _ControlPoints_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ControlPoints  - A set of stochastic three-dimensional control points

--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "ControlPoint.h"
#include "Database4Cpp.h"


//------------------------------------------------------------------------------
/// A vector of control points
//------------------------------------------------------------------------------

class ControlPoints : public VectorPoint<ControlPoint>
{
  public:

    /// Default constructor
    ControlPoints() : VectorPoint<ControlPoint>() {}

    /// Construct by converting a C object
    ControlPoints(CtrlPts *ctrlpts);

    /// Copy constructor
    ControlPoints(const ControlPoints &pts) : VectorPoint<ControlPoint>()
        { insert(begin(), pts.begin(), pts.end()); }

    /// Destructor
    ~ControlPoints() {};

    /// Copy assignment
    ControlPoints& operator=(const ControlPoints& pts)
    {
      if (!empty()) erase(begin(), end());
      insert(begin(), pts.begin(), pts.end());
      return *this;
    }

    /// Conversion from C to C++ object
    void C2Cpp(CtrlPts *);

    /// Conversion from C++ to C object
    void Cpp2C(CtrlPts **) const;

    /// Read control points from a file
    int Read(char *filename);

    /// Write control points to a file
    int Write(char *filename) const;

    /// Print the control points to stdout
    void Print() const;
};
#endif /* _ControlPoints_h_ */   /* Do NOT add anything after this line */
