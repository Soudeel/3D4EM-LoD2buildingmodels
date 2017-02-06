
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
 * \brief Interface to Class ControlPoint - A control point in an object coordinate system
 *
 */
/*!
 * \class ControlPoint
 * \ingroup Photogrammetry
 * \brief Interface to Class ControlPoint - A control point in an object coordinate system
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


#ifndef _ControlPoint_h_
#define _ControlPoint_h_

/*
--------------------------------------------------------------------------------
  ControlPoint  - A control point in an object coordinate system
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Point3DPlain.h"
#include "Database4Cpp.h"

//------------------------------------------------------------------------------
/// Control point in an object coordinate system
//------------------------------------------------------------------------------

class ControlPoint : public Point3DPlain {

  protected:
    /// Kind of control point (full, planimetric, height, or none)
    int status;

  public:

    /// Default constructor
    ControlPoint() : Point3DPlain() {}

    /// Construct from coordinates, number and status
    ControlPoint(double x, double y, double z, int num, int stat)
      : Point3DPlain(x, y, z, num)
        { status = stat; } 
	  
    /// Construct from coordinate vector and number
    ControlPoint(const Vector3D &vec, const PointNumber &n)
      : Point3DPlain(vec, n) {}

    /// Construct by converting a C object
    ControlPoint(CtrlPt *ctrlpt) : Point3DPlain()
      { C2Cpp(ctrlpt); }

    /// Copy constructor
    ControlPoint(const ControlPoint &pt) : Point3DPlain()
      { *this = pt; } 

    /// Destructor
    ~ControlPoint() {};

    /// Copy assignment
    ControlPoint & operator = (const ControlPoint & point);
      
    /// Conversion from C to C++ object
    void C2Cpp(CtrlPt *);

    /// Conversion from C++ to C object
    void Cpp2C(CtrlPt **) const;
};
#endif /* _ControlPoint_h_ */   /* Do NOT add anything after this line */
