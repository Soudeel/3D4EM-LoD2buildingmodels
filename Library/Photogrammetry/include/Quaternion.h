
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
 * \brief Interface to Class Quaternion -  Quaternion parameterisation of a rotation matrix
 *
 */
/*!
 * \class Quaternion
 * \ingroup LinearAlgebra
 * \brief Interface to Class Quaternion -  Quaternion parameterisation of a rotation matrix
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A quaternion (with q0 = 1)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Quaternion_h_
#define _Quaternion_h_


#include "ExteriorOrientation.h"
#include "AbsoluteOrientation.h"

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Quaternion     -  Quaternion parameterisation of a rotation matrix

--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
/// A quaternion (with q0 = 1)
//------------------------------------------------------------------------------

class Quaternion {

  protected:

    /// The quaternion elements q1, q2 and q3
    double q[3];

  public:

    /// Default constructor
    Quaternion() {};

    /// Construct quaternion from the rotation of an exterior orientation
    Quaternion(const ExteriorOrientation &extor);   
    
    /// Construct quaternion from the rotation of an absolute orientation
    Quaternion(const AbsoluteOrientation &absor);   

    /// Default destructor
    ~Quaternion() {};

    /// Get a quaternion element (index 0 delivers q1, etc.)
    double GetQuaternion(int index) const
      { return q[index]; }
};
#endif /* _Quaternion_h_ */   /* Do NOT add anything after this line */
