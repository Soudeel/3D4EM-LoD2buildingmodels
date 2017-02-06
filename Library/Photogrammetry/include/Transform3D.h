
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
 * \brief Interface to Class Transform3D - Pose transformation in a three-dimensional coordinate system
 *
 */
/*!
 * \class Transform3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Transform3D - Pose transformation in a three-dimensional coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li A transformation of a pose in a 3D coordinate system. The transformation is defined by a translation vector and a rotataion.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Transform3D_h_
#define _Transform3D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Transform3D    - Pose transformation in a three-dimensional coordinate system

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Vector3D.h"
#include "Rotation3D.h"

//------------------------------------------------------------------------------
/// A transformation of a pose in a 3D coordinate system
/** The transformation is defined by a translation vector and a rotataion. */
//------------------------------------------------------------------------------

class Transform3D : public Vector3D, public Rotation3D {

  public:

    /// Default constructor
    Transform3D() : Vector3D(), Rotation3D() {}

    /// Default destructor
    ~Transform3D() {}
};
#endif /* _Transform3D_h_ */   /* Do NOT add anything after this line */
