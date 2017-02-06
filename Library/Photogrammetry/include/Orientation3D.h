
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
 * \brief Interface to Class Orientation3D
 *
 */
/*!
 * \class Orientation3D
 * \ingroup POrientation
 * \brief Interface to Class Orientation3D
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li An orientation in a three dimensional coordinate system
 * This orientation consists of a translation and a rotation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Orientation3D_H_
#define _Orientation3D_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Vector3D, Position3D, Rotation and 
*	Orientation3D.
*
*--------------------------------------------------------------------*/

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <iostream>
#include <list>

#include "Database4Cpp.h"
#include "Rotation3D.h"

class Orientation3D;
class GenericRotation;

/// An orientation in a three dimensional coordinate system
/** This orientation consists of a translation and a rotation. */

class Orientation3D : public Vector3D, public Rotation3D {

public:
	/// Default constructor.
	Orientation3D() : Vector3D(), Rotation3D() {}

	/// Construct from pointers to a translation vector and a rotatation
	Orientation3D(const Vector3D *translation, const Rotation3D *rotation)
		: Vector3D(*translation), Rotation3D(*rotation) {}

	/// Construct froma translation vector and a rotation
	Orientation3D(const Vector3D &translation, const Rotation3D &rotation)
		: Vector3D(translation), Rotation3D(rotation) {}

	/// Copy constructor
	Orientation3D(const Orientation3D &o);

	/// Assignment operator
	const Orientation3D &operator = (const Orientation3D &o);

	/// Destructor.
	~Orientation3D(){}

	/// Convert from an exterior orientation (old C structure!)
	const Orientation3D &operator=(const Exterior &e);

        /// Returns a constant reference to the object
        const Orientation3D& OrientRef() const
                { return *this; }
      
        /// Returns a writeble reference to the object   
        Orientation3D& OrientRef()
              { return *this; }

	/// Combine two subsequent orientations
        /** The rotation matrices are multiplied as R_this * R_o.
            The translation is composed as R_this * T_o + T_this.
        */
	const Orientation3D &operator *= (const Orientation3D &o);

	/// Invert the transform.
        /** The rotation is transposed (= inverted = R^{-1}). The inverted
            translation is defined as -R^{-1} T.
        */
	Orientation3D Inverse() const;

        // Not documented. Function not present in CppInterface library.
	friend std::ostream &operator<<(std::ostream &os, const Orientation3D &p);

        /// Apply the orientation to a vector.
        /** The resulting vector is defined R1 * v2 + T1 */
	friend Vector3D operator *(const Orientation3D &o1, const Vector3D &v2);

        /// Combine two orientations.
        /** The rotation is defined by R1 * R2.
            The combined translation is composed as R1 * T2 + T1.
        */
	friend Orientation3D operator*(const Orientation3D &o1,
                                       const Orientation3D &o2);
};



#endif
