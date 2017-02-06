
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
 * \brief Interface to Class Vector2D - A vector in a two-dimensional coordinate system
 *
 */
/*!
 * \class Vector2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Vector2D - A vector in a two-dimensional coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define base geometries, Vector2D.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _VECTOR2D_H_
#define _VECTOR2D_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Vector2D.
*
*
*--------------------------------------------------------------------*/

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <strings.h>
#include <iostream>

class Vector3D;

/// A vector in a two-dimensional coordinate system
class Vector2D {

protected:
        /// The two elements of the vector
	double x[2];

public:
	/// Default constructor.
	Vector2D() { x[0] = x[1] = 0; }
	
	/// Constructor with initialization.
	Vector2D(double xv, double yv) {
		x[0] = xv; x[1] = yv;
	}

	/// Constructor with initialization.
	Vector2D(const double *v) { x[0] = v[0]; x[1] = v[1]; }

	/// Construct from Vector3D (ignore Z).
	Vector2D(const Vector3D &v);

	/// Copy constructor.
	Vector2D(const Vector2D &v) { x[0] = v.x[0]; x[1] = v.x[1]; }
/*
	/// Assignment operator.
	const Vector2D &operator = (const Vector2D &p) {
		x[0] = p.x[0]; x[1] = p.x[1]; return *this; }
*/
	/// Destructor.
	~Vector2D() {}

        /// Copy assignament
        Vector2D& operator=(const Vector2D &v)
          { x[0] = v.x[0]; x[1] = v.x[1]; 
            return *this;
          }

	/// Element access, readonly.
        /** @param i Index, should be either 0 or 1. */
	double X(int i) const { return x[i]; }

	/// Element access, readonly.
	double X() const { return x[0]; }

	/// Element access, readonly.
	double Y() const { return x[1]; }

	/// Element access, readonly.
	double operator[] (int i) const { return x[i]; }

	/// Element access, writable.
        /** @param i Index, should be either 0 or 1. */
	double &X(int i) { return x[i]; }

	/// Element access, writable.
	double &X() { return x[0]; }

	/// Element access, writable.
	double &Y() { return x[1]; }

	/// Element access, writable.
	double &operator[] (int i) { return x[i]; }

	/// Squared length.
	double SqLength() const;

	/// Euclidian length.
	double Length() const;

	/// Partial derivative of a translation with respect to a coordinate.
        /** @param param Coordinate index, should be eiter 0 or 1. */
	Vector2D PartialDeriv(int param) const;

	/// Return a const reference to the vector
	const class Vector2D &vect() const { return *this; }

	/// Return a reference to the vector
	Vector2D &vect() { return *this; }

	/// Convert a Vector3D to a Vector2D (ignore Z).
	const Vector2D &operator = (const Vector3D &v);

	/// Inner product with another vector.
	double DotProduct(const Vector2D &v) const;

	/// Returns vector 'v' projected onto 'this' vector.
	Vector2D Align(const Vector2D &v) const;

        /// Checks whether a point (this) is left or right of a line defined by two points
        int AreaSign(const Vector2D &p1, const Vector2D &p2) const;

        /// Add another 2D vector
	const Vector2D &operator += (const Vector2D &v);

        /// Subtract another 2D vector
	const Vector2D &operator -= (const Vector2D &v);

        /// Multiply vector with a scalar
	const Vector2D &operator *= (double d);

        /// Divide vector by a scalar
	const Vector2D &operator /= (double d);

        /// Add two 2D vectors
	friend Vector2D operator +(const Vector2D &v1, const Vector2D &v2);

        /// Subtract two 2D vectors
	friend Vector2D operator -(const Vector2D &v1, const Vector2D &v2);

        /// Multiply a 2D vector by a scalar
	friend Vector2D operator *(const Vector2D &v1, double d);
	friend Vector2D operator *(double d, const Vector2D &v1);

        /// Divide a 2D vector by a scalar
	friend Vector2D operator /(const Vector2D &v1, double d);

        /// Test if two 2D vectors are identical
	friend bool operator == (const Vector2D &v1, const Vector2D &v2);
	friend bool operator != (const Vector2D &v1, const Vector2D &v2) { return !(v1==v2); }
        /// Return the angle between two vectors.
        /** The angle is in radians
            between -pi and +pi and is positive in counterclockwise direction.
        */
	friend double Angle2D(const Vector2D &v1, const Vector2D &v2);

        /// Construct the normal vector to a vector
        Vector2D Normal() const;
};

#endif
