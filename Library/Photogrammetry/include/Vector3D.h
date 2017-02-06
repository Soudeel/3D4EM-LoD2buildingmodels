
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
 * \brief Interface to Class Vector3D - A vector in a three-dimensional coordinate system
 *
 */
/*!
 * \class Vector3D
 * \ingroup Photogrammetry
 * \brief Interface to Class Vector3D - A vector in a three-dimensional coordinate system
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define base geometries, Vector3D.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _VECTOR3D_H_
#define _VECTOR3D_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Vector3D.
*
*
*--------------------------------------------------------------------*/

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <strings.h>
#include <iostream>
#include <fstream> // necessary, because it includes <iosfwd>, defining istream
                   // as basic_istream<char>, required for insertion operator

#include "Vector2D.h"

#define W_DIM 3			/* world dimension */

class Vector2D;
class Orientation3D;
class Rotation3D;

/// A vector in a three-dimensional coordinate system

class Vector3D {
//friend class Rotation3D;

protected:
	/// The vector elements.
	double x[W_DIM];

public:
	/// Default constructor.
	Vector3D() { x[0] = x[1] = x[2] = 0; }
	
	/// Constructor with initialisation from coordinates.
	Vector3D(double xv, double yv, double zv) {
	  x[0] = xv; x[1] = yv; x[2] = zv;
	}

	/// Constructor with initialisation from a double array.
	Vector3D(const double *v) { x[0] = v[0]; x[1] = v[1]; x[2] = v[2]; }

	/// Copy constructor.
	Vector3D(const Vector3D &v) {
		x[0] = v.x[0]; x[1] = v.x[1]; x[2] = v.x[2];
	}

	/// Construct from 2D vector and z-coordinate.
	Vector3D(const Vector2D &v, double z);

	/// Destructor.
	~Vector3D() {}

	/// Member access, readonly.
        /** @param i Index of vector element, should be 0, 1, or 2. */
	double X(int i) const { return x[i]; }

	/// Member access, readonly.
	double X() const { return x[0]; }

	/// Member access, readonly.
	double Y() const { return x[1]; }

	/// Member access, readonly.
	double Z() const { return x[2]; }

	/// Member access, readonly.
        /** @param i Index of vector element, should be 0, 1, or 2. */
	double operator[] (int i) const { return x[i]; }

	/// Member access, writable.
        /** @param i Index of vector element, should be 0, 1, or 2. */
	double &X(int i) { return x[i]; }

	/// Member access, writable.
	double &X() { return x[0]; }

	/// Member access, writable.
	double &Y() { return x[1]; }

	/// Member access, writable.
	double &Z() { return x[2]; }

	/// Member access, writable.
        /** @param i Index of vector element, should be 0, 1, or 2. */
	double &operator[] (int i) { return x[i]; }

	/// Partial derivative of a translation with respect to vector element i.
        /** @param i Index of vector element, should be 0, 1, or 2. */
    	Vector3D PartialDeriv(int i) const;

	/// Squared length
	double SqLength() const;

	/// Euclidian length
	double Length() const;

	/// Squared length of first two elements
	double SqLength2D() const;

	/// Euclidian length of first two elements.
	double Length2D() const;

	/// divide by Euclidian length.
	Vector3D Normalize() const;

	/// Return the const reference to the vector
	const Vector3D &vect() const { return *this; }

	/// Return the writable reference to the vector
	Vector3D &vect() { return *this; }

	/// Assignment operator
	Vector3D &operator = (const Vector3D &v);

	/// Outer product with another vector.
	Vector3D VectorProduct(const Vector3D &v) const;

	/// Inner product with another vector.
	double DotProduct(const Vector3D &v) const;

	/// Returns vector 'v' projected onto 'this' vector.
	Vector3D Align(const Vector3D &v) const;

	/// Returns the index which contains the (absolute) biggest value.
	int AbsLargest() const;

	/// Compute two vectors that are perpendicular to each other and this one.
        /** Before invoking this function, be sure that 'this' is normalised. */
	void PerpendicularVectors(Vector3D &xv, Vector3D &yv) const;

	/// Add another 3D vector
	const Vector3D &operator += (const Vector3D &v);

	/// Substract another 3D vector
	const Vector3D &operator -= (const Vector3D &v);

	/// Multiply with a scalar
	const Vector3D &operator *= (double d);

	/// Pre-multiply with a 3D rotation matrix
	const Vector3D &operator *= (const Rotation3D &r);

	/// Pre-multiply with the rotation of an 3D orientation
	const Vector3D &operator *= (const Orientation3D &o);

	/// Pre-multiply with the rotation of an 3D orientation
	Vector3D &operator *= (Orientation3D &o);

	/// Divide by a scalar
	const Vector3D &operator /= (double d);

	/// Add two 3D vectors
	friend Vector3D operator +(const Vector3D &v1, const Vector3D &v2);

	/// Subtract two 3D vectors
	friend Vector3D operator -(const Vector3D &v1, const Vector3D &v2);

	/// Multiply a vector with a scalar
	friend Vector3D operator *(const Vector3D &v, double d);

	/// Multiply a scalar with a vector
	friend Vector3D operator *(double d, const Vector3D &v);

	/// Divide a vector by a scalar
	friend Vector3D operator /(const Vector3D &v, double d);

	/// Multiply a 3D rotation matrix with a 3D vector
	friend Vector3D operator *(const Rotation3D &r, const Vector3D &v);

	/// Multiply a rotation matrix of a 3D orientation with a 3D vector
	friend Vector3D operator *(const Orientation3D &o, const Vector3D &v);

	/// Multiply a rotation matrix of a 3D orientation with a 3D vector
	friend Vector3D operator *(Orientation3D &o, Vector3D &v);

        /// Test if two vectors are identical
	friend bool operator ==(const Vector3D &v1, const Vector3D &v2);
	
	friend bool operator <(const Vector3D &v1, const Vector3D &v2);

	/// Add another 3D vector
	Vector3D Add(const Vector3D &v) const
	  {return(*this + v);}

/* Additions outside STW project */

        /// Return the direction in the XOY-plane from 'this' to 'dest' in radians
        double Direction2D(const Vector3D &dest) const;

        /// Return the angle between p1, this and p2 in radians in the XOY-plane
        double Angle2D(const Vector3D &p1, const Vector3D &p2) const;

        /// Create a 2D vector from the first two elements of the 3D vector
       const Vector2D vect2D() const
         { return Vector2D(x[0], x[1]); }

       /// Angle (in radians) between two vectors
       friend double Angle(const Vector3D &v1, const Vector3D &v2);

 
        /* Additions for Piper */

	///Construct from angles
	Vector3D(double phi, double theta)  {
	  x[0] = cos(phi)*sin(theta);
	  x[1] = sin(phi)*sin(theta);
	  x[2] = cos(theta);
	}

	/// Print the vector
	void PrintVector();
	
	/// Check if vector is a unit vector
	bool Unity();

	/// Summation of all elements
	double Sum() const;

	/// Maximum element
	double AbsMax(int* element) const;

	/// Input operator
	friend std::istream &operator>>(std::istream &is, Vector3D &v)
	{ return is >> v.x[0] >> v.x[1] >> v.x[2]; }

	friend std::ostream &operator<<(std::ostream &os, const Vector3D &v);
	//{ return os << v.x[0] << ' ' << v.x[1] << ' ' << v.x[2]; }

    /// Check if a vector is horizontal
    /** @param angle_tolerance Tolerance in radians
        @return true if horizontal
    */
    bool IsHorizontal(double angle_tolerance) const;
 };


#endif
