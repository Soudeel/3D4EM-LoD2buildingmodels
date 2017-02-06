
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



#ifndef _VECTOR3D_F_H_
#define _VECTOR3D_F_H_ 
/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : March 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A float version of Vector3Df to save sapce.
*
*--------------------------------------------------------------------*/

#include <limits.h>

#include "Vector2D.h"

#define W_DIM 3			/* world dimension */

class Vector2D;
//class Rotation3D;
class Orientation3D;
//class GenericRotation;

/// 3D vector.
class Vector3Df {
friend class Rotation3D;

protected:
	/// vector elements.
	float x[W_DIM];

public:
	/// Default constructor.
	Vector3Df() { x[0] = x[1] = x[2] = 0; }
	
	/// Constructor with initialization.
	Vector3Df(double x1, double y, double z) {
		x[0] = x1; x[1] = y; x[2] = z;
	}

	/// Constructor with initialization.
	Vector3Df(const double *p) { x[0] = p[0]; x[1] = p[1]; x[2] = p[2]; }

	/// Copy constructor.
	Vector3Df(const Vector3Df &p) {
		x[0] = p.x[0]; x[1] = p.x[1]; x[2] = p.x[2];
	}

	/// Convert 2d vector to 3d. (z=0)
	Vector3Df(const Vector2D &v, double z);
	
	Vector3Df(const Vector3D& v);

	/// Destructor.
	~Vector3Df() {}

/*	/// Copy assignment
	Vector3Df& operator=(const Vector3Df &v)
		{ x[0] = v.x[0]; x[1] = v.x[1]; x[2] = v.x[2];
			return *this;
		}
*/
	Vector3Df &operator = (const Vector3Df &p);
	
	/// Member access.
	double X(int i) const { return x[i]; }

	/// Member access.
	double X() const { return x[0]; }

	/// Member access.
	double Y() const { return x[1]; }

	/// Member access.
	double Z() const { return x[2]; }

	/// Member access.
	double operator[] (int i) const { return x[i]; }

	/// Member access, writable.
	float& X(int i) { return x[i]; }

	/// Member access, writable.
	float& X() { return x[0]; }

	/// Member access, writable.
	float&Y() { return x[1]; }

	/// Member access, writable.
	float&Z() { return x[2]; }

	/// Member access, writable.
	float&operator[] (int i) { return x[i]; }

	/// length of first two elements.
	double Length2D() const;

	/// Partial derivative of a translation with respect to param.
    Vector3Df PartialDeriv(int param) const;

	/// Squared length
	double SqLength() const;

	/// Euclidian length
	double Length() const;

	/// divide by Euclidian length.
	Vector3Df Normalize() const;

	/// Usefull for classes that inherit Vector3Df.
	const Vector3Df &vect() const { return *this; }

	/// Usefull for classes that inherit Vector3Df.
	Vector3Df &vect() { return *this; }

	/// outer product.
	Vector3Df VectorProduct(const Vector3Df &q) const;

	/// inner product.
	double DotProduct(const Vector3Df &q) const;

	/// Returns 'vec' projected onto 'this'.
	Vector3Df Align(const Vector3Df &vec) const;

	/// Returns the index which contains the (absolute) biggest value.
	int AbsLargest() const;

	/** Compute two vectors that are perpendicular to each other and this one.
		Be sure 'this' is normalized.
	*/
	void PerpendicularVectors(Vector3Df &xv, Vector3Df &yv) const;

	///
	const Vector3Df &operator += (const Vector3Df &p);

	///
	const Vector3Df &operator -= (const Vector3Df &p);

	///
	const Vector3Df &operator *= (double d);

	///
	const Vector3Df &operator *= (const Rotation3D &p);

	///
	const Vector3Df &operator *= (const Orientation3D &o);

	///
	Vector3Df &operator *= (Orientation3D &o);

	///
	const Vector3Df &operator /= (double d);

	///
	friend Vector3Df operator +(const Vector3Df &p1, const Vector3Df &p2);

	///
	friend Vector3Df operator -(const Vector3Df &p1, const Vector3Df &p2);

	///
	friend Vector3Df operator *(const Vector3Df &p1, double d);

	///
	friend Vector3Df operator /(const Vector3Df &p1, double d);

	///
	friend Vector3Df operator *(const Rotation3D &r1,const Vector3Df &p1);

	///
	friend Vector3Df operator *(const Orientation3D &o1, const Vector3Df &p2);

	///
	friend Vector3Df operator *(Orientation3D &o1, Vector3Df &p2);

        ///
	friend bool operator ==(const Vector3Df &p1, const Vector3Df &p2);

/* Additions outside STW project */

        /// Direction from this to pto
        double Direction2D(const Vector3Df &pto) const;

        /// Angle between p1, this and p2
        double Angle2D(const Vector3Df &p1, const Vector3Df &p2) const;

       const Vector2D vect2D() const
         { return Vector2D(x[0], x[1]); }

//       Vector2D vect2D()
//         { return Vector2D(x[0], x[1]); }

       /// Angle (in radians) between two vectors
       friend double Angle(const Vector3Df &v1, const Vector3Df &v2);

	/// Phi and kappa angle of vector
	/// Phi is angle between Vector and x-axes, kappa between z and vector
//	Vector2D Angles() const;

/* Additions for Piper */

	///Construct from angles
	Vector3Df(double phi, double theta)  {
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
	
	double SqLength2D() const ;
	
	

};

/// output operator
std::ostream &operator<<(std::ostream &os, const Vector3Df &p);

/** */
//typedef Vector3Df Position3D;


#endif
