
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
 * \brief Interface to Class GenericRotation
 *
 */
/*!
 * \class GenericRotation
 * \ingroup LinearAlgebra
 * \brief Interface to Class GenericRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ROTATIONPARAMETER_H_
#define _ROTATIONPARAMETER_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define rotation parametrizations.
*	Orientation.
*
*--------------------------------------------------------------------*/

#define M_PI           3.14159265358979323846  /* pi */

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <iostream>

//#include "parameter.h"
#include "Rotation3D.h"

class Parameter;
class EulerRotation;
class AngleAxisRotation;
class VectorRotation;
class QuaternionRotation;
class TrinionRotation;


/** Type of Orientation representation.
	Possible representations are:
		Euler, AngleAxis, Vector, Vector2, Quaternion, Trinion
*/
enum RotationType {
	Euler, AngleAxis, Vector, Vector2, Quaternion, Trinion };

typedef enum RotationType RotationType;


/// The abstract base class for the parameterised rotations.

class GenericRotation {
protected:
public:
	/// Default constructor.
	GenericRotation() {};

	/// Destructor.
	virtual ~GenericRotation(){}

	/// Element access, read only.
	virtual double operator[](int i) const = 0;

	/// Element access, writable.
	virtual double &operator[](int i) = 0;

	/// Returns the number of parameters
	virtual int DOF() const = 0;

	/// Returns the type of rotation.
	virtual RotationType Type() const = 0;

	/// Returns the name of the parameter.
	virtual const char *Name(int i) const = 0;

	/// What is the 'length' of this representation.
	virtual double Length() const = 0;

	/// Conversion from rotation matrix.
	virtual void from_matrix(const Rotation3D &r) = 0;

	/// Conversion to rotation matrix.
	virtual Rotation3D to_matrix(Rotation3D &r) const = 0;

	/// Conversion to rotation matrix.
	virtual Rotation3D to_matrix() const = 0;

	/// Check if parameters obey parameterization, adjust if necessary/possible.
	virtual int Check() = 0;

	/// Partial derivative with respect to one of the parameters.
	virtual Rotation3D PartialDeriv(int param) const = 0;
};

/** Copy a rotation parametrization. */
GenericRotation *CopyRotation(const GenericRotation &rot);


//------------------------------------------------------------------------------
/*!
 * \class EulerRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class EulerRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Rotation parameterised with three Euler angles (omega, phi, kappa)
class EulerRotation : public GenericRotation {
protected:
	/// First rotation angle around the X-axis
	double omega;
	/// Second rotation angle around the Y-axis
	double phi;
	/// Third rotation angle around the Z-axis
	double kappa;

public:
	/// Default constructor.
	EulerRotation();

	/// Construct from specified angles
	EulerRotation(double omega, double phi, double kappa);

	/// Construct from angles specified in a double array
	EulerRotation(const double *angles);

	/// Construct from a rotation matrix
	EulerRotation(const Rotation3D &rot);

	/// Copy constructor
	EulerRotation(const EulerRotation &euler) : GenericRotation()
          { *this = euler; }

	/// Default destructor
	~EulerRotation(){}

	/// Element access.
	double operator[](int i) const;

	/// Element access, writable.
	double &operator[](int i);

	/// Returns the number of parameters
	int DOF() const { return 3; }

	/// Returns the type of rotation representation (Euler in this case)
	RotationType Type() const { return Euler; }

	/// Returns the name of the parameter.
	const char *Name(int i) const;

	/// What is the 'length' of this representation.
        /** For Euler angles, this length is meaningless. It is the sum of
            the three angles in radians.
        */
	double Length() const { return kappa + omega + phi; }

	/// Extract the Euler angles from a rotation matrix
	void from_matrix(const Rotation3D &r);

	/// Convert the Euler angles to a rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Convert the Euler angles to a rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assignment operator. Conversion from matrix representation.
	EulerRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check() { return 0; }

	/// Partial derivative with respect to one of the parameters.
	Rotation3D PartialDeriv(int param) const;
};


//------------------------------------------------------------------------------
/*!
 * \class AngleAxisRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class AngleAxisRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Rotation parameterised by a rotation axis and an angle.

class AngleAxisRotation : public GenericRotation , public Vector3D {

protected:
	/// Rotation angle.
	double length;

public:
	/// Default constructor.
	AngleAxisRotation();

	/// Construct from an axis vector and an angle
	AngleAxisRotation(const Vector3D &axis, double angle);

	/// Construct from an axis vector and angle in a double array
	AngleAxisRotation(const double *d);

	/// Construct from a rotation matrix
	AngleAxisRotation(const Rotation3D &rot);

	/// Copy constructor
	AngleAxisRotation(const AngleAxisRotation &a) :
          GenericRotation(), Vector3D()
          { *this = a; }

	/// Destructor
	~AngleAxisRotation(){}

	/// Element access, readable
	double operator[](int i) const { return (i==3)? length: x[i]; }

	/// Element access, writable
	double &operator[](int i) { return (i==3)? length: x[i]; }

	/// Returns the number of parameters
	int DOF() const { return 4; }

	/// Returns the type of rotation representation (AngleAxis in this case)
	RotationType Type() const { return AngleAxis; }

	/// Returns the name of the parameter.
	const char *Name(int i) const;

	/// What is the 'length' of this representation.
        /** For the rotation type AngleAxis, the 'length' is the size of the
            rotation angle around the axis in radians.
        */
	double Length() const { return length; }

	/// Rotation angle.
	double Angle() const { return length; }

	/// Conversion from a vector rotation.
	void from_vector(const VectorRotation &r);

	/// Conversion from a rotation matrix.
	void from_matrix(const Rotation3D &r);

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assignment operator. Conversion from matrix representation.
	AngleAxisRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check();

	/** Partial derivative with respect to one of the parameters.
	Not implemented for AngleAxisRotation.
	*/
	Rotation3D PartialDeriv(int param) const { int p=param, q; q=p; p=q; return Rotation3D(); }
};


//------------------------------------------------------------------------------
/*!
 * \class VectorRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class VectorRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Rotation around an axis, with a length.
/** The length of the rotation axis is the size of the rotation angle */

class VectorRotation : public GenericRotation , public Vector3D {

protected:

public:
	/// Default constructor.
	VectorRotation();

	/// Initialization constructor
	VectorRotation(const Vector3D &a);

	/// Construct from vector elements
        /** @param X X-component of rotation axis
            @param Y Y-component of rotation axis
            @param Z Z-component of rotation axis
            @param degrees If true, the vector length is converted from
                   degrees to radians.
        */
	VectorRotation(double X, double Y, double Z, int degrees=0) :
			Vector3D(X, Y, Z) {if (degrees) *this *= M_PI/180.; }

	/// Construct from the vector elements in a double array
	VectorRotation(const double *p) : Vector3D(p[0],p[1],p[2]) {}

	/// Construct from a rotation matrix
	VectorRotation(const Rotation3D &rot);

	/// Copy constructor
	VectorRotation(const VectorRotation &a) : GenericRotation(), Vector3D()
          { *this = a; }

	/// Destructor
	~VectorRotation(){}

	/// Element access, readable
	double operator[](int i) const { return x[i]; }

	/// Element access, writable
	double &operator[](int i) { return x[i]; }

	/// Returns the number of parameters
	int DOF() const { return 3; }

	/// Returns the type of rotation representation (Vector in this case)
	RotationType Type() const { return Vector; }

	/// Returns the name of the parameter.
	const char *Name(int i) const;

	/// What is the 'length' of this representation.
	double Length() const { return vect().Length(); }

	/// Conversion from an angle axis representation
	void from_angleaxis(const AngleAxisRotation &r);

	/// Conversion from a rotation matrix.
	void from_matrix(const Rotation3D &r);

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assignment operator. Conversion from matrix representation.
	VectorRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check();

	/// Partial derivative with respect to one of the parameters.
	Rotation3D PartialDeriv(int param) const;
};


//------------------------------------------------------------------------------
/*!
 * \class Vector2DRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class Vector2DRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Unknown rotation type with only two parameters

class Vector2DRotation : public GenericRotation {
protected:
	/// The two rotation parameters
	double x[2];

public:
	/// Default constructor.
	Vector2DRotation();

	/// Initialization constructor.
	Vector2DRotation(double x0, double x1);

	/// Initialization constructor.
	Vector2DRotation(const double *p) { x[0] = p[0]; x[1] = p[1]; }

	/// Construct from matrix rotation
	Vector2DRotation(const Rotation3D &r);

	/// Copy constructor.
	Vector2DRotation(const Vector2DRotation &a) : GenericRotation()
          { *this = a; }

	/// Destructor.
	~Vector2DRotation(){}

	/// Element access.
	double operator[](int i) const { return x[i]; }

	/// Element access, writable.
	double &operator[](int i) { return x[i]; }

	/// Returns the number of parameters
	int DOF() const { return 2; }

	/// Returns the type of rotation representation.
	RotationType Type() const { return Vector2; }

	/// Returns the name of the parameter.
	const char *Name(int i) const;

	/// What is the 'length' of this representation.
	double Length() const { return (sqrt(pow(x[0], 2)+pow(x[1], 2))); }

	/// Conversion from rotation matrix.
	void from_matrix(const Rotation3D &r);

	/// Conversion to rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Conversion to rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assingment operator. Conversion from matrix representation.
	Vector2DRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check();

	/// Partial derivative with respect to one of the parameters.
	Rotation3D PartialDeriv(int param) const;

};


//------------------------------------------------------------------------------
/*!
 * \class QuaternionRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class QuaternionRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Rotation parameterisation with a full quaternion
class QuaternionRotation : public GenericRotation {

protected:
	/// The four quaternion elements
	double q[4];

public:
	/// Default constructor
	QuaternionRotation();

	/// Construct from the four quaterion elements
	QuaternionRotation(double q0, double q1, double q2, double q3);

	/// Construct from a double array with quaternion elements
	QuaternionRotation(const double *q0);

	/// Construct from a rotation matrix 
	QuaternionRotation(const Rotation3D &r);

	/// Copy constructor
	QuaternionRotation(const QuaternionRotation &qr) : GenericRotation()
          { *this = qr; }

	/// Destructor
	~QuaternionRotation(){}

	/// Element access, readable
	double operator[](int i) const { return q[i]; }

	/// Element access, writable
	double &operator[](int i) { return q[i]; }

	/// Returns the number of parameters
	int DOF() const { return 4; }

	/// Returns the type of rotation representation (Quaterion in this case)
	RotationType Type() const { return Quaternion; }

	/// Returns the name of the parameter
	const char *Name(int i) const;

	/// Returns the length of the quaterion vector
	double Length() const { return sqrt(SqLength()); }

	/// Conversion from a rotation matrix.
	void from_matrix(const Rotation3D &r);

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Conversion to a rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assignment operator. Conversion from matrix representation.
	QuaternionRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Squared sum of elements
	double SqLength() const;

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check();

	/// Partial derivative with respect to one of the parameters.
	Rotation3D PartialDeriv(int param) const;
};


//------------------------------------------------------------------------------
/*!
 * \class TrinionRotation
 * \ingroup Photogrammetry
 * \brief Interface to Class TrinionRotation
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define rotation parametrizations. Orientation.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


/// Rotation parameterisation with three parameters for a rotation symmetric (Z-axis) model

class TrinionRotation : public GenericRotation {

protected:
	/// The three trinion elements.
	double t[3];

public:
	/// Default constructor.
	TrinionRotation();

	/// Initialization constructor.
	TrinionRotation(double t0, double t1, double t2);

	/// Initialization constructor.
	TrinionRotation(const double *t0);

	/// Construct from rotation matrix 
	TrinionRotation(const Rotation3D &r);

	/// Copy constructor.
	TrinionRotation(const TrinionRotation &qr) : GenericRotation()
          { *this = qr; }

	/// Destructor.
	~TrinionRotation(){}

	/// Element access.
	double operator[](int i) const { return t[i]; }

	/// Element access, writable.
	double &operator[](int i) { return t[i]; }

	/// Returns the number of parameters
	int DOF() const { return 3; }

	/// Returns the type of rotation representation.
	RotationType Type() const { return Trinion; }

	/// Returns the name of the parameter.
	const char *Name(int i) const;

	/// What is the 'length' of this representation.
	double Length() const { return sqrt(SqLength()); }

	/// Conversion from rotation matrix.
	void from_matrix(const Rotation3D &r);

	/// Conversion to rotation matrix.
	Rotation3D to_matrix(Rotation3D &r) const;

	/// Conversion to rotation matrix.
	Rotation3D to_matrix() const { Rotation3D r; return to_matrix(r); }

	/// Assingment operator. Conversion from matrix representation.
	TrinionRotation operator=(const Rotation3D &a) { from_matrix(a); return *this; }

	/// Squared sum of elements
	double SqLength() const;

	/// Check if parameters obey parametrisation, adjust if necessary/possible.
	int Check();

	/// Partial derivative with respect to one of the parameters.
	Rotation3D PartialDeriv(int param) const;
};

#endif /*_ROTATIONPARAMETER_H_*/ 
