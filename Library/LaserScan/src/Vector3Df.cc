
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : March 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A float version of Vector3Df to save sapce.
*
*--------------------------------------------------------------------*/
//#include "overall.h"
#include <math.h>

#include "Orientation3D.h"
#include "Vector2D.h"
#include "Vector3Df.h"
#include <memory.h>

Vector3Df::Vector3Df(const Vector2D &v, double z) {
	x[0] = v[0];	
	x[1] = v[1];	
	x[2] = z;
}

Vector3Df &Vector3Df::operator = (const Vector3Df &p) {
	memmove(this, &p, sizeof(Vector3Df));
	return *this;
}

double Vector3Df::SqLength() const {

	double sum = 0;
	for (int j=0; j<W_DIM; j++)
		sum += x[j]*x[j];
	return sum;
}

double Vector3Df::SqLength2D() const {
	return (x[0]*x[0] + x[1]*x[1]);
}

double Vector3Df::Length() const { return sqrt(SqLength()); }

double Vector3Df::Length2D() const { return sqrt(SqLength2D()); }

Vector3Df Vector3Df::Normalize() const { 

	double len = sqrt(SqLength());
	if (len == 0.0) {
		std::cerr<<"Vector3Df::Normalize error; can't normalize a nil vector"<<std::endl;
		return *this;
	}
	return *this/len;
}

Vector3Df Vector3Df::PartialDeriv(int param) const {

	Vector3Df deriv;
	deriv[param] = 1.0;
	return deriv;
}

Vector3Df Vector3Df::VectorProduct(const Vector3Df &q) const {
	Vector3Df r;
	r.x[0] = x[1] * q.x[2] - q.x[1]*x[2];
	r.x[1] = x[2] * q.x[0] - q.x[2]*x[0];
	r.x[2] = x[0] * q.x[1] - q.x[0]*x[1];
	return r;
}

double Vector3Df::DotProduct(const Vector3Df &q) const {
	return x[0]*q.x[0] + x[1]*q.x[1] + x[2]*q.x[2];
}

Vector3Df Vector3Df::Align(const Vector3Df &vec) const {

	double len = SqLength();
	if (len == 0.0) return Vector3Df();
	return *this * (DotProduct(vec)/SqLength());
}

int Vector3Df::AbsLargest() const {

	int max_i = 0;
	double max = fabs(x[0]);
	if (max < fabs(x[1])) {
		max_i = 1;
		max = fabs(x[1]);
	}
	if (max < fabs(x[2])) {
		max_i = 2;
		max = fabs(x[2]);
	}
	return max_i;
}

void Vector3Df::PerpendicularVectors(Vector3Df &xv, Vector3Df &yv) const {

	int li = AbsLargest();		// choose biggest to avoid numerical problems.
	int ia = (li + 1)%3;		// other indices
	int ib = (li + 2)%3;		// other indices
	//Vector3Df n = Normalize();	// be sure it is normalized.
	const float *n = x;

	xv[li] = 0;				// one element free to choose.

	// length of xv == 1 and dotproduct of xv.this == 0
	// given these equations, one can compute all elements of xv:

	if (n[ib] != 0.0) {
		// the sign of this element is not determined, but not important either.
		xv[ia] = 1./sqrt(1. + n[ia]*n[ia]/(n[ib]*n[ib]));
	
		xv[ib] = -xv[ia]*n[ia]/n[ib];

	} else {
		xv[ia] = 0;
		xv[ib] = 1;
	}

	// yv is perpendicular to xv as well as n -> vectorproduct(xv, this);
	yv = xv.VectorProduct(*this);
}

// Vector3Df operators **************************************************

const Vector3Df &Vector3Df::operator += (const Vector3Df &p) {

	for (int j=0; j<W_DIM; j++)
		x[j] += p.x[j];
	return *this;
}

const Vector3Df &Vector3Df::operator -= (const Vector3Df &p) {

	for (int j=0; j<W_DIM; j++)
		x[j] -= p.x[j];
	return *this;
}

const Vector3Df &Vector3Df::operator *= (double d) {
	for (int j=0; j<W_DIM; j++)
		x[j] *= d;
	return *this;
}

const Vector3Df &Vector3Df::operator *= (const Rotation3D &r) {
	*this = r**this;
	return *this;
}

const Vector3Df &Vector3Df::operator *= (const Orientation3D &o) {
	*this = o**this;
	return *this;
}

Vector3Df &Vector3Df::operator *= (Orientation3D &o) {
	*this = o**this;
	return *this;
}

const Vector3Df &Vector3Df::operator /= (double d) {
	for (int j=0; j<W_DIM; j++)
		x[j] /= d;
	return *this;
}

Vector3Df operator +(const Vector3Df &p1, const Vector3Df &p2) {
	Vector3Df p;
	for (int j=0; j<W_DIM; j++){
		p.x[j] = p1.x[j] + p2.x[j];
	}
	return p;
}

Vector3Df operator -(const Vector3Df &p1, const Vector3Df &p2) {
	Vector3Df p;
	for (int j=0; j<W_DIM; j++){
		p.x[j] = p1.x[j] - p2.x[j];
	}
	return p;
}

Vector3Df operator *(const Vector3Df &p1, double d) {
	Vector3Df p;
	for (int j=0; j<W_DIM; j++){
		p.x[j] = p1.x[j]*d;
	}
	return p;
}

Vector3Df operator /(const Vector3Df &p1, double d) {
	Vector3Df p;
	for (int j=0; j<W_DIM; j++){
		p.x[j] = p1.x[j]/d;
	}
	return p;
}


Vector3Df operator *(const Rotation3D &r1, const Vector3Df &p1) {

	Vector3Df p;
	for (int i=0; i<W_DIM; i++) {
		for (int j=0; j<W_DIM; j++) {
			p.x[i] += r1.R(i,j) * p1.x[j];
		}
	}
	return p;
}

Vector3Df::Vector3Df(const Vector3D& v)
{
	x[0]=v.X();
	x[1]=v.Y();
	x[2]=v.Z();
}
Vector3Df operator *(const Orientation3D &o1, const Vector3Df &p2) {

	return o1.rotation()*p2 + o1.vect();
}

Vector3Df operator *(Orientation3D &o1, Vector3Df &p2) {

	return o1.rotation()*p2 + o1.vect();
}

bool operator ==(const Vector3Df &p1, const Vector3Df &p2) 
{ 
	return p1.x[0] == p2.x[0] && p1.x[1] == p2.x[1] 
			&& p1.x[2] == p2.x[2]; 
}

/* Additions outside STW project */

double Vector3Df::Direction2D(const Vector3Df &pto) const
{
  if (*this == pto) return(0);
  return(atan2(pto.x[1]-x[1], pto.x[0]-x[0]));
}

double Vector3Df::Angle2D(const Vector3Df &p1, const Vector3Df &p2) const
{
  if (*this == p1 || *this == p2) return(-1);
  double angle = Direction2D(p2) - Direction2D(p1);
  if (angle < 0.0) angle += 8.0 * atan(1.0);
  return(angle);
}

double Angle(const Vector3Df &v1, const Vector3Df &v2)
{
  return(acos(v1.DotProduct(v2)/(v1.Length() * v2.Length())));
}

// functions PrintVector(), Unity(), Sum(), AbsMax() added 03.06.2003
void Vector3Df::PrintVector()  {
	std::cout << "X(): " << x[0];
	std::cout << " Y(): " << x[1];
	std::cout << " Z(): " << x[2] << std::endl;
}

bool Vector3Df::Unity()  {
	if (Length() < 1.01 && Length() > 0.99) return 1;
	return 0;
}

double Vector3Df::Sum() const  {
        double sum  = x[0] + x[1] + x[2];
	return sum;
}

double Vector3Df::AbsMax(int* element) const  {
        if (fabs(x[0]) >= fabs(x[1]) && fabs(x[0]) >= fabs(x[2])) {
	  *element = 0;
	  return x[0];
	}
	else if (fabs(x[1]) > fabs(x[0]) && fabs(x[1]) >= fabs(x[2])) {
	  *element = 1;
	  return x[1];
	}
	else if (fabs(x[2]) > fabs(x[0]) && fabs(x[2]) > fabs(x[1])) {
	  *element = 2;
	  return x[2];
	}
}
