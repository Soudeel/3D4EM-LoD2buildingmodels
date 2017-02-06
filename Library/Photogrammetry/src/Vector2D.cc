
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



/*********************************************************************
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Position, 
*
**********************************************************************/
//#include "overall.h"

#include <math.h>

#include "Vector2D.h"
#include "Vector3D.h"

Vector2D::Vector2D(const Vector3D &p) {
	x[0] = p[0];
	x[1] = p[1];
}
/*
Vector2D::Vector2D(const Vector2D &p) {
	x[0] = p.x[0];
	x[1] = p.x[1];
}

const Vector2D &Vector2D::operator = (const Vector2D &p) {
	x[0] = p.x[0];
	x[1] = p.x[1];
    return *this;
}
*/

double Vector2D::SqLength() const { return x[0]*x[0] + x[1]*x[1]; }

double Vector2D::Length() const { return sqrt(SqLength()); }

Vector2D Vector2D::PartialDeriv(int param) const {
	Vector2D deriv;
	deriv[param] = 1.0;
	return deriv;
}

double Vector2D::DotProduct(const Vector2D &q) const {
	return x[0]*q.x[0] + x[1]*q.x[1];
}

Vector2D Vector2D::Align(const Vector2D &vec) const {

	double len = SqLength();
	if (len == 0.0) return Vector2D();
	return *this * (DotProduct(vec)/SqLength());
}

int Vector2D::AreaSign(const Vector2D &b, const Vector2D &c) const
{
	double area2;
	
	area2 = (b.X() - X()) * (double)(c.Y() - Y()) -
                (c.X() - X()) * (double)(b.Y() - Y());

	if 		( area2 > 0 )	return 1;
	else if		( area2 < 0)	return -1;
	else	{	return 0;	};
}

// Vector2D operators **************************************************

const Vector2D &Vector2D::operator = (const Vector3D &p) {

	x[0] = p[0];
	x[1] = p[1];
	return *this;
}

const Vector2D &Vector2D::operator += (const Vector2D &p) {

	x[0] += p.x[0];
	x[1] += p.x[1];
	return *this;
}

const Vector2D &Vector2D::operator -= (const Vector2D &p) {

	x[0] -= p.x[0];
	x[1] -= p.x[1];
	return *this;
}

const Vector2D &Vector2D::operator *= (double d) {
	x[0] *= d;
	x[1] *= d;
	return *this;
}


const Vector2D &Vector2D::operator /= (double d) {
	x[0] /= d;
	x[1] /= d;
	return *this;
}

Vector2D operator +(const Vector2D &p1, const Vector2D &p2) {
	Vector2D p;
	p.x[0] = p1.x[0] + p2.x[0];
	p.x[1] = p1.x[1] + p2.x[1];
	return p;
}

Vector2D operator -(const Vector2D &p1, const Vector2D &p2) {
	Vector2D p;
	p.x[0] = p1.x[0] - p2.x[0];
	p.x[1] = p1.x[1] - p2.x[1];
	return p;
}

Vector2D operator *(const Vector2D &p1, double d) {
	Vector2D p;
	p.x[0] = p1.x[0]*d;
	p.x[1] = p1.x[1]*d;
	return p;
}

Vector2D operator *(double d, const Vector2D &p1) {
	Vector2D p;
	p.x[0] = p1.x[0]*d;
	p.x[1] = p1.x[1]*d;
	return p;
}

Vector2D operator /(const Vector2D &p1, double d) {
	Vector2D p;
	p.x[0] = p1.x[0]/d;
	p.x[1] = p1.x[1]/d;
	return p;
}

bool operator ==(const Vector2D &p1, const Vector2D &p2) 
{ 
	return p1.x[0] == p2.x[0] && p1.x[1] == p2.x[1]; 
}  


// angle is from p1 to p2, positive anticlockwise
// result is between -pi and pi
double Angle2D(const Vector2D &p1, const Vector2D &p2)
{
    double dtheta, theta1, theta2;
    double PI = 4 * atan(1);
    theta1 = atan2(p1.x[1], p1.x[0]);
    theta2 = atan2(p2.x[1], p2.x[0]);
    dtheta = theta2 - theta1;
    while(dtheta > PI)
       dtheta -= 2 * PI;
    while(dtheta < -PI)
       dtheta += 2 * PI;   
    return dtheta;   
}

// Normal vector
Vector2D Vector2D::Normal() const
{
  Vector2D normal;

  if (!SqLength()) return(Vector2D(0.0, 0.0));
  normal = Vector2D(Y(), -X());
  return(normal / normal.Length());
}
