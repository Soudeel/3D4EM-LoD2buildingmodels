
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
*   Purpose   : Implementing base geometries, Rotation3D
*	Orientation3D.
*
**********************************************************************/

#include <math.h>

#include "Rotation3D.h"
#include "RotationParameter.h"

//#include "csgtree.h"
//#include "projectheader.h"
//#include "globaldata.h"
//#include "Orientation3D.h"
//#include "parameter.h"



Rotation3D::Rotation3D(double num) {
	r[0][0] = r[0][1] = r[0][2] = num;
	r[1][0] = r[1][1] = r[1][2] = num;
	r[2][0] = r[2][1] = r[2][2] = num;
}

Rotation3D::Rotation3D(const Rotation3D &p) {
	memmove(this, &p, sizeof(Rotation3D));
}

/*
const Rotation3D &Rotation3D::operator = (const Rotation3D &p) {
	memmove(this, &p, sizeof(Rotation3D));
	return *this;
}
*/


Rotation3D::Rotation3D(const GenericRotation &gr) { gr.to_matrix(*this); }

Rotation3D Rotation3D::operator = (const GenericRotation &gr) {
	return gr.to_matrix(*this);
}

// Rotation constructors ********************************************

Rotation3D::Rotation3D() {
	r[0][0] = r[1][1] = r[2][2] = 1;
	r[0][1] = r[1][2] = r[0][2] = 0;
	r[1][0] = r[2][1] = r[2][0] = 0;
}

Rotation3D::Rotation3D(double w, double f, double k) {
	r[0][0] =  cos(k) * cos(f);
	r[0][1] = -sin(k) * cos(f);
	r[0][2] =  sin(f);

	r[1][0] =  sin(w) * cos(k) * sin(f) + cos(w) * sin(k);
	r[1][1] =  cos(w) * cos(k) - sin(w) * sin(k) * sin(f);
	r[1][2] = -sin(w) * cos(f);

	r[2][0] =  sin(w) * sin(k) - cos(w) * cos(k) * sin(f);
	r[2][1] =  cos(w) * sin(k) * sin(f) + sin(w) * cos(k);
	r[2][2] =  cos(w) * cos(f);
}

void Rotation3D::DeriveAngles(double &omega, double &phi, double &kappa) const
{
  phi   = asin(r[0][2]);
  if (cos(phi) != 0.0) {
    omega = atan2(-r[1][2], r[2][2]);
    kappa = atan2(-r[0][1], r[0][0]);
  }
  else { // gimbal lock, assume kappa = 0.0
    omega = atan2(r[1][0], -r[2][0]);
    kappa = 0.0;
  }
}

Rotation3D Rotation3D::PartialDeriv(int param) const
{
  Rotation3D rot;
  double     omega, phi, kappa;
  
  DeriveAngles(omega, phi, kappa);
  
  // temp variables
  double so = sin(omega);
  double co = cos(omega);
  double sp = sin(phi);
  double cp = cos(phi);
  double sk = sin(kappa);
  double ck = cos(kappa);

  switch (param) {
    case 0: // Partial derivatives to omega
      rot.R(0,0) = 0.0;
      rot.R(0,1) = 0.0;
      rot.R(0,2) = 0.0;
      rot.R(1,0) = co * sp * ck - so * sk;
      rot.R(1,1) = -co * sp * sk - so * ck;
      rot.R(1,2) = -co * cp;
      rot.R(2,0) = so * sp * ck + co * sk;
      rot.R(2,1) = -so * sp * sk + co * ck;
      rot.R(2,2) = -so * cp;
      break;

    case 1: // Partial derivatives to phi
      rot.R(0,0) = -sp * ck;
      rot.R(0,1) = sp * sk;
      rot.R(0,2) = cp;
      rot.R(1,0) = so * cp * ck;
      rot.R(1,1) = -so * cp * sk;
      rot.R(1,2) = so * sp;
      rot.R(2,0) = -co * cp * ck;
      rot.R(2,1) = co * cp * sk;
      rot.R(2,2) = -co * sp;
      break;

    case 2: // Partial derivatives to kappa
      rot.R(0,0) = -cp * sk;
      rot.R(0,1) = -cp * ck;
      rot.R(0,2) = 0.0;
      rot.R(1,0) = -so * sp * sk + co * ck;
      rot.R(1,1) = -so * sp * ck - co * sk;
      rot.R(1,2) = 0.0;
      rot.R(2,0) = co * sp * sk + so * ck;
      rot.R(2,1) = co * sp * ck - so * sk;
      rot.R(2,2) = 0.0;
      break;

    default:
	  std::cerr<<"Rotation3D::PartialDeriv error; unknown parameter"<<std::endl;
      break;

  }
  return(rot);
}

Rotation3D::Rotation3D(double xi, double yi, double zi, double l) {

	double cl = cos(l);
	double sl = sin(l);
	double vl = 1 - cl;
	
	r[0][0] = xi*xi*vl + cl;
	r[0][1] = xi*yi*vl + zi*sl;
	r[0][2] = xi*zi*vl - yi*sl;

	r[1][0] = yi*xi*vl - zi*sl;
	r[1][1] = yi*yi*vl + cl;
	r[1][2] = yi*zi*vl + xi*sl;

	r[2][0] = zi*xi*vl + yi*sl;
	r[2][1] = zi*yi*vl - xi*sl;
	r[2][2] = zi*zi*vl + cl;
}

Rotation3D::Rotation3D(const Vector3D &p, double l) {
	*this = Rotation3D(p.X(), p.Y(), p.Z(), l);
}

Rotation3D::Rotation3D( double r11, double r12, double r13,
				double r21, double r22, double r23,
				double r31, double r32, double r33) {
	r[0][0] = r11; r[0][1] = r12; r[0][2] = r13;
	r[1][0] = r21; r[1][1] = r22; r[1][2] = r23;
	r[2][0] = r31; r[2][1] = r32; r[2][2] = r33;
}

// Rotation operators **************************************************


const Rotation3D &Rotation3D::operator *=(const Rotation3D &r) {

	Rotation3D t = *this * r;
	*this = t;
	return *this;
}

Rotation3D operator *(const Rotation3D &r1, const Rotation3D &r2) {

	Rotation3D r;
	for (int i=0; i<W_DIM; i++) {
		for (int j=0; j<W_DIM; j++) {

			r.r[i][j] = 0;
			for (int k=0; k<W_DIM; k++) {
				r.r[i][j] += r1.r[i][k]*r2.r[k][j];
			}
		}
	}
	return r;
}


Rotation3D Rotation3D::Transpose() const {

	Rotation3D R;
	for (int i=0; i<W_DIM; i++) {
		for (int j=0; j<W_DIM; j++) {
			R.r[i][j] = r[j][i];
		}
	}
	return R;
}

void Rotation3D::as_axis(Vector3D &p, double &l) const {

	l = acos((r[0][0] + r[1][1] + r[2][2] - 1)/2);
	double hl = (l != 0)? 0.5/sin(l): 1;

	p.X() = hl*( r[2][1] - r[1][2]);
	p.Y() = hl*( r[0][2] - r[2][0]);
	p.Z() = hl*( r[1][0] - r[0][1]);
}

Vector3D Rotation3D::Row(int i) const {
	Vector3D res(r[i][0], r[i][1], r[i][2]);
	return res;
}

Vector3D Rotation3D::Column(int i) const {
	Vector3D res(r[0][i], r[1][i], r[2][i]);
	return res;
}

void Rotation3D::Print(std::ostream &os) const {

	std::cerr << r[0][0] <<",  "<< r[0][1] <<",  "<< r[0][2] << std::endl;
	std::cerr << r[1][0] <<",  "<< r[1][1] <<",  "<< r[1][2] << std::endl;
	std::cerr << r[2][0] <<",  "<< r[2][1] <<",  "<< r[2][2] << std::endl;
}

