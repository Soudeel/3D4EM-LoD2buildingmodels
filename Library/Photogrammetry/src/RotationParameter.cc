
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
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : juli 1998
*   Author    : Pierre Ermes
*   Purpose   : Implementation of rotation member functions
*	Modified  :
*
*--------------------------------------------------------------------*/
//#include "overall.h"
#include "digphot_arch.h"
#ifdef windows
#  include <stdlib.h>
#else
#  include <alloca.h>
#endif
#include <math.h>
#include <new>
#include <limits.h>


//#include "csgnode.h"
//#include "csgtree.h"
#include "defstrings.h"
//#include "globaldata.h"
//#include "linpack.h"
//#include "orientation.h"
//#include "primitive.h"
//#include "projectheader.h"

#include "RotationParameter.h"

/*
void rotation_test() {

	Rotation3D r;
	EulerRotation er(1,1,1);
	r = er;
	EulerRotation er2(r);

	AngleAxisRotation aar(Vector3D(4.0/5.0, 3.0/5.0, 0), 1);
	r = aar;
	AngleAxisRotation aar2(r);

	VectorRotation vr(-1,-1,-1);
	r = vr;
	VectorRotation vr2(r);

	vr = Vector2DRotation(M_PI, 0);
	r = vr;

	vr = Vector2DRotation(M_PI, M_PI/10.);
	r = vr;

	vr = Vector2DRotation(M_PI*1.1, 0);
	r = vr;

	double diff = 1e-8;
	for (int i=-8; i< -1; i++) {
		Vector2DRotation vra(diff,0);
		r = vra;
		Vector2DRotation vra2(r);

		Vector2DRotation vrb((M_PI - diff),0);
		r = vrb;
		Vector2DRotation vrb2(r);

		cerr << "exp: "<< diff <<", diff: "<< vra[0] - vra2[0] <<", diff2: "<< vrb[0] - vrb2[0] << endl;
		cerr << "exp: "<< diff <<", diff: "<< vra[1] - vra2[1] <<", diff2: "<< vrb[1] - vrb2[1] << endl << endl;
		//cerr << "exp: "<< diff <<", diff: "<< vra[2] - vra2[2] <<", diff2: "<< vrb[2] - vrb2[2] << endl << endl;
		diff *= 10.;
	}
	exit(0);
	Vector2DRotation vr2d(-1,-1);
	r = vr2d;
	Vector2DRotation vrd2(r);

	Vector2DRotation vra2d( -1e-8, 0);
	r = vra2d;
	Vector2DRotation vrad2(r);

	Vector2DRotation vrb2d((M_PI - 1e-8),0);
	r = vrb2d;
	Vector2DRotation vrbd2(r);

	QuaternionRotation q(0.5, 0.5, 0.5, 0.5);
	r = q;
	QuaternionRotation qa(r);

	QuaternionRotation q1(1, 0, 0, 1E-8);
	r = q1;
	QuaternionRotation q1a(r);

}
*/
/*
void rotation_partial_test() {

	Rotation3D r(8);
	Vector2DRotation vr2(0,0);
	r = vr2.PartialDeriv(0);
	r = vr2.PartialDeriv(1);

	vr2 = Vector2DRotation(M_PI, 0);
	r = vr2.PartialDeriv(0);
	r = vr2.PartialDeriv(1);

	VectorRotation vr(0,0,0);
	r = vr.PartialDeriv(0);
	r = vr.PartialDeriv(1);
	r = vr.PartialDeriv(2);

	vr = VectorRotation(0.00001, 0, 0);
	r = vr.PartialDeriv(0);
	r = vr.PartialDeriv(1);
	r = vr.PartialDeriv(2);

	vr = VectorRotation(M_PI/2, M_PI/2, 0);
	r = vr.PartialDeriv(0);
	r = vr.PartialDeriv(1);
	r = vr.PartialDeriv(2);

	vr = VectorRotation(M_PI, 0, 0);
	// try to simulate partial derivative.
	Rotation r1 = vr;
	Rotation r2 = VectorRotation(M_PI + 0.001, 0, 0);
	Rotation r3 = VectorRotation(M_PI, 0.001, 0);
	r = vr.PartialDeriv(0);
	r = vr.PartialDeriv(1);
	r = vr.PartialDeriv(2);

	vr = VectorRotation(0, M_PI, 0);
	r = vr.PartialDeriv(0);
	r = vr.PartialDeriv(1);
	r = vr.PartialDeriv(2);

	QuaternionRotation qr(1,0,0,0);
	r = qr.PartialDeriv(0);
	r = qr.PartialDeriv(1);
	r = qr.PartialDeriv(2);
	r = qr.PartialDeriv(3);

	qr = QuaternionRotation(0,1,0,0);
	r = qr.PartialDeriv(0);
	r = qr.PartialDeriv(1);
	r = qr.PartialDeriv(2);
	r = qr.PartialDeriv(3);
}
*/

GenericRotation *CopyRotation(const GenericRotation &rot) {

	switch (rot.Type()) {
	case Vector: return new VectorRotation(rot);
	case Quaternion: return new QuaternionRotation(rot);
	case Trinion: return new TrinionRotation(rot);
	default:
		std::cerr<<"CopyRotation error, unknown rotation representation"<<std::endl;
		break;
	}
	return 0;
}

/*--------------------------------------------------------------------
*
*	EulerRotation member functions
*
*--------------------------------------------------------------------*/

EulerRotation::EulerRotation() {
	omega = phi = kappa = 0;
}

EulerRotation::EulerRotation(const Rotation3D &r) { from_matrix(r); }

EulerRotation::EulerRotation(double o, double p, double k) {
	omega = o; phi = p; kappa = k;
}

const char *EulerRotation::Name(int i) const {

	switch (i) {
	case 0: return "omega";
	case 1: return "phi";
	case 2: return "kappa";
	default: return "error";
	}
}

void EulerRotation::from_matrix(const Rotation3D &r) {

	omega = atan2(-r.R(2,1), r.R(2,2));
	phi = asin(r.R(2,0));
	kappa = atan2(-r.R(1,0), r.R(0,0));
}

Rotation3D EulerRotation::to_matrix(Rotation3D &r) const {
// Watch out: transposed rotation compared to implementation in Rotation3D !
	// temp variables
	double so = sin(omega);
	double co = cos(omega);
	double sp = sin(phi);
	double cp = cos(phi);
	double sk = sin(kappa);
	double ck = cos(kappa);

	r.R(0,0) = cp*ck;
	r.R(0,1) = so*sp*ck + co*sk;
	r.R(0,2) = -co*sp*ck + so*sk;

	r.R(1,0) = -cp*sk;
	r.R(1,1) = -so*sp*sk + co*ck;
	r.R(1,2) = co*sp*sk + so*ck;

	r.R(2,0) = sp;
	r.R(2,1) = -so*cp;
	r.R(2,2) = co*cp;
	return r;
}

double EulerRotation::operator[](int i) const {
	switch(i) {
	case 0: return omega;
	case 1: return phi;
	case 2: return kappa;
	default:
		std::cerr <<"EulerRotation::operator[]; invalid index "<< i << std::endl;
	}
	return 0;
}

double &EulerRotation::operator[](int i) {
	switch(i) {
	case 0: return omega;
	case 1: return phi;
	case 2: return kappa;
	default:
		std::cerr <<"EulerRotation::operator[]; invalid index "<< i << std::endl;
	}
	static double d=0;
	return d;
}

Rotation3D EulerRotation::PartialDeriv(int param) const {
	Rotation3D rot(0);

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
		std::cerr<<"EulerRotation::PartialDeriv error; unknown parameter"<<std::endl;
                break;

        }
        return(rot);
}

/*--------------------------------------------------------------------
*
*	AngleAxisRotation member functions
*
*--------------------------------------------------------------------*/





AngleAxisRotation::AngleAxisRotation() : Vector3D(1,0,0) {
	length = 0;
}

AngleAxisRotation::AngleAxisRotation(const Rotation3D &r) { from_matrix(r); }

AngleAxisRotation::AngleAxisRotation(const Vector3D &a, double l) : Vector3D(a) {
	length = l;
}

const char *AngleAxisRotation::Name(int i) const {

	switch (i) {
	case 0: return RX_STR;
	case 1: return RY_STR;
	case 2: return RZ_STR;
	case 3: return LENGTH_STR;
	default: return "error";
	}
}

// DBL_DIG = 15 for SGI O2
void AngleAxisRotation::from_matrix(const Rotation3D &r) {

	const double lowertreshold =  1.0E-14;

	double cl = 0.5*(r.Trace() - 1);

	if (1.0 - cl <= lowertreshold) { 	// no rotation, assume an axis
		length = 0;
		x[0] = 1;
		x[1] = 0;
		x[2] = 0;

	} else if (1.0 + cl <= lowertreshold) {	// 180 rotation, special case

		length = M_PI;
		x[0] = sqrt(0.5*(r.R(0,0) + 1));

		x[1] = sqrt(0.5*(r.R(1,1) + 1));
		if (r.R(0,1) < 0) x[1] *= -1;

		x[2] = sqrt(0.5*(r.R(2,2) + 1));
		if (r.R(0,2) < 0) x[2] *= -1;

	} else {
	
		length = acos(cl);
		double sl = 0.5/sin(length);
		x[0] = (r.R(2,1) - r.R(1,2))*sl;
		x[1] = (r.R(0,2) - r.R(2,0))*sl;
		x[2] = (r.R(1,0) - r.R(0,1))*sl;
	}
}

Rotation3D AngleAxisRotation::to_matrix(Rotation3D &r) const {

	// temp variables
	double cl = cos(length);
	double vl = 1 - cl;
	double sl = sin(length);

	double x00 = x[0]*x[0]*vl;
	double x01 = x[0]*x[1]*vl;
	double x02 = x[0]*x[2]*vl;
	double x11 = x[1]*x[1]*vl;
	double x12 = x[1]*x[2]*vl;
	double x22 = x[2]*x[2]*vl;
	double x0  = x[0]*sl;
	double x1  = x[1]*sl;
	double x2  = x[2]*sl;

	r.R(0,0) = x00 + cl;
	r.R(0,1) = x01 - x2;
	r.R(0,2) = x02 + x1;
	
	r.R(1,0) = x01 + x2;
	r.R(1,1) = x11 + cl;
	r.R(1,2) = x12 - x0;

	r.R(2,0) = x02 - x1;
	r.R(2,1) = x12 + x0;
	r.R(2,2) = x22 + cl;

	return r;
}

void AngleAxisRotation::from_vector(const VectorRotation &a) {

	length = a.Length();
	if (length == 0) {
		x[0] = 1;
		x[1] = 0;
		x[2] = 0;
	} else {
		x[0] = a[0]/length;
		x[1] = a[1]/length;
		x[2] = a[2]/length;
	}
}

int AngleAxisRotation::Check() {
	vect() = vect().Normalize();
	return 0;
}

/*--------------------------------------------------------------------
*
*	VectorRotation member functions
*
*--------------------------------------------------------------------*/



VectorRotation::VectorRotation() : Vector3D(0,0,0) {}

VectorRotation::VectorRotation(const Vector3D &a) : Vector3D(a) {}

VectorRotation::VectorRotation(const Rotation3D &r) { from_matrix(r); }

const char *VectorRotation::Name(int i) const {

	switch (i) {
	case 0: return RX_STR;
	case 1: return RY_STR;
	case 2: return RZ_STR;
	default: return "error";
	}
}

#define ZERO_TRESHOLD (1e-12)
#define PI_TRESHOLD (1e-10)

void VectorRotation::from_matrix(const Rotation3D &r) {

	double dx[3];
	dx[0] = r.R(1,2) - r.R(2,1);
	dx[1] = r.R(2,0) - r.R(0,2);
	dx[2] = r.R(0,1) - r.R(1,0);

	int index = 0;
	double maxside = fabs(dx[0]);
	if (fabs(dx[1]) > maxside) {
		maxside = fabs(dx[1]);
		index = 1;
	} else if (fabs(dx[2]) > maxside) {
		maxside = fabs(dx[2]);
		index = 2;
	}
	double trace = r.Trace();

	if (3 - trace <= ZERO_TRESHOLD) {	// angle close to zero, take linearization

		x[0] = r.R(2,1);
		x[1] = -r.R(2,0);
		x[2] = r.R(1,0);

	} else if (trace + 1 <= PI_TRESHOLD ) {
		// sin(angle) close to zero


		x[0] = sqrt(0.5*(1 + r.R(0,0)));
		if (dx[0] > 0)
			x[0] *= -1;

		x[1] = sqrt(0.5*(1 + r.R(1,1)));
		if (dx[1] > 0)
			x[1] *= -1;

		x[2] = sqrt(0.5*(1 + r.R(2,2)));
		if (dx[2] > 0)
			x[2] *= -1;

		// substitute (pi - theta) for the rotation angle
		// (theta always <= 0)
		double theta = 0.5*dx[index]/x[index];
		theta += M_PI;
		*this *= theta;
	} else {

		double cp = 0.5*(trace - 1);
		double phi = acos(cp);
		double sp = sin(phi);
		double mult = phi / (2*sp);
		//double sp = sqrt(1 - cp*cp);
		//double mult = acos(cp) / (2*sp);
		x[0] = mult*(r.R(2,1) - r.R(1,2));
		x[1] = mult*(r.R(0,2) - r.R(2,0));
		x[2] = mult*(r.R(1,0) - r.R(0,1));
	}
}

Rotation3D VectorRotation::to_matrix(Rotation3D &r) const {

	AngleAxisRotation a;
	a.from_vector(*this);
	return a.to_matrix(r);
}

void VectorRotation::from_angleaxis(const AngleAxisRotation &a) {
	double l = a.Length();
	*this = a*l;		// multiply with rotation angle
}

// check whether this vector can be shortened.
// by subtracting n*2PI
int VectorRotation::Check() {

	double l = Length();
	if (l > M_PI) {

		int mult = (int)((l + M_PI)/(2*M_PI));
		*this -= (*this) * mult * 2 * M_PI;
		std::cout<<"VectorRotation::Check; vector shortened."<<std::endl;
		return 1;
	}
	return 0;
}

Rotation3D VectorRotation::PartialDeriv(int param) const {
	
	// code generated by maple.
	double t1, t2, t3, t4, t5, t6, t7, t8, t9;
	double t10, t11, t12, t13, t14, t16, t17, t19;
	double t21, t23, t25, t26, t27, t28, t29;
	double t30, t31, t32, t33, t34, t35, t36, t37, t38, t39;
	double t40, t41, t42, t43, t44, t45, t46, t47, t49;
	double t51, t53, t56, t57, t58, t59;
	double t60, t61, t62, t63, t64, t65, t69;
	Rotation3D rot(0);

	// the first four are the same for all param
	// use these to check for singularities.
    t1 = x[0]*x[0];
    t2 = x[1]*x[1];
    t3 = x[2]*x[2];
    t4 = t1+t2+t3;

	switch (param) {
	case 0:
		if (t4 <= ZERO_TRESHOLD) {
			rot.R(1,2) = -1.0;
			rot.R(2,1) = 1.0;
			break;
		}
      t5 = 1/t4;
      t7 = sqrt(t4);
      t8 = cos(t7);
      t9 = 1.0-t8;
      t11 = t1*x[0];
      t12 = t4*t4;
      t13 = 1/t12;
      t17 = 1/t7/t4;
      t19 = sin(t7);
      t21 = 1/t7;
      t23 = x[0]*t21*t19;
      t25 = t5*x[1];
      t26 = t25*t9;
      t27 = t1*t13;
      t29 = t27*x[1]*t9;
      t30 = t1*t17;
      t32 = t30*x[1]*t19;
      t34 = t19*x[0];
      t35 = x[2]*t17*t34;
      t36 = x[2]*t5;
      t37 = t8*x[0];
      t38 = t36*t37;
      t40 = t36*t9;
      t41 = x[2]*t9;
      t42 = t27*t41;
      t44 = t30*x[2]*t19;
      t45 = x[1]*t17;
      t46 = t45*t34;
      t47 = t25*t37;
      t51 = t9*x[0];
      t58 = x[1]*t13*t41*x[0];
      t61 = t45*x[2]*x[0]*t19;
      t62 = t21*t19;
      t63 = t30*t19;
      t65 = t1*t5*t8;
      rot.R(0, 0) = 2.0*x[0]*t5*t9-2.0*t11*t13*t9+t11*t17*t19-t23;
      rot.R(0, 1) = t26-2.0*t29+t32+t35-t38;
      rot.R(0, 2) = t40-2.0*t42+t44-t46+t47;
      rot.R(1, 0) = t26-2.0*t29+t32-t35+t38;
      rot.R(1, 1) = -2.0*t2*t13*t51+t2*t17*t34-t23;
      rot.R(1, 2) = -2.0*t58+t61-t62+t63-t65;
      rot.R(2, 0) = t40-2.0*t42+t44+t46-t47;
      rot.R(2, 1) = -2.0*t58+t61+t62-t63+t65;
      rot.R(2, 2) = -2.0*t3*t13*t51+t3*t17*t34-t23;
		break;
	case 1:
		if (t4 <= ZERO_TRESHOLD) {
			rot.R(0,2) = 1.0;
			rot.R(2,0) = -1.0;
			break;
		}
      t5 = t4*t4;
      t6 = 1/t5;
      t8 = sqrt(t4);
      t9 = cos(t8);
      t10 = 1.0-t9;
      t14 = 1/t8/t4;
      t16 = sin(t8);
      t19 = 1/t8;
      t21 = x[1]*t19*t16;
      t23 = t2*t6;
      t25 = t23*t10*x[0];
      t26 = 1/t4;
      t28 = x[0]*t26*t10;
      t29 = t2*t14;
      t30 = x[0]*t16;
      t31 = t29*t30;
      t32 = x[1]*t14;
      t33 = t16*x[2];
      t34 = t32*t33;
      t35 = x[1]*t26;
      t37 = t35*t9*x[2];
      t39 = x[1]*t6;
      t40 = x[2]*t10;
      t42 = t39*t40*x[0];
      t45 = t32*x[2]*x[0]*t16;
      t46 = t19*t16;
      t47 = t29*t16;
      t49 = t2*t26*t9;
      t53 = t2*x[1];
      t60 = t26*x[2]*t10;
      t61 = t23*t40;
      t62 = t29*t33;
      t63 = t32*t30;
      t65 = t35*t9*x[0];
      rot.R(0, 0) = -2.0*t1*t6*x[1]*t10+t1*t14*x[1]*t16-t21;
      rot.R(0, 1) = -2.0*t25+t28+t31+t34-t37;
      rot.R(0, 2) = -2.0*t42+t45+t46-t47+t49;
      rot.R(1, 0) = -2.0*t25+t28+t31-t34+t37;
      rot.R(1, 1) = 2.0*t35*t10-2.0*t53*t6*t10+t53*t14*t16-t21;
      rot.R(1, 2) = t60-2.0*t61+t62+t63-t65;
      rot.R(2, 0) = -2.0*t42+t45-t46+t47-t49;
      rot.R(2, 1) = t60-2.0*t61+t62-t63+t65;
      rot.R(2, 2) = -2.0*t39*t3*t10+t32*t3*t16-t21;
		break;
	case 2:
		if (t4 <= ZERO_TRESHOLD) {
			rot.R(0,1) = -1.0;
			rot.R(1,0) = 1.0;
			break;
		}
      t5 = t4*t4;
      t6 = 1/t5;
      t8 = sqrt(t4);
      t9 = cos(t8);
      t10 = 1.0-t9;
      t11 = x[2]*t10;
      t14 = 1/t8/t4;
      t16 = sin(t8);
      t17 = x[2]*t16;
      t19 = 1/t8;
      t21 = x[2]*t19*t16;
      t23 = x[1]*t6;
      t25 = t23*t11*x[0];
      t26 = x[1]*t14;
      t29 = t26*x[2]*x[0]*t16;
      t30 = t19*t16;
      t31 = t3*t14;
      t32 = t31*t16;
      t33 = 1/t4;
      t35 = t3*t33*t9;
      t39 = t3*t6*t10*x[0];
      t41 = x[0]*t33*t10;
      t42 = x[0]*t16;
      t43 = t31*t42;
      t44 = t26*t17;
      t45 = x[1]*t33;
      t47 = t45*t9*x[2];
      t56 = t23*t3*t10;
      t57 = t45*t10;
      t59 = t26*t3*t16;
      t61 = x[2]*t14*t42;
      t62 = x[2]*t33;
      t64 = t62*t9*x[0];
      t69 = t3*x[2];
      rot.R(0, 0) = -2.0*t1*t6*t11+t1*t14*t17-t21;
      rot.R(0, 1) = -2.0*t25+t29-t30+t32-t35;
      rot.R(0, 2) = -2.0*t39+t41+t43-t44+t47;
      rot.R(1, 0) = -2.0*t25+t29+t30-t32+t35;
      rot.R(1, 1) = -2.0*t2*t6*t11+t2*t14*t17-t21;
      rot.R(1, 2) = -2.0*t56+t57+t59+t61-t64;
      rot.R(2, 0) = -2.0*t39+t41+t43+t44-t47;
      rot.R(2, 1) = -2.0*t56+t57+t59-t61+t64;
      rot.R(2, 2) = 2.0*t62*t10-2.0*t69*t6*t10+t69*t14*t16-t21;
		break;
	default:
		std::cerr<<"VectorRotation::PartialDeriv error; unknown parameter"<<std::endl;
		break;
	}
	return rot;
}

/*--------------------------------------------------------------------
*
*	Vector2DRotation member functions
*
*--------------------------------------------------------------------*/

Vector2DRotation::Vector2DRotation() {
	x[0] = x[1] = 0;
};

Vector2DRotation::Vector2DRotation(double x0, double x1) {
	x[0] = x0; x[1] = x1;
}

Vector2DRotation::Vector2DRotation(const Rotation3D &r) { from_matrix(r); }

const char *Vector2DRotation::Name(int i) const {

	switch (i) {
	case 0: return RX_STR;
	case 1: return RY_STR;
	default: return "error";
	}
}

void Vector2DRotation::from_matrix(const Rotation3D &r) {

#define ZERO_2D_TRESHOLD (1e-12)
#define PI_2D_TRESHOLD (1e-10)

	if (1 - r.R(2,2) <= ZERO_2D_TRESHOLD) {

		// rotation close to zero
		x[0] = -r.R(1,2);
		x[1] =  r.R(0,2);

	} else if (1 + r.R(2,2) <= PI_2D_TRESHOLD) {
		
		// rotation close to pi
		double th = -sqrt( r.R(1,2)*r.R(1,2) + r.R(0,2)*r.R(0,2) );
		
		// normalize vector
		if (th == 0) {
			x[0] = sqrt(0.5*(1 + r.R(0,0)));
			x[1] = sqrt(0.5*(1 + r.R(1,1)));
			if (r.R(0,1) < 0) x[1] *= -1;
		} else {
			x[0] =  r.R(1,2)/th;
			x[1] = -r.R(0,2)/th;
		}

		// multiply with angle
		th += M_PI;
		x[0] *= th;
		x[1] *= th;
		
	} else {

		double phi = acos(r.R(2,2));
		double norm = 1/sqrt(1 - r.R(2,2)*r.R(2,2));
		x[0] = -r.R(1,2)*norm*phi;
		x[1] =  r.R(0,2)*norm*phi;
	}
}

Rotation3D Vector2DRotation::to_matrix(Rotation3D &r) const {

	double theta = sqrt( x[0]*x[0] + x[1]*x[1]);
	double kx = (theta == 0)? 0: x[0]/theta;
	double ky = (theta == 0)? 0: x[1]/theta;
	double st = sin(theta);
	double ct = cos(theta);
	double vt = 1 - ct;
	double kxs = kx*st;
	double kys = ky*st;
	double kxyv = kx*ky*vt;

	r.R(0,0) = kx*kx*vt + ct;
	r.R(0,1) = kxyv;
	r.R(0,2) = kys;

	r.R(1,0) = kxyv;
	r.R(1,1) = ky*ky*vt + ct;
	r.R(1,2) = -kxs;

	r.R(2,0) = -kys;
	r.R(2,1) = kxs;
	r.R(2,2) = ct;

	return r;
}

int Vector2DRotation::Check() { return 0; }

Rotation3D Vector2DRotation::PartialDeriv(int param) const {

	double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t15, t16, t18;
	double t20, t22, t24, t25, t28, t29, t31, t32, t33, t34, t35, t37, t39;
	double t45, t46, t48;
	Rotation3D rot(0);

    t1 = x[0]*x[0];
    t2 = x[1]*x[1];
    t3 = t1+t2;

	if (param == 0) {
		if (t3 <= ZERO_TRESHOLD) {
			rot.R(1,2) = -1.0;
			rot.R(2,1) = 1.0;
			return rot;
		}
      t4 = 1/t3;
      t6 = sqrt(t3);
      t7 = cos(t6);
      t8 = 1.0-t7;
      t10 = t1*x[0];
      t11 = t3*t3;
      t12 = 1/t11;
      t16 = 1/t6/t3;
      t18 = sin(t6);
      t20 = 1/t6;
      t22 = x[0]*t20*t18;
      t24 = t4*x[1];
      t29 = t1*t16;
      t32 = t24*t8-2.0*t1*t12*x[1]*t8+t29*x[1]*t18;
      t34 = t18*x[0];
      t35 = x[1]*t16*t34;
      t37 = t24*t7*x[0];
      t45 = t20*t18;
      t46 = t29*t18;
      t48 = t1*t4*t7;
      rot.R(0, 0) = 2.0*x[0]*t4*t8-2.0*t10*t12*t8+t10*t16*t18-t22;
      rot.R(0, 1) = t32;
      rot.R(0, 2) = -t35+t37;
      rot.R(1, 0) = t32;
      rot.R(1, 1) = -2.0*t2*t12*t8*x[0]+t2*t16*t34-t22;
      rot.R(1, 2) = -t45+t46-t48;
      rot.R(2, 0) = t35-t37;
      rot.R(2, 1) = t45-t46+t48;
      rot.R(2, 2) = -t22;

	} else {
		if (t3 <= ZERO_TRESHOLD) {
			rot.R(0,2) = 1.0;
			rot.R(2,0) = -1.0;
			return rot;
		}
      t4 = t3*t3;
      t5 = 1/t4;
      t7 = sqrt(t3);
      t8 = cos(t7);
      t9 = 1.0-t8;
      t13 = 1/t7/t3;
      t15 = sin(t7);
      t18 = 1/t7;
      t20 = x[1]*t18*t15;
      t25 = 1/t3;
      t28 = t2*t13;
      t29 = x[0]*t15;
      t31 = -2.0*t2*t5*t9*x[0]+x[0]*t25*t9+t28*t29;
      t32 = t18*t15;
      t33 = t28*t15;
      t35 = t2*t25*t8;
      t37 = t25*x[1];
      t39 = t2*x[1];
      t46 = x[1]*t13*t29;
      t48 = t37*t8*x[0];
      rot.R(0, 0) = -2.0*t1*t5*x[1]*t9+t1*t13*x[1]*t15-t20;
      rot.R(0, 1) = t31;
      rot.R(0, 2) = t32-t33+t35;
      rot.R(1, 0) = t31;
      rot.R(1, 1) = 2.0*t37*t9-2.0*t39*t5*t9+t39*t13*t15-t20;
      rot.R(1, 2) = t46-t48;
      rot.R(2, 0) = -t32+t33-t35;
      rot.R(2, 1) = -t46+t48;
      rot.R(2, 2) = -t20;
	}
	return rot;
}

/*--------------------------------------------------------------------
*
*	QuaternionRotation member functions
*
*--------------------------------------------------------------------*/



QuaternionRotation::QuaternionRotation() {
	q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
}

QuaternionRotation::QuaternionRotation(
	double q0, double q1, double q2, double q3) {

	q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
}

QuaternionRotation::QuaternionRotation(const Rotation3D &r) { from_matrix(r); }

QuaternionRotation::QuaternionRotation(const double *q0) {

	q[0] = q0[0]; q[1] = q0[1]; q[2] = q0[2]; q[3] = q0[3];
}

const char *QuaternionRotation::Name(int i) const {

	switch (i) {
	case 0: return Q0_STR;
	case 1: return Q1_STR;
	case 2: return Q2_STR;
	case 3: return Q3_STR;
	default: return "error";
	}
}
/* Algorithm from Shuster, "A Survey of Attitude Representations"
	Journal of the Astronautical Sciences, oct-dec 1993

	Be carefull!! Shuster is using passive rotations and puts
	q[0] at the end of the quaternion (index 4).
*/
void QuaternionRotation::from_matrix(const Rotation3D &r) {

	int index = 0;
	double alt, sum = 1 + r.Trace();

	alt = 1 + r.R(0,0) - r.R(1,1) - r.R(2,2);
	if (alt > sum) {
		sum = alt;
		index = 1;
	}
	alt = 1 - r.R(0,0) + r.R(1,1) - r.R(2,2);
	if (alt > sum) {
		sum = alt;
		index = 2;
	}
	alt = 1 - r.R(0,0) - r.R(1,1) + r.R(2,2);
	if (alt > sum) {
		sum = alt;
		index = 3;
	}

	// switch to index for maximum numerical resolution
	double tmp;
	switch (index) {
	case 0:
		q[0] = 0.5*sqrt(sum);
		tmp = 0.25/q[0];
		q[1] = (r.R(2,1) - r.R(1,2)) *tmp;
		q[2] = (r.R(0,2) - r.R(2,0)) *tmp;
		q[3] = (r.R(1,0) - r.R(0,1)) *tmp;
		break;

	case 1:
		q[1] = 0.5*sqrt(sum);
		tmp = 0.25/q[1];
		q[0] = (r.R(2,1) - r.R(1,2)) *tmp;
		q[2] = (r.R(1,0) + r.R(0,1)) *tmp;
		q[3] = (r.R(2,0) + r.R(0,2)) *tmp;
		break;
	
	case 2:
		q[2] = 0.5*sqrt(sum);
		tmp = 0.25/q[2];
		q[0] = (r.R(0,2) - r.R(2,0)) *tmp;
		q[1] = (r.R(1,0) + r.R(0,1)) *tmp;
		q[3] = (r.R(2,1) + r.R(1,2)) *tmp;
		break;
	
	case 3:
		q[3] = 0.5*sqrt(sum);
		tmp = 0.25/q[3];
		q[0] = (r.R(1,0) - r.R(0,1)) *tmp;
		q[1] = (r.R(2,0) + r.R(0,2)) *tmp;
		q[2] = (r.R(2,1) + r.R(1,2)) *tmp;
		break;
	
	default:
		std::cerr<<"QuaternionRotation::from_matrix: error in index"<< std::endl;
	}
}


Rotation3D QuaternionRotation::to_matrix(Rotation3D &r) const {

	// temp variables
	double q00 = q[0]*q[0];
	double q01 = q[0]*q[1];
	double q02 = q[0]*q[2];
	double q03 = q[0]*q[3];

	double q11 = q[1]*q[1];
	double q12 = q[1]*q[2];
	double q13 = q[1]*q[3];

	double q22 = q[2]*q[2];
	double q23 = q[2]*q[3];
	double q33 = q[3]*q[3];
	double sql = q00 + q11 + q22 + q33;

	if (sql < 1E-15) {
		std::cerr<<"QuaternionDRotation::to_matrix; squared length == 0"<< std::endl;
		r = Rotation3D();		// assign unit rotation
	}

	r.R(0,0) = (q00 + q11 - q22 - q33)/sql;
	r.R(0,1) = 2*(q12 - q03)/sql;
	r.R(0,2) = 2*(q13 + q02)/sql;

	r.R(1,0) = 2*(q12 + q03)/sql;
	r.R(1,1) = (q00 - q11 + q22 - q33)/sql;
	r.R(1,2) = 2*(q23 - q01)/sql;

	r.R(2,0) = 2*(q13 - q02)/sql;
	r.R(2,1) = 2*(q23 + q01)/sql;
	r.R(2,2) = (q00 - q11 - q22 + q33)/sql;

	return r;
}

double QuaternionRotation::SqLength() const {
	return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
}

int QuaternionRotation::Check() { return 0; }

Rotation3D QuaternionRotation::PartialDeriv(int param) const {

	double t1, t2, t3, t4, t5, t6, t7, t9;
	double t10, t14, t15, t16;
	double t21, t22, t23, t36, t37, t38;
	Rotation3D rot;

    t1 = q[0]*q[0];
    t2 = q[1]*q[1];
    t3 = q[2]*q[2];
    t4 = q[3]*q[3];
    t5 = t1+t2+t3+t4;
    t6 = 1/t5;

	switch (param) {
	case 0:
      t7 = q[0]*t6;
      t9 = t5*t5;
      t10 = 1/t9;
      t14 = q[3]*t6;
      t15 = q[1]*q[2];
      t16 = q[0]*q[3];
      t21 = q[2]*t6;
      t22 = q[1]*q[3];
      t23 = q[0]*q[2];
      t36 = q[1]*t6;
      t37 = q[2]*q[3];
      t38 = q[0]*q[1];
      rot.R(0, 0) = 2.0*t7-2.0*(t1+t2-t3-t4)*t10*q[0];
      rot.R(0, 1) = -2.0*t14-4.0*(t15-t16)*t10*q[0];
      rot.R(0, 2) = 2.0*t21-4.0*(t22+t23)*t10*q[0];
      rot.R(1, 0) = 2.0*t14-4.0*(t15+t16)*t10*q[0];
      rot.R(1, 1) = 2.0*t7-2.0*(t1-t2+t3-t4)*t10*q[0];
      rot.R(1, 2) = -2.0*t36-4.0*(t37-t38)*t10*q[0];
      rot.R(2, 0) = -2.0*t21-4.0*(t22-t23)*t10*q[0];
      rot.R(2, 1) = 2.0*t36-4.0*(t37+t38)*t10*q[0];
      rot.R(2, 2) = 2.0*t7-2.0*(t1-t2-t3+t4)*t10*q[0];
		break;

	case 1:
      t7 = q[1]*t6;
      t9 = t5*t5;
      t10 = 1/t9;
      t14 = q[2]*t6;
      t15 = q[1]*q[2];
      t16 = q[0]*q[3];
      t21 = q[3]*t6;
      t22 = q[1]*q[3];
      t23 = q[0]*q[2];
      t36 = q[0]*t6;
      t37 = q[2]*q[3];
      t38 = q[0]*q[1];
      rot.R(0, 0) = 2.0*t7-2.0*(t1+t2-t3-t4)*t10*q[1];
      rot.R(0, 1) = 2.0*t14-4.0*(t15-t16)*t10*q[1];
      rot.R(0, 2) = 2.0*t21-4.0*(t22+t23)*t10*q[1];
      rot.R(1, 0) = 2.0*t14-4.0*(t15+t16)*t10*q[1];
      rot.R(1, 1) = -2.0*t7-2.0*(t1-t2+t3-t4)*t10*q[1];
      rot.R(1, 2) = -2.0*t36-4.0*(t37-t38)*t10*q[1];
      rot.R(2, 0) = 2.0*t21-4.0*(t22-t23)*t10*q[1];
      rot.R(2, 1) = 2.0*t36-4.0*(t37+t38)*t10*q[1];
      rot.R(2, 2) = -2.0*t7-2.0*(t1-t2-t3+t4)*t10*q[1];
		break;

	case 2:
      t7 = q[2]*t6;
      t9 = t5*t5;
      t10 = 1/t9;
      t14 = q[1]*t6;
      t15 = q[1]*q[2];
      t16 = q[0]*q[3];
      t21 = q[0]*t6;
      t22 = q[1]*q[3];
      t23 = q[0]*q[2];
      t36 = q[3]*t6;
      t37 = q[2]*q[3];
      t38 = q[0]*q[1];
      rot.R(0, 0) = -2.0*t7-2.0*(t1+t2-t3-t4)*t10*q[2];
      rot.R(0, 1) = 2.0*t14-4.0*(t15-t16)*t10*q[2];
      rot.R(0, 2) = 2.0*t21-4.0*(t22+t23)*t10*q[2];
      rot.R(1, 0) = 2.0*t14-4.0*(t15+t16)*t10*q[2];
      rot.R(1, 1) = 2.0*t7-2.0*(t1-t2+t3-t4)*t10*q[2];
      rot.R(1, 2) = 2.0*t36-4.0*(t37-t38)*t10*q[2];
      rot.R(2, 0) = -2.0*t21-4.0*(t22-t23)*t10*q[2];
      rot.R(2, 1) = 2.0*t36-4.0*(t37+t38)*t10*q[2];
      rot.R(2, 2) = -2.0*t7-2.0*(t1-t2-t3+t4)*t10*q[2];
		break;

	case 3:
      t7 = q[3]*t6;
      t9 = t5*t5;
      t10 = 1/t9;
      t14 = q[0]*t6;
      t15 = q[1]*q[2];
      t16 = q[0]*q[3];
      t21 = q[1]*t6;
      t22 = q[1]*q[3];
      t23 = q[0]*q[2];
      t36 = q[2]*t6;
      t37 = q[2]*q[3];
      t38 = q[0]*q[1];
      rot.R(0, 0) = -2.0*t7-2.0*(t1+t2-t3-t4)*t10*q[3];
      rot.R(0, 1) = -2.0*t14-4.0*(t15-t16)*t10*q[3];
      rot.R(0, 2) = 2.0*t21-4.0*(t22+t23)*t10*q[3];
      rot.R(1, 0) = 2.0*t14-4.0*(t15+t16)*t10*q[3];
      rot.R(1, 1) = -2.0*t7-2.0*(t1-t2+t3-t4)*t10*q[3];
      rot.R(1, 2) = 2.0*t36-4.0*(t37-t38)*t10*q[3];
      rot.R(2, 0) = 2.0*t21-4.0*(t22-t23)*t10*q[3];
      rot.R(2, 1) = 2.0*t36-4.0*(t37+t38)*t10*q[3];
      rot.R(2, 2) = 2.0*t7-2.0*(t1-t2-t3+t4)*t10*q[3];
		break;

	default:
		std::cerr<<"QuaternionRotation::PartialDeriv error; unknown parameter"<<std::endl;
		break;
	}
	return rot;
}

/*
int GenericRotation::AddConstraint2NormalEq(double *ata, double *aty,
	const ParameterIntMap &parmap, Parameter **quat,
	double effect, double weight) const {

	double resid;
	double *a = (double*)alloca(parmap.size()*sizeof(double));

	if (Equation(a, &resid, parmap, quat, effect))
		Update_Normal_Eq(a, resid, weight, ata, aty, parmap.size());
	return 0;
}


int QuaternionRotation::Equation(double *a, double *resid,
	const ParameterIntMap &parmap, Parameter **quat, double effect) const {

	int parcount = parmap.size();
	bzero(a, parcount*sizeof(double));
	double q_len = Length();
	ParameterIntMap::const_iterator pi_iter;
	for (int j=0; j<4; j++) if (quat[j]) {

		pi_iter = parmap.find(quat[j]);
		if (pi_iter == parmap.end()) {
			cerr<<"Quaternion::Equation error, did not find quaternion parameter in map"<<endl;
			return 1;
		}
		a[(*pi_iter).second] = effect*quat[j]->Value()/q_len;
		quat[j] = 0;
	}
	*resid = effect*(q_len - 1.0);
	return 1;
}
*/
/*--------------------------------------------------------------------
*
*	TrinionRotation member functions
*
*--------------------------------------------------------------------*/



TrinionRotation::TrinionRotation() {
	t[0] = 1; t[1] = 0; t[2] = 0;
}

TrinionRotation::TrinionRotation(
	double t0, double t1, double t2) {

	t[0] = t0; t[1] = t1; t[2] = t2;
}

TrinionRotation::TrinionRotation(const Rotation3D &r) { from_matrix(r); }

TrinionRotation::TrinionRotation(const double *t0) {

	t[0] = t0[0]; t[1] = t0[1]; t[2] = t0[2];
}

const char *TrinionRotation::Name(int i) const {

	switch (i) {
	case 0: return T0_STR;
	case 1: return T1_STR;
	case 2: return T2_STR;
	default: return "error";
	}
}

void TrinionRotation::from_matrix(const Rotation3D &r) {

	t[0] = r.R(0,2); t[1] = r.R(1,2); t[2] = r.R(2,2);
}


Rotation3D TrinionRotation::to_matrix(Rotation3D &r) const {

	double len = Length();
	if (len < (10.0e-15)) {
		std::cerr<<"TrinionRotation::to_matrix error, length == 0"<<std::endl;
		return r;
	}
	Vector3D zax(t);
	zax /= len;
	r.R(0,2) = zax[0];
	r.R(1,2) = zax[1];
	r.R(2,2) = zax[2]; 

	Vector3D yax;
	yax[0] = r.R(0,1) = 0;	// constrain rotation around z-axis

		// may result in singularities otherwise.
	if (fabs(zax[1]) < (10e-15)) {
		yax[1] = r.R(1,1) = 1;
		yax[2] = r.R(2,1) = 0;

	} else {
		double sqz = zax[1]*zax[1] + zax[2]*zax[2];
		yax[2] = r.R(2,1) = zax[1]/sqrt(sqz);		// sign doesn't matter
		yax[1] = r.R(1,1) = -zax[2]*yax[2]/zax[1];
	}
	// determine x-axis using outer product
	Vector3D xax = yax.VectorProduct(zax);
	r.R(0,0) = xax[0];
	r.R(1,0) = xax[1];
	r.R(2,0) = xax[2];

	return r;
}

double TrinionRotation::SqLength() const {
	return t[0]*t[0] + t[1]*t[1] + t[2]*t[2];
}

int TrinionRotation::Check() { return 0; }

Rotation3D TrinionRotation::PartialDeriv(int param) const {

	Rotation3D rot(0,0,0,0,0,0,0,0,0);
	double sql = SqLength();
	double len = sqrt(sql);
	double parval = t[param]/sql;

	rot.R(param, 2) = (1. - parval)/len;
	int ia = (param + 1)%3;
	rot.R(ia, 2) = -t[ia]*parval/len;
	ia = (param + 2)%3;
	rot.R(ia, 2) = -t[ia]*parval/len;

	/* alternative
	rot.R(param, 2) = 1.;
	*/

	return rot;
}
/*
int TrinionRotation::AddConstraint2NormalEq(double *ata, double *aty,
	const ParameterIntMap &parmap, Parameter **trin,
	double effect_est, double weight) const {

	int parcount = parmap.size();
	double *a = (double*)alloca(parcount *sizeof(double));
	bzero(a, parcount*sizeof(double));

	double q_len = Length();
	ParameterIntMap::const_iterator pi_iter;
	for (int j=0; j<3; j++) if (trin[j]) {

		pi_iter = parmap.find(trin[j]);
		if (pi_iter == parmap.end()) {
			cerr<<"Trinion::AddConstraint2NormalEq error, did not find trinion parameter in map"<<endl;
			return 1;
		}
		a[(*pi_iter).second] = effect_est*trin[j]->Value()/q_len;
		trin[j] = 0;
	}
	double resid = q_len - 1.0;
	Update_Normal_Eq(a, effect_est*resid, weight, ata, aty, parcount);
	return 0;
}
*/
/*
int TrinionRotation::Equation(double *a, double *resid,
	const ParameterIntMap &parmap, Parameter **trin, double effect) const {

	bzero(a, parmap.size()*sizeof(double));
	double q_len = Length();
	ParameterIntMap::const_iterator pi_iter;
	for (int j=0; j<3; j++) if (trin[j]) {

		pi_iter = parmap.find(trin[j]);
		if (pi_iter == parmap.end()) {
			cerr<<"Quaternion::Equation error, did not find quaternion parameter in map"<<endl;
			return 1;
		}
		a[(*pi_iter).second] = effect*trin[j]->Value()/q_len;
		trin[j] = 0;
	}
	*resid = effect*(q_len - 1.0);
	return 1;
}
*/
