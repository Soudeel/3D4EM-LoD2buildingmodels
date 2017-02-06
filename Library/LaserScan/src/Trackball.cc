
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
*   File made : July 2002
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to Trackball like mouse manipulation for OpenGL.
*
*	Note	  : The internal representation of quaternion differs from 
				QuaternionRotation(qr). If internal is qt then:
				qt[3] = qr[0]; 
				qt[0] = qr[1];
				qt[1] = qr[2];
				qt[2] = qr[3];
				Additionally internal representation when converted to
				Rotation3D() gives a transposed matrix (same as the one used 
				by OpenGL).
*--------------------------------------------------------------------*/
#include "OpenGLHeaders.h"
#include <math.h>
#include <iostream>
#include "Trackball.h"
#include "OpenGLUtility.h"

#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"

/*
 * Pass the x and y coordinates of the last and current positions of
 * the mouse, scaled so they are from (-1.0 ... 1.0).
 *
 * The resulting rotation is returned as a quaternion rotation in the
 * first paramater.
 */
void trackball (double q[4], double p1x, double p1y, double p2x, double p2y, double speed=1.0);

/*
 * Given two quaternions, add them together to get a third quaternion.
 * Adding quaternions to get a compound rotation is analagous to adding
 * translations to get a compound translation.  When incrementally
 * adding rotations, the first argument here should be the new
 * rotation, the second and third the total rotation (which will be
 * over-written with the resulting new total rotation).
 */
void add_quats (double *q1, double *q2, double *dest);

static void
normalize_quat (double q[4]);


/*
 * A useful function, builds a rotation matrix in Matrix based on
 * given quaternion.
 */
void build_rotmatrix (double m[4][4], double q[4]);

/*
 * This function computes a quaternion based on an axis (defined by
 * the given vector) and an angle about which to rotate.  The angle is
 * expressed in radians.  The result is put into the third argument.
 */
void axis_to_quat (double a[3], double phi, double q[4]);

void quat_to_angle_axis(double q[4],double a[3],double& angle);


VirtualTrackball::VirtualTrackball ()
{
	double axis[3] = { 1, 1, 1 };

	axis_to_quat (axis, M_PI / 3.00, totalQuaternion);
	build_rotmatrix (rotationMatrix, totalQuaternion);
}

void 
VirtualTrackball::SetAngleAxis(double angle,double x, double y, double z)
{
	double axis[3] = { x, y, z };
	angle = angle*M_PI/180.00;
	axis_to_quat (axis, angle, totalQuaternion);
	build_rotmatrix (rotationMatrix, totalQuaternion);
}


///Sets quaternion.
void VirtualTrackball::SetQuaternion(const QuaternionRotation& q_in)
{
	totalQuaternion[3] = -q_in[0];
	
	for(int i=0;i<3;i++)
		totalQuaternion[i] = q_in[i+1];	
		
	normalize_quat(totalQuaternion);
	
	build_rotmatrix (rotationMatrix, totalQuaternion);
}

		
///Get quaternion.
QuaternionRotation VirtualTrackball::GetQuaternion()const
{
	return QuaternionRotation(-totalQuaternion[3],totalQuaternion[0],totalQuaternion[1],totalQuaternion[2]);
}

void
VirtualTrackball::GetAngleAxis(double& angle,double& x, double &y, double& z)
{
	double axis[3];
	double ang;
	quat_to_angle_axis(totalQuaternion,axis,ang);
	angle = ang;
	x = axis[0]; y = axis[1]; z = axis[2];
}

void
VirtualTrackball::IncrementAngle(double inc)
{
	double angle,x,y,z;
	GetAngleAxis(angle,x,y,z);
	printf("(%f %f %f) ^ %f\n",x,y,z,angle);
	SetAngleAxis(angle+inc,x,y,z);
}
	

void
VirtualTrackball::ProcessMouse (int x1, int y1, int x2, int y2, int width,
								int height,double speed)
{
#if 1	
	double currentQuaternion[4];

	trackball (currentQuaternion,
			   (2.0 * x1 - width) / (double) width,
			   (height - 2.0 * y1) / (double) height,
			   (2.0 * x2 - width) / (double) width,
			   (height - 2.0 * y2) / (double) height,
			   speed);
	add_quats (currentQuaternion, totalQuaternion, totalQuaternion);
#else
	QuaternionRotation qOld = GetQuaternion();
	AngleAxisRotation rot(Vector3D(1,0,0),(x1-x2)/(double)(width)*M_PI);
	Rotation3D rNew = Rotation3D(rot)*Rotation3D(qOld);
	SetQuaternion(QuaternionRotation(rNew));
#endif	
}

void
VirtualTrackball::ProcessMouse (int x1, int y1, int x2, int y2, int width,
								int height)
{
	return ProcessMouse (x1, y1, x2, y2, width, height, 1.00);
}

void
VirtualTrackball::MultiplyWithRotationMatrix ()
{
	build_rotmatrix (rotationMatrix, totalQuaternion);
	glMultMatrixd (rotationMatrix[0]);
}

double *
VirtualTrackball::GetRotationMatrix ()
{
	build_rotmatrix (rotationMatrix, totalQuaternion);
	return rotationMatrix[0];
}

void
VirtualTrackball::Slerp(double ax,double ay, double az, double alpha,
	  double bx, double by, double bz, double beta,
	  double t)
{
	double q0[4],q1[4];
	double a0[3] = {ax,ay,az};
	double a1[3] = {bx,by,bz};
	
	axis_to_quat (a0, alpha*M_PI/180.0, q0);
	axis_to_quat (a1, beta*M_PI/180.0, q1);
	
	double phi = 0;
	for(int i=0;i<4;i++)
		phi += q0[i]*q1[i];
	phi = acos(phi);
	
	double c0 = sin((1.0-t)*phi)/sin(phi);
	double c1 = sin(t*phi)/sin(phi);
	
	for(int i=0;i<4;i++)
		totalQuaternion[i] = c0*q0[i]+c1*q1[i];
}

void
VirtualTrackball::Slerp(double dq0[],double dq1[],double t)
{
	double q0[4],q1[4];
	for(int i=0;i<4;i++)
	{
		q0[i]=dq0[i];
		q1[i]=dq1[i];
	}

	normalize_quat(q0);
	normalize_quat(q1);
	
	double phi = 0;
	for(int i=0;i<4;i++)
		phi += q0[i]*q1[i];

	if(fabs(phi-1)<1e-3)
	{
		//zero angle we better not process it as will produce nan's
		return;
	}
	
	phi = acos(phi);

	double c0 = sin((1.0-t)*phi)/sin(phi);
	double c1 = sin(t*phi)/sin(phi);
	
	
	for(int i=0;i<4;i++)
	{
		totalQuaternion[i] = c0*q0[i]+c1*q1[i];
	}
}

void
VirtualTrackball::GetQuaternion(double dq[])
{
	for(int i=0;i<4;i++)
		dq[i]=(double)totalQuaternion[i];
}

	

	
	



/*
 * Trackball code:
 *
 * Implementation of a virtual trackball.
 * Implemented by Gavin Bell, lots of ideas from Thant Tessman and
 *   the August '88 issue of Siggraph's "Computer Graphics," pp. 121-129.
 *
 * Vector manip code:
 *
 * Original code from:
 * David M. Ciemiewicz, Mark Grossman, Henry Moreton, and Paul Haeberli
 *
 * Much mucking with by:
 * Gavin Bell
 */

/*
 * This size should really be based on the distance from the center of
 * rotation to the point on the object underneath the mouse.  That
 * point would then track the mouse as closely as possible.  This is a
 * simple example, though, so that is left as an Exercise for the
 * Programmer.
 */
#define TRACKBALLSIZE  (0.8)

/*
 * Local function prototypes (not defined in trackball.h)
 */
static double tb_project_to_sphere (double, double, double);
static void normalize_quat (double[4]);

static void
vzero (double *v)
{
	v[0] = 0.0;
	v[1] = 0.0;
	v[2] = 0.0;
}

static void
vset (double *v, double x, double y, double z)
{
	v[0] = x;
	v[1] = y;
	v[2] = z;
}

static void
vsub (const double *src1, const double *src2, double *dst)
{
	dst[0] = src1[0] - src2[0];
	dst[1] = src1[1] - src2[1];
	dst[2] = src1[2] - src2[2];
}

static void
vcopy (const double *v1, double *v2)
{
	register int i;
	for (i = 0; i < 3; i++)
		v2[i] = v1[i];
}

static void
vcross (const double *v1, const double *v2, double *cross)
{
	double temp[3];

	temp[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	temp[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	temp[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
	vcopy (temp, cross);
}

static double
vlength (const double *v)
{
	return sqrt (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void
vscale (double *v, double div)
{
	v[0] *= div;
	v[1] *= div;
	v[2] *= div;
}

static void
vnormal (double *v)
{
	vscale (v, 1.0 / vlength (v));
}

static double
vdot (const double *v1, const double *v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

static void
vadd (const double *src1, const double *src2, double *dst)
{
	dst[0] = src1[0] + src2[0];
	dst[1] = src1[1] + src2[1];
	dst[2] = src1[2] + src2[2];
}

/*
 * Ok, simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */
void
trackball (double q[4], double p1x, double p1y, double p2x, double p2y,double speed)
{
	double a[3];					/* Axis of rotation */
	double phi;					/* how much to rotate about axis */
	double p1[3], p2[3], d[3];
	double t;

	if (p1x == p2x && p1y == p2y)
	{
		/* Zero rotation */
		vzero (q);
		q[3] = 1.0;
		return;
	}
	
	double trackballSize = TRACKBALLSIZE/speed;
	/*
	 * First, figure out z-coordinates for projection of P1 and P2 to
	 * deformed sphere
	 */
	vset (p1, p1x, p1y, tb_project_to_sphere (trackballSize, p1x, p1y));
	vset (p2, p2x, p2y, tb_project_to_sphere (trackballSize, p2x, p2y));

	/*
	 *  Now, we want the cross product of P1 and P2
	 */
	vcross (p2, p1, a);

	/*
	 *  Figure out how much to rotate around that axis.
	 */
	vsub (p1, p2, d);
	t = vlength (d) / (2.0 * trackballSize);

	/*
	 * Astatic void problems with out-of-control values...
	 */
	if (t > 1.0)
		t = 1.0;
	if (t < -1.0)
		t = -1.0;
	phi = 2.0 * asin (t);

	axis_to_quat (a, phi, q);
}

/*
 *  Given an axis and angle, compute quaternion.
 */
void
axis_to_quat (double a[3], double phi, double q[4])
{
	vnormal (a);
	vcopy (a, q);
	vscale (q, sin (phi / 2.0));
	q[3] = cos (phi / 2.0);
}


//Change the quaternion to axis angle.
void quat_to_angle_axis(double q[4],double a[3],double& angle)
{
	double scale = 0;
	for(int i=0;i<3;i++)
		scale += q[i]*q[i];
	
	scale = sqrt(scale);
	if(scale>0)
	{
		for(int i=0;i<3;i++)
			a[i]=q[i]/scale;
		
		angle = (acos(q[3])*2)*180.00/M_PI;
	}
	else
	{
		angle = 0;
		
		a[0]=1; a[1]=a[2]=0;
	}
}

/*
 * Project an x,y pair onto a sphere of radius r OR a hyperbolic sheet
 * if we are away from the center of the sphere.
 */
static double
tb_project_to_sphere (double r, double x, double y)
{
	double d, t, z;

	d = sqrt (x * x + y * y);
	if (d < r * 0.70710678118654752440)
	{							/* Inside sphere */
		z = sqrt (r * r - d * d);
	}
	else
	{							/* On hyperbola */
		t = r / 1.41421356237309504880;
		z = t * t / d;
	}
	return z;
}

/*
 * Given two rotations, e1 and e2, expressed as quaternion rotations,
 * figure out the equivalent single rotation and stuff it into dest.
 *
 * This routine also normalizes the result every RENORMCOUNT times it is
 * called, to keep error from creeping in.
 *
 * NOTE: This routine is written so that q1 or q2 may be the same
 * as dest (or each other).
 */

#define RENORMCOUNT 97

void
add_quats (double q1[4], double q2[4], double dest[4])
{
	static int count = 0;
	double t1[4], t2[4], t3[4];
	double tf[4];

	vcopy (q1, t1);
	vscale (t1, q2[3]);

	vcopy (q2, t2);
	vscale (t2, q1[3]);

	vcross (q2, q1, t3);
	vadd (t1, t2, tf);
	vadd (t3, tf, tf);
	tf[3] = q1[3] * q2[3] - vdot (q1, q2);

	dest[0] = tf[0];
	dest[1] = tf[1];
	dest[2] = tf[2];
	dest[3] = tf[3];

	if (++count > RENORMCOUNT)
	{
		count = 0;
		normalize_quat (dest);
	}
}

/*
 * Quaternions always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
 * If they don't add up to 1.0, dividing by their magnitued will
 * renormalize them.
 *
 * Note: See the following for more information on quaternions:
 *
 * - Shoemake, K., Animating rotation with quaternion curves, Computer
 *   Graphics 19, No 3 (Proc. SIGGRAPH'85), 245-254, 1985.
 * - Pletinckx, D., Quaternion calculus as a basic tool in computer
 *   graphics, The Visual Computer 5, 2-13, 1989.
 */
void
normalize_quat (double q[4])
{
	int i;
	double mag;

	mag = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	
	for (i = 0; i < 4; i++)
		q[i] /= mag;
}

/*
 * Build a rotation matrix, given a quaternion rotation.
 *
 */
void
build_rotmatrix (double m[4][4], double q[4])
{
	normalize_quat(q);
	
	m[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
	m[0][1] = 2.0 * (q[0] * q[1] - q[2] * q[3]);
	m[0][2] = 2.0 * (q[2] * q[0] + q[1] * q[3]);
	m[0][3] = 0.0;

	m[1][0] = 2.0 * (q[0] * q[1] + q[2] * q[3]);
	m[1][1] = 1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]);
	m[1][2] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
	m[1][3] = 0.0;

	m[2][0] = 2.0 * (q[2] * q[0] - q[1] * q[3]);
	m[2][1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
	m[2][2] = 1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]);
	m[2][3] = 0.0;

	m[3][0] = 0.0;
	m[3][1] = 0.0;
	m[3][2] = 0.0;
	m[3][3] = 1.0;
}
