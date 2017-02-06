
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
*   File made : March 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Transform 3D object. Keeps rotation and translation together.
*		Its being reimplemented because of memory problems with Exterior Orientation class.
*
*--------------------------------------------------------------------*/
#include "LaserTransform3D.h"
#include "RegistrationStructs.h"
//Constructor.
LaserTransform3D::LaserTransform3D(const QuaternionRotation& r, const Vector3D& t)
:rot(r),trans(t)
{
}

//Constructor.
LaserTransform3D::LaserTransform3D(double q0,double q1,double q2,double q3,
		double t0, double t1,double t2)
{
	rot = QuaternionRotation(q0,q1,q2,q3);
	trans = Vector3D(t0,t1,t2);
}

//Print some info to a file or stderr.
void LaserTransform3D::Print(FILE* pFile)
{
	fprintf(pFile,"Transform: Quaternion (%f, %f, %f, %f)  Translation (%f, %f, %f)\n",rot[0],rot[1],rot[2],rot[3],trans[0],trans[1],trans[2]);
}

//Apply the transform to a given object and return the parameters after transformation.
vector<double> LaserTransform3D::ApplyToObject(RegistrationTarget target)
{
	vector<double> p = target.parameters;

	if(target.type=="LaserPlane")
	{
		Vector3D normal(p[0],p[1],p[2]);
		double rho = p[3];

		//Rotate the normal vector.
		normal = rot.to_matrix()*normal;
		rho = rho + normal.DotProduct(trans);

		//Now copy them back.
		p[0] = normal.X(); p[1] = normal.Y(); p[2] = normal.Z();
		p[3] = rho;
	}
	else if(target.type=="LaserCylinder")
	{
		Vector3D axis(p[0],p[1],p[2]);
		Vector3D ptOnAxis(p[3],p[4],p[5]);

		//Rotate the axis vector.
		axis = rot.to_matrix()*axis;

		ptOnAxis = (rot.to_matrix()*ptOnAxis) + trans;

		//Make ptOnAxis point closest to origin.

		ptOnAxis = ptOnAxis - axis*(axis.DotProduct(ptOnAxis));

		//Now copy the values back.
		for(int i=0;i<3;i++)
			p[i] = axis[i];
		for(int i=0;i<3;i++)
			p[i+3] = ptOnAxis[i];
		/*
		cerr<<"Cylinder: ";
		for(int i=0;i<6;i++)
			cerr<<p[i]<<"   ";
		cerr<<endl;
		*/
	}
	else if(target.type == "LaserSphere")
	{
		Vector3D center(p[0],p[1],p[2]);

		//Rotate the normal vector.
		center = rot.to_matrix()*center + trans;

		//Now copy them back.
		for(int k=0;k<3;k++)
			p[k] = center[k];				
	}
	else if(target.type == "LaserTorus")
	{
		Vector3D axis(p[0],p[1],p[2]);
		Vector3D ptOnAxis(p[3],p[4],p[5]);

		//Rotate the axis vector.
		axis = rot.to_matrix()*axis;

		ptOnAxis = rot.to_matrix()*ptOnAxis + trans;

		//Now copy the values back.
		for(int i=0;i<3;i++)
			p[i] = axis[i];
		for(int i=0;i<3;i++)
			p[i+3] = ptOnAxis[i];
	}

	//don't forget to return p.
	return p;
}

Vector3D LaserTransform3D::operator * (const Vector3D& v)const
{
	Rotation3D R = rot.to_matrix();
	return R*v + trans;
}

//Transform a Laser point.
LaserPoint LaserTransform3D::operator * (const LaserPoint& v)const
{
	Vector3D m = (*this)*((Vector3D)v);
	LaserPoint p = v;
	for(int i=0;i<3;i++)
		p[i] = m[i];
	return p;
}

//Transform vector of laser points.
LaserPoints LaserTransform3D::operator * (const LaserPoints& v)const
{
	LaserPoints result = v;
	Rotation3D R = rot.to_matrix();
	for(int i=0;i<v.size();i++)
	{
		Vector3D a = R*Vector3D(v[i]) + trans;
		for(int j=0;j<3;j++)
			result[i][j] = a[j];
	}
	return result;
}


//Get the inverse transformation
LaserTransform3D LaserTransform3D::Inverse()const
{
	NEWMAT::Matrix m = ToMatrix();
	NEWMAT::Matrix m_inv = m.i();
	return LaserTransform3D(m_inv);
}

//A wrapper for inverst
LaserTransform3D LaserTransform3D::i()const
{
	return Inverse();
}

//Operator for inverse
LaserTransform3D LaserTransform3D::operator!()const
{
	return Inverse();
}

//Converts to a 4x4 matrix
NEWMAT::Matrix LaserTransform3D::ToMatrix()const
{
	NEWMAT::Matrix m(4,4);
	Rotation3D R = rot.to_matrix();

	for(int i=0;i<3;i++)
	{
		m(i+1,4) = trans[i];
		for(int j=0;j<3;j++)
			m(i+1,j+1) = R.R(i,j);
	}
	m(4,4) = 1;
	return m;
}

//Constructor from matrix
LaserTransform3D::LaserTransform3D (NEWMAT::Matrix m)
{
	Rotation3D R;
	for(int i=0;i<3;i++)
	{
		trans[i] = m(i+1,4);
		for(int j=0;j<3;j++)
			R.R(i,j) = m(i+1,j+1);
	}
	rot = QuaternionRotation(R);
}


//multiply two transforms together.
LaserTransform3D operator*(const LaserTransform3D& t1, const LaserTransform3D& t2)
{
	return LaserTransform3D(t1.ToMatrix()*t2.ToMatrix());
}

///ostream << operator.
ostream& operator<<(ostream& os,const LaserTransform3D& t)
{
	os<<"Rotation: \n";
	Rotation3D p(t.rot);
	os << p.R(0,0) <<" "<< p.R(0,1) <<" " << p.R(0,2) <<" "<<endl;
	os << p.R(1,0) <<" "<< p.R(1,1) <<" " << p.R(1,2) <<" "<<endl;
	os << p.R(2,0) <<" "<< p.R(2,1) <<" " << p.R(2,2) <<" "<<endl;
	
	os<<"Quaternion: \n";
	for(int i=0;i<4;i++)
		os<<t.rot[i]<<"  ";
	os<<endl;
	
	os<<"Translation: \n";
	for(int i=0;i<3;i++)
		os<<t.trans[i]<<"  ";
	os<<endl;
	return os;
}
