
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
#ifndef _TRANSFORM_3D__H_
#define _TRANSFORM_3D__H_

#include <vector>
#include <string.h>
#include <stdio.h>
#include "Vector3D.h"
#include "Rotation3D.h"
#include "RotationParameter.h"
#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines
class LaserPoints;
class RegistrationTarget;
class LaserPoint;
class NEWMAT::Matrix;

struct LaserTransform3D
{
	//Rotation is represented as a quaternion.
	QuaternionRotation rot;
	
	//Translation as vector3d
	Vector3D trans;
	
	//Constructor.
	LaserTransform3D(const QuaternionRotation& r=QuaternionRotation(1,0,0,0), const Vector3D& t=Vector3D(0,0,0));
	
	//Constructor with 7-elements.
	LaserTransform3D(double q0,double q1,double q2,double q3,
			double t0, double t1,double t2);
		
	//Print some info to a file or stderr.
	void Print(FILE* pFile=stderr);
	
	//Apply the transform to a given object and return the parameters after transformation.
	std::vector<double> ApplyToObject(RegistrationTarget target);
	//Load values from an iterator.
	template<class T>
	bool LoadFromVector(T vals)
	{
		rot = QuaternionRotation(vals[0],vals[1],vals[2],vals[3]);
		trans = Vector3D(vals[4],vals[5],vals[6]);
	}
	
	//Return rotation matrix.
	Rotation3D Rotation()const
	{
		return rot.to_matrix();
	}
	
	//return translation vector.
	Vector3D Translation()const
	{
		return trans;
	}
	
	
	//Save values from an iterator.
	template<class T>
	bool SaveToVector(T vals)
	{
		for(int i=0;i<4;i++)
		{
			*vals = rot[i];
			vals++;
		}
		
		for(int i=0;i<3;i++)
		{
			*vals = trans[i];
			vals++;
		}
	}
	
	//Transform a 3D vector;
	Vector3D operator * (const Vector3D& v)const;
	
	//Transform a Laser point.
	LaserPoint operator * (const LaserPoint& v)const;
	
	//Transform a vector of laser points.
	LaserPoints operator * (const LaserPoints& v)const;
	
	///ostream << operator.
	friend ostream& operator<<(ostream& os,const LaserTransform3D& trans);
		
	//Friend operator for multiplication.
	friend LaserTransform3D operator*(const LaserTransform3D& t1, const LaserTransform3D& t2);
	
	//Return the inverse transformation.
	LaserTransform3D Inverse()const;
	
	//A wrapper for inverse
	LaserTransform3D i()const;
	
	//! operator for inverse.
	LaserTransform3D operator!()const;
		
	//Convert to a 4x4 matrix
	NEWMAT::Matrix ToMatrix()const;
	
	//Constructor from matrix
	LaserTransform3D (NEWMAT::Matrix m);
		
};

#endif //_TRANSFORM_3D__H_
