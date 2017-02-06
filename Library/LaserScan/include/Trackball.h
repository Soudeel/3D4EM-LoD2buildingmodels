
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
*--------------------------------------------------------------------*/
#ifndef _VIRTUAL_TRACKBALL_H_
#define _VIRTUAL_TRACKBALL_H_
#include "Rotation3D.h"
#include "RotationParameter.h"
#include "MyConstants.h"
class VirtualTrackball
{
	public:
		VirtualTrackball();
		void ProcessMouse(int x1,int y1,int x2, int y2, int width, int height);
		void ProcessMouse(int x1,int y1,int x2, int y2, int width, int height,double speed);
		void MultiplyWithRotationMatrix();
		double* GetRotationMatrix();
		//Angle is in degrees.
		void SetAngleAxis(double angle,double x, double y, double z);
		AngleAxisRotation GetAngleAxis()
		{
			double angle,x,y,z;
			GetAngleAxis(angle,x,y,z);
			return AngleAxisRotation(Vector3D(x,y,z),angle*DEGREE_TO_RADIAN);
		}
		
		void SetAngleAxis(AngleAxisRotation a)
		{
			SetAngleAxis(a.Length()*RADIAN_TO_DEGREE,a.X(),a.Y(),a.Z());
		}
		void GetAngleAxis(double& angle,double& x, double &y, double& z);
		void IncrementAngle(double inc=5);
		
		//Spherical linear interpolation.
		void Slerp	(double ax,double ay, double az, double alpha,
	  				double bx, double by, double bz, double beta,
	  				double t);
		//Slerp using quaternions.
		void Slerp(double dq0[],double dq1[],double t);
		
		//Copies the contents of totalQuaternion to the passed double array.
		void GetQuaternion(double dq[]);
		
		///Sets quaternion.
		void SetQuaternion(const QuaternionRotation& q);
		
		///Get quaternion.
		QuaternionRotation GetQuaternion()const;


	private:
		double totalQuaternion[4];
		double rotationMatrix[4][4];
};


#endif //_VIRTUAL_TRACKBALL_H_
