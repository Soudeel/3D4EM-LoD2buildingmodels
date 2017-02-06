
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
*   File made : May 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : GLObjects for drawing simple geometries like cylinder,plane etc.
*
*--------------------------------------------------------------------*/
#ifndef _GL_SIMPLE_OBJECTS_H_
#define _GL_SIMPLE_OBJECTS_H_

#include "GLObject.h"
#include "LaserObjects.h"

class GLCylinder:public GLObject
{
public:
	GLCylinder():GLObject(){cylinder=LaserCylinder();};
	GLCylinder(const LaserCylinder& lc):GLObject(),cylinder(lc){}
	virtual ~GLCylinder(){};
	virtual void Draw();
	virtual void GetBoundingBox(Vector3D& minimum,Vector3D& maximum);
	virtual string ToString();
	virtual GLCylinder* MakeCopy();
	virtual GLObjectTypeId GetObjectType(){return GLCylinderType;}; 
		
protected:
	LaserCylinder cylinder;
};

class GLPlaneTR:public GLObject
{
public:
	GLPlaneTR():GLObject(){plane = LaserPlaneTR();};
	GLPlaneTR(const LaserPlaneTR& lp):GLObject(),plane(lp){}
	virtual ~GLPlaneTR(){};
	virtual void Draw();
	virtual void GetBoundingBox(Vector3D& minimum,Vector3D& maximum);
	virtual string ToString();
	virtual GLPlaneTR* MakeCopy();
	virtual GLObjectTypeId GetObjectType(){return GLPlaneTRType;}; 
		
protected:
	LaserPlaneTR plane;
};


#endif //_GL_SIMPLE_OBJECTS_H_


