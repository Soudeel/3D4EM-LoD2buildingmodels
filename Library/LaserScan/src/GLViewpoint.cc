
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
*   Purpose   : Class to describe custom view point. Similar to Viewpoint node in Vrml 2.0
*
*--------------------------------------------------------------------*/
#include "GLViewpoint.h"


GLViewpoint::GLViewpoint()
{
	rotationAngleDegrees = 0;
	fovAngleDegrees = 60;
}

GLViewpoint::GLViewpoint(Vector3D tr,Vector3D rotAxis,double angle,double fov,string s)
{
	translation = tr;
	rotationAxis = rotAxis;
	rotationAngleDegrees = angle;
	fovAngleDegrees = fov;
	name = s;
}
GLViewpoint::GLViewpoint(const GLViewpoint& vp)
{
	if(&vp == this)
		return;
	
	this->translation = vp.translation;
	this->rotationAxis = vp.rotationAxis;
	this->rotationAngleDegrees = vp.rotationAngleDegrees;
	this->fovAngleDegrees = vp.fovAngleDegrees;
	this->name = vp.name;
}
	
GLViewpoint::~GLViewpoint()
{
}

string GLViewpoint::GetName()
{
	return name;
}

void GLViewpoint::SetName(string s)
{
	name = s;
}

void GLViewpoint::ApplyGLTransformation(double aspectRatio,double nearZ,double farZ)
{
	//order is fixed
	//first translate then rotate.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovAngleDegrees,aspectRatio,nearZ,farZ);
	
	if(0)
	{
		cerr<<"fovAngleDegrees: "<<fovAngleDegrees<<endl;
		cerr<<"aspectRatio: "<<aspectRatio<<endl;
		cerr<<"nearZ: "<<nearZ<<endl;
		cerr<<"farZ: "<<farZ<<endl;
		cerr<<"translation: "<<translation.X()<<" "<<translation.Y()<<" "<<translation.Z()<<endl;
		cerr<<"rotationAngleDegrees: "<<rotationAngleDegrees<<endl;
		cerr<<"rotationAxis: "<<rotationAxis.X()<<" "<<rotationAxis.Y()<<" "<<rotationAxis.Z()<<endl;
	}
		
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(rotationAngleDegrees,rotationAxis.X(),rotationAxis.Y(),rotationAxis.Z());
	glTranslatef(translation.X(),translation.Y(),translation.Z());
	
}


