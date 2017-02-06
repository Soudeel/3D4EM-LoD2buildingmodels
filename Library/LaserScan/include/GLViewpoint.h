
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
#ifndef _GL_VIEWPOINT_H_
#define _GL_VIEWPOINT_H_

#include <vector>
#include <set>

#include "GLObject.h"
#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"

using namespace std;

class GLViewpoint
{
	public:
		GLViewpoint();
		GLViewpoint(Vector3D tr,Vector3D rotAxis,double angle,double fov,string name);
		GLViewpoint(const GLViewpoint& vp);
		~GLViewpoint();
		string GetName();
		void SetName(string s);
		void ApplyGLTransformation(double aspectRatio,double nearZ,double farZ);
	protected:
		Vector3D translation;
		Vector3D rotationAxis;
		double rotationAngleDegrees;
		double fovAngleDegrees;
		string name;
};

typedef std::vector<GLViewpoint> GLViewpointVector;
#endif //_GL_VIEWPOINT_H_


