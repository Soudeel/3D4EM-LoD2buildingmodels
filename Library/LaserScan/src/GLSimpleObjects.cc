
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
*   Purpose   : GLCylinders for drawing simple geometries like cylinder,plane etc.
*
*--------------------------------------------------------------------*/

#include "GLSimpleObjects.h"

	 
void GLCylinder::Draw()
{
	glEnable(GL_LIGHTING);
	cylinder.Draw();
}

void GLCylinder::GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
{  
	//cerr<<"GLCylinder::GetBoundingBox"<<endl;
	minimum = cylinder.pointBegin;
	maximum = cylinder.pointEnd;
}

string GLCylinder::ToString()
{  
	return "GLCylinder";
}


GLCylinder* GLCylinder::MakeCopy()
{  
	return new GLCylinder(*this);
}

void GLPlaneTR::Draw()
{
	glEnable(GL_LIGHTING);
	plane.Draw();
}

void GLPlaneTR::GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
{  
	plane.GetBoundingBox(minimum,maximum);
}

string GLPlaneTR::ToString()
{  
	return "GLPlaneTR";
}


GLPlaneTR* GLPlaneTR::MakeCopy()
{  
	return new GLPlaneTR(*this);
}

