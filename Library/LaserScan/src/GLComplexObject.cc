
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
*   File made : February 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Complex object. Conatins a vector of virtual GLObject pointers.
*
*--------------------------------------------------------------------*/
#include "GLComplexObject.h"
#include "math.h"

#define DEBUG printf

GLComplexObject::GLComplexObject()
{
	color = Vector3D(1,1,1);
	id = -1;
	selectionCallback = NULL;
	selectionCallbackData = NULL;
}


GLComplexObject::GLComplexObject(Vector3D c,int newId):color(c)
{
	id = newId;
	selectionCallback = NULL;
}

GLComplexObject::GLComplexObject(const GLComplexObject& newObject)
{
	if(this == &newObject)
	{
		return;
	}
	
	glObjectPointers = newObject.glObjectPointers;
	color = newObject.color;
	id = newObject.id;
	selectionCallback = newObject.selectionCallback;
	selectionCallbackData = newObject.selectionCallbackData;
}

GLComplexObject::~GLComplexObject()
{
	//call virtual destructor for all contained pointers.
	for(int i=0;i<glObjectPointers.size();i++)
	{
		delete glObjectPointers[i];
	}
}


GLObject* 
GLComplexObject::MakeCopy()
{
	GLComplexObject* newObject = new GLComplexObject(*this);
	return (GLObject*)newObject;
}


GLObjectTypeId 
GLComplexObject::GetObjectType()
{
	return GLComplexObjectType;
}


GLObjectPointers& 
GLComplexObject::GetGLObjectPointers()
{
	return glObjectPointers;
}


void 
GLComplexObject::SetGLObjectPointers(const GLObjectPointers& newPointers)
{
	for(int i=0;i<glObjectPointers.size();i++)
	{
		delete glObjectPointers[i];
	}
	glObjectPointers = newPointers;
}

void
GLComplexObject::Draw()
{
	for(int i=0;i<glObjectPointers.size();i++)
	{
		glObjectPointers[i]->Draw();
	}
}

void
GLComplexObject::GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
{
	
	if(glObjectPointers.empty())
	{
		minimum = Vector3D(-10,-10,-10);
		maximum = Vector3D(10,10,10);
	}
	else
	{
		//Combine bounding boxes of all component objects.
		Vector3D mx,mn;
		
		minimum = Vector3D(1e10,1e10,1e10);
		maximum = Vector3D(-1e10,-1e10,-1e10);
		for(int i=0;i<glObjectPointers.size();i++)
		{
			Vector3D mn,mx;
			glObjectPointers[i]->GetBoundingBox(mn,mx);
			minimum.X() = MIN(minimum.X(),mn.X());
			minimum.Y() = MIN(minimum.Y(),mn.Y());
			minimum.Z() = MIN(minimum.Z(),mn.Z());
			maximum.X() = MAX(maximum.X(),mx.X());
			maximum.Y() = MAX(maximum.Y(),mx.Y());
			maximum.Z() = MAX(maximum.Z(),mx.Z());
		}	
	}
}

string
GLComplexObject::ToString()
{
	char buff[1024];
	sprintf(buff,"GLComplexObject with %d objects at address %p",glObjectPointers.size(),this);
	return string(buff);
}

Vector3D
GLComplexObject::GetColor()
{
	return color;
}

void
GLComplexObject::SetColor(const Vector3D& c)
{
	color = c;
}




