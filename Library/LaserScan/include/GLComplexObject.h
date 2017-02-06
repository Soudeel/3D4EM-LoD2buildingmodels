
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
#ifndef _GL_COMPLEX_OBJECT_H_
#define _GL_COMPLEX_OBJECT_H_

#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "GLObject.h"
#include "MyTypeDefinitions.h"
#include "GLObject.h"


//using namespace std;

class GLComplexObject:public GLObject
{
	public:
		GLComplexObject();
		GLComplexObject(Vector3D c,int id);
		GLComplexObject(const GLComplexObject& newFaceSet);
		~GLComplexObject();
		GLObject* MakeCopy();
		GLObjectTypeId GetObjectType(); 

		GLObjectPointers& GetGLObjectPointers();
		void SetGLObjectPointers(const GLObjectPointers& newPointers);
		void Draw();
		void GetBoundingBox(Vector3D& minimum,Vector3D& maximum);
		string ToString();
		Vector3D GetColor();
		void SetColor(const Vector3D& c);
	private:
		GLObjectPointers glObjectPointers;
		Vector3D color;
};


#endif //_GL_COMPLEX_OBJECT_H_


