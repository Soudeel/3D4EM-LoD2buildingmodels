
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
*   Purpose   : GLObject for Indexed lineset.
*
*--------------------------------------------------------------------*/
#ifndef _GL_INDEXED_LINE_SET_H_
#define _GL_INDEXED_LINE_SET_H_

#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "GLObject.h"
#include "MyTypeDefinitions.h"
#include "GLObject.h"


class GLIndexedLineSet:public GLObject
{
public:
	GLIndexedLineSet();
	GLIndexedLineSet(Vector3D c,int id);
	GLIndexedLineSet(const GLIndexedLineSet& newLineSet);
	~GLIndexedLineSet();
	GLObject* MakeCopy();
	GLObjectTypeId GetObjectType(){return GLIndexedLineSetType;}; 

	CoordinateVector& GetCoordinateVector();
	void SetVectors(CoordinateVector& cv, IndicesVector& iv);
	IndicesVector& GetIndicesVector();
	ColorsVector& GetColorsVector();
	void Draw();
	void GetBoundingBox(Vector3D& minimum,Vector3D& maximum);
	string ToString();
	Vector3D GetColor();
	void SetColor(const Vector3D& c);
	GLIndexedLineSet& operator += (GLIndexedLineSet ls);
	GLIndexedLineSet& append(GLIndexedLineSet& ls);
	
private:
	CoordinateVector coordinateVector;
	IndicesVector indicesVector;
	ColorsVector colorsVector;
	Vector3D color;
};


#endif //_GL_INDEXED_LINE_SET_H_


