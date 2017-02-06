
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
*   Purpose   : GLObject for simple pointset.
*
*--------------------------------------------------------------------*/
#ifndef _GL_POINT_SET_H_
#define _GL_POINT_SET_H_

#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "GLObject.h"
#include "MyTypeDefinitions.h"


class GLPointSet:public GLObject
{
public:
	GLPointSet();
	GLPointSet(Vector3D c,int id);
	GLPointSet(const GLPointSet& newPointSet);
	~GLPointSet();
	GLObject* MakeCopy();
	GLObjectTypeId GetObjectType(){return GLPointSetType;}; 

	CoordinateVector& GetCoordinateVector();
	void SetCoordinateVector(const CoordinateVector& v);
	
	void SetVectors(const CoordinateVector& cv, const IndicesVectorVector& ivv);
	IndicesVectorVector& GetIndicesVectorVector();
	void Draw();
	void GetBoundingBox(Vector3D& minimum,Vector3D& maximum);
	string ToString();
	Vector3D GetColor();
	void SetColor(const Vector3D& c);
	
	//Get and set colors vector.
	ColorsVector& GetColorsVector();
	void SetColorsVector(const ColorsVector&cv);
	
	//Assumes that transformations has been done
	//only allocation of selection buffer, drawing and parsing of selection stack is done. 
	//Make sure this is called for one object at a time. 
	//Sub selection for multiple objects may not make sense.
	void SubSelect();

	//Draw selected and sub-selected view of the data. Assumes that its called only when selection
	//state is on.
	void DrawSelection();


private:
	CoordinateVector coordinateVector;
	IndicesVectorVector indicesVectorVector;
	ColorsVector colorsVector;
	Vector3D color;
	IndicesSet subSelectedSet;
};


#endif //_GL_POINT_SET_H_


