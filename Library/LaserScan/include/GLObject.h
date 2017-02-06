
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
*   Purpose   : Defines the parent class for GLObjects. Contains common virtual functions and common data
*
*--------------------------------------------------------------------*/
#ifndef _GL_OBJECT_H_
#define _GL_OBJECT_H_

#include <vector>
#include <set>
#include "OpenGLHeaders.h"
#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "OpenGLUtility.h"

//Constants to identify different tokens on selection stack.
#define SELECTION_ACIS_BODY 					1
#define SELECTION_ACIS_FACE 					2
#define SELECTION_ACIS_EDGE  					4
#define SELECTION_ACIS_VERTEX  					8
#define IS_SELECTION_ACIS(a)					((a)==SELECTION_ACIS_BODY || (a)==SELECTION_ACIS_FACE || (a)==SELECTION_ACIS_EDGE || (a)==SELECTION_ACIS_VERTEX)
#define SELECTION_GL_OBJECT 					16
#define IS_SELECTION_GL_OBJECT(a)				((a)==SELECTION_GL_OBJECT)
#define SELECTION_GL_POINTSET					17



//using namespace std;

class GLObject;
typedef void (*GLObjectUserDefinedCallback)(GLObject*,void*);
typedef void (*UserDefinedCallback)(void*);

enum GLObjectTypeId{
				GLObjectType,
				GLIndexedFaceSetType,
				GLIndexedLineSetType,
				GLComplexObjectType,
				GLPointSetType,
				GLAcisBodyObjectType,
				GLAcisPhlObjectType,
				GLTransformType,
				GLCylinderType,
				GLPlaneType,
				GLPlaneTRType,
				GLSphereType,
				GLTorusType,
				GLCubeType};

class GLObject
{
public:
	GLObject(){selectionCallback=NULL;selectionCallbackData=NULL;id=-1;};
	virtual ~GLObject(){};
	virtual void Draw(){};
	virtual void GetBoundingBox(Vector3D& minimum,Vector3D& maximum){};
	virtual string ToString(){};
	void SetDescription(string s){description=s;};
	virtual int InvokeSelectionCallback();
	virtual GLObject* MakeCopy(){return NULL;};
	virtual GLObjectTypeId GetObjectType(){return GLObjectType;}; 
	virtual void SetSelectionCallback(GLObjectUserDefinedCallback cb,void* data){selectionCallback=cb;selectionCallbackData = data;};
	virtual	GLObjectUserDefinedCallback GetSelectionCallback(){return selectionCallback;};
	virtual void SetId(int ii){id=ii;};
	virtual int GetId(){return id;};
	//Assumes that transformations has been done
	//only allocation of selection buffer, drawing and parsing of selection stack is done. 
	//Make sure this is called for one object at a time. 
	//Sub selection for multiple objects may not make sense.
	virtual void SubSelect();
	
	//Draw selected and sub-selected view of the data. Assumes that its called only when selection
	//state is on.
	virtual void DrawSelection();
	
protected:
	string description;
	GLObjectUserDefinedCallback selectionCallback;
	void* selectionCallbackData;
	int id;
};

typedef std::vector<GLObject*> GLObjectPointers;
typedef std::vector<int> IndicesVector;
typedef std::set<int> IndicesSet;
typedef std::vector<Vector3D> CoordinateVector;
typedef std::vector<Vector3D> ColorsVector;
typedef std::vector<IndicesVector> IndicesVectorVector;

#endif //_GL_OBJECT_H_


