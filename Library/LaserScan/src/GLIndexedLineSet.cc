
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
*   Purpose   : GLObject for Indexed faceset.
*
*--------------------------------------------------------------------*/
#include "GLIndexedLineSet.h"
#include "stdmath.h"

#define DEBUG printf

GLIndexedLineSet::GLIndexedLineSet()
{
	color = Vector3D(1,1,1);
	id = -1;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
	
}

GLIndexedLineSet::GLIndexedLineSet(Vector3D newColor, int newId)
{
	color = newColor;
	id = newId;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
}


GLIndexedLineSet::GLIndexedLineSet(const GLIndexedLineSet& newLineSet)
{
	if(this==&newLineSet)
		return;
	color = newLineSet.color;
	id = newLineSet.id;
	coordinateVector = newLineSet.coordinateVector;
	indicesVector = newLineSet.indicesVector;
	colorsVector = newLineSet.colorsVector;
	selectionCallback = newLineSet.selectionCallback;
	selectionCallbackData = newLineSet.selectionCallbackData;
}
	
GLIndexedLineSet::~GLIndexedLineSet()
{

}


CoordinateVector&
GLIndexedLineSet::GetCoordinateVector()
{
	return coordinateVector;
}
ColorsVector& 
GLIndexedLineSet::GetColorsVector()
{
	return colorsVector;
}

void
GLIndexedLineSet::SetVectors(CoordinateVector& cv, IndicesVector& iv)
{
	//TO DO: Check input vectors for validity.
	coordinateVector = cv;
	indicesVector = iv;
}

IndicesVector& 
GLIndexedLineSet::GetIndicesVector()
{
	return indicesVector;
}

void 
GLIndexedLineSet::Draw()
{
	if(indicesVector.empty())
		return;
	
	const int lineThickness = 2;
	
	glColor3f(color.X(),color.Y(),color.Z());
	glLineWidth(lineThickness);
	
	int glLighting;
	glGetIntegerv(GL_LIGHTING,&glLighting);
	
	glDisable(GL_LIGHTING);
	for(int i=0;i<indicesVector.size();i++)
	{
		glBegin(GL_LINE_STRIP);
		while(indicesVector[i]>=0)
		{
			Vector3D v = coordinateVector[indicesVector[i]];
			glVertex3f(v.X(),v.Y(),v.Z());
			i++;
		}
		glEnd();
	}
	glEnable(GL_LIGHTING);
}
		

void
GLIndexedLineSet:: GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
{
	if(coordinateVector.empty())
	{
		minimum = Vector3D(-10,-10,-10);
		maximum = Vector3D(10,10,10);
		return;
	}
	minimum = coordinateVector[0];
	maximum = coordinateVector[0];
	
	for(int i=0;i<coordinateVector.size();i++)
	{
		minimum.X() = MIN(minimum.X(),coordinateVector[i].X());
		minimum.Y() = MIN(minimum.Y(),coordinateVector[i].Y());
		minimum.Z() = MIN(minimum.Z(),coordinateVector[i].Z());
		
		maximum.X() = MAX(maximum.X(),coordinateVector[i].X());
		maximum.Y() = MAX(maximum.Y(),coordinateVector[i].Y());
		maximum.Z() = MAX(maximum.Z(),coordinateVector[i].Z());
	}
	
	//cerr<< "GLIndex bounding box: ("<<minimum.X() <<" "<<minimum.Y()<<" "<<minimum.Z()<<" "<<maximum.X()<<" "<<maximum.Y()<<" "<<maximum.Z()<<endl; 
}

string
GLIndexedLineSet:: ToString()
{
	char buff[1024];
	sprintf(buff,"GLIndexedLineSet with id: %d and  coordinateVector size: %d",id,coordinateVector.size());
	return string(buff);
}

Vector3D
GLIndexedLineSet:: GetColor()
{
	return color;
}

void
GLIndexedLineSet:: SetColor(const Vector3D& c)
{
	color = c;
	colorsVector = vector<Vector3D>(coordinateVector.size(),c);
}


GLObject* 
GLIndexedLineSet::MakeCopy()
{
	return ((GLObject*)(new GLIndexedLineSet(*this)));
}

GLIndexedLineSet& 
GLIndexedLineSet::operator += (GLIndexedLineSet ls)
{
	CoordinateVector &cv = ls.GetCoordinateVector();
	IndicesVector &iv = ls.GetIndicesVector();
	
	coordinateVector.reserve(coordinateVector.size()+cv.size());	
	indicesVector.reserve(indicesVector.size()+iv.size());
	
	for(int i=0;i<iv.size();i++)
	{
		if(iv[i]>=0)
		{
			indicesVector.push_back(iv[i]+coordinateVector.size());
		}
		else
			indicesVector.push_back(iv[i]);
	}
	
	for(int i=0;i<cv.size();i++)
		coordinateVector.push_back(cv[i]);
		
	ColorsVector& clv = ls.GetColorsVector();
	for(int i=0;i<clv.size();i++)
		colorsVector.push_back(clv[i]);
			
	return *this;
}

GLIndexedLineSet& 
GLIndexedLineSet::append(GLIndexedLineSet& ls)
{
	CoordinateVector &cv = ls.GetCoordinateVector();
	IndicesVector &iv = ls.GetIndicesVector();
	
	coordinateVector.reserve(coordinateVector.size()+cv.size());	
	indicesVector.reserve(indicesVector.size()+iv.size());
	
	for(int i=0;i<iv.size();i++)
	{
		if(iv[i]>=0)
		{
			indicesVector.push_back(iv[i]+coordinateVector.size()-1);
		}
		else
			indicesVector.push_back(iv[i]);
	}
		
	for(int i=0;i<cv.size();i++)
		coordinateVector.push_back(cv[i]);
	
	ColorsVector& clv = ls.GetColorsVector();
	for(int i=0;i<clv.size();i++)
		colorsVector.push_back(clv[i]);
	
	return *this;
}

		
