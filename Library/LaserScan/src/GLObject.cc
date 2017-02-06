
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
#include "GLObject.h"


int 
GLObject::InvokeSelectionCallback()
{
	if(selectionCallback)
	{
		selectionCallback(this,selectionCallbackData);
		return 0;
	}
	else
	{
		return 1;
	}
}

void 
GLObject::SubSelect()
{
	cerr<<"GLObject::SubSelect() not implemented for this object yet"<<endl;
}

	
//Draw selected and sub-selected view of the data. Assumes that its called only when selection
//state is on.
void 
GLObject::DrawSelection()
{
	//By default we will just draw a cube having size equal to bounding box.
	Vector3D minimum,maximum,middle,range;
	
	this->GetBoundingBox(minimum,maximum);
			
	range = maximum - minimum;
	middle.X() = 0.5*range.X()+minimum.X();
	middle.Y() = 0.5*range.Y()+minimum.Y();
	middle.Z() = 0.5*range.Z()+minimum.Z();

	//Turn off lights.
	glDisable(GL_LIGHTING);
	
	glLineWidth(3);
	glColor3f(1,0,0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef (middle.X(), middle.Y(), middle.Z());
		glScalef(range.X(),range.Y(),range.Z());
		glutWireCube(1);
	glPopMatrix();
	
	//Turn on lights again.
	glEnable(GL_LIGHTING);

}


