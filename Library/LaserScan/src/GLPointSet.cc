
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
#include "GLPointSet.h"
#include "math.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"

GLPointSet::GLPointSet()
{
	color = Vector3D(1,1,1);
	id = -1;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
	
}

GLPointSet::GLPointSet(Vector3D newColor, int newId)
{
	color = newColor;
	id = newId;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
}


GLPointSet::GLPointSet(const GLPointSet& newPointSet)
{
	if(this==&newPointSet)
		return;
	color = newPointSet.color;
	id = newPointSet.id;
	coordinateVector = newPointSet.coordinateVector;
	indicesVectorVector = newPointSet.indicesVectorVector;
	colorsVector = newPointSet.colorsVector;
	selectionCallback = newPointSet.selectionCallback;
	selectionCallbackData = newPointSet.selectionCallbackData;
	subSelectedSet = newPointSet.subSelectedSet;
}
	
GLPointSet::~GLPointSet()
{

}


CoordinateVector&
GLPointSet::GetCoordinateVector()
{
	return coordinateVector;
}

void 
GLPointSet::SetCoordinateVector(const CoordinateVector& v )
{
	coordinateVector = v;
}

void
GLPointSet::SetVectors(const CoordinateVector& cv, const IndicesVectorVector& ivv)
{
	//TO DO: Check input vectors for validity.
	coordinateVector = cv;
	indicesVectorVector = ivv;
	subSelectedSet.clear(); 
}

IndicesVectorVector& 
GLPointSet::GetIndicesVectorVector()
{
	return indicesVectorVector;
}

void 
GLPointSet::Draw()
{
	if(indicesVectorVector.empty())
		return;
	
	//Turn off lighting.
	glDisable(GL_LIGHTING);
	
	glColor3f(color.X(),color.Y(),color.Z());
	
	//Push pointSet type name.
	glPushName(SELECTION_GL_POINTSET);
	
	//Push one name for index, afterwards just load.
	glPushName(-1);
	
	for(int i=0;i<indicesVectorVector.size();i++)
	{
		//Load current index.
		glLoadName(i);
		
		//Draw current index.
		glBegin(GL_POINTS);
		const IndicesVector& iv = indicesVectorVector[i];
		glPointSize(3);
		for(int j=0;j<iv.size();j++)
		{
			int index = iv[j];
			Vector3D v = coordinateVector[index];
			Vector3D c;
			if(index<colorsVector.size())
				c = colorsVector[index];
			else
				c = color;
				
			glColor3f(c.X(),c.Y(),c.Z());
			glVertex3f(v.X(),v.Y(),v.Z());
		}
		glEnd();
	}
	//Pop indices name.
	glPopName();
	//Pop pointSet type name.
	glPopName();
	
	//Turn on lighting again.
	glEnable(GL_LIGHTING);
}
		

void
GLPointSet:: GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
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
	
	//cerr<< "GLPointSet bounding box: ("<<minimum.X() <<" "<<minimum.Y()<<" "<<minimum.Z()<<" "<<maximum.X()<<" "<<maximum.Y()<<" "<<maximum.Z()<<endl; 
}

string
GLPointSet:: ToString()
{
	char buff[1024];
	sprintf(buff,"GLPointSet with id: %d and  coordinateVector size: %d",id,coordinateVector.size());
	return string(buff);
}

Vector3D
GLPointSet:: GetColor()
{
	return color;
}

void
GLPointSet:: SetColor(const Vector3D& c)
{
	color = c;
}


GLObject* 
GLPointSet::MakeCopy()
{
	return ((GLObject*)(new GLPointSet(*this)));
}

//Assumes that transformations has been done
//only allocation of selection buffer, drawing and parsing of selection stack is done. 
//Make sure this is called for one object at a time. 
//Sub selection for multiple objects may not make sense.
void
GLPointSet:: SubSelect()
{
	DEBUG("GLPointSet:: SubSelect()");
	
	// Initializing buffer in which selected points are stored
	int selectionBufferSize = 1024*1024;

	//Return if zero elements.
	if (!selectionBufferSize)
		return ;

	GLuint *selectionBuffer = new GLuint[selectionBufferSize];
	glSelectBuffer (selectionBufferSize, selectionBuffer);

	GLuint i, hits, picked;
	// Set the rendermode to the "select" mode
	glRenderMode (GL_SELECT);
	glEnable (GL_DEPTH_TEST);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glInitNames ();

	//Just draw all sub-parts of the object.
	this->Draw();	
	
	glFlush ();
	
	//Clear sub-slected set.
	subSelectedSet.clear();
	
	hits = glRenderMode (GL_RENDER);
	if (hits <= 0)
	{
		DEBUG("GLPointSet::SubSelect()    Nothing selected");
	}
	else
	{
		DEBUG ("GLPointSet::SubSelect()   Selection hits = %d", hits);
			
		GLuint *ptr = selectionBuffer;
        GLuint  names;
        GLuint  z_min;
        GLuint  z_max;
        GLuint  hit_name;
        GLuint  nearest = 0xffffffff;
		GLuint selected = 0;
        for( int i=0; i<hits; i++ )
		{
			DEBUG("\n**************************************\nRecord %d ",i);
			names = *ptr++;
			z_min = *ptr++;
			z_max = *ptr++;
			
			if( z_min<nearest )
			{
				nearest = z_min; 
			}     
			for( int j=0; j<names; j++ )
			{
				hit_name = *ptr++;
				
				if(hit_name==SELECTION_GL_POINTSET)
				{
					unsigned int selectedIndex = (ptr[0]);
					subSelectedSet.insert(selectedIndex);
			
					DEBUG("Subselecting PointSet index : %d",selectedIndex);
					DEBUG("z_min: %f ",z_min*1e-10);
					DEBUG("z_max: %f ",z_max*1e-10);
					//Increment both j and ptr as we have read the value of pointer already.			
					ptr++;j++; 
				}
				else
				{
					cerr<<"GLPointSet::SubSelect()  Unknown type: "<<*ptr<<endl;
				}
			} 
		}
	}
	DEBUG("GLPointSetSubSelection size: %d",subSelectedSet.size());

	delete[]selectionBuffer;


}
	
//Draw selected and sub-selected view of the data. Assumes that its called only when selection
//state is on.
void 
GLPointSet::DrawSelection()
{
	DEBUG("GLPointSet::DrawSelection()");
	
	//Draw bounding box to show the pointSet is selected.
	Vector3D maximum,minimum;
	GetBoundingBox(minimum,maximum);
	
	glColor3f(1.0,1.0,1.0);
	DrawBoundingBox(minimum, maximum,false);
	
	//Draw sub-seleced Indices in a big point size and with different color.
	const int pointSize = 2*2;
	const float pointColor[3] = {1,0,0};
	
	//Turn off lighting.
	glDisable(GL_LIGHTING);
	
	glColor3f(pointColor[0],pointColor[1],pointColor[2]);
	glPointSize(pointSize);
	
	IndicesSet::iterator iter = subSelectedSet.begin();
	
	for(;iter!=subSelectedSet.end();iter++)
	{
		int currentIndex = *iter;
		if(currentIndex>=0 && currentIndex<indicesVectorVector.size())
		{
			const IndicesVector& iv = indicesVectorVector[currentIndex];		
			
			//Draw current selection.
			glBegin(GL_POINTS);
			for(int j=0;j<iv.size();j++)
			{
				int index = iv[j];
				Vector3D v = coordinateVector[index];
				glVertex3f(v.X(),v.Y(),v.Z());
			}
			glEnd();
		}
		else
		{
			cerr<<"GLPointSet::DrawSelection invalid index in selection set\n";
		}
	}
	
	//Turn on lighting again.
	glEnable(GL_LIGHTING);

}

//Get and set colors vector.
ColorsVector& 
GLPointSet::GetColorsVector()
{
	return colorsVector;
}

void GLPointSet::SetColorsVector(const ColorsVector&cv)
{
	colorsVector = cv;
}
