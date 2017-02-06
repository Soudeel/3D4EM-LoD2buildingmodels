
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
#include "GLIndexedFaceSet.h"
#include "math.h"

#define DEBUG printf

GLIndexedFaceSet::GLIndexedFaceSet()
{
	color = Vector3D(1,1,1);
	id = -1;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
	
}

GLIndexedFaceSet::GLIndexedFaceSet(Vector3D newColor, int newId)
{
	color = newColor;
	id = newId;
	selectionCallback = (GLObjectUserDefinedCallback)(0);
	selectionCallbackData = (void*)0;
}


GLIndexedFaceSet::GLIndexedFaceSet(const GLIndexedFaceSet& newFaceSet)
{
	if(this==&newFaceSet)
		return;
	color = newFaceSet.color;
	id = newFaceSet.id;
	coordinateVector = newFaceSet.coordinateVector;
	indicesVector = newFaceSet.indicesVector;
	selectionCallback = newFaceSet.selectionCallback;
	selectionCallbackData = newFaceSet.selectionCallbackData;
}
	
GLIndexedFaceSet::~GLIndexedFaceSet()
{

}


CoordinateVector&
GLIndexedFaceSet::GetCoordinateVector()
{
	return coordinateVector;
}

void
GLIndexedFaceSet::SetVectors(const CoordinateVector& cv,const IndicesVector& iv,const NormalVector& nv)
{
	//TO DO: Check input vectors for validity.
	coordinateVector = cv;
	indicesVector = iv;
	normalVector = nv;
}

IndicesVector& 
GLIndexedFaceSet::GetIndicesVector()
{
	return indicesVector;
}

void 
GLIndexedFaceSet::Draw()
{
	if(indicesVector.empty())
		return;
	
	glColor3f(color.X(),color.Y(),color.Z());
	
//	cerr<<"color is: "<<color.X()<<","<<color.Y()<<","<<color.Z()<<endl;
	
	Vector3D vertices[10];
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	
	const bool smoothNormals = true;
	
	if(normalVector.empty())
	{
		NormalVector localNormals;
		//normalVector.resize(coordinateVector.size());
		if(smoothNormals)
		{
			std::vector<std::vector<int> > neighbourVector;
			neighbourVector.resize(coordinateVector.size());
			localNormals.resize(indicesVector.size());
		
			//Calculate normals of each face and save them in normal vector.
			int count = 0;
			for(int i=0;i<indicesVector.size();i++)
			{
				//Calculate normal.
				int k=0;
				while(indicesVector[i]>=0)
				{
					vertices[k] = coordinateVector[indicesVector[i]];
					neighbourVector[indicesVector[i]].push_back(count);
					i++;k++;
				}
				Vector3D v1 = vertices[0] - vertices[1];
				Vector3D v2 = vertices[2] - vertices[1];
				v1 = v1.Normalize();
				v2 = v2.Normalize();
		
				Vector3D normal;
				normal= v2.VectorProduct(v1);
				normal = normal.Normalize();
		
				localNormals[count]=(normal);
				count++;
			}
			
			count = 0;
			for(int i=0;i<indicesVector.size();i++)
			{
				//Calculate smooth normal.
				std::vector<Vector3D> smoothNormals(256);
				Vector3D currentNormal = localNormals[count];
				int total = 0;
			
				int c=0;
				int j=i;
				int k;
				while(indicesVector[j]>=0)
				{
					int index = indicesVector[j];
					smoothNormals[c] = Vector3D(0,0,0);
							
					for(k=0;k<(neighbourVector[index].size());k++)
					{
						Vector3D newVector = localNormals[neighbourVector[index][k]];
						if((currentNormal.DotProduct(newVector))>0.9)
						{
							smoothNormals[c].X() = smoothNormals[c].X() + newVector.X();
							smoothNormals[c].Y() = smoothNormals[c].Y() + newVector.Y();
							smoothNormals[c].Z() = smoothNormals[c].Z() + newVector.Z();
							total++;
						}
					}
				
					if(total>0)
					{
						smoothNormals[c].X() = smoothNormals[c].X() * (double)(1.00/(double)total);
						smoothNormals[c].Y() = smoothNormals[c].Y() * (double)(1.00/(double)total);
						smoothNormals[c].Z() = smoothNormals[c].Z() * (double)(1.00/(double)total);
					}
					else
					{
						smoothNormals[c] = currentNormal;
					}
					j++;c++;
				}
			
				c = 0;
				glBegin(GL_POLYGON);
		
				while(indicesVector[i]>=0)
				{
					glNormal3f(smoothNormals[c].X(),smoothNormals[c].Y(),smoothNormals[c].Z());
					Vector3D v = coordinateVector[indicesVector[i]];
					glColor3f(color.X(),color.Y(),color.Z());
					glVertex3f(v.X(),v.Y(),v.Z());
					i++;c++;
				
				}
				glEnd();
				count++;
			}
		}
		else
		{
			for(int i=0;i<indicesVector.size();i++)
			{
				//Calculate normal.
				int j=i;
				int k=0;
				while(indicesVector[j]>=0)
				{
					vertices[k] = coordinateVector[indicesVector[j]];
					j++;k++;
				}
				Vector3D v1 = vertices[0] - vertices[1];
				Vector3D v2 = vertices[2] - vertices[1];
				v1 = v1.Normalize();
				v2 = v2.Normalize();
		
				Vector3D normal;
				normal= v2.VectorProduct(v1);
		
				//draw normals.
				const bool drawNormals = 0;
				if(drawNormals)
				{
					Vector3D mid;
					mid.X() = 0.33*(vertices[0].X() + vertices[1].X() + vertices[2].X());
					mid.Y() = 0.33*(vertices[0].Y() + vertices[1].Y() + vertices[2].Y());
					mid.Z() = 0.33*(vertices[0].Z() + vertices[1].Z() + vertices[2].Z());;
		
					glLineWidth(3);
					glBegin(GL_LINES);
					glColor3f(1.0,0,0);
					//glVertex3f(mid.X(),mid.Y(),mid.Z());
		

					float l = 30;
					glVertex3f(mid.X()-l*normal.X(),mid.Y()-l*normal.Y(),mid.Z()-l*normal.Z());
					glColor3f(0,1,0);
					glVertex3f(mid.X()+l*normal.X(),mid.Y()+l*normal.Y(),mid.Z()+l*normal.Z());
					glEnd();
				}
		

				glNormal3f(normal.X(),normal.Y(),normal.Z());
		
				glBegin(GL_POLYGON);
				while(indicesVector[i]>=0)
				{
					Vector3D v = coordinateVector[indicesVector[i]];
					glColor3f(color.X(),color.Y(),color.Z());
					glVertex3f(v.X(),v.Y(),v.Z());
					i++;
				}
				glEnd();
			}
		}
		
	}
	else //assume the normals are calculated.
	{
		//cerr<<"Using input normals\n";
		for(int i=0;i<indicesVector.size();i++)
		{				
			glBegin(GL_POLYGON);
			while(indicesVector[i]>=0)
			{
				Vector3D v = coordinateVector[indicesVector[i]];
				glColor3f(color.X(),color.Y(),color.Z());
				Vector3D normal = normalVector[indicesVector[i]];
				glNormal3f(normal.X(),normal.Y(),normal.Z());
				glVertex3f(v.X(),v.Y(),v.Z());
				i++;
			}
			glEnd();
		}
	}
}
		

void
GLIndexedFaceSet:: GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
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
GLIndexedFaceSet:: ToString()
{
	char buff[1024];
	sprintf(buff,"GLIndexedFaceSet with id: %d and  coordinateVector size: %d",id,coordinateVector.size());
	return string(buff);
}

Vector3D
GLIndexedFaceSet:: GetColor()
{
	return color;
}

void
GLIndexedFaceSet:: SetColor(const Vector3D& c)
{
	color = c;
}


GLObject* 
GLIndexedFaceSet::MakeCopy()
{
	return ((GLObject*)(new GLIndexedFaceSet(*this)));
}

