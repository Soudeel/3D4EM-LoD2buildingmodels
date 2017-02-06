
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
*   File made : July 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Provides various utility functions to work with OpenGL.
*
*--------------------------------------------------------------------*/
#include <vector>
#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "gl2ps.h"
#include "DebugFunc.h"
#include "GeneralUtility.h"
#include "OpenGLUtility.h"

using std::vector;

//Draws a solid/wire cube to show the bounding box using opengl.
// minimum and maximum give the bounds of the box.
// if solid is true solid-box is drawn else wire box.
// min gives the minimum dimension, its to avoid the case of zero dimesion boxes which don't make sense.
void
DrawBoundingBox(Vector3D minimum, Vector3D maximum, bool solid ,double min)
{
	Vector3D range, middle;
	//Process bounds to get cube extents, scale etc.
	range = maximum - minimum;
	middle.X() = 0.5*range.X()+minimum.X();
	middle.Y() = 0.5*range.Y()+minimum.Y();
	middle.Z() = 0.5*range.Z()+minimum.Z();
	
	range.X() = (fabs(range.X())>min)?range.X():min;
	range.Y() = (fabs(range.Y())>min)?range.Y():min;
	range.Z() = (fabs(range.Z())>min)?range.Z():min;
	
	//Draw cube.
	glEnable(GL_DEPTH_TEST);
	glLineWidth(3);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef (middle.X(), middle.Y(), middle.Z());
		glScalef(range.X(),range.Y(),range.Z());
		if(!solid)
		{
			//For wires lights can be troublesome.
			glDisable(GL_LIGHTING);
			glutWireCube(1);
			glEnable(GL_LIGHTING);
		}
		else
		{
			glEnable(GL_LIGHTING);
			glutSolidCube(1);
		}
	glPopMatrix();
}

/** Draws cylinder between given bounds.

	@param point1 start point
	@param point2 end point
	@param radius radius of cylinder
	@param drawEndCaps if true draw planes on both sides.
	
	@return void
*/
void
DrawCylinder(Vector3D point1, Vector3D point2, double radius,bool drawEndCaps)
{
	const int stackCount = 15;
	double angle;
	//Direction of cylinder axis.(required)
	Vector3D direction = point2 - point1;
	
	if(direction.Length())
		direction = direction.Normalize();
	
	//Direction of cylinder given by gluCylinder. (z-axis)
	Vector3D zAxis(0,0,1);
	
	//Rotation axis is normal to the plane defined by zAxis and direction.
	Vector3D rotAxis = zAxis.VectorProduct(direction);
	
	if(rotAxis.Length()==0)
	{
		//printf("zero length shifting to z\n");
		rotAxis = zAxis;
		angle = 0;
	}
	else
	{
	
		//Find the angle between the two vectors.
		angle = Angle(zAxis,direction);
		angle = RADIAN_TO_DEGREE*angle; 
	}
	//Find height of cylinder.
	double height = (point2-point1).Length();
	
	
	//Draw the cylinder. (The last transformations are applied first in openGL)
	GLUquadricObj* cylinder = gluNewQuadric();
	gluQuadricNormals(cylinder,GLU_SMOOTH);
	gluQuadricTexture(cylinder,GL_TRUE);
	gluQuadricDrawStyle(cylinder,GLU_FILL);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef(point1.X(),point1.Y(),point1.Z());
		glRotatef(angle,rotAxis.X(),rotAxis.Y(),rotAxis.Z());
		gluCylinder(cylinder,radius,radius,height,stackCount,stackCount);
	gluDeleteQuadric(cylinder);
	
	if(drawEndCaps)
	{
		//Draw bottom.
		GLUquadricObj* bottom = gluNewQuadric();
		gluQuadricNormals(bottom,GLU_SMOOTH);
		gluQuadricTexture(bottom,GL_TRUE);
		gluQuadricDrawStyle(bottom,GLU_FILL);
		gluDisk(bottom,0.0, radius,stackCount,1);
		gluDeleteQuadric(bottom);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(point2.X(),point2.Y(),point2.Z());
			glRotatef(angle,rotAxis.X(),rotAxis.Y(),rotAxis.Z());
			GLUquadricObj* top = gluNewQuadric();
			gluQuadricNormals(top,GLU_SMOOTH);
			gluQuadricTexture(top,GL_TRUE);
			gluQuadricDrawStyle(top,GLU_FILL);
			gluDisk(top,0.0, radius,stackCount,1);
			gluDeleteQuadric(top);
		glPopMatrix();
	}
	else
	{
		glPopMatrix();
	}
}



//Read the depth buffer and return the float image.
//Its the responsibility of calling function to free the returned pointer.
//Use free(pData) and DONOT use delete[].
float *
ReadDepthBuffer (int width, int height, bool bScaleBuffer ,double scaleFactor )
{
	float *image;
	int i;
	float min, max, factor;

	/* Allocate our buffer for the image */
	if ((image = (float *) calloc (width * height, sizeof (float))) == NULL)
	{
		fprintf (stderr,
				 "ReadDepthBuffer - Failed to allocate memory for image\n");
		return (FALSE);
	}

	glPixelStorei (GL_PACK_ALIGNMENT, 1);

	/* Copy the image into our buffer */
	glReadBuffer (GL_BACK);
	glPixelTransferf (GL_DEPTH_BIAS, 0.0);
	glPixelTransferf (GL_DEPTH_SCALE, 1.000);
	glReadPixels (0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, image);

	if (bScaleBuffer)
	{
		//find max and min and scale the data so that it fits byte range.
		max = min = image[0];
		for (i = 0; i < width * height; i++)
		{
			if (image[i] < min)
				min = image[i];

			if (image[i] > max)
				max = image[i];
		}

		//OpenGL perceives depth in the reverse direction, so subtract every thing from max.
		for (i = 0; i < width * height; i++)
		{
			image[i] = max - image[i];
		}

		//Find the new max and min. Don't consider zero as a valid range value, as it represents the background.
		//find max and min and scale the data so that it fits byte range.
		max = -1e20;
		min = 1e20;
		for (i = 0; i < width * height; i++)
		{
			if (image[i] > 0)
			{
				if (image[i] < min)
					min = image[i];

				if (image[i] > max)
					max = image[i];
			}
		}


		float offsetFactor = 0.0;
		factor = scaleFactor/ (max - min);
		float offset = offsetFactor*factor;
		factor = (1.00-offsetFactor)*factor;

		printf ("max: %f min: %f \n", max, min);

		for (i = 0; i < width * height; i++)
		{
			if (image[i] > 0)
			{
				image[i] = (image[i] - min) * factor+offset;
			}
			else
			{
				image[i] = 0;
			}
		}
	}

	//return the memory.
	return image;

}

//Reads and converts the depth buffer to laser points.
LaserPoints
DepthBufferToLaserPoints (int width, int height)
{
	int i, j;
	LaserPoints points;
	
	//Read the depth buffer.
	float *image = ReadDepthBuffer (width, height);
	
	if (image)
	{
		points.reserve (width * height);
		for (j = 0; j < height; j++)
		{
			int nOffset = j * width;
			for (i = 0; i < width; i++)
			{
				//Skip points with zero height.
				if (image[nOffset] != 0.0)
				{
					points.push_back (LaserPoint(i, j, image[nOffset], (int)image[nOffset]));
				}
				nOffset++;
			}
		}

		//free image memory.
		free (image);
	}
	
	//Return the laser points
	return points;
}

//Reads color buffer and return the rgb data.
//pointer must be freed after use, using free and not delete[]
BYTE *
ReadColorBuffer (int width, int height)
{
	unsigned char *image;

	// Allocate our buffer for the image 
	if ((image =
		 (unsigned char *) calloc (3 * width * height,
								   sizeof (char))) == NULL)
	{
		fprintf (stderr,
				 "ReadColorBuffer - Failed to allocate memory for image\n");
		return (FALSE);
	}

	glPixelStorei (GL_PACK_ALIGNMENT, 1);

	// Copy the image into our buffer 
	glReadBuffer (GL_BACK_LEFT);
	glReadPixels (0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

	//return with allocated pointer.
	return image;
}

/** Draws a colorbar for given color scheme. Assumes that viewport has been properly set up.

	@param glColorScheme  enum value of desired color scheme.
	
	@return void
*/
void
DrawColorbar (GLColorScheme glColorScheme)
{

	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);

	GLfloat stepSize = 1.00 / 256.00;
	
	//Draw quads for color bar.
	glBegin (GL_QUADS);
	for (int i = 0; i < 255; i++)
	{
		glColor3ub (paletteData[i][0], paletteData[i][1], paletteData[i][2]);
		glVertex2f (0, i * stepSize);
		glVertex2f (1, i * stepSize);

		glColor3ub (paletteData[i + 1][0], paletteData[i + 1][1],
					paletteData[i + 1][2]);
		glVertex2f (1, (i + 1) * stepSize);
		glVertex2f (0, (i + 1) * stepSize);
	}
	glEnd ();

}

/** Writes eps file from FL_OBJECT of type GL_CANVAS.
	A typical call would be like this:
	
	int opt = GL2PS_SIMPLE_LINE_OFFSET | GL2PS_SILENT 
						|GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT;//| GL2PS_DRAW_BACKGROUND;
    WriteEPS(canvas,GL2PS_EPS, GL2PS_BSP_SORT, opt, 0, (char*)fileName);
    
    Use gl2ps publicly available library for rendering from OpenGL feedback buffer.
*/
			
int 
WriteEPS(int width, int height, int format, 
		int sort, 
		int options, 
		int nbcol, 
		char *file)
{
	FILE *fp;
	int state = GL2PS_OVERFLOW, buffsize = 0;
	GLint viewport[4];

	//Activate the canvas.
	//fl_activate_glcanvas(canvas);
	
	//Get geometry
	viewport[0] = 0;
	viewport[1] = 0;
	viewport[2] = width;
	viewport[3] = height;
	
	glGetIntegerv(GL_VIEWPORT, viewport);

	fp = fopen(file, "w");

	if(!fp)
	{
		fprintf(stderr,"Unable to open file %s for writing\n", file);
		return 1;
	}

	fprintf(stderr,"Saving image to file %s... ", file);
	fflush(stderr);

	while(state == GL2PS_OVERFLOW)
	{
		int oneMB = 1024*1024;
		buffsize = (10*oneMB);
		gl2psBeginPage(file, "test", viewport, format, sort, options,
			GL_RGBA, 0, NULL, nbcol, nbcol, nbcol, 
			buffsize, fp, file);
		//fl_redraw_object(canvas);
		state = gl2psEndPage();
	}

	fclose(fp);
	fprintf(stderr,"Done!\n");
	
	//return with function status.
	return 0;
}



/** Returns viewer position assuming that model-view matrix is all we have manipulated

*/
Vector3D GetGLEyePosition()
{
	double m[16];
	
	glGetDoublev(GL_MODELVIEW_MATRIX,m);
	
	//The matrices in OpenGL are column major, 
	//Translation is given by a3,a7,a11
	
	return Vector3D(m[3],m[7],m[11]);
}

/** Returns target Direction assuming that model-view matrix is all we have manipulated

*/
Vector3D GetGLTargetDirection()
{
	double m[16];
	
	glGetDoublev(GL_MODELVIEW_MATRIX,m);
	
	//The matrices in OpenGL are column major, 
	//Z-axis is given by a2,a6,a10
	//We want -Z axis
	
	return Vector3D(-m[2],-m[6],-m[10]);
}

/** Print current ModelView matrix

*/
void PrintModelViewMatrix()
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(100,100,100);
	glRotatef(75,0,0,1);
	double t[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,t);
	cerr<<"Model view matrix\n";
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			cerr<<t[i*4+j]<<"\t\t";
		}
		cerr<<endl;
	}
	cerr<<endl;
	glPopMatrix();
}



void DrawAxisIndicator(Vector3D mid , Vector3D length, double lineThickness)
{	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef(mid.X(),mid.Y(),mid.Z());
		
		glEnable(GL_LIGHTING);
		//glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		
		glColor3f(1.0f,1.0f,1.0f);
		glutSolidCube(0.2*MIN(length.X(),MIN(length.Y(),length.Z())));
		
		glDisable(GL_LIGHTING);
		glLineWidth(lineThickness);

		glBegin(GL_LINES);
		//x-axis	
		glColor3f(1.0,0,0);
		glVertex3f(0,0,0);
		glVertex3f(length.X(),0,0);

		//y-axis	
		glColor3f(0,1.0,0);
		glVertex3f(0,0,0);
		glVertex3f(0,length.Y(),0);

		//z-axis	
		glColor3f(0,0,1.0);
		glVertex3f(0,0,0);
		glVertex3f(0,0,length.Z());

		glEnd();
	
	glPopMatrix();
}

void
AnalyzeFeedbackAndDepthBuffer (GLint size,
							   GLfloat * feedbackBuffer,
							   float *depthBuffer,
							   int width,
							   int height,
							   vector < int >&selectedIndices,
							   int x1, int y1, int x2, int y2,double threshold)
{
	GLint count;
	GLfloat token;
	GLuint pointsCount = 0;
	int currentIndex = -1;
	float point[3];
	int i;
	float min,max;
	
	min=1e7;
	max=-1e7;
	for(i=0;i<width*height;i++)
	{
		if(depthBuffer[i]!=1)
		{
			max = MAX(depthBuffer[i],max);
			min = MIN(depthBuffer[i],min);
		}
	}
	//threshold = 0.05*(max-min);
	count = size;
	selectedIndices.clear();
	selectedIndices.reserve(width*height);
	while (count)
	{
		token = feedbackBuffer[size - count];
		count--;
		if (token == GL_PASS_THROUGH_TOKEN)
		{
			//printf ("GL_PASS_THROUGH_TOKEN\n");
			//printf ("  %4.2f\n", feedbackBuffer[size-count]);
			currentIndex = (int)feedbackBuffer[size - count];
			count--;

		}
		else if (token == GL_POINT_TOKEN)
		{
			//printf ("GL_POINT_TOKEN\n");
			for (i = 0; i < 7; i++)
			{
				//printf ("%4.2f ", feedbackBuffer[size-(count)]);
				if (i < 3)
				{
					point[i] = feedbackBuffer[size - (count)];
				}
				count = count - 1;
			}
			float depth =
				depthBuffer[(int)((int)(point[1]+0.5) * width) + (int) point[0]];
			float diff = fabs ((double)(depth - point[2]));
			
			if(threshold < 1)
			{
				if (diff > depth*threshold)
				{
				}
				else
				{
					if (point[0] > x1 && point[0] < x2 && point[1] > y1
						&& point[1] < y2)
					{
						selectedIndices.push_back (currentIndex);
					}
				}
			}
			else if(depth !=1)
			{
				selectedIndices.push_back (currentIndex);
			}
				
		}
		/*
		else if (token == GL_LINE_TOKEN)
		{
			printf ("GL_LINE_TOKEN\n");
			print3DcolorVertex (size, &count, feedbackBuffer);
			print3DcolorVertex (size, &count, feedbackBuffer);
		}
		else if (token == GL_LINE_RESET_TOKEN)
		{
			printf ("GL_LINE_RESET_TOKEN\n");
			print3DcolorVertex (size, &count, feedbackBuffer);
			print3DcolorVertex (size, &count, feedbackBuffer);
		}
		*/
	}
#if 0	
	DEBUG ("\nPointsCount in selection feedbackBuffer is %d\n", pointsCount);
	DEBUG("Threshold %f\n",threshold);
#endif	
}

typedef vector<vector<int> > SegmentsVector;
void
GetSegmentedLaserIndices (GLint size,
						GLfloat * feedbackBuffer,
						float *depthBuffer,
						unsigned int* segmentedBuffer,
						int width,
						int height,
						SegmentsVector& segmentedVector,
						int x1, int y1, int x2, int y2)
{
	GLint count;
	GLfloat token;
	GLuint pointsCount = 0;
	int currentIndex = -1;
	float point[3];
	int i;
	float min,max,threshold;
	unsigned int labelsCount=0;
	int currentLabel = 0;
	int xIndex = 0;
	int yIndex = 0;
	
	min=1e7;
	max=-1e7;
	for(i=0;i<width*height;i++)
	{
		if(depthBuffer[i]!=1)
		{
			max = MAX(depthBuffer[i],max);
			min = MIN(depthBuffer[i],min);
		}
	}
	threshold = 0.05*(max-min);
	count = size;
	
	//Get the maximum value of label
	labelsCount = 0;
	for(i=0;i<width*height;i++)
	{
		labelsCount = MAX(segmentedBuffer[i],labelsCount);
	}
	//Labels count is one more than the maximum value.
	labelsCount++;

	//Reserve space.
	segmentedVector.clear();
	segmentedVector.resize(labelsCount);
	
	for(i=0;i<labelsCount;i++)
	{
		segmentedVector.reserve(width*height/labelsCount + 1);
	}

	while (count)
	{
		token = feedbackBuffer[size - count];
		count--;
		if (token == GL_PASS_THROUGH_TOKEN)
		{
			//printf ("GL_PASS_THROUGH_TOKEN\n");
			//printf ("  %4.2f\n", feedbackBuffer[size-count]);
			currentIndex = (int)feedbackBuffer[size - count];
			count--;

		}
		else if (token == GL_POINT_TOKEN)
		{
			//printf ("GL_POINT_TOKEN\n");
			for (i = 0; i < 7; i++)
			{
				//printf ("%4.2f ", feedbackBuffer[size-(count)]);
				if (i < 3)
				{
					point[i] = feedbackBuffer[size - (count)];
				}
				count = count - 1;
			}
			
			xIndex = (int)point[0];
			yIndex = (int)(point[1]+0.5);
			float depth =
				depthBuffer[(int)(yIndex * width) + xIndex];
			float diff = fabs ((double)(depth - point[2]));
			if (diff > threshold)
			{
				//Skip this point.
				//printf("*** x: %f y: %f z: %f depth: %f diff: %f\n",point[0],point[1],point[2],depth,diff); 
			}
			else
			{
				if (xIndex > x1 && xIndex < x2 && yIndex > y1
					&& yIndex < y2)
				{
					
					currentLabel = segmentedBuffer[(int)(yIndex * width) + xIndex]; 
					//printf("adding label %d\t ",currentLabel);
					segmentedVector[currentLabel].push_back (currentIndex);
				}
				else
				{
						//printf("skipping\n");
				}
			}
		}
	}
	DEBUG("\nPointsCount in selection feedbackBuffer is %d\n", pointsCount);
	DEBUG("Threshold %f labelsCount %d  vecsize %d\n",threshold,labelsCount,segmentedVector.size());
}


Rotation3D  
GetGLRotation()
{
	
	//Read the current matrix.
	glMatrixMode(GL_MODELVIEW);

	double pMatrix[16];
	glGetDoublev (GL_MODELVIEW_MATRIX, pMatrix);
	
	return Rotation3D (
				pMatrix[0], pMatrix[4],pMatrix[8],
				pMatrix[1], pMatrix[5], pMatrix[9],
				pMatrix[2], pMatrix[6], pMatrix[10] );
	
// 	return Rotation3D (
// 				pMatrix[0], pMatrix[1],pMatrix[2],
// 				pMatrix[4], pMatrix[5], pMatrix[6],
// 				pMatrix[8], pMatrix[9], pMatrix[10] );
}

void 
SetGLRotation(const Rotation3D& rot)
{
	
	//Multiply the current matrix.
	glMatrixMode(GL_MODELVIEW);

	double pMatrix[16];
	pMatrix[0] = rot.R(0,0);
	pMatrix[4] = rot.R(0,1);
	pMatrix[8] = rot.R(0,2);
	
	pMatrix[1] = rot.R(1,0);
	pMatrix[5] = rot.R(1,1);
	pMatrix[9] = rot.R(1,2);
	
	pMatrix[2] = rot.R(2,0);
	pMatrix[6] = rot.R(2,1);
	pMatrix[10] = rot.R(2,2);
	
	pMatrix[3] = pMatrix[7] = pMatrix[11] = 0;
	pMatrix[12] = pMatrix[13] = pMatrix[14] = 0;
	pMatrix[15] = 1;
	
	glMultMatrixd (pMatrix);
	
}


void 
SetGLTranslation(const Vector3D& t)
{
	glMatrixMode(GL_MODELVIEW);
	glTranslated(t[0],t[1],t[2]);	
}


Vector3D  
GetGLTranslation ()
{
	//Read the current matrix.
	double pMatrix[16];
	glGetDoublev (GL_MODELVIEW_MATRIX, pMatrix);
	Vector3D p(pMatrix[12],pMatrix[13],pMatrix[14]);
	return p;
}
