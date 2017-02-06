
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
#ifndef _OPENGL_UTILITY_H_
#define _OPENGL_UTILITY_H_
#include "OpenGLHeaders.h"
#include <GL/glut.h>
#include "gl2ps.h"
#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "Vector3D.h"
#include "Rotation3D.h"
#include "LaserPoints.h"


/** Draws a solid/wire cube to show the bounding box using opengl.

    @param minimum the lowest point of the box
    @param maximum the highest point of th box.
    @param solid solid box if true, wire box if false
    @param min minimum dimension of the box (to avoid boxes with zero dimension)
    @return void
*/
void
DrawBoundingBox(Vector3D minimum = Vector3D(-5,-5,-5), 
				Vector3D maximum = Vector3D(+5,+5,+5), 
				bool solid = true ,
				double min = 1e-1);
				
/** Draws cylinder between given bounds.

	@param point1 start point
	@param point2 end point
	@param radius radius of cylinder
	@param drawEndCaps if true draw planes on both sides.
	
	@return void
*/
void
DrawCylinder(Vector3D point1 = Vector3D(0,0,0), 
			Vector3D point2 = Vector3D(10,0,0), 
			double radius=1,
			bool drawEndCaps=true);
				

/** Reads the depth buffer of current context and returns as float*.
	The memory returned is allocated by malloc and MUST be freed by free
	DONOT use delte or delte[] to free the returned memory

    @param width width of the OpenGL context
    @param height height of the OpenGL context
    @param bScaleBuffer whether to scale read values or not
    @param scaleFactor scale Factor to apply in case above flag is true
    @return float* memory containing the read values.
*/
								
float*
ReadDepthBuffer (int width, 
				int height, 
				bool bScaleBuffer = true,
				double scaleFactor = 255);
/** Reads the depth buffer of current context and returns as converts it to
	LaserPoints.

    @param width width of the OpenGL context
    @param height height of the OpenGL context
    @return LaserPoints containing the read values.
*/
LaserPoints
DepthBufferToLaserPoints (int width, int height);


/** Reads color buffer and return the rgb data.
	returned pointer must be freed after use using free and not delete[]

    @param width width of the OpenGL context
    @param height height of the OpenGL context
    @return BYTE* containing the raw image read from color buffer.
*/
BYTE *
ReadColorBuffer (int width, int height);

/** Draws a colorbar for given color scheme. Assumes that viewport has been properly set up.

	@param glColorScheme  enum value of desired color scheme.
	
	@return void
*/
void
DrawColorbar (GLColorScheme glColorScheme);

/** Writes eps file from FL_OBJECT of type GL_CANVAS.
	A typical call would be like this:
	
	int opt = GL2PS_SIMPLE_LINE_OFFSET | GL2PS_SILENT 
						|GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT;//| GL2PS_DRAW_BACKGROUND;
    WriteEPS(canvas,GL2PS_EPS, GL2PS_BSP_SORT, opt, 0, (char*)fileName);
    
    Use gl2ps publicly available library for rendering from OpenGL feedback buffer.
*/
int 
WriteEPS(int width, int height,
		int format, 
		int sort, 
		int options, 
		int nbcol, 
		char *file);
		

//Draw axis indicator at the given position, with specified sizes and line thickness.
void DrawAxisIndicator(Vector3D mid = Vector3D(), Vector3D length=Vector3D(1,1,1), double lineThickness = 2);

typedef vector<vector<int> > SegmentsVector;
void
GetSegmentedLaserIndices (GLint size,
						GLfloat * feedbackBuffer,
						float *depthBuffer,
						unsigned int* segmentedBuffer,
						int width,
						int height,
						SegmentsVector& segmentedVector,
						int x1, int y1, int x2, int y2);
						
void
AnalyzeFeedbackAndDepthBuffer (GLint size,
							   GLfloat * feedbackBuffer,
							   float *depthBuffer,
							   int width,
							   int height,
							   vector < int >&selectedIndices,
							   int x1, int y1, int x2, int y2,double threshold=0.1);
			
///Convert current matrix to Rotation 3D							   				   
Rotation3D  GetGLRotation();

///Convert current translation to vector3D.
Vector3D  GetGLTranslation();


///Multiply with the given rotation.
void 
SetGLRotation(const Rotation3D& rot);

///Set vector3d as translation.
void 
SetGLTranslation(const Vector3D& t);
						

#endif //_OPENGL_UTILITY_H_


