
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
*   Purpose   : Provides various utility functions to work with LaserPoints.
*
*--------------------------------------------------------------------*/
#ifndef _LASERPOINT_UTILITY_H_
#define _LASERPOINT_UTILITY_H_
#include <string>
#include "LaserPoints.h"
#include "MyConstants.h"
#include "Palettes.h"
#include "LaserObjects.h"
#include "LaserPointsProcessing.h"
#include "MyTypeDefinitions.h"
using std::string;

/** Histogram equalizes reflectance of the given laserpoints.

    @param laserPoints input laser points. The reflectance is modified on return.
    @return void (The input laser points are changed)
*/
void HistoEqualizeLaserPoints(LaserPoints& laserPoints);

/**Adds uniformly distributed multiplicative noise to x,y,z values of laser points.
	The formula is a = (a + additiveNoiseLevel*random(-1,1))*(1+multiplicativeNoise*random(-1,1))
	
	@param laserPoints  input points are modified on return.
	@param noiseLevelAdditive	level of additive noise
	@param noiseLevelMultiplicative	level of multiplicative noise
	
	@return void
*/	
void
AddNoiseToLaserPoints (LaserPoints & laserPoints, 
						double noiseLevelAdditive = 1, 
						double noiseLevelMultiplicative= 0);
						

/* Generates ideal laserPoint data for a given cylinder.

	@param radius	array of double with cylinder radius (vary for cone, keep fixed for cylinder)
	@param radiusSize	size of the above array
	@param circlePoints	number of points on the circle
	@param circleExtent	0 to 1, specifies percentage of circle (1 or higher means full circle)
	@param stacks	array of doubles specifying z-values.
	@param stackCount number of entries in stacks.
	
	@return LaserPoints containing the generated points.
*/	
	
LaserPoints
GenerateCylinderPoints (
			double *radius = NULL, 
			int radiusSize = 10, 
			int circlePoints = 10,
			double circleExtent = 1,
			double* stacks = NULL, 
			int stackSize = 10);
			
/** Make laser points for spatially varying sine wave of given frequencies.

	@param x1,x2,y1,y2 Extents of space
	@praam countX,countY number of points in X and Y
	@param frequencyX spatial frequency in X direction
	@param frequencyY spatial frequency in Y direction.
	
	@return laserpoints containing height variation of given sine freq.
*/
LaserPoints
MakeSinePoints (double x1=-5,double x2=5,
				double y1=-5,double y2=5,
				int countX=50, int countY=50,
				double frequencyX=2,double frequencyY=2);
				
/** Converts reflectance value of passed laser points to indices. Can be used for palette lookup.
	
	@param laserPoints  	input laser points whose reflectance is to be mapped.
	@param useReflectance	use reflectance if true else use z-value.
	@param minIndex			minimum index of palette.
	@param maxIndex			maximum index of palette.
	
	@return 	vector<int> containing one index for each point
*/
vector<int>
ReflectanceToIndices(
				const LaserPoints& laserPoints,
				bool useReflectance = true,
				int minIndex = 0,
				int maxIndex = 255);
				
/** Converts reflectance value colors. Uses specified palette.
	
	@param laserPoints  	input laser points whose reflectance is to be mapped.
	@param glColorschem		color scheme to use.
	@param useReflectance	use reflectance if true else use z-value.
*/
vector<Color3D>
ReflectanceToColor3D(
				const LaserPoints& laserPoints,
				GLColorScheme glColorScheme,
				bool useReflectance);

/** Draws laser points in openGL. The colors are taken from the requested palette.

	@param laserPoints	input laser points to be drawn.
	@param useReflectance	if true use reflectance, else use z-value
	@param glColorScheme	The identifier of the color scheme to be used.
	
	@return  void
*/
void
DrawLaserPointsGL(const LaserPoints& laserPoints,
				int colorMappingMode = 1,
				GLColorScheme glColorScheme = GLColorSchemeScaledGray,
				int subSamplingRatio = 1);
				
/** Draws laser points as spheres. The colors are taken from the requested palette.

	@param laserPoints	input laser points to be drawn.
	@param useReflectance	if true use reflectance, else use z-value
	@param glColorScheme	The identifier of the color scheme to be used.
	
	@return  void
*/
void
DrawLaserPointsAsSpheresGL(const LaserPoints& laserPoints,
				double radius = 1,
				double stacks = 5,
				bool useReflectance = true,
				GLColorScheme glColorScheme = GLColorSchemeScaledGray,
				int subSamplingRatio = 1);				
				
/** Draws segments vector in OpenGL. 
	The colors are either taken from user specified array, or are generated at random.

	@param laserPoints	input laser points to be drawn.
	@param segmentsVector    segmentation result
	@param colors   colors for each segments, if not of right size use random colors
	
	@return  void
*/
void
DrawSegmentsVectorGL(
				const LaserPoints& laserPoints,
				SegmentsVector& segmentsVector,
				vector<Color3D> colors = vector<Color3D>());

						

/** Draws TIN for a given laserPoints in OpenGL.
Requires tin to be derived already.

	@param laserPoints		input laserpoints.
	@param TIN				derived tin
	@param glColorScheme  	color scheme to be used.
	@param useReflectance	if true use reflectance else use z-values.
	@param smoothNormals	if true smooth normals, else use flat shading.
*/	
void
DrawLaserPointsTINGL(LaserPoints & laserPoints,
					TIN& tin,
					GLColorScheme glColorScheme = GLColorSchemeScaledGray, 
					bool useReflectance = true,
					bool smoothNormals = false);

//Draw in one color.										
void
DrawLaserPointsTINGL(LaserPoints & laserPoints,
					TIN& tin,
					Vector3D color=Vector3D(1,1,1));					
/** Dra TIN for a given laserPoints in OpenGL.
Derives tin for passed laserPoints internally.

	@param laserPoints		input laserpoints.
	@param glColorScheme  	color scheme to be used.
	@param useReflectance	if true use reflectance else use z-values.
	@param smoothNormals	if true smooth normals, else use flat shading.
	
	@return		void
*/
		
void
DrawLaserPointsTINGL(LaserPoints & laserPoints,
					  GLColorScheme glColorScheme = GLColorSchemeScaledGray, 
					  bool useReflectance = true,
					  bool smoothNormals = false);
					  
/** Draws selected laserPoints using opengl.

	@param	laserPoints, input laser data
	@param	selectedIndices, selection indices
	@param	glColorScheme, color scheme desired.
	@param	useReflectance, if true use reflectance, else use z-values.
	
	@return	void
*/	
void
DrawLaserPointsSelectionGL (
					LaserPoints & laserPoints,
					vector<int> selectedIndices,
				  	GLColorScheme glColorScheme = GLColorSchemeScaledRed,
					bool useReflectance = true);
					
/** Draws LaserPoints normals in opengl.
	
	@param laserPoints  points whose normals are to be drawn.
	@param normals  a vector containing normals (must be of same size as laser points)
	@param normalLengths  length of normals.
	@param begin/end Colorscheme.  start and end color of lines drawn to show normals.
	@param useReflectance if true use reflectance for scaling of color
	@param scalingVector  a vector of scaling factors. If NULL no individual scaling 
	
	@return void
*/	
	
void
DrawLaserPointsNormalsGL(
				LaserPoints & laserPoints,
				LaserPoints & normalPoints,
				double specifiedNormalLength,
			    GLColorScheme glColorSchemeBegin=GLColorSchemeScaledGreen,
			    GLColorScheme glColorSchemeEnd=GLColorSchemeScaledRed,
			    bool useReflectance=true,
				vector<double> *pScalingVector=NULL);
					
/** Draws TIN3D in openGL. Uses the specified color scheme and options.

	@param laserPoints   input laser points
	@param tin3D   TIN3D of laser points
	@param glColorScheme    palette number
	@param useReflectance   if true use reflectance else use z value
	@param smoothNormals	if true smooth normals else use flat shading.
*/
/*
void
DrawTIN3DGL(
		LaserPoints & laserPoints,
		TIN3D & tin3D,
		GLColorScheme glColorScheme = GLColorSchemeScaledGray,
		bool useReflectance = true,
		bool smoothNormals = false);
*/		


/*Convert Indices to laser points */
LaserPoints
Indices2LaserPoints(const LaserPoints& pts,const IndicesVector& iv);

/* Assign reflectance to all points */
LaserPoints& AssignReflectance(LaserPoints& pts, double ref);

/* Combine laser points and return the aggregate one */
LaserPoints Combine(const LaserPoints& p1,const LaserPoints& p2);

LaserPoints& Append(LaserPoints& dest, LaserPoints& src);

/* Return max reflectance */
double MaxReflectance(const LaserPoints& laserPoints);

/* Return min Reflectance */
double MinReflectance(const LaserPoints& laserPoints);

/*Scale laser points */
LaserPoints Scale(const LaserPoints& laserPoints,
	const double sx=1,const double sy=1,const double sz=1,const double sr=1);

/*Filter TIN and remove edges whose length exceeds a certain threshold */
TIN
FilterTIN(const LaserPoints& laserPoints,const TIN& tin, double maxLength);

/*Filter TIN and remove edges whose length exceeds a certain threshold 
  this version uses a separate threshold for each point.
*/
TIN
FilterTIN(const LaserPoints& laserPoints,
			const TIN& tin, 
			const vector<double>& thresholds);
			
void
CopyTIN(TIN& src,TIN& dest);			

//Save and load functions for laser objects.
//void WriteAsciiFile(string fileName,vector<LaserObject*> &objects);
//void WriteBnfFile(string fileName,vector<LaserObject*> &objects);
//void ReadAsciiFile(string fileName,vector<LaserObject*>& objects);
//void WriteVRMLObjects(string fileName,vector<LaserObject*>& laserObjects);

LaserPoints ToLaserPoints(const vector<Vector3D>& v);
LaserPoint ToLaserPoint(const Vector3D& v);

LaserPoints ToLaserPoints(const vector<Vector3D>& v,const vector<double>& colors,bool takeLog=false);
LaserPoints ToLaserPoints(const LaserPoints& v,const vector<double>& colors,bool takeLog=false);

Vector3D ToVector3D(LaserPoint pt);
Vector3Ds ToVector3Ds(const LaserPoints& pts);




#endif //_LASERPOINT_UTILITY_H_


