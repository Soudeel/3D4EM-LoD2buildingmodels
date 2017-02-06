
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
#include <iostream>
#include "LaserPoints.h"
#include "LaserPointsUtility.h"
#include "Palettes.h"
#include "GeneralUtility.h"
#include "OpenGLUtility.h"
#include "ParseUtility.h"
#include "GLIndexedFaceSet.h"

using namespace std;

/** Histogram equalizes reflectance of the given laserpoints.

    @param laserPoints input laser points. The reflectance is modified on return.
    @return void (The input laser points are changed)
*/
void HistoEqualizeLaserPoints(LaserPoints& laserPoints)
{
	#define HISTO_SIZE 4096*4
	double pHistogram[HISTO_SIZE];
	double pLut[HISTO_SIZE];
	double scalingFactor;
	int i;
	double maxReflectance,minReflectance;
	
	//Initialize buffers to zero.
	memset(pHistogram,0,sizeof(*pHistogram)*HISTO_SIZE);
	memset(pLut,0,sizeof(*pLut)*HISTO_SIZE);
	
	//Find min and max of reflectance.
	maxReflectance = minReflectance = laserPoints[0].Reflectance();
	for(i=0;i<laserPoints.size();i++)
	{
		maxReflectance = MAX(maxReflectance,laserPoints[i].Reflectance());
		minReflectance = MIN(minReflectance,laserPoints[i].Reflectance());
	}
	
	scalingFactor = (double)(HISTO_SIZE-1)/(double)(maxReflectance-minReflectance);
	
	//Scale Reflectance to 0 to 255.
	for(i=0;i<laserPoints.size();i++)
	{
		laserPoints[i].Reflectance() = ((laserPoints[i].Reflectance()-minReflectance)*scalingFactor);
	}
	
	//Make histogram.
	for(i=0;i<laserPoints.size();i++)
	{
		pHistogram[laserPoints[i].Reflectance()]++;
	}
	
	//Calculative cumulative histogram.
	for(i=1;i<HISTO_SIZE;i++)
	{
		pHistogram[i] = pHistogram[i]+pHistogram[i-1];
	}
	
	//Calculate scaling factor, that will convert the cumulative histogram to lut.
	scalingFactor = (double)(HISTO_SIZE)/(double)(pHistogram[HISTO_SIZE-1]-pHistogram[0]+1e-3);
	
	//Update lut.
	for(i=0;i<HISTO_SIZE;i++)
	{
		pLut[i] = scalingFactor*(pHistogram[i]-pHistogram[0]);
		//cerr<<"pLut["<<i<<"]: "<<pLut[i]<<endl;
	}
	
	//Apply lut to laserPoints Reflectances.
	for(i=0;i<laserPoints.size();i++)
	{
		laserPoints[i].Reflectance() = pLut[laserPoints[i].Reflectance()];
	}
}

/** Adds uniformly distributed multiplicative and additive noise to x,y,z values of laser points.
	The formula is a = (a + additiveNoiseLevel*random(-1,1))*(1+multiplicativeNoise*random(-1,1))
	
	@param laserPoints  input points are modified on return.
	@param noiseLevelAdditive	level of additive noise
	@param noiseLevelMultiplicative	level of multiplicative noise
*/	
void
AddNoiseToLaserPoints (LaserPoints & laserPoints, double noiseLevelAdditive, double noiseLevelMultiplicative)
{
	double x,y,z;
	for (int i = 0; i < laserPoints.size (); i++)
	{
		x = laserPoints[i].X();
		y = laserPoints[i].Y();
		z = laserPoints[i].Z();
		
		laserPoints[i].X() = (x+noiseLevelAdditive*GenerateRandom(-1,1))*(1+noiseLevelMultiplicative*GenerateRandom(-1,1));
		laserPoints[i].Y() = (y+noiseLevelAdditive*GenerateRandom(-1,1))*(1+noiseLevelMultiplicative*GenerateRandom(-1,1));
		laserPoints[i].Z() = (z+noiseLevelAdditive*GenerateRandom(-1,1))*(1+noiseLevelMultiplicative*GenerateRandom(-1,1));
	}
}
	
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
GenerateCylinderPoints (double *radius, 
			int radiusSize, 
			int circlePoints,
			double circleExtent,
			double* stacks, 
			int stackSize)
{

	int i, j, k;
	LaserPoints cylinderPoints;
	double *radiusLocal,*stacksLocal;
	
	radiusLocal = radius;
	stacksLocal = stacks;
	
	
	if(!radiusLocal)
	{
		radiusSize = (radiusSize>=1)?radiusSize:10;
		radiusLocal = new double[radiusSize];
		
		for(i=0;i<radiusSize;i++)
		{
			radiusLocal[i]=1;
		}
	}
	
	if(!stacksLocal)
	{
		stackSize = (stackSize>=1)?stackSize:10;
		stacksLocal = new double[stackSize];
		
		for(i=0;i<stackSize;i++)
			stacks[i] = i;
	} 
		
	//Make LUT and calculate factors.
	circleExtent = (circleExtent<=0)?1:circleExtent;
	circleExtent = (circleExtent>1)?1:circleExtent;
	double thetaFactor = circleExtent*2 * M_PI / (double) circlePoints;
	double *pSinData = new double[circlePoints];
	double *pCosData = new double[circlePoints];

	for (i = 0; i < circlePoints; i++)
	{
		pSinData[i] = sin (thetaFactor * i);
		pCosData[i] = cos (thetaFactor * i);
	}

	//Reserve enough space.
	cylinderPoints.reserve (radiusSize * circlePoints*stackSize);
	
	//Populate the points.
	for (j = 0; j < radiusSize; j++)
	{
		double r = radiusLocal[j];
		double z = j / (double) radiusSize;
		double x, y;

		for (i = 0; i < circlePoints; i++)
		{
			x = r * pSinData[i];
			y = r * pCosData[i];
			
			for(k=0;k<stackSize;k++)
			{
				z = stacksLocal[k];
				cylinderPoints.push_back(LaserPoint (x, y, z, z));
			}
		}
	}

	//Free allocated resources.
	delete[]pSinData;
	delete[]pCosData;
	
	if(radiusLocal!=radius)
		delete[] radiusLocal;
		
	if(stacksLocal!=stacks)
		delete[] stacksLocal;

	//return with points.
	return cylinderPoints;
}


/** Make laser points for spatially varying sine wave of given frequencies.

	@param x1,x2,y1,y2 Extents of space
	@praam countX,countY number of points in X and Y
	@param frequencyX spatial frequency in X direction
	@param frequencyY spatial frequency in Y direction.
	
	@return laserpoints containing height variation of given sine freq.
*/
LaserPoints
MakeSinePoints (double x1,double x2,
				double y1,double y2,
				int countX, int countY,
				double frequencyX,double frequencyY)
{
	LaserPoints sinePoints;

	//calculate step sizes.
	double dX = (x2 - x1) / (double) countX;
	double dY = (y2 - y1) / (double) countY;
	double radiansPerUnitX = 2 * M_PI * frequencyX / (x2 - x1);
	double radiansPerUnitY = 2 * M_PI * frequencyY / (y2 - y1);
	double x, y;
	
	//iterate and populate laser points.
	for (x = x1; x < x2; x += dX)
	{
		for (y = y1; y < y2; y += dY)
		{
			double z = sin (sqrt(pow(x * radiansPerUnitX,2) + pow(y * radiansPerUnitY,2)));
			sinePoints.push_back (LaserPoint(x, y,z,(z+1)*50));
		}
	}
	
	//return laser points.
	return sinePoints;
}

//Set of macros, used by each list function for palette lookup.             
#define R_VALUE(a) (unsigned char)(((a)-zMin)*zFactor + zOffset)
#define G_VALUE(a) (unsigned char)(((a)-zMin)*zFactor + zOffset)
#define B_VALUE(a) (unsigned char)(((a)-zMin)*zFactor + zOffset)
#define Z_OFFSET_PERCENT 0.3
#define Z_FACTOR_PERCENT (1-Z_OFFSET_PERCENT-0.05)

/** Converts reflectance value of passed laser points to indices. Can be used for palette lookup.
	
	@param laserPoints  	input laser points whose reflectance is to be mapped.
	@param useReflectance	use reflectance if true else use z-value.
	@param minIndex			minimum index of palette.
	@param maxIndex			maximum index of palette.
*/
vector<int>
ReflectanceToIndicesOld(
				const LaserPoints& laserPoints,
				bool useReflectance,
				int minIndex,
				int maxIndex)
{
	//Calculate scaling factors for palette lookup.
	double factor = 1.00/(maxIndex-minIndex);
	int indexRange = maxIndex - minIndex;
	
	vector<int> indices(laserPoints.size());
	
	//Make a new list.Will replace the last one.
	if (useReflectance)
	{
		double maxReflectance = -1e30;
		double minReflectance = 1e30;
		
		//Find min,max reflectance.
		for (int i = 0; i < laserPoints.size (); i++)
		{
			LaserPoint currentPoint = laserPoints[i];
			if (currentPoint.Reflectance () < minReflectance)
				minReflectance = currentPoint.Reflectance ();
			if (currentPoint.Reflectance () > maxReflectance)
				maxReflectance = currentPoint.Reflectance ();
		}

		double grayScaleOffset = 0.2 * indexRange;
		double grayScaleFactor = 0.8;
		double reflectanceRange = fabs (maxReflectance - minReflectance);
		if (reflectanceRange > 1e-3)
		{
			grayScaleFactor = indexRange * 0.8 / reflectanceRange;
		}
		else
		{
			grayScaleOffset = 0.8*indexRange;
		}


		for (int i = 0; i < laserPoints.size (); i++)
		{
			LaserPoint currentPoint = laserPoints[i];
			int index = (unsigned int) (grayScaleOffset +	grayScaleFactor *(currentPoint.Reflectance () - minReflectance));
			indices[i] = index - minIndex;
		}
	}
	else
	{
		//Use z-value for scaling.
		DataBoundsLaser dataBounds = ((LaserPoints)laserPoints).DeriveDataBounds (0);
		double zMin = dataBounds.Minimum ().Z ();
		double zMax = dataBounds.Maximum ().Z ();
		double zFactor = Z_FACTOR_PERCENT * (double) indexRange / (zMax - zMin);
		double zOffset = Z_OFFSET_PERCENT * (double) indexRange;
		
		for (int i = 0; i < laserPoints.size (); i++)
		{			
			LaserPoint currentPoint = laserPoints[i];
			int index = (unsigned char)(((currentPoint.Z())-zMin)*zFactor + zOffset);
			indices[i] = index - minIndex;
		}
	}
	
	if(laserPoints.size()<=2)
		indices = vector<int>(laserPoints.size(),(minIndex+maxIndex)*0.5);
		
	//return function status.
	return indices;


}		

vector<int>
ReflectanceToIndices(
				const LaserPoints& laserPoints,
				bool useReflectance,
				int minIndex,
				int maxIndex)
{
	//Calculate scaling factors for palette lookup.
	double factor = 1.00/(maxIndex-minIndex);
	int indexRange = maxIndex - minIndex;
	
	vector<int> indices(laserPoints.size());
	
	//Make a new list.Will replace the last one.
	if (useReflectance)
	{
		double maxReflectance = laserPoints.DataBounds().Maximum().Reflectance();
		double minReflectance = laserPoints.DataBounds().Minimum().Reflectance();
		
		double grayScaleOffset = 0.2 * indexRange;
		double grayScaleFactor = 0.8;
		double reflectanceRange = fabs (maxReflectance - minReflectance);
		if (reflectanceRange > 1e-3)
		{
			grayScaleFactor = indexRange * 0.8 / reflectanceRange;
		}
		else
		{
			grayScaleOffset = 0.8*indexRange;
		}


		for (int i = 0; i < laserPoints.size (); i++)
		{
			LaserPoint currentPoint = laserPoints[i];
			int index = (unsigned int) (grayScaleOffset +	grayScaleFactor *(currentPoint.Reflectance () - minReflectance));
			indices[i] = index - minIndex;
		}
	}
	else
	{
		//Use z-value for scaling.
		DataBoundsLaser dataBounds = laserPoints.DataBounds();
		double zMin = dataBounds.Minimum ().Z ();
		double zMax = dataBounds.Maximum ().Z ();
		double zFactor = Z_FACTOR_PERCENT * (double) indexRange / (zMax - zMin);
		double zOffset = Z_OFFSET_PERCENT * (double) indexRange;
		
		for (int i = 0; i < laserPoints.size (); i++)
		{			
			LaserPoint currentPoint = laserPoints[i];
			int index = (unsigned char)(((currentPoint.Z())-zMin)*zFactor + zOffset);
			indices[i] = index - minIndex;
		}
	}
	
	if(laserPoints.size()<=2)
		indices = vector<int>(laserPoints.size(),(minIndex+maxIndex)*0.5);
		
	//return function status.
	return indices;


}		
		
/** Converts reflectance value colors. Uses specified palette.
	
	@param laserPoints  	input laser points whose reflectance is to be mapped.
	@param useReflectance	use reflectance if true else use z-value.
*/
vector<Color3D>
ReflectanceToColor3D(
				const LaserPoints& laserPoints,
				GLColorScheme glColorScheme,
				bool useReflectance)
{
	
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	vector<Color3D> colors(laserPoints.size());
	double scaleFactor = 1.00/255.00;
	
	for(int i=0;i<laserPoints.size();i++)
	{
		BYTE index = indices[i];
		colors[i] = Color3D(paletteData[index][0]*scaleFactor,
							paletteData[index][1]*scaleFactor,
							paletteData[index][2]*scaleFactor);
	}

	
	//return function status.
	return colors;


}				
						

/** Draws laser points in openGL. The colors are taken from the requested palette.

	@param laserPoints	input laser points to be drawn.
	@param useReflectance	if true use reflectance, else use z-value
	@param glColorScheme	The identifier of the color scheme to be used.
	
	@return  void
*/
void
DrawLaserPointsGL(const LaserPoints& laserPoints,
				int useReflectance,
				GLColorScheme glColorScheme,
				int subSamplingRatio)
{				
	
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	//Start drawing.
	glBegin(GL_POINTS);
	for(int i=0;i<=(laserPoints.size()-subSamplingRatio);i+=subSamplingRatio)
	{
		BYTE index = indices[i];
		glColor3ub(paletteData[index][0],paletteData[index][1],paletteData[index][2]);
		glVertex3f(laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z());
	}
	glEnd();
}

void
DrawLaserPointsAsSpheresGL(const LaserPoints& laserPoints,
				double radius,
				double stacks,
				bool useReflectance,
				GLColorScheme glColorScheme,
				int subSamplingRatio)
{			
	
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	//Start drawing.
	glMatrixMode(GL_MODELVIEW);
	for(int i=0;i<(laserPoints.size()-subSamplingRatio);i+=subSamplingRatio)
	{
		BYTE index = indices[i];
		glColor3ub(paletteData[index][0],paletteData[index][1],paletteData[index][2]);
		glPushMatrix();
			glTranslatef(laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z());
			glutSolidSphere(radius,stacks,stacks);
		glPopMatrix();
	}
}							

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
				vector<Color3D> colors)
{
	int i,j;	
	static vector<Color3D> localColors;
	//Generate random colors.
	
	for(j=localColors.size();j<segmentsVector.size();j++)
		localColors.push_back(GenerateRandomColor());
	
	glBegin(GL_POINTS);
	for(i=0;i<segmentsVector.size();i++)
	{
		//specify color
		if(colors.size()!=segmentsVector.size())
		{
			glColor3f(localColors[i].X(),localColors[i].Y(),localColors[i].Z());
		}
		else
			glColor3f(colors[i].X(),colors[i].Y(),colors[i].Z());
		
		//draw segments.
		for(j=0;j<segmentsVector[i].size();j++)
		{
			LaserPoint pt = laserPoints[segmentsVector[i][j]];
			glVertex3f(pt.X(),pt.Y(),pt.Z());
		}
	}
	glEnd();
			
}		

//Helper function for smoothing of normals.
static void AddVector3D(Vector3D& dest, Vector3D& src)
{
	dest.X() = dest.X() + src.X();
	dest.Y() = dest.Y() + src.Y();
	dest.Z() = dest.Z() + src.Z();
}

/** Draws TIN for a given laserPoints in OpenGL.

	@param laserPoints		input laserpoints.
	@param glColorScheme  	color scheme to be used.
	@param useReflectance	if true use reflectance else use z-values.
	@param smoothNormals	if true smooth normals, else use flat shading.
*/	
void
DrawLaserPointsTINGL(LaserPoints & laserPoints,
					  GLColorScheme glColorScheme, 
					  bool useReflectance,
					  bool smoothNormals)
{
	if (!laserPoints.size ())
		return ;
		
	//Derieve TIN and get a ref to it.
	laserPoints.DeriveTIN();
	TIN& tinRef = laserPoints.TINReference();
	
	return
	DrawLaserPointsTINGL(laserPoints,
					  tinRef,
					  glColorScheme, 
					  useReflectance,
					  smoothNormals);

}

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
					GLColorScheme glColorScheme, 
					bool useReflectance,
					bool smoothNormals)
{
	if (!laserPoints.size ())
		return ;
		
	Vector3D v1, v2, v3;

	//Get palette lookup indices.
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	//Load selected palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	//calculate normals for tin.
	vector<Vector3D> normals;
	vector<int> count;

	//Calculate and smooth normals.
	if(smoothNormals)
	{
		normals.resize(laserPoints.size());	
		count.resize(laserPoints.size());	
		for (int i = 0; i < tin.size (); i++)
		{
			TINMesh currentMesh = tin[i];
			PointNumber *points = tin[i].Nodes ();
		
			if(points)
			{
				v1 = laserPoints[points[0].Number()] - laserPoints[points[1].Number()];
				v2 = laserPoints[points[2].Number()] - laserPoints[points[1].Number()];
				v3 = v2.VectorProduct(v1);
			
				for(int j=0;j<3;j++)
				{
					AddVector3D(normals[points[j].Number()],v3);
					count[points[j].Number()]++;
				}
			}
		}
	
		for(int i=0;i<laserPoints.size();i++)
		{
			if(count[i])
				normals[i]*(1.00/count[i]);
		}
	}

	//Start drawing.
	glBegin (GL_TRIANGLES);
	for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		PointNumber *points = tin[i].Nodes ();
		
		if(!smoothNormals)
		{
			v1 = laserPoints[points[0].Number()] - laserPoints[points[1].Number()];
			v2 = laserPoints[points[2].Number()] - laserPoints[points[1].Number()];
			v3 = v2.VectorProduct(v1);
	
			glNormal3f(v3.X(),v3.Y(),v3.Z());
		}
		
		for (int j = 0; j < 3; j++)
		{
			LaserPoint currentPoint = laserPoints[points[j].Number()];
			
			if(smoothNormals)
			{
				v3 = normals[points[j].Number()];
				glNormal3f(v3.X(),v3.Y(),v3.Z());
			}
			BYTE index = indices[points[j].Number()];
			glColor3ub (paletteData[index][0], paletteData[index][1],paletteData[index][2]);
			glVertex3f (currentPoint.X (), currentPoint.Y (),currentPoint.Z ());
		}			
	}
	glEnd ();
}


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
					Vector3D color)
{
	if (!laserPoints.size ())
		return ;
		
	Vector3D v1, v2, v3;

	//Start drawing.
	glBegin (GL_TRIANGLES);
	glColor3f(color.X(),color.Y(),color.Z());
	for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		PointNumber *points = tin[i].Nodes ();
		
		v1 = laserPoints[points[0].Number()] - laserPoints[points[1].Number()];
		v2 = laserPoints[points[2].Number()] - laserPoints[points[1].Number()];
		v3 = v2.VectorProduct(v1);
	
		glNormal3f(v3.X(),v3.Y(),v3.Z());
		
		for (int j = 0; j < 3; j++)
		{
			LaserPoint currentPoint = laserPoints[points[j].Number()];
			glVertex3f (currentPoint.X (), currentPoint.Y (),currentPoint.Z ());
		}			
	}
	glEnd ();
}


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
				  	GLColorScheme glColorScheme,
					bool useReflectance)
{
	LaserPoints temp;
	temp.resize(selectedIndices.size());

	
	for(int i=0;i<selectedIndices.size();i++)				
	{
		temp[i] = laserPoints[selectedIndices[i]];
	}
	temp.DeriveDataBounds(0);
	return DrawLaserPointsGL(temp,useReflectance,glColorScheme);
}


//Draws LaserPoints normals.
void
DrawLaserPointsNormalsGL(
				LaserPoints & laserPoints,
				LaserPoints & normalPoints,
				double specifiedNormalLength,
			    GLColorScheme glColorSchemeBegin,
			    GLColorScheme glColorSchemeEnd,
			    bool useReflectance,
				vector<double> *pScalingVector)
{
	if (!(laserPoints.size ())|| (laserPoints.size () > normalPoints.size ()))
		return ;

	//Get palette lookup indices.
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	//Make a palette.
	BYTE paletteBegin[256][3];
	LoadSelectedPalette (paletteBegin, 256, glColorSchemeBegin);
	BYTE paletteEnd[256][3];
	LoadSelectedPalette (paletteEnd, 256, glColorSchemeEnd);

	//Make a new list for line.
	glBegin (GL_LINES);
	double normalLength = specifiedNormalLength;
	for (int i = 0; i < laserPoints.size (); i++)
	{
		LaserPoint currentPoint = laserPoints[i];
		LaserPoint currentNormal = normalPoints[i];
		
		BYTE index = indices[i];

		//Color and location of vertex 1 of normal line
		glColor3ub (paletteBegin[index][0],paletteBegin[index][1],paletteBegin[index][2]);
		glVertex3f (currentPoint.X (), currentPoint.Y (), currentPoint.Z ());

		//Color and location of vertex 2 of normal line
		glColor3ub (paletteEnd[index][0],paletteEnd[index][1],paletteEnd[index][2]);
							
		if(pScalingVector && pScalingVector->size()>=laserPoints.size())
		{
			normalLength = specifiedNormalLength*((*pScalingVector)[i]);
		}
		glVertex3f (currentPoint.X () + normalLength * currentNormal.X (),
					currentPoint.Y () + normalLength * currentNormal.Y (),
					currentPoint.Z () + normalLength * currentNormal.Z ());
	}
	glEnd ();
}

/** Draws TIN3D in openGL. Uses the specified color scheme and options.
*/
void
DrawTIN3DGL(
		LaserPoints & laserPoints,
		TIN3D & tin3D,
		GLColorScheme glColorScheme,
		bool useReflectance,
		bool smoothNormals)
{
	if (!(laserPoints.size ()) || !(tin3D.size ()))
		return ;
		
	Vector3D v1, v2, v3;
	int i,j;
	
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	//Get indices.
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	//calculate normals for tin3D.
	vector<Vector3D> normals;
	vector<int> count;

	if(smoothNormals)
	{
		normals.resize(laserPoints.size());	
		count.resize(laserPoints.size());	
		for (int i = 0; i < tin3D.size (); i++)
		{			
			v1 = laserPoints[tin3D[i][0]] - laserPoints[tin3D[i][1]];
			v2 = laserPoints[tin3D[i][2]] - laserPoints[tin3D[i][1]];
			v3 = v2.VectorProduct(v1);
			
			for(j=0;j<3;j++)
			{
				AddVector3D(normals[tin3D[i][j]],v3);
				count[tin3D[i][j]]++;
			}
		}
	
		for(int i=0;i<laserPoints.size();i++)
		{
			if(count[i])
				normals[i]*(1.00/count[i]);
		}
	}

	//Start drawing.
	glBegin (GL_TRIANGLES);
	for (int i = 0; i < tin3D.size (); i++)
	{
		if(!smoothNormals)
		{
			v1 = laserPoints[tin3D[i][0]] - laserPoints[tin3D[i][1]];
			v2 = laserPoints[tin3D[i][2]] - laserPoints[tin3D[i][1]];
			v3 = v2.VectorProduct(v1);						
			glNormal3f(v3.X(),v3.Y(),v3.Z());
		}

		for (int j = 0; j < 3; j++)
		{
			LaserPoint currentPoint = laserPoints[tin3D[i][j]];
			
			if(smoothNormals)
			{
				v3 = normals[tin3D[i][j]];
				glNormal3f(v3.X(),v3.Y(),v3.Z());
			}
			BYTE index = indices[i];
			glColor3ub(paletteData[index][0],paletteData[index][1],paletteData[index][2]);		
			glVertex3f (currentPoint.X (), currentPoint.Y (),currentPoint.Z ());
		}			
	}
	glEnd ();
}

/*Convert Indices to laser points */
LaserPoints
Indices2LaserPoints(const LaserPoints& pts,const IndicesVector& iv)
{
	LaserPoints sel;
	sel.reserve(iv.size());
	for(int i=0;i<iv.size();i++)
	{
		if(iv[i]>=0 && iv[i]<pts.size())
			sel.push_back(pts[iv[i]]);
	}
	return sel;
}


/* Assign reflectance to all points */
LaserPoints& AssignReflectance(LaserPoints& pts, double ref)
{
	for(int i=0;i<pts.size();i++)
		pts[i].Reflectance() = ref;
	return pts;
}

/* Combine laser points and return the aggregate one */
LaserPoints Combine(const LaserPoints& p1,const LaserPoints& p2)
{
	LaserPoints pts;
	pts.reserve(p1.size()+p2.size());
	
	for(int i=0;i<p1.size();i++)
		pts.push_back(p1[i]);
		
	for(int i=0;i<p2.size();i++)
		pts.push_back(p2[i]);
		
	return pts;
}

LaserPoints& Append(LaserPoints& dest, LaserPoints& src)
{
	dest.reserve(dest.size()+src.size());
	for(int i=0;i<src.size();i++)
		dest.push_back(src[i]);
	return dest;
}
		
/* Return max reflectance */
double MaxReflectance(const LaserPoints& laserPoints)
{
	double maxR = 0;
	if(laserPoints.size())
	{
		maxR = laserPoints[0].Reflectance();
		for(int i=0;i<laserPoints.size();i++)
		{
			maxR = MAX( laserPoints[i].Reflectance(),maxR);
		}
	}
	return maxR;
}

/* Return min Reflectance */
double MinReflectance(const LaserPoints& laserPoints)
{
	double minR = 0;
	if(laserPoints.size())
	{
		minR = laserPoints[0].Reflectance();
		for(int i=0;i<laserPoints.size();i++)
		{
			minR = MIN( laserPoints[i].Reflectance(),minR);
		}
	}
	return minR;
}


/*Filter TIN and remove edges whose length exceeds a certain threshold */
TIN
FilterTIN(const LaserPoints& laserPoints,const TIN& tin, double maxLength)
{
	TIN ans;
	ans.reserve(tin.size());
	
	if (!laserPoints.size ())
		return ans;
	
	Vector3D v1,v2,v3;	
	for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		const PointNumber *points = tin[i].Nodes ();

		if(points)
		{
			v1 = laserPoints[points[0].Number()] - laserPoints[points[1].Number()];
			v2 = laserPoints[points[2].Number()] - laserPoints[points[1].Number()];
			v3 = laserPoints[points[2].Number()] - laserPoints[points[0].Number()];
			if(v1.Length()<=maxLength && v2.Length()<=maxLength && v3.Length()<=maxLength)
				ans.push_back(currentMesh);
		}
	}
	return ans;
}

/*Filter TIN and remove edges whose length exceeds a certain threshold 
  this version uses a separate threshold for each point.
*/
TIN
FilterTIN(const LaserPoints& laserPoints,
			const TIN& tin, 
			const vector<double>& thresholds)
{
	TIN ans;
	ans.reserve(tin.size());
	
	if (!laserPoints.size ())
		return ans;
	
	for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		const PointNumber *points = tin[i].Nodes ();

		if(points)
		{
			#define SH(a) cerr<<#a<<": "<<a<<"  ";
			double v1 = (laserPoints[points[0].Number()] - laserPoints[points[1].Number()]).Length();
			double v2 = (laserPoints[points[2].Number()] - laserPoints[points[1].Number()]).Length();
			double v3 = (laserPoints[points[2].Number()] - laserPoints[points[0].Number()]).Length();
			double t0 = thresholds[points[0].Number()];
			double t1 = thresholds[points[1].Number()];
			double t2 = thresholds[points[2].Number()];
			//SH(v1) SH(v2) SH(v3) SH(t0) SH(t1) SH(t2) cerr<<endl;
			vector<double> s(3); s[0]=t0;s[1]=t1;s[2]=t2;
			sort(s.begin(),s.end());
			t0 = t1 = t2 = s[0]+s[1];
			
			
			//This one ensures one condition of each of three edges
			//if((v1<=t0 || v1<=t1) && (v2<=t2 || v2<=t1) && (v3<=t2 || v2<=t0))
			
			//If only for two edges, lenient criteria is desired
			if(((v1<=t0 && v1<=t1) + (v2<=t2 && v2<=t1) + (v3<=t2 && v2<=t0))>=2)
			{
				ans.push_back(currentMesh);
			}
		}
	}
	return ans;
}

/*Scale laser points */
LaserPoints Scale(const LaserPoints& laserPoints,
	const double sx,const double sy,const double sz,const double sr)
{
	LaserPoints pts = laserPoints;
	
	for(int i=0;i<pts.size();i++)
	{
		pts[i].X() = pts[i].X()*sx;
		pts[i].Y() = pts[i].Y()*sy;
		pts[i].Z() = pts[i].Z()*sz;
		pts[i].Reflectance() = pts[i].Reflectance()*sr;
	}
	return pts;
}	

void
CopyTIN(TIN& src,TIN& dest)
{
	dest.resize(0);
	
	for (int i = 0; i < src.size (); i++)
	{
		dest.push_back(src[i]);
	}
}

LaserPoints ToLaserPoints(const vector<Vector3D>& v)
{
	LaserPoints pts;
	pts.resize(v.size());
	
	for(int i=0;i<v.size();i++)
		pts[i] = LaserPoint(v[i].X(),v[i].Y(),v[i].Z());
		
	return pts;
}

LaserPoints ToLaserPoints(const vector<Vector3D>& v,const vector<double>& colors,bool takeLog)
{
	if(colors.size()>=v.size())
	{
		double max_color = *max_element(colors.begin(),colors.end());
		double min_color = *min_element(colors.begin(),colors.end());

		double factor = (max_color != min_color)?1e5/(max_color-min_color):1;

		LaserPoints pts;
		pts.resize(v.size());

		for(int i=0;i<v.size();i++)
		{
			if(takeLog)
			{
				pts[i] = LaserPoint(v[i].X(),v[i].Y(),v[i].Z(),1e5*log(fabs((colors[i]-min_color)*factor)+1));
			}
			else
				pts[i] = LaserPoint(v[i].X(),v[i].Y(),v[i].Z(),(colors[i]-min_color)*factor);
		}

		return pts;
	}
	else
		return ToLaserPoints(v);
}


LaserPoints ToLaserPoints(const LaserPoints& v,const vector<double>& colors,bool takeLog)
{
	if(colors.size()>=v.size())
	{
		double max_color = *max_element(colors.begin(),colors.end());
		double min_color = *min_element(colors.begin(),colors.end());

		double factor = (max_color != min_color)?1e5/(max_color-min_color):1;

		LaserPoints pts;
		pts.resize(v.size());

		for(int i=0;i<v.size();i++)
		{
			if(takeLog)
			{
				pts[i] = LaserPoint(v[i].X(),v[i].Y(),v[i].Z(),1e5*log(fabs((colors[i]-min_color)*factor)+1));
			}
			else
				pts[i] = LaserPoint(v[i].X(),v[i].Y(),v[i].Z(),(colors[i]-min_color)*factor);
		}

		return pts;
	}
	else
		return v;
}

Vector3D ToVector3D(LaserPoint pt)
{
	return Vector3D(pt.X(),pt.Y(),pt.Z());
}

Vector3Ds ToVector3Ds(const LaserPoints& pts)
{
	Vector3Ds v(pts.size());
	
	for(int i=0;i<pts.size();i++)
	{
		v[i] = ToVector3D(pts[i]);
	}
	
	return v;
}

LaserPoint ToLaserPoint(const Vector3D& v)
{
	return LaserPoint(v.X(),v.Y(),v.Z());
}


GLIndexedFaceSet ToGLIndexedFaceSet(const LaserPoints& laserPoints,const TIN& tin)
{
	GLIndexedFaceSet fs;
	fs.GetCoordinateVector() = ToVector3Ds(laserPoints);
	IndicesVector& iv = fs.GetIndicesVector();
	
	for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		const PointNumber *points = tin[i].Nodes ();
		
		for (int j = 0; j < 3; j++)
		{
			iv.push_back(points[j].Number());
		}			
		iv.push_back(-1);
	}
	return fs;
}

