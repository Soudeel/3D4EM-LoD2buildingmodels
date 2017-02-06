
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
*   Purpose   : Provides various utility functions for viewing and visualization.
*
*--------------------------------------------------------------------*/
#include "LaserPoints.h"
#include "MyConstants.h"
#include "Palettes.h"
#include "VisualizationUtility.h"
#include "LaserPointsProcessing.h"
#include "VrmlUtility.h"
#include "stdlib.h"


/** Cluster normals and show as 2D image.

	@param normalPoints  assumes properly initialized and normalized.
	@param thetaCount bins in theta parameter
	@param phiCount	 bins in phi parameter.
	@param takeLog	if true take log of reflectance.
	
	@return none (just show the final result in a new viewer)
*/
void
ShowNormalsAs2DImage(const LaserPoints& normalPoints,
					int thetaCount,
					int phiCount,
					bool takeLog)
{
	//theta count and factor should be calculated dynamically, to cause uniform distribution 
	//of points on sphere. We will keep thetaCount equal to max but adjust thetafactor when
	//updating the image.
	double phiFactor = (phiCount-1)/(M_PI/2);
	thetaCount = 2*M_PI*phiFactor;

	#define IMAGE_TYPE float
	IMAGE_TYPE *image = new float[thetaCount*phiCount];
	memset(image,0,sizeof(IMAGE_TYPE)*thetaCount*phiCount);
	double *thetaFactorLUT = new double[phiCount];
	int i,j,k,offset,phiIndex,thetaIndex;
	
	for(i=0;i<phiCount;i++)
	{
		double phi = i/phiFactor;
		if(sin(phi)<=0)
		{
			thetaFactorLUT[i] = 0;
		}
		else
		{
			int thetaSize = 2*M_PI*sin(phi)*phiFactor;
			thetaFactorLUT[i] = 1.00 / (2*M_PI/(double)(thetaSize-1));
		}
	}

	
	//Update cluster image.
	LaserPoint pt;
	for(i=0;i<normalPoints.size();i++)
	{
		pt = normalPoints[i];
		phiIndex = (acos(pt.Z()))*phiFactor;
		thetaIndex = (atan2(pt.Y(),pt.X())+M_PI)*thetaFactorLUT[phiIndex];
		
		image[thetaIndex*phiCount+phiIndex]++;
	}
	
	//Apply a box filter.
	int s[] = {thetaCount,phiCount};
	int f[] = {10,10};
	
	IMAGE_TYPE **ppImage = new float*[thetaCount];
	for(i=0;i<thetaCount;i++)
		ppImage[i] = image + i*phiCount;
	
	//TODO: box filter doesn't link properly, resolve the issue.	
	//BoxFilter2(ppImage,s,f);
	delete[] ppImage;
	
	//take Log if directed.	
	if(takeLog)
	{
		for(i=0;i<thetaCount*phiCount;i++)
		{
			image[i] = log(image[i]+1);
		}
	}

	//scale reflectance.
	IMAGE_TYPE max = -1000;
	int mi,mj;
	for(i=0;i<thetaCount;i++)
	{
		for(j=0;j<phiCount;j++)
		{
			if(image[i*phiCount+j]>max)
			{
				max = image[i*phiCount+j];
				mi = i; mj = j;
			}
		}
	}
	double theta = mi/thetaFactorLUT[mj];
	double phi = mj/phiFactor;
	
	//convert to laser points
	fprintf(stderr,"NormalsAs2DImage: max %f at (%f,%f,%f)\n",max,cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi));
	double refScaleFactor = 1e6/max;
	LaserPoints imagePoints;
	imagePoints.reserve(thetaCount*phiCount);
	
	for(i=0;i<thetaCount;i++)
	{
		for(j=0;j<phiCount;j++)
		{
			imagePoints.push_back(LaserPoint(i,j,0,image[i*phiCount+j]*refScaleFactor));
		}
	}
	
	//Show LaserPoints.
	ShowLaserPoints(imagePoints,"Normals shown in 2D");
	
	
	#undef IMAGE_TYPE
	
	//free allocate resources.
	delete[] image;
	delete[] thetaFactorLUT;
}

/** Show 3D array in a new viewer.

	@param pppData   input 3D array
	@param s1...s3   sizes of array
	@param title   title of final window
	@param missZeros   if true skip zeros in data
	@param f1...f3   scaling factors for three dimensions.
	
	@return void (just show the result in a new window)
*/	 
template<class T>
void Show3DArray(T*** pppData,
				int s1,int s2,int s3,
				char* title,
				bool missZeros,
				double f1,double f2, double f3)
{
	int j,k,l;
	LaserPoints laserPoints;
	laserPoints.reserve(s1*s2*s3);
	for(j=0;j<s1;j++)
	{
		for(k=0;k<s2;k++)		
		{
			for(l=0;l<s3;l++)
			{
				if((missZeros && pppData[j][k][l])||(!missZeros))
				{
					laserPoints.push_back(LaserPoint(j*f1,k*f2,l*f3,pppData[j][k][l]));
				}
			}
		}
	}
	ShowLaserPoints(laserPoints,title);
}

/** Show 2D array in a new viewer.

	@param ppData   input 2D array
	@param s1...s2   sizes of array
	@param title   title of final window
	@param missZeros   if true skip zeros in data
	@param f1...f2   scaling factors for two dimensions.
	
	@return void (just show the result in a new window)
*/	
template<class T>
void Show2DArray(T** ppData,
				int s1,int s2,
				char* title,
				bool missZeros,
				double f1,double f2)
{
	int j,k;
	LaserPoints laserPoints;
	laserPoints.reserve(s1*s2);
	for(j=0;j<s1;j++)
	{
		for(k=0;k<s2;k++)		
		{
			if((missZeros && ppData[j][k])||(!missZeros))
				laserPoints.push_back(LaserPoint(j*f1,k*f2,0,ppData[j][k]));		
		}
	}
	ShowLaserPoints(laserPoints,title);
}

/** Show 1D array in a new viewer.

	@param pData   input 1D array
	@param s1   sizes of array
	@param title   title of final window
	@param missZeros   if true skip zeros in data
	@param f1   scaling factors for x.
	
	@return void (just show the result in a new window)
*/	

template<class T>
void Show1DArray(T* pData,int s1,char* title,bool missZeros,double f1)
{
	int j;
	LaserPoints laserPoints;
	laserPoints.reserve(s1);
	for(j=0;j<s1;j++)
	{
		if((missZeros && pData[j])||(!missZeros))
			laserPoints.push_back(LaserPoint(j*f1,0,0,pData[j]));				
	}
	ShowLaserPoints(laserPoints,title);
}

/** Calculates and shows distances from a given plane.

	@param laserPoints   points to calculate distances for.
	@param normal   normal of the plane.
	@param distance   rho of the plane
	
	@return none
*/	
void 
ShowDistanceFromPlane(const LaserPoints& laserPoints,
					Vector3D normalDirection,
					double distance)
{
	vector<double> distances;
	distances.resize(laserPoints.size());
	int i;
	for(i=0;i<laserPoints.size();i++)
	{
		LaserPoint pt = laserPoints[i];
		distances[i] = fabs(normalDirection.X()*pt.X()+normalDirection.Y()*pt.Y()+normalDirection.Z()*pt.Z()-distance);
	}
	double maxDistance = distances[0];
	for(i=0;i<distances.size();i++)
	{
		if(distances[i]>maxDistance)
			maxDistance = distances[i];
	}
	LaserPoints displayPoints = laserPoints;
	double factor = 1e4/maxDistance;
	
	for(i=0;i<laserPoints.size();i++)
	{
		displayPoints[i].Reflectance() = distances[i]*factor;
	}
	ShowLaserPoints(displayPoints,"Distance from Plane");
}														

/** Shows laser points in viewer. 
	Currently uses vrml view with a randomly generated file name.
	
	@return none
*/	
void
ShowLaserPoints(
		const LaserPoints& laserPoints,
		const char* title,
		GLColorScheme color,
		bool useReflectance)
{
	
	char fileName[1024];
	static int counter = 0; counter++;
	
	if(title == NULL)
	{
		sprintf(fileName,"ShowLaserPoints__%d.wrl",counter);
	}
	else
	{
		sprintf(fileName,"%s____ShowLaserPoints%d.wrl",title,counter);
	}
	
	VrmlFile vrmlFile(fileName);
	vrmlFile.Write(laserPoints,color,useReflectance);
	vrmlFile.Show();

	
/*
	//Code to show in laser viewer.
	LaserViewer *viewer = new LaserViewer ();
	viewer->SetLaserPoints (laserPoints);
	viewer->Show ();
	if(title)
		fl_set_form_title(viewer->GetForm(),title);
*/	
	
	
	
}

void
ShowSegmentation(
		const LaserPoints& laserPoints,
		const vector<vector<int> >& segments,
		const char* title,
		bool useReflectance)
{
	char fileName[1024];
	static int counter = 0; counter++;
	
	if(title == NULL)
	{
		sprintf(fileName,"ShowSegmentation__%d.wrl",counter);
	}
	else
	{
		sprintf(fileName,"%s____ShowSegmentation__%d.wrl",title,counter);
	}
	
	VrmlFile vrmlFile(fileName);
	vrmlFile.Write(laserPoints,segments,useReflectance);
	vrmlFile.Show();	
}	

void
ShowLaserPoints(
		const LaserPoints& laserPoints,
		const LaserPoints& normalPoints,
		const char* title,
		const double normalLength,
        const Color3D beginColor,
        const Color3D endColor,
		GLColorScheme color,
		bool useReflectance)
{
	
	char fileName[1024];
	static int counter = 0; counter++;
	
	if(title == NULL)
	{
		sprintf(fileName,"ShowLaserPoints__%d.wrl",counter);
	}
	else
	{
		sprintf(fileName,"%s____ShowLaserPoints%d.wrl",title,counter);
	}
	
	VrmlFile vrmlFile(fileName);
	vrmlFile.Write(laserPoints,color,useReflectance);
	LaserPointsNormals2Vrml(vrmlFile.GetFile(),laserPoints,normalPoints,
							normalLength,beginColor,endColor);
	vrmlFile.Show();

	
/*
	//Code to show in laser viewer.
	LaserViewer *viewer = new LaserViewer ();
	viewer->SetLaserPoints (laserPoints);
	viewer->Show ();
	if(title)
		fl_set_form_title(viewer->GetForm(),title);
*/	
	
	
	
}

/** Shows vrml file in Vrmlview.
	
	@param vrmlFileName   name of the file to show
	
	@return none
*/	
void ShowInVrmlview(const char* vrmlFileName)
{
	char buff[2048];
	snprintf(buff,2048,"vrmlview \"%s\" &",vrmlFileName);
	fprintf(stderr,"Executing: %s\n",buff);
	system(buff);
}
										
/** Show given points in a new laser viewer
*/
void
ShowInViewer(const LaserPoints& pts,char* title)
{
/*	
	LaserViewer *viewer = new LaserViewer ();
	viewer->SetLaserPoints (pts);
	char name[] = "ShowLaserPointsInLaserViewer";
	char* temp = (title)?title:name;
	
	viewer->SetTitle(temp);
	viewer->Show ();
*/	
}

