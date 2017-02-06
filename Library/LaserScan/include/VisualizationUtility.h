
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
#ifndef _VISUALIZATION_UTILITY_H_
#define _VISUALIZATION_UTILITY_H_
#include "LaserPoints.h"
#include "MyConstants.h"
#include "Palettes.h"
#include "MyTypeDefinitions.h"

/** Cluster normals and show as 2D image.

	@param normalPoints  assumes properly initialized and normalized.
	@param thetaCount bins in theta parameter
	@param phiCount	 bins in phi parameter.
	@param takeLog	if true take log of reflectance.
	
	@return none (just show the final result in a new viewer)
*/
void
ShowNormalsAs2DImage(const LaserPoints& normalPoints,
					int thetaCount = 25,
					int phiCount = 25,
					bool takeLog = false);



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
				char* title = "Show3DArray",
				bool missZeros = true,
				double f1=1,double f2=1, double f3=1);


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
				char* title = "Show2DArray",
				bool missZeros = true,
				double f1 = 1,double f2=1);


/** Show 1D array in a new viewer.

	@param pData   input 1D array
	@param s1   sizes of array
	@param title   title of final window
	@param missZeros   if true skip zeros in data
	@param f1   scaling factors for x.
	
	@return void (just show the result in a new window)
*/	

template<class T>
void Show1DArray(T* pData,
				int s1,
				char* title = "Show1DArray",
				bool missZeros=true,
				double f1=1);


/** Calculates and shows distances from a given plane.

	@param laserPoints   points to calculate distances for.
	@param normal   normal of the plane.
	@param distance   rho of the plane
	
	@return none
*/	
void 
ShowDistanceFromPlane(const LaserPoints& laserPoints,
					Vector3D normalDirection,
					double distance);

/** Shows laser points in viewer. 
	Currently uses vrml view with a randomly generated file name.
	
	@return none
*/	
void
ShowLaserPoints(
			const LaserPoints& laserPoints,
			const char* title = NULL,
			GLColorScheme glColorSchem = GLColorSchemeStandard,
			bool bUseReflectance = true);
void
ShowLaserPoints(
		const LaserPoints& laserPoints,
		const LaserPoints& normalPoints,
		const char* title = NULL,
		const double normalLength = 0,
        const Color3D beginColor = Color3D(0,1,0),
        const Color3D endColor = Color3D(1,0,0),
		GLColorScheme color = GLColorSchemeStandard,
		bool useReflectance = true);
		
void
ShowSegmentation(
		const LaserPoints& laserPoints,
		const vector<vector<int> >& segments,
		const char* title = NULL,
		bool useReflectance=false);

		

/** Shows vrml file in Vrmlview.
	
	@param vrmlFileName   name of the file to show
	
	@return none
*/	
void ShowInVrmlview(const char* vrmlFileName);

/** Show given points in a new laser viewer
*/
void 
ShowInViewer(const LaserPoints& pts,char* title=NULL);

	


#endif // _VISUALIZATION_UTILITY_H_
