
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
*   Purpose   : Provides utility functions for vrml file creation
*
*--------------------------------------------------------------------*/
#ifndef _VRML_UTILITY_H_
#define _VRML_UTILITY_H_

#include "LaserPoints.h"
#include "LaserPointsProcessing.h"
#include "LaserPointsUtility.h"
#include "MyConstants.h"
#include "MyTypeDefinitions.h"
#include "Palettes.h"
#include "GLObject.h"
#include "VisualizationUtility.h"
#include "GLIndexedLineSet.h"
#include <iostream>
#include <string>

using std::string;

class VrmlFile
{
public:	
	///Constructor. We don't need a default one.
	VrmlFile(const char* fileName);
	
	///Destructor. Closes the file.
	~VrmlFile();
	
	/** Writes the laser points as a point set to vrml file.
		@param pts   laser points to write
		@param glColorScheme   color scheme to use
		@param bUseReflectance   if true use Reflectance else use z.
		
		@return 0 on success 
	*/
	int
	Write(const LaserPoints &pts,
			GLColorScheme glColorScheme = GLColorSchemeStandard,
			bool bUseReflectance = true);
			
	/** Writes SegmentsVector to vrml file.
	
	@param lp   laser points 
	@param seg	segments vector to be written.
	@param useReflectance   if true use reflectance else use z
	
	@return 0 on success
	*/	
	int 
	Write(const LaserPoints& laserPoints,
			const SegmentsVector& segmentsVector,
			bool bUseReflectance=true );
			
	/*Write axes 3d indicator */			
			
	void WriteAxes3D(Vector3D trans=Vector3D(0,0,0),
			Vector3D scale = Vector3D(1,1,1),
			Vector3D rotAxis = Vector3D(1,0,0),
			double rotAngle=0);		
				
	///Return FILE* to opened file.
	FILE* GetFile();
	
	///closes file first and then shows in vrmlview
	void Show();
	
	///Closes the file
	void Close();
	
	///Checks for validiy of pFile.
	bool IsValid();
	
private:
	FILE* pFile;	
	char fileName[1024];
	
};	

void Axes3D2Vrml(FILE* pFile,Vector3D trans=Vector3D(0,0,0),
			Vector3D scale = Vector3D(1,1,1),
			Vector3D rotAxis = Vector3D(1,0,0),
			double rotAngle=0);

/** Creates a new vrml file and puts a valid header if creation succeeds.

	@param fileName   name of file to create.
	@return FILE* to opened file, may be null.
*/
FILE*
OpenVrmlFile(const char* fileName);

/** Closes a vrml file.

	@param FILE* points to the file to be closed
	@return none.
*/
void
CloseVrmlFile(FILE* pFile);

/** Writes laser points with given colors to given vrml file.
	
	@param lp   laser points to be written
	@param colors    color of each point.
	
	@return none
*/	
void LaserPoints2Vrml(FILE* pFile,
			const LaserPoints& lp,
			vector<Color3D> colors);


/** Writes laser points to given vrml file.
	
	@parame file   pointer to FILE must be valid
	@param lp   laser points to write
	@param glColorScheme   palette to use
	@param useReflectance   if true use reflectance else use z
	
	@return none
*/	
void LaserPoints2Vrml(FILE* pFile,
			const LaserPoints& lp,
			GLColorScheme glColorScheme = GLColorSchemeScaledGray,
			bool bUseReflectance = true);
			
/** Writes SegmentsVector to vrml file.
	
	@param file   pointer to FILE must be valid
	@param lp   laser points 
	@param seg	segments vector to be written.
	@param useReflectance   if true use reflectance else use z
	
	@return none
*/	
void SegmentsVector2Vrml(FILE* pFile,
			const LaserPoints& laserPoints,
			const SegmentsVector& segmentsVector,
			bool bUseReflectance =true);
			
/** Writes material with given colors to vrml file.
	
	@return none
*/	
void 
Material2Vrml(FILE* pFile,
			Color3D diffuseColor= Color3D(0.8,0.8,0.8),
			Color3D specualrColor= Color3D(1,1,1),
			double shininess=0.7);
string& 
Material2Vrml(string& s,Color3D diffuseColor= Color3D(0.8,0.8,0.8),Color3D specualrColor= Color3D(1,1,1),double shininess=0.7);
			
			
void 
Material2Vrml(ostream& of,
			Color3D diffuseColor= Color3D(0.8,0.8,0.8),
			Color3D specualrColor= Color3D(1,1,1),
			double shininess=0.7);
			
			
/** Writes laser points normals to vrml file as indexed line set.
	
	@param laserPoints   point cloud
	@param normalPoints  normal points
	@param normalLength  scaleFactor for normal length.
	@param Begin/End Color	color for normal lines.
	
	@return void
*/	
void
LaserPointsNormals2Vrml(
			FILE* pFile,
			const LaserPoints& points,
            const LaserPoints& normals,
            const double normalLength = 0,
            const Color3D beginColor = Color3D(1,0,0),
            const Color3D endColor = Color3D(0,1,0));
	    
/** Writes TIN3D as indexed faceset in Vrml. Uses the specified color scheme and options.

	@return none
*/
/*
void
TIN3D2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN3D & tin3D,
		GLColorScheme glColorScheme = GLColorSchemeScaledGray,
		bool useReflectance = true
		);
*/
		
/** Writes Indexed line set to the given file

	@param pFile file should be valid
	@param cv  CoordinateVector 
	@param iv  IndicesVector, each line is terminated by -1
	@param colorVectors should have same number of entries as cv
	@param color used for all lines if colorVector is not big enough.
	
	@return none.
*/	
void 
IndexedLineSet2Vrml(FILE* pFile,
			const CoordinateVector& cv,
			const IndicesVector& iv,
			const vector<Vector3D>& colorVector=vector<Vector3D>(),
			const Vector3D color=Vector3D(0.8,0.8,0.8));
			
void 
IndexedLineSet2Vrml(FILE* pFile,
			GLIndexedLineSet& ils);
		
			
/** Writes Indexed face set to the given file

	@param pFile file should be valid
	@param cv  CoordinateVector 
	@param iv  IndicesVector, each face is terminated by -1
	@param colorVectors should have same number of entries as cv
	@param color used for all lines if colorVector is not big enough.
	
	@return none.
*/	
void 
IndexedFaceSet2Vrml(FILE* pFile,
			const CoordinateVector& cv,
			const IndicesVector& iv,
			const vector<Vector3D>& colorVector=vector<Vector3D>(),
			const Vector3D color=Vector3D(0.8,0.8,0.8));
			
/** Writes the outline of Indexed face set to the given file,
	as an IndexedLineSet.

	@param pFile file should be valid
	@param cv  CoordinateVector 
	@param iv  IndicesVector, each face is terminated by -1
	@param colorVectors should have same number of entries as cv
	@param color used for all lines if colorVector is not big enough.
	
	@return none.
*/	
void 
IndexedFaceSet2VrmlWireframe(FILE* pFile,
			const CoordinateVector& cv,
			const IndicesVector& iv,
			const vector<Vector3D>& colorVector=vector<Vector3D>(),
			const Vector3D color=Vector3D(0.8,0.8,0.8));
			
			
			
			
/** Write LaserPoints TIN as indexed faceset in Vrml. Uses the specified color scheme and options.
Derives a TIN internally.
*/
void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		GLColorScheme glColorScheme = GLColorSchemeScaledGray,
		bool useReflectance = true
		);
		
/** Write LaserPoints TIN as indexed faceset in Vrml. Uses the specified color scheme and options.
Assumes tin to be derived already.
*/
void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN& tin,
		GLColorScheme glColorScheme = GLColorSchemeScaledGray,
		bool useReflectance = true
		);

//Use one color for writing.
void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color = Vector3D(1,1,1));	
		
string&
LaserPointsTIN2Vrml(
		string& s,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color = Vector3D(1,1,1));	
		
void
LaserPointsTIN2Vrml(
		ofstream& file,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color);				
		
			
		
/** Write an arrow. If asked also write prototype defintion else assume
prototype to be there.
*/
void
Arrow2Vrml(
		FILE* pFile,
		bool writeProto = true,
		Vector3D direction = Vector3D(1,1,1),
		Vector3D position = Vector3D(),
		double size = 1,
		Color3D color = Color3D(1,1,0));
		
		
/*
Write an arrow from a given starting position to a given ending position
*/
void
Vector2Vrml(
		FILE* pFile,
		bool writeProto,
		Vector3D begin,
		Vector3D end,
		double radius,
		Color3D color);
		
/* Write an image to a vrml file.
Just make a rectangle equal to image width and height, and texture map it
*/
void
Image2Vrml(
	FILE* pFile,
	const char* imageFileName);
	
			
/*
write a sphere at a given position with given radius and color
*/
void Sphere2Vrml(
			FILE* pFile,
			Vector3D pos,
			double radius,
			Color3D color = Color3D(0.8,0.8,0.8),
			bool writeProto = true);

/*
	Write a given cylinder to vrml file
*/			
void Cylinder2Vrml(
			FILE* pFile,
			Vector3D point1,
			Vector3D point2,
			double radius = 10,
			Color3D color = Color3D(0.8,0.8,0.8),
			bool writeProto=true);
			
//Another version that calculates the point1 and point2 from the given laser points by projection along axis.
void Cylinder2Vrml(FILE* pFile,
				const LaserPoints& pts,
				Vector3D axis,
				Vector3D position,
				double radius=10,
				Color3D color = Color3D(0.8,0.8,0.8),
				bool writeProto=true);			
/* 
Represent each laser point by a separate sphere of a given color,
doesn't use reflectance value
*/
void LaserPoints2SphereVrml(FILE* pFile,
				const LaserPoints& laserPoints,
				double radius,
				Color3D color);
				
/* write contents of a string to vrml file and show is requested */
void ToVrmlFile(string& vrml,string fileName="",bool show=false);

/* Write a view point to vrml file. */
void Viewpoint2Vrml(FILE* pFile,
					Vector3D position=Vector3D(0,0,1000),
					Vector3D rotAxis=Vector3D(1,0,0),
					double angleInRadians=0,
					string name = "",
					double fov = 0.8,
					bool jump = true);
					
#endif //_VRML_UTILITY_H_		
	    
			
			
			

