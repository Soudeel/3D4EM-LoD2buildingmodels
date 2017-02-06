
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "LaserPointsUtility.h"
#include "LaserPointsFitting.h"
#include "GeneralUtility.h"
#include "VisualizationUtility.h"

#include "VrmlUtility.h"

/** Creates a new vrml file and puts a valid header if creation succeeds.

	@param fileName   name of file to create.
	@return FILE* to opened file, may be null.
*/
FILE*
OpenVrmlFile(const char* fileName)
{
	
	FILE* pFile = fopen(fileName,"wt");
		
	if(pFile)
	{
		fprintf(pFile,"#VRML V2.0 utf8\n");

		time_t currentTime;
		time ( &currentTime );
		fprintf(pFile,"\n# %s created on %s\n\n",fileName,ctime(&currentTime));
	}
	
	//return with FILE*
	return pFile;
}

/** Closes a vrml file.

	@param FILE* points to the file to be closed
	@return none.
*/
void
CloseVrmlFile(FILE* pFile)
{
	if(pFile)
	{
		fclose(pFile);
	}
}

/** Writes laser points with given colors to given vrml file.
	
	@param lp   laser points to be written
	@param colors    color of each point.
	
	@return none
*/	
void LaserPoints2Vrml(FILE* pFile,
			const LaserPoints& lp,
			vector<Color3D> colors)
{
	if(!pFile)
		return;
		
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving LaserPoints as PointSet\n");
	fprintf(pFile,"\n");
//Turn only if scaling of data is required 
//fprintf(pFile,"Transform{\nscale 1000 1000 1000 \nchildren[\n");
	fprintf(pFile,"Shape {\n geometry PointSet {\ncoord Coordinate { point [ \n");

	//write points.
	for(int i=0;i<lp.size();i++)
	{
		fprintf(pFile," %f %f %f,  ",lp[i].X(),lp[i].Y(),lp[i].Z());
	}
	fprintf(pFile,"\n ] }\n");
	
	fprintf(pFile,"color Color { color [\n");
	
	for (int i = 0; i < colors.size (); i++)
	{
		fprintf(pFile," %f %f %f , ",colors[i].X(),colors[i].Y(),colors[i].Z());
	}
	fprintf(pFile,"\n] }");
	
	fprintf(pFile,"}\n }\n");
//	fprintf(pFile,"]}\n");

}						



/** Writes laser points to given vrml file.
	
	@param file   pointer to FILE must be valid
	@param lp   laser points to write
	@param glColorScheme   palette to use
	@param useReflectance   if true use reflectance else use z
	
	@return none
*/	
void LaserPoints2Vrml(FILE* pFile,
			const LaserPoints& lp,
			GLColorScheme glColorScheme,
			bool bUseReflectance)
{
	if(!pFile)
		return;
		
	//Get colors.
	vector<Color3D> colors = ReflectanceToColor3D(lp,glColorScheme,bUseReflectance);
	
	return LaserPoints2Vrml(pFile,lp,colors);
}


void Axes3D2Vrml(FILE* pFile,Vector3D trans,Vector3D scale, Vector3D rotAxis, double rotAngle)
{
//return a string for 3D axis indicator. translation and rotation are done
//first and then after applying the scale. As would be expected.
	if(!pFile)
		return;
	
	fprintf(pFile,"#VRML V2.0 utf8\n");
	scale = scale*0.001;

	fprintf(pFile,"Transform\n{scale %f %f %f translation %f %f %f rotation %f %f %f %f children[",scale[0],scale[1],scale[2],trans[0],trans[1],trans[2],rotAxis[0],rotAxis[1],rotAxis[2],rotAngle*M_PI/180.00);
	fprintf(pFile,"\n#put a sphere to show origin.\nTransform\n{\ttranslation 0 0 0\n        children[\n");
	fprintf(pFile,"Shape{\nappearance DEF ORIGIN_APPEARANCE Appearance { material Material {diffuseColor 0.8 0.8 0.8 specularColor 0.3 0.3 0.3 shininess 1 }} geometry Sphere { radius 100 }}]}\n");
	fprintf(pFile,"\n#put a cylinder to show axis.\n#Y-Axis\nTransform{translation 0 400 0 children[Shape{appearance DEF Y_APPEARANCE Appearance { material Material {diffuseColor 0 1 0  specularColor 0.3 0.3 0.3 shininess 1 }} geometry DEF ARROW_CYL Cylinder {radius 60 height 800} } Transform {  translation 0 500 0 children[Shape{appearance USE Y_APPEARANCE  geometry DEF ARROW_CONE Cone{bottomRadius 100 height 200}}]}]}Transform{rotation 0 0 1 -1.5708 children[Transform{translation 0 400 0 children[Shape{appearance DEF X_APPEARANCE Appearance { material Material {diffuseColor 1 0 0 specularColor 0.3 0.3 0.3 }} geometry USE ARROW_CYL }Transform {translation 0 500 0 children[Shape{appearance USE X_APPEARANCE geometry USE ARROW_CONE }]} ]}]}\n");
	fprintf(pFile,"#Z-Axis\nTransform{rotation 1 0 0 1.5708 children[Transform{translation 0 400 0children[Shape{appearance DEF Z_APPEARANCE Appearance { material Material {diffuseColor 0 0 1 specularColor 0.3 0.3 0.3}} geometry USE ARROW_CYL }Transform{translation 0 500 0 children[Shape{appearance USE Z_APPEARANCE geometry USE ARROW_CONE }]}]}]}]}\n");
	
}



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
			bool bUseReflectance )
{			
	//local variables.
	LaserPoints segPoints;
	vector<Color3D> segColors;
	double minRef,maxRef,scaleFactor;
	
	
	if(bUseReflectance)
	{
		scaleFactor = laserPoints.ReflectanceRange(&minRef,&maxRef);
		scaleFactor = 1.00/std::max(scaleFactor,1e-12);
	}

	segColors.reserve(laserPoints.size());
	segPoints.reserve(laserPoints.size());
	for(int i=0;i<segmentsVector.size();i++)
	{
		const IndicesVector& iv = segmentsVector[i];
		Vector3D color = GenerateRandomColor();
		
		for(int j=0;j<iv.size();j++)
		{
			segPoints.push_back(laserPoints[iv[j]]);
			if(bUseReflectance)
			{
				segColors.push_back(color*(laserPoints[iv[j]].Reflectance()-minRef)*scaleFactor);
			}
			else
				segColors.push_back(color);
		}
	}
	LaserPoints2Vrml(pFile,segPoints,segColors);
	
}

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
            const double normalLengthIn,
            const Color3D beginColor,
            const Color3D endColor)
{
	int i,j,k;
	double normalLength = normalLengthIn;
	
    //Put vrml header.
	fprintf(pFile,"#Saving laser point normals as indexed line set\n");
	
	if(normalLength<=0)
	{
		normalLength = 0.1* Min(points.Extents());
	}

	//Save normals as indexed line-sets.
    fprintf(pFile,"Shape {\n");
    fprintf(pFile,"\tgeometry IndexedLineSet{\n");
    fprintf(pFile,"coord Coordinate{ \n");

    //Write coordinates of lines.
    fprintf(pFile,"point [ \n");

    //Write start points of lines.
    for(i=0;i<points.size();i++)
    {
    	fprintf(pFile,"%f %f %f , \n",points[i].X(),points[i].Y(),points[i].Z());
    }

	//Write end points of lines.
    for(i=0;i<points.size();i++)
    {
	    fprintf(pFile,"%f %f %f , \n",points[i].X()+normals[i].X()*normalLength,
        							points[i].Y()+normals[i].Y()*normalLength,
                                    points[i].Z()+normals[i].Z()*normalLength);
    }
    fprintf(pFile,"]\n");
    fprintf(pFile,"} \n");

    //Write line coordinate indices.
    fprintf(pFile,"coordIndex [ \n");
    for(i=0;i<points.size();i++)
    {
    	fprintf(pFile,"%d %d %d , \n",i,i+points.size(),-1);
    }
    fprintf(pFile,"]\n");

    //Write colors for normals.
    fprintf(pFile,"\tcolor Color {\n");
    fprintf(pFile,"\tcolor [\n");

    //Write normal start color.
    for(i=0;i<points.size();i++)
    {
	    fprintf(pFile,"%f %f %f , \n",beginColor.X(),beginColor.Y(),beginColor.Z());
    }

    //Write normal end color.
    for(i=0;i<points.size();i++)
    {
	    fprintf(pFile,"%f %f %f , \n",endColor.X(),endColor.Y(),endColor.Z());
    }
    fprintf(pFile,"]\n}\n");

    //We want color to be interpolated for each normal.
    fprintf(pFile,"colorPerVertex TRUE\n");

    fprintf(pFile,"}\n");
    fprintf(pFile,"}\n");
}



/** Writes material with given colors to vrml file.
	
	@return none
*/	
void 
Material2Vrml(FILE* pFile,Color3D diffuseColor,Color3D specualrColor,double shininess)
{
	string s;
	Material2Vrml(s,diffuseColor,specualrColor,shininess);
	fprintf(pFile,s.c_str());
}

string& 
Material2Vrml(string& s,Color3D diffuseColor,Color3D specualrColor,double shininess)
{
	sprintfa(s, "\t appearance Appearance{\n");
	sprintfa(s,"\t\tmaterial Material{\n");
	sprintfa(s,"\t\t\t diffuseColor %f   %f  %f\n",diffuseColor.X(),diffuseColor.Y(),diffuseColor.Z());
	sprintfa(s,"\t\t\t specularColor %f  %f  %f\n",specualrColor.X(),specualrColor.Y(),specualrColor.Z());
	sprintfa(s,"\t\t\t shininess  %f\n",shininess);
	sprintfa(s,"\t\t}\n");
	sprintfa(s,"\t}\n");
	return s;
}


/** Writes material with given colors to vrml file.
	
	@return none
*/
void 
Material2Vrml(ostream& file,Color3D diffuseColor,Color3D specualrColor,double shininess)
{
	file<< "\t appearance Appearance{\n";
	file<<"\t\tmaterial Material{\n";
	file<<"\t\t\t diffuseColor  "<<diffuseColor.X()<<"  "<<diffuseColor.Y()<<"  "<<diffuseColor.Z()<<endl;
	file<<"\t\t\t specularColor  "<<specualrColor.X()<<"  "<<specualrColor.Y()<<"  "<<specualrColor.Z()<<endl;
	file<<"\t\t\t shininess  "<<shininess<<endl;
	file<<"\t\t}\n";
	file<<"\t}\n";
}
	


/** Write TIN3D as indexed faceset in Vrml. Uses the specified color scheme and options.
*/
void
TIN3D2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN3D & tin3D,
		GLColorScheme glColorScheme,
		bool useReflectance
		)
{
	if (!(laserPoints.size ()) || !(tin3D.size ())||!pFile)
		return ;
		
	Vector3D v1, v2, v3;
	int i,j;
	
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	//Get indices.
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving TIN3D as IndexedFaceSet\n");
	fprintf(pFile,"\n");
	fprintf(pFile,"Shape {\n");
	
	Material2Vrml(pFile);
  	
	fprintf(pFile,"geometry IndexedFaceSet {\n");
	fprintf(pFile,"creaseAngle 0.5\n");
    fprintf(pFile,"coord Coordinate {\n");
   	fprintf(pFile,"point [\n");
	
	for(int i=0;i<laserPoints.size();i++)
		fprintf(pFile,"%f %f %f, ",laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z());
	
    

	//Now start the indices
	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
    fprintf(pFile,"coordIndex [ \n");
	for (int i = 0; i < tin3D.size (); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			LaserPoint currentPoint = laserPoints[tin3D[i][j]];
			
			fprintf(pFile,"%d  ",tin3D[i][j]);
			
			BYTE index = indices[i];
			//glColor3ub(paletteData[index][0],paletteData[index][1],paletteData[index][2]);		
			//glVertex3f (currentPoint.X (), currentPoint.Y (),currentPoint.Z ());
		}	
		fprintf(pFile,"%d  ",-1);		
	}
	fprintf(pFile,"\n]\n");
	fprintf(pFile,"}\n");
	fprintf(pFile,"}\n");
}

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
			const vector<Vector3D>& colorVector,
			const Vector3D color)
{
	if(!pFile)
		return;
		
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving IndexedLineSet\n");
	fprintf(pFile,"\n");
	fprintf(pFile,"Shape {\n");
	fprintf(pFile,"appearance Appearance{material Material{\n");
	fprintf(pFile,"diffuseColor	%f %f %f \n",color.X(),color.Y(),color.Z());
	fprintf(pFile,"shininess         0.9\n");
	fprintf(pFile,"specularColor     1 1 1\n");
	fprintf(pFile,"}}\n");
  	fprintf(pFile,"geometry IndexedLineSet {\n");
    fprintf(pFile,"coord Coordinate {\n");
   	fprintf(pFile,"point [\n");
	
	for(int i=0;i<cv.size();i++)
		fprintf(pFile,"%f %f %f, ",cv[i].X(),cv[i].Y(),cv[i].Z());
	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
    fprintf(pFile,"coordIndex [ \n");
    for(int i=0;i<iv.size();i++)
    	fprintf(pFile,"%d  ",iv[i]);
	fprintf(pFile,"\n]\n");
	
	if(colorVector.size()>=cv.size())
	{
		fprintf(pFile,"color Color{\n");
		fprintf(pFile,"color[\n");
		
		for(int i=0;i<colorVector.size();i++)
		{
			fprintf(pFile,"%f %f %f, ",colorVector[i].X(),colorVector[i].Y(),colorVector[i].Z());
		}
		fprintf(pFile,"\n");
		fprintf(pFile,"]\n");
   		fprintf(pFile,"}\n");
    	
		/*
		fprintf(pFile,"colorIndex [ \n");
    	for(int i=0;i<colorVector.size();i++)
	    	fprintf(pFile,"%d  ",i);
		fprintf(pFile,"-1\n]\n");
		*/
	}
	else
	{
		cerr<<"cannot use color per vertex in IndexedLineSet"
			<<" coordinate vector: "<<cv.size()
			<<" colorVector: "<<colorVector.size()<<endl;
	}
	
	fprintf(pFile,"}\n");
	fprintf(pFile,"}\n");
}

void 
IndexedLineSet2Vrml(FILE* pFile,
			GLIndexedLineSet& ils)
{
	return 		
	IndexedLineSet2Vrml(pFile,
				ils.GetCoordinateVector(),
				ils.GetIndicesVector(),
				ils.GetColorsVector());
}	

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
			const vector<Vector3D>& colorVector,
			const Vector3D color)
{
	if(!pFile)
		return;
		
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving IndexedFaceSet\n");
	fprintf(pFile,"\n");
	fprintf(pFile,"Shape {\n");
	fprintf(pFile,"appearance Appearance{material Material{\n");
	fprintf(pFile,"diffuseColor	%f %f %f \n",color.X(),color.Y(),color.Z());
	fprintf(pFile,"shininess         0.9\n");
	fprintf(pFile,"specularColor     1 1 1\n");
	fprintf(pFile,"}}\n");
  	fprintf(pFile,"geometry IndexedFaceSet {\n");
	fprintf(pFile,"creaseAngle 0.5\n");
    fprintf(pFile,"coord Coordinate {\n");
   	fprintf(pFile,"point [\n");
	
	for(int i=0;i<cv.size();i++)
		fprintf(pFile,"%f %f %f, ",cv[i].X(),cv[i].Y(),cv[i].Z());
	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
    fprintf(pFile,"coordIndex [ \n");
    for(int i=0;i<iv.size();i++)
    	fprintf(pFile,"%d  ",iv[i]);
	fprintf(pFile,"\n]\n");
	
	if(colorVector.size()==cv.size())
	{
		fprintf(pFile,"color Color{\n");
		fprintf(pFile,"color[\n");
		
		for(int i=0;i<colorVector.size();i++)
		{
			fprintf(pFile,"%f %f %f, ",colorVector[i].X(),colorVector[i].Y(),colorVector[i].Z());
		}
		fprintf(pFile,"\n");
		fprintf(pFile,"]\n");
   		fprintf(pFile,"}\n");
	}

	fprintf(pFile,"}\n");
	fprintf(pFile,"}\n");
}



void 
IndexedFaceSet2VrmlWireframe(FILE* pFile,
			const CoordinateVector& cv,
			const IndicesVector& iv,
			const vector<Vector3D>& colorVector,
			const Vector3D color)
{
	if(!pFile)
		return;
		
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving the outline of IndexedFaceSet\n");
	fprintf(pFile,"\n");
	fprintf(pFile,"Shape {\n");
	fprintf(pFile,"appearance Appearance{material Material{\n");
	fprintf(pFile,"diffuseColor	%f %f %f \n",color.X(),color.Y(),color.Z());
	fprintf(pFile,"shininess         0.9\n");
	fprintf(pFile,"specularColor     1 1 1\n");
	fprintf(pFile,"}}\n");
  	fprintf(pFile,"geometry IndexedLineSet {\n");
    fprintf(pFile,"coord Coordinate {\n");
   	fprintf(pFile,"point [\n");
	
	for(int i=0;i<cv.size();i++)
		fprintf(pFile,"%f %f %f, ",cv[i].X(),cv[i].Y(),cv[i].Z());
	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
    fprintf(pFile,"coordIndex [ \n");
    for(int i=0;i<iv.size();i++)
    	fprintf(pFile,"%d  ",iv[i]);
	fprintf(pFile,"\n]\n");
	
	if(colorVector.size()==cv.size())
	{
		fprintf(pFile,"color Color{\n");
		fprintf(pFile,"color[\n");
		
		for(int i=0;i<colorVector.size();i++)
		{
			fprintf(pFile,"%f %f %f, ",colorVector[i].X(),colorVector[i].Y(),colorVector[i].Z());
		}
		fprintf(pFile,"\n");
		fprintf(pFile,"]\n");
   		fprintf(pFile,"}\n");
	}

	fprintf(pFile,"}\n");
	fprintf(pFile,"}\n");
}



///Constructor. We don't need a default one.
VrmlFile::VrmlFile(const char* fn)
{
	strcpy(fileName,fn);
	pFile = OpenVrmlFile(fileName);
}

///Destructor. Closes the file.
VrmlFile::~VrmlFile()
{
	if(IsValid())
	{
		CloseVrmlFile(pFile);
		pFile = NULL;
	}
}

/** Writes the laser points as a point set to vrml file.
	@param pts   laser points to write
	@param glColorScheme   color scheme to use
	@param bUseReflectance   if true use Reflectance else use z.
	
	@return 0 on success 
*/
int
VrmlFile::Write(const LaserPoints &pts,GLColorScheme glColorScheme,bool bUseReflectance)
{
	if(IsValid())
	{
		LaserPoints2Vrml(pFile,pts,glColorScheme,bUseReflectance);
		return 0;
	}
	else
		return 1;
}
int 
VrmlFile::Write(const LaserPoints& laserPoints,const SegmentsVector& segmentsVector,bool bUseReflectance )

{
	if(IsValid())
	{
		SegmentsVector2Vrml(pFile,laserPoints,segmentsVector,bUseReflectance);
		return 0;
	}
	else
		return 1;
}
			
		
///Return FILE* to opened file.
FILE* 
VrmlFile::GetFile()
{
	return pFile;
}

///closes file first and then shows in vrmlview
void
VrmlFile::Show()
{
	if(IsValid())
	{
		Close();
		ShowInVrmlview(fileName);
	}
}

///Closes the file
void
VrmlFile::Close()
{
	if(IsValid())
	{
		fclose(pFile);
		pFile = NULL;
	}
}

///Checks for validiy of pFile.
bool
VrmlFile::IsValid()
{
	return (pFile!=NULL);
}


/** Write LaserPoints TIN as indexed faceset in Vrml. Uses the specified color scheme and options.
*/
void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		GLColorScheme glColorScheme,
		bool useReflectance
		)
{
	if (!(laserPoints.size ()) ||!pFile)
		return ;
		
	//Derieve TIN and get a ref to it.
	((LaserPoints&)laserPoints).DeriveTIN();
	TIN& tinRef = ((LaserPoints&)laserPoints).TINReference();

	return
	LaserPointsTIN2Vrml(pFile,laserPoints,tinRef,glColorScheme,useReflectance);
}

/** Write LaserPoints TIN as indexed faceset in Vrml. Uses the specified color scheme and options.
Assumes tin to be derived already.
*/
void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN& tin,
		GLColorScheme glColorScheme,
		bool useReflectance
		)
{
	if (!(laserPoints.size ()) ||!pFile)
		return ;
	
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#Saving LaserPoints TIN as IndexedFaceSet\n");
	fprintf(pFile,"\n");
	fprintf(pFile,"Shape {\n");
	
	Material2Vrml(pFile);
  	
	fprintf(pFile,"geometry IndexedFaceSet {\n");
	fprintf(pFile,"creaseAngle 0.5\n");
    fprintf(pFile,"coord Coordinate {\n");
   	fprintf(pFile,"point [\n");
	
	for(int i=0;i<laserPoints.size();i++)
		fprintf(pFile,"%f %f %f, ",laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z());
	
    

	//Now start the indices
	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
    fprintf(pFile,"coordIndex [ \n");
    
    for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		const PointNumber *points = tin[i].Nodes ();
		
		for (int j = 0; j < 3; j++)
		{
			fprintf(pFile,"%d  ",points[j].Number());
		}	
		fprintf(pFile,"%d  ",-1);		
	}

	fprintf(pFile,"\n]\n");
	
		
	//Make a palette.
	BYTE paletteData[256][3];
	LoadSelectedPalette (paletteData, 256, glColorScheme);
	
	//Get indices.
	vector<int> indices = ReflectanceToIndices(laserPoints,useReflectance,0,255);
	double factor = 1.00/255.00;
	
	//Write colors.
	fprintf(pFile,"color Color {\n");
   	fprintf(pFile,"color [\n");

	for(int j=0;j<indices.size();j++)
	{
		int i = indices[j];
		fprintf(pFile,"%f %f %f, ",paletteData[i][0]*factor,paletteData[i][1]*factor,paletteData[i][2]*factor);
	}

	fprintf(pFile,"\n");
	fprintf(pFile,"]\n");
   	fprintf(pFile,"}\n");
	
		//End indexed faceSet and shape.
	fprintf(pFile,"}\n");
	fprintf(pFile,"}\n");
}

void
LaserPointsTIN2Vrml(
		FILE* pFile,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color)
{
	string s;
	fprintf(pFile,LaserPointsTIN2Vrml(s,laserPoints,tin,color));
}

void
LaserPointsTIN2Vrml(
		ofstream& file,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color)
{
	string s;
	file<<LaserPointsTIN2Vrml(s,laserPoints,tin,color);
}
		
string&
LaserPointsTIN2Vrml(
		string& s,
		const LaserPoints & laserPoints,
		const TIN& tin,
		Vector3D color )		
{
	if (!laserPoints.size ())
		return s;
	
	s.reserve(1024*1024);
	sprintfa(s,"#VRML V2.0 utf8\n");
	sprintfa(s,"#Saving LaserPoints TIN as IndexedFaceSet\n");
	sprintfa(s,"\n");
	sprintfa(s,"Shape {\n");
	
	Material2Vrml(s);
  	
	sprintfa(s,"geometry IndexedFaceSet {\n");
	sprintfa(s,"creaseAngle 0.5\n");
    sprintfa(s,"coord Coordinate {\n");
   	sprintfa(s,"point [\n");
	
	for(int i=0;i<laserPoints.size();i++)
		sprintfa(s,"%f %f %f, ",laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z());
	
    

	//Now start the indices
	sprintfa(s,"\n");
	sprintfa(s,"]\n");
   	sprintfa(s,"}\n");
    sprintfa(s,"coordIndex [ \n");
    
    for (int i = 0; i < tin.size (); i++)
	{
		TINMesh currentMesh = tin[i];
		const PointNumber *points = tin[i].Nodes ();
		
		if(points)
		{
			for (int j = 0; j < 3; j++)
			{
				sprintfa(s,"%d  ",points[j].Number());
			}	
			sprintfa(s,"%d  ",-1);
		}		
	}

	sprintfa(s,"\n]\n");
	
		
	//Write colors.
	sprintfa(s,"color Color {\n");
   	sprintfa(s,"color [\n");

	for(int j=0;j<laserPoints.size();j++)
	{
		sprintfa(s,"%f %f %f, ",color.X(),color.Y(),color.Z());
	}

	sprintfa(s,"\n");
	sprintfa(s,"]\n");
   	sprintfa(s,"}\n");
	
		//End indexed faceSet and shape.
	sprintfa(s,"}\n");
	sprintfa(s,"}\n");
	
	return s;
}	


		

void
WriteArrowProto(FILE* pFile)
{
	static char szArrowPrototype[]="#Prototype for Arrow\n"
"PROTO Arrow\n"
"[\n"
"	field        SFVec3f  translation 0 0 0               # position of the arrow\n"
"	field        SFRotation  rotation 0 0 1 0          # rotation of the arrow\n"
"	field 		 SFNode		appearance NULL\n"
"	field		SFVec3f 	scale 1 1 1\n"
"] \n"
"{"
"Transform{"
"	rotation IS rotation"
"	translation IS translation"
"	scale IS scale"
"	children["
"		Transform{"
"			rotation 1 0 0 1.5708"
"			children["
"				Transform"
"				{"
"					translation 0 1 0"
"					children["
"						Shape{"
"								appearance IS appearance "
"								geometry DEF ARROW_CYL Cylinder {radius 0.15 height 2}"
"							}"
"						Transform"
"						{"
"								translation 0 1.3 0"
"								children["
"								Shape{"
"									appearance IS appearance"
"									geometry DEF ARROW_CONE Cone{bottomRadius 0.3 height 0.6}"
"									}"
"								]"
"							}"
"					]"
"				}"
"			]"
"			}"
"			]"
"	}"
"}\n";

	fprintf(pFile,"%s",szArrowPrototype);
}


void WriteColoredAppearanceProto(FILE* pFile)
{

	static char szColoredAppearanceProto[]="#colored appearance proto\n"
"PROTO ColoredAppearance \n"
"[ \n"
"	field SFColor color 1 1 1 \n"
"	field SFColor emColor 0 0 0\n"
"] \n"
"{ "
"	Appearance { "
"				material Material{ "
"					ambientIntensity  0.4  "
"					diffuseColor  IS color "
"					shininess         0.6     "    
"					specularColor     1 1 1  "
"					transparency      0  "
"					emissiveColor IS emColor"     		
"				}  "
"	} 	"
"}		"
"\n";

	fprintf(pFile,"%s",szColoredAppearanceProto);
}



void
Arrow2Vrml(
		FILE* pFile,
		bool writeProto,
		Vector3D direction,
		Vector3D position,
		double size,
		Color3D color)
{
	if (!pFile)
		return ;
		
	//Get the rotation for given direction.
	Vector3D nc(0,0,1);
	Vector3D w = direction.Normalize();
	double angle = Angle(nc,w);
	Vector3D v = nc.VectorProduct(w);
	if(v.Length())
		v = v.Normalize();
	else
	{
		v = nc;
		angle = 0;
	}
	
	fprintf(pFile,"#VRML V2.0 utf8\n");
	
	if(writeProto)
	{
		WriteArrowProto(pFile);
		WriteColoredAppearanceProto(pFile);
	}
		

	fprintf(pFile,"\n");
	double emissiveFactor = 0.04;
	fprintf(pFile,"Arrow{appearance ColoredAppearance{ color %f %f %f emColor %f %f %f} scale %f %f %f translation %f %f %f rotation %f %f %f %f}\n",
									color.X(),color.Y(),color.Z(),
									color.X()*emissiveFactor,color.Y()*emissiveFactor,color.Z()*emissiveFactor,
									size,size,size,
									position.X(),position.Y(),position.Z(),
									v.X(),v.Y(),v.Z(),angle);
}

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
		Color3D color)
{
	if (!pFile)
		return ;
		
	//Get the rotation for given direction.
	Vector3D nc(0,0,1);
	Vector3D direction = end - begin;
	
	if(direction.Length()==0)
		return;
	
	Vector3D w = direction.Normalize();
	double angle = Angle(nc,w);
	Vector3D v = nc.VectorProduct(w);
	if(v.Length())
		v = v.Normalize();
	else
	{
		if(w.Z()<0)
		{
			//Its negative z-axis, we have to reflect along y-axis.
			v = Vector3D(0,1,0);
			angle = M_PI;
		}
		else
		{
			//Its positive z-axis, we don't need any rotation at all.
			v = nc;
			angle = 0;
		}
	}
	
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#drawing vector from (%f,%f,%f)->(%f,%f,%f)\n",begin.X(),begin.Y(),begin.Z(),end.X(),end.Y(),end.Z());
	
	if(writeProto)
	{
		WriteArrowProto(pFile);
		WriteColoredAppearanceProto(pFile);
	}
	
	char szVectorCylinder[] = "Transform{"
		"	rotation %f %f %f %f"
		"	translation %f %f %f "
		"	children["
		"		Transform{"
		"			rotation 1 0 0 1.5708"
		"			children["
		"				Transform"
		"				{"
		"					translation 0 %f 0"
		"					children["
		"						Shape{"
		"								appearance %s "
		"								geometry Cylinder {radius %f height %f}"
		"							}"
		"						Transform"
		"						{"
		"								translation 0 %f 0"
		"								children["
		"								Shape{"
		"									appearance %s"
		"									geometry Cone{bottomRadius %f height %f}"
		"									}"
		"								]"
		"							}"
		"					]"
		"				}"
		"			]"
		"			}"
		"			]"
		"	}\n";
	
		
	
	fprintf(pFile,"\n");
	double emissiveFactor = 0.04;
	double coneSize = 4*radius;
	double length = (end-begin).Length()- coneSize;
	
	
	char szAppearance[2048];
	sprintf(szAppearance,"ColoredAppearance{ color %f %f %f emColor %f %f %f} ",color.X(),color.Y(),color.Z(),
								color.X()*emissiveFactor,color.Y()*emissiveFactor,color.Z()*emissiveFactor);
	char szBuffer[1024*10];
	sprintf(szBuffer,szVectorCylinder,
					v.X(),v.Y(),v.Z(),angle,
					begin.X(),begin.Y(),begin.Z(),
					length*0.5, szAppearance,radius,length,
					length*0.5+0.5*coneSize,szAppearance,radius*2,coneSize);
	fprintf(pFile,"%s\n\n",szBuffer);
		
}

/*
write a sphere at a given position with given radius and color
*/
void Sphere2Vrml(
			FILE* pFile,
			Vector3D pos,
			double radius,
			Color3D color ,
			bool writeProto)
{
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#drawing sphere at (%f,%f,%f)\n",pos.X(),pos.Y(),pos.Z());
	
	if(writeProto)
	{
		WriteColoredAppearanceProto(pFile);
	}
	double emFactor = 0.01;
	fprintf(pFile,"Transform{translation %f %f %f children[Shape {appearance ColoredAppearance"
					"{color %f %f %f emColor %f %f %f} geometry Sphere{radius %f}}]}\n",
					pos.X(),pos.Y(),pos.Z(),
					color.X(),color.Y(),color.Z(),
					emFactor*color.X(),emFactor*color.Y(),emFactor*color.Z(),
					radius);
					
	fprintf(pFile,"\n");					
}

/*
	Write a given cylinder to vrml file
*/
void Cylinder2Vrml(
			FILE* pFile,
			Vector3D point1,
			Vector3D point2,
			double radius,
			Color3D color ,
			bool writeProto)
{
	fprintf(pFile,"#VRML V2.0 utf8\n");
	fprintf(pFile,"#drawing cylinder \n");
	
	if(writeProto)
	{
		WriteColoredAppearanceProto(pFile);
	}
	double emFactor = 0.01;
	double angle;
	
	//Find height of cylinder.
	double height = (point2-point1).Length();

	//Find direction of symmetry axis.
	Vector3D direction = (point2-point1).Normalize();

	//Direction of cylinder given by gluCylinder. (z-axis)
	Vector3D yAxis(0,1,0);

	//Rotation axis is normal to the plane defined by zAxis and direction.
	Vector3D rotAxis = yAxis.VectorProduct(direction);

	if(rotAxis.Length()==0)
	{
		rotAxis = Vector3D(1,0,0);
		if(direction.Y()<0)angle = 180;
		else angle=M_PI;
	}
	else
	{
		//Find the angle between the two vectors.
		angle = Angle(yAxis,direction);		
	}

	fprintf(pFile,"Transform {\n");
	fprintf(pFile," translation %f %f %f  \n",(point1.X()+point2.X())/2 ,(point1.Y()+point2.Y())/2 ,(point1.Z()+point2.Z())/2);
	fprintf(pFile," rotation %f %f %f %f ",rotAxis.X(),rotAxis.Y(),rotAxis.Z(),angle);

	fprintf(pFile,"children [Shape {\n");
	fprintf(pFile,"appearance ColoredAppearance"
			"{color %f %f %f emColor %f %f %f}\n",
					color.X(),color.Y(),color.Z(),
					emFactor*color.X(),emFactor*color.Y(),emFactor*color.Z());
					
	fprintf(pFile, "\t geometry Cylinder {radius %f height %f }}]}",radius,height);
	fprintf(pFile,"\n");					
}

//Another version that calculates the point1 and point2 from the given laser points by projection along axis.
void Cylinder2Vrml(FILE* pFile,
				const LaserPoints& pts,
				Vector3D axis,
				Vector3D position,
				double radius,
				Color3D color,
				bool writeProto)
{
	Vector3D point1, point2;
	LaserCylinder cyl(pts,axis,position,radius);
	
	return Cylinder2Vrml(pFile,cyl.pointBegin,cyl.pointEnd,radius,color,writeProto);	
}

/* 
Represent each laser point by a separate sphere of a given color,
doesn't use reflectance value
*/

void LaserPoints2SphereVrml(FILE* pFile,
				const LaserPoints& laserPoints,
				double radius,
				Color3D color)
{
	for(int i=0;i<laserPoints.size();i++)
		Sphere2Vrml(pFile,Vector3D(laserPoints[i]),radius,color,i==0);
				
}												
				

/*
void TestVector2Vrml()
{
	char fileName[] = "TestVector2Vrml.wrl";
	FILE* pFile = OpenVrmlFile(fileName);
	Vector3D begin(3,4,5), end(30,5,-9);
	
	Vector2Vrml(pFile,1,begin,end,1,Color3D(0.5,1,0));
	Sphere2Vrml(pFile,begin,2,Color3D(1,0,0));
	Sphere2Vrml(pFile,end,2,Color3D(0,1,0));
	
	CloseVrmlFile(pFile);
	ShowInVrmlview(fileName);
}struct MyTT{
	MyTT(){TestVector2Vrml();};
};
static MyTT myTT;
*/



void ToVrmlFile(string& vrml,string fileName,bool show)
{
	string name = (fileName.length())?fileName:"temp_ToVrmlFile.wrl";
	
	FILE* pFile = OpenVrmlFile(fileName.c_str());
	fprintf(pFile, "%s\n",vrml.c_str());
	CloseVrmlFile(pFile);
	
	if(show)
		ShowInVrmlview(name.c_str());
}
	
			
/* Write a view point to vrml file. */
void Viewpoint2Vrml(FILE* pFile,
				Vector3D position,
				Vector3D rotAxis,
				double angleInRadians,
				string title,
				double fov,
				bool jump)
{

	fprintf(pFile,"\nViewpoint { \n");
	fprintf(pFile,"\tfieldOfView    %4.7f \n",fov);
	fprintf(pFile,"\tjump  %s\n",jump?"TRUE":"FALSE");
	fprintf(pFile,"\torientation    %f %f %f %f\n",rotAxis.X(),rotAxis.Y(),rotAxis.Z(),angleInRadians);
	fprintf(pFile,"\tposition       %f %f %f\n",position.X(),position.Y(),position.Z());
	
	static int count = 0; count++;
	if(title.length()<=0)
	{
		sprintf(title,"Viewpoint_%d",count);
	}
	fprintf(pFile,"\tdescription    \"%s\"\n",title.c_str());
	
	fprintf(pFile,"\n}\n\n");
}

void VrmlFile::WriteAxes3D(Vector3D trans,
			Vector3D scale,
			Vector3D rotAxis,
			double rotAngle)		
{
	Axes3D2Vrml(pFile,trans,scale,rotAxis,rotAngle);
}
