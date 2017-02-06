/*
Copyright 2010 University of Twente

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

#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "VectorPoint.h"
#include "LaserPoints.h"
//#include "Photogrammetry.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "LineTopologiesComplex.h"

// GDAL OSR
#include <gdal_priv.h>
#include <gdal.h>
#include <ogrsf_frmts.h>
#include <ogr_geometry.h>
using namespace std;

void PrintUsage()
{
	printf("Usage: pcm2shp      -ip <input map points (3D object points)>\n");
	printf("                    -it <input  map topologies>\n");
	printf("                    -odir  <output shapefile>\n");
	printf("                    [-model (e.g. 3D building PCM version)]\n");
}

//Dong write
void WriteShapefile(const char *inPointsFile, const char *inTopLinesFile, char *outShapefile, bool isModel);
void ModelToSHP(OGRFeature *poFeature, const ObjectPoints &inObjPnts, const LineTopologies &inTopLines);
void WriteModelSHP(const char *fileName, const ObjectPoints &inObjPnts, const LineTopologies &inTopLinesFile);
void WritePolygon(const ObjectPoints &inObjPnts, const LineTopology &inPolygon, OGRPolygon& outPolygon);
//void WritePoint(const ObjectPoints::const_iterator &inItrObjPnt, const LineTopology::const_iterator &inTpline, OGRPoint &outPoint);

//-ip 2_bld.objpts -it 2_bld.top -odir 2_bld.shp -model
int main(int argc, char *argv[])
{
	InlineArguments args(argc, argv);

	// Check on required input files
	if (args.Contains("-usage") ||
		!args.Contains("-ip") ||
		!args.Contains("-it") ||
		!args.Contains("-odir"))
	    {
			if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
			PrintUsage();
			return EXIT_SUCCESS;
	    }

	// Call the main function
	WriteShapefile(args.String("-ip"), 
				   args.String("-it"),
				   args.String("-odir"), 
				   args.Contains("-model")
			      );
			
	return EXIT_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////
//The main function
//////////////////////////////////////////////////////////////////////////
void WriteShapefile(const char *inPointsFile, const char *inTopLinesFile, char *outShapefile, bool isModel)
{
	if (!outShapefile || !inPointsFile || !inTopLinesFile)
		return;

	ObjectPoints objPnts;
	LineTopologies topLines;

	// Read input data
	if (!objPnts.Read(inPointsFile)) {
		cout<<"Error reading map points from file %s\n";
		return;
	}

	if (!topLines.Read(inTopLinesFile)) {
		cout<<"Error reading map lines from file %s\n";
		return;
	}

//	if (isModel)
		WriteModelSHP(outShapefile, objPnts, topLines);
//	else
//		objPnts.WriteMapSHP(outShapefile, topLines);
}


void WriteModelSHP(const char *fileName, const ObjectPoints &inObjPnts, const LineTopologies &inTopLinesFile) 
{
	if (!fileName)
		return;

	CPLSetConfigOption("GDAL_DATA","D:\\OpenSource\\GDAL\\gdal-1.11.0\\data");
	OGRRegisterAll();

	const char *pszDriverName = "ESRI Shapefile"; 
	OGRSFDriver *pDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(pszDriverName);  
	if( pDriver == NULL ){  
		std::cout<<"%s driver not available.\n"<<pszDriverName; 
		return;
	}  

	OGRDataSource *pDS = pDriver->CreateDataSource(fileName, NULL);  
	if( pDS == NULL ){  
		std::cout<<"Creation of output file failed.\n";  
		return;
	}  

	OGRLayer *pLayer = pDS->CreateLayer("test", NULL, wkbPolygon25D, NULL);  
	if( pLayer == NULL ){  
		std::cout<<"Layer creation failed.\n";  
		return;
	} 

	OGRFieldDefn fieldName("FileNumber", OFTInteger);
	fieldName.SetWidth(20);
	if(pLayer->CreateField(&fieldName)!=OGRERR_NONE)
	{
		std::cout<<"Create FileNumber field failed.\n";
		return;
	}

	OGRFieldDefn fieldName2("Label", OFTInteger);
	fieldName2.SetWidth(20);
	if(pLayer->CreateField(&fieldName2)!=OGRERR_NONE)
	{
		std::cout<<"Create FileNumber field failed.\n";
		return;
	}

	OGRFieldDefn fieldName3("dipID", OFTReal);
	fieldName3.SetWidth(20);
	if(pLayer->CreateField(&fieldName3)!=OGRERR_NONE)
	{
		std::cout<<"Create FileNumber field failed.\n";
		return;
	}


	OGRFeature *pFeature = OGRFeature::CreateFeature(pLayer->GetLayerDefn());
	if( pFeature == NULL ){  
		std::cout<<"Feature creation failed.\n";
		return;
	}

	LineTopologies::const_iterator itrTpLine;
	LineTopology::const_iterator itrInVertex;

	ObjectPoints::const_iterator objPoint;
	OGRPolygon polygon;
	//std::stringstream roofID;
	int ID;

	//int lineCounter = 0;
	for (itrTpLine=inTopLinesFile.begin(); itrTpLine!=inTopLinesFile.end(); ++itrTpLine) 
	{		
		WritePolygon(inObjPnts, *itrTpLine, polygon);

		//write attributes 
//		if (itrTpLine->HasAttribute(FileNameTag))
//		{
//			int FileNumber = itrTpLine->Attribute(FileNameTag);
//			pFeature->SetField("FileNumber", FileNumber);
//		}

		if(itrTpLine->HasAttribute(LineLabelTag))
		{
			int labelTag = itrTpLine->Attribute(LineLabelTag);
			pFeature->SetField("Label", labelTag);
		}

		if (itrTpLine->HasAttribute(IDNumberTag))
		{
			ID = itrTpLine->Attribute(IDNumberTag);
			pFeature->SetField("dipID", ID);
		}else 
			pFeature->SetField("dipID", 1);


		pFeature->SetGeometry(&polygon);
		if(pLayer->CreateFeature(pFeature) != OGRERR_NONE){  
			std::cout<<"Failed to create feature in SHP.\n";  
			return;		
		}
	
		polygon.empty();
	}

	OGRFeature::DestroyFeature(pFeature);
	OGRDataSource::DestroyDataSource(pDS);
}

void ModelToSHP(OGRFeature *poFeature, const ObjectPoints &inObjPnts, const LineTopologies &inTopLines)
{
	LineTopologies::const_iterator itrPolygon;
	OGRPolygon ogrPolygon;

	for (itrPolygon=inTopLines.begin(); itrPolygon!=inTopLines.end(); ++itrPolygon) 
	{
		WritePolygon(inObjPnts,*itrPolygon,ogrPolygon);
		poFeature->SetGeometry(&ogrPolygon);
		ogrPolygon.empty();
	}
}

void WritePolygon(const ObjectPoints &inObjPnts, const LineTopology &inPolygon, OGRPolygon& outPolygon)
{
	OGRPoint ogrPoint;
	OGRLinearRing ogrRing;

	ObjectPoints::const_iterator objPoint;
	LineTopology::const_iterator itrVertex;
	bool isClosed = true;
	
	for (itrVertex=inPolygon.begin(); itrVertex!=inPolygon.end(); ++itrVertex) 
	{
		objPoint = inObjPnts.ConstPointIterator(*itrVertex);
		if (objPoint == inObjPnts.end()) {
			isClosed = false;
			break;
		}
		
		ogrPoint.setX(objPoint->GetX());
		ogrPoint.setY(objPoint->GetY());
		ogrPoint.setZ(objPoint->GetZ());
		ogrRing.addPoint(&ogrPoint);
	}

	ogrRing.closeRings();		
	outPolygon.addRing(&ogrRing);
}

//ToDo
void WritePoint(const ObjectPoints::const_iterator &inItrObjPnt, const LineTopology::const_iterator &inTpline, OGRPoint &outPoint)
{
	outPoint.setX(inItrObjPnt->GetX());
	outPoint.setY(inItrObjPnt->GetY());
	outPoint.setZ(inItrObjPnt->GetZ());		
}


