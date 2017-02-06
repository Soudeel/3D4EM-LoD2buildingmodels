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
#include <fstream>
#include "InlineArguments.h"
#include <vector>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string>

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

typedef map<string, int> BGTINDEX;

void PrintUsage()
{
	printf("Usage:  shp2pcm  -ishp <input shapefile>\n");
	printf("                 -op <output  pcm points>\n");
	printf("                 -ot <output pcm toplines>\n");
	printf("                 [-bgtInd (e.g. input BGT index from txt)]\n");
	printf("                 [-ibgt (e.g. Input the pathway of bgt txt>]\n");
}

void ReadShape2PCM(const char *inShapefile, const char *outPoints, const char *outToplines, bool isInBGT, const char* inBGTFile);
bool ReadExtRing(OGRPolygon *pInPolygon, int &inPntNumber, int inTopLineNum, 
				 ObjectPoints &outObjPnts, LineTopology &outTopLine );
bool ReadInRing(OGRPolygon *pInPolygon, int &inPntNum, int inTopLineNum,
				ObjectPoints &outObjPnts, LineTopologies &outTopLines );
bool ReadLineString(OGRLineString *pInLineString, int &inPntNum, int inTopLineNum, 
					ObjectPoints &outObjPnts, LineTopology &outTopLine);
void ReadField(const BGTINDEX inBGTInd, OGRFeature *inPoFeature, OGRFeatureDefn *inPoFeatureDefn, LineTopology &outTopLine);
void SetLineLabel(const BGTINDEX inBGTInd, const string &rowName, LineTopology &outTopLine);
int GetLineLabel(BGTINDEX inBGTInd, const string inObjType);
BGTINDEX InitialBGTIndex();
BGTINDEX InitialBGTIndex(const char* inBgtIndFile);
void WriteLogFile(const std::vector<int> inVecID);

//-ishp 1.shp -op 1.objpts -ot 1.top -bgtInd -ibgt bgt.txt 

int main(int argc, char *argv[])
{
	InlineArguments args(argc, argv);
	
	// Check on required input files
	if (args.Contains("-usage") ||
		!args.Contains("-ishp") ||
		!args.Contains("-op")   ||
		!args.Contains("-ot"))
	    {
			if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
			PrintUsage();
			return EXIT_SUCCESS;
	    }

	// Call the main function
	ReadShape2PCM(args.String("-ishp"),
				  args.String("-op"),
				  args.String("-ot"),
				  args.Contains("-bgtInd"),
				  args.String("-ibgt"));
			
	return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
//The main function
//////////////////////////////////////////////////////////////////////////

void ReadShape2PCM(const char *inShapeFile, const char *outPointsFile, const char *outTopLinesFile, bool isInBGT, const char* inBGTFile)
{
	if (!inShapeFile || !outPointsFile || !outTopLinesFile)
		return;

	OGRRegisterAll();

	OGRDataSource *poDS = OGRSFDriverRegistrar::Open(inShapeFile, false);  
	if( poDS == NULL ){  
		std::cout<<"Creation of output file failed.\n";  
		return;
	} 

	OGRLayer *poLayer = poDS->GetLayer(0);	
	poLayer->ResetReading();
		
	ObjectPoints outObjPnts;
	LineTopology temOutLineTop;
	LineTopologies outTopLines;

	OGRFeature *poFeature;
	OGRGeometry *poGeometry;
	OGRPolygon *ptrPolygon;
	OGRFeatureDefn *poFDefn;
	int ID;
	vector<int> vecID;
	BGTINDEX BGTInd;
	OGRwkbGeometryType geoType;

	//Initial BGT index
	if (!isInBGT) {
		BGTInd = InitialBGTIndex();
	}
	else{
		BGTInd = InitialBGTIndex(inBGTFile);
	}
	
	int PntNumber = 0;
	int count = poLayer->GetFeatureCount();
	for(int i=0; i<poLayer->GetFeatureCount(); ++i) 
	{
		poFeature = poLayer->GetFeature(i);
		if( poFeature == NULL ){  
			std::cout<<"Feature creation failed.\n";
			continue;
		}

		//Read geometry
		poGeometry = poFeature->GetGeometryRef();
		if (poGeometry == NULL){
			std::cout<<"Geometry creation failed.\n";
			continue;
		}

		ptrPolygon = (OGRPolygon*)poGeometry;
		geoType = wkbFlatten(poGeometry->getGeometryType());
		if( geoType == wkbPolygon||
			geoType == wkbPolygon25D||
			geoType == wkbMultiPolygon||
			geoType == wkbMultiPolygon25D)
		{
			//Read field
			poFDefn = poLayer->GetLayerDefn();

			if (0==BGTInd.size())
			{
				BGTInd = InitialBGTIndex();
			}

			ReadField(BGTInd, poFeature, poFDefn, temOutLineTop);

			if (temOutLineTop.HasAttribute(IDNumberTag))
				ID = temOutLineTop.Attribute(IDNumberTag);
			else
				ID = i;
		
			bool isExtSuccess, isInSuccess;
			isExtSuccess = ReadExtRing(ptrPolygon, PntNumber, ID, outObjPnts, temOutLineTop);
			if (isExtSuccess && 1!=outObjPnts.size())
			{	
				isInSuccess = ReadInRing(ptrPolygon, PntNumber, ID, outObjPnts, outTopLines);
			    if(isInSuccess) {
						outTopLines.push_back(temOutLineTop);
				}else{
					vecID.push_back(ID);
					WriteLogFile(vecID);
				}					
			}else{ // if wrong polygon: extrings share one ID				
			    vecID.push_back(ID);
				WriteLogFile(vecID);
			}	
		} else if (geoType == wkbLineString ||
			       geoType == wkbLineString25D)
		{
			if(ReadLineString((OGRLineString*)poGeometry, PntNumber, i, outObjPnts, temOutLineTop))
				outTopLines.push_back(temOutLineTop);
		}
	
		OGRFeature::DestroyFeature(poFeature);		
	}

	OGRDataSource::DestroyDataSource(poDS);
	if(!outTopLines.empty()){
		outObjPnts.Write(outPointsFile);
		outTopLines.Write(outTopLinesFile);
	}
}

bool ReadExtRing(OGRPolygon *pInPolygon, int &inPntNum, int inTopLineNum, 
				 ObjectPoints &outObjPnts, LineTopology &outTopLine)
{	
	outTopLine.clear();
	OGRLinearRing *ptrExtRing = pInPolygon->getExteriorRing();
	int pntCounter = ptrExtRing->getNumPoints();
	double inX, inY, inZ;
	ObjectPoint temPnt;	

	//Exclude the endPnt
	for (int i=0; i<pntCounter-1; ++i){
		temPnt.X() = ptrExtRing->getX(i);
		temPnt.Y() = ptrExtRing->getY(i);
		temPnt.Z() = ptrExtRing->getZ(i);
		temPnt.Number() = inPntNum++;

		outObjPnts.push_back(temPnt);
		outTopLine.push_back(temPnt.Number());
	}

	if (0==outTopLine.size())
	    return false;

	outTopLine.SetAttribute(HoleTag, 0);
	outTopLine.SetAttribute(IDNumberTag, inTopLineNum);
	outTopLine.push_back(outTopLine[0]);
	return true;
}

bool ReadInRing(OGRPolygon *pInPolygon, int &inPntNum, int inTopLineNum, 
				ObjectPoints &outObjPnts, LineTopologies &outTopLines)
{	
	LineTopology temTLine;
	int inRingCount = pInPolygon->getNumInteriorRings();
	OGRLinearRing *pInRing;
	int pntCount;
	ObjectPoint temPnt;
	
	for(int i=0; i<inRingCount; ++i) {
		pInRing = pInPolygon->getInteriorRing(i);
		pntCount = pInRing->getNumPoints();
		
		//Exclude the endPnt
		temTLine.clear();
		for (int j=0; j<pntCount-1; ++j){
			temPnt.X() = pInRing->getX(j);
			temPnt.Y() = pInRing->getY(j);
			temPnt.Z() = pInRing->getZ(j);
			temPnt.Number() = inPntNum++;

			outObjPnts.push_back(temPnt);
			temTLine.push_back(temPnt.Number());
		}

		if (0==temTLine.size())
			return false;

		temTLine.push_back(temTLine[0]);
		temTLine.SetAttribute(HoleTag, 1);
		temTLine.SetAttribute(IDNumberTag, inTopLineNum);
		outTopLines.push_back(temTLine);
	}

	return true;
}


bool ReadLineString(OGRLineString *pInLineString, int &inPntNum, int inTopLineNum, 
				 ObjectPoints &outObjPnts, LineTopology &outTopLine)
{	
	if (!pInLineString) return false;
	outTopLine.clear();
	int pntCounter = pInLineString->getNumPoints();
	if (pntCounter==0) return false;

	double inX, inY, inZ;
	ObjectPoint temPnt;	

	//Exclude the endPnt
	for (int i=0; i<pntCounter; ++i) {
		temPnt.X() = pInLineString->getX(i);
		temPnt.Y() = pInLineString->getY(i);
		temPnt.Z() = pInLineString->getZ(i);
		temPnt.Number() = inPntNum++;

		outObjPnts.push_back(temPnt);
		outTopLine.push_back(temPnt.Number());
	}

	outTopLine.SetAttribute(IDNumberTag, inTopLineNum);

	return true;
}

void SetLineLabel(const BGTINDEX inBGTInd, const string &rowName, LineTopology &outTopLine)
{
	//ToDo Mon 8 Juni
	int temp = GetLineLabel(inBGTInd, rowName);
	if(temp!=0)
		outTopLine.SetAttribute(LineLabelTag, temp);
}

//todo id
//any type to unsigned int: char, string, int
//any name: ID or ....
void ReadField(const BGTINDEX inBGTInd, OGRFeature *inPoFeature, OGRFeatureDefn *inPoFeatureDefn, LineTopology &outTopLine)
{
	bool bHasID = false;
	for(int i=0; i<inPoFeatureDefn->GetFieldCount(); ++i) {
		OGRFieldDefn *poFieldDefn = inPoFeatureDefn->GetFieldDefn(i);

		string fieldname = poFieldDefn->GetNameRef();
		if(fieldname=="dipID") {
			bHasID = true;
			int ID = inPoFeature->GetFieldAsInteger(i);
			outTopLine.SetAttribute(IDNumberTag, ID);
		}
		else if(fieldname=="Objecttype")
		{
			string fieldname = inPoFeature->GetFieldAsString(i);
			SetLineLabel(inBGTInd, fieldname, outTopLine);
		} 
	}

	if (!bHasID)
		outTopLine.RemoveAttribute(IDNumberTag);
}

int GetLineLabel(BGTINDEX inBGTInd, const string inObjType)
{
	BGTINDEX::iterator itrBgtInd = inBGTInd.find(inObjType);
	if(itrBgtInd == inBGTInd.end())
		return 0;

	return itrBgtInd->second;
}

//BGT INDEX could not have duplicate key
BGTINDEX InitialBGTIndex()
{
	BGTINDEX bgtInd;

	bgtInd.insert(pair<string, int>("Pand", 1001));
	bgtInd.insert(pair<string, int>("OverigBouwwerk", 1001));
	bgtInd.insert(pair<string, int>("OTerrein", 5001));
	bgtInd.insert(pair<string, int>("Wegdeel", 2001));
	bgtInd.insert(pair<string, int>("OndersteunendWegdeel", 2001));
	bgtInd.insert(pair<string, int>("BegroeidTerreindeel", 5001));
	bgtInd.insert(pair<string, int>("Waterdeel", 6001));
	bgtInd.insert(pair<string, int>("OndersteunendWaterdeel", 6001));
	bgtInd.insert(pair<string, int>("Overbruggingsdeel",7001));
	bgtInd.insert(pair<string, int>("Tunneldeel", 8001));

	//Wegdeel
	bgtInd.insert(pair<string, int>("OV-baan", 2101));
	bgtInd.insert(pair<string, int>("overweg", 2102));
	bgtInd.insert(pair<string, int>("spoorbaan", 2103));
	bgtInd.insert(pair<string, int>("baan voor vliegverkeer", 2104));
	bgtInd.insert(pair<string, int>("rijbaan autosnelweg", 2105));
	bgtInd.insert(pair<string, int>("rijbaan autoweg", 2106));
	bgtInd.insert(pair<string, int>("rijbaan regionale weg", 2107));
	bgtInd.insert(pair<string, int>("rijbaan lokale weg", 2108));
	bgtInd.insert(pair<string, int>("fietspad", 2109));
	bgtInd.insert(pair<string, int>("voetpad", 2110));
	bgtInd.insert(pair<string, int>("voetpad op trap", 2111));
	bgtInd.insert(pair<string, int>("ruiterpad", 2112));
	bgtInd.insert(pair<string, int>("parkeervlak", 2113));
	bgtInd.insert(pair<string, int>("voetgangersgebied", 2114));
	bgtInd.insert(pair<string, int>("inrit", 2115));
	bgtInd.insert(pair<string, int>("woonerf", 2116));
	bgtInd.insert(pair<string, int>("gesloten verharding", 2117));
	bgtInd.insert(pair<string, int>("open verharding", 2118));
	bgtInd.insert(pair<string, int>("half verhard", 2119));
	bgtInd.insert(pair<string, int>("onverhard", 2120));	
	bgtInd.insert(pair<string, int>("verkeerseiland", 2121));
	bgtInd.insert(pair<string, int>("berm", 2122));
	bgtInd.insert(pair<string, int>("transitie", 2123));
	bgtInd.insert(pair<string, int>("voetgangersgeb", 2124));

	return bgtInd;
}

BGTINDEX InitialBGTIndex(const char* inBgtIndFile)
{
	BGTINDEX bgtInd;
	BGTINDEX::iterator itrBgtInd;
	int label;
	std::string objType;

	std::ifstream file; 
	file.open(inBgtIndFile, ios::in);
	file >> noskipws;

	if(file.fail()){
		cout<<"Error Reading file"<<endl;
		file.close();
		return bgtInd;
	}else
	{	 	
		while(file>>label) 
		{ 
			getline(file, objType);
			//std::cout<<label<<objType<<"\n";
			bgtInd.insert(pair<string, int>(objType, label));	
		} 

		for(itrBgtInd=bgtInd.begin(); itrBgtInd!=bgtInd.end(); ++itrBgtInd)
			//cout<<itrBgtInd->first<<"\t"<<itrBgtInd->second<<"\n";
		file.close(); 		
	}

	return bgtInd;
}

void WriteLogFile(const std::vector<int> inVecID)
{
	int counter = inVecID.size();
	ofstream logFile("WrongPolygonID.txt");
	if(logFile.is_open()){
		logFile<<"Numbers of wrong polygons"<<inVecID.size()<<"\n";
		for(unsigned i=0; i<counter; ++i)			
			logFile<<"WrongPolygonID is:"<<inVecID[i]<<"\n";
	}

	logFile.close();
}
