/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 29-07-2013
Description: 
Revise:
----------------------------------------------------------*/
#include "LaserPoints.h"
#include "TINEdges.h"
#include "TINEdgeSet.h"
#include "LineTopology.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "PointNumber.h"
#include "PointNumberList.h"
#include "Plane.h"
#include "Vector3D.h"
#include "Position3D.h"
#include "TINEdges.h"
#include "Vector3D.h"
#include "HoughSpace.h"
#include "LineSegment2D.h"
#include "LineSegments2D.h"

#include "InferenceEngine.h"
#include <vector>
#include <math.h>
#include <map>

using namespace std;

IEDll_API  bool RepairMapData(ObjectPoints& mapPnts, LineTopologies& mapTops)
{
	printf("Repairing map data\n");
	mapPnts.RemoveDoublePoints(mapTops, 0.01, true);

#ifdef _DEBUG
	mapPnts.Write("debug.objpts");
	mapTops.Write("debug.top");
#endif
	
	//////////////////////////////////////////////////////////////////////////
	//add polygon ID number
	//use polygon number as polygon ID number
	if (!mapTops.empty() && !mapTops[0].HasAttribute(IDNumberTag)) {
		for (unsigned int i=0; i<mapTops.size(); i++)
			mapTops[i].Attribute(IDNumberTag) = mapTops[i].Attribute(PolygonNumberTag);
	}

	//////////////////////////////////////////////////////////////////////////
	//add hole tag
	for (unsigned int i=0; i<mapTops.size(); i++) {
		if (!mapTops[i].HasAttribute(HoleTag))
			mapTops[i].Attribute(HoleTag) = false;
	}

	//////////////////////////////////////////////////////////////////////////
	//remove repeat nodes
	int lastNode, curNode;
	LineTopologies::iterator curTop;
	for (int i=0; i<mapTops.size(); ++i) {
		//remove open polygons
		curTop = mapTops.begin()+i;
		if (!curTop->IsClosed()) {
			mapTops.erase(curTop);
			i--;
			continue;
		}
		if (i==490)
		{
			int aaa = 0;
		}
		//remove repeat nodes
		lastNode = mapTops[i][0].Number();
		for (int j=1; j<curTop->size(); ++j) {
			curNode = (*curTop)[j].Number();
			if (curNode==279)
			{
				int aaaa = 1;
			}
			if (curNode == lastNode) {
				curTop->erase(curTop->begin()+j);
				j--;
			}
			lastNode = curNode;
		}
		//curTop->SetAttribute(GradientTag, curTop->Attribute(HoleTag));

		if(curTop->HasAttribute(HoleTag))
			curTop->SetAttribute(LabelTag, curTop->Attribute(HoleTag));
		else
			curTop->SetAttribute(LabelTag, 0);
	}

	mapTops.RemoveCollinearNodes(mapPnts);
	//renumber the points, and desert the unreferenced points
	mapPnts.RemoveUnusedPnts(mapTops);

	/*
#ifdef _DEBUG
	ObjectPoints testObjPnts;
	LineTopologies testLineTops;
	LineTopology testLineTop;
	LineTopology::iterator node;
	ObjectPoints::const_iterator point;
	for (int i=0; i<mapTops.size(); ++i) {
		printf("%d\r",i);
		testLineTop = mapTops[i];
		testLineTops.clear();
		testObjPnts.clear();

		for (node=mapTops[i].begin(); node!=mapTops[i].end()-1; node++) {
			point = mapPnts.ConstPointIterator(node->Number());
			testObjPnts.push_back(*point);
		}
		testLineTops.push_back(mapTops[i]);
		testLineTops.ReNumber(testObjPnts);

		//if (i==0 || i==43||i==45||i==47||i==67) continue;
		if (i==68) {
			printf("%d\n",i);
			testLineTops.Write("F:\\test_data\\triangle_err\\err68.top");
			testObjPnts.Write("F:\\test_data\\triangle_err\\err68.objpts");
		}
		testObjPnts.Triangulate(testLineTops);
	}
#endif*/

#ifdef _DEBUG
	mapPnts.Write("debug.objpts");
	mapTops.Write("debug.top");
#endif

	return true;
}

#include <time.h>
//match points with polygons
IEDll_API  bool MatchPointsWithMap(LaserPoints& lasPnts, ObjectPoints& mapObjPts, LineTopologies& mapTops, LaserPointTag tag)
{
	printf("Matching points with maps\n");
	LaserPoints::iterator itrCloud;
	LineTopologies::iterator poly;

	//////////////////////////////////////////////////////////////////////////
	//use bond box for coarsely searching
	//mapTops[i]
	vector<DataBounds2D> polyBounds;
	DataBounds2D temBound;
	for (int i=0; i<mapTops.size(); i++) {
		temBound.Initialise();
		for (int j=0; j<mapTops[i].size(); ++j) {
			temBound.Update(mapObjPts[mapTops[i][j].Number()].Position2DOnly());
		}
		polyBounds.push_back(temBound);
	}

	int mapTopCounts = mapTops.size();
	lasPnts.RemoveAttribute(tag);
	//lasPnts.RemoveAttribute(ComponentNumberTag);
	//#pragma omp parallel for reduction(+:sum) private(x)
	
	time_t start;
	time( &start );
	//#pragma omp parallel for private(poly, itrCloud)
	//for (itrCloud=lasPnts.begin(); itrCloud!=lasPnts.end(); ++itrCloud) {
/*	for (int i=0; i<lasPnts.size();++i) {
		printf("  Match %7d point\r", i);
		itrCloud = lasPnts.begin()+i;
		for (int iLine=0; iLine<mapTopCounts; ++iLine) {
			//coarsely check
			if (!polyBounds[iLine].Inside(*itrCloud)) continue;
			poly = mapTops.begin()+iLine;
			if (itrCloud->InsidePolygon(mapObjPts, *poly)) {
			//if(MyIsInsidePoly(*itrCloud, *poly, mapObjPts)) {
				itrCloud->SetAttribute(PolygonNumberTag, poly->Number());
				itrCloud->SetAttribute(ComponentNumberTag, poly->Number());
#ifdef _DEBUG
				itrCloud->SetAttribute(ComponentNumberTag, poly->Number());
#endif
				break;
			}
		}
	}*/
	for (int i=0; i<lasPnts.size();++i) {
		printf("  Match %7d point\r", i);
		for (int iLine=0; iLine<mapTopCounts; ++iLine) {
			//coarsely check
			if (mapTops[iLine].Attribute(HoleTag)==1) continue;//hole 
			if (!polyBounds[iLine].Inside(lasPnts[i])) continue;
			//int polyNum = mapTops[iLine].Number();
			//if (lasPnts[i].InsidePolygon(mapObjPts, mapTops[iLine])) {
			if(MyIsInsidePoly(lasPnts[i], mapTops[iLine], mapObjPts)) {
				lasPnts[i].SetAttribute(tag, mapTops[iLine].Attribute(IDNumberTag));
			//	lasPnts[i].SetAttribute(ComponentNumberTag, mapTops[iLine].Number());
#ifdef _DEBUG
				lasPnts[i].SetAttribute(ComponentNumberTag, mapTops[iLine].Attribute(IDNumberTag));
#endif
				break;
			}
		}
	}
	time_t end;
	time(&end);
	printf("  Time used for match data: %.2f second\n",difftime(end, start));

#ifdef _DEBUG
	lasPnts.Write("debug.laser");
#endif
	//MyDeriveSegPNL(lasPnts, m_PnlPolyTag, PolygonNumberTag);

	return true;
}

IEDll_API  bool MyMergeMapPolygons(ObjectPoints& mapObjPts, LineTopologies& mapTops)
{
	mapTops.MergePolygons();
	mapTops.RemoveCollinearNodes(mapObjPts);
	mapObjPts.RemoveUnusedPnts(mapTops);

	return true;
}