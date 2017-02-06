/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : April 11, 2012
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
#include "AdjacentGraph.h"

#include "InferenceEngine.h"
#include <vector>
#include <math.h>
#include <map>

using namespace std;

IEDll_API bool RefineCornerOfXFaces(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	AdjacentGraph adjGraph = AdjacentGraph(gLaserPnts, &localLineTops, vecLocalSegPnts);
	adjGraph.DoSearchCliques();
	vector<vector<int> > vecCliques = adjGraph.GetLeastCliques();

	vector<int> clique;
	vector<Position3D> vecObjPnts;
	Position3D aveObjPnt, temObjPnt, ObjPnt1, ObjPnt2;
	int segNum1, segNum2;
	double temDist1, temDist2, leastDist, meanDist;
	vector<double> vecDists;
	LineTopologies vecTopLines;
	LineTopology temTopLine;
	bool bFailFlag;
	int indLine;
	vector<Position3D> vecIntPnts;
	vector<Line3D> vecLines;
	vector<Line3D>::iterator itrLine1, itrLine2;

	for (int iC=0; iC<vecCliques.size(); ++iC) {
		clique = vecCliques[iC];
		if (clique.size() <= 4 ) continue;

		//get intersection lines
		bFailFlag = false;
		vecTopLines.clear();
		vecLines.clear();
		vecDists.clear();
		vecIntPnts.clear();
		for (int iNode=0; iNode<clique.size(); ++iNode) {
			segNum1 = clique[iNode];
			if (iNode == clique.size()-1)
				segNum2 = clique[0];
			else
				segNum2 = clique[iNode+1];

			SearchLineBy2Faces(localLineTops, segNum1, segNum2, indLine);
			if (indLine == -1) {
				bFailFlag = true;
				break;
			}
			vecTopLines.push_back(localLineTops[indLine]);
			ObjPnt1 = localObjPnts[localLineTops[indLine][0].Number()];
			ObjPnt2 = localObjPnts[localLineTops[indLine][1].Number()];
			vecLines.push_back(Line3D(ObjPnt1, ObjPnt2));
		}
		if (bFailFlag) continue;

		//get intersection point
		aveObjPnt = Position3D(0.0, 0.0, 0.0);
		meanDist = 0.0;
		for (int iLine=0; iLine<vecLines.size(); ++iLine) {
			itrLine1 = vecLines.begin()+iLine;
			if (iLine==vecLines.size()-1)
				itrLine2 = vecLines.begin();
			else
				itrLine2 = vecLines.begin()+iLine+1;

			MyIntersect2Lines(*itrLine1, *itrLine2, temObjPnt);
		
			ObjPnt1 = localObjPnts[vecTopLines[iLine][0].Number()];
			ObjPnt2 = localObjPnts[vecTopLines[iLine][1].Number()];
			temDist1 = temObjPnt.Distance(ObjPnt1);
			temDist2 = temObjPnt.Distance(ObjPnt2);
			leastDist = temDist1<temDist2?temDist1:temDist2;

			vecDists.push_back(leastDist);
			vecIntPnts.push_back(temObjPnt);
			aveObjPnt += temObjPnt/vecLines.size();
			meanDist += leastDist/vecLines.size();
		}

		meanDist = 0.0;
		for (int i=0; i<vecIntPnts.size(); i++)
			meanDist += vecIntPnts[i].Distance(aveObjPnt)/vecIntPnts.size();

		if (meanDist > 2.5)
			continue;

		for (int iLine=0; iLine<vecLines.size(); ++iLine) {
			ReplaceLineNode(localObjPnts, vecTopLines[iLine], aveObjPnt, 10.0);
			//ReplaceLineNode(localObjPnts, vecTopLines[iLine], vecIntPnts[iLine], 10.0);
		}
	}
	//	continue;
	return false;
}

