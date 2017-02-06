/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 26-01-2011
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
//#include "LineTopsIterVector.h"
//#include "VectorPoint.h"
//#include "VRML_io.h"
//#include "dxf.h"
//#include "TIN.h"
//#include "Building.h"
//#include "Buildings.h"
//#include "stdmath.h"
//#include "triangle.h"
//#include "Database.h"
#include "InferenceEngine.h"
#include <vector>
#include <math.h>
#include <map>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <strings.h>
//#include <Matrix3.h>
//using namespace std;
using namespace std;

//LaserPoints g_laserPnts;//
//TINEdges g_edges;
//0 Attribute left hand segment number; 
//1 Attribute left hand segment number
//2: refined or not
//LineTopologies g_BuildLineTops;

std::vector<PointNumberList> gVecLocalSegPnts;


IEDll_API bool SearchNeigborNodes(ObjectPoints& localObjPnts, vector<bool>& vecIsSearched, 
	PointNumber numCurObjPnt, vector<PointNumber>& vecAdjPntNum, double fMinDist)
{
	vecIsSearched[numCurObjPnt.Number()] = true;
	vecAdjPntNum.clear();

	ObjectPoint curObjPnt = localObjPnts.PointByNumber(numCurObjPnt.Number());	
	ObjectPoint temObjPnt;
	PointNumber numTemObjPnt;
	for (int i=0; i<(int)localObjPnts.size(); i++)
	{
		temObjPnt = localObjPnts[i];
		numTemObjPnt = temObjPnt.Number();

		//has been searched
		if (vecIsSearched[numTemObjPnt.Number()])
			continue;
		//find suitable point
		if (temObjPnt.Distance(curObjPnt) <= fMinDist)
		{
			vecIsSearched[numTemObjPnt.Number()] = true;
			vecAdjPntNum.push_back(numTemObjPnt);
		}
	}

	if (vecAdjPntNum.empty())
		return false;
	else
		return true;	
}

IEDll_API bool SearchAdjacentSegments(const LaserPoints& gLaserPnts, TINEdges& gEdges, 
	vector<PointNumberList>& gVecPnlSegs, PointNumberList pnlOrgSeg, 
	vector<PointNumberList>& vecAdjSegPnts, int nMinSegPntNum, int nMinAdjPntNum)
{
	vecAdjSegPnts.clear();

	if (pnlOrgSeg.empty())
		return false;

//	TINEdges edges;
//	edges.Derive(gLaserPnts.TINReference());
//	TINEdgeSet tesNeibor;
	//PointNumberList pnlAdjPnts;
	TINEdges::iterator iTesNeib;
	vector<int> vecSegNumber;
	LaserPoint lpTemPnt;
	int curSegNumber = (gLaserPnts[pnlOrgSeg[0].Number()]).Attribute(SegmentNumberTag);
	int temSegNumber ;
	bool bStored = false;
	if (curSegNumber == 0)
		int aaaa = 0;
	//search for SegmentNumber of possible adjacent segments
	for (int i=0; i<(int)pnlOrgSeg.size(); i++)	{
		//tesNeibor = g_edges[pnlOrgSeg[i].Number()];
		iTesNeib = gEdges.begin() + pnlOrgSeg[i].Number();
		if (iTesNeib->empty())
			continue;
		
		for (PointNumberList::iterator itr=iTesNeib->begin(); itr!=iTesNeib->end(); itr++) {
			lpTemPnt = gLaserPnts[itr->Number()];
			if(!lpTemPnt.HasAttribute(SegmentNumberTag))
				continue;//invalid segment label

			temSegNumber = lpTemPnt.Attribute(SegmentNumberTag);
			if (temSegNumber == curSegNumber || lpTemPnt.Attribute(IsProcessedTag) != -1)
				continue;//laser point in current segment or this segment is processed
			bStored = false;
			for (int j=0; j<(int)vecSegNumber.size(); j++) {
				if (temSegNumber == vecSegNumber[j]) {
					bStored = true;//this segment number has been stored
					break;
				}
			}

			if(!bStored)
				vecSegNumber.push_back(temSegNumber);
		}
	}

	//for debug
//	FILE* file = fopen("c:\\adjacent segments.txt","a+b");
//	fprintf(file, "current segment: %d\nPossible adjacent segment:\n", curSegNumber);
//	for (int i=0; i<vecSegNumber.size(); ++i) {
//		fprintf(file,"%d  ", vecSegNumber[i]);
//	}
//	fprintf(file,"\nreal adjacent segments:\n");

	//search for adjacent segments
	PointNumberList pnlTemSeg;
	LaserPoints orgLaserPnts;
	for (int j=0; j<(int)pnlOrgSeg.size(); ++j)
		orgLaserPnts.push_back(gLaserPnts[pnlOrgSeg[j].Number()]);

	//LaserPoints temLaserPnts;
	for (int i=0; i<(int)vecSegNumber.size(); ++i) {
		temSegNumber = vecSegNumber[i];
		if ((temSegNumber== 1&& curSegNumber==5 )
			||(temSegNumber==5 && curSegNumber==1 )) {
			int aaaa = 0;
		}
		int indSeg = IndexSegBySegNumber(gLaserPnts, gVecPnlSegs, temSegNumber);
		if (indSeg < 0) continue;
		pnlTemSeg = gVecPnlSegs[indSeg];
		//pnlTemSeg = gLaserPnts.TaggedPointNumberList(SegmentNumberTag, temSegNumber);
	
		if ((int)pnlTemSeg.size()<nMinSegPntNum)
			continue;//this segment is too small

		//temLaserPnts.clear();
		//for (int j=0; j<(int)pnlTemSeg.size(); ++j)
		//	temLaserPnts.push_back(gLaserPnts[pnlTemSeg[j].Number()]);
			
		//if (IsAdjacent(&orgLaserPnts, &temLaserPnts,INTERSECT_FACE_MIN_DIST, nMinAdjPntNum))
		if (gBldRecPar.bStepEdges) {//when create step edges, check 2D neighboring
			if (IsAdjacent2D(gLaserPnts,gEdges,pnlOrgSeg,pnlTemSeg,gBldRecPar.minPntDist,nMinAdjPntNum))
				vecAdjSegPnts.push_back(pnlTemSeg);
		}
		else {
			if (IsAdjacent(gLaserPnts,gEdges,pnlOrgSeg,pnlTemSeg,gBldRecPar.minPntDist,nMinAdjPntNum))
			 vecAdjSegPnts.push_back(pnlTemSeg);
		}
	}

//	fprintf(file,"\n\n");
//	fclose(file);

	if (vecAdjSegPnts.empty())
		return false;
	else
		return true;
}


IEDll_API bool SearchLineBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, int& outLineInd)
{
	LineTopology temLineTop;
	bool bFlag = false;
	vector<int> vecTemLineTopInds;
	outLineInd = -1;

	SearchLinesOnPlane(localLineTops, faceNum1, vecTemLineTopInds);

	for (int i=0; i<(int)vecTemLineTopInds.size(); i++) {
		temLineTop = localLineTops[vecTemLineTopInds[i]];

		if (temLineTop.Attribute(SegmentLabel) == faceNum2 
			|| temLineTop.Attribute(SecondSegmentLabel) == faceNum2)	{
			//lineTop = temLineTop;
			bFlag = true;
			outLineInd = vecTemLineTopInds[i];
			break;
		}
	}

	return bFlag;
}

//search faces which are neighboring to face 1 and face 2 at the same time
IEDll_API bool SearchFaceBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, std::vector<int>& vecFaceNums)
{
	vecFaceNums.clear();

	vector<int> AdjFaces1, AdjFaces2, totAdjFace;
	SearchAdjacentFaces(localLineTops, faceNum1, AdjFaces1);
	SearchAdjacentFaces(localLineTops, faceNum2, AdjFaces2);

	totAdjFace = AdjFaces1;
	totAdjFace.insert(totAdjFace.end(), AdjFaces2.begin(), AdjFaces2.end());
	std::sort(totAdjFace.begin(), totAdjFace.end());

	for (int i=0; i<(int)totAdjFace.size()-1; i++)
	{
		if (totAdjFace[i] == faceNum1 || totAdjFace[i] == faceNum2)
			continue;

		if (totAdjFace[i] == totAdjFace[i+1] )
			vecFaceNums.push_back(totAdjFace[i]);
	}

	if (vecFaceNums.empty())
		return false;
	else 
		return true;
}

//search lines whose node is near to current node of current node,
//if one of these lines has been refined, stop search
IEDll_API bool SearchNodesOnNeigborLine(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops, 
	const int numCurLine, const int curNode, vector<int>& vecNeibNodes, double minDist)
{
	Position3D starPnt = (Position3D)localObjPnts[localLineTops[numCurLine][0].Number()];
	Position3D endPnt = (Position3D)localObjPnts[localLineTops[numCurLine][1].Number()];
	Position3D curPnt = (Position3D)localObjPnts[localLineTops[numCurLine][curNode].Number()];
	Line3D curLine(starPnt, endPnt);

	LineTopology temLineTop;
	Position3D temStartPnt, temEndPnt;
	Vector3D intersectPnt;
	Line3D temLine;
	int temNodeNum;

	vecNeibNodes.clear();
	for (int i=0; i<(int)localLineTops.size(); i++)
	{
		if (i == numCurLine )
			continue;

		temLineTop = localLineTops[i];
		temStartPnt = (Position3D)localObjPnts[temLineTop[0].Number()];
		temEndPnt = (Position3D)localObjPnts[temLineTop[1].Number()];
		temLine = Line3D(temStartPnt, temEndPnt);

		if (!MyIntersect2Lines(curLine, temLine, intersectPnt)
			|| (intersectPnt-(Vector3D)curPnt).Length() > minDist)
			continue;

		if((intersectPnt-temStartPnt).Length() < minDist)
		{
			temNodeNum = temLineTop[0].Number();
			if(temLineTop.Attribute(2) != -1)
				return false;//stop search
		}
		else if((intersectPnt-temEndPnt).Length() < minDist)
		{
			temNodeNum = temLineTop[1].Number();
			if(temLineTop.Attribute(2) != -1)
				return false;//stop search
		}	
		else
			continue;

		vecNeibNodes.push_back(temNodeNum);
	}

	if (vecNeibNodes.empty())
		return false;
	else
		return true;
}

/*
bool Detect3DLineByHoughTrans(const LaserPoints& lLaserPnts, const Plane& refPlane, std::vector<Position3D>& vecLineNodes)
{
	HoughSpace houghspace;
	LaserPoints temLaserPnts = lLaserPnts;
	Position3D temPos;
	Vector3D dir = refPlane.Normal();
	Line2D  line;
	double minPntNums = 5;
	double minDist = 0.2;
	double dist;
	vector<int> vecLineSegPnts;
	Position2D p1, p2;
	Position3D pos3d1, pos3d2;
	int pntNums;
	LaserPoint temLaserPnt;
	
	//transform points to x-y plane according to their original plane
	for (int i=0; i<lLaserPnts.size(); i++)
	{
		if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
		{
			temPos.X() = lLaserPnts[i].X();
			temPos.Y() = lLaserPnts[i].Y();
			temPos.Z() = 0;
		}
		else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
		{
			temPos.X() = lLaserPnts[i].Z();
			temPos.Y() = lLaserPnts[i].Y();
			temPos.Z() = 0;
		}
		else//on x-z plane
		{
			temPos.X() = lLaserPnts[i].X();
			temPos.Y() = lLaserPnts[i].Z();
			temPos.Z() = 0;
		}

		temLaserPnts[i].Position3DRef() = temPos;
	}

	//construct Hough space
	temLaserPnts.Label(-1);
	temLaserPnts.InitialiseHoughSpace(houghspace, -1, 85*PI/180, 3*PI/180, 0.5, 1, LabelTag);
	temLaserPnts.IncrementHoughSpace(houghspace);

	do 
	{
		pntNums = 0;
		line = houghspace.BestLine(&pntNums, 0, 3); 

		if (pntNums < minPntNums) continue;
		//pntNums = selLaserPnts.Label(plane, minDist, -1, -1,  1, 0);
		
		//remove detected points from hough space
		//and get planes of corresponding segments
		pntNums = 0;
		vecLineSegPnts.clear();
		for (int i=0; i<temLaserPnts.size(); i++)
		{	
			temLaserPnt = temLaserPnts[i];
			dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());
			if (dist > minDist) continue;

			pntNums++;
			houghspace.RemovePoint(temLaserPnt.X(), temLaserPnt.Y(), temLaserPnt.Z());

			vecLineSegPnts.push_back(i);
		}

		//to do, fit the line again

		if (pntNums < minPntNums) continue;
		
		double min = 9999999.0;
		double max = -99999999.0;
		double scalar;
		//search for start and end node
		for (int i=0; i<vecLineSegPnts.size(); i++)
		{
			temLaserPnt = temLaserPnts[i];
			scalar = line.Scalar(temLaserPnt.Position2DOnly());

			if (scalar < min) min = scalar;
			if (scalar > max) max = scalar;
		}

		p1 = line.Position(min);
		p2 = line.Position(max);

		//re-transform to original plane
		if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
		{
			pos3d1.X() = p1.X(); pos3d1.Y() = p1.Y(); pos3d1.Z() = 0.0;
			pos3d2.X() = p2.X(); pos3d2.Y() = p2.Y(); pos3d2.Z() = 0.0;
		}
		else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
		{
			pos3d1.X() = 0.0; pos3d1.Y() = p1.Y(); pos3d1.Z() = p1.X();
			pos3d2.X() = 0.0; pos3d2.Y() = p2.Y(); pos3d2.Z() = p2.X();
		}
		else//on x-z plane
		{
			pos3d1.X() = p1.X(); pos3d1.Y() = 0.0; pos3d1.Z() = p1.Y();
			pos3d2.X() = p2.X(); pos3d2.Y() = 0.0; pos3d2.Z() = p2.Y();
		}

		pos3d1 = refPlane.Project(pos3d1);
		pos3d2 = refPlane.Project(pos3d2);

		//out put
		ObjectPoint objPnt;
		LineTopology lineTop;

		objPnt.X() = pos3d1.X(); objPnt.Y() = pos3d1.Y(); objPnt.Z() = pos3d1.Z(); 
		objPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());

		objPnt.X() = pos3d2.X(); objPnt.Y() = pos3d2.Y(); objPnt.Z() = pos3d2.Z(); 
		objPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());

		lineTop.Number() = localLineTops.size();
		lineTop.SetAttribute(0, plane.Number());
		lineTop.SetAttribute(1, temPlane.Number());
		localLineTops.push_back(lineTop);
		
	} while (pntNums > minPntNums);
}*/

IEDll_API bool IsAdjacent(LaserPoints* pLLaserPnts, LaserPoints* pRLaserPnts, double nMinDistance, int nMinAdjPnts)
{
	if (!pLLaserPnts || !pRLaserPnts)
		return false;

	LaserPoints totPnts;
	pLLaserPnts->Label( 0);
	pRLaserPnts->Label( 1);
	int lPntsCount = totPnts.AddPoints(*pLLaserPnts);
	int rPntsCount = totPnts.AddPoints(*pRLaserPnts);
	
	int lSegNum = pLLaserPnts->at(0).Attribute(SegmentNumberTag); 
    int rSegNum = pRLaserPnts->at(0).Attribute(SegmentNumberTag);
	if(lSegNum==0 || rSegNum ==0)
	   int aaa = 0;


	totPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(totPnts.TINReference());
	LaserPoints::const_iterator itrCurPnt, itrTemPnt;
	TINEdgeSet neibTinEdgeSet;
	PointNumberList pnlAdjacent;
	PointNumberList::iterator itrPnlCurPnt;
	//Search for neighbor laser point between 2 segments
	for (itrCurPnt=totPnts.begin(); itrCurPnt!=totPnts.end(); itrCurPnt++) {
		if (!itrCurPnt->HasAttributeValue(LabelTag, 0)) 
			continue;//not left segment laser point

		neibTinEdgeSet = edges[itrCurPnt-totPnts.begin()];
		for (itrPnlCurPnt=neibTinEdgeSet.begin(); itrPnlCurPnt<neibTinEdgeSet.end(); itrPnlCurPnt++) {
			itrTemPnt = totPnts.begin() + itrPnlCurPnt->Number();
			if (!itrTemPnt->HasAttributeValue(LabelTag, 1))
				continue;//not right laser laser point

			if (nMinDistance > (itrTemPnt->Position3DRef() - itrCurPnt->Position3DRef()).Length()){
				pnlAdjacent.push_back(PointNumber(itrCurPnt-totPnts.begin()));
				pnlAdjacent.push_back(itrPnlCurPnt->Number());
			}
		}
	}

	//erase repeat points
	pnlAdjacent.Sort();
	for (PointNumberList::iterator itr=pnlAdjacent.begin(); itr!=pnlAdjacent.end(); itr++) {
		PointNumberList::iterator rightItr = itr+1;
		if (rightItr == pnlAdjacent.end())
			break;
		if(rightItr->Number() == itr->Number())
			pnlAdjacent.erase(rightItr);
	}

	//for debug
/*	LaserPoints temLPnts;
	for (int i=0; i<pnlAdjacent.size(); i++) {
	    temLPnts.push_back(totPnts[pnlAdjacent.at(i).Number()]);
    }
	temLPnts.Label(7);
	temLPnts.Write("E:\\test_data\\ripperda4_adjpnt.laser", 0, true);
    int nSize =	pnlAdjacent.size();
	*/
	
	if ((int)pnlAdjacent.size() >= nMinAdjPnts)
		return true;
	else
		return false;
}


IEDll_API bool IsAdjacent(const LaserPoints& gLaserPnts, const TINEdges& gEdges, const PointNumberList& pnlSeg1, 
	const PointNumberList& pnlSeg2, double fMinDistance, int nMinAdjPnts)
{
	PointNumberList::const_iterator itrPnl;
	LaserPoints::const_iterator itrLPnt, itrTemLpnt;
	itrLPnt = gLaserPnts.begin()+pnlSeg1.begin()->Number();
	int numSeg1 = itrLPnt->Attribute(SegmentNumberTag);
	itrLPnt = gLaserPnts.begin()+pnlSeg2.begin()->Number();
	int numSeg2 = itrLPnt->Attribute(SegmentNumberTag);
	TINEdges::const_iterator itrEdgeSet;
	PointNumberList pnlNeibPnts;
	TINEdgeSet::const_iterator itrEdge;
	bool bNeib;

	if ((numSeg1==120&&numSeg2==100)||(numSeg1==100&&numSeg2==120))
	{
		int aaa = 0;
	}
	
	//search segment 1
//	int temSegNum;
	for (itrPnl=pnlSeg1.begin(); itrPnl!=pnlSeg1.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;

		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			itrTemLpnt = gLaserPnts.begin()+itrEdge->Number();
			//temSegNum = itrTemLpnt->Attribute(SegmentNumberTag);
			//if (numSeg2 == itrTemLpnt->Attribute(SegmentNumberTag)
			if (numSeg2 == itrTemLpnt->Attribute(SegmentNumberTag)
				&& itrLPnt->Position3DRef().Distance(itrTemLpnt->Position3DRef()) < fMinDistance
              ){ 
				bNeib = true;	
				break;
            }
		}
		
		if (bNeib) 
           pnlNeibPnts.push_back(*itrPnl);
	}

	//search segment 2
	for (itrPnl=pnlSeg2.begin(); itrPnl!=pnlSeg2.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;
		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			itrTemLpnt = gLaserPnts.begin()+itrEdge->Number();
			if (numSeg1 == itrTemLpnt->Attribute(SegmentNumberTag)
				&& itrLPnt->Position3DRef().Distance(itrTemLpnt->Position3DRef()) < fMinDistance
             ) {
				bNeib = true;	
				break;
            }
		}

		if (bNeib) 
           pnlNeibPnts.push_back(*itrPnl);
	}
	
	if ((int)pnlNeibPnts.size() >= nMinAdjPnts)
		return true;
	else
		return false;
}

//Feb 27, 2013, Biao Xiong
//usually check whether the two segments are adjacent on 2D,
//therefore here only check 2d distance
IEDll_API bool IsAdjacent2D(const LaserPoints& gLaserPnts, const TINEdges& gEdges, const PointNumberList& pnlSeg1, 
	const PointNumberList& pnlSeg2, double fMinDistance, int nMinAdjPnts)
{
	PointNumberList::const_iterator itrPnl;
	LaserPoints::const_iterator itrLPnt, itrTemLpnt;
	itrLPnt = gLaserPnts.begin()+pnlSeg1.begin()->Number();
	int numSeg1 = itrLPnt->Attribute(SegmentNumberTag);
	itrLPnt = gLaserPnts.begin()+pnlSeg2.begin()->Number();
	int numSeg2 = itrLPnt->Attribute(SegmentNumberTag);
	TINEdges::const_iterator itrEdgeSet;
	PointNumberList pnlNeibPnts;
	TINEdgeSet::const_iterator itrEdge;
	bool bNeib;

	//search segment 1
	//	int temSegNum;
	for (itrPnl=pnlSeg1.begin(); itrPnl!=pnlSeg1.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;

		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			itrTemLpnt = gLaserPnts.begin()+itrEdge->Number();
			if (numSeg2 != itrTemLpnt->Attribute(SegmentNumberTag)) continue;

			if (itrLPnt->Position2DOnly().Distance(itrTemLpnt->Position2DOnly())< fMinDistance)
			{ 
				bNeib = true;	
				break;
			}
		}

		if (bNeib) pnlNeibPnts.push_back(*itrPnl);
	}

	//search segment 2
	for (itrPnl=pnlSeg2.begin(); itrPnl!=pnlSeg2.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;
		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			itrTemLpnt = gLaserPnts.begin()+itrEdge->Number();
			if (numSeg1 == itrTemLpnt->Attribute(SegmentNumberTag)
				&& itrLPnt->Position2DOnly().Distance(itrTemLpnt->Position2DOnly())<fMinDistance) 
			{
				bNeib = true;	
				break;
			}
		}

		if (bNeib) pnlNeibPnts.push_back(*itrPnl);
	}

	if ((int)pnlNeibPnts.size() >= nMinAdjPnts)
		return true;
	else
		return false;
}


//new version
//Biao Xiong, April 4, 2012
//Use temperate storage of segment feature to accelerate indexing speed
/*
IEDll_API bool IsAdjacent(const LaserPoints& gLaserPnts, const TINEdges& gEdges, const PointNumberList& pnlSeg1, 
	const PointNumberList& pnlSeg2, double fMinDistance, int nMinAdjPnts)
{
	PointNumberList::const_iterator itrPnl;
	LaserPoints::const_iterator itrLPnt, itrTemLpnt;
//	itrLPnt = gLaserPnts.begin()+pnlSeg1.begin()->Number();
//	int numSeg1 = itrLPnt->Attribute(SegmentNumberTag);
//	itrLPnt = gLaserPnts.begin()+pnlSeg2.begin()->Number();
//	int numSeg2 = itrLPnt->Attribute(SegmentNumberTag);
	int numSeg1 = 1, numSeg2 =2;
	TINEdges::const_iterator itrEdgeSet;
	PointNumberList pnlNeibPnts;
	TINEdgeSet::const_iterator itrEdge;
	bool bNeib;
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//initial the segment number
	map<int, int> mapSegNum;
	int temPntNum;
	//segment 1
	for (itrPnl=pnlSeg1.begin(); itrPnl!=pnlSeg1.end(); ++itrPnl) {
		temPntNum = itrPnl->Number();
		mapSegNum[temPntNum] = numSeg1;
	}
	//segment 2
	for (itrPnl=pnlSeg2.begin(); itrPnl!=pnlSeg2.end(); ++itrPnl) {
		temPntNum = itrPnl->Number();
		mapSegNum[temPntNum] = numSeg2;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//search points on boundary
	//search segment 1
	//int temSegNum;
	for (itrPnl=pnlSeg1.begin(); itrPnl!=pnlSeg1.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;
		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			temPntNum = itrEdge->Number();
			itrTemLpnt = gLaserPnts.begin()+temPntNum;
			if (numSeg2 == mapSegNum[temPntNum]
			&& itrLPnt->Position3DRef().Distance(itrTemLpnt->Position3DRef()) < fMinDistance)
			{ 
					bNeib = true;	
					break;
			}
		}
		
		if (bNeib) 
           pnlNeibPnts.push_back(*itrPnl);
	}

	//search segment 2
	for (itrPnl=pnlSeg2.begin(); itrPnl!=pnlSeg2.end(); ++itrPnl) {
		itrEdgeSet = gEdges.begin()+itrPnl->Number();
		itrLPnt = gLaserPnts.begin()+itrPnl->Number();
		bNeib = false;
		for (itrEdge=itrEdgeSet->begin(); itrEdge!=itrEdgeSet->end(); ++itrEdge) {
			temPntNum = itrEdge->Number();
			itrTemLpnt = gLaserPnts.begin()+temPntNum;
			if (numSeg1 == mapSegNum[temPntNum]
			&& itrLPnt->Position3DRef().Distance(itrTemLpnt->Position3DRef()) < fMinDistance) 
			{
				bNeib = true;	
				break;
			}
		}

		if (bNeib) 
           pnlNeibPnts.push_back(*itrPnl);
	}
	
	if (pnlNeibPnts.size() >= nMinAdjPnts)
		return true;
	else
		return false;
}


//iterative search neighbor segments and detect intersection lines between them
//if two adjacent segments are parallel, or one is horizontal and the other one is much smaller,
//merge the two segments.
//old version. April 7, 2012
/*
IEDll_API bool ReconstructLocalSegments(LaserPoints& gLaserPnts, TINEdges& gEdge, 
	vector<PointNumberList>& gVecPnlSegs, vector<Plane>& vecLocalPlanes, 
	int curSegNum,	ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList	pnlCurSegPnts;
	vector<PointNumberList> vecAdjSegments;
	PointNumberList pnlTemSegPnts;
	Plane temPlane, curPlane;
	int curPlaneInd, temPlaneInd;
	bool bMerge = true;
	double pointRatio;
	Position3D startPos, endPos;
	double distance;
	int temSegNumber;
	int indTemSeg, indCurSeg;

	indCurSeg = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, curSegNum);
	assert(indCurSeg > -1);
	pnlCurSegPnts = gVecLocalSegPnts[indCurSeg];
	gLaserPnts.Label(pnlCurSegPnts, 1, IsProcessedTag);
	
	//when merge segments, the adjacent segments should search again
	while (bMerge) {
		//SearchAdjacentSegments(g_laserPnts, pnlCurSegPnts, vecAdjSegments, 10, 7);
		SearchAdjacentSegments(gLaserPnts, gEdge, gVecPnlSegs, 
			pnlCurSegPnts, vecAdjSegments, MIN_SEG_PNT_NUM, 7);

		curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curSegNum);
		if (curPlaneInd == -1) {//unfitted
			curPlane = gLaserPnts.FitPlane(pnlCurSegPnts);
			if (curPlane.IsVertical(10*PI/180))
				return true;

			//plane constraint
			if (USING_CONSTRAINT == 1
				&& ConstraintPlane(gLaserPnts, pnlCurSegPnts, curPlane))
				curPlane.Label() = 1;
			else
				curPlane.Label() = 0;

			curPlane.Number() = curSegNum;
			vecLocalPlanes.push_back(curPlane);
			curPlaneInd = vecLocalPlanes.size()-1;
		}
		else 
			curPlane = vecLocalPlanes[curPlaneInd];

		bMerge = false;
		for (int i=0; i<vecAdjSegments.size(); i++) {
			pnlTemSegPnts = vecAdjSegments[i];
			temSegNumber = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
			if (temSegNumber == curSegNum)//temp segment may have same number with current segment
				continue;
			indTemSeg = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, temSegNumber);
			if (indTemSeg != -1) //the temp segment point may already be changed, retrieval it from the localSegPnts
				pnlTemSegPnts = gVecLocalSegPnts[indTemSeg];

			temPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber);
			if (temPlaneInd == -1){//unfitted		
				temPlane = gLaserPnts.FitPlane(pnlTemSegPnts);
				if (temPlane.IsVertical(10*PI/180))
					continue;

				//plane constraint
				if (USING_CONSTRAINT == 1
					&& ConstraintPlane(gLaserPnts, pnlTemSegPnts, temPlane))
					temPlane.Label() = 1;
				else
					temPlane.Label() = 0;

				temPlane.Number() = temSegNumber;
				vecLocalPlanes.push_back(temPlane);
				temPlaneInd = vecLocalPlanes.size()-1;
			}
			else 
				temPlane = vecLocalPlanes[temPlaneInd];

			//Biao March 26, 2012
			//try to merge segments
			if (IsMergeAble(gLaserPnts,  pnlCurSegPnts, pnlTemSegPnts, curPlane, temPlane)) {
				indCurSeg = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, curSegNum);
				indTemSeg = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, temSegNumber);
				//re-tag temp segment
				for (int iPnt=0; iPnt<pnlTemSegPnts.size(); iPnt++) 
					gLaserPnts[pnlTemSegPnts[iPnt].Number()].SetAttribute(SegmentNumberTag, curSegNum);
				gLaserPnts.Label(pnlTemSegPnts, 1, IsProcessedTag);
			//	if(curSegNum == 2246 && temSegNumber==2418) {
			//		int aaa = 0;
			//	}
				pnlCurSegPnts.reserve(pnlCurSegPnts.size()+pnlTemSegPnts.size());
				pnlCurSegPnts.insert(pnlCurSegPnts.end(), pnlTemSegPnts.begin(), pnlTemSegPnts.end());
				gVecLocalSegPnts[indCurSeg] = pnlCurSegPnts;
				if (indTemSeg != -1)
					gVecLocalSegPnts.erase(gVecLocalSegPnts.begin() + indTemSeg);
				indCurSeg = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, curSegNum);
				pnlCurSegPnts = gVecLocalSegPnts[indCurSeg];
				
				temPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber);
				if (temPlaneInd != -1)
					vecLocalPlanes.erase(vecLocalPlanes.begin()+temPlaneInd);
				curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curSegNum);
				curPlane = gLaserPnts.FitPlane(pnlCurSegPnts);
				vecLocalPlanes[curPlaneInd].Normal()  = curPlane.Normal();
				vecLocalPlanes[curPlaneInd].Distance()  = curPlane.Distance();
				bMerge = true;

				int temSegNumber2, temPlaneInd2, temSegInd2;
				PointNumberList pnlTemSegPnts2;
				Plane temPlane2;
				vector<int> vecTemLines;

				//remove old line between current segment and temp segment
				SearchLinesOnPlane(localLineTops, curSegNum, vecTemLines);
				for (int iLine=0; iLine<vecTemLines.size(); iLine++) {
					temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(0);
					if (temSegNumber2 == curSegNum) 
						temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(1);

					if (temSegNumber2 == temSegNumber) {
						localLineTops.erase(localLineTops.begin()+vecTemLines[iLine]);
						break;
					}
				}

				//re-fit  old lines supported by current segment
				SearchLinesOnPlane(localLineTops, curSegNum, vecTemLines);
				for (int iLine=0; iLine<vecTemLines.size(); iLine++) {
					temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(0);
					if (temSegNumber2 == curSegNum) 
						temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(1);
					temPlaneInd2 = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber2);
					temSegInd2 = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, temSegNumber2);
					if (temPlaneInd2 == -1 || temSegInd2 == -1) continue;
					pnlTemSegPnts2 = gVecLocalSegPnts[temSegInd2];
					temPlane2 = vecLocalPlanes[temPlaneInd2];
					gLaserPnts.IntersectFaces(pnlCurSegPnts, pnlTemSegPnts2, curPlane, temPlane2, 
						INTERSECT_FACE_MIN_DIST, startPos, endPos);
					localObjPnts[localLineTops[vecTemLines[iLine]][0].Number()].Position3DRef() = startPos;
					localObjPnts[localLineTops[vecTemLines[iLine]][1].Number()].Position3DRef() = endPos;
					localLineTops[vecTemLines[iLine]].Attribute(0) = curSegNum;
					localLineTops[vecTemLines[iLine]].Attribute(1) = temSegNumber2;
				}

				//re-fit  old lines supported by temp segment
				SearchLinesOnPlane(localLineTops, temSegNumber, vecTemLines);
				LineTopology temLines2;
				for (int iLine=0; iLine<vecTemLines.size(); iLine++) {
					temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(0);
					if (temSegNumber2 == temSegNumber) 
						temSegNumber2 = localLineTops[vecTemLines[iLine]].Attribute(1);

					//remove redundant line
					//the temp segment2 connect with current segment, so this line is redundant
					if (SearchLineBy2Faces(localLineTops, curSegNum, temSegNumber2, temLines2)) {
						localLineTops.erase(localLineTops.begin()+vecTemLines[iLine]);
						SearchLinesOnPlane(localLineTops, temSegNumber, vecTemLines);
						iLine--;
						continue;
					}
					temPlaneInd2 = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber2);
					temSegInd2 = IndexSegBySegNumber(gLaserPnts, gVecLocalSegPnts, temSegNumber2);
					if (temPlaneInd2 == -1 || temSegInd2 == -1) continue;
					pnlTemSegPnts2 = gVecLocalSegPnts[temSegInd2];
					temPlane2 = vecLocalPlanes[temPlaneInd2];
					gLaserPnts.IntersectFaces(pnlCurSegPnts, pnlTemSegPnts2, curPlane, temPlane2, 
						INTERSECT_FACE_MIN_DIST, startPos, endPos);
					localObjPnts[localLineTops[vecTemLines[iLine]][0].Number()].Position3DRef() = startPos;
					localObjPnts[localLineTops[vecTemLines[iLine]][1].Number()].Position3DRef() = endPos;
					localLineTops[vecTemLines[iLine]].Attribute(0) = curSegNum;
					localLineTops[vecTemLines[iLine]].Attribute(1) = temSegNumber2;
				}

				continue; //go to process next adjacent segment
			}

//			if (pnlCurSegPnts.size() < MIN_SEG_PNT_NUM
//				|| pnlTemSegPnts.size() < MIN_SEG_PNT_NUM)
//				continue;//one segment is too small
			LineTopology temTopline;
			if(SearchLineBy2Faces(localLineTops, curSegNum, temSegNumber, temTopline))
				continue;//the line formed by current segment and temp segment is already added

			//get intersection line
			gLaserPnts.IntersectFaces(pnlCurSegPnts, pnlTemSegPnts, curPlane, temPlane, 
				INTERSECT_FACE_MIN_DIST, startPos, endPos);
			distance = startPos.Distance(endPos);

			if (startPos.Distance(endPos) < 0.6 )
				continue;//intersection line is too short

			if (USING_CONSTRAINT == 1)
			{//inter sect line constraint
				Plane plan1 = curPlane;
				Plane plan2 = temPlane;

				plan1.Label() = 1;
				plan1.Number() = curSegNum;
				vecLocalPlanes[curPlaneInd] = plan1;

				plan2.Label() = 1;
				plan2.Number() = temSegNumber;
				vecLocalPlanes[temPlaneInd] = plan2;
			}

			//out put
			AddTopLine(localObjPnts, localLineTops, startPos, endPos, curSegNum, temSegNumber, -1);

			//Iterative search and reconstruct all segments neighbor with this segments 
			//And their neighbors
			if (indTemSeg == -1)
				gVecLocalSegPnts.push_back(pnlTemSegPnts);
			ReconstructLocalSegments(gLaserPnts, gEdge, gVecPnlSegs, vecLocalPlanes,
				temSegNumber, localObjPnts, localLineTops);
		}
	}

	return true;
}*/

vector<LineSegment3D> ReconstructStepEdges(LaserPoints& gLaserPnts, TINEdges& gEdge, 
	std::vector<Plane>& vecLocalPlanes, PointNumberList& pnlSeg1,PointNumberList& pnlSeg2)
{
	PointNumberList pnlLSeg1 = pnlSeg1; 
	PointNumberList	pnlLSeg2 = pnlSeg2;

	vector<LineSegment3D> vecLineSegs;
	LineSegment3D lineSeg3D;
	Position3D pos3d11, pos3d12, pos3d21, pos3d22;
	Line3D wallLine;
	//vector<Plane> result;
	int segNum1 = gLaserPnts[pnlLSeg1[0].Number()].Attribute(SegmentNumberTag);
	int segNum2 = gLaserPnts[pnlLSeg2[0].Number()].Attribute(SegmentNumberTag);
	if ((segNum1==2033&&segNum2==2043)||(segNum1==2043 && segNum2==2033))	{
		int aaa =0;
	}

	int indPlnae1 = IndexPlaneBySegNumber(vecLocalPlanes, segNum1);
	int indPlnae2 = IndexPlaneBySegNumber(vecLocalPlanes, segNum2);
	if (indPlnae1==-1||indPlnae2==-1) return vecLineSegs;

	Plane plane1 = vecLocalPlanes[indPlnae1];
	Plane plane2 = vecLocalPlanes[indPlnae2];

	Vector3D plumb(0.0,0.0,1.0);
	//make sure plane1 more sloper 
	plane1.Normal().Normalize();
	plane2.Normal().Normalize();
	if (fabs(plane1.Normal().Z())>fabs(plane2.Normal().Z())) {
		std::swap(pnlLSeg1, pnlLSeg2);
		std::swap(plane1, plane2);
		std::swap(indPlnae1, indPlnae2);
		std::swap(segNum1, segNum2);
	}
	//////////////////////////////////////////////////////////////////////////
	//collect border points
	bool bBorder;
	vector<int> vecBorners;
	int temSegNum;
	LaserPoints::iterator pnt1, pnt2;
	for (int i=0; i<pnlLSeg1.size(); i++) {
		TINEdgeSet& edgeSet = gEdge[pnlLSeg1[i].Number()];
		bBorder = false;
		pnt1 = gLaserPnts.begin()+pnlLSeg1[i].Number();
		for (int j=0; j<edgeSet.size(); ++j) {
			int& pntNum = edgeSet[j].Number();
			temSegNum = gLaserPnts[pntNum].Attribute(SegmentNumberTag);
			if(temSegNum!=segNum2) continue;
			if (pnt1->Position2DOnly().Distance(gLaserPnts[pntNum].Position2DOnly())>0.8)
				continue;
			bBorder = true;
			vecBorners.push_back(pntNum);
			break;
		}
		if (bBorder) vecBorners.push_back(pnlLSeg1[i].Number());
	}

	std::sort(vecBorners.begin(), vecBorners.end());
	vector<int>::iterator itr = std::unique(vecBorners.begin(), vecBorners.end());
	vecBorners = vector<int>(vecBorners.begin(), itr);
	LaserPoints borderCloud;
	for (int i=0; i<vecBorners.size(); ++i) {
		borderCloud.push_back(gLaserPnts[vecBorners[i]]);
		borderCloud[i].Z() = 0.0;
	}

#ifdef _DEBUG
	borderCloud.Write("debug.laser");
#endif

	//////////////////////////////////////////////////////////////////////////
	//use hough transform to collect 2D lines
	DataBounds2D bounds;
	for (int i=0; i<borderCloud.size();i++)
		bounds.Update(borderCloud[i].Position2DOnly());
	//initialize
	HoughSpace houghspace;
	double maxDistHough = sqrt(bounds.XRange()*bounds.XRange()+bounds.YRange()*bounds.YRange())/2.0;
	//if one plan is unhorizontal, use its horizontal eave as constraints
	if(!plane1.IsHorizontal(20*PI/180)) {
		Vector3D conDir;
		conDir = plane1.Normal().VectorProduct(plumb);
		double pDir[2];
		pDir[0] = atan(conDir.Y()/(conDir.X()+0.00001));
		pDir[1] = pDir[0]+PI/2.0;
		houghspace.Initialise(2, pDir, int(2*maxDistHough/0.1), -maxDistHough, maxDistHough);
	}
	else
		houghspace.Initialise(int(180), int(2*maxDistHough/0.1), 0.0, -maxDistHough, PI, maxDistHough, 1);
	
	houghspace.SetOffsets(bounds.MidPoint().X(), bounds.MidPoint().Y(), 0.0);
	for (int i=0; i<borderCloud.size(); i++)
		houghspace.AddPoint(borderCloud[i].Position2DOnly());

	Line2D line;
	int pntCount;
	PointNumberList pnlLineSeg;
	double dist;
	double minDist = 0.35;
	LaserPoints::iterator laser;
	Position2D pos2D1, pos2D2;
	LineSegment2D lineSeg;
	LineSegments2D vecLineSegs2D;

	line = houghspace.BestLine(&pntCount, 0, 9);
	while(pntCount>1.2*gBldRecPar.minRidgeNum) 
	{
		//and get points of corresponding line segment
		pntCount = 0;
		pnlLineSeg.clear();

		for (int i=0; i<borderCloud.size(); i++) {
			dist = line.DistanceToPoint(borderCloud[i].Position2DOnly());
			if (dist > minDist) continue;
			pntCount++;
			pnlLineSeg.push_back(PointNumber(i));
		}

		//fit the line again
		//if (!MyFitLine(borderCloud, pnlLineSeg, line)) continue;
		double scalar, minScalar = 9999999, maxScalar = -99999999;
		for (int i=0; i<pnlLineSeg.size(); i++)	{
			laser = borderCloud.begin()+pnlLineSeg[i].Number();
			houghspace.RemovePoint(laser->Position2DOnly());
			scalar = line.Scalar(laser->Position2DOnly());
			if (scalar < minScalar) minScalar = scalar;
			if (scalar > maxScalar) maxScalar = scalar;
		}

#ifdef _DEBUG
		pos2D1 = line.Position(minScalar);
		pos3d11.X()=pos2D1.X(); pos3d11.Y()=pos2D1.Y(); pos3d11.Z() = 0.0;
		pos2D2 = line.Position(maxScalar);
		pos3d12.X()=pos2D2.X(); pos3d12.Y()=pos2D2.Y(); pos3d12.Z() = 0.0;
		LineTopologies debugTops;
		ObjectPoints debugObjPnts;
		AddTopLine(debugObjPnts, debugTops, pos3d11, pos3d12);
		debugTops.Write("debug.top");
		debugObjPnts.Write("debug.objpts");
#endif

		lineSeg = LineSegment2D(line, minScalar, maxScalar);
		vecLineSegs2D.push_back(lineSeg);
		line = houghspace.BestLine(&pntCount, 0, 9);
	}  

	//////////////////////////////////////////////////////////////////////////
	//remove redundant lines
	//when two lines are parallel and within 1 meter distance, one is redundant
	//the step lines which are on the highest roof face and have lowest hight 
	//are the most stable one, and kept

	//make sure first segment is higher than second segment
	if (gLaserPnts[pnlLSeg1[0].Number()].Z()<gLaserPnts[pnlLSeg2[0].Number()].Z()) {
		std::swap(pnlLSeg1, pnlLSeg2);
		std::swap(plane1, plane2);
		std::swap(indPlnae1, indPlnae2);
		std::swap(segNum1, segNum2);
	}

	LineSegments2D::iterator lSeg1, lSeg2;
	for (int i=0;i<vecLineSegs2D.size(); i++) {
		lSeg1 = vecLineSegs2D.begin()+i;
		pos2D1 = lSeg1->BeginPoint();
		pos3d11.X()=pos2D1.X(); pos3d11.Y()=pos2D1.Y(); pos3d11.Z() = 0.0;
		wallLine = Line3D(pos3d11, Vector3D(0.0,0.0,1.0));
		IntersectLine3DPlane(wallLine, plane1, pos3d11);

		for (int j=i+1; j<vecLineSegs2D.size(); j++) {
			lSeg2 = vecLineSegs2D.begin()+j;
			//no parallel, keep both
			if (Angle2Lines(*lSeg1, *lSeg2)>10*PI/180) continue;
			dist = lSeg1->DistanceToPoint(lSeg2->BeginPoint());
			dist += lSeg1->DistanceToPoint(lSeg2->EndPoint());
			if (dist>2.0) continue;//too far, keep both

			pos2D2 = lSeg2->BeginPoint();
			pos3d21.X()=pos2D2.X(); pos3d21.Y()=pos2D2.Y(); pos3d21.Z() = 0.0;
			wallLine = Line3D(pos3d21, Vector3D(0.0,0.0,1.0));
			IntersectLine3DPlane(wallLine, plane1, pos3d21);

			//step edge1 is higher than step edge 2
			if (pos3d11.Z()>pos3d21.Z()) {
				std::swap(vecLineSegs2D[i], vecLineSegs2D[j]);
				std::swap(pos3d11, pos3d21);
			}

			vecLineSegs2D.erase(lSeg2);
			j--;
		}
	}


	///////////////////////////////////////////////////////////////////////////
	//get step edges
	for (int i=0;i<vecLineSegs2D.size(); i++) {
		lineSeg = vecLineSegs2D[i];
		//wall 1
		pos2D1 = lineSeg.BeginPoint();
		pos3d11.X()=pos2D1.X(); pos3d11.Y()=pos2D1.Y(); pos3d11.Z() = 0.0;
		wallLine = Line3D(pos3d11, Vector3D(0.0,0.0,1.0));
		IntersectLine3DPlane(wallLine, plane1, pos3d11);
		IntersectLine3DPlane(wallLine, plane2, pos3d21);
		//wall 2
		pos2D2 = lineSeg.EndPoint();
		pos3d12.X()=pos2D2.X(); pos3d12.Y()=pos2D2.Y(); pos3d12.Z() = 0.0;
		wallLine = Line3D(pos3d12, Vector3D(0.0,0.0,1.0));
		IntersectLine3DPlane(wallLine, plane1, pos3d12);
		IntersectLine3DPlane(wallLine, plane2, pos3d22);
		//step edge 1
		lineSeg3D = LineSegment3D(pos3d11, pos3d12); 
		lineSeg3D.Number() = segNum1;
		vecLineSegs.push_back(lineSeg3D);
		//step edge 2
		lineSeg3D = LineSegment3D(pos3d21, pos3d22); 
		lineSeg3D.Number() = segNum2;
		vecLineSegs.push_back(lineSeg3D);
	}

	return vecLineSegs;
}

//April 7, 2012 Biao Xiong
//search adjacent roof segments
//but do not store the accurate intersection location
IEDll_API bool ReconstructLocalSegments(LaserPoints& gLaserPnts, TINEdges& gEdge, 
	std::vector<PointNumberList>& gVecPnlSegs, std::vector<Plane>& vecLocalPlanes, 
	PointNumberList pnlCurSegPnts,
	ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	if (pnlCurSegPnts.size() < gBldRecPar.minSegNum)
		return false;

	vector<PointNumberList> vecAdjSegments;
	PointNumberList pnlTemSegPnts;
	Plane temPlane, curPlane;
	int curPlaneInd, temPlaneInd;
	Position3D startPos, endPos;
	int curSegNum, temSegNum;
//	int indTemSeg, indCurSeg;

	curSegNum = gLaserPnts[pnlCurSegPnts[0].Number()].Attribute(SegmentNumberTag);
	gLaserPnts.Label(pnlCurSegPnts, 1, IsProcessedTag);
	gVecLocalSegPnts.push_back(pnlCurSegPnts);

	//SearchAdjacentSegments(g_laserPnts, pnlCurSegPnts, vecAdjSegments, 10, MIN_RIDGE_PNT_NUM);
	SearchAdjacentSegments(gLaserPnts, gEdge, gVecPnlSegs, 
		pnlCurSegPnts, vecAdjSegments, gBldRecPar.minSegNum, gBldRecPar.minRidgeNum);

	curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curSegNum);
	if (curPlaneInd == -1) {//unfitted
		curPlane = gLaserPnts.FitPlane(pnlCurSegPnts);
		if (curPlane.IsVertical(10*PI/180))
			return true;

		curPlane.Label() = 0;
		curPlane.Number() = curSegNum;
		vecLocalPlanes.push_back(curPlane);
		curPlaneInd = vecLocalPlanes.size()-1;
	}
	else 
		curPlane = vecLocalPlanes[curPlaneInd];

	for (int i=0; i<(int)vecAdjSegments.size(); i++) {
		pnlTemSegPnts = vecAdjSegments[i];
		temSegNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		temPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, temSegNum);
		if (temPlaneInd == -1){//unfitted		
			temPlane = gLaserPnts.FitPlane(pnlTemSegPnts);
			if (temPlane.IsVertical(10*PI/180))
				continue;

			temPlane.Label() = 0;
			temPlane.Number() = temSegNum;
			vecLocalPlanes.push_back(temPlane);
			temPlaneInd = vecLocalPlanes.size()-1;
		}
		else 
			temPlane = vecLocalPlanes[temPlaneInd];

		//intersection line
		if(IsAdjacent(gLaserPnts,gEdge,pnlCurSegPnts,pnlTemSegPnts,gBldRecPar.minPntDist,gBldRecPar.minRidgeNum))
		{
			//Jan 10, 2013, the line position should be added
			//get intersection line
			gLaserPnts.IntersectFaces(pnlCurSegPnts, pnlTemSegPnts, curPlane, temPlane, 
				gBldRecPar.minPntDist, startPos, endPos);
			//			distance = startPos.Distance(endPos);
			//			if (startPos.Distance(endPos) < 0.6 )
			//				continue;//intersection line is too short

			//out put
			AddTopLine(localObjPnts, localLineTops, startPos, endPos, curSegNum, temSegNum, -1);
		}
		else if(gBldRecPar.bStepEdges){//step lines
			vector<LineSegment3D> steps;
			steps = ReconstructStepEdges(gLaserPnts,gEdge,vecLocalPlanes,pnlCurSegPnts,pnlTemSegPnts);
			for (int iS=0; iS<steps.size(); iS++) {
				AddTopLine(localObjPnts, localLineTops, steps[iS].BeginPoint(), 
					steps[iS].EndPoint(), steps[iS].Number(), -1, -1);
			}
		}

		//Iterative search and reconstruct all segments neighbor with this segments 
		//And their neighbors
		gVecLocalSegPnts.push_back(pnlTemSegPnts);
		ReconstructLocalSegments(gLaserPnts, gEdge, gVecPnlSegs, vecLocalPlanes, 
			pnlTemSegPnts, localObjPnts, localLineTops);
	}

	return true;
}

IEDll_API bool MergeLocalSegments(LaserPoints& gLaserPnts, vector<PointNumberList>& vecLocalSegPnts, 
	std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	if (gVecLocalSegPnts.size() < 2)
		return true;

	vector<PointNumberList> vecAdjSegments;
	PointNumberList pnlSegPnts1, pnlSegPnts2;
	Plane Plane1, Plane2;
	int indPlane1, indPlane2;
//	double pointRatio;
	Position3D startPos, endPos;
//	double distance;
	int segNum1, segNum2;
	int indSeg1, indSeg2;
	LineTopologies::iterator topLine;
	int temSegNumber2, temPlaneInd2, temSegInd2;
	PointNumberList pnlTemSegPnts2;
	Plane temPlane2;
	vector<int> vecTemLines;
	vector<char> vecLineValid(localLineTops.size(), true);

	for (int iLine=0; iLine<(int)localLineTops.size(); ++iLine)	{
		if (!vecLineValid[iLine])//invalid
			continue;

		topLine = localLineTops.begin() + iLine;
		segNum1 = topLine->Attribute(SegmentLabel);
		segNum2 = topLine->Attribute(SecondSegmentLabel);
		if (segNum1 == -1 || segNum2 == -1)
			continue;

		indSeg1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum1);
		indSeg2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum2);
		indPlane1 = IndexPlaneBySegNumber(vecLocalPlanes, segNum1);
		indPlane2 = IndexPlaneBySegNumber(vecLocalPlanes, segNum2);

		if (indSeg1 == -1 || indSeg2 == -1 || indPlane1==-1 || indPlane2==-1)
			continue;

		pnlSegPnts1 = vecLocalSegPnts[indSeg1];
		pnlSegPnts2 = vecLocalSegPnts[indSeg2];
		Plane1 = vecLocalPlanes[indPlane1];
		Plane2 = vecLocalPlanes[indPlane2];

	//	PointNumberList pnlSegPnts3 = pnlSegPnts1;
	//	pnlSegPnts3.insert(pnlSegPnts3.end(), pnlSegPnts2.begin(), pnlSegPnts2.end());
	//	for (int jj=0; jj<pnlSegPnts2.size(); jj++) {
	//		pnlSegPnts3.push_back(pnlSegPnts2[jj]);
	//	}
		
//		vector<int> testvec1 = vector<int>(100, 1);
//		vector<int> testvec2 = testvec1;
//		testvec2.insert(testvec2.end(), testvec1.begin(), testvec1.end());

		//make the large segment to be kept
	/*	if (vecLocalSegPnts[indSeg2].size() > vecLocalSegPnts[indSeg1].size() ) {
			std::swap(segNum1, segNum2);
			std::swap(indSeg1, indSeg2);
			std::swap(pnlSegPnts1, pnlSegPnts2);
			std::swap(Plane1, Plane2);
		}*/

		if ((segNum1==2&&segNum2==4)||(segNum1==4&&segNum2==2)){
			int aaa = 0;
		}
			
		if (IsDeleteAble(gLaserPnts,  pnlSegPnts1, pnlSegPnts2, Plane1, Plane2)) {
			//the top line can be deleted
			//localLineTops.erase(topLine);
			vecLineValid[iLine] = false;
		}
		else if (IsMergeAble(gLaserPnts,  pnlSegPnts1, pnlSegPnts2, Plane1, Plane2)) {
			//refit top line
			//merge the two supporting roof segments
			//remove current top line
			vecLineValid[iLine] = false;

			//arrange the laser points and segment point number point
			for (int iPnt=0; iPnt<(int)pnlSegPnts2.size(); iPnt++) 
				gLaserPnts[pnlSegPnts2[iPnt].Number()].SetAttribute(SegmentNumberTag, segNum1);
			pnlSegPnts1.reserve(pnlSegPnts1.size()+pnlSegPnts2.size());
			pnlSegPnts1.insert(pnlSegPnts1.end(), pnlSegPnts2.begin(), pnlSegPnts2.end());
			vecLocalSegPnts.erase(vecLocalSegPnts.begin()+indSeg2);
			indSeg1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, segNum1);
			vecLocalSegPnts[indSeg1] = pnlSegPnts1;
			indSeg2 = -1;

			//Plane1 = gLaserPnts.FitPlane(pnlSegPnts1);
			Plane1 = gLaserPnts.FitPlane(vecLocalSegPnts[indSeg1]);
			vecLocalPlanes[indPlane1].Normal()  = Plane1.Normal();
			vecLocalPlanes[indPlane1].Distance()  = Plane1.Distance();

			//re-fit  old lines supported by segment 1
			SearchLinesOnPlane(localLineTops, segNum1, vecTemLines);
			for (int jLine=0; jLine<(int)vecTemLines.size(); jLine++) {
				if (!vecLineValid[vecTemLines[jLine]]) continue;
				temSegNumber2 = localLineTops[vecTemLines[jLine]].Attribute(SegmentLabel);
				if (temSegNumber2 == segNum1) 
					temSegNumber2 = localLineTops[vecTemLines[jLine]].Attribute(SecondSegmentLabel);
				temPlaneInd2 = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber2);
				temSegInd2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, temSegNumber2);
				if (temPlaneInd2 == -1 || temSegInd2 == -1) continue;
				pnlTemSegPnts2 = vecLocalSegPnts[temSegInd2];
				temPlane2 = vecLocalPlanes[temPlaneInd2];
				gLaserPnts.IntersectFaces(pnlSegPnts1, pnlTemSegPnts2, Plane1, temPlane2, 
					gBldRecPar.minPntDist, startPos, endPos);
				localObjPnts[localLineTops[vecTemLines[jLine]][0].Number()].Position3DRef() = startPos;
				localObjPnts[localLineTops[vecTemLines[jLine]][1].Number()].Position3DRef() = endPos;
				localLineTops[vecTemLines[jLine]].Attribute(SegmentLabel) = segNum1;
				localLineTops[vecTemLines[jLine]].Attribute(SecondSegmentLabel) = temSegNumber2;
			}

			//re-fit  old lines supported by temp segment
			SearchLinesOnPlane(localLineTops, segNum2, vecTemLines);
			//LineTopology temLines2;
			int indTemLine2;
			for (int jLine=0; jLine<(int)vecTemLines.size(); jLine++) {
				if (!vecLineValid[vecTemLines[jLine]]) continue;
				temSegNumber2 = localLineTops[vecTemLines[jLine]].Attribute(SegmentLabel);
				if (temSegNumber2 == segNum2) 
					temSegNumber2 = localLineTops[vecTemLines[jLine]].Attribute(SecondSegmentLabel);

				//remove redundant line
				//the temp segment2 connect with segment 1, so this line is redundant
				if (SearchLineBy2Faces(localLineTops, segNum1, temSegNumber2, indTemLine2)) {
					//localLineTops.erase(localLineTops.begin()+vecTemLines[jLine]);
					//SearchLinesOnPlane(localLineTops, segNum2, vecTemLines);//rearrange the line number
					//jLine--;
					//iLine --;
					vecLineValid[vecTemLines[jLine]] = false;
					continue;
				}

				temPlaneInd2 = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber2);
				temSegInd2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, temSegNumber2);
				if (temPlaneInd2 == -1 || temSegInd2 == -1) continue;
				pnlTemSegPnts2 = vecLocalSegPnts[temSegInd2];
				temPlane2 = vecLocalPlanes[temPlaneInd2];
				gLaserPnts.IntersectFaces(pnlSegPnts1, pnlTemSegPnts2, Plane1, temPlane2, 
					gBldRecPar.minPntDist, startPos, endPos);
				localObjPnts[localLineTops[vecTemLines[jLine]][0].Number()].Position3DRef() = startPos;
				localObjPnts[localLineTops[vecTemLines[jLine]][1].Number()].Position3DRef() = endPos;
				localLineTops[vecTemLines[jLine]].Attribute(SegmentLabel) = segNum1;
				localLineTops[vecTemLines[jLine]].Attribute(SecondSegmentLabel) = temSegNumber2;
			}
		}
		else {
			gLaserPnts.IntersectFaces(pnlSegPnts1, pnlSegPnts2, 
				Plane1, Plane2, gBldRecPar.minPntDist, startPos, endPos);
		//	double len = startPos.Distance(endPos);
			if (startPos.Distance(endPos) < gBldRecPar.minRidgeLen)//this line is too short
				vecLineValid[iLine] = false;
			localObjPnts[(*topLine)[0].Number()].Position3DRef() = startPos;
			localObjPnts[(*topLine)[1].Number()].Position3DRef() = endPos;
		}//end operation
	}

	//////////////////////////////////////////////////////////////////////////
	//get valid top line
	LineTopologies temLineTops, invalidLineTops;
	for (int iLine=0; iLine<localLineTops.size(); ++iLine) {
		if (vecLineValid[iLine])
			temLineTops.push_back(localLineTops[iLine]);
		else
			invalidLineTops.push_back(localLineTops[iLine]);
	}

	localLineTops = temLineTops;

	//////////////////////////////////////////////////////////////////////////
	//retag invalid segments
	std::vector<int> vecValidNum = CollectRoofFacesByLines(localLineTops);
	std::vector<int> vecInvalidNum = CollectRoofFacesByLines(invalidLineTops);
	std::vector<int>::iterator itrNum;
	for (unsigned int i=0; i<vecInvalidNum.size(); i++) {
		itrNum = std::find(vecValidNum.begin(), vecValidNum.end(), vecInvalidNum[i]);
		if (vecValidNum.end()!=itrNum) {//this segment is valid
			vecInvalidNum.erase(vecInvalidNum.begin()+i);
			i--;
		}
	}

	//if all top lines are deleted, keep the first segment
	if (vecValidNum.empty()) {
		vecValidNum.push_back(segNum1);
		vecInvalidNum.erase(std::find(vecInvalidNum.begin(), vecInvalidNum.end(), segNum1));
	}

	int indPnl;
	//vector<PointNumberList>& gVecSegPnl, 
	for (unsigned int i=0; i<vecInvalidNum.size(); i++) {
		indPnl = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, vecInvalidNum[i]);
		if (indPnl==-1) continue;
		
		gLaserPnts.Label(vecLocalSegPnts[indPnl], -1, IsProcessedTag);
		vecLocalSegPnts.erase(vecLocalSegPnts.begin()+indPnl);
	}

	return true;
}

IEDll_API bool ConstraintPlane(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, Plane& plane, double minAngle)
{
	Vector3D normal;
	bool bFlag = false;

	if (plane.IsHorizontal(minAngle))
	{
		normal = Vector3D(0,0,1);
		plane= gLaserPnts.FitPlane (pnlSegPnts, normal, plane.Number());
		bFlag = true;
	}
	else if(plane.IsVertical(minAngle))
	{
		normal = plane.Normal();
		normal = Vector3D(normal.X(), normal.Y(), 0);
		normal = normal.Normalize();
		plane= gLaserPnts.FitPlane(pnlSegPnts, normal, plane.Number());
		bFlag = true;
	}

	return bFlag;
}

IEDll_API bool ConstraintIntersecLine(const LaserPoints& gLaserPnts, const PointNumberList& pnlLeftSeg, const PointNumberList& pnlRightSeg,
	Plane& leftPlane, Plane& rightPlane, Position3D& startPos, Position3D& endPos,	double minAngle)
{
	//one plane is already horizontal/vertical
	//the intersect line is sure to be horizontal/vertical
	//it is not needed to constraint the intersect line
	if (leftPlane.IsHorizontal(minAngle) || rightPlane.IsHorizontal(minAngle)
		|| leftPlane.IsVertical(minAngle) || rightPlane.IsVertical(minAngle) )
		return false;

	Vector3D direction = (Vector3D)startPos - (Vector3D)endPos;
	direction.Normalize();

	if (direction.Z() < 0)
		direction *= -1;

	bool bFlag = false;

	if ( direction.DotProduct(Vector3D(0,0,1)) < sin(minAngle))
	{//horizontal intersect line
		Vector3D leftNormal = leftPlane.Normal();
		Vector3D rightNormal = rightPlane.Normal();
		double sigmal;

		if (leftNormal.Z() < 0)
			leftNormal *= -1;

		if (rightNormal.Z() < 0)
			rightNormal *= -1;

		//constraint the two planes intersect vertical line with same angle
		if (fabs(leftNormal.Z()-rightNormal.Z()) < sin(minAngle))
		{
			leftNormal.Z() = (leftNormal.Z()+rightNormal.Z())/2;
			leftNormal.Z() = rightNormal.Z();
		}

		//constraint the intersect line to be horizontal
		if ( fabs(leftNormal.X()) > fabs(leftNormal.Y()) )
		{
			sigmal = (leftNormal.Y()/leftNormal.X() + rightNormal.Y()/rightNormal.X())/2;
			leftNormal.Y() = sigmal*leftNormal.X();
			rightNormal.Y() = sigmal*rightNormal.X();
		}
		else
		{
			sigmal = (leftNormal.X()/leftNormal.Y() + rightNormal.X()/rightNormal.Y())/2;
			leftNormal.X() = sigmal*leftNormal.Y();
			rightNormal.X() = sigmal*rightNormal.Y();
		}

		leftNormal.Normalize();
		rightNormal.Normalize();
		leftPlane = gLaserPnts.FitPlane(pnlLeftSeg, leftNormal, leftPlane.Number());
		rightPlane = gLaserPnts.FitPlane(pnlRightSeg, rightNormal, rightPlane.Number());

		gLaserPnts.IntersectFaces(pnlLeftSeg, pnlRightSeg, leftPlane, rightPlane, gBldRecPar.minPntDist, startPos, endPos);
		bFlag = true;
	}
	else if (direction.DotProduct(Vector3D(0,0,1)) < cos(minAngle))
	{//vertical intersect line
		//do nothing now
		bFlag = false;
	}
	else
	{//no constraint can be used
		bFlag = false;
	}

	return bFlag;
}

IEDll_API int IndexPlaneBySegNumber(const vector<Plane>& vecPlanes, int segNumber)
{
	int index = -1;
	vector<Plane>::const_iterator itrPlane;
	for (itrPlane=vecPlanes.begin(); itrPlane<vecPlanes.end(); ++itrPlane) 	{
		if (itrPlane->Number() == segNumber) {
			index = itrPlane-vecPlanes.begin();
			break;
		}
	}
	return index;
}

IEDll_API int IndexSegBySegNumber(const LaserPoints& gLaserPnts, const vector<PointNumberList>& vecLocalSegPnts, 
	int segNumber, LaserPointTag tag)
{
	int index = -1;
	LaserPoints::const_iterator itrPnt;
	for (int i=0; i<(int)vecLocalSegPnts.size(); i++) 	{
		if (vecLocalSegPnts[i].empty())
			return -1;
		itrPnt = gLaserPnts.begin()+vecLocalSegPnts[i][0].Number();
		if (segNumber == itrPnt->Attribute(tag)) {
			index = i;
			break;
		}
	}
	return index;
}

//try to find 3 faces intersect at one point 
//then hypotheses this intersect point
IEDll_API bool RefineCornerOf3Faces(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxAngle = 5*PI/180;
	double maxDistance = 10.0;

	vector<int> curAdjFaceNums;
	vector<int> temAdjFaceNums;
	vector<int> thirdFaceNums;//store the third face which can form 3face corner with iPlane and temFace;

	LineTopology lineTop1, lineTop2, lineTop3;
	int indLineTop1, indLineTop2, indLineTop3;
	Line3D line1, line2, line3, stableLine;
	Plane plane1, plane2, plane3, plane;
	Position3D intesectPnt;
	int pntNum11, pntNum12, pntNum21, pntNum22, pntNum31, pntNum32;
	int planeNum1, planeNum2, planeNum3;

	for (int iPlane=0; iPlane<(int)vecLocalPlanes.size(); ++iPlane)
		vecLocalPlanes[iPlane].Label() = -1;//-1 un-refined; 1 refined

	for (int iPlane=0; iPlane<vecLocalPlanes.size(); ++iPlane) 	{
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, curAdjFaceNums);

		for (int j=0; j<curAdjFaceNums.size(); ++j)	{
			planeNum2 = curAdjFaceNums[j];
			if (vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum2)].Label() != -1)
				continue;//refined

			SearchFaceBy2Faces(localLineTops, planeNum1, planeNum2, thirdFaceNums);

			for (int k=0; k<thirdFaceNums.size(); ++k)	{
				planeNum3 = thirdFaceNums[k];
				if (vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum3)].Label() != -1)
					continue;//refined

				if (!SearchLineBy2Faces(localLineTops, planeNum1, planeNum2, indLineTop1)
					|| !SearchLineBy2Faces(localLineTops, planeNum1, planeNum3, indLineTop2)
					|| !SearchLineBy2Faces(localLineTops, planeNum3, planeNum2, indLineTop3))
					continue;

				lineTop1 = localLineTops[indLineTop1];
				lineTop2 = localLineTops[indLineTop2];
				lineTop3 = localLineTops[indLineTop3];

				pntNum11 = lineTop1[0].Number();
				pntNum12 = lineTop1[1].Number();
				pntNum21 = lineTop2[0].Number();
				pntNum22 = lineTop2[1].Number();
				pntNum31 = lineTop3[0].Number();
				pntNum32 = lineTop3[1].Number();

				line1 = Line3D(localObjPnts[pntNum11] , localObjPnts[pntNum12]);
				line2 = Line3D(localObjPnts[pntNum21] , localObjPnts[pntNum22]);
				line3 = Line3D(localObjPnts[pntNum31] , localObjPnts[pntNum32]);
				plane1 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum1)];
				plane2 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum2)];
				plane3 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum3)];

				//find most stable line, which is vertical or horizontal
				if (Angle(line1.Direction(), Vector3D(0, 0, 1)) < maxAngle 
					|| line1.IsHorizontal(maxAngle) ) {
					stableLine = line1;
					plane = plane3;
				}
				else if (Angle(line2.Direction(), Vector3D(0, 0, 1)) < maxAngle 
					|| line2.IsHorizontal(maxAngle)) {
					stableLine = line2;
					plane = plane2;
				}
				else {
					stableLine = line3;
					plane = plane1;
				}

				//those three planes intersect at one point
				if (MyIntersectLine3DPlane(stableLine, plane, intesectPnt)) {
					//check whether another segment can form 3d-corner with any two of the three segments
					vector<int> thirdFaceNums2, thirdFaceNums3;
					SearchFaceBy2Faces(localLineTops, planeNum1, planeNum3, thirdFaceNums2);
					SearchFaceBy2Faces(localLineTops, planeNum2, planeNum3, thirdFaceNums3);
					thirdFaceNums3.insert(thirdFaceNums3.end(), thirdFaceNums.begin(), thirdFaceNums.end());
					thirdFaceNums3.insert(thirdFaceNums3.end(), thirdFaceNums2.begin(), thirdFaceNums2.end());		
					std::sort(thirdFaceNums3.begin(), thirdFaceNums3.end());
					int nRepeat = 0;
					bool bCross = false;
					for (int i=1; i<thirdFaceNums3.size(); ++i) {
						if (thirdFaceNums3[i] == thirdFaceNums3[i-1])
							nRepeat++;
						else
							nRepeat = 0;
						if (nRepeat==2)	{
							bCross = true;
							break;
						}
					}
					if (bCross)	{
						double dist1, dist2, dist3, dist11, dist12;
						dist11 = intesectPnt.Distance(localObjPnts[pntNum11]);
						dist12 = intesectPnt.Distance(localObjPnts[pntNum12]);
						dist1 = dist11<dist12 ? dist11:dist12;
						dist11 = intesectPnt.Distance(localObjPnts[pntNum21]);
						dist12 = intesectPnt.Distance(localObjPnts[pntNum22]);
						dist2 = dist11<dist12 ? dist11:dist12;
						dist11 = intesectPnt.Distance(localObjPnts[pntNum31]);
						dist12 = intesectPnt.Distance(localObjPnts[pntNum32]);
						dist3 = dist11<dist12 ? dist11:dist12;
						if (dist1>4.0 || dist2>4.0 || dist3>4.0)
							continue;//too far away ,skip this corner
					}

					ReplaceLineNode(localObjPnts, lineTop1, intesectPnt);
					ReplaceLineNode(localObjPnts, lineTop2, intesectPnt);
					ReplaceLineNode(localObjPnts, lineTop3, intesectPnt);
					
					/*double flag1, flag2, halfSpace1, halfSpace2, allHalfSpace;
					PointNumberList pnlSeg1, pnlSeg2, pnlSeg3;
					int indSeg1, indSeg2, indSeg3;
					indSeg1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, planeNum1);
					indSeg2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, planeNum2);
					indSeg3 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, planeNum3);
					if (indSeg1==-1 || indSeg2==-1 || indSeg3==-1)
						continue;
					pnlSeg1 = vecLocalSegPnts[indSeg1];
					pnlSeg2 = vecLocalSegPnts[indSeg2];
					pnlSeg3 = vecLocalSegPnts[indSeg3];

					//line 1
					flag1 = (localObjPnts[pntNum11].Position3DRef()-intesectPnt).DotProduct(plane3.Normal());
					flag2 = (localObjPnts[pntNum12].Position3DRef()-intesectPnt).DotProduct(plane3.Normal());
					if (flag1*flag2 >= 0)
						ReplaceLineNode(localObjPnts, lineTop1, intesectPnt);
					else {
						halfSpace1 = 0.0;
						Vector3D planNorm = plane3.Normal();
						for (int iPnt=0; iPnt<pnlSeg1.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg1[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace1 += 1;
							else
								halfSpace1 -= 1;							
						}
						halfSpace1 /= pnlSeg1.size()+0.001;
						halfSpace2 = 0;
						for (int iPnt=0; iPnt<pnlSeg2.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg2[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace2 += 1;
							else
								halfSpace2 -= 1;							
						}
						halfSpace2 /= pnlSeg2.size()+0.001;
						allHalfSpace = halfSpace1+halfSpace2;
						if (allHalfSpace*flag1 > 0)
							localObjPnts[pntNum12].Position3DRef() = intesectPnt;
						else
							localObjPnts[pntNum11].Position3DRef() = intesectPnt;
					}

					//line 2
					flag1 = (localObjPnts[pntNum21].Position3DRef()-intesectPnt).DotProduct(plane2.Normal());
					flag2 = (localObjPnts[pntNum22].Position3DRef()-intesectPnt).DotProduct(plane2.Normal());
					if (flag1*flag2 >= 0)
						ReplaceLineNode(localObjPnts, lineTop2, intesectPnt);
					else {
						halfSpace1 = 0.0;
						Vector3D planNorm = plane2.Normal();
						for (int iPnt=0; iPnt<pnlSeg1.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg1[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace1 += 1;
							else
								halfSpace1 -= 1;							
						}
						halfSpace1 /= pnlSeg1.size()+0.001;
						halfSpace2 = 0;
						for (int iPnt=0; iPnt<pnlSeg3.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg3[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace2 += 1;
							else
								halfSpace2 -= 1;							
						}
						halfSpace2 /= pnlSeg3.size()+0.001;
						allHalfSpace = halfSpace1+halfSpace2;
						if (allHalfSpace*flag1 > 0)
							localObjPnts[pntNum22].Position3DRef() = intesectPnt;
						else
							localObjPnts[pntNum21].Position3DRef() = intesectPnt;
					}

					//line 3
					flag1 = (localObjPnts[pntNum31].Position3DRef()-intesectPnt).DotProduct(plane3.Normal());
					flag2 = (localObjPnts[pntNum32].Position3DRef()-intesectPnt).DotProduct(plane3.Normal());
					if (flag1*flag2 >= 0)
						ReplaceLineNode(localObjPnts, lineTop3, intesectPnt);
					else {
						halfSpace1 = 0.0;
						Vector3D planNorm = plane1.Normal();
						for (int iPnt=0; iPnt<pnlSeg2.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg2[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace1 += 1;
							else
								halfSpace1 -= 1;							
						}
						halfSpace1 /= pnlSeg2.size()+0.001;
						halfSpace2 = 0;
						for (int iPnt=0; iPnt<pnlSeg3.size(); ++iPnt) {
							if ((gLaserPnts[pnlSeg3[iPnt].Number()].Position3DRef()-intesectPnt).DotProduct(planNorm)>0)
								halfSpace2 += 1;
							else
								halfSpace2 -= 1;							
						}
						halfSpace2 /= pnlSeg3.size()+0.001;
						allHalfSpace = halfSpace1+halfSpace2;
						if (allHalfSpace*flag1 > 0)
							localObjPnts[pntNum32].Position3DRef() = intesectPnt;
						else
							localObjPnts[pntNum31].Position3DRef() = intesectPnt;
					}*/
				}
				//those three planes DO NOT intersect at one point
				else {
					vector<int> adjFaceNum1, adjFaceNum2, adjFaceNum3;
					int temPN1 = planeNum1;
					int temPN2 = planeNum2;
					int temPN3 = planeNum3;
					SearchAdjacentFaces(localLineTops, temPN1, adjFaceNum1);
					SearchAdjacentFaces(localLineTops, temPN2, adjFaceNum2);
					SearchAdjacentFaces(localLineTops, temPN3, adjFaceNum3);

					//Make sure first segment is stable, 
					//Stable segment only has two neighbor segments
					//if all three segments do not have other neighbor segment, 
					//when those tree intersection lines have same length,
					//the segment which has minimum area is chosen
					//otherwise choose the segment which has shortest intersection line
					if (adjFaceNum1.size() > adjFaceNum2.size()) {
						std::swap(temPN1, temPN2);
						std::swap(adjFaceNum1, adjFaceNum2);
					}
					if (adjFaceNum1.size() > adjFaceNum3.size()) {
						std::swap(temPN1, temPN3);
						std::swap(adjFaceNum1, adjFaceNum3);
					}
					if (adjFaceNum2.size() > adjFaceNum3.size()) {
						std::swap(temPN2, temPN3);
						std::swap(adjFaceNum2, adjFaceNum3);
					}
					//stable segment only has 2 neighbor segments
					//so this stable segment is not stable
					if (adjFaceNum1.size() > 2) continue;

					PointNumberList pnlSeg1, pnlSeg2, pnlSeg3;
					int indSeg1, indSeg2, indSeg3;
					indSeg1 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, temPN1);
					indSeg2 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, temPN2);
					indSeg3 = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, temPN3);
					if (indSeg1==-1 || indSeg2==-1 || indSeg3==-1)
						continue;
					pnlSeg1 = vecLocalSegPnts[indSeg1];
					pnlSeg2 = vecLocalSegPnts[indSeg2];
					pnlSeg3 = vecLocalSegPnts[indSeg3];
					if (adjFaceNum2.size() == 2) {
						if (pnlSeg1.size() > pnlSeg2.size()) {
							std::swap(temPN1, temPN2);
							std::swap(adjFaceNum1, adjFaceNum2);
							std::swap(pnlSeg1, pnlSeg2);
						}
					}
					if (adjFaceNum3.size() == 2) {
						if (pnlSeg1.size() > pnlSeg3.size()) {
							std::swap(temPN1, temPN3);
							std::swap(adjFaceNum1, adjFaceNum3);
							std::swap(pnlSeg1, pnlSeg3);
						}
					}
					if (adjFaceNum3.size() == 2) {
						if (pnlSeg2.size() > pnlSeg3.size()) {
							std::swap(temPN2, temPN3);
							std::swap(adjFaceNum2, adjFaceNum3);
							std::swap(pnlSeg2, pnlSeg3);
						}
					}

					//comparing the length of intersection lines
					if (adjFaceNum2.size() == 2) {
						SearchLineBy2Faces(localLineTops, temPN1, temPN3, indLineTop2);
						SearchLineBy2Faces(localLineTops, temPN2, temPN3, indLineTop3);
						lineTop2 = localLineTops[indLineTop2];
						lineTop3 = localLineTops[indLineTop3];
						double lineLen2 = localObjPnts[lineTop2[0].Number()].Distance(localObjPnts[lineTop2[1].Number()]);
						double lineLen3 = localObjPnts[lineTop3[0].Number()].Distance(localObjPnts[lineTop3[1].Number()]);

						if ( (lineLen2-lineLen3)>4.0 ) {
							std::swap(temPN1, temPN2);
							std::swap(adjFaceNum1, adjFaceNum2);
							std::swap(pnlSeg1, pnlSeg2);
						}
					}
					
					SearchLineBy2Faces(localLineTops, temPN1, temPN2, indLineTop1);
					SearchLineBy2Faces(localLineTops, temPN1, temPN3, indLineTop2);
					lineTop1 = localLineTops[indLineTop1];
					lineTop2 = localLineTops[indLineTop2];
					pntNum11 =lineTop1[0].Number();
					pntNum12 = lineTop1[1].Number();
					pntNum21 = lineTop2[0].Number();
					pntNum22 = lineTop2[1].Number();
					plane = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, temPN1)];
					
					//////////////////////////////////////////////////////
					//first rectangle
					Position3D endPos1,endPos2,endPos3,endPos4;
					//refine  line1
					line1 = Line3D(localObjPnts[pntNum11] , localObjPnts[pntNum12]);
					endPos1 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line1);
					line1.Direction(line1.Direction() * -1);//inverse
					endPos2 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line1);
					//refine  line2
					line2 = Line3D(localObjPnts[pntNum21] , localObjPnts[pntNum22]);
					if(line1.Direction().DotProduct(line2.Direction())>0)
					   	line2.Direction(line2.Direction() * -1);
                    endPos3 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line2);
					line2.Direction(line2.Direction() * -1);//inverse
					endPos4 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line2);
					ReplaceLineNode(localObjPnts, lineTop1, endPos1,100);
					ReplaceLineNode(localObjPnts, lineTop1, endPos2,100);
					ReplaceLineNode(localObjPnts, lineTop2, endPos3,100);
					ReplaceLineNode(localObjPnts, lineTop2, endPos4,100);
					AddTopLine(localObjPnts, localLineTops, endPos1, endPos3, temPN1, -1, 1);
					AddTopLine(localObjPnts, localLineTops, endPos2, endPos4, temPN1, -1, 1);

					double lineDist = line1.DistanceToPoint(endPos3);
					lineDist += line1.DistanceToPoint(endPos4);
					lineDist /= 2;

					//when the two line are too near, the direction is unstable
					if (lineDist>0.3)	{
						line1 = Line3D(endPos1, endPos3);
						line2 = Line3D(endPos2, endPos4);
					}
					else {
						Vector3D vec = line1.Direction().VectorProduct(plane.Normal());
						line1 = Line3D(endPos1, vec);
						line2 = Line3D(endPos2, vec);
					}

				//	line1 = Line3D(endPos1, endPos3);
				//	line2 = Line3D(endPos2, endPos4);
					endPos1 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line1);
					line1.Direction(line1.Direction() * -1);//inverse
					endPos2 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line1);
					endPos3 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line2);
					line2.Direction(line2.Direction() * -1);//inverse
					endPos4 = ComputeEndPnt(gLaserPnts, pnlSeg1, plane, line2);
					AddTopLine(localObjPnts, localLineTops, endPos1, endPos2, temPN1, -1, -1);
					AddTopLine(localObjPnts, localLineTops, endPos3, endPos4, temPN1, -1, -1);
					AddTopLine(localObjPnts, localLineTops, endPos1, endPos3, temPN1, -1, -1);
					AddTopLine(localObjPnts, localLineTops, endPos2, endPos4, temPN1, -1, -1);
				}
			}
		}

		vecLocalPlanes[iPlane].Label() = 1;
	}

	return true;
}

IEDll_API bool Search2AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, std::vector<std::vector<int> >& vecOutAdjFaces)
{
	vecOutAdjFaces.clear();
	vector<int> AdjFaces1, AdjFaces2;
	int planeNum1, planeNum2;
	vector<int> curAdjFaceNum;

	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++) {
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, AdjFaces1);

		for (int jPlane=0; jPlane<AdjFaces1.size(); ++jPlane) {
			planeNum2 = AdjFaces1[jPlane];
			if (planeNum2 < planeNum1) continue;//
			SearchFaceBy2Faces(localLineTops, planeNum1, planeNum2, AdjFaces2);
			if (!AdjFaces2.empty()) continue;
			
			curAdjFaceNum.clear();
			curAdjFaceNum.push_back(planeNum1);
			curAdjFaceNum.push_back(planeNum2);
			vecOutAdjFaces.push_back(curAdjFaceNum);
		}
	}

	return true;
}

IEDll_API bool Search3AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, vector< vector<int> >& vecOutAdjFaces)
{
	vecOutAdjFaces.clear();
	vector<int> AdjFaces1, AdjFaces2;
	int planeNum1, planeNum2, planeNum3;
	vector<int> curAdjFaceNum;

	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++) {
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, AdjFaces1);

		for (int jPlane=0; jPlane<AdjFaces1.size(); ++jPlane) {
			planeNum2 = AdjFaces1[jPlane];
			if (planeNum2 < planeNum1) continue;//
			SearchFaceBy2Faces(localLineTops, planeNum1, planeNum2, AdjFaces2);
			for (int kPlane=0; kPlane<AdjFaces2.size(); ++kPlane) {
				planeNum3 = AdjFaces2[kPlane];
				if (planeNum3 < planeNum2) continue;

				curAdjFaceNum.clear();
				curAdjFaceNum.push_back(planeNum1);
				curAdjFaceNum.push_back(planeNum2);
				curAdjFaceNum.push_back(planeNum3);
				vecOutAdjFaces.push_back(curAdjFaceNum);
			}
		}
	}

	return true;
}

IEDll_API bool Search4AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, vector<vector<int> >& vecOutAdjFaces)
{
	vecOutAdjFaces.clear();
	vector<int> AdjFaces1, AdjFaces2, AdjFaces3, AdjFaces4;
	vector<int> forthFaces;
	int planeNum1, planeNum2, planeNum3, planeNum4;
	bool bFlag = false;
	vector<int> curAdjFaceNum, temAdjFaceNums;
	//LineTopology temLinTop;
	int indTemLine;

	//search for 4face corners
	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++) {
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, AdjFaces1);

		for (int j=0; j<AdjFaces1.size(); j++) {
			planeNum2 = AdjFaces1[j];

			for (int k=j+1; k<AdjFaces1.size(); k++) {
				planeNum3 = AdjFaces1[k];

				//there is a link between face 2 and face 3, they form a 3 face corner with face 1;
				if (SearchLineBy2Faces(localLineTops, planeNum2, planeNum3, indTemLine))
					continue;
				//temLinTop = localLineTops[indTemLine];

				if(!SearchFaceBy2Faces(localLineTops, planeNum2, planeNum3, forthFaces))
					continue;

				//  1 ----2
				//  |    /|
				//  |  5  |
				//  | /   |
				//  3 ----4
				//search whether there is a node connecting face 2 and face 3 which is not face 1 and face 4
				//In this way faces 1-2-3-4 can not form 4face corner, but 2-3-4-5 and 1-2-3-5 form 4face corner
				//if(forthFaces.size() > 2) continue;

				for (int l=0; l< forthFaces.size(); l++) {
					//  1 ----2
					//  | \  /
					//  |   4  
					//  | /   
					//  3 
					//face 1 was searched
					//or face 1 and face 4 have link, so face 1, 4, 2 and face 1, 4, 3 form two 3-face corner
					planeNum4 = forthFaces[l];
					if (planeNum4 == planeNum1
						|| SearchLineBy2Faces(localLineTops, planeNum1, planeNum4, indTemLine))
						continue;

					//  1 ----2
					//  | \   |
					//  |  5  |
					//  |   \ |
					//  3 ----4
					//search is there a node connecting face 1 and face 4 which is not face 2 and face 3
					//In this way faces 1-2-3-4 can not form 4face corner, but 1-3-4-5 and 1-2-3-5 form 4face corner
					vector<int> temVec;
					SearchFaceBy2Faces(localLineTops, planeNum1, planeNum4, temVec);
					if ( temVec.size() > 2) continue;
//					SearchFaceBy2Faces(localLineTops, planeNum2, planeNum3, temVec);
//					if ( temVec.size() > 2) continue;

					temAdjFaceNums.clear();
					temAdjFaceNums.push_back(planeNum1);
					temAdjFaceNums.push_back(planeNum2);
					temAdjFaceNums.push_back(planeNum4);
					temAdjFaceNums.push_back(planeNum3);
					vecOutAdjFaces.push_back(temAdjFaceNums);
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//sort the planes according to graph and their number
/*	for (int i=0; i<vecOutAdjFaces.size(); i++) {
		planeNum1 = vecOutAdjFaces[i][0];
		planeNum2 = vecOutAdjFaces[i][1];
		planeNum3 = vecOutAdjFaces[i][2];
		planeNum4 = vecOutAdjFaces[i][3];

		if (planeNum1 > planeNum4)
			std::swap(planeNum1, planeNum4);

		if (planeNum2 > planeNum3)
			std::swap(planeNum2, planeNum3);

		if (planeNum1 > planeNum2) {
			std::swap(planeNum1, planeNum2);
			std::swap(planeNum3, planeNum4);
		}

		vecOutAdjFaces[i][0] = planeNum1;
		vecOutAdjFaces[i][1] = planeNum2;
		vecOutAdjFaces[i][2] = planeNum3;
		vecOutAdjFaces[i][3] = planeNum4;
	}*/
	//sort the planes according to graph and their number
	vector<int> temAdjFaces;
	int nFaceNum, ind, minNum;
	for (int i=0; i<(int)vecOutAdjFaces.size(); i++) {
		minNum = 999999999;
		temAdjFaces = vecOutAdjFaces[i];
		nFaceNum = temAdjFaces.size();
		for (int j=0; j<(int)temAdjFaces.size(); ++j) {
			if (minNum > temAdjFaces[j]) {
				ind = j;
				minNum = temAdjFaces[j];
			}
		}
		temAdjFaces.insert(temAdjFaces.end(), 
			temAdjFaces.begin(), temAdjFaces.begin()+ind);
		temAdjFaces.erase(temAdjFaces.begin(), temAdjFaces.begin()+ind);
		vecOutAdjFaces[i] = temAdjFaces;

		//inverse direction
		vecOutAdjFaces[i][0] = temAdjFaces[0]; 
		if (temAdjFaces[nFaceNum-1]<temAdjFaces[1]) {
			for (int j=1; j<nFaceNum; ++j) 
				vecOutAdjFaces[i][j] = temAdjFaces[nFaceNum-j];
		}
		
	}

	/////////////////////////////////////////////////////
	//remove repeating loops
	for (int i=0; i<vecOutAdjFaces.size(); i++) {
		curAdjFaceNum = vecOutAdjFaces[i];
		for (int j=i+1; j<vecOutAdjFaces.size(); j++) {
			temAdjFaceNums = vecOutAdjFaces[j];

			if (curAdjFaceNum[0]==temAdjFaceNums[0] 
			&& curAdjFaceNum[1]==temAdjFaceNums[1]
			&& curAdjFaceNum[2]==temAdjFaceNums[2] 
			&& curAdjFaceNum[3]==temAdjFaceNums[3]) {
				vecOutAdjFaces.erase(vecOutAdjFaces.begin()+j);
				j--;
			}
		}
	}

	if (vecOutAdjFaces.empty())
		return false;
	else
		return true;
}


IEDll_API bool RefineCornerOf4Faces(std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxAngle = 5*PI/180;
	double maxDistance = 10.0;
	double angPlan2Line = 20*PI/180;

	vector<int> curAdjFaceNums;
	vector<int> thirdFaceNums;//store the third face who can form 3face corner with iPlane and temFace;

	LineTopology lineTop1, lineTop2, lineTop3, lineTop4;
	int indLine1, indLine2, indLine3, indLine4;
	Line3D line1, line2, line3, line4, stableLine;
	Plane plane1, plane2, plane3, plane4;
	Plane plane;
	Position3D intesectPnt, averagePnt;
	int nIntPntNum;
	int pntNum11, pntNum12, pntNum21, pntNum22, pntNum31,pntNum32, pntNum41, pntNum42;
//	int planeNum1, planeNum2, planeNum3, planeNum4;
	vector<vector<int> > vecAdjFaces;
	double ang1, ang2, ang3, ang4;
	double dist1, dist2, dist3, dist4, temDist1, temDist2;

	Search4AdjFaces(vecLocalPlanes, localLineTops, vecAdjFaces);

	for (int i=0; i<vecAdjFaces.size(); i++) {
		curAdjFaceNums = vecAdjFaces[i];
		if (curAdjFaceNums[0]==1529 && curAdjFaceNums[1]==1808
			&&curAdjFaceNums[2]==1809 && curAdjFaceNums[3]==1810)
		{
			int aaa = 0;
		}

		//  0 -1--1
		//  |     |
		//  4     2
		//  |     |
		//  3 -3--2
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[0], curAdjFaceNums[1], indLine1);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[1], curAdjFaceNums[2], indLine2);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[2], curAdjFaceNums[3], indLine3);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[3], curAdjFaceNums[0], indLine4);
		lineTop1 = localLineTops[indLine1];
		lineTop2 = localLineTops[indLine2];
		lineTop3 = localLineTops[indLine3];
		lineTop4 = localLineTops[indLine4];

		pntNum11 = lineTop1[0].Number(); pntNum12 = lineTop1[1].Number();
		pntNum21 = lineTop2[0].Number(); pntNum22 = lineTop2[1].Number();
		pntNum31 = lineTop3[0].Number(); pntNum32 = lineTop3[1].Number();
		pntNum41 = lineTop4[0].Number(); pntNum42 = lineTop4[1].Number();

		line1 = Line3D(localObjPnts[pntNum11], localObjPnts[pntNum12]);
		line2 = Line3D(localObjPnts[pntNum21], localObjPnts[pntNum22]);
		line3 = Line3D(localObjPnts[pntNum31], localObjPnts[pntNum32]);
		line4 = Line3D(localObjPnts[pntNum41], localObjPnts[pntNum42]);

		//determine whether intersection points are near enough
		Position3D intPnt1, intPnt2, intPnt3, intPnt4;
		MyIntersect2Lines(line1,line2,intPnt1);
		MyIntersect2Lines(line2,line3,intPnt2);
		MyIntersect2Lines(line3,line4,intPnt3);
		MyIntersect2Lines(line4,line1,intPnt4);

		//compute intersect distance and intersection angle
		averagePnt.X() = 0; averagePnt.Y() = 0; averagePnt.Z() = 0;
		averagePnt.X() += intPnt1.X()/4; averagePnt.Y() += intPnt1.Y()/4; averagePnt.Z() += intPnt1.Z()/4;
		averagePnt.X() += intPnt2.X()/4; averagePnt.Y() += intPnt2.Y()/4; averagePnt.Z() += intPnt2.Z()/4;
		averagePnt.X() += intPnt3.X()/4; averagePnt.Y() += intPnt3.Y()/4; averagePnt.Z() += intPnt3.Z()/4;
		averagePnt.X() += intPnt4.X()/4; averagePnt.Y() += intPnt4.Y()/4; averagePnt.Z() += intPnt4.Z()/4;
		ang1 = Angle(line1.Direction(), line2.Direction());
		ang2 = Angle(line2.Direction(), line3.Direction());
		ang3 = Angle(line3.Direction(), line4.Direction());
		ang4 = Angle(line4.Direction(), line1.Direction());
		ang1 *= 180/PI; ang2 *= 180/PI; ang3 *= 180/PI; ang4 *= 180/PI;
/*		temDist1 = averagePnt.Distance(localObjPnts[pntNum11]);
		temDist2 = averagePnt.Distance(localObjPnts[pntNum12]);
		dist1 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = averagePnt.Distance(localObjPnts[pntNum21]);
		temDist2 = averagePnt.Distance(localObjPnts[pntNum22]);
		dist2 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = averagePnt.Distance(localObjPnts[pntNum31]);
		temDist2 = averagePnt.Distance(localObjPnts[pntNum32]);
		dist3 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = averagePnt.Distance(localObjPnts[pntNum41]);
		temDist2 = averagePnt.Distance(localObjPnts[pntNum42]);
		dist4 = temDist1<temDist2?temDist1:temDist2;*/
		temDist1 = intPnt1.Distance(localObjPnts[pntNum11]);
		temDist2 = intPnt1.Distance(localObjPnts[pntNum12]);
		dist1 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = intPnt2.Distance(localObjPnts[pntNum21]);
		temDist2 = intPnt2.Distance(localObjPnts[pntNum22]);
		dist2 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = intPnt3.Distance(localObjPnts[pntNum31]);
		temDist2 = intPnt3.Distance(localObjPnts[pntNum32]);
		dist3 = temDist1<temDist2?temDist1:temDist2;
		temDist1 = intPnt4.Distance(localObjPnts[pntNum41]);
		temDist2 = intPnt4.Distance(localObjPnts[pntNum42]);
		dist4 = temDist1<temDist2?temDist1:temDist2;

//		dist2 = averagePnt.Distance(intPnt2);
//		dist3 = averagePnt.Distance(intPnt3);
//		dist4 = averagePnt.Distance(intPnt4);

		int wrongEdgeNum = 0;
		if(dist1>2.0) wrongEdgeNum++;
		if(dist2>2.0) wrongEdgeNum++;
		if(dist3>2.0) wrongEdgeNum++;
		if(dist4>2.0) wrongEdgeNum++;
		//only bear one edge is wrong
		//in this case, two intersection lines are parallel. 
		if(wrongEdgeNum >= 2) continue;

//		if (dist1>1.0 || dist2>1.0 || dist3>1.0 || dist4>1.0 ) {
//			if ((ang1>10 && ang1<170) && (ang2>10 && ang2<170)
//				&& (ang3>10 && ang3<170) && (ang4>10 && ang4<170))	{
//				continue;//if the intersection pints are too far away, they cannot form 4faces corner
//			}	
//		}

		int planSegNum1 = curAdjFaceNums[0];
		int planSegNum2 = curAdjFaceNums[1];
		int planSegNum3 = curAdjFaceNums[2];
		int planSegNum4 = curAdjFaceNums[3];

		plane1 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum1)];
		plane2 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum2)];
		plane3 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum3)];
		plane4 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum4)];
		if (planSegNum1 == 40
			|| planSegNum2 == 40
			|| planSegNum3 == 40
			|| planSegNum4 == 40)	{
				int aaa = 0;
		}

		bool bFlag = false;
		nIntPntNum = 0;
		averagePnt.X() = 0;
		averagePnt.Y() = 0;
		averagePnt.Z() = 0;

		// fist line
		if (MyIsParaller(line1.Direction(), Vector3D(0, 0, 1), maxAngle) 
			|| line1.IsHorizontal(maxAngle)) {
			if (!MyIsParaller(plane3, line1,  angPlan2Line)){
				bFlag = true;
				MyIntersectLine3DPlane(line1, plane3, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line1, angPlan2Line))	{
				bFlag = true;
				MyIntersectLine3DPlane(line1, plane4, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		// second line
		if (MyIsParaller(line2.Direction(), Vector3D(0, 0, 1), maxAngle) 
			|| line2.IsHorizontal(maxAngle)) {
			if (!MyIsParaller(plane1, line2, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line2, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane4, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//third line
		if (MyIsParaller(line3.Direction(), Vector3D(0, 0, 1), maxAngle)
			|| line3.IsHorizontal(maxAngle)) {
			if (!MyIsParaller(plane1, line3, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line3, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//forth line
		if (MyIsParaller(line4.Direction(), Vector3D(0, 0, 1), maxAngle) < maxAngle 
			|| line4.IsHorizontal(maxAngle)) {
			if (!MyIsParaller(plane2, line4, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane3, line4, angPlan2Line)) {
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane3, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//if there is no stable line, just choose one of the intersection lines. 
		//later the longest line would be chosen as stable line
		if (bFlag == false) {
			if (!MyIsParaller(plane3, line1, angPlan2Line)) {
				MyIntersectLine3DPlane(line1, plane3, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line1, angPlan2Line)) {
				MyIntersectLine3DPlane(line1, plane4, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane1, line2, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line2, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane4, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane1, line3, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line3, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane3, line4, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane3, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line4, angPlan2Line)) {
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		averagePnt.X() /= nIntPntNum;
		averagePnt.Y() /= nIntPntNum;
		averagePnt.Z() /= nIntPntNum;

		ReplaceLineNode(localObjPnts, lineTop1, averagePnt, 10.0);
		ReplaceLineNode(localObjPnts, lineTop2, averagePnt, 10.0);
		ReplaceLineNode(localObjPnts, lineTop3, averagePnt, 10.0);
		ReplaceLineNode(localObjPnts, lineTop4, averagePnt, 10.0);
	}

	return true;
}

IEDll_API bool RefineOutBoundaryPlnae(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts;
	PointNumberList pnlTemSegPnts;
	vector<int> vecTemLineNums;
	int curPlaneNum;
	LaserPoint temLaserPnt;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	bool bNearEdge;
	Plane plane;
	int pntNums;
	int minPntNums = 5;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNum;

	//double minDist = 0.3;

	//search for out boundary points
	//which first are boundary points and don't near inner edge
	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);
		MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts);
	
		for (int j=0; j<pnlTemBoundPnts.size(); j++)
		{
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];

			bNearEdge = false;
			for (int k=0; k<localLineTops.size(); k++)
			{
				p1 = localObjPnts[localLineTops[k][0].Number()];
				p2 = localObjPnts[localLineTops[k][1].Number()];
				temLine = Line3D(p1, p2);

				dist = temLine.DistanceToPoint( (Position3D)temLaserPnt);
				if (dist < minDist)
				{
					bNearEdge = true;
					break;
				}
			}

			if (!bNearEdge) pnlOrgAllBoundPnts.push_back(pnlTemBoundPnts[j]);
		}
	}

	LaserPoints selLaserPnts;
	for (int i=0; i<pnlOrgAllBoundPnts.size(); i++)
		selLaserPnts.push_back(gLaserPnts[pnlOrgAllBoundPnts[i].Number()]);
	
	//selLaserPnts.Write("c:\\laserpoints.laser", 0, true);
	selLaserPnts.Label(-1);
	
	//search planes which fit the out boundary points
	HoughSpace houghspace;
	selLaserPnts.InitialiseHoughSpace(houghspace, -1, 85*PI/180, 5*PI/180, 0.5, 1, LabelTag);
	selLaserPnts.IncrementHoughSpace(houghspace);
	do 
	{
		pntNums = 0;
		plane = houghspace.BestPlane(&pntNums, 0, 3); 
		
		if (pntNums < minPntNums) continue;
		pntNums = selLaserPnts.Label(plane, minDist, -1, -1,  1, 0);
		
		//fit the plane again
		if (pntNums < minPntNums) continue;
		plane = selLaserPnts.FitPlane(1, 1);
		if (pntNums < minPntNums) continue;
		pntNums = selLaserPnts.Label(plane, minDist, 0, 1, 1, 0);
		
		if (pntNums >= minPntNums)
		{
			pnlTemSegPnts = selLaserPnts.TaggedPointNumberList(LabelTag,1);
			vecPlneNums.clear();
			pnlOrgTemBoundPnts.clear();
			//double minHeight = 99999999;

			//remove detected points from hough space
			//and get planes of corresponding segments
			for (int i=0; i<pnlTemSegPnts.size(); i++)
			{	
				temLaserPnt = selLaserPnts[pnlTemSegPnts[i].Number()];
				houghspace.RemovePoint(temLaserPnt.X(), temLaserPnt.Y(), temLaserPnt.Z());

				//if (temLaserPnt.Z() < minHeight) 
				//	minHeight = temLaserPnt.Z();

				//transform template point number to original point number
				orgPntNum = pnlOrgAllBoundPnts[pnlTemSegPnts[i].Number()];
				pnlOrgTemBoundPnts.push_back(orgPntNum);
				int planNum= gLaserPnts[orgPntNum.Number()].Attribute(SegmentNumberTag);
				//IndexPlaneBySegNumber(vecLocalPlanes, gLaserPnts[orgPntNum.Number()].Attribute(SegmentNumberTag));
				vecPlneNums.push_back(planNum);
			}

			//remove redundancy plane numbers
			std::sort(vecPlneNums.begin(), vecPlneNums.end());
			for (int i=1; i<vecPlneNums.size(); i++)
			{
				if (vecPlneNums[i] == vecPlneNums[i-1])
				{
					vecPlneNums.erase(vecPlneNums.begin()+i);
					i--;
				}
			}
            
            Plane temPlane;
			//if plane is near horizontal, constraint fit it to be horizontal
			if (plane.IsHorizontal(15*PI/180))
			{
				temPlane = gLaserPnts.FitPlane(pnlOrgTemBoundPnts, Vector3D(0,0,1), 0);
				plane = temPlane;
            }
				
			//Warning:
			//hypothesis plane does not correspond to a point segment
			//so it's number do not have corresponding segment number
			plane.Number() = 100000000+vecLocalPlanes.size();
			vecLocalPlanes.push_back(plane);
			
			//intersect hypothesis plane with planes of segment points
			for (int i=0; i<vecPlneNums.size(); i++)
			{
				int planInd= IndexPlaneBySegNumber(vecLocalPlanes, vecPlneNums[i]) ;
				if(planInd<0) continue; 
				Plane temPlane = vecLocalPlanes[planInd];
				Position3D p1, p2;
				bool bFindSeg = false;
				PointNumberList pnlTemSeg;
				
				//search for corresponding point segment with plane number
				for (int j=0; j<vecLocalSegPnts.size(); j++)
				{
					pnlTemSeg = vecLocalSegPnts[j];
					if (gLaserPnts[pnlTemSeg[0].Number()].Attribute(SegmentNumberTag) == temPlane.Number())
					{
						bFindSeg = true;
						break;
					}
				}
				if (!bFindSeg) continue;//cannot find corresponding point segment, skip it

				//MyIsParaller()
				gLaserPnts.IntersectFaces(pnlTemSeg, pnlOrgTemBoundPnts, temPlane, plane, gBldRecPar.minPntDist, p1, p2);

				if (p1.Distance(p2) < 0.5 )
					continue;//intersection line is too short

				ObjectPoint objPnt;
				LineTopology lineTop;
				
				objPnt.X() = p1.X(); objPnt.Y() = p1.Y(); objPnt.Z() = p1.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				objPnt.X() = p2.X(); objPnt.Y() = p2.Y(); objPnt.Z() = p2.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				lineTop.Number() = localLineTops.size();
				lineTop.SetAttribute(SegmentLabel, plane.Number());
				lineTop.SetAttribute(SecondSegmentLabel, temPlane.Number());
				localLineTops.push_back(lineTop);
			}
		}
	} while (pntNums > minPntNums);
	
	return true;
}

//nstruct this point
/*bool RefineLocalBuilding(const vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
//	Line3D curLine, leftLine, rightLine;
//	Vector3D dir;
//	Plane plane;
//	Position3D intPnt;
LineTopology topCurLine, topLeftLine, topRightLine;
vector<LineTopology> vecLineTops;
int lFaceNum, rFaceNum;
vector<LineTopology> vecPossibleNeibLines, vecNeibLineTops;
vector<int> vecNeibPlanes;
int numNeibPlane;

for (int i=0; i< localLineTops.size(); i++)
{
if (!localLineTops[i].HasAttribute(0) && !localLineTops[i].HasAttribute(1))
return false;//each intersect line are not formed by intersecting two faces
}

//for (int i=0; i< localLineTops.size(); i++)
//	localLineTops[i].SetAttribute(2, -1);//-1 un-refined; 1 refined

for (int i=0; i< localLineTops.size(); i++)
{
topCurLine = localLineTops[i];

//if(topCurLine.Attribute(2) == 1)
//	continue;//this line is refined, move to next line

//if (!topCurLine.HasAttribute(0) && !topCurLine.HasAttribute(1))
//	return false;

lFaceNum = topCurLine.Attribute(0);
rFaceNum = topCurLine.Attribute(1);

//search for lines which are on the left face or right face of current line
vecPossibleNeibLines.clear();
for (int j=i+1; j < localLineTops.size(); j++)
{
LineTopology temLine = localLineTops[j];

if (temLine.Attribute(0) == lFaceNum || temLine.Attribute(0) == rFaceNum 
|| temLine.Attribute(1) == lFaceNum || temLine.Attribute(1) == lFaceNum )
vecPossibleNeibLines.push_back(temLine);			
}

if(vecPossibleNeibLines.size() < 2)
continue; //not enough neighbor lines

//search for face which is neighbor with left face and right at the same time
vecNeibPlanes.clear();
vecNeibLineTops.clear();
for (int j=0; j < vecPossibleNeibLines.size(); j++)
{		
topLeftLine = vecPossibleNeibLines[j]; 

for (int k=j+1; k < vecPossibleNeibLines.size(); k++)
{
//has same neighbor face
topRightLine = vecPossibleNeibLines[k];

if ( topLeftLine.Attribute(0) == topRightLine.Attribute(0) 
|| topLeftLine.Attribute(0) == topRightLine.Attribute(1))
numNeibPlane = topLeftLine.Attribute(0);
else if (topLeftLine.Attribute(1) == topRightLine.Attribute(0)
|| topLeftLine.Attribute(1) == topRightLine.Attribute(1))
numNeibPlane = topLeftLine.Attribute(1);
else
continue;

//the same neighbor face should not be the left face or right face of current line
if (numNeibPlane != lFaceNum && numNeibPlane != rFaceNum)
{
vecNeibPlanes.push_back(numNeibPlane);
vecLineTops.push_back(topLeftLine);
vecLineTops.push_back(topRightLine);
}
}
}

//refine end points of 3 lines which intersect at one point
Position3D startPnt = (Position3D)localObjPnts[topCurLine[0].Number()];
Position3D endPnt =  (Position3D)localObjPnts[topCurLine[1].Number()];
Line3D curLine = Line3D(startPnt, endPnt);

for (int j=0; j<vecNeibPlanes.size(); j++)
{
int planeIndex = IndexPlaneBySegNumber(vecLocalPlanes, vecNeibPlanes[j]);
if (planeIndex == -1)
continue;

Plane plane = vecLocalPlanes[planeIndex];
Position3D intPnt;

if (!MyIntersectLine3DPlane(curLine, plane, intPnt))
continue; //line does not intersect with plane

ReplaceLineNode(localObjPnts, topCurLine, intPnt);
ReplaceLineNode(localObjPnts, vecLineTops[j+0], intPnt);
ReplaceLineNode(localObjPnts, vecLineTops[j+1], intPnt);
}
}
}*/

//try to find nearby nodes on nearby lines
//then replays this nodes with their average points
IEDll_API bool RefineLocalBuilding(const vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxPntDist = 0.5;
	Plane curPlane;
	vector<int> vecTemTopNums;
	Vector3D pnt1, pnt2, pnt3, pnt4;
	int numPnt1, numPnt2, numPnt3, numPnt4;
	Line3D curLine, temLine;
	Vector3D intPos;

	for (int i=0; i<vecLocalPlanes.size(); i++)
	{
		curPlane = vecLocalPlanes[i];
		if(!SearchLinesOnPlane(localLineTops, curPlane.Number(), vecTemTopNums)
			|| vecTemTopNums.size() < 2)
			continue;

		for (int j=0; j<vecTemTopNums.size(); j++ )
		{
			if (localLineTops[j].Attribute(2) != -1)
				continue;

			numPnt1 = localLineTops[vecTemTopNums[j]][0].Number();
			numPnt2 = localLineTops[vecTemTopNums[j]][1].Number();
			pnt1 = (Vector3D)localObjPnts[numPnt1];
			pnt2 = (Vector3D)localObjPnts[numPnt2];
			curLine = Line3D(pnt1, pnt2-pnt1);

			for (int k=j+1; k<vecTemTopNums.size(); k++)
			{
				if (localLineTops[k].Attribute(2) != -1)
					continue;

				numPnt3 = localLineTops[vecTemTopNums[k]][0].Number();
				numPnt4 = localLineTops[vecTemTopNums[k]][1].Number();
				pnt3 = (Vector3D)localObjPnts[numPnt3];
				pnt4 = (Vector3D)localObjPnts[numPnt4];
				temLine = Line3D(pnt3, pnt4-pnt3);

				if(!MyIntersect2Lines(curLine, temLine, intPos))
					continue;

				/*		if ((pnt1-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt1].SetX(intPos.X());
				localObjPnts[numPnt1].SetY(intPos.Y());
				localObjPnts[numPnt1].SetZ(intPos.Z());
				} 
				else if ( (pnt2-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt2].SetX(intPos.X());
				localObjPnts[numPnt2].SetY(intPos.Y());
				localObjPnts[numPnt2].SetZ(intPos.Z());
				}

				if ( (pnt3-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt3].SetX(intPos.X());
				localObjPnts[numPnt3].SetY(intPos.Y());
				localObjPnts[numPnt3].SetZ(intPos.Z());
				} 
				else if ( (pnt4-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt4].SetX(intPos.X());
				localObjPnts[numPnt4].SetY(intPos.Y());
				localObjPnts[numPnt4].SetZ(intPos.Z());
				}*/
			}
		}
	}

	/*	vector<int> vecNeibNodes;
	Vector3D averagePnt;

	for (int i=0; i<localLineTops.size(); i++)
	{
	if (localLineTops[i].Attribute(2) != -1)
	continue;//this line has been refined

	//first node of this line	
	if (SearchNodesOnNeigborLine(localObjPnts, localLineTops, i, 0, vecNeibNodes, 1.0))
	{
	averagePnt = Vector3D(0, 0, 0);
	for (int j=0; j<vecNeibNodes.size(); j++)
	averagePnt += (Vector3D)localObjPnts[vecNeibNodes[j]];

	averagePnt /= localObjPnts.size();

	localObjPnts[localLineTops[i][0].Number()].SetX(averagePnt.X());
	localObjPnts[localLineTops[i][0].Number()].SetY(averagePnt.Y());
	localObjPnts[localLineTops[i][0].Number()].SetZ(averagePnt.Z());

	for (int j=0; j<vecNeibNodes.size(); j++)
	{
	localObjPnts[j].SetX(averagePnt.X());
	localObjPnts[j].SetY(averagePnt.Y());
	localObjPnts[j].SetZ(averagePnt.Z());
	}
	}

	//second node of this line
	if (SearchNodesOnNeigborLine(localObjPnts, localLineTops, i, 1, vecNeibNodes,1.0))
	{		
	averagePnt = Vector3D(0, 0, 0);
	for (int j=0; j<vecNeibNodes.size(); j++)
	averagePnt += (Vector3D)localObjPnts[vecNeibNodes[j]];

	averagePnt /= localObjPnts.size();

	localObjPnts[localLineTops[i][1].Number()].SetX(averagePnt.X());
	localObjPnts[localLineTops[i][1].Number()].SetY(averagePnt.Y());
	localObjPnts[localLineTops[i][1].Number()].SetZ(averagePnt.Z());

	for (int j=0; j<vecNeibNodes.size(); j++)
	{
	localObjPnts[j].SetX(averagePnt.X());
	localObjPnts[j].SetY(averagePnt.Y());
	localObjPnts[j].SetZ(averagePnt.Z());
	}
	}

	localLineTops[i].SetAttribute(2, 1);
	}*/

	return true;
}

IEDll_API bool RefineSingleRoof(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
//	if (vecLocalSegPnts.size()!=1 || vecLocalSegPnts[0].size() < 50) 
//		return false;
	/*
	LaserPoints las = gLaserPnts;
	las.Write("e:\\seg.laser", 0, 0);

	FILE* file = new FILE;
	file = fopen("e:\\info.txt","w");
	fprintf(file, "%.4f, %.4f, %.4f, %.4f", vecLocalPlanes[0].Normal().X(), vecLocalPlanes[0].Normal().Y(), 
		vecLocalPlanes[0].Normal().Z(), vecLocalPlanes[0].Distance());
	fclose(file);
	*/

	//return RefineOutBoundaryLine2(gLaserPnts, vecLocalPlanes, vecLocalSegPnts, localObjPnts, localLineTops);

	int curSegNum;
	vector<int> vecTopLineNums;
	vector<PointNumberList>::iterator itrSeg; 
	Plane curPlane;


	for (itrSeg=vecLocalSegPnts.begin(); itrSeg!=vecLocalSegPnts.end(); ++itrSeg) {
		if(itrSeg->size() < gBldRecPar.minSegNum) continue;

		curSegNum = gLaserPnts[(*itrSeg)[0].Number()].Attribute(SegmentNumberTag);
		SearchLinesOnPlane(localLineTops, curSegNum, vecTopLineNums);
		if (!vecTopLineNums.empty()) continue;//this segment is not single

		int indPlane = IndexPlaneBySegNumber(vecLocalPlanes, curSegNum);
		if (indPlane==-1) continue;
		curPlane = vecLocalPlanes[indPlane];
		if (curPlane.IsVertical(10.0*PI/180.0)) continue;

		RefineOutBoundaryLine2(gLaserPnts, vecLocalPlanes, *itrSeg, localObjPnts, localLineTops);
		//RefineOutBoundaryLine8(gLaserPnts, vecLocalPlanes, *itrSeg, localObjPnts, localLineTops);
	}

	return true;
}

/*-----------------------------------
//0: start point; 1: end point; 2: max distance point; 3 last point
1------1-------2
 \              \
  2              4
   \			  \
    3------3-------4
*/
IEDll_API bool RefineSimpleDormer(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	Plane curPlane, supPlane;
	//LineTopologies temLineTops;
	vector<int> vecLineNums, vecAdjFaceNums;
	LineTopology curLineTop, temLineTop;
	Line3D line1, line2, line3, line4;
	PointNumberList pnlCurSeg, pnlSupSeg;
	Vector3D dir2, supDir;
	Position3D pnt1, pnt2, pnt3, pnt4, temPnt;
//	LaserPoints laserPnts;
	double maxDist;
	int curSegNum, supSegNum;
	double scaleBegin, scaleEnd, supSca1, supSca2;

	for (int i=0; i<vecLocalPlanes.size(); ++i) {
		curPlane = vecLocalPlanes[i];
		vecLineNums.clear();
		curSegNum = curPlane.Number();
		SearchLinesOnPlane(localLineTops, curPlane.Number(), vecLineNums);

		if ( vecLineNums.size() != 1 //dormer plane only have one intersect line
			//|| !curPlane.IsHorizontal(10*PI/180) //dormer plane should be horizontal
			/*|| temLineTops[0].Attribute(2) != -1*/) //intersect line should not be refined
			continue;

		curLineTop = localLineTops[vecLineNums[0]];
		//search neighbor segment number
		supSegNum = curLineTop.Attribute(SegmentLabel);
		if (supSegNum==curSegNum) supSegNum=curLineTop.Attribute(SecondSegmentLabel);

		//refine start point and end point according to laser points of this plane
		pnt1 = localObjPnts[curLineTop[0].Number()].Position3DRef();
		pnt2 = localObjPnts[curLineTop[1].Number()].Position3DRef();
		if (pnt1.Distance(pnt2)<0.1) continue;
		line1 = Line3D(pnt1, pnt2);
		int curSegInd = IndexSegBySegNumber(gLaserPnts,vecLocalSegPnts, curSegNum);
		if (curSegInd==-1) continue;
		pnlCurSeg = vecLocalSegPnts[curSegInd];
		gLaserPnts.FaceNearLine(pnlCurSeg, line1, 100, scaleBegin, scaleEnd);

		//if has no support surface, consider it is dormer,
		//otherwise the support surface should have more strict rules
		if (supSegNum!=-1) {
			SearchAdjacentFaces(localLineTops, supSegNum, vecAdjFaceNums);
			//support segment should have more than one neighbor segment
			if(vecAdjFaceNums.size()<2) continue;
			int neibPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, supSegNum);
			if(neibPlaneInd == -1) continue;
			supPlane = vecLocalPlanes[neibPlaneInd];
			if (supPlane.IsHorizontal(15*PI/180))
				continue;//support segment should be sloped

			int supSegInd = IndexSegBySegNumber(gLaserPnts,vecLocalSegPnts, supSegNum);
			if (supSegInd==-1) continue;
			pnlSupSeg = vecLocalSegPnts[supSegInd];
			gLaserPnts.FaceNearLine(pnlSupSeg, line1, 100, supSca1, supSca2);	
			//ridge length on support segment should be longer then one on dormer
			if (fabs(scaleBegin-scaleEnd) > 0.75*fabs(supSca1-supSca2))
				continue;
		}

		//intersection line should be horizontal
		if (!line1.IsHorizontal(10.0*PI/180.0)){
			localLineTops[vecLineNums[0]].SetAttribute(2, 1);//refined
			continue;
		}
			
		pnt1 = line1.Position(scaleBegin);
		pnt2 = line1.Position(scaleEnd);
		if (pnt1.Z() > pnt2.Z())
			std::swap(pnt1,pnt2);//make sure point1 is higher than point2
		if (line1.IsHorizontal(10*PI/180))
			dir2 = curPlane.Normal().VectorProduct(line1.Direction());
		else
			dir2 = curPlane.Normal().VectorProduct(Vector3D(0,0,1));
		line2 = Line3D(pnt1, dir2);
		line4 = Line3D(pnt2, dir2);

		//search point 3
//		laserPnts.clear();
//		laserPnts.AddTaggedPoints(gLaserPnts, curPlane.Number(), SegmentNumberTag);
		maxDist = -99999.0;
		double temDist;
		for (int j=0; j<pnlCurSeg.size(); ++j)	{
			temPnt = line2.Project( gLaserPnts[pnlCurSeg[j].Number()].Position3DRef());
			temDist = temPnt.Distance(pnt1);
			if(temDist > maxDist )	{
				maxDist = temDist;
				pnt3 = temPnt;
			}
		}
		//pnt4 = pnt3 - pnt1 + pnt2;
		pnt4 = line4.Project(pnt3);
		dir2 = pnt3 - pnt1;

		if (supSegNum!=-1) {
			//dormer roof should stand at same side with support roof
			//half space is divided by the plane of intersection line and zenith direction
			if (!supPlane.IsHorizontal(15*PI/180)) {
				supDir = supPlane.Normal().VectorProduct(Vector3D(0,0,1));
				supDir = supPlane.Normal().VectorProduct(supDir);
				if (supDir.Z() > 0.0) supDir *= -1.0;		
			}
			else {
				Position3D pntCenter = gLaserPnts.CentreOfGravity(pnlSupSeg);
				Position3D pntProj = line1.Project(pntCenter);
				supDir = pntCenter - pntProj;
			}

			Vector3D halfSpace = line1.Direction().VectorProduct(Vector3D(0,0,1));
			if (halfSpace.DotProduct(supDir)*halfSpace.DotProduct(dir2) < 0)
				continue;//different half space
		}
		
		//horizontal dormer should have a width less then 10 meter
		//and slop dormer should have a with less then 2 meter
//		bool bIsDormer = false;
//		if (curPlane.IsHorizontal(10*PI/180) )
//			if (maxDist < 10)
//				bIsDormer = true;
//		else
//			if (maxDist < 2)
//				bIsDormer = true;
//		if (!bIsDormer) continue;
		//first line
		localObjPnts[curLineTop[0].Number()].Position3DRef() = pnt1;
		localObjPnts[curLineTop[1].Number()].Position3DRef() = pnt2;
		localLineTops[vecLineNums[0]].SetAttribute(2, 1);//refined
		//add one aux line to say first line is refined
		//AddTopLine(localObjPnts,localLineTops, pnt1, pnt2, supSegNum, -1, -1);
		AddTopLine(localObjPnts,localLineTops, pnt1, pnt3, curSegNum, -1, -1);
		AddTopLine(localObjPnts,localLineTops, pnt3, pnt4, curSegNum, -1, -1);
		AddTopLine(localObjPnts,localLineTops, pnt4, pnt2, curSegNum, -1, -1);

/*		int objPntNum = localObjPnts.size();
		int lineTopNum = localLineTops.size();
		//second line
		localObjPnts.push_back(ObjectPoint(pnt3.GetX(),pnt3.GetY(),pnt3.GetZ(),objPntNum,0,0,0,0,0,0));
		temLineTop = LineTopology(lineTopNum, -1, curLineTop[0].Number(), objPntNum);
		temLineTop.SetAttribute(SegmentLabel, curPlane.Number());
		temLineTop.SetAttribute(SecondSegmentLabel, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);

		//third line
		localObjPnts.push_back(ObjectPoint(pnt4.GetX(),pnt4.GetY(),pnt4.GetZ(),objPntNum+1,0,0,0,0,0,0));
		temLineTop = LineTopology(lineTopNum+1, -1, objPntNum+1, objPntNum);
		temLineTop.SetAttribute(SegmentLabel, curPlane.Number());
		temLineTop.SetAttribute(SecondSegmentLabel, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);

		//forth line
		temLineTop = LineTopology(lineTopNum+2, -1, curLineTop[1].Number(), objPntNum+1);
		temLineTop.SetAttribute(SegmentLabel, curPlane.Number());
		temLineTop.SetAttribute(SecondSegmentLabel, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);*/
	}

	return true;
}

IEDll_API bool SearchLinesOnPlane(const LineTopologies& localLineTops, const int curPlaneNum, vector<int>& vecOutTopInds)
{
	//	int planeNum = plane.Number();
	LineTopologies::const_iterator itr;
	vecOutTopInds.clear();

	for (itr=localLineTops.begin(); itr!=localLineTops.end(); ++itr){
		if (itr->Attribute(SegmentLabel) == curPlaneNum
			|| itr->Attribute(SecondSegmentLabel) == curPlaneNum) {
			vecOutTopInds.push_back(itr-localLineTops.begin());
		}
	}

	if (vecOutTopInds.size()>0)
		return true;
	else
		return false;
}


IEDll_API bool SearchAdjacentFaces(const LineTopologies& localLineTops, const int curPlaneNum, vector<int>& vecAdjFaceNums)
{
	vecAdjFaceNums.clear();

	vector<int> vecLineTopInds;
	if (!SearchLinesOnPlane(localLineTops, curPlaneNum, vecLineTopInds))
		return false;

	int lPlaneNum, rPlaneNum;
	for ( int i=0; i<vecLineTopInds.size(); i++) {
		lPlaneNum = localLineTops[vecLineTopInds[i]].Attribute(SegmentLabel);
		rPlaneNum = localLineTops[vecLineTopInds[i]].Attribute(SecondSegmentLabel);

		if (lPlaneNum != -1 && lPlaneNum != curPlaneNum)
			vecAdjFaceNums.push_back(lPlaneNum);
		else if( rPlaneNum != -1 )
			vecAdjFaceNums.push_back(rPlaneNum);
	}

	return true;
}


IEDll_API bool ReplaceLineNode(ObjectPoints& localObjPnts, const LineTopology& lineTop, const Position3D& point, double maxDistance )
{
	if (lineTop[0].Number() < 0
		|| lineTop[1].Number() < 0
		|| lineTop[0].Number() >= localObjPnts.size() 
		|| lineTop[1].Number() >= localObjPnts.size())
		return false;

	Position3D startPnt = (Position3D)localObjPnts[lineTop[0].Number()];
	Position3D endPnt = (Position3D)localObjPnts[lineTop[1].Number()];
	double dist1 = point.Distance(startPnt);
	double dist2 = point.Distance(endPnt);

	if (dist1 <= dist2 && dist1 < maxDistance) {
		localObjPnts[lineTop[0].Number()].SetX(point.X());
		localObjPnts[lineTop[0].Number()].SetY(point.Y());
		localObjPnts[lineTop[0].Number()].SetZ(point.Z());
	}
	else if(dist2 < maxDistance) {
		localObjPnts[lineTop[1].Number()].SetX(point.X());
		localObjPnts[lineTop[1].Number()].SetY(point.Y());
		localObjPnts[lineTop[1].Number()].SetZ(point.Z());
	}

	return true;
}

//attribute 1: left segment number; attribute 2: right segment number; 
//attribute 3: is stable or not; -1 -->no 1 --> yes
//attribute 4: line gradient [0-6]
//Point sequence is sensitive 
IEDll_API bool AddTopLine(ObjectPoints& localObjPnts, LineTopologies& localLineTops, const Position3D& point1, 
	const Position3D& point2, int att1, int att2, int att3, int att4)
{
	ObjectPoint objPnt;
	int nSize = localObjPnts.size();

	objPnt.X() = point1.X();
	objPnt.Y() = point1.Y();
	objPnt.Z() = point1.Z();
	objPnt.Number() = localObjPnts.size();
	localObjPnts.push_back(objPnt);

	objPnt.X() = point2.X();
	objPnt.Y() = point2.Y();
	objPnt.Z() = point2.Z();
	objPnt.Number() = localObjPnts.size();
	localObjPnts.push_back(objPnt);

	LineTopology temLineTop;
	temLineTop.push_back(PointNumber(nSize));
	temLineTop.push_back(PointNumber(nSize+1));
	temLineTop.Number() = localLineTops.size();
	temLineTop.SetAttribute(SegmentLabel, att1);
	temLineTop.SetAttribute(SecondSegmentLabel, att2);
	temLineTop.SetAttribute(2, att3);
	temLineTop.SetAttribute(3, att4);
	localLineTops.push_back(temLineTop);

	return true;
}
