/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : Oct-24-2011
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
#include "Building.h"
#include "Buildings.h"
//#include "Quadrics.h"

#include "InferenceEngine.h"
#include "GraphEditDictionary.h"
#include <vector>
#include <math.h>

using namespace std;

//line attribute tag
//left plane attribute and right plane attribute are used to store numbers of planes 
//which form the intersection line
//if the intersection only has one neighbor roof, one of the attributes is set to be -1
IEDll_API bool ReconstructBuilding(LaserPoints& lasPnts, vector<Plane>& vecPlanes, 
	Buildings& pcmBlds, ObjectPoints& modelObjPnts)
{
//	Quadrics q;

	vector<PointNumberList> gVecSegPnls;
	MyDeriveSegPNL(lasPnts, gVecSegPnls, SegmentNumberTag);



	TINEdges gEdges;
	vecPlanes.clear();
	lasPnts.DeriveTIN();
	gEdges.Derive(lasPnts.TINReference());
	assert(gEdges.size()==lasPnts.size() 
		&& "point number does not match with TIN edge number");
	lasPnts.SetAttribute(IsProcessedTag, -1); //-1 unprocessed 1 processed

	//Attribute 0: left hand face Number; Attribute 1: right hand face Number; Attribute 2: is/not refined tag
	LineTopologies localLineTops;
	if(gVecSegPnls.empty()) {
		return false;//point cloud has not been segmented
		//printf("laser point cloud has not been segmented");
	}

	vector<int> vecAllSegNumber, vecCurSegNumber;
	gVecLocalSegPnts.reserve(100);
	vector<Plane> vecLocalPlanes;
	vecLocalPlanes.reserve(100);
	ObjectPoints localObjPnts;
	Plane temPlane;
	temPlane.Label() = -1;//-1 unfitted 1 fitted
	int nBuildingCount = 0;
	int curSegNum;

	//main loop for reconstruct roof segment by segment
	for (int iSeg=0; iSeg<gVecSegPnls.size(); ++iSeg)
	{//reconstruct the building which contain current laser point segment
		if (gVecSegPnls[iSeg].size() < gBldRecPar.minSegNum
			|| lasPnts[gVecSegPnls[iSeg][0].Number()].Attribute(IsProcessedTag) != -1)
			continue;//this segment has no laser data or is processed

		gVecLocalSegPnts.clear();	
		vecLocalPlanes.clear();
		localObjPnts.clear();
		localLineTops.clear();

		//gVecLocalSegPnts.push_back();
		//curSegNum = lasPnts[gVecSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
		if(!ReconstructLocalSegments(lasPnts, gEdges, gVecSegPnls, vecLocalPlanes,
			 gVecSegPnls[iSeg], localObjPnts, localLineTops))
			 continue;

		//need to be done before refine other faces
		//MergeLocalSegments(lasPnts,gVecSegPnls, vecLocalPlanes, localObjPnts, localLineTops);
		MergeLocalSegments(lasPnts,gVecLocalSegPnts, vecLocalPlanes, localObjPnts, localLineTops);


		RefineSingleRoof(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		RefineSimpleDormer(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
    	RefineCornerOf3Faces(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		RefineCornerOf4Faces(vecLocalPlanes, localObjPnts, localLineTops);
		RefineCornerOfXFaces(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		RefineOutBoundaryLine6(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		ComputeBoundLineCoef(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		ComputeRoofFaceConf(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		//ComputeRoofFaceConf2(lasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);


		LineTopology temLineTop;
		ObjectPoint temObjPnt;
		int objPntNum;
		Building curBld(pcmBlds.NextNumber(), &modelObjPnts);
		int att1, att2;
		for (int i=0; i < localLineTops.size(); i++) {
			temLineTop = localLineTops[i];
			att1 = temLineTop.Attribute(SegmentLabel);
			att2 = temLineTop.Attribute(SecondSegmentLabel);
			AddLine2PcmBld(curBld, modelObjPnts, localObjPnts[temLineTop[0].Number()],
				localObjPnts[temLineTop[1].Number()], temLineTop.Attribute(SegmentLabel), 
				temLineTop.Attribute(SecondSegmentLabel), temLineTop.Attribute(3));
		}

		pcmBlds.push_back(curBld);

	}

	lasPnts.RemoveAttribute(IsProcessedTag); 

	return true;
}

IEDll_API bool ReconstructBldWithRidges(LaserPoints& gLasPnts, Building& bld, ObjectPoints& modelObjPnts)
{
	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(gLasPnts, vecSegPnls, SegmentNumberTag); 
	vector<int> vecSegNums;

	LaserPoints lBldPnts;
	int segInd;
	if (bld.MapDataPtr() == NULL) {
		lBldPnts = gLasPnts;
	}
	else {
		SearchSegNumsInBld(bld, vecSegNums);
		for (int iSeg=0; iSeg<vecSegNums.size(); ++iSeg) {
			segInd = IndexSegBySegNumber(gLasPnts, vecSegPnls, vecSegNums[iSeg]);
			if (segInd == -1) continue;
			for (int iPnt=0; iPnt<vecSegPnls[segInd].size(); ++iPnt)
				lBldPnts.push_back(gLasPnts[vecSegPnls[segInd][iPnt].Number()]);
		}
	}

	LineTopologies localLineTops;
	ObjectPoints localObjPnts;
	LineTopologies* bldLineTops = bld.MapDataPtr();
	LineTopologies::iterator line;
	int pntNum1, pntNum2;
	if (bldLineTops) {
		for (line=bldLineTops->begin(); line!=bldLineTops->end(); line++) {
			pntNum1 = (*line)[0].Number();
			pntNum2 = (*line)[1].Number();
			if (modelObjPnts[pntNum1].Distance(modelObjPnts[pntNum2])<0.05)
				continue;
			AddTopLine(localObjPnts, localLineTops, modelObjPnts[pntNum1], modelObjPnts[pntNum2],
				line->Attribute(SegmentLabel), line->Attribute(SecondSegmentLabel));
		}
	}


	vector<PointNumberList> vecLocalSegPnls;
	vector<Plane> vecLocalPlanes;
	MyDeriveSegPNL(lBldPnts, vecLocalSegPnls, SegmentNumberTag);
	Plane temPlane;
	for (int iSeg=0; iSeg<vecLocalSegPnls.size(); ++iSeg) {
		temPlane = lBldPnts.FitPlane(vecLocalSegPnls[iSeg]);
		temPlane.Number() = lBldPnts[vecLocalSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
		vecLocalPlanes.push_back(temPlane);
	}

	RefineSingleRoof(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);
	RefineSimpleDormer(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);
	RefineCornerOf3Faces(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);
	RefineCornerOf4Faces(vecLocalPlanes, localObjPnts, localLineTops);
	RefineCornerOfXFaces(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);
	RefineOutBoundaryLine6(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);

	//ComputeBoundLineCoef()
	ComputeBoundLineCoef(lBldPnts, vecLocalPlanes, vecLocalSegPnls, localObjPnts, localLineTops);
	ComputeRoofFaceConf(gLasPnts, vecLocalPlanes, vecSegPnls, localObjPnts, localLineTops);

	if (bld.MapDataPtr() != NULL)
		bld.MapDataPtr()->clear();//clear original data
	LineTopology temLineTop;
	ObjectPoint temObjPnt;
	int objPntNum;
	int att1, att2;
	for (int i=0; i < localLineTops.size(); i++) {
		temLineTop = localLineTops[i];
		att1 = temLineTop.Attribute(SegmentLabel);
		att2 = temLineTop.Attribute(SecondSegmentLabel);
		AddLine2PcmBld(bld, modelObjPnts, localObjPnts[temLineTop[0].Number()],
			localObjPnts[temLineTop[1].Number()], temLineTop.Attribute(SegmentLabel), 
			temLineTop.Attribute(SecondSegmentLabel), temLineTop.Attribute(3));
	}

	return true;
}

IEDll_API bool ReconstructBuildingLOD1(LaserPoints& gLasPnts, Buildings& pcmBlds, ObjectPoints& modelObjPnts, const OutliningParameters& para)
{
	vector<PointNumberList> gVecSegPnls;
	MyDeriveSegPNL(gLasPnts, gVecSegPnls, SegmentNumberTag);


	LineTopologies localLineTops;

	vector<int> vecAllSegNumber, vecCurSegNumber;
	gVecLocalSegPnts.reserve(100);
	vector<Plane> vecLocalPlanes;
	vecLocalPlanes.reserve(100);
	ObjectPoints localObjPnts;
	Plane temPlane;
	temPlane.Label() = -1;//-1 unfitted 1 fitted
	int nBuildingCount = 0;
	int curSegNum;

	//main loop for reconstruct roof segment by segment
	for (int iSeg=0; iSeg<gVecSegPnls.size(); ++iSeg)
	{//reconstruct the building which contain current laser point segment
		if (gVecSegPnls[iSeg].size() < gBldRecPar.minSegNum) continue;

		gVecLocalSegPnts.clear();	
		vecLocalPlanes.clear();
		localObjPnts.clear();
		localLineTops.clear();

		curSegNum = gLasPnts[gVecSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
		temPlane = gLasPnts.FitPlane(gVecSegPnls[iSeg], Vector3D(0.0, 0.0, 1.0), curSegNum);
		vecLocalPlanes.push_back(temPlane);
		gVecLocalSegPnts.push_back(gVecSegPnls[iSeg]);
		
		RefineOutBoundaryLine8(gLasPnts, vecLocalPlanes, gVecSegPnls[iSeg], localObjPnts, localLineTops);

		ComputeBoundLineCoef(gLasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
		ComputeRoofFaceConf(gLasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);


		LineTopology temLineTop;
		ObjectPoint temObjPnt;
		int objPntNum;
		Building curBld(pcmBlds.NextNumber(), &modelObjPnts);
		int att1, att2;
		for (int i=0; i < localLineTops.size(); i++) {
			temLineTop = localLineTops[i];
			att1 = temLineTop.Attribute(SegmentLabel);
			att2 = temLineTop.Attribute(SecondSegmentLabel);
			AddLine2PcmBld(curBld, modelObjPnts, localObjPnts[temLineTop[0].Number()],
				localObjPnts[temLineTop[1].Number()], temLineTop.Attribute(SegmentLabel), 
				temLineTop.Attribute(SecondSegmentLabel), temLineTop.Attribute(3));
		}

		pcmBlds.push_back(curBld);

	}


	return true;
}

IEDll_API bool ReconstructBuildingLOD1(LaserPoints& gLasPnts, int segNum, Building& bld, ObjectPoints& modelObjPnts, const OutliningParameters& para)
{
	vector<PointNumberList> gVecSegPnls;
	MyDeriveSegPNL(gLasPnts, gVecSegPnls, SegmentNumberTag);
	int indSeg = IndexSegBySegNumber(gLasPnts, gVecSegPnls, segNum);
	if (gVecSegPnls[indSeg].size() < gBldRecPar.minSegNum) return false;
	Plane plane = gLasPnts.FitPlane(gVecSegPnls[indSeg], Vector3D(0.0, 0.0, 1.0), segNum);

	LineTopologies localLineTops;
	ObjectPoints localObjPnts;
	vector<Plane> vecLocalPlanes;
	
	vecLocalPlanes.push_back(plane);
	gVecLocalSegPnts.clear();
	gVecLocalSegPnts.push_back(gVecSegPnls[indSeg]);

	RefineOutBoundaryLine8(gLasPnts, vecLocalPlanes, gVecSegPnls[indSeg],localObjPnts, localLineTops);

	ComputeBoundLineCoef(gLasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
	ComputeRoofFaceConf(gLasPnts, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);


	if (bld.MapDataPtr() != NULL) bld.MapDataPtr()->clear();//clear original data

	LineTopology temLineTop;
	ObjectPoint temObjPnt;
	int objPntNum;
	int att1, att2;
	for (int i=0; i < localLineTops.size(); i++) {
		temLineTop = localLineTops[i];
		att1 = temLineTop.Attribute(SegmentLabel);
		att2 = temLineTop.Attribute(SecondSegmentLabel);
		AddLine2PcmBld(bld, modelObjPnts, localObjPnts[temLineTop[0].Number()],
			localObjPnts[temLineTop[1].Number()], temLineTop.Attribute(SegmentLabel), 
			temLineTop.Attribute(SecondSegmentLabel), temLineTop.Attribute(3));
	}

	return true;
}

IEDll_API bool ScatterBuilding(const LaserPoints& gLasPnts, Building& bld, ObjectPoints& modelObjPnts)
{
	LineTopologies newMapData;
	LineTopologies::iterator line;
	int lPlaneNum, rPlaneNum;

	for (line=bld.MapDataPtr()->begin(); line!=bld.MapDataPtr()->end(); ++line)	{
		lPlaneNum = line->Attribute(SegmentLabel);
		rPlaneNum = line->Attribute(SecondSegmentLabel);
		if(lPlaneNum != -1 && rPlaneNum != -1)
			newMapData.push_back(*line);
	}

	//bld.MapDataPtr();
	//bld.AddMapData(newMapData);
	bld.MapDataPtr()->clear();
	//bld.MapDataPtr()->insert(bld.MapDataPtr()->end(), newMapData.begin(), newMapData.end());
	if (newMapData.empty()) {
		return false;
	}
	else {
		for (line=newMapData.begin(); line != newMapData.end(); ++line) 
			bld.AddMapData(*line);
		return true;
	}
	

}

IEDll_API bool AddLine2PcmBld(Building& bld, ObjectPoints& modelObjPnts, Position3D startPnt, 
	Position3D endPnt, int leftRoofNum, int rightRoofNum, int gradient)
{
	int nObjPntNum = modelObjPnts.size();
	ObjectPoint objPnt;

	//objPnt.X() = startPnt.X();
	objPnt.Position3DRef() = startPnt;
	objPnt.Number() = nObjPntNum;
	modelObjPnts.push_back(objPnt);

	objPnt.Position3DRef() = endPnt;
	objPnt.Number() = nObjPntNum+1;
	modelObjPnts.push_back(objPnt);

	LineTopology topLine;
	topLine.push_back(PointNumber(nObjPntNum));
	topLine.push_back(PointNumber(nObjPntNum+1));

	topLine.SetAttribute(SegmentLabel, leftRoofNum);
	topLine.SetAttribute(SecondSegmentLabel, rightRoofNum);
	topLine.SetAttribute(BuildingNumberTag, bld.Number());
	topLine.Label() = MapLabel;
	topLine.Attribute(GradientTag) = gradient;
	bld.AddMapData(topLine);

	return true;
}


IEDll_API int IndPcmBldBySegNum(Buildings& blds, int segNum)
{
	int bldInd = -1;
	Buildings::iterator bld;
	LineTopologies* polygons;
	int lSegNum, rSegNum;

	for (bld = blds.begin(); bld != blds.end(); ++bld) {
		polygons = bld->MapDataPtr();
		if (polygons == NULL) continue;

		for (int j=0; j<polygons->size(); ++j) {
			lSegNum = (*polygons)[j].Attribute(SegmentLabel);
			rSegNum = (*polygons)[j].Attribute(SecondSegmentLabel);
			if (lSegNum == segNum || rSegNum == segNum)	{
				bldInd = bld - blds.begin();
				return bldInd;
			}
		}
	}

	return -1;
}


IEDll_API int IndPcmBldByPolygonID(Buildings& blds, int polID)
{
	int bldInd = -1;
	Buildings::iterator bld;
	LineTopologies* polygons;

	for (bld = blds.begin(); bld != blds.end(); ++bld) {
		polygons = bld->MapDataPtr();
		if (polygons == NULL || polygons->empty()) continue;
		if (!(*polygons)[0].HasAttribute(IDNumberTag)) continue;
		
		if ((*polygons)[0].Attribute(IDNumberTag) == polID)
			return bld-blds.begin();
	}

	return -1;
}


IEDll_API void SearchSegNumsInBld(Building& bld, vector<int>& vecSegNums )
{
	vecSegNums.clear();

	LineTopologies* polygons;
	int lSegNum, rSegNum;
	vector<int> vecTemSegNums;
	
	polygons = bld.MapDataPtr();
	//no map top, return directly
	if (polygons == NULL) return ;

	for (int i=0; i<polygons->size(); ++i) {
		lSegNum = (*polygons)[i].Attribute(SegmentLabel);
		rSegNum = (*polygons)[i].Attribute(SecondSegmentLabel);
		if (lSegNum != -1) vecTemSegNums.push_back(lSegNum);
		if (rSegNum != -1) vecTemSegNums.push_back(rSegNum);
	}

	std::sort(vecTemSegNums.begin(), vecTemSegNums.end());
	vector<int>::iterator itrLast;
	itrLast = std::unique(vecTemSegNums.begin(), vecTemSegNums.end());
//	vecSegNums.reserve(vecTemSegNums.size());
	vecSegNums.insert(vecSegNums.begin(), vecTemSegNums.begin(), itrLast);
}

IEDll_API void SearchSegNumsInBlds(Buildings& blds, std::vector<int>& vecSegNums)
{
	vecSegNums.clear();
	//	vecSegNums.reserve(blds.size()*8);
	vector<int> temSegNums;
	for (int i=0; i<blds.size(); ++i) {
		SearchSegNumsInBld(blds[i], temSegNums);
		vecSegNums.insert(vecSegNums.end(), temSegNums.begin(), temSegNums.end());
	}

	temSegNums =  vecSegNums;
	std::sort(temSegNums.begin(), temSegNums.end());
	vector<int>::iterator itrLast;
	itrLast = std::unique(temSegNums.begin(), temSegNums.end());
	//	vecSegNums = vector<int>(temSegNums.begin(), itrLast);
	vecSegNums.clear();
	vecSegNums.insert(vecSegNums.begin(), temSegNums.begin(), itrLast);


}

IEDll_API void SearchSegNumsInBld(LineTopologies& topLines, vector<int>& vecSegNums )
{
	vecSegNums.clear();

	int lSegNum, rSegNum;
	vector<int> vecTemSegNums;
	LineTopologies::iterator itrLine;

	for (int i=0; i<topLines.size(); ++i) {
		itrLine= topLines.begin()+i;
		lSegNum = itrLine->Attribute(SegmentLabel);
		rSegNum = itrLine->Attribute(SecondSegmentLabel);
		if (lSegNum != -1) vecTemSegNums.push_back(lSegNum);
		if (rSegNum != -1) vecTemSegNums.push_back(rSegNum);
	}

	std::sort(vecTemSegNums.begin(), vecTemSegNums.end());
	vector<int>::iterator itrLast;
	itrLast = std::unique(vecTemSegNums.begin(), vecTemSegNums.end());
	//	vecSegNums.reserve(vecTemSegNums.size());
	vecSegNums.insert(vecSegNums.begin(), vecTemSegNums.begin(), itrLast);
}

IEDll_API LineTopologies SearchRidgesInBld(Building& bld)
{
	LineTopologies ltResult;
	if (!bld.MapDataPtr()) return ltResult;

	ltResult = *(bld.MapDataPtr());
	for (int iLine=0; iLine<ltResult.size(); iLine++) {
		if (ltResult[iLine].Attribute(SegmentLabel) == -1
			||ltResult[iLine].Attribute(SecondSegmentLabel) == -1)
		{
			ltResult.erase(ltResult.begin()+iLine);
			iLine--;
		}
	}

	return ltResult;
}

IEDll_API LineTopologies SearchRidgesInBld(LineTopologies& lineTops)
{
	for (int iLine=0; iLine<lineTops.size(); iLine++) {
		if (lineTops[iLine].Attribute(SegmentLabel) == -1
			||lineTops[iLine].Attribute(SecondSegmentLabel) == -1)
		{
			lineTops.erase(lineTops.begin()+iLine);
			iLine--;
		}
	}

	return lineTops;
}

IEDll_API bool SearchLineBy2FacesPcm(const LineTopologies& localLineTops, const int faceNum1, 
	const int faceNum2, int& indLine)
{
	LineTopology temLineTop;
	bool bFlag = false;
	vector<int> vecTemLineTopNums;
	indLine = -1;

	SearchLinesOnPlanePcm(localLineTops, faceNum1, vecTemLineTopNums);

	for (int i=0; i<vecTemLineTopNums.size(); i++) {
		temLineTop = localLineTops[vecTemLineTopNums[i]];

		if (temLineTop.Attribute(SegmentLabel) == faceNum2 
			|| temLineTop.Attribute(SecondSegmentLabel) == faceNum2)	{
				indLine = vecTemLineTopNums[i];
				bFlag = true;
				break;
		}
	}

	return bFlag;
}

IEDll_API std::vector<int> CollectRoofFacesByLines(const LineTopologies& localLineTops)
{
	//////////////////////////////////////////////////////////////////////////
	//Get all plane num in the building
	LineTopologies::const_iterator line;
	vector<int> vecFaceNums;
	int planeNum1, planeNum2;
	for (line=localLineTops.begin(); line!=localLineTops.end(); ++line) {
		if (!line->HasAttribute(SegmentLabel) || !line->HasAttribute(SecondSegmentLabel))
			continue;
		planeNum1 = line->Attribute(SegmentLabel);
		planeNum2 = line->Attribute(SecondSegmentLabel);
		if (planeNum1!=-1)
			vecFaceNums.push_back(planeNum1);
		if (planeNum2!=-1)
			vecFaceNums.push_back(planeNum2);
	}
	std::sort(vecFaceNums.begin(), vecFaceNums.end());
	vector<int>::iterator endItr = std::unique(vecFaceNums.begin(), vecFaceNums.end());
	vecFaceNums.erase(endItr, vecFaceNums.end());

	return vecFaceNums;
}

#include <set>
IEDll_API std::vector<std::set<int> > CollectConnectedRoofFacesByLines(const LineTopologies& localLineTops)
{
	std::vector<std::set<int> > vecConnectedFaces;
	std::vector<std::set<int> >::iterator curConFaces;

	std::vector<bool> vecUsed(localLineTops.size(), false);
	int planeNum1, planeNum2;
	bool bNewConnection;


	for (LineTopologies::const_iterator line0=localLineTops.begin(); line0!=localLineTops.end(); ++line0) {
		if (vecUsed[line0-localLineTops.begin()]) continue;
		if (!line0->HasAttribute(SegmentLabel) || !line0->HasAttribute(SecondSegmentLabel)) continue;
		planeNum1 = line0->Attribute(SegmentLabel);
		planeNum2 = line0->Attribute(SecondSegmentLabel);
		
		//create new components
		vecConnectedFaces.push_back(std::set<int>());
		curConFaces = vecConnectedFaces.end()-1;
		if (planeNum1!=-1) curConFaces->insert(planeNum1);
		if (planeNum2!=-1) curConFaces->insert(planeNum2);
		vecUsed[line0-localLineTops.begin()] = true;


		//look for all connecting roof faces
		bNewConnection = true;
		while (bNewConnection) {
			bNewConnection = false;

			for (LineTopologies::const_iterator line1=localLineTops.begin(); line1!=localLineTops.end(); ++line1) {
				if (vecUsed[line1-localLineTops.begin()]) continue;
				if (!line1->HasAttribute(SegmentLabel) || !line1->HasAttribute(SecondSegmentLabel)) continue;
				planeNum1 = line1->Attribute(SegmentLabel);
				planeNum2 = line1->Attribute(SecondSegmentLabel);

				if (curConFaces->count(planeNum1) || curConFaces->count(planeNum2)) {
					if (planeNum1!=-1) curConFaces->insert(planeNum1);
					if (planeNum2!=-1) curConFaces->insert(planeNum2);
					bNewConnection = true;
					vecUsed[line1-localLineTops.begin()] = true;
				}
			}
		}
	}


	return vecConnectedFaces;
}

IEDll_API bool SearchLinesOnPlanePcm(const LineTopologies& localLineTops, 
	const int curPlaneNum, std::vector<int>& vecOutTopNums)
{
	LineTopologies::const_iterator itr;
	vecOutTopNums.clear();

	for (itr=localLineTops.begin(); itr!=localLineTops.end(); ++itr){
		if (itr->Attribute(SegmentLabel) == curPlaneNum
			|| itr->Attribute(SecondSegmentLabel) == curPlaneNum) {
				vecOutTopNums.push_back(itr-localLineTops.begin());
		}
	}

	if (vecOutTopNums.size()>0)
		return true;
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////
//remove redundant lines
void RemoveRedundentLines(const ObjectPoints& ObjPnts, const LineTopologies& orgLineTops, vector<int>& vecLineNums)
{
	LineTopologies::const_iterator line1, line2;
	ObjectPoints::const_iterator pnt1, pnt2, pnt3, pnt4;
	for (int iLine=0; iLine<vecLineNums.size(); ++iLine) {
		line1 = orgLineTops.begin()+vecLineNums[iLine];
		pnt1 = ObjPnts.begin() + (*line1)[0].Number();
		pnt2 = ObjPnts.begin() + (*line1)[1].Number();

		for (int jLine=iLine+1; jLine<vecLineNums.size(); ++jLine) {
			line2 = orgLineTops.begin()+vecLineNums[jLine];
			pnt3 = ObjPnts.begin() + (*line2)[0].Number();
			pnt4 = ObjPnts.begin() + (*line2)[1].Number();
			if ((pnt1->Position3DRef()==pnt3->Position3DRef() 
				&&pnt2->Position3DRef()==pnt4->Position3DRef()) 
				|| (pnt1->Position3DRef()==pnt4->Position3DRef() 
				&&pnt2->Position3DRef()==pnt3->Position3DRef())) {
					vecLineNums.erase(vecLineNums.begin()+jLine);
					jLine--;
			}
		}
	}
}

void RemoveZeroLengthLines(const ObjectPoints& ObjPnts, const LineTopologies& orgLineTops, vector<int>& vecLineNums)
{
	LineTopologies::const_iterator line0;
	ObjectPoints::const_iterator pnt0, pnt1;

	for (int iLine=0; iLine<vecLineNums.size(); ++iLine) {
		line0 = orgLineTops.begin()+vecLineNums[iLine];
		pnt0 = ObjPnts.begin() + (*line0)[0].Number();
		pnt1 = ObjPnts.begin() + (*line0)[1].Number();

		if (pnt0->X()==pnt1->X() && 
			pnt0->Y()==pnt1->Y() && 
			pnt0->Z()==pnt1->Z()) 
		{
			vecLineNums.erase(vecLineNums.begin()+iLine);
			iLine--;
		}
	}
}


bool RemoveLooseLines(const ObjectPoints& localobjPnts, 
	const LineTopologies& localLineTops, vector<int>& vecTopInds)
{
	bool bRep1, bRep2;
	ObjectPoints::const_iterator obj11, obj12, obj21, obj22;
	int lineNum;

	for (int i=0; i<vecTopInds.size(); i++)	{
		bRep1 = bRep2 = false;
		lineNum = vecTopInds[i];
		obj11 = localobjPnts.begin()+localLineTops[vecTopInds[i]][0].Number();
		obj12 = localobjPnts.begin()+localLineTops[vecTopInds[i]][1].Number();

		for (int j=0; j<vecTopInds.size(); j++)	{
			if (i==j) continue;
			obj21 = localobjPnts.begin()+localLineTops[vecTopInds[j]][0].Number();
			obj22 = localobjPnts.begin()+localLineTops[vecTopInds[j]][1].Number();

			if (obj11->Position3DRef()==obj21->Position3DRef()
				||obj11->Position3DRef()==obj22->Position3DRef())
				bRep1 = true;
			if (obj12->Position3DRef()==obj21->Position3DRef()
				||obj12->Position3DRef()==obj22->Position3DRef())
				bRep2 = true;
		}
		if (!(bRep1&&bRep2))
			vecTopInds.erase(vecTopInds.begin()+i);
	}

	return true;
}


int IndexStartLine(const ObjectPoints& ObjPnts, const LineTopologies& orgLineTops,
	const vector<int>& vecLineNums, const vector<char>& vecLineProcessed)
{
	int indStartLine = -1;
	LineTopologies::const_iterator line1, line2;
	ObjectPoints::const_iterator pnt1, pnt2, pnt3, pnt4;
	//0.000001;
	double dist13, dist14, dist23, dist24;

	for (int iLine=0; iLine<vecLineNums.size(); ++iLine) {
		if(vecLineProcessed[iLine])
			continue;

		line1 = orgLineTops.begin()+vecLineNums[iLine];
		pnt1 = ObjPnts.begin() + (*line1)[0].Number();
		pnt2 = ObjPnts.begin() + (*line1)[1].Number();

		for (int jLine=iLine+1; jLine<vecLineNums.size(); ++jLine) {
			line2 = orgLineTops.begin()+vecLineNums[jLine];
			pnt3 = ObjPnts.begin() + (*line2)[0].Number();
			pnt4 = ObjPnts.begin() + (*line2)[1].Number();
			
			dist13 = (pnt1->pos()-pnt3->pos()).Length();
			dist14 = (pnt1->pos()-pnt4->pos()).Length();
			dist23 = (pnt2->pos()-pnt3->pos()).Length();
			dist24 = (pnt2->pos()-pnt4->pos()).Length();

			if (dist13<0.000001 || dist14<0.000001 ||
				dist23<0.000001 || dist24<0.000001) {
				indStartLine = iLine;
				goto endFlag;
			}
			
			//if (pnt1->Position3DRef()==pnt3->Position3DRef() ||
			//	pnt1->Position3DRef()==pnt4->Position3DRef() ||
			//	pnt2->Position3DRef()==pnt3->Position3DRef() ||
			//	pnt2->Position3DRef()==pnt4->Position3DRef()) {
			//	indStartLine = iLine;
			//	goto endFlag;
			//}
		}
	}

endFlag:
	return indStartLine;
}


IEDll_API void ClosePolygon(const LineTopologies& orgLineTops, const ObjectPoints& ObjPnts, LineTopologies& destLineTops)
{
	if (orgLineTops.empty() || ObjPnts.empty()) return;
	destLineTops.clear();

	LineTopology polygon;
	LineTopologies::const_iterator line;
	int planeNum1, planeNum2;

	//////////////////////////////////////////////////////////////////////////
	//Get all plane num in the building
	vector<int> vecPlanNums;
	for (line=orgLineTops.begin(); line!=orgLineTops.end(); ++line) {
		if (!line->HasAttribute(SegmentLabel) || !line->HasAttribute(SecondSegmentLabel))
			continue;
		planeNum1 = line->Attribute(SegmentLabel);
		planeNum2 = line->Attribute(SecondSegmentLabel);
		if (planeNum1!=-1)
			vecPlanNums.push_back(planeNum1);
		if (planeNum2!=-1)
			vecPlanNums.push_back(planeNum2);
	}
	std::sort(vecPlanNums.begin(), vecPlanNums.end());
	vector<int>::iterator endItr = std::unique(vecPlanNums.begin(), vecPlanNums.end());
	vecPlanNums.erase(endItr, vecPlanNums.end());

	//////////////////////////////////////////////////////////////////////////
	//enclose lines on one plane
	vector<int> vecLineNums;
	Position3D curPnt, startPnt;
	vector<char> vecIsProcessed;
	bool bIsClosed, bNewLine;
	int nLoops;
	int indStartLine;
	int curPlanNum, curLineNum;
	int pntNum1, pntNum2;

	//for all roofs
	for (int iSeg=0; iSeg<vecPlanNums.size(); ++iSeg) {
		curPlanNum = vecPlanNums[iSeg];
		if (curPlanNum == 35)	{
			int aaaa = 0;
		}
		SearchLinesOnPlanePcm(orgLineTops, vecPlanNums[iSeg], vecLineNums);
		RemoveZeroLengthLines(ObjPnts, orgLineTops, vecLineNums);
		RemoveRedundentLines(ObjPnts, orgLineTops, vecLineNums);
		//RemoveLooseLines(ObjPnts, orgLineTops, vecLineNums);

		if (vecLineNums.size() < 3)
			continue;//not enough lines to form polygon

		vecIsProcessed = vector<char>(vecLineNums.size(), 0);
//		indStartLine = IndexStartLine(ObjPnts,orgLineTops, vecLineNums, vecIsProcessed);
//		if (indStartLine == -1) continue;
		//search all polygons in one roof
		while (-1 != (indStartLine = IndexStartLine(ObjPnts,orgLineTops, vecLineNums, vecIsProcessed))) {
			polygon.clear();
			polygon = orgLineTops[vecLineNums[indStartLine]];
			polygon.Attribute(SegmentLabel) = vecPlanNums[iSeg];
			polygon.RemoveAttribute(SecondSegmentLabel);

			bIsClosed = false;
			vecIsProcessed[indStartLine] = true;
			startPnt = ObjPnts[polygon[0].Number()].Position3DRef();
			bNewLine = true;
			nLoops = 0;

			bIsClosed = false;
			while(!bIsClosed && bNewLine && nLoops<500) {
				nLoops++;
				curPnt = ObjPnts[polygon[polygon.size()-1].Number()];
				for (int iLine=0; iLine<vecLineNums.size(); ++iLine) {
					if (vecIsProcessed[iLine] == true)
						continue;

					bNewLine = false;
					curLineNum = vecLineNums[iLine];
					line = orgLineTops.begin()+vecLineNums[iLine];

					pntNum1 = (*line)[0].Number();
					pntNum2 = (*line)[1].Number();

					double dist1 = ObjPnts[pntNum1].Position3DRef().Distance(curPnt);
					double dist2 = ObjPnts[pntNum2].Position3DRef().Distance(curPnt);
					if (ObjPnts[pntNum2].Position3DRef().Distance(curPnt)<=0.000001) 
						std::swap(pntNum1, pntNum2);
					if (ObjPnts[pntNum1].Position3DRef().Distance(curPnt)>0.000001)
						continue;
					vecIsProcessed[iLine] = true;

					//close polygon
					double dist2StartPnt = ObjPnts[pntNum2].Position3DRef().Distance(startPnt);
					if (dist2StartPnt<0.000001) {
						polygon.push_back(polygon[0].Number());
						bIsClosed = true;
					}
					else {//add new point
						polygon.push_back(PointNumber(pntNum2));
						bNewLine = true;
					}



					if (bIsClosed || bNewLine) break;
				}
			}//end single polygon

			//add polygon
			if (polygon[0].Number() == polygon[polygon.size()-1].Number())
				destLineTops.push_back(polygon);//only store closed polygon
		}//end one roof

	}//end all roofs

#ifdef _DEBUG
	ObjPnts.Write("debug.objpts");
	destLineTops.Write("debug.top");
#endif


	LineTopologies::iterator iPolyg;
	double dist;


	for (iPolyg = destLineTops.begin(); iPolyg!= destLineTops.end(); ++iPolyg) {
		for (int j=1; j<iPolyg->size(); j++) {
			pntNum1 = (*iPolyg)[j-1].Number();
			pntNum2 = (*iPolyg)[j].Number();

			dist = ObjPnts[pntNum1].Distance(ObjPnts[pntNum2].Position3DRef());
			if (dist < 0.0005) {
				iPolyg->erase(iPolyg->begin() + j);
				j--;
			}
		}

		if ((*iPolyg)[0] != (*iPolyg)[iPolyg->size()-1])
			iPolyg->push_back((*iPolyg)[0]);
	}

#ifdef _DEBUG
	ObjPnts.Write("debug.objpts");
	destLineTops.Write("debug.top");
#endif
}


IEDll_API void ComputeBoundLineCoef(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	LineTopologies::iterator lineTop;
	int lRoofNum, rRoofNum;
	Line3D line;
	double bufferWidth, bufferLeng;
	int indPlane, indSeg;
	double coef;
	int nValidRoofs;
	ObjectPoints::iterator objPnt1, objPnt2;

	for (lineTop=localLineTops.begin(); lineTop!=localLineTops.end(); ++lineTop) {
		objPnt1 = localObjPnts.begin()+(*lineTop)[0].Number();
		objPnt2 = localObjPnts.begin()+(*lineTop)[1].Number();

		if (objPnt1->Distance(objPnt2->Position3DRef())==0.0){
			lineTop->Attribute(3) = 0;
			continue;
		}

		line = Line3D(objPnt1->Position3DRef(), objPnt2->Position3DRef());
		bufferWidth = objPnt1->Distance(*objPnt2);
		coef = 0.0;
		nValidRoofs = 0;

		//left roof
		lRoofNum = lineTop->Attribute(SegmentLabel);
		if (lRoofNum>=0) {
			indPlane = IndexPlaneBySegNumber(vecLocalPlanes, lRoofNum);
			indSeg = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, lRoofNum);
			if (indPlane<0 || indSeg<0)
				continue;

			coef += ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[indSeg], 
				vecLocalPlanes[indPlane], line, 1.5, bufferWidth);
			nValidRoofs++;
		}

		//right roof
		rRoofNum = lineTop->Attribute(SecondSegmentLabel);
		if (rRoofNum >= 0) {
			indPlane = IndexPlaneBySegNumber(vecLocalPlanes, rRoofNum);
			indSeg = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, rRoofNum);
			if (indPlane<0 || indSeg<0)
				continue;

			coef += ComputeBoundLineCoef(gLaserPnts, vecLocalSegPnts[indSeg], 
				vecLocalPlanes[indPlane], line, 1.5, bufferWidth);
			nValidRoofs++;
		}

		if (nValidRoofs!=0)
			coef /= nValidRoofs;

		lineTop->Attribute(3) = int(coef*6+0.5);
	}
}


IEDll_API void ComputeRoofFaceConf(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, 
	LineTopologies& localLineTops)
{
	double sigma=0.3;
	double lamida = 0.5;


	vector<int> roofFaceNums = CollectRoofFacesByLines(localLineTops);
	
	int indSeg, indPolygon, indPlane, roofNum;
	double lineCoef, pntCoef, lineLeng, lineLengSum;
	double pntDist, temPntDist;
	LineTopologies roofPolygons;
	LineTopologies roofLines;
	vector<int> roofLineInds;
	LineTopologies::iterator roofLine;
	LineTopologies::iterator polygon;
	std::vector<PointNumberList>::iterator curSeg;
	LaserPoints::iterator curPnt;
	vector<double> vecDist;
	vector<Line3D> vecPolyLines;
	ObjectPoints::iterator itrPnt1, itrPnt2;

	//ClosePolygon(localLineTops, localObjPnts, roofPolygons);
	for (int iRoof=0; iRoof<roofFaceNums.size(); ++iRoof) {
		roofNum = roofFaceNums[iRoof];
		indSeg = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, roofNum);
		indPlane = IndexPlaneBySegNumber(vecLocalPlanes, roofNum);
		if (indSeg==-1 || indPlane==-1) continue;
		curSeg = vecLocalSegPnts.begin()+indSeg;

		//calculate line confidence for one roof face
		SearchLinesOnPlane(localLineTops, roofNum, roofLineInds);
		lineCoef = 0.0;
		lineLengSum = 0.0;
		roofLines.clear();
		vecPolyLines.clear();
		for (int iLine=0; iLine<roofLineInds.size(); ++iLine) {
			roofLine = localLineTops.begin() + roofLineInds[iLine];
			roofLines.push_back(*roofLine);
			lineLeng = localObjPnts[(*roofLine)[0].Number()].Distance(
				localObjPnts[(*roofLine)[1].Number()]);
			lineCoef += lineLeng*roofLine->Attribute(3)/7.0;
			lineLengSum += lineLeng;
			itrPnt1 = localObjPnts.begin()+(*roofLine)[0].Number();
			itrPnt2 = localObjPnts.begin()+(*roofLine)[1].Number();
			
			if (itrPnt1->Distance(*itrPnt2)==0.0)
				continue;

			vecPolyLines.push_back(Line3D(itrPnt1->Position3DRef(), itrPnt2->Position3DRef()));
			//vecPolyLines.push_back(Line3D(localObjPnts[(*roofLine)[0].Number()],
			//	localObjPnts[(*roofLine)[1].Number()]));
		}
		lineCoef /= lineLengSum;

		//calculate face confidence for one roof face
		vecDist.clear();
		pntCoef = 0.0;
		ClosePolygon(roofLines, localObjPnts, roofPolygons);
		for (int iPnt=0; iPnt<curSeg->size(); ++iPnt) {
			curPnt = gLaserPnts.begin()+(*curSeg)[iPnt].Number();
			pntDist = 2.0;
			for(int iPolygon=0; iPolygon<roofPolygons.size(); ++iPolygon){
				if (curPnt->InsidePolygon(localObjPnts, roofPolygons[iPolygon]))
					pntDist = fabs(vecLocalPlanes[indPlane].Distance(*curPnt));
			}
			
			/////////////
			//use orthogonal distance to polygon
			if (pntDist == 2.0)	{
				//pntDist =  Distance2Polygons(roofPolygons, localObjPnts, *curPnt);
				pntDist =  Distance2Polygons(vecPolyLines, *curPnt);
			}

			vecDist.push_back(pntDist);

			curPnt->Residual() = pntDist;

			//vecDist.push_back(pntDist);
			pntDist = exp(-pntDist*pntDist/(2*sigma*sigma));
			//vecCoef.push_back(pntDist);
			curPnt->SetAttribute(LabelTag, 10-int(10*pntDist));//isInside
			pntCoef += pntDist;
		}
		pntCoef /= curSeg->size();
		
		//for debug
		//Histgram(vecDist, 1.0, 0.0, 0.02);

		for (int iPnt=0; iPnt<curSeg->size(); ++iPnt) {
			//gLaserPnts[(*curSeg)[iPnt].Number()].SetAttribute(
			//	LabelTag, int(10*(lamida*lineCoef+(1-lamida)*pntCoef)));
			//gLaserPnts[(*curSeg)[iPnt].Number()].SetAttribute(
			//	ResidualTag, 10-int(10*pntCoef));
		}
	}



}


IEDll_API void ComputeRoofFaceConf2(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, 
	LineTopologies& localLineTops)
{
	double sigma=0.035;
	double lamida = 0.5;

	vector<int> roofFaceNums = CollectRoofFacesByLines(localLineTops);

	int indSeg, indPolygon, indPlane, roofNum;
	double lineCoef, pntCoef, lineLeng, lineLengSum;
	double pntDist, temPntDist;
	LineTopologies roofPolygons;
	LineTopologies roofLines;
	vector<int> roofLineInds;
	LineTopologies::iterator roofLine;
	LineTopologies::iterator polygon;
	std::vector<PointNumberList>::iterator curSeg;
	LaserPoints::iterator curPnt;
	vector<double> vecDist;
	vector<Line3D> vecPolyLines;
	ObjectPoints::iterator itrPnt1, itrPnt2;
	vector<int> vecHistOfRes;
	vecHistOfRes.reserve(500);
	double gauss, obsP;

	//ClosePolygon(localLineTops, localObjPnts, roofPolygons);
	for (int iRoof=0; iRoof<roofFaceNums.size(); ++iRoof) {
		roofNum = roofFaceNums[iRoof];
		indSeg = IndexSegBySegNumber(gLaserPnts, vecLocalSegPnts, roofNum);
		indPlane = IndexPlaneBySegNumber(vecLocalPlanes, roofNum);
		if (indSeg==-1 || indPlane==-1) continue;
		curSeg = vecLocalSegPnts.begin()+indSeg;

		//calculate line confidence for one roof face
		SearchLinesOnPlane(localLineTops, roofNum, roofLineInds);
		lineCoef = 0.0;
		lineLengSum = 0.0;
		roofLines.clear();
		vecPolyLines.clear();
		for (int iLine=0; iLine<roofLineInds.size(); ++iLine) {
			roofLine = localLineTops.begin() + roofLineInds[iLine];
			roofLines.push_back(*roofLine);
			lineLeng = localObjPnts[(*roofLine)[0].Number()].Distance(
				localObjPnts[(*roofLine)[1].Number()]);
			lineCoef += lineLeng*roofLine->Attribute(3)/7.0;
			lineLengSum += lineLeng;
			itrPnt1 = localObjPnts.begin()+(*roofLine)[0].Number();
			itrPnt2 = localObjPnts.begin()+(*roofLine)[1].Number();

			if (itrPnt1->Distance(*itrPnt2)==0.0)
				continue;

			vecPolyLines.push_back(Line3D(itrPnt1->Position3DRef(), itrPnt2->Position3DRef()));
			//vecPolyLines.push_back(Line3D(localObjPnts[(*roofLine)[0].Number()],
			//	localObjPnts[(*roofLine)[1].Number()]));
		}
		lineCoef /= lineLengSum;

		//calculate face confidence for one roof face
		pntCoef = 0.0;
		ClosePolygon(roofLines, localObjPnts, roofPolygons);
		vecDist.clear();
		for (int iPnt=0; iPnt<curSeg->size(); ++iPnt) {
			curPnt = gLaserPnts.begin()+(*curSeg)[iPnt].Number();
			pntDist = 2.0;
			for(int iPolygon=0; iPolygon<roofPolygons.size(); ++iPolygon){
				if (curPnt->InsidePolygon(localObjPnts, roofPolygons[iPolygon]))
					pntDist = fabs(vecLocalPlanes[indPlane].Distance(*curPnt));
			}

			/////////////
			//use orthogonal distance to polygon
			if (pntDist == 2.0)	{
				//pntDist =  Distance2Polygons(roofPolygons, localObjPnts, *curPnt);
				pntDist =  Distance2Polygons(vecPolyLines, *curPnt);
			}

			vecDist.push_back(pntDist);
			curPnt->Residual() = pntDist;

			//vecDist.push_back(pntDist);
			pntDist = exp(-pntDist*pntDist/(sigma*sigma));
			//vecCoef.push_back(pntDist);
			//curPnt->SetAttribute(LabelTag, 10-int(10*pntDist));//isInside
			pntCoef += pntDist;
		}


		pntCoef = pntCoef/(0.85*curSeg->size());

		//computer STD localLy
		sigma = 0.0;
		int validPnt=0;
		for (int i=0; i<vecDist.size();i++)	{
			if (vecDist[i]<0.3) {
				sigma += vecDist[i]*vecDist[i];
				validPnt++;
			}
		}
		sigma = sqrt(sigma/(validPnt+0.000001));

		Histgram(vecDist, 1.0, 0.0, 0.02);
		double binLen;
		vecHistOfRes = Histgram(vecDist, 50, binLen);
		vector<int>::iterator itrMax = std::max_element(vecHistOfRes.begin(), vecHistOfRes.end());
		int maxHist = *itrMax;
		//pntCoef /= curSeg->size();
		double G_Test=0.0;
		vector<float> vecGTest;
		vector<float> vecGauss;
		vector<float> vecObs;
		for (int i=0; i<vecHistOfRes.size(); i++) {
			if(vecHistOfRes[i]==0) continue;
			pntDist = (i+0.5)*binLen;
			gauss = binLen*exp(-(pntDist*pntDist)/(sigma*sigma))/(sqrt(3.1415926)*sigma);
			vecGauss.push_back(gauss);
			obsP = (0.5*vecHistOfRes[i])/vecDist.size();
			//obsP = (0.5*vecHistOfRes[i])/maxHist;
			vecObs.push_back(obsP);
			//double chisquare = (obsP-gauss)*(obsP-gauss)/gauss;
			//gauss = pow((1.0*vecHistOfRes[i])/vecDist.size()-gauss, 2)/gauss;
			vecGTest.push_back(obsP*log10(obsP/gauss));
			G_Test += 2*obsP*log10(obsP/gauss);
		}

		
		//G_Test /= vecHistOfRes.size();

		if (pntCoef<=1.0) pntCoef = 1.0;
		else pntCoef = 1.0/pntCoef;

		for (int iPnt=0; iPnt<curSeg->size(); ++iPnt) {

		}
	}
}

IEDll_API bool ConstructPCMModel(Buildings& buildings, ObjectPoints& map_points)
{
	Buildings::iterator curBld;
	LineTopologies* curMapTops;
	LineTopologies roofPolygons;
	LineTopology wallLineTop;


	double temHeight;
	double minHeight = 999999.9;
	vector<double> vecHeight;
	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) {
		curMapTops = curBld->MapDataPtr();
		if (curMapTops == NULL) continue;

		for (int i=0; i<curMapTops->size(); ++i) {
			if((*curMapTops)[i].Attribute(SegmentLabel) < 0
				&& (*curMapTops)[i].Attribute(SecondSegmentLabel) < 0)
				continue;

			temHeight = map_points[(*curMapTops)[i][0].Number()].Z();
			vecHeight.push_back(temHeight);
			//if (temHeight <minHeight) minHeight = temHeight;

			temHeight = map_points[(*curMapTops)[i][1].Number()].Z();
			vecHeight.push_back(temHeight);
			//if (temHeight<minHeight) minHeight = temHeight;
		}
	}
	std::sort(vecHeight.begin(), vecHeight.end());
	//	minHeight = vecHeight[int(vecHeight.size()*0.1)];
	if (vecHeight.empty())
		minHeight = 0.0;
	else
		minHeight = vecHeight[int(vecHeight.size()*0.05)];
		
	minHeight -= 4.5;



	int leftRoofNum, rightRoofNum;
	ObjectPoint temObjPnt;
	int objPntNum;
	std::vector<int> vecAdjFaceNums;

	wallLineTop.push_back(PointNumber(0));
	wallLineTop.push_back(PointNumber(0));
	wallLineTop.push_back(PointNumber(0));
	wallLineTop.push_back(PointNumber(0));
	wallLineTop.push_back(PointNumber(0));
	wallLineTop.Label() = WallLabel;

	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) {
		curMapTops = curBld->MapDataPtr();
		if (curMapTops == NULL) continue;

		//add roofs
		curBld->clear();
		//curBld->DeleteData(RoofLabel);
		roofPolygons.clear();
		ClosePolygon(*curMapTops, map_points, roofPolygons);
		for (int i=0; i<roofPolygons.size(); ++i) 	{
			//roofPolygons[i].SetAttribute;
			leftRoofNum = roofPolygons[i].Attribute(SegmentLabel);
			SearchAdjacentFaces(*curMapTops, leftRoofNum, vecAdjFaceNums);
			//skip polygons which are for single face and self crossing
			if (vecAdjFaceNums.size() == 0 && MyIsSelfCross(roofPolygons[i], map_points))
				continue;

			roofPolygons[i].Label() = RoofLabel;
			curBld->AddModelData(int(0), &map_points, roofPolygons[i]);
		}

		//////////////////////////////////////////////////////////////////////////
		//add walls
		if (gBldRecPar.bHasWalls) {
			for (int i=0; i<curMapTops->size(); ++i) {
				leftRoofNum = (*curMapTops)[i].Attribute(SegmentLabel);
				rightRoofNum = (*curMapTops)[i].Attribute(SecondSegmentLabel);
				if (leftRoofNum!=-1 && rightRoofNum!=-1)
					continue;

				objPntNum = map_points.size();
				temObjPnt = map_points[(*curMapTops)[i][0].Number()];
				temObjPnt.Z() = minHeight;
				temObjPnt.Number() = objPntNum;
				map_points.push_back(temObjPnt);
				temObjPnt = map_points[(*curMapTops)[i][1].Number()];
				temObjPnt.Z() = minHeight;
				temObjPnt.Number() = objPntNum+1;
				map_points.push_back(temObjPnt);

				wallLineTop[0] = (*curMapTops)[i][0];
				wallLineTop[1] = (*curMapTops)[i][1];
				wallLineTop[2].Number() = objPntNum+1;
				wallLineTop[3].Number() = objPntNum;
				wallLineTop[4] = (*curMapTops)[i][0];

				curBld->AddModelData(int(0), &map_points, wallLineTop);
			}
		}

	}

	return true;
}

IEDll_API bool RemoveMapUnusedPnts(Buildings& buildings, ObjectPoints& map_points)
{
	Buildings::iterator curBld;
	LineTopologies* curMapTops, *curMapPTops;
	LineTopologies::iterator itrTop;
	vector<int> vecValidNum = vector<int>(map_points.size(), -1);
	ObjectPoints::iterator curPntItr;
	int index;

	//check valid points
	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) 
	{
		if (curMapTops = curBld->MapDataPtr()) {
			for (itrTop=curMapTops->begin(); itrTop!=curMapTops->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = map_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					vecValidNum[index] = 1;
				}
			}
		}

		if (curMapPTops = curBld->MapPartitionDataPtr()) {
			for (itrTop=curMapPTops->begin(); itrTop!=curMapPTops->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = map_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					vecValidNum[index] = 1;
				}
			}
		}
	}

	//compute the new number
	int validPntCount = 0;
	for (int iPnt=0; iPnt<vecValidNum.size(); ++iPnt) {
		if (vecValidNum[iPnt]==-1) continue;

		vecValidNum[iPnt] = validPntCount;
		validPntCount++;
	}

	int temNum;
	//renumber line topology
	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) 
	{
		if (curMapTops = curBld->MapDataPtr()) {
			for (itrTop=curMapTops->begin(); itrTop!=curMapTops->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = map_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					(*itrTop)[iPnt].Number() = vecValidNum[index];
				}
			}
		}

		if (curMapPTops = curBld->MapPartitionDataPtr()) {
			for (itrTop=curMapPTops->begin(); itrTop!=curMapPTops->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = map_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					(*itrTop)[iPnt].Number() = vecValidNum[index];
				}
			}
		}
	}

	ObjectPoints temObjPnts;
	ObjectPoint temObjPnt;
	for (int iPnt=0; iPnt<map_points.size(); ++iPnt) {
		if (vecValidNum[iPnt] ==-1) continue;
		temObjPnt = map_points[iPnt];
		temObjPnt.Number() = vecValidNum[iPnt];
		temObjPnts.push_back(temObjPnt);
	}

	std::swap(temObjPnts, map_points);
	//renumber object points

	return true;
}

IEDll_API bool RemoveModelUnusedPnts(Buildings& buildings, ObjectPoints& model_points)
{
	Buildings::iterator curBld;
	vector<LineTopologies*>* vecCurMapTops;
	vector<LineTopologies*>::iterator itrVecTops;
	LineTopologies::iterator itrTop;
	vector<int> vecValidNum = vector<int>(model_points.size(), -1);
	ObjectPoints::iterator curPntItr;
	int index;

	//check valid points
	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) {
		vecCurMapTops = curBld->ModelTopology();
		if (vecCurMapTops == NULL) continue;

		for (itrVecTops=vecCurMapTops->begin(); itrVecTops!=vecCurMapTops->end(); ++itrVecTops) {
			if (*itrVecTops == NULL) continue;

			for (itrTop=(*itrVecTops)->begin(); itrTop!=(*itrVecTops)->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = model_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					vecValidNum[index] = 1;
				}
			}
		}
	}

	//compute the new number
	int validPntCount = 0;
	for (int iPnt=0; iPnt<vecValidNum.size(); ++iPnt) {
		if (vecValidNum[iPnt]==-1) continue;

		vecValidNum[iPnt] = validPntCount;
		validPntCount++;
	}

	int temNum;
	//renumber line topology
	for(curBld=buildings.begin(); curBld!=buildings.end(); curBld++) {
		vecCurMapTops = curBld->ModelTopology();
		if (vecCurMapTops == NULL) continue;

		for (itrVecTops=vecCurMapTops->begin(); itrVecTops!=vecCurMapTops->end(); ++itrVecTops) {
			if (*itrVecTops == NULL) continue;

			for (itrTop=(*itrVecTops)->begin(); itrTop!=(*itrVecTops)->end(); ++itrTop) {
				for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
					index = model_points.FindPoint((*itrTop)[iPnt]);
					if (index == -1) continue;
					(*itrTop)[iPnt].Number() = vecValidNum[index];
				}
			}
		}
	}

	ObjectPoints temObjPnts;
	ObjectPoint temObjPnt;
	for (int iPnt=0; iPnt<model_points.size(); ++iPnt) {
		if (vecValidNum[iPnt] ==-1) continue;
		temObjPnt = model_points[iPnt];
		temObjPnt.Number() = vecValidNum[iPnt];
		temObjPnts.push_back(temObjPnt);
	}

	std::swap(temObjPnts, model_points);

	return true;
}
