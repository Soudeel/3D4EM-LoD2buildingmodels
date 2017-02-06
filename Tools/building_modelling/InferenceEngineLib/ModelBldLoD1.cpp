#include <assert.h>
#include "ModelBldLoD1.h"
#include "InferenceEngine.h"
#include "OccupyMap.h"
#include "LaserPoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "LineSegments2D.h"
#include "Position2D.h"
#include "AdjacentGraph.h"
#include "stdint.h"
#include "ObjectPoint.h"

#include "AnchorPoints.h"


using namespace std;

 ModelBldLoD1::ModelBldLoD1(
	 LaserPoints* inLasPnts, const LineTopologies footTops, const ObjectPoints& footPnts, 
	 bool bOnlyInnerRidge, const OutliningParameters& outLinePara)
	 : ModelBldWithOccMap(inLasPnts, footTops, footPnts, bOnlyInnerRidge, outLinePara)
 {

 }


 ModelBldLoD1::ModelBldLoD1(LaserPoints* inLasPnts, bool bOnlyInnerRidge, const OutliningParameters& outLinePara)
	 : ModelBldWithOccMap(inLasPnts, bOnlyInnerRidge, outLinePara)
 {

 }

 
bool ModelBldLoD1::ModelOneBld()
{
	RemoveNoiseAndWallPnts();

	vector<PointNumberList> vecLayers = DeriveRoofLayers(m_curLasPnts, ComponentNumberTag);

#ifdef _DEBUG
	m_curLasPnts.Write("debug.laser");
#endif

	SortRoofLayers(m_curLasPnts, vecLayers);
	m_curLasPnts.SetAttribute(IsFilteredTag, false);


	PointNumberList pnlTemContour;
	LineTopology temTop;
	m_curObjPnts.clear();
	m_curContours.clear();

	if (m_bHasFootPrint)  {

		ObjectPoint temObjPnt;
		temTop = m_curFootPrint;//copy attributes
		temTop.clear();
		for (int i=0; i<m_curFootPrint.size()-1; i++) {
			temObjPnt = m_footPts[m_curFootPrint[i].Number()];
			temObjPnt.Number() = i;
			m_curObjPnts.push_back(temObjPnt);
			temTop.push_back(temObjPnt.NumberRef());
		}
		temTop.push_back(temTop[0]);
		m_curFootPrint = temTop;
	}
	else {
		DeriveWholeContour(m_curLasPnts, m_curObjPnts, m_curFootPrint);
	}
	m_curFootPrint.Attribute(BuildingPartNumberTag) = FOOT_PRINT_NUM;

	Plane groundPlane;
	groundPlane.Normal() = Vector3D(0.0, 0.0, 1.0);
	groundPlane.Distance() = 0.0;

	for (int iL=0; iL<vecLayers.size(); iL++) {
		RemoveOverlapedPnts(m_curLasPnts, vecLayers[iL]);
		if(vecLayers[iL].size()<gBldRecPar.minSegNum) continue;
		//MyDeriveContour(m_curLasPnts, vecLayers[iL], pnlTemContour, 4.0, &groundPlane);
		DeriveContour_AlphaShape2D(m_curLasPnts, vecLayers[iL], pnlTemContour, gBldRecPar.alphaDia, gBldRecPar.bAdaptiveAlpha);
		SimplifyContour(m_curLasPnts, pnlTemContour, m_curObjPnts, temTop);
		if (temTop.empty()) continue;
		int componentNum = m_curLasPnts[vecLayers[iL][0].Number()].Attribute(ComponentNumberTag);
		temTop.Attribute(BuildingPartNumberTag) = componentNum;
//		SnapToOldContours(m_curObjPnts, temTop, iL==(vecLayers.size()-1));

//		IntersectWithRoofLayer(m_curLasPnts, vecLayers[iL], m_curObjPnts, temTop);
		temTop.Number() = m_curContours.size();
		m_curContours.push_back(temTop);
		InvalidOverlapedPnts(m_curObjPnts, temTop, vecLayers, iL);
	}

	for (int i=0; i<vecLayers.size(); i++)
		if (vecLayers[i].empty()) vecLayers.erase(vecLayers.begin()+i, vecLayers.begin()+i+1);


	int bldNum=-1;
	if (m_curFootPrint.HasAttribute(IDNumberTag))
		bldNum = m_curFootPrint.Attribute(IDNumberTag);

	Buildings::iterator curBld = m_pcmBlds.BuildingIterator(bldNum);
	if (curBld==m_pcmBlds.end()) {
		if (bldNum==-1) bldNum = m_pcmBlds.NextNumber();
		m_pcmBlds.push_back(Building(bldNum, &m_modelObjPnts));
		curBld = m_pcmBlds.begin()+m_pcmBlds.size()-1;
	}

	if(!m_bOnlyInnerRidge) {
		double groundHei = DeriveGroundHeight(true);
		CreateBottomFloor(*curBld, groundHei);
	}

	if (m_curContours.empty() || (m_curContours.size()==1 && m_curContours[0].empty())) {
		CreateDummyBlockBuilding(*curBld);
	} else {
		SnapAllContoursToFoot();
		ComputeContourHeight(vecLayers);
		ModelRoofLayers(*curBld, vecLayers);
	}

	return true;
}

bool  ModelBldLoD1::ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers)
{
	//get a local copy of laser points
	LaserPoints localLasPnts;
	PointNumberList localPnl;
	PointNumberList localContour;
	Plane groundPlane;
	groundPlane.Normal() = Vector3D(0.0, 0.0, 1.0);
	groundPlane.Distance() = 0.0;
	vector<Plane> localPlanes;
	vector<PointNumberList> localPnlSegs;
	Plane temPlane;
	ObjectPoints outObjPnts;
	LineTopologies outLineTops;

	for (int iL=0; iL<vecLayers.size(); iL++) {
		localLasPnts.clear();
		localPlanes.clear();
		//////////////////////////////////////////////////////////////////////////
		//some preparing work
		for (int j=0; j<vecLayers[iL].size(); j++) 
			localLasPnts.push_back(m_curLasPnts[vecLayers[iL][j].Number()]);
		
		if (localLasPnts.empty()) 
			continue;

		localPnl.clear();
		for(int i=0; i<localLasPnts.size(); i++)
			localPnl.push_back(PointNumber(i));
		
		MyDeriveContour(localLasPnts, localPnl, localContour, 4.0, &groundPlane);
		MyDeriveSegPNL(localLasPnts, localPnlSegs, SegmentNumberTag);
		for (int jSeg=0; jSeg<localPnlSegs.size(); jSeg++) {
			temPlane = localLasPnts.FitPlane(localPnlSegs[jSeg]);
			temPlane.Number() = localLasPnts[localPnlSegs[jSeg][0].Number()].Attribute(SegmentNumberTag);
			if(temPlane.Number() == 1368) {
				int aaa = 0;
			}
			localPlanes.push_back(temPlane);
		}

		//snapped contour for current layer
		int indCont = m_curContours.FindLineByTagValue(BuildingPartNumberTag, localLasPnts[0].Attribute(ComponentNumberTag));
		if (indCont==-1) continue;

		outObjPnts = m_curObjPnts;
		outLineTops.clear();
		outLineTops.push_back(m_curContours[indCont]);
		EncloseRoofPolygons(outObjPnts, outLineTops);

		outLineTops.ReNumber(outObjPnts, m_modelObjPnts.size(), 0);
		m_modelObjPnts.insert(m_modelObjPnts.end(), outObjPnts.begin(), outObjPnts.end());
		int ID = curBld.Number();
		for (int i=0; i<outLineTops.size(); i++) {
			outLineTops[i].SetAttribute(BuildingNumberTag, curBld.Number());
			outLineTops[i].SetAttribute(IDNumberTag, curBld.Number());
			curBld.AddModelData(0, &outObjPnts, outLineTops[i]);
		}
	}
	
	return true;
}

bool ModelBldLoD1::EncloseRoofPolygons(ObjectPoints& roofObjPnts, LineTopologies& roofLineTops)
{
	double aveHei = 0.0;
	int nPnts = 0;
	for (int i=0; i<roofLineTops.size(); i++) {
	roofLineTops[i].Label() = RoofLabel;

		for (int j=0; j<roofLineTops[i].size()-1; j++) {
			aveHei += roofObjPnts[roofLineTops[i][j].Number()].Z();
			nPnts++;
		}
	}
	aveHei /= nPnts;

	for (int i=0; i<roofLineTops.size(); i++) {
		for (int j=0; j<roofLineTops[i].size()-1; j++) {
			roofObjPnts[roofLineTops[i][j].Number()].Z() = aveHei;
		}
	}


	double groudHei;
	BldRecPar bldRecPar = GetIEGlobelVari();
	if (!bldRecPar.bFootPrintHei)//global height
		groudHei = m_gGrndHight;
	else {//use footprint hight
		groudHei = DBL_MAX;

		LineTopology temFootPrint = m_footTops[0];
		for (unsigned int i=0; i<m_footTops.size(); i++) {
			if (m_footTops[i].HasAttribute(IDNumberTag) &&
				m_footTops[i].Attribute(IDNumberTag)==m_curFootPrint.Attribute(IDNumberTag)) {
					temFootPrint = m_footTops[i];
					break;
			}
			else if (m_footTops[i].HasAttribute(BuildingNumberTag) &&
				m_footTops[i].Attribute(BuildingNumberTag)==m_curFootPrint.Attribute(BuildingNumberTag)) {
					temFootPrint = m_footTops[i];
					break;
			}
		}

		double temHei;
		for (int i = 0; i < temFootPrint.size()-1; i++) {
			temHei = m_footPts[temFootPrint[i].Number()].Z();
			if (temHei<groudHei) groudHei = temHei;
		}
	}


	//////////////////////////////////////////////////////////////////////////
	//add walls
	int leftRoofNum;
	LineTopologies outRoofPols = roofLineTops;
	LineTopology wallLineTop;
	for (int i=0; i<5; i++)
		wallLineTop.push_back(PointNumber(0));
	wallLineTop.Label() = WallLabel;


	int rightRoofNum, objPntNum;
	ObjectPoint temObjPnt;
	if (gBldRecPar.bHasWalls) {
		for (int i=0; i<roofLineTops[0].size()-1; ++i) {

			objPntNum = roofObjPnts.size();
			temObjPnt = roofObjPnts[roofLineTops[0][i].Number()];
			temObjPnt.Z() = groudHei;
			temObjPnt.Number() = objPntNum;
			roofObjPnts.push_back(temObjPnt);
			temObjPnt = roofObjPnts[roofLineTops[0][i+1].Number()];
			temObjPnt.Z() = groudHei;
			temObjPnt.Number() = objPntNum+1;
			roofObjPnts.push_back(temObjPnt);

			wallLineTop[0] = roofLineTops[0][i];
			wallLineTop[1] = roofLineTops[0][i+1];
			wallLineTop[2].Number() = objPntNum+1;
			wallLineTop[3].Number() = objPntNum;
			wallLineTop[4] = roofLineTops[0][i];

			outRoofPols.push_back(wallLineTop);
		}
	}

	std::swap(roofLineTops, outRoofPols);

	return true;
}
