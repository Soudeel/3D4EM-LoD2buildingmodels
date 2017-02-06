#include <assert.h>
#include "ModelBldWithRoofTopGraph.h"
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
//#include "RoofLayerContouring.h"

using namespace std;

 ModelBldWithRoofTopGraph::ModelBldWithRoofTopGraph(
	 LaserPoints* inLasPnts, const LineTopologies footTops, const ObjectPoints& footPnts, bool bOnlyInnerRidge)
	 : ModelBldWithOccMap(inLasPnts, footTops, footPnts, bOnlyInnerRidge)
 {

 }

 ModelBldWithRoofTopGraph::ModelBldWithRoofTopGraph(LaserPoints* inLasPnts, bool bOnlyInnerRidge)
	 : ModelBldWithOccMap(inLasPnts, bOnlyInnerRidge)
 {

 }

bool ModelBldWithRoofTopGraph::ModelOneBld()
{
	RemoveNoiseAndWallPnts();

	vector<PointNumberList> vecLayers = DeriveRoofLayers(m_curLasPnts, ComponentNumberTag);

#ifdef _DEBUG
	m_curLasPnts.Write("debug.laser");
#endif

	SortRoofLayers(m_curLasPnts, vecLayers);
	m_curLasPnts.SetAttribute(IsFilteredTag, false);

	//////////////////////////////////////////////////////////////////////////
	//derive footprint contour
	PointNumberList pnlTemContour;
	LineTopology temTop;
	m_curObjPnts.clear();
	m_curContours.clear();

	if (m_bHasFootPrint)  {
		//transform footprint data from global to local 
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

	if (m_curContours.empty() || (m_curContours.size()==1 && m_curContours[0].empty()))
		return false;



	int bldNum=-1;
	if (m_curFootPrint.HasAttribute(IDNumberTag))
		bldNum = m_curFootPrint.Attribute(IDNumberTag);

	Buildings::iterator curBld = m_pcmBlds.BuildingIterator(bldNum);
	if (curBld==m_pcmBlds.end()) {
		if (bldNum==-1) bldNum = m_pcmBlds.NextNumber();
		m_pcmBlds.push_back(Building(bldNum, &m_modelObjPnts));
		curBld = m_pcmBlds.begin()+m_pcmBlds.size()-1;
	}

	SnapAllContoursToFoot();
	ComputeContourHeight(vecLayers);
	ModelRoofLayers(*curBld, vecLayers);

	return true;
}

 
bool  ModelBldWithRoofTopGraph::ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers)
{
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

		for (int j=0; j<vecLayers[iL].size(); j++) 
			localLasPnts.push_back(m_curLasPnts[vecLayers[iL][j].Number()]);
		
		if (localLasPnts.empty()) continue;

		localPnl.clear();
		for(int i=0; i<localLasPnts.size(); i++)
			localPnl.push_back(PointNumber(i));
		
		MyDeriveContour(localLasPnts, localPnl, localContour, 4.0, &groundPlane);
		MyDeriveSegPNL(localLasPnts, localPnlSegs, SegmentNumberTag);
		for (int jSeg=0; jSeg<localPnlSegs.size(); jSeg++) {
			temPlane = localLasPnts.FitPlane(localPnlSegs[jSeg]);
			temPlane.Number() = localLasPnts[localPnlSegs[jSeg][0].Number()].Attribute(SegmentNumberTag);
			localPlanes.push_back(temPlane);
		}


		int indCont = m_curContours.FindLineByTagValue(BuildingPartNumberTag, localLasPnts[0].Attribute(ComponentNumberTag));
		if (indCont==-1) continue;

		TINEdges gEdges;
		localLasPnts.DeriveTIN();
		gEdges.Derive(localLasPnts.TINReference());
		localLasPnts.SetAttribute(IsProcessedTag, -1); 

		ReconstructLocalSegments(localLasPnts, gEdges, localPnlSegs, localPlanes,
			localPnlSegs[0], outObjPnts, outLineTops);
		MergeLocalSegments(localLasPnts,localPnlSegs, localPlanes, outObjPnts, outLineTops);
		RefineCornerOf3Faces(localLasPnts, localPlanes, localPnlSegs, outObjPnts, outLineTops);
		RefineCornerOf4Faces(localPlanes, outObjPnts, outLineTops);
		RefineCornerOfXFaces(localLasPnts, localPlanes, localPnlSegs, outObjPnts, outLineTops);

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

bool ModelBldWithRoofTopGraph::EncloseRoofPolygons(ObjectPoints& roofObjPnts, LineTopologies& roofLineTops)
{
	LineTopologies roofPolygons;
	ClosePolygon(roofLineTops, roofObjPnts, roofPolygons);
	int leftRoofNum;
	vector<int> vecAdjFaceNums;
	LineTopologies outRoofPols;

	for (int i=0; i<roofPolygons.size(); ++i) {
		//roofPolygons[i].SetAttribute;
		leftRoofNum = roofPolygons[i].Attribute(SegmentLabel);
		SearchAdjacentFaces(roofLineTops, leftRoofNum, vecAdjFaceNums);
		//skip polygons which are for single face and self crossing
		if (vecAdjFaceNums.size() == 0 && MyIsSelfCross(roofPolygons[i], roofObjPnts))
			continue;

		roofPolygons[i].Label() = RoofLabel;
		outRoofPols.push_back(roofPolygons[i]);
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


	LineTopology wallLineTop;
	for (int i=0; i<5; i++)
		wallLineTop.push_back(PointNumber(0));
	wallLineTop.Label() = WallLabel;

	
	int rightRoofNum, objPntNum;
	ObjectPoint temObjPnt;
	if (gBldRecPar.bHasWalls) {
		for (int i=0; i<roofLineTops.size(); ++i) {
			leftRoofNum = roofLineTops[i].Attribute(SegmentLabel);
			rightRoofNum = roofLineTops[i].Attribute(SecondSegmentLabel);
			if (leftRoofNum!=-1 && rightRoofNum!=-1)
				continue;

			objPntNum = roofObjPnts.size();
			temObjPnt = roofObjPnts[roofLineTops[i][0].Number()];
			temObjPnt.Z() = groudHei;
			temObjPnt.Number() = objPntNum;
			roofObjPnts.push_back(temObjPnt);
			temObjPnt = roofObjPnts[roofLineTops[i][1].Number()];
			temObjPnt.Z() = groudHei;
			temObjPnt.Number() = objPntNum+1;
			roofObjPnts.push_back(temObjPnt);

			wallLineTop[0] = roofLineTops[i][0];
			wallLineTop[1] = roofLineTops[i][1];
			wallLineTop[2].Number() = objPntNum+1;
			wallLineTop[3].Number() = objPntNum;
			wallLineTop[4] = roofLineTops[i][0];

			outRoofPols.push_back(wallLineTop);
		}
	}

	std::swap(roofLineTops, outRoofPols);

	return true;
}
