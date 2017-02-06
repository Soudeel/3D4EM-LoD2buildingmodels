#include <assert.h>
#include "ModelBldWithOccMap.h"
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
#include <time.h>
#include "PolylineGeneralization.h"

#include <limits>
#include <set>
using namespace std;

ModelBldWithOccMap:: ModelBldWithOccMap(
	LaserPoints* inLasPnts, const LineTopologies footTops, 
	const ObjectPoints& footPnts, bool bOnlyInnerRidge,
	const OutliningParameters& outLinePara)
	:m_pPntCloud(inLasPnts), m_footTops(footTops), 
	m_footPts(footPnts), m_bOnlyInnerRidge(bOnlyInnerRidge),
	m_outLinePara(outLinePara)
{
	m_bHasFootPrint = true;
	m_gGrndHight = DBL_MIN;
}

ModelBldWithOccMap::ModelBldWithOccMap(LaserPoints* inLasPnts, bool bOnlyInnerRidge, const OutliningParameters& outLinePara)
	:m_pPntCloud(inLasPnts), m_bOnlyInnerRidge(bOnlyInnerRidge), m_outLinePara(outLinePara)
{
	m_bHasFootPrint = false;
	m_gGrndHight = DBL_MIN;
}

bool ModelBldWithOccMap::DoModel()
{
	//only check 3D
	assert(m_pPntCloud);
	BldRecPar bldRecPar = GetIEGlobelVari();
	bldRecPar.bStepEdges = false;
	UpdataIEGlobelVari(bldRecPar);
	m_gGrndHight = DeriveGroundHeight(false);

	//use footprint to detect one building
	time_t startTime, endTime;
	time(&startTime);
	printf("Do modeling\n");
	
	if (m_bHasFootPrint) {
		//MyMergeMapPolygons(m_footPts, m_footTops);
		RepairMapData(m_footPts, m_footTops);
		MatchData(*m_pPntCloud, m_footPts, m_footTops, PolygonIDTag);
		vector<PointNumberList> vecComponents;
		MyDeriveSegPNL(*m_pPntCloud, vecComponents, PolygonIDTag);
		
		for (int iPol=0; iPol<m_footTops.size(); iPol++) {
			printf("Modeling %4d building\r", iPol);
			m_curLasPnts.clear();
			m_curFootPrint = m_footTops[iPol];
			if(!m_curFootPrint.IsClosed()) continue;

			bool bHasID = m_curFootPrint.HasAttribute(IDNumberTag);
			int ID = m_curFootPrint.Attribute(IDNumberTag);

			//collect points in current building
			for (int iComp=0; iComp<vecComponents.size(); iComp++) {
				PointNumberList& curCompnent = vecComponents[iComp];
				if (curCompnent.empty()) continue;
				if((*m_pPntCloud)[curCompnent[0].Number()].Attribute(PolygonIDTag)
					!= m_curFootPrint.Attribute(IDNumberTag))
					continue;

				for (int i=0; i<curCompnent.size(); i++)
					m_curLasPnts.push_back((*m_pPntCloud)[curCompnent[i].Number()]);
				break;
			}
			m_curLasPnts.RemoveAttribute(ComponentNumberTag);
			
			//do model current building
			ModelOneBld();
		}
	}
	//use component to detect one building
	else {
		TINEdges edges;
		m_pPntCloud->DeriveTIN();
		edges.Derive(m_pPntCloud->TINReference());
		m_pPntCloud->RemoveLongEdges(edges, 1.0, true);//2D
		m_pPntCloud->LabelComponents(edges, ComponentNumberTag);
		vector<PointNumberList> vecComponents;
		MyDeriveSegPNL(*m_pPntCloud, vecComponents, ComponentNumberTag);

		for (int iBld=0; iBld<vecComponents.size(); iBld++)	{
			printf("Modeling %4d building\r", iBld);
			//collect points
			PointNumberList& curCompnent = vecComponents[iBld];
			if (curCompnent.size()< gBldRecPar.minSegNum) continue;
			m_curLasPnts.clear();

			for (int i=0; i<curCompnent.size(); i++) 
				m_curLasPnts.push_back((*m_pPntCloud)[curCompnent[i].Number()]);
			m_curLasPnts.RemoveAttribute(ComponentNumberTag);



			ModelOneBld();
		}
	}

	time(&endTime);
	printf("Time used for modeling %.2f second\n", difftime(endTime, startTime));

	m_pPntCloud->RemoveAttribute(ComponentNumberTag);
	return true;
}

double ModelBldWithOccMap::DeriveGroundHeight(bool bFPHeight)
{
	double groundHei = 0.0;
	vector<double> vecHeis;

	//derive the lowest point
	if (!bFPHeight && m_pPntCloud && !m_pPntCloud->empty()) {
		for (LaserPoints::iterator pnt=m_pPntCloud->begin(); pnt!=m_pPntCloud->end(); pnt++)
			vecHeis.push_back(pnt->Z());

		std::sort(vecHeis.begin(), vecHeis.end());
		groundHei = vecHeis[int(vecHeis.size()*0.01)]-0.3;
	} else if(!m_footTops.empty()){
		//the height of current foot print has already be changed
		groundHei = std::numeric_limits<double>::max();
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
			if (temHei<groundHei) groundHei = temHei;
		}
	}

	return groundHei;
}


bool ModelBldWithOccMap::WritePcmModel(char* pcmModelPath)
{
	string path, strTopPath, strObjPath;
	path = pcmModelPath;
	path = path.substr(0, path.rfind('.'));
	strTopPath = path + ".top";
	strObjPath = path + ".objpts";

	m_pcmBlds.WriteMapData(strTopPath.c_str());
	m_footPts.Write(strObjPath.c_str());

	return true;
}

bool ModelBldWithOccMap::GetPcmModel(Buildings& outPcmBlds, ObjectPoints& outPcmObjPnts)
{


	LineTopologies localModTops;

	m_pcmBlds.CollectModelData(localModTops);
	localModTops.RemoveCollinearNodes(m_modelObjPnts);
	m_modelObjPnts.RemoveDoublePoints(localModTops, 0.00001);
	m_modelObjPnts.RemoveUnusedPnts(localModTops);

	int numPntStart = outPcmObjPnts.empty()? 0:outPcmObjPnts[outPcmObjPnts.size()-1].Number()+1;

	int numTopStart = 0;
	localModTops.ReNumber(m_modelObjPnts, numPntStart, numTopStart);
	outPcmObjPnts.insert(outPcmObjPnts.end(), m_modelObjPnts.begin(), m_modelObjPnts.end());


	std::multimap<int, LineTopologies::iterator> mapID2Tops;
	std::multimap<int, LineTopologies::iterator>::iterator itrMap0, itrMap1, itrMapUp, itrMapLow;
	LineTopologies::iterator itrTop;
	int ID;
	for (itrTop=localModTops.begin(); itrTop!=localModTops.end(); itrTop++) {
		if (itrTop->HasAttribute(IDNumberTag))
			ID = itrTop->Attribute(IDNumberTag);
		else
			ID = 0;

		mapID2Tops.insert(std::pair<int, LineTopologies::iterator>(ID, itrTop));
	}


	Buildings::iterator bld;
	std::set<int> setID;
	for (itrMap0=mapID2Tops.begin(); itrMap0!=mapID2Tops.end(); itrMap0++) {
		ID = itrMap0->first;
		if (setID.count(ID)) continue;//already processed
		setID.insert(ID);


		bld = outPcmBlds.BuildingIterator(ID);
		if (bld!=outPcmBlds.end()) {
			bld->DeleteData(ModelData);
		}
		else {
			outPcmBlds.push_back(Building(ID));
			bld = outPcmBlds.begin()+outPcmBlds.size()-1;
		}
		

		itrMapUp = mapID2Tops.upper_bound(ID);
		itrMapLow = mapID2Tops.lower_bound(ID);
		for (itrMap1=itrMapLow; itrMap1!=itrMapUp; itrMap1++) {
			bld->AddModelData(0, &outPcmObjPnts, *itrMap1->second);
		}
	}

	return true;
}

#include "IndexPointInPolygon.h"
#include "time.h"

bool ModelBldWithOccMap::MatchData(LaserPoints& lasPnts, ObjectPoints& mapObjPts, LineTopologies& mapTops, LaserPointTag tag)
{
	printf("Matching points with maps\n");

	time_t start;
	time( &start );

	IndexPointInPolygon indexer(&lasPnts, & mapTops, &mapObjPts, PolygonIDTag);
	indexer.Index();

	time_t end;
	time(&end);
	printf("  Time used for match data: %.2f second\n",difftime(end, start));



	return true;
}



vector<PointNumberList>  ModelBldWithOccMap::DeriveRoofLayers(LaserPoints& inPnts, const LaserPointTag& tag)
{
	vector<PointNumberList> vecLayers;
	vector<PointNumberList> gVecSegPnls;
	MyDeriveSegPNL(inPnts, gVecSegPnls, SegmentNumberTag);
	if(gVecSegPnls.empty())	return vecLayers;

	TINEdges gEdges;
//	vector<Plane> ;
	inPnts.DeriveTIN();
	gEdges.Derive(inPnts.TINReference());
	assert(gEdges.size()==inPnts.size()
		&& "point number does not match with TIN edge number");
	inPnts.SetAttribute(IsProcessedTag, -1); //-1 unprocessed 1 processed

	//Attribute 0: left hand face Number; Attribute 1: right hand face Number; Attribute 2: is/not refined tag
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
	PointNumberList temLayer;
	int layerNum=0;

	for (int iSeg=0; iSeg<gVecSegPnls.size(); ++iSeg) {
		if (gVecSegPnls[iSeg].size() < gBldRecPar.minSegNum
			|| inPnts[gVecSegPnls[iSeg][0].Number()].Attribute(IsProcessedTag) != -1)
			continue;//this segment has no laser data or is processed

		gVecLocalSegPnts.clear();
		vecLocalPlanes.clear();
		localObjPnts.clear();
		localLineTops.clear();

		//gVecLocalSegPnts.push_back();
		curSegNum = inPnts[gVecSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
		ReconstructLocalSegments(inPnts, gEdges, gVecSegPnls, vecLocalPlanes,
			 gVecSegPnls[iSeg], localObjPnts, localLineTops);
		
		
		MergeLocalSegments(inPnts, gVecLocalSegPnts, vecLocalPlanes, localObjPnts, localLineTops);

		std::vector<std::set<int> > vecGroupFaces = CollectConnectedRoofFacesByLines(localLineTops);
		//vecGroupFaces = CollectConnectedRoofFacesByLines(localLineTops);
		if (vecGroupFaces.empty()) {
			curSegNum = inPnts[gVecSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
			std::set<int> temGroupFaces;
			temGroupFaces.insert(curSegNum);
			vecGroupFaces.push_back(temGroupFaces);
			int indLocSeg = IndexSegBySegNumber(inPnts, gVecLocalSegPnts, curSegNum);
			if (indLocSeg!=-1)
				inPnts.Label(gVecLocalSegPnts[indLocSeg], 1, IsProcessedTag);
		}
		//current segment is ignored when merge local segments
		else if (inPnts[gVecSegPnls[iSeg][0].Number()].Attribute(IsProcessedTag)==-1)
			iSeg--;
		

		for (int j=0; j<vecGroupFaces.size(); j++) {
			temLayer.clear();

			for (std::set<int>::iterator itrSet=vecGroupFaces[j].begin(); itrSet!=vecGroupFaces[j].end(); itrSet++) {
				int indLocSeg = IndexSegBySegNumber(inPnts, gVecLocalSegPnts, *itrSet);
				//int indSeg = IndexSegBySegNumber(inPnts, gVecSegPnls, *itrSet);
				if (indLocSeg == -1) continue;
				temLayer.insert(temLayer.end(), gVecLocalSegPnts[indLocSeg].begin(), gVecLocalSegPnts[indLocSeg].end());
			}

			for (int j=0; j<temLayer.size(); j++)
				inPnts[temLayer[j].Number()].SetAttribute(tag, layerNum);
			layerNum++;
			vecLayers.push_back(temLayer);
		}
	}

	inPnts.RemoveAttribute(IsProcessedTag);
	return vecLayers;
}

bool ModelBldWithOccMap::ModelOneBld()
{
	RemoveNoiseAndWallPnts();
	vector<PointNumberList> vecLayers = DeriveRoofLayers(m_curLasPnts, ComponentNumberTag);


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
		//m_curFootPrint.SetAttribute(IDNumberTag, m_pcmBlds.NextNumber())
	}
	m_curFootPrint.Attribute(BuildingPartNumberTag) = FOOT_PRINT_NUM;

	Plane groundPlane;
	groundPlane.Normal() = Vector3D(0.0, 0.0, 1.0);
	groundPlane.Distance() = 0.0;
	for (int iL=0; iL<vecLayers.size(); iL++) {
		RemoveOverlapedPnts(m_curLasPnts, vecLayers[iL]);
		if(vecLayers[iL].size()<gBldRecPar.minSegNum) continue;
		MyDeriveContour(m_curLasPnts, vecLayers[iL], pnlTemContour, 4.0, &groundPlane);
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

	int bldNum=-1;
	if (m_curFootPrint.HasAttribute(IDNumberTag))
		bldNum = m_curFootPrint.Attribute(IDNumberTag);

	Buildings::iterator curBld = m_pcmBlds.BuildingIterator(bldNum);
	if (curBld==m_pcmBlds.end()) {
		if (bldNum==-1) bldNum = m_pcmBlds.NextNumber();
		m_pcmBlds.push_back(Building(bldNum, &m_modelObjPnts));
		curBld = m_pcmBlds.begin()+m_pcmBlds.size()-1;
	}

	double groundHei = DeriveGroundHeight(true);
	CreateBottomFloor(*curBld, groundHei);

	if (m_curContours.empty() || (m_curContours.size()==1 && m_curContours[0].empty())) {
		CreateDummyBlockBuilding(*curBld);
	} else {
		SnapAllContoursToFoot();
		ComputeContourHeight(vecLayers);
		ModelRoofLayers(*curBld, vecLayers);
		CreateWalls(*curBld);
	}

	

	return true;
}

bool ModelBldWithOccMap::ModelRoofLayers(Building& curBld, vector<PointNumberList>& vecLayers)
{
	return false;
}

bool ModelBldWithOccMap::CreateDummyBlockBuilding(Building& curBld)
{
	double groudHei = DeriveGroundHeight(true);

	//////////////////////////////////////////////////////////////////////////
	//top part
	LineTopologies temFootPrintTops;
	temFootPrintTops.push_back(m_curFootPrint);
	ObjectPoints temFootPrintPnts = m_curObjPnts;
	//top height
	for (int i=0; i<temFootPrintPnts.size(); i++)
		temFootPrintPnts[i].Z() = groudHei+3.0;

	temFootPrintPnts.RemoveUnusedPnts(temFootPrintTops);
	temFootPrintTops.ReNumber(temFootPrintPnts, m_modelObjPnts.size(), 0);
	m_modelObjPnts.insert(m_modelObjPnts.end(), temFootPrintPnts.begin(), temFootPrintPnts.end());
	temFootPrintTops[0].Label() = RoofLabel;
	curBld.AddModelData(0, &m_modelObjPnts, temFootPrintTops[0]);

	//////////////////////////////////////////////////////////////////////////
	//wall
	LineTopology wallTop=this->m_curFootPrint;//copy attributes
	wallTop.clear();
	ObjectPoint wallPnt;
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.Label() = WallLabel;
	int objPntNum;

	for (int iPnt=0; iPnt<temFootPrintTops[0].size()-1; iPnt++) {
		objPntNum = m_modelObjPnts.size();

		wallPnt = m_modelObjPnts[temFootPrintTops[0][iPnt].Number()];
		wallPnt.Z() = groudHei;
		wallPnt.Number() = objPntNum;
		m_modelObjPnts.push_back(wallPnt);

		wallPnt = m_modelObjPnts[temFootPrintTops[0][iPnt+1].Number()];
		wallPnt.Z() = groudHei;
		wallPnt.Number() = objPntNum+1;
		m_modelObjPnts.push_back(wallPnt);

		wallTop[0] = temFootPrintTops[0][iPnt];
		wallTop[1] = temFootPrintTops[0][iPnt+1];
		wallTop[2].Number() = objPntNum+1;
		wallTop[3].Number() = objPntNum;
		wallTop[4] = wallTop[0];

		curBld.AddModelData(0, &m_modelObjPnts, wallTop);
	}
	
	return true;
}


bool ModelBldWithOccMap::CreateBottomFloor(Building& curBld, double height)
{
	double grndHei = height;

	LineTopologies temFootPrintTops;
	temFootPrintTops.push_back(m_curFootPrint);
	ObjectPoints temFootPrintPnts = m_curObjPnts;
	//bottom height
	for (int i=0; i<temFootPrintPnts.size(); i++)
		temFootPrintPnts[i].Z() = grndHei;

	temFootPrintPnts.RemoveUnusedPnts(temFootPrintTops);
	temFootPrintTops.ReNumber(temFootPrintPnts, m_modelObjPnts.size(), 0);
	m_modelObjPnts.insert(m_modelObjPnts.end(), temFootPrintPnts.begin(), temFootPrintPnts.end());
	temFootPrintTops[0].Label() = WallLabel;
	curBld.AddModelData(0, &m_modelObjPnts, temFootPrintTops[0]);

	return true;
}

bool ModelBldWithOccMap::CreateWalls(Building& curBld)
{
	double groudHei;// = m_gGrndHeight;
	BldRecPar bldRecPar = GetIEGlobelVari();
	if (bldRecPar.bFootPrintHei)
		groudHei = m_gGrndHight;
	else {//use footprint hight
		groudHei = DBL_MAX;
		double temHei;
		for (int i = 0; i < m_curFootPrint.size()-1; i++) {
			temHei = m_curObjPnts[m_curFootPrint[i].Number()].Z();
			if (temHei<groudHei) groudHei = temHei;
		}
	}
	CreateBottomFloor(curBld, groudHei);

	LineTopologies contours = m_curContours;
	ObjectPoints contObjPnts = m_curObjPnts;
	contours.ReNumber(contObjPnts, m_modelObjPnts.size(), 0);
	m_modelObjPnts.insert(m_modelObjPnts.end(), contObjPnts.begin(), contObjPnts.end());
	
	LineTopology wallTop;
	ObjectPoint wallPnt;
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.push_back(PointNumber(0));
	wallTop.Label() = WallLabel;
	int objPntNum;

	for (int iCont = 0; iCont<contours.size(); iCont++) {
		for (int iPnt=0; iPnt<contours[iCont].size()-1; iPnt++) {
			objPntNum = m_modelObjPnts.size();
			wallPnt = m_modelObjPnts[contours[iCont][iPnt].Number()];
			wallPnt.Z() = groudHei;
			wallPnt.Number() = objPntNum;
			m_modelObjPnts.push_back(wallPnt);

			wallPnt = m_modelObjPnts[contours[iCont][iPnt+1].Number()];
			wallPnt.Z() = groudHei;
			wallPnt.Number() = objPntNum+1;
			m_modelObjPnts.push_back(wallPnt);

			wallTop[0] = contours[iCont][iPnt];
			wallTop[1] = contours[iCont][iPnt+1];
			wallTop[2].Number() = objPntNum+1;
			wallTop[3].Number() = objPntNum;
			wallTop[4] = wallTop[0];

			curBld.AddModelData(0, &m_modelObjPnts, wallTop);
		}
	}

	return true;
}

bool ModelBldWithOccMap::DeriveWholeContour(const LaserPoints& lasPnts, ObjectPoints& localObjPnts, LineTopology& localTop)
{
	localTop.clear();
	PointNumberList pnlAll, contour;

	for (int i=0; i<lasPnts.size(); i++)
		pnlAll.push_back(PointNumber(i));
	
	Position3D cent = lasPnts.CentreOfGravity(pnlAll);
	Plane plane(cent, Vector3D(0.0, 0.0, 1.0));
	DeriveContour_AlphaShape2D(lasPnts, pnlAll, contour, gBldRecPar.alphaDia, gBldRecPar.bAdaptiveAlpha);
	PolylineGeneralizeByLineGrow(lasPnts, contour, localObjPnts, localTop);


	IntersectWithRoofLayer(m_curLasPnts, pnlAll, localObjPnts, localTop);

	return true;
}

bool ModelBldWithOccMap::RemoveNoiseAndWallPnts()
{
	//find noisy and wall points points
	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(m_curLasPnts, vecSegPnls, SegmentNumberTag);
	m_curLasPnts.SetAttribute(IsSelectedTag, false);
	Plane plane;
	vector<PointNumberList>::iterator itrSeg;

	//tag valid point
	for (itrSeg=vecSegPnls.begin(); itrSeg!=vecSegPnls.end(); itrSeg++)	{
		if (itrSeg->size() < gBldRecPar.minSegNum) continue;
		plane = m_curLasPnts.FitPlane(*itrSeg);
		if (plane.IsVertical(15*PI/180.0)) continue;

		for (int i=0; i<itrSeg->size(); ++i)
			m_curLasPnts[(*itrSeg)[i].Number()].SetAttribute(IsSelectedTag, true);
	}

	if (m_curLasPnts.empty()) return false;

	LaserPoints::iterator point1, point2;
	int invalidPntCount = 0;
	bool bFind;
	for (int i=0; i<m_curLasPnts.size()-invalidPntCount; ++i) {
		point1 = m_curLasPnts.begin()+i;
		if (point1->Attribute(IsSelectedTag) == true) continue;

		//find a valid point to swap with the current invalid point
		bFind = false;
		for (int j=m_curLasPnts.size()-1-invalidPntCount; j>i; j--) {
			point2 = m_curLasPnts.begin()+j;
			++invalidPntCount;

			if (point2->Attribute(IsSelectedTag) == true) {
				std::swap(*point1, *point2);
				bFind = true;
				break;
			}
		}

		//it is the last invalid point
		if (bFind==false)
			++invalidPntCount;
	}

	m_curLasPnts.erase(m_curLasPnts.end()-invalidPntCount, m_curLasPnts.end());

	m_curLasPnts.RemoveAttribute(IsSelectedTag);
	return true;
}

bool ModelBldWithOccMap::IsHigher(const PointNumberList& L1, const PointNumberList& L2)
{
	Position3D pnt1 = m_curLasPnts.CentreOfGravity(L1);
	Position3D pnt2 = m_curLasPnts.CentreOfGravity(L2);
	return pnt1.Z()>pnt2.Z();
}


//biao 2017/01/19, v1 no simpligy
bool ModelBldWithOccMap::SimplifyContour(const LaserPoints& lasPnts, 
	const PointNumberList& contour,	ObjectPoints& localObjPnts, LineTopology& localTop)
{
	localTop.clear();
	if (contour.empty() || contour[0].Number()!=contour[contour.size()-1].Number())
		return false;

	ObjectPoint temPnt;
	for (int i=0; i<contour.size()-1; i++) {
		temPnt.Position3DRef() = lasPnts[contour[i].Number()].Position3DRef();
		temPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(temPnt);
		localTop.push_back(temPnt.NumberRef());
	}
	localTop.push_back(localTop[0]);
	return true;
	
	
	LineTopology temTop;
	LineTopologies temTops;
	ObjectPoints temObjPnts;
	PolylineGeneralizeByLineGrow(lasPnts, contour, temObjPnts, temTop, m_outLinePara);
	temTops.push_back(temTop);
	temTops.ReNumber(temObjPnts, localObjPnts.size(), 0);
	localObjPnts.insert(localObjPnts.end(), temObjPnts.begin(), temObjPnts.end());
	localTop = temTops[0];

#ifdef _DEBUG
	if (lasPnts[contour[0].Number()].HasAttribute(SegmentNumberTag) && 
		lasPnts[contour[0].Number()].Attribute(SegmentNumberTag)==278)
	{
		LineTopologies oneTops;
		oneTops.push_back(localTop);
		localObjPnts.Write("debug.objpts");
		oneTops.Write("debug.top");
	}
#endif

	return true;
}

/*
//v2 simple simplify
bool ModelBldWithOccMap::SimplifyContour(const LaserPoints& lasPnts, 
	const PointNumberList& contour,	ObjectPoints& localObjPnts, LineTopology& localTop)
{
	localTop.clear();
	if (contour.empty() || contour[0].Number()!=contour[contour.size()-1].Number())
		return false;

	ObjectPoint temPnt;
	for (int i=0; i<contour.size()-1; i++) {
		temPnt.Position3DRef() = lasPnts[contour[i].Number()].Position3DRef();
		temPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(temPnt);
		localTop.push_back(temPnt.NumberRef());
	}
	localTop.push_back(localTop[0]);
	
	Plane groundPlane;
	groundPlane.Distance() = 0.0;
	groundPlane.Normal() = Vector3D(0.0, 0.0, 1.0);
	MySimplifyContour(lasPnts, contour, groundPlane, localObjPnts, localTop);
	


#ifdef _DEBUG
	if (lasPnts[contour[0].Number()].HasAttribute(SegmentNumberTag) && 
		lasPnts[contour[0].Number()].Attribute(SegmentNumberTag)==278)
	{
		LineTopologies oneTops;
		oneTops.push_back(localTop);
		localObjPnts.Write("debug.objpts");
		oneTops.Write("debug.top");
	}
#endif

	return true;
}

*/

/*
//v3 simplify by line grow
bool ModelBldWithOccMap::SimplifyContour(const LaserPoints& lasPnts, 
	const PointNumberList& contour,	ObjectPoints& localObjPnts, LineTopology& localTop)
{
	localTop.clear();
	if (contour.empty() || contour[0].Number()!=contour[contour.size()-1].Number())
		return false;

	//ObjectPoint temPnt;
	//for (int i=0; i<contour.size()-1; i++) {
	//	temPnt.Position3DRef() = lasPnts[contour[i].Number()].Position3DRef();
	//	temPnt.Number() = localObjPnts.size();
	//	localObjPnts.push_back(temPnt);
	//	localTop.push_back(temPnt.NumberRef());
	//}
	//localTop.push_back(localTop[0]);
	
	//Plane groundPlane;
	//groundPlane.Distance() = 0.0;
	//groundPlane.Normal() = Vector3D(0.0, 0.0, 1.0);
	//MySimplifyContour(lasPnts, contour, groundPlane, localObjPnts, localTop);
	
	LineTopology temTop;
	LineTopologies temTops;
	ObjectPoints temObjPnts;
	PolylineGeneralizeByLineGrow(lasPnts, contour, temObjPnts, temTop, m_outLinePara);
	temTops.push_back(temTop);
	temTops.ReNumber(temObjPnts, localObjPnts.size(), 0);
	localObjPnts.insert(localObjPnts.end(), temObjPnts.begin(), temObjPnts.end());
	localTop = temTops[0];

#ifdef _DEBUG
	if (lasPnts[contour[0].Number()].HasAttribute(SegmentNumberTag) && 
		lasPnts[contour[0].Number()].Attribute(SegmentNumberTag)==278)
	{
		LineTopologies oneTops;
		oneTops.push_back(localTop);
		localObjPnts.Write("debug.objpts");
		oneTops.Write("debug.top");
	}
#endif

	return true;
}*/


#include "KNNFinder.h"
bool ModelBldWithOccMap::IntersectWithRoofLayer(const LaserPoints& lasPnts, const PointNumberList& layer,
	ObjectPoints& localObjPnts, LineTopology& localTop)
{
	LaserPoints lLasPnts;
	lLasPnts.reserve(layer.size());
	for (int i=0; i<layer.size(); i++)
		lLasPnts.push_back(lasPnts[layer[i].Number()]);
	KNNFinder<LaserPoint> knn(lLasPnts, 2);

	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(lLasPnts, vecSegPnls, SegmentNumberTag);
	vector<Plane> vecLPlanes;
	Plane plane;
	for (int i=0; i<vecSegPnls.size(); i++) {
		plane = lLasPnts.FitPlane(vecSegPnls[i]);
		plane.Number() = lLasPnts[vecSegPnls[i][0].Number()].Attribute(SegmentNumberTag);
		vecLPlanes.push_back(plane);
	}

	ObjectPoints::iterator itrCorner;
	LaserPoint temLasPnt;
	vector<int> neibs;
	PointNumberList pnlNeibs;
	
	Position3D cent, cross;
	Line3D plumb;
	
	for (int i=0; i<localTop.size(); i++) {
		itrCorner = localObjPnts.begin() + localTop[i].Number();
		temLasPnt.Position3DRef() = itrCorner->Position3DRef();
		neibs = knn.FindIndices(temLasPnt, 10);
		pnlNeibs.clear();
		for (int i=0; i<neibs.size(); i++)
			pnlNeibs.push_back(PointNumber(neibs[i]));

		cent = lLasPnts.CentreOfGravity(pnlNeibs);
		//plane = lLasPnts.FitPlane(pnlNeibs);

		int domSegNum = GetDominaintSegNum(lLasPnts, pnlNeibs);
		if (domSegNum==-1) {
			itrCorner->Z() = cent.Z();
			continue;
		}
		
		int idxPlane = IndexPlaneBySegNumber(vecLPlanes, domSegNum);
		if (idxPlane==-1 ) {
			itrCorner->Z() = cent.Z();
			continue;
		}

		plane = vecLPlanes[idxPlane];
		plumb = Line3D(itrCorner->Position3DRef(), Vector3D(0.0,0.0,1.0));
		if(!MyIntersectLine3DPlane(plumb, plane, cross)) {
			itrCorner->Z() = cent.Z();
			continue;
		}
		
		itrCorner->Z() = cross.Z();

	}


	return true;
}

bool ModelBldWithOccMap::ComputeContourHeight(std::vector<PointNumberList> vecLayers)
{
	int bldPartNum;
	int indContour;
	for (int iL=0; iL<vecLayers.size(); iL++) {
		if (vecLayers[iL].empty()) continue;
		bldPartNum = m_curLasPnts[vecLayers[iL][0].Number()].Attribute(ComponentNumberTag);
		indContour = m_curContours.FindLineByTagValue(BuildingPartNumberTag, bldPartNum);
		if (indContour==-1) continue;
		LineTopology& curContour = m_curContours[indContour];
		IntersectWithRoofLayer(m_curLasPnts, vecLayers[iL], m_curObjPnts, curContour);
	}

	return true;
}

//invalid points of lower layers that are in the contours of current layer
bool ModelBldWithOccMap::InvalidOverlapedPnts(ObjectPoints& localObjPnts, LineTopology& localTop,
	std::vector<PointNumberList>& vecLayers, int curLayInd)
{
	LaserPoints::iterator pnt;

	assert(curLayInd>=0 && curLayInd<vecLayers.size());
	for(int i=curLayInd+1; i<vecLayers.size(); i++){
		for (int j=0; j<vecLayers[i].size(); j++) {
			pnt = m_curLasPnts.begin()+vecLayers[i][j].Number();
			if (pnt->InsidePolygon(localObjPnts, localTop))
				pnt->SetAttribute(IsFilteredTag, true);
		}
	}

	return true;
}

void CreateDemoData(ObjectPoints& localObjPnts, LineTopologies& localTops)
{
	localTops.clear();
	localObjPnts.clear();

	LineTopology temTop;
	ObjectPoint temPnt;
	temPnt.Z() = 0.0;
	int pntNum = 0;

	//polygon 0
	temPnt.X() = -3; temPnt.Y() = 1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = 3; temPnt.Y() = 1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = 3; temPnt.Y() = -1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = -3; temPnt.Y() = -1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temTop.push_back(temTop[0]);
	temTop.Attribute(BuildingPartNumberTag) = 0;
	localTops.push_back(temTop);


	temTop.clear();
	temPnt.X()=-1; temPnt.Y()=1; temPnt.Number()=pntNum++;	localObjPnts.push_back(temPnt);	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X()=0; temPnt.Y()=1; temPnt.Number()=pntNum++; localObjPnts.push_back(temPnt); temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X()=1; temPnt.Y()=1; temPnt.Number()=pntNum++; localObjPnts.push_back(temPnt); temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X()=1; temPnt.Y()=0; temPnt.Number()=pntNum++; localObjPnts.push_back(temPnt); temTop.push_back(PointNumber(temPnt.Number()));
	//temPnt.X()=0; temPnt.Y()=0; temPnt.Number()=pntNum++; localObjPnts.push_back(temPnt); temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X()=-1; temPnt.Y()=0; temPnt.Number()=pntNum++; localObjPnts.push_back(temPnt); temTop.push_back(PointNumber(temPnt.Number()));
	temTop.push_back(temTop[0]);
	temTop.Attribute(BuildingPartNumberTag) = 1;
	localTops.push_back(temTop);

	//polygon 2
	temTop.clear();
	temPnt.X() = -3; temPnt.Y() = -1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = 1; temPnt.Y() = -1; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = 1; temPnt.Y() = -3; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temPnt.X() = -3; temPnt.Y() = -3; temPnt.Number() = pntNum++;
	localObjPnts.push_back(temPnt);
	temTop.push_back(PointNumber(temPnt.Number()));
	temTop.push_back(temTop[0]);
	temTop.Attribute(BuildingPartNumberTag) = 2;
	localTops.push_back(temTop);
}

void CreateDemoData1(ObjectPoints& localObjPnts, LineTopologies& localTops)
{
	localTops.clear();
	localObjPnts.clear();

	localObjPnts.Read("problem_polygon1.objpts");
	localTops.Read("problem_polygon1.top");
}

#include "PPrepair/PlanarPartition.h"
bool ModelBldWithOccMap::SnapToOldContours(ObjectPoints& localObjPnts, LineTopology& localTop, bool bLastCompt)
{

	LineTopologies localTops;
	vector<LineSegment2D> vecLineSegs;
	for (int i=0; i<m_curFootPrint.size()-1; i++) {
		ObjectPoint& pnt1 = localObjPnts[m_curFootPrint[i].Number()];
		ObjectPoint& pnt2 = localObjPnts[m_curFootPrint[i+1].Number()];
		vecLineSegs.push_back(LineSegment2D(pnt1.Position2DOnly(), pnt2.Position2DOnly()));
	}

	Position2D projPnt;
	for (int i=0; i<localTop.size()-1; i++) {
		ObjectPoints::iterator curPnt = localObjPnts.begin()+localTop[i].Number();
		if(!m_curFootPrint.BoundPoint(localObjPnts, *curPnt)) continue;//only precess points inside the footprint

		curPnt->Z() = 1.0;

		double minDist = 9999999.9, temDist;
		int nearstLineInd;
		for (int j=0; j<vecLineSegs.size(); j++) {
			temDist = vecLineSegs[j].DistanceToPoint(curPnt->Position2DOnly());
			if (temDist<minDist) {
				minDist = temDist;
				nearstLineInd = j;
			}
		}

		if (minDist<0.3) {
			
			projPnt = vecLineSegs[nearstLineInd].Project(curPnt->Position2DOnly());
			projPnt = projPnt+0.001*(projPnt-curPnt->Position2DOnly());
			curPnt->X() = projPnt.X();
			curPnt->Y() = projPnt.Y();
		}
	}
	localTop.RemoveCollinearNodes(localObjPnts);
	

	
	localTops.push_back(localTop);
	localTops.insert(localTops.end(), m_curContours.begin(), m_curContours.end());
	localTops.push_back(m_curFootPrint);
	

	vector<int> priorList;
	for (int i=0; i<localTops.size(); i++)
		priorList.push_back(localTops[i].Attribute(BuildingPartNumberTag));
		



	//do process
	libPP::PlanarPartition pp;

	pp.addToTriangulation(localObjPnts, localTops);

	pp.tagTriangulation();



	pp.RepairBySnaptoFootPrint(priorList, !bLastCompt);


	pp.reconstructPolygons();


	ObjectPoints snapObjPts;
	LineTopologies snapTops;
	pp.exportPolygons(snapObjPts, snapTops);



	LineTopology outLineTop;
	ObjectPoint temObjPnt;
	for (int i=0; i<snapTops.size(); i++) {
		if (snapTops[i].Attribute(BuildingPartNumberTag)==localTop.Attribute(BuildingPartNumberTag)) {
			snapTops[i].RemoveCollinearNodes(snapObjPts);
			for (int j=0; j<snapTops[i].size(); j++) {
				temObjPnt = snapObjPts[snapTops[i][j].Number()];
				temObjPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(temObjPnt);
				outLineTop.push_back(temObjPnt.NumberRef());
			}
			outLineTop.Attribute(BuildingPartNumberTag) = snapTops[i].Attribute(BuildingPartNumberTag);
			
			localTop = outLineTop;
			break;
		}
	}

	return true;
}

bool ModelBldWithOccMap::SnapAllContoursToFoot()
{	
	//////////////////////////////////////////////////////////////////////////
	//only one or none contour, just return the footprint 
	if (m_curContours.size()<2) {
		int partNum = 0;
		if (m_curContours.size()==1) 
			partNum = m_curContours[0].Attribute(BuildingPartNumberTag);
		m_curContours.clear();
		m_curContours.push_back(m_curFootPrint);
		m_curContours[0].Attribute(BuildingPartNumberTag) = partNum;
		return true;
	}

//#ifdef _DEBUG
	m_curContours.Write("debug.top");
	m_curObjPnts.Write("debug.objpts");
//#endif

	//////////////////////////////////////////////////////////////////////////
	//snap points to foot print parts within short distance
	vector<LineSegment2D> vecLineSegs;
	for (int i=0; i<m_curFootPrint.size()-1; i++) {
		ObjectPoint& pnt1 = m_curObjPnts[m_curFootPrint[i].Number()];
		ObjectPoint& pnt2 = m_curObjPnts[m_curFootPrint[i+1].Number()];
		vecLineSegs.push_back(LineSegment2D(pnt1.Position2DOnly(), pnt2.Position2DOnly()));
	}

	Position2D projPnt;
	for (int iCont=0; iCont<m_curContours.size(); iCont++) {
		LineTopology& localTop = m_curContours[iCont];
		if (localTop.empty()) continue;
		for (int i=0; i<localTop.size()-1; i++) {
			ObjectPoints::iterator curPnt = m_curObjPnts.begin()+localTop[i].Number();
			if(!m_curFootPrint.BoundPoint(m_curObjPnts, *curPnt)) continue;//only precess points inside the footprint
			curPnt->Z() = 1.0;

			double minDist = 9999999.9, temDist;
			int nearstLineInd;
			for (int j=0; j<vecLineSegs.size(); j++) {
				temDist = vecLineSegs[j].DistanceToPoint(curPnt->Position2DOnly());
				if (temDist<minDist) {
					minDist = temDist;
					nearstLineInd = j;
				}
			}

			if (minDist<0.3) {		
				projPnt = vecLineSegs[nearstLineInd].Project(curPnt->Position2DOnly());
				projPnt = projPnt+0.001*(projPnt-curPnt->Position2DOnly());
				curPnt->X() = projPnt.X();
				curPnt->Y() = projPnt.Y();
			}
		}
		localTop.RemoveCollinearNodes(m_curObjPnts);
	}
	

	LineTopologies localTops = m_curContours;
	localTops.push_back(m_curFootPrint);
	ObjectPoints localObjPnts = m_curObjPnts;

	vector<int> priorList;
	for (unsigned int i=0; i<localTops.size(); i++)
		priorList.push_back(localTops[i].Attribute(BuildingPartNumberTag));
	//priorList.push_back(localTops[localTops.size()-1].Attribute(BuildingPartNumberTag));

//#ifdef _DEBUG
	localTops.Write("debug.top");
	localObjPnts.Write("debug.objpts");
//#endif

	//do process
	libPP::PlanarPartition pp;
	//	pp.addToTriangulation(currentFile->first.c_str(), currentFile->second);
	pp.addToTriangulation(localObjPnts, localTops);

	pp.tagTriangulation();
//#ifdef _DEBUG
	pp.exportTriangulation("debug_triangle.dxf", true, false, false);
//#endif
	pp.RepairBySnaptoFootPrint2(priorList, false);
	//pp.RepairBySnaptoFootPrint3(priorList, false);
//#ifdef _DEBUG
	pp.exportTriangulation("debug_triangle_after.dxf", true, false, false);
//#endif
	pp.reconstructPolygons();

	ObjectPoints snapObjPts;
	LineTopologies snapTops, snapTopsUnique;
	//pp.exportPolygons(snapObjPts, snapTops);
	pp.exportPolygons(m_curObjPnts, m_curContours);


//#ifdef _DEBUG
	m_curObjPnts.Write("debug.objpts");
	m_curContours.Write("debug.top");
//#endif


	return true;
}

bool ModelBldWithOccMap::RemoveOverlapedPnts(const LaserPoints& curLasPnts, PointNumberList& pnlCurCom)
{
	LaserPoints::const_iterator itrPnt;
	for (int i=0; i<pnlCurCom.size(); i++) {
		itrPnt = curLasPnts.begin()+pnlCurCom[i].Number();
		if (itrPnt->Attribute(IsFilteredTag)) {
			pnlCurCom.erase(pnlCurCom.begin()+i);
			i--;
		}
	}

	return true;
}

bool ModelBldWithOccMap::SortRoofLayers(const LaserPoints& lasPnts, std::vector<PointNumberList>& vecLayers)
{
	vector<Position3D> vecCents;
	Position3D temCent;
	//pnt1 = m_curLasPnts.CentreOfGravity(L1);
	for (int i=0; i<vecLayers.size(); i++) 
		vecCents.push_back(lasPnts.CentreOfGravity(vecLayers[i]));
	
	for (int i=0; i<vecLayers.size(); i++)	{
		for (int j=i+1; j<vecLayers.size(); j++) {
			if (vecCents[i].Z()>vecCents[j].Z()) continue;

			std::swap(vecLayers[i], vecLayers[j]);
			std::swap(vecCents[i], vecCents[j]);
		}
	}

	//std::sort(vecLayers.begin(), vecLayers.end(), IsHigher);
	return true;
}
