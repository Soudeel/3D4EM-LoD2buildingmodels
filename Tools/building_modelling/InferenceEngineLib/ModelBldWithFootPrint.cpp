
#include "ModelBldWithFootPrint.h"
#include "DataBounds2D.h"
#include "InferenceEngine.h"
#include "LaserPoint.h"
//#include "QuadricsSk.h"
#include "Plane.h"
#include "tin.h"
//#include "MxBoundQSlim.h"
#include <Eigen/dense>
#include "KNNFinder.h"
#include <time.h>
#include "Buildings.h"

#include <map>

using namespace Eigen;
using namespace std;

ModelBldWithFootPrint::ModelBldWithFootPrint()
{

}

ModelBldWithFootPrint::~ModelBldWithFootPrint()
{

}

void ModelBldWithFootPrint::Initialize()
{
	m_Cloud.Read(m_CloudPath.c_str());
	m_mapPnts.Read(m_mapPntsPath.c_str());
	m_mapTops.Read(m_mapTopPath.c_str());
}

void ModelBldWithFootPrint::DoModeling()
{
	MergePolygons();
	//RepairMapData();

	MatchData();
	//MatchData2();
	Modeling();
	//	Simplify();
}

void ModelBldWithFootPrint::MergePolygons()
{

	//for (int iPoly=0; iPoly<m_mapTops.size(); iPoly++)
	//{
	//	m_mapTops[iPoly].RemoveCollinearNodes(m_mapPnts);
	//}
	m_mapTops.MergePolygons();
	m_mapTops.RemoveCollinearNodes(m_mapPnts);
	m_mapPnts.RemoveUnusedPnts(m_mapTops);
	
#ifdef _DEBUG
	m_mapPnts.Write("local.objpts");
	m_mapTops.Write("local.top");
#endif	
}

void ModelBldWithFootPrint::Modeling()
{
	LineTopologies::iterator poly;
	LaserPoints localCloud;
	vector<PointNumberList> gVecSegPnls;
	TINEdges gEdges;
	vector<Plane> vecPlanes;
	vector<int> vecAllSegNumber, vecCurSegNumber;
	gVecLocalSegPnts.reserve(100);
	vector<Plane> vecLocalPlanes;
	vecLocalPlanes.reserve(100);
	Plane temPlane;
	temPlane.Label() = -1;//-1 unfitted 1 fitted
	int nBuildingCount = 0;
	int curSegNum;
	//Attribute 0: left hand face Number; Attribute 1: right hand face Number; Attribute 2: is/not refined tag
	LineTopologies localLineTops;
	ObjectPoints localObjPnts;
	//Buildings m_pcmBlds;
	//ObjectPoints m_modelObjPnts;
	BldRecPar bldRecPar;
	bldRecPar.bStepEdges = true;
	UpdataIEGlobelVari(bldRecPar);

	for (poly=m_mapTops.begin(); poly!=m_mapTops.end(); poly++) 
	{
		printf("  Model %4d building\r", poly-m_mapTops.begin());
		GetGroupPnts(localCloud, poly->Number(), PolygonNumberTag);

		gVecSegPnls.clear();
		MyDeriveSegPNL(localCloud, gVecSegPnls, SegmentNumberTag);
		vecPlanes.clear();
		localCloud.DeriveTIN();
		gEdges.Derive(localCloud.TINReference());
		assert(gEdges.size()==localCloud.size() && "point number does not match with TIN edge number");
		localCloud.SetAttribute(IsProcessedTag, -1); //-1 unprocessed 1 processed

		//main loop for reconstruct roof segment by segment
		for (int iSeg=0; iSeg<gVecSegPnls.size(); ++iSeg)
		{//reconstruct the building which contain current laser point segment
			if (gVecSegPnls[iSeg].size() < bldRecPar.minSegNum
				|| localCloud[gVecSegPnls[iSeg][0].Number()].Attribute(IsProcessedTag) != -1)
				continue;//this segment has no laser data or is processed

			gVecLocalSegPnts.clear();	
			vecLocalPlanes.clear();
			localObjPnts.clear();
			localLineTops.clear();

			//gVecLocalSegPnts.push_back();
			//curSegNum = localCloud[gVecSegPnls[iSeg][0].Number()].Attribute(SegmentNumberTag);
			if(!ReconstructLocalSegments(localCloud, gEdges, gVecSegPnls, vecLocalPlanes,
				gVecSegPnls[iSeg], localObjPnts, localLineTops))
				continue;

			//need to be done before refine other faces
			MergeLocalSegments(localCloud, gVecSegPnls, vecLocalPlanes, localObjPnts, localLineTops);

			IntersectWallsWithRoofs2(localCloud,*poly,m_mapPnts,vecLocalPlanes,gVecSegPnls,localObjPnts,localLineTops);

			RefineSingleRoof(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			RefineSimpleDormer(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			RefineCornerOf3Faces(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			RefineCornerOf4Faces(vecLocalPlanes, localObjPnts, localLineTops);
			RefineCornerOfXFaces(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			RefineOutBoundaryLine7(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			//ComputeBoundLineCoef(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);
			//ComputeRoofFaceConf(localCloud, vecLocalPlanes, gVecLocalSegPnts, localObjPnts, localLineTops);

			/////////////////////////////////////////////////////////
			//write out
			LineTopology temLineTop;
			ObjectPoint temObjPnt;
			int objPntNum;
			Building curBld(m_pcmBlds.NextNumber(), &m_modelObjPnts);
			int att1, att2;
			for (int i=0; i < localLineTops.size(); i++) {
				temLineTop = localLineTops[i];
				att1 = temLineTop.Attribute(SegmentLabel);
				att2 = temLineTop.Attribute(SecondSegmentLabel);
				AddLine2PcmBld(curBld, m_modelObjPnts, localObjPnts[temLineTop[0].Number()],
					localObjPnts[temLineTop[1].Number()], temLineTop.Attribute(SegmentLabel), 
					temLineTop.Attribute(SecondSegmentLabel), temLineTop.Attribute(3));
			}

			m_pcmBlds.push_back(curBld);
			//printf("%dst building has been reconstructed\n", ++nBuildingCount);
		}
	}
}

bool ModelBldWithFootPrint::MatchData()
{
	printf("Matching points with maps\n");
	LaserPoints::iterator itrCloud;
	LineTopologies::iterator poly;

	//////////////////////////////////////////////////////////////////////////
	//use bond box for coarsely searching
	//m_mapTops[i]
	vector<DataBounds2D> polyBounds;
	DataBounds2D temBound;
	for (int i=0; i<m_mapTops.size(); i++) {
		temBound.Initialise();
		for (int j=0; j<m_mapTops[i].size(); ++j) {
			temBound.Update(m_mapPnts[m_mapTops[i][j].Number()].Position2DOnly());
		}
		polyBounds.push_back(temBound);
	}

	int mapTopCounts = m_mapTops.size();
	m_Cloud.RemoveAttribute(PolygonNumberTag);
	m_Cloud.RemoveAttribute(ComponentNumberTag);
	//#pragma omp parallel for reduction(+:sum) private(x)
	
	time_t start;
	time( &start );
	for (int i=0; i<m_Cloud.size();++i) {
		printf("  Match %7d point\r", i);
		for (int iLine=0; iLine<mapTopCounts; ++iLine) {
			//coarsely check
			if (m_mapTops[iLine].Attribute(HoleTag)==1) continue;//hole 
			if (!polyBounds[iLine].Inside(m_Cloud[i])) continue;
			//int polyNum = m_mapTops[iLine].Number();
			//if (m_Cloud[i].InsidePolygon(m_mapPnts, m_mapTops[iLine])) {
			if(MyIsInsidePoly(m_Cloud[i], m_mapTops[iLine], m_mapPnts)) {
				m_Cloud[i].SetAttribute(PolygonNumberTag, m_mapTops[iLine].Number());
				//m_Cloud[i].SetAttribute(ComponentNumberTag, m_mapTops[iLine].Number());
#ifdef _DEBUG
				m_Cloud[i].SetAttribute(ComponentNumberTag, m_mapTops[iLine].Number());
#endif
				break;
			}
		}
	}
	time_t end;
	time(&end);
	printf("  Time used for match data: %.2f second\n",difftime(end, start));


	MyDeriveSegPNL(m_Cloud, m_PnlPolyTag, PolygonNumberTag);

	return true;
}



bool ModelBldWithFootPrint::MatchData3()
{
	return false;
}

bool ModelBldWithFootPrint::GetGroupPnts(LaserPoints& cloud, int tagNum, LaserPointTag tag)
{
	cloud.clear();
	PointNumberList curPnl;
	int ind;
	vector<PointNumberList>* vecPnls;

	switch(tag)
	{
	case SegmentNumberTag:
		//vecPnls = &m_PnlSegTag;
		break;
	case PolygonNumberTag:
		vecPnls = &m_PnlPolyTag;
		break;
	default:
		return false;
	}

	ind = IndexSegBySegNumber(m_Cloud, *vecPnls, tagNum, tag);
	if (ind == -1) return false;

	for (int i=0; i<(*vecPnls)[ind].size(); ++i) 
		cloud.push_back(m_Cloud[(*vecPnls)[ind][i].Number()]);
	return true;
}

bool ModelBldWithFootPrint::WriteOutModels(const char* modelObjPath, const char* modelTopPath)
{
	if (!modelObjPath && !modelTopPath) return false;
	m_pcmBlds.WriteMapData(modelTopPath);
	//m_pcmBlds.Write(modelTopPath);
	m_modelObjPnts.Write(modelObjPath);
	m_Cloud.Write(m_CloudPath.c_str());//the laser points have been changed
}

//bool ModelBldWithFootPrint::IntersectWallsWithRoofs(LaserPoints cloud, )
//{

//}

bool ModelBldWithFootPrint::IntersectWallsWithRoofs(LaserPoints& cloud, LineTopology& footprintTop, 
	ObjectPoints& footprintPnts, std::vector<Plane>& vecLocalPlanes, 
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	LineSegment3D lineSeg;
	Plane wallPlane;
	ObjectPoint footPnt1, footPnt2, footPnt3;
	double distThr = 0.6;

	LaserPoints::iterator itrPnt;
	double scalar;
	int segmentNum;
	std::map<int, PointNumberList> mapAdjPntNums;
	std::map<int, PointNumberList>::iterator itrMap;
	int indPlane;
	Plane roofPlane;
	Position3D posBeg, posEnd;
	vector< vector<int> > vecEaveNums;
	vecEaveNums = vector< vector<int> >(footprintTop.size()-1, vector<int>());


	LineSegment3D curLineSeg;
	vector<LineSegment3D> vecLineSegs;
	vector<LineSegment3D>::iterator lineSeg1, lineSeg2;
	LineSegment2D footLine;

	//intersect roofs with each wall
	for (int iLine=0; iLine<footprintTop.size()-1; ++iLine) {
		footPnt1 = footprintPnts[footprintTop[iLine].Number()];
		footPnt2 = footprintPnts[footprintTop[iLine+1].Number()];
		footLine = LineSegment2D(footPnt1.Position2DOnly(), footPnt2.Position2DOnly());
		footPnt3 = footPnt1;
		footPnt3.Z() += 1000.0;
		//lineSeg = LineSegment3D(footPnt1, footPnt2);
		wallPlane.Erase();
		wallPlane.AddPoint(footPnt1.Position3DRef(), false);
		wallPlane.AddPoint(footPnt2.Position3DRef(), false);
		wallPlane.AddPoint(footPnt3.Position3DRef(), false);
		wallPlane.Recalculate();


		mapAdjPntNums.clear();
		for (itrPnt=cloud.begin(); itrPnt!=cloud.end(); itrPnt++) {
			if (footLine.DistanceToPoint(itrPnt->Position2DOnly())>distThr) continue;
			scalar = footLine.Scalar(itrPnt->Position2DOnly());
			if (scalar<footLine.ScalarBegin()||scalar>footLine.ScalarEnd()) continue;

			if (itrPnt->HasAttribute(SegmentNumberTag))
				segmentNum = itrPnt->Attribute(SegmentNumberTag);
			else
				segmentNum = -1;


			mapAdjPntNums[segmentNum].push_back(PointNumber(itrPnt-cloud.begin()));
		}


		vecLineSegs.clear();
		for (itrMap=mapAdjPntNums.begin(); itrMap!=mapAdjPntNums.end(); ++itrMap) {
			segmentNum = itrMap->first;
			if (segmentNum==-1) continue;//no segment, skip now
			indPlane = IndexPlaneBySegNumber(vecLocalPlanes, segmentNum);
			if (indPlane==-1) continue;
			roofPlane = vecLocalPlanes[indPlane];
			if (!cloud.IntersectFaces(itrMap->second, itrMap->second, 
				wallPlane, roofPlane, distThr, posBeg, posEnd))
				continue;
			if (posBeg.Distance(posEnd)<1.5) continue;

			if (segmentNum==522){
				int aaa =0;
			}
			//out put
			curLineSeg = LineSegment3D(posBeg, posEnd);
			curLineSeg.numb() = roofPlane.Number();
			vecLineSegs.push_back(curLineSeg);

		}


		double scale1, scale2;
		double bContained;
		
		for(int iSeg=0; iSeg<vecLineSegs.size(); iSeg++){
			lineSeg1 = vecLineSegs.begin()+iSeg;
			if (lineSeg1->Number()==529){
				int aaa = 0;
			}

			bContained = false;
			for (int jSeg=0; jSeg<vecLineSegs.size(); jSeg++)	{
				if (iSeg==jSeg) continue;
				lineSeg2 = vecLineSegs.begin()+jSeg;

				if (lineSeg2->Length()<lineSeg1->Length()) continue;
				scale1 = lineSeg1->Scalar(lineSeg2->BeginPoint());
				scale2 = lineSeg1->Scalar(lineSeg2->EndPoint());
				if (scale1>scale2) std::swap(scale1, scale2);
			

				if (scale1<lineSeg1->ScalarBegin()&&scale2>lineSeg1->ScalarEnd()) {
					bContained = true;
					break;
				}
			}

			if (bContained) {
				vecLineSegs.erase(lineSeg1);
				iSeg--;
			}
		}

		for(int iSeg=0; iSeg<vecLineSegs.size(); iSeg++){
			LineSegment3D& line = vecLineSegs[iSeg];
			AddTopLine(localObjPnts, localLineTops, line.BeginPoint(),
				line.EndPoint(), line.Number(), -1);
			vecEaveNums[iLine].push_back(localLineTops.size()-1);
		}
		//mapAdjPntNums.begin()
	}
	

	int curL, nexL, priL;//line index for current, next and prior foot print lines;
	int curS, nexS, priS;//segment number for current, next and prior roof face;
	Line3D wallLine, eaveLine1, eaveLine2;
	for (int curL=0; curL<footprintTop.size()-1; curL++) {
		priL = curL-1;
		nexL = curL+1;
		if (curL==0) priL = footprintTop.size()-2;
		else if (curL==footprintTop.size()-2) nexL = 0;

		if (!vecEaveNums[curL].empty()) continue;

		if (vecEaveNums[priL].size()!=1||vecEaveNums[nexL].size()!=1) continue;
		priS = localLineTops[vecEaveNums[priL][0]].Attribute(SegmentLabel);
		nexS = localLineTops[vecEaveNums[nexL][0]].Attribute(SegmentLabel);
		if (priS!=nexS) continue;//the two footprints should map to one segment
		indPlane = IndexPlaneBySegNumber(vecLocalPlanes, priS);
		if (indPlane==-1) continue;

		footPnt1 = footprintPnts[footprintTop[curL].Number()];
		wallLine = Line3D(footPnt1, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(wallLine, vecLocalPlanes[indPlane], posBeg);
		footPnt2 = footprintPnts[footprintTop[curL+1].Number()];
		wallLine = Line3D(footPnt2, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(wallLine, vecLocalPlanes[indPlane], posEnd);
		//out put
		AddTopLine(localObjPnts, localLineTops, posBeg, posEnd, priS, -1);
		vecEaveNums[curL].push_back(localLineTops.size()-1);


		eaveLine1 = Line3D(posBeg, posEnd);

		LineTopology& eave1 = localLineTops[vecEaveNums[priL][0]];
		eaveLine2=Line3D(localObjPnts[eave1[0].Number()].pos(),localObjPnts[eave1[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		if (localObjPnts[eave1[0].Number()].Distance(posBeg)
			<localObjPnts[eave1[1].Number()].Distance(posBeg))
			localObjPnts[eave1[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave1[1].Number()].pos() = posBeg;
		//for next eave
		LineTopology& eave2 = localLineTops[vecEaveNums[nexL][0]];
		eaveLine2=Line3D(localObjPnts[eave2[0].Number()].pos(),localObjPnts[eave2[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		if (localObjPnts[eave2[0].Number()].Distance(posBeg)
			<localObjPnts[eave2[1].Number()].Distance(posBeg))
			localObjPnts[eave2[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave2[1].Number()].pos() = posBeg;
	}


	for (int curL=0; curL<footprintTop.size()-1; curL++) {
		nexL = curL+1;
		if (curL==footprintTop.size()-2) nexL = 0;
		//only care the footprints mapping to one segment
		if (vecEaveNums[curL].size()!=1||vecEaveNums[nexL].size()!=1) continue;
		priS = localLineTops[vecEaveNums[curL][0]].Attribute(SegmentLabel);
		nexS = localLineTops[vecEaveNums[nexL][0]].Attribute(SegmentLabel);
		if (priS!=nexS) continue;//the two footprints should map to one segment


		LineTopology& eave1 = localLineTops[vecEaveNums[curL][0]];
		eaveLine1=Line3D(localObjPnts[eave1[0].Number()].pos(),localObjPnts[eave1[1].Number()].pos());
		LineTopology& eave2 = localLineTops[vecEaveNums[nexL][0]];
		eaveLine2=Line3D(localObjPnts[eave2[0].Number()].pos(),localObjPnts[eave2[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		//for current eave
		if (localObjPnts[eave1[0].Number()].Distance(posBeg)
			<localObjPnts[eave1[1].Number()].Distance(posBeg))
			localObjPnts[eave1[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave1[1].Number()].pos() = posBeg;
		//for next eave
		if (localObjPnts[eave2[0].Number()].Distance(posBeg)
			<localObjPnts[eave2[1].Number()].Distance(posBeg))
			localObjPnts[eave2[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave2[1].Number()].pos() = posBeg;
	}


	return true;
}



bool ModelBldWithFootPrint::IntersectWallsWithRoofs2(LaserPoints& cloud, LineTopology& footprintTop, 
	ObjectPoints& footprintPnts, std::vector<Plane>& vecLocalPlanes, 
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{

	LineSegment3D lineSeg;
	Plane wallPlane;
	ObjectPoint footPnt1, footPnt2, footPnt3;
	double distThr = 0.6;

	LaserPoints::iterator itrPnt;
	double scalar;
	int segmentNum;
	std::map<int, PointNumberList> mapAdjPntNums;//first: segment tag, second: segment numbers
	std::map<int, PointNumberList>::iterator itrMap;
	int indPlane;
	Plane roofPlane;
	Position3D posBeg, posEnd;
	vector< vector<int> > vecEaveNums;//intersection lines for each footprint lines
	vecEaveNums = vector< vector<int> >(footprintTop.size()-1, vector<int>());


	LineSegment3D curLineSeg;
	vector<LineSegment3D> vecLineSegs;
	vector<LineSegment3D>::iterator lineSeg1, lineSeg2;
	LineSegment2D footLine;


	for (int iLine=0; iLine<footprintTop.size()-1; ++iLine) {
		footPnt1 = footprintPnts[footprintTop[iLine].Number()];
		footPnt2 = footprintPnts[footprintTop[iLine+1].Number()];
		footLine = LineSegment2D(footPnt1.Position2DOnly(), footPnt2.Position2DOnly());
		footPnt3 = footPnt1;
		footPnt3.Z() += 1000.0;
		//lineSeg = LineSegment3D(footPnt1, footPnt2);
		wallPlane.Erase();
		wallPlane.AddPoint(footPnt1.Position3DRef(), false);
		wallPlane.AddPoint(footPnt2.Position3DRef(), false);
		wallPlane.AddPoint(footPnt3.Position3DRef(), false);
		wallPlane.Recalculate();


		mapAdjPntNums.clear();
		for (itrPnt=cloud.begin(); itrPnt!=cloud.end(); itrPnt++) {
			if (footLine.DistanceToPoint(itrPnt->Position2DOnly())>distThr) continue;
			scalar = footLine.Scalar(itrPnt->Position2DOnly());
			if (scalar<footLine.ScalarBegin()||scalar>footLine.ScalarEnd()) continue;
			//if (wallPlane.Distance(itrPnt->Position3DRef())>distThr) continue;
			//scalar = lineSeg.Scalar(itrPnt->Position3DRef());
			//if (scalar<lineSeg.ScalarBegin() || scalar>lineSeg.ScalarEnd()) continue;
			if (itrPnt->HasAttribute(SegmentNumberTag))
				segmentNum = itrPnt->Attribute(SegmentNumberTag);
			else
				segmentNum = -1;

			//if (iLine==11) 
			//	boundPnts.push_back(*itrPnt);
			mapAdjPntNums[segmentNum].push_back(PointNumber(itrPnt-cloud.begin()));
		}


		vecLineSegs.clear();
		for (itrMap=mapAdjPntNums.begin(); itrMap!=mapAdjPntNums.end(); ++itrMap) {
			segmentNum = itrMap->first;
			if (segmentNum==-1) continue;//no segment, skip now
			indPlane = IndexPlaneBySegNumber(vecLocalPlanes, segmentNum);
			if (indPlane==-1) continue;
			roofPlane = vecLocalPlanes[indPlane];
			if (!cloud.IntersectFaces(itrMap->second, itrMap->second, 
				wallPlane, roofPlane, distThr, posBeg, posEnd))
				continue;
			

			if (segmentNum==522){
				int aaa =0;
			}
			//out put
			curLineSeg = LineSegment3D(posBeg, posEnd);
			if (posBeg.Distance(posEnd)<1.0)continue;
			//if (posBeg.Distance(posEnd)<1.5) curLineSeg.label = 1;
			//else curLineSeg.label = 0;

			curLineSeg.numb() = roofPlane.Number();
			vecLineSegs.push_back(curLineSeg);
			//AddTopLine(localObjPnts, localLineTops, posBeg, posEnd, roofPlane.Number(), -1);
			//vecEaveNums[iLine].push_back(localLineTops.size()-1);
		}


		double scale1, scale2;
		double bContained;

		for(int iSeg=0; iSeg<vecLineSegs.size(); iSeg++){
			lineSeg1 = vecLineSegs.begin()+iSeg;
			if (lineSeg1->Number()==529){
				int aaa = 0;
			}

			bContained = false;
			for (int jSeg=0; jSeg<vecLineSegs.size(); jSeg++)	{
				if (iSeg==jSeg) continue;
				lineSeg2 = vecLineSegs.begin()+jSeg;

				if (lineSeg2->Length()<lineSeg1->Length()) continue;
				scale1 = lineSeg1->Scalar(lineSeg2->BeginPoint());
				scale2 = lineSeg1->Scalar(lineSeg2->EndPoint());
				if (scale1>scale2) std::swap(scale1, scale2);

				//line 1 is contained by line 2
				if (scale1<lineSeg1->ScalarBegin()&&scale2>lineSeg1->ScalarEnd()) {
					bContained = true;
					break;
				}
			}

			if (bContained) {
				vecLineSegs.erase(lineSeg1);
				iSeg--;
			}
		}

		for(int iSeg=0; iSeg<vecLineSegs.size(); iSeg++){
			LineSegment3D& line = vecLineSegs[iSeg];
			AddTopLine(localObjPnts, localLineTops, line.BeginPoint(),
				line.EndPoint(), line.Number(), -1);
			vecEaveNums[iLine].push_back(localLineTops.size()-1);
		}
		//mapAdjPntNums.begin()
	}


	int curL, nexL, priL;//line index for current, next and prior foot print lines;
	int curS, nexS, priS;//segment number for current, next and prior roof face;
	Line3D wallLine, eaveLine1, eaveLine2;
	for (int curL=0; curL<footprintTop.size()-1; curL++) {
		priL = curL-1;
		nexL = curL+1;
		if (curL==0) priL = footprintTop.size()-2;
		else if (curL==footprintTop.size()-2) nexL = 0;
		//only care the footprint which has no intersection with wall
		if (!vecEaveNums[curL].empty()) continue;
		//only care the footprints mapping to one segment
		if (vecEaveNums[priL].size()!=1||vecEaveNums[nexL].size()!=1) continue;
		priS = localLineTops[vecEaveNums[priL][0]].Attribute(SegmentLabel);
		nexS = localLineTops[vecEaveNums[nexL][0]].Attribute(SegmentLabel);
		if (priS!=nexS) continue;//the two footprints should map to one segment
		indPlane = IndexPlaneBySegNumber(vecLocalPlanes, priS);
		if (indPlane==-1) continue;

		footPnt1 = footprintPnts[footprintTop[curL].Number()];
		wallLine = Line3D(footPnt1, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(wallLine, vecLocalPlanes[indPlane], posBeg);
		footPnt2 = footprintPnts[footprintTop[curL+1].Number()];
		wallLine = Line3D(footPnt2, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(wallLine, vecLocalPlanes[indPlane], posEnd);
		//out put
		AddTopLine(localObjPnts, localLineTops, posBeg, posEnd, priS, -1);
		vecEaveNums[curL].push_back(localLineTops.size()-1);


		eaveLine1 = Line3D(posBeg, posEnd);

		LineTopology& eave1 = localLineTops[vecEaveNums[priL][0]];
		eaveLine2=Line3D(localObjPnts[eave1[0].Number()].pos(),localObjPnts[eave1[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		if (localObjPnts[eave1[0].Number()].Distance(posBeg)
			<localObjPnts[eave1[1].Number()].Distance(posBeg))
			localObjPnts[eave1[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave1[1].Number()].pos() = posBeg;
		//for next eave
		LineTopology& eave2 = localLineTops[vecEaveNums[nexL][0]];
		eaveLine2=Line3D(localObjPnts[eave2[0].Number()].pos(),localObjPnts[eave2[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		if (localObjPnts[eave2[0].Number()].Distance(posBeg)
			<localObjPnts[eave2[1].Number()].Distance(posBeg))
			localObjPnts[eave2[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave2[1].Number()].pos() = posBeg;
	}


	for (int curL=0; curL<footprintTop.size()-1; curL++) {
		nexL = curL+1;
		if (curL==footprintTop.size()-2) nexL = 0;
		//only care the footprints mapping to one segment
		if (vecEaveNums[curL].size()!=1||vecEaveNums[nexL].size()!=1) continue;
		priS = localLineTops[vecEaveNums[curL][0]].Attribute(SegmentLabel);
		nexS = localLineTops[vecEaveNums[nexL][0]].Attribute(SegmentLabel);
		if (priS!=nexS) continue;//the two footprints should map to one segment


		LineTopology& eave1 = localLineTops[vecEaveNums[curL][0]];
		eaveLine1=Line3D(localObjPnts[eave1[0].Number()].pos(),localObjPnts[eave1[1].Number()].pos());
		LineTopology& eave2 = localLineTops[vecEaveNums[nexL][0]];
		eaveLine2=Line3D(localObjPnts[eave2[0].Number()].pos(),localObjPnts[eave2[1].Number()].pos());
		Intersection2Lines(eaveLine1, eaveLine2, posBeg);
		//for current eave
		if (localObjPnts[eave1[0].Number()].Distance(posBeg)
			<localObjPnts[eave1[1].Number()].Distance(posBeg))
			localObjPnts[eave1[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave1[1].Number()].pos() = posBeg;
		//for next eave
		if (localObjPnts[eave2[0].Number()].Distance(posBeg)
			<localObjPnts[eave2[1].Number()].Distance(posBeg))
			localObjPnts[eave2[0].Number()].pos() = posBeg;
		else
			localObjPnts[eave2[1].Number()].pos() = posBeg;
	}


	return true;
}
