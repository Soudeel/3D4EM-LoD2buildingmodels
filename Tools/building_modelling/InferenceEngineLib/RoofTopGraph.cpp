/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : May 12, 2015
Description: 
Revise:
----------------------------------------------------------*/

#include <assert.h>
#include "RoofTopGraph.h"
#include "InferenceEngine.h"
#include "LaserPoint.h"
#include "Vector3D.h"
#include "Plane.h"
#include "Line3D.h"
#include "TIN.h"
#include "LaserPoints.h"
#include "TINEdges.h"
#include "LineSegment2D.h"
#include "LineSegments2D.h"

#include <algorithm>
#include <vector>
#include <stack>
#include <queue>
#include <set>
#include <map>

using namespace std;


RoofTopGraphSearcher::RoofTopGraphSearcher(
	const LaserPoints& cloud, const std::vector<Plane>& vecSurPlanes, 
	const ObjectPoints& pContourObjPnt, const LineTopology& pContourTop,
	LaserPointTag surfaceTag)
{
	m_surfaceTag = surfaceTag;

	m_Cloud = cloud;
	m_vecSurPlanes = vecSurPlanes;

	//if (!m_pTin) m_pTin = m_pCloud->DeriveTIN();
	//assert(m_pTin);
	
	m_contourObjPnts = pContourObjPnt;
	m_contourTop = pContourTop;

}


bool RoofTopGraphSearcher::CleanAllData()
{
	m_Cloud.clear();
	m_Tin.clear();
	m_TINEdges.clear();
	m_contourObjPnts.clear();
	m_contourTop.clear();
	m_pnlContour.clear();

	return true;
}

bool RoofTopGraphSearcher::Initialize()
{
	CombineLasAndContour();
	m_contourSegs = LineSegments2D(m_contourObjPnts, m_contourTop);

	MakeTin();
	m_TINEdges.Derive(m_Tin);
	if (m_TINEdges.size()!=m_Cloud.size()) {
		CleanAllData();
		return false;
	}

	assert(m_TINEdges.size()==m_Cloud.size());
	SurfaceGrowContourPnts();
	RepairSegmentTag();
	MyDeriveSegPNL(m_Cloud, m_pnlSegs, m_surfaceTag);

	return true;
}

RoofTopGraphSearcher::~RoofTopGraphSearcher()
{

}


bool RoofTopGraphSearcher::DoCompute()
{
	TINEdges gEdges;
	m_Cloud.DeriveTIN();
	gEdges.Derive(m_Cloud.TINReference());
	m_Cloud.SetAttribute(IsProcessedTag, -1); //-1 unprocessed 1 processed

	ReconstructLocalSegments(m_Cloud, gEdges, m_pnlSegs, m_vecSurPlanes,
		m_pnlSegs[0], m_objPntOutLines, m_lineTopOutLines);
	MergeLocalSegments(m_Cloud, m_pnlSegs, m_vecSurPlanes, m_objPntOutLines, m_lineTopOutLines);
	//	RefineSingleRoof(m_Cloud, m_vecSurPlanes, m_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	//	RefineSimpleDormer(m_Cloud, m_vecSurPlanes, m_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	RefineCornerOf3Faces(m_Cloud, m_vecSurPlanes, m_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	RefineCornerOf4Faces(m_vecSurPlanes, m_objPntOutLines, m_lineTopOutLines);
	RefineCornerOfXFaces(m_Cloud, m_vecSurPlanes, m_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	//RefineOutBoundaryLine6(lasPnts, vecm_vecSurPlanes, gm_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	//ComputeBoundLineCoef(lasPnts, vecm_vecSurPlanes, gm_pnlSegs, m_objPntOutLines, m_lineTopOutLines);
	//ComputeRoofFaceConf(lasPnts, vecm_vecSurPlanes, gm_pnlSegs, m_objPntOutLines, m_lineTopOutLines);


	return false;
}

bool RoofTopGraphSearcher::DeriveRidges()
{
	return false;
}

void RoofTopGraphSearcher::ExtendRidgeOrCreateAuxilityLine()
{
	/*
	if (m_lineTopOutLines.size()<=1) return ;

	//////////////////////////////////////////////////////////////////////////
	//detect stabel points and line, only record one time for points in same position
	typedef ObjectPoints::iterator OPIterator;
	typedef LineTopologies::iterator LTopIterator;
	OPIterator objPnt1, objPnt2;
	vector<bool> vecRepOPnts(m_objPntOutLines.size(), false);
	bool bLeftRepeat, bRightRepeat;

	for (int i=0; i<m_lineTopOutLines.size(); ++i) {
		//if(m_lineTopOutLines[i].Attribute(2) == 1) continue;//processed
		objPnt1 = m_objPntOutLines.begin() + m_lineTopOutLines[i][0].Number();
		objPnt2 = m_objPntOutLines.begin() + m_lineTopOutLines[i][1].Number();
		bLeftRepeat = bRightRepeat = false;
		for (OPIterator itrOp=m_objPntOutLines.begin(); itrOp!=m_objPntOutLines.end(); ++itrOp) {
			if (objPnt1 != itrOp && objPnt1->X()==itrOp->X()
				&& objPnt1->Y()==itrOp->Y() && objPnt1->Z()==itrOp->Z()) {
					bLeftRepeat = true; 
					vecRepOPnts[itrOp-m_objPntOutLines.begin()] = true;
			}
			else if (objPnt2 != itrOp && objPnt2->X()==itrOp->X()
				&& objPnt2->Y()==itrOp->Y()&&objPnt2->Z()==itrOp->Z()){
					bRightRepeat = true;
					vecRepOPnts[itrOp-m_objPntOutLines.begin()] = true;
			}
		}

		if (bLeftRepeat && bRightRepeat)
			m_lineTopOutLines[i].SetAttribute(2, 1);//stable line
		else
			m_lineTopOutLines[i].SetAttribute(2, -1);//un-stable line
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//detect out boundary line direction 
	int lPlaneNum, rPlaneNum, lPlaneInd, rPlaneInd, lSegInd, rSegInd;
	Line3D curLine, lLine, rLine, lAuxLine, rAuxLine;
	//Vector3D curDir(1,1,1),lAuxDir(1,1,1), rAuxDir(1,1,1);
	Position3D lTemPos, rTemPos, startPos, endPos;
	typedef LaserPoints::iterator LPntIterator;
	LPntIterator temLPnt;
	//	double scalar, maxRScal, maxLScal;
	Position3D 	posEndPnt;
	LineTopology temLineTop;
	ObjectPoint temObjPnt;
	//	bool bVerHorLine;
	int orgLineNum = m_lineTopOutLines.size();
	vector<int> vecNodeInds;
	vecNodeInds.reserve(20);

	for (int i=0; i<orgLineNum; ++i) {
		if(m_lineTopOutLines[i].Attribute(2) == 1) continue;
		lPlaneNum = m_lineTopOutLines[i].Attribute(SegmentLabel);
		rPlaneNum = m_lineTopOutLines[i].Attribute(RightPlaneTag);

#ifdef _DEBUG
		if ((lPlaneNum == 3 && rPlaneNum == 1)
			||(lPlaneNum == 1 && rPlaneNum == 3))
		{
			int aaa = 0;
		}
#endif

		lPlaneInd = IndexPlaneBySegNumber(m_vecSurPlanes, lPlaneNum);
		rPlaneInd = IndexPlaneBySegNumber(m_vecSurPlanes, rPlaneNum);
		lSegInd = IndexSegBySegNumber(m_Cloud, m_pnlSegs, lPlaneNum);
		rSegInd = IndexSegBySegNumber(m_Cloud, m_pnlSegs, rPlaneNum);
		if (lSegInd==-1 || rSegInd==-1) continue;

		bLeftRepeat = vecRepOPnts[m_lineTopOutLines[i][0].Number()];
		bRightRepeat = vecRepOPnts[m_lineTopOutLines[i][1].Number()];

		//determine possible end node direction
		//the direction of this line is very important
		vecNodeInds.clear();
		if (!bLeftRepeat && !bRightRepeat) {//no stable node point
			vecNodeInds.push_back(m_lineTopOutLines[i][0].Number());
			vecNodeInds.push_back(m_lineTopOutLines[i][1].Number());
			vecNodeInds.push_back(m_lineTopOutLines[i][1].Number());
			vecNodeInds.push_back(m_lineTopOutLines[i][0].Number());
		}
		else if (bLeftRepeat  && !bRightRepeat){//left node is stable
			vecNodeInds.push_back(m_lineTopOutLines[i][0].Number());
			vecNodeInds.push_back(m_lineTopOutLines[i][1].Number());
		}
		else if (!bLeftRepeat  && bRightRepeat){//right node is stable
			vecNodeInds.push_back(m_lineTopOutLines[i][1].Number());
			vecNodeInds.push_back(m_lineTopOutLines[i][0].Number());
		}

		//point 1 --> point 2
		//determine end node
		for (int iNode=0; iNode<vecNodeInds.size(); iNode+=2) {
			double dis1, dis2;
			int longerFlag = 0;//-1:right longer then left; 0: equal; 1 left longer then right
			objPnt1 = m_objPntOutLines.begin() + vecNodeInds[iNode+0];
			objPnt2 = m_objPntOutLines.begin() + vecNodeInds[iNode+1];
			curLine = Line3D(*objPnt1, *objPnt2);

			if (lPlaneInd >= 0)
				lTemPos = ComputeEndPnt(m_Cloud,m_pnlSegs[lSegInd], 
				m_vecSurPlanes[lPlaneInd], curLine);
			if (rPlaneInd >= 0) 
				rTemPos = ComputeEndPnt(m_Cloud,m_pnlSegs[rSegInd], 
				m_vecSurPlanes[rPlaneInd], curLine);

			if (lPlaneInd>=0 && rPlaneInd>=0) {
				dis1 = lTemPos.Distance(objPnt1->Position3DRef());
				dis2 = rTemPos.Distance(objPnt1->Position3DRef());

				//left edge is longer or not then right edge
				if (dis1>dis2) {
					longerFlag = 1;
					if (dis1-dis2<0.5) {
						posEndPnt = lTemPos;
						longerFlag = 0;
					}
					else 
						posEndPnt = rTemPos;
				}
				else {
					longerFlag = -1;
					if (dis2-dis1<0.5) {
						posEndPnt = rTemPos;
						longerFlag = 0;
					}
					else 
						posEndPnt = lTemPos;
				}


				//	if (dis1>dis2) 
				//		posEndPnt = rTemPos;
				//	else
				//		posEndPnt = lTemPos;

				//	longerFlag = 0;
			}

			Vector3D temDir;
			if (lPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(m_vecSurPlanes[lPlaneInd].Normal());
				ComputeNextLineDir(m_Cloud,m_pnlSegs[lSegInd],
					m_vecSurPlanes[lPlaneInd],posEndPnt,curLine.Direction(), 
					longerFlag, temDir);
				lLine = Line3D(posEndPnt,temDir);
			}
			if (rPlaneInd >= 0) {
				temDir = curLine.Direction().VectorProduct(m_vecSurPlanes[rPlaneInd].Normal());
				ComputeNextLineDir(m_Cloud,m_pnlSegs[rSegInd],
					m_vecSurPlanes[rPlaneInd],posEndPnt,curLine.Direction(),
					-longerFlag, temDir);
				rLine = Line3D(posEndPnt,temDir);
			}

			//refine original end point
			//the refined line should not be too short
			if (objPnt1->Distance(posEndPnt) > 0.005)
				objPnt2->Position3DRef() = posEndPnt;
			vecRepOPnts[objPnt2-m_objPntOutLines.begin()] = true;
			m_lineTopOutLines[i].SetAttribute(2, 1);//stable

			//add left line
			if (lPlaneInd >= 0){
				AddTopLine(m_objPntOutLines, m_lineTopOutLines, lLine.FootPoint(), 
					lLine.FootPoint()+lLine.Direction(), lPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}
			//add right line
			if (rPlaneInd >= 0) {
				AddTopLine(m_objPntOutLines, m_lineTopOutLines, rLine.FootPoint(), 
					rLine.FootPoint()+rLine.Direction(), rPlaneNum, -1, -1);
				vecRepOPnts.push_back(true);
				vecRepOPnts.push_back(false);
			}
		}
	}*/
}

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include "PolygonFunctions.h"

bool RoofTopGraphSearcher::CombineLasAndContour()
{
	//////////////////////////////////////////////////////////////////////////
	//Step 1: density the contour
	LineTopologies temLineTops;
	temLineTops.push_back(m_contourTop);
	m_contourObjPnts.RemoveDoublePoints(temLineTops, 0.000001);
	m_contourObjPnts.RemoveUnusedPnts(temLineTops);
	m_contourTop = temLineTops[0];
	m_contourTop.Densify(m_contourObjPnts, 0.5);

	LineTopology newContourTop;
	LaserPoint newLasPnt;
	newLasPnt.SetAttribute(m_surfaceTag, -1);
	LaserPoints::iterator itrPnt;
	ObjectPoints::iterator conPnt;

	/*
#ifdef _DEBUG
	LineTopologies debugTops;
	debugTops.push_back(m_contourTop);
	debugTops.Write("debug.top");
	m_contourObjPnts.Write("debug.objpts");
	m_Cloud.Write("debug.laser");
#endif
	*/

	//////////////////////////////////////////////////////////////////////////
	//Step 2: clean laser points outside the contour
	std::vector<bool> vecValid = std::vector<bool> (m_Cloud.size(), true);
/*	LineSegments2D contourLineSegs = LineSegments2D(m_contourObjPnts, m_contourTop);
	for (LASITR pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
#ifdef _DEBUG
		if (fabs(pnt->X()-185526.80)<0.01 && fabs(pnt->Y()-319972.30)<0.01)
		{
			int aaa = 1;
		}
#endif
		if (!pnt->InsidePolygonJordan(m_contourObjPnts, m_contourTop))
			vecValid[pnt-m_Cloud.begin()] = false;

		//check whether the point is on the contour
		if (vecValid[pnt-m_Cloud.begin()] == false) continue;
		
		Position2D temPnt2D = pnt->Position2DOnly();
		for (unsigned int i=0; i<contourLineSegs.size(); i++){
			double dist = contourLineSegs[i].DistanceToPoint(temPnt2D);
			if (contourLineSegs[i].DistanceToPoint(temPnt2D)>0.00000001)continue;
			vecValid[pnt-m_Cloud.begin()] = false;
			break;
		}
	}*/
	ObjectPoint temObjPnt;
	for (LASITR pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
#ifdef _DEBUG
		if (fabs(pnt->X()-185526.80)<0.01 && fabs(pnt->Y()-319972.30)<0.01)
		{
			int aaa = 1;
		}
#endif
		temObjPnt.Position3DRef() = pnt->Position3DRef();
		if (1!=BoundedSidePolygon(m_contourObjPnts, m_contourTop, temObjPnt))
			vecValid[pnt-m_Cloud.begin()] = false;
	}

	{
		/*typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
		typedef K::Point_2 Point;

		std::vector<bool> vecValid = std::vector<bool> (m_Cloud.size(), true);
		std::vector<Point> localPnts;
		localPnts.reserve(m_contourTop.size());

		for (unsigned int i=0; i<m_contourTop.size(); i++) {
			ObjectPoint& temPnt = m_contourObjPnts[m_contourTop[i].Number()];
			localPnts.push_back(Point(temPnt.X(), temPnt.Y()));
		}

		for (LASITR pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
			switch (CGAL::bounded_side_2(localPnts.begin(), localPnts.end(), 
				Point(pnt->X(), pnt->Y()), K()))
			{
			case CGAL::ON_BOUNDED_SIDE :
				break;
			case CGAL::ON_BOUNDARY:
			case CGAL::ON_UNBOUNDED_SIDE:
				vecValid[pnt-m_Cloud.begin()]=false; break;
			}
		}*/

		for (unsigned int i=0; i<m_Cloud.size(); i++) {
			if (vecValid[i]) continue;
			vecValid.erase(vecValid.begin()+i);
			m_Cloud.erase(m_Cloud.begin()+i);
			i--;
		}
		if (m_Cloud.empty()) {
			m_contourObjPnts.clear();
			m_contourTop.clear();
			return false;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//step 3: add contour points to laser points
	for (unsigned int i=0; i!=m_contourTop.size()-1; i++) {
		conPnt = m_contourObjPnts.begin()+m_contourTop[i].Number();
		newLasPnt.Position3DRef() = conPnt->Position3DRef();

		for (itrPnt=m_Cloud.begin(); itrPnt!=m_Cloud.end(); itrPnt++) {
			if (itrPnt->X()==newLasPnt.X() && itrPnt->Y()==newLasPnt.Y()) 
				break;
		}
		
		if (itrPnt == m_Cloud.end()) {//cannot find
			m_Cloud.push_back(newLasPnt);
			newContourTop.push_back(PointNumber(m_Cloud.size()-1));
		}
		else {//find
			newContourTop.push_back(PointNumber(itrPnt-m_Cloud.begin()));
		}
	}
	newContourTop.push_back(newContourTop[0]);

	//////////////////////////////////////////////////////////////////////////
	//step 4: make contour to match with new point numbers
	ObjectPoint newObjPnt;
	ObjectPoints newContourObjPnts;
	newContourObjPnts.reserve(m_Cloud.size());
	for (itrPnt=m_Cloud.begin(); itrPnt!=m_Cloud.end(); itrPnt++) {
		newObjPnt.Position3DRef() = itrPnt->Position3DRef();
		newObjPnt.Number() = itrPnt-m_Cloud.begin();
		newContourObjPnts.push_back(newObjPnt);
	}

	m_contourObjPnts = newContourObjPnts;
	m_contourTop = newContourTop;

	//////////////////////////////////////////////////////////////////////////
	//make point number list for contour
	m_pnlContour = m_contourTop;

	
#ifdef _DEBUG
	LineTopologies debugTops;
	debugTops.clear();
	debugTops.push_back(m_contourTop);
	debugTops.Write("debug.top");
	m_contourObjPnts.Write("debug.objpts");
	m_Cloud.Write("debug.laser");
#endif
	
	//TIN tin = localBndPnts.Triangulate(localBndTops, holeCents);
	return true;
}

bool RoofTopGraphSearcher::MakeTin()
{
	LineTopologies lineTops;
	lineTops.push_back(m_contourTop);
	m_Tin = m_contourObjPnts.Triangulate(lineTops);

	
#ifdef _DEBUG
	LineTopologies debugTops(m_Tin);
	debugTops.Write("debug.top");
	m_contourObjPnts.Write("debug.objpts");
	m_Cloud.Write("debug.laser");
#endif
	
	return true;
}

bool RoofTopGraphSearcher::IsOnContour(LASITR pnt)
{
	assert(pnt!=m_Cloud.end());
	return IsOnContour(pnt-m_Cloud.begin());
}

bool RoofTopGraphSearcher::IsOnContour(int pntNum)
{
	assert(pntNum>=0 && pntNum<m_Cloud.size());
	return m_pnlContour.end()!=find(m_pnlContour.begin(), m_pnlContour.end(), pntNum);
}


bool RoofTopGraphSearcher::ExportContour(ObjectPoints& destObjPnts, LineTopologies& destLineTops) 
{
	if (m_contourTop.size()<1) return false;

	int segNum1 = m_contourTop.Attribute(BuildingPartNumberTag);
	int segNum2 = -1;
	ObjectPoints::iterator temPnt0, temPnt1;
	m_contourTop.RemoveCollinearNodes(m_contourObjPnts, false);

	for (int iPos=0; iPos<m_contourTop.size()-1; iPos++) {
		temPnt0 = m_contourObjPnts.begin() + m_contourTop[iPos].Number();
		temPnt1 = m_contourObjPnts.begin() + m_contourTop[iPos+1].Number();
		AddTopLine(destObjPnts, destLineTops, *temPnt0, *temPnt1, segNum1, segNum2);
	}

	return true;
}

bool RoofTopGraphSearcher::IsInsideContour(LASITR lasPnt0)
{
	return lasPnt0->InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool RoofTopGraphSearcher::IsInsideContour(int indLasPnt0)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size());
	return IsInsideContour(m_Cloud.begin()+indLasPnt0);
}

bool RoofTopGraphSearcher::IsInsideContour(LASITR lasPnt0, LASITR lasPnt1)
{
	LaserPoint temPnt;
	temPnt.Position3DRef() = (lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2;
	return temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool RoofTopGraphSearcher::IsInsideContour(int indLasPnt0, int indLasPnt1)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size() &&
		indLasPnt1>=0 && indLasPnt1<m_Cloud.size());

	return IsInsideContour(m_Cloud.begin()+indLasPnt0,
		m_Cloud.begin()+indLasPnt1);
}

bool RoofTopGraphSearcher::IsInsideContour(LASITR lasPnt0, LASITR lasPnt1, LASITR lasPnt2)
{
	LaserPoint temPnt;
	temPnt.Position3DRef() = (lasPnt0->Position3DRef()+
		lasPnt1->Position3DRef()+lasPnt2->Position3DRef())/3;
	return temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool RoofTopGraphSearcher::IsInsideContour(int indLasPnt0, int indLasPnt1, int indLasPnt2)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size() &&
		indLasPnt1>=0 && indLasPnt1<m_Cloud.size() &&
		indLasPnt2>=0 && indLasPnt2<m_Cloud.size());

	return IsInsideContour(m_Cloud.begin()+indLasPnt0,
		m_Cloud.begin()+indLasPnt1,
		m_Cloud.begin()+indLasPnt2);
}

bool RoofTopGraphSearcher::IsNearContour(Vector3D pos, double errDist)
{
#ifdef _DEBUG
	vector<double> vecDists;
	for (unsigned int i=0; i<m_contourSegs.size(); i++)
		vecDists.push_back(m_contourSegs[i].DistanceToPoint(pos.vect2D()));
#endif
	//LineSegment2D::
	for (unsigned int i=0; i<m_contourSegs.size(); i++) {
		if (m_contourSegs[i].DistanceToPoint(pos.vect2D())<errDist)
			return true;
	}

	return false;
}

//set contour points with surface tag
void RoofTopGraphSearcher::SurfaceGrowContourPnts()
{
	std::vector<LASITR> unSegPnts;
	for (LASITR pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
		if (pnt->Attribute(m_surfaceTag) == -1)
			unSegPnts.push_back(pnt);
	}
	std::vector<LASITR> unProjectedPnts = unSegPnts;

	//////////////////////////////////////////////////////////////////////////
	//first round, tag point with all neighbor points from one segment
	LASITR curPnt, neibPnt;
	std::set<int> neibSegTags;
	for (int i=unSegPnts.size()-1; i>=0; i--) {
		curPnt = unSegPnts[i];
		const TINEdgeSet& neibEdges = m_TINEdges[curPnt-m_Cloud.begin()];
		
		//collect all valid segment numbers of neighboring points
		neibSegTags.clear();
		for (unsigned int j=0; j<neibEdges.size(); j++) {
			neibPnt = m_Cloud.begin()+neibEdges[j].Number();
			//has valid segment number
			if (neibPnt->Attribute(m_surfaceTag) != -1)
				neibSegTags.insert(neibPnt->Attribute(m_surfaceTag));
		}

		//only one valid neighboring point
		if (neibSegTags.size() == 1) {
			curPnt->SetAttribute(m_surfaceTag, *neibSegTags.begin());
			unSegPnts.erase(unSegPnts.begin()+i);
		}

		if (i==0) break;
	}

	//////////////////////////////////////////////////////////////////////////
	//second round, check all the tin facet with untagged nodes
	std::vector<int> vecSegPnts;
	std::queue<LASITR> queUnSegPnts;
	int maxFreTag;
	for (unsigned int i=0; i<unSegPnts.size(); i++)
		queUnSegPnts.push(unSegPnts[i]);
	
	int nLoop = 0;
	while (!queUnSegPnts.empty()) {
		curPnt = queUnSegPnts.front();
		queUnSegPnts.pop();

		if (nLoop++>1000) break;
		
		//collect all valid segment numbers of neighboring points
		const TINEdgeSet& neibEdges = m_TINEdges[curPnt-m_Cloud.begin()];
		vecSegPnts.clear();
		for (unsigned int i=0; i<neibEdges.size(); i++) {
			neibPnt = m_Cloud.begin()+neibEdges[i].Number();
			//has valid segment number
			if (neibPnt->Attribute(m_surfaceTag) != -1)
				vecSegPnts.push_back(neibPnt->Attribute(m_surfaceTag));
		}

		maxFreTag = MaxFrequentValue(vecSegPnts);
		if (maxFreTag != -1)
			curPnt->SetAttribute(m_surfaceTag, maxFreTag);
		else
			queUnSegPnts.push(curPnt);
	}

	/*LASITR pnt0, pnt1, pnt2;
	int segNum0, segNum1, segNum2;
	for (unsigned int i=0; i<m_Tin.size(); i++)	{
		TINMesh& facet = m_Tin[i];
		pnt0 = m_Cloud.begin() + facet.Nodes()[0].Number();
		pnt1 = m_Cloud.begin() + facet.Nodes()[1].Number();
		pnt2 = m_Cloud.begin() + facet.Nodes()[2].Number();
		segNum0 = pnt0->Attribute(m_surfaceTag);
		segNum1 = pnt1->Attribute(m_surfaceTag);
		segNum2 = pnt2->Attribute(m_surfaceTag);
		if (segNum0!=-1 && segNum1!=-1 && segNum2!=-1)
			continue;

		//make sure the untagged points are after tagged points
		//there is no situation that all three points are untagged
		if (segNum0 == -1) {
			if (segNum1 != -1) {
				std::swap(pnt0, pnt1);
				std::swap(segNum0, segNum1);
			}
			else {
				std::swap(pnt0, pnt2);
				std::swap(segNum0, segNum2);
			}
		}

		if (segNum1 == -1) {
			std::swap(pnt1, pnt2);
			std::swap(segNum1, segNum2);
		}

		//do tag
		if (segNum1 == -1) {
			pnt1->SetAttribute(m_surfaceTag, segNum0);
			pnt2->SetAttribute(m_surfaceTag, segNum0);
		}
		else {
			if (segNum0 == segNum1)
				pnt2->SetAttribute(m_surfaceTag, segNum0);
			else {
				double dist20=pnt2->Position2DOnly().Distance(pnt0->Position2DOnly());
				double dist21=pnt2->Position2DOnly().Distance(pnt1->Position2DOnly());
				segNum2 = dist20<=dist21?segNum0:segNum1;
				pnt2->SetAttribute(m_surfaceTag, segNum2);
			}
		}
	}*/

	//////////////////////////////////////////////////////////////////////////
	//project the boundary points to the destination plane
	Line3D plumb;
	std::vector<LASITR>::iterator itr;
	int indPlane;
	Position3D destPos;
	for (itr=unProjectedPnts.begin(); itr!=unProjectedPnts.end(); itr++) {
		indPlane = IndexPlaneBySegNumber(m_vecSurPlanes, (*itr)->Attribute(m_surfaceTag));
		if (indPlane==-1) continue;
		plumb = Line3D((*itr)->Position3DRef(), Vector3D(0.0, 0.0, 1.0));
		if(!IntersectLine3DPlane(plumb, m_vecSurPlanes[indPlane], destPos)) continue;
		(*itr)->Z() = destPos.Z();
	}

#ifdef _DEBUG
	m_Cloud.Write("debug.laser");
#endif
}

//do component analyse, tag small region as neighboring region tag
void RoofTopGraphSearcher::RepairSegmentTag()
{
	m_Cloud.SetAttribute(AveragePulseLengthTag, -1);
	int regionNum = 0;
	LASITR curPnt, nextPnt;
	std::stack<LASITR> regStack;

	//////////////////////////////////////////////////////////////////////////
	//step 1
	//do region growing
	for (int iPnt=0; iPnt<m_TINEdges.size(); iPnt++) {
		curPnt = m_Cloud.begin() + iPnt;
		if (!curPnt->HasAttribute(m_surfaceTag) || curPnt->Attribute(AveragePulseLengthTag)!=-1) continue;
		regStack.push(curPnt);
		
		while(!regStack.empty()) {
			curPnt = regStack.top();
			regStack.pop();
			curPnt->SetAttribute(AveragePulseLengthTag, regionNum);
			
			TINEdgeSet& neibPnts = m_TINEdges[curPnt-m_Cloud.begin()];
			for (int jPnt=0; jPnt<neibPnts.size(); jPnt++) {
				nextPnt = m_Cloud.begin() + neibPnts[jPnt].Number();
				if (!nextPnt->HasAttribute(m_surfaceTag)) continue;
				if (nextPnt->Attribute(AveragePulseLengthTag)!=-1) continue;
				if (curPnt->Attribute(m_surfaceTag)!=nextPnt->Attribute(m_surfaceTag)) continue;
				regStack.push(nextPnt);
			}
		}
		regionNum++;
	}
	/*
#ifdef _DEBUG
	LaserPoints debugPnts1 = m_Cloud;
	for (unsigned int i=0; i<debugPnts1.size(); i++) {
		debugPnts1[i].SetAttribute(ComponentNumberTag, debugPnts1[i].Attribute(AveragePulseLengthTag));
	}
	debugPnts1.Write("debug.laser");
#endif
	*/
	//get points for each region
	vector<PointNumberList> vecPnlRegions;
	MyDeriveSegPNL(m_Cloud, vecPnlRegions, AveragePulseLengthTag);
	std::map<int, vector<PointNumberList> > mapRegions;
	typedef std::map<int, vector<PointNumberList> >::iterator ITRMAP;
	int segNum;
	for (unsigned int i=0; i<vecPnlRegions.size(); i++) {
		segNum = m_Cloud[vecPnlRegions[i][0].Number()].Attribute(m_surfaceTag);
		ITRMAP itrMap = mapRegions.find(segNum);
		if (itrMap!=mapRegions.end())
			itrMap->second.push_back(vecPnlRegions[i]);
		else
			mapRegions[segNum] = vector<PointNumberList>(1, vecPnlRegions[i]);
	}

	//////////////////////////////////////////////////////////////////////////
	//step 2
	//keep the biggest region in all regions with same segment number tag
	//the point should vertically projected to the new plane
	Plane destPlane;
	Line3D plumbLine;
	Position3D destPos;
	int indMaxReg;
	int maxRegSize;
	std::queue<PointNumberList> unRepairedRegs;
	std::vector<LASITR> vecUnProjPnts;
	std::vector<bool> vecStableStatus = std::vector<bool>(m_Cloud.size(), false);
	for (ITRMAP itrMap=mapRegions.begin(); itrMap!=mapRegions.end(); itrMap++) {
		//get max region
		maxRegSize = 0;
		for (unsigned int i=0; i<itrMap->second.size(); i++) {
			if (maxRegSize<(itrMap->second)[i].size()) {
				maxRegSize = (itrMap->second)[i].size();
				indMaxReg = i;
			}
		}

		//mark max region stable, and store other regions to be tagged
		for (unsigned int i=0; i<itrMap->second.size(); i++) {
			PointNumberList& curReg = itrMap->second[i];

			if (indMaxReg==i) {
				for (unsigned int i=0; i<curReg.size(); i++)
					vecStableStatus[curReg[i].Number()] = true;
			}
			else
				unRepairedRegs.push(curReg);
		}
	}

	//iteratively re-tag small regions
	PointNumberList curReg;
	int newTag;
	int nLoop = 0;
	while(!unRepairedRegs.empty()) {
		if (nLoop++>1000) break;
		curReg = unRepairedRegs.front();
		unRepairedRegs.pop();

		newTag = GetMaximumNeibRegionTag(curReg, vecStableStatus);

		if (newTag == -1)	{//waiting for next iteration
			unRepairedRegs.push(curReg);
		}
		else if(newTag == -2)	{//keep current tag
			for (unsigned int i=0; i<curReg.size(); i++) 
				vecStableStatus[curReg[i].Number()] = true;
		}
		else {//new tag
			for (unsigned int i=0; i<curReg.size(); i++) {
				curPnt = m_Cloud.begin()+curReg[i].Number();
				curPnt->SetAttribute(m_surfaceTag, newTag);
				vecUnProjPnts.push_back(curPnt);
				vecStableStatus[curReg[i].Number()] = true;
			}
		}			
	}

	/*
#ifdef _DEBUG
	LaserPoints debugPnts2 = m_Cloud;
	for (unsigned int i=0; i<debugPnts2.size(); i++) {
		debugPnts2[i].SetAttribute(ComponentNumberTag, debugPnts2[i].Attribute(AveragePulseLengthTag));
	}
	debugPnts2.Write("debug.laser");
#endif
	*/

	//////////////////////////////////////////////////////////////////////////
	//step 3
	//repair silver points
	//-------
	//|     |-o-o-o
	//|     |
	//|     |
	//-------
	std::stack<LASITR> stackUnRepairPnts;
	std::vector<LASITR> vecNiebsSameTag;
	for (int iPnt=0; iPnt<m_TINEdges.size(); iPnt++) {
		curPnt = m_Cloud.begin() + iPnt;
		if (!curPnt->HasAttribute(m_surfaceTag)) continue;
		vecNiebsSameTag = GetNeibPntWithSameTag(curPnt, m_surfaceTag);
		if (vecNiebsSameTag.size()==1) 
			stackUnRepairPnts.push(curPnt);
	}

	while (!stackUnRepairPnts.empty()) {
		curPnt = stackUnRepairPnts.top();
		stackUnRepairPnts.pop();
		vecUnProjPnts.push_back(curPnt);

		vecNiebsSameTag = GetNeibPntWithSameTag(curPnt, m_surfaceTag);
		curPnt->SetAttribute(m_surfaceTag, GetMaxFrequentNeibTag(curPnt, m_surfaceTag));
		
		//check next point
		if (!vecNiebsSameTag.empty()) {
			nextPnt = vecNiebsSameTag[0];
			vecNiebsSameTag = GetNeibPntWithSameTag(nextPnt, m_surfaceTag);
			if (vecNiebsSameTag.size()==1)
				stackUnRepairPnts.push(nextPnt);
		}
	}


	//////////////////////////////////////////////////////////////////////////
	//step 4
	//project unstable points to destination plane
	int indPlane;
	for (unsigned int i=0; i<vecUnProjPnts.size(); i++) {
		curPnt = vecUnProjPnts[i];
		indPlane = IndexPlaneBySegNumber(m_vecSurPlanes, curPnt->Attribute(m_surfaceTag));
		if (indPlane == -1) continue;
		plumbLine = Line3D(curPnt->Position3DRef(), Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(plumbLine, m_vecSurPlanes[indPlane], destPos);
		curPnt->Z() = destPos.Z();
	}

	
#ifdef _DEBUG
	LaserPoints debugPnts = m_Cloud;
	for (unsigned int i=0; i<debugPnts.size(); i++) {
		debugPnts[i].SetAttribute(ComponentNumberTag, debugPnts[i].Attribute(AveragePulseLengthTag));
	}
	debugPnts.Write("debug.laser");
#endif
}

#include <set>


//>=0: valid neighboring tag
//-1: no neighboring tag
//-2: all neighbours have same tag with this region
int RoofTopGraphSearcher::GetMaximumNeibRegionTag(const PointNumberList& curRegion, const std::vector<bool>& vecStabality)
{
	map<int, std::set<LASITR> > mapSegPnts;
	typedef map<int, std::set<LASITR> >::iterator ITRMAP;
	LASITR curPnt, nextPnt;
	int curRegNum = m_Cloud[curRegion[0].Number()].Attribute(m_surfaceTag);
	int nextRegNum;

	for (unsigned int i=0; i<curRegion.size(); i++) {
		TINEdgeSet& neibPnts = m_TINEdges[curRegion[i].Number()];
		for (int j=0; j<neibPnts.size(); j++) {
			nextPnt = m_Cloud.begin() + neibPnts[j].Number();

			//unstable points (from untagged small region)
			if (vecStabality[neibPnts[j].Number()] == false) continue;

			if (!nextPnt->HasAttribute(m_surfaceTag)) continue;
			nextRegNum = nextPnt->Attribute(m_surfaceTag);
			//if (nextRegNum == curRegNum) continue;

			ITRMAP itrMap = mapSegPnts.find(nextRegNum);
			if (itrMap!=mapSegPnts.end())
				itrMap->second.insert(nextPnt);
			else {
				std::set<LASITR> temSet;
				temSet.insert(nextPnt);
				mapSegPnts[nextRegNum] = temSet;
			}			
		}
	}

	ITRMAP maxReg;
	int maxNeib = 0;
	for (ITRMAP itrMap=mapSegPnts.begin(); itrMap!=mapSegPnts.end(); itrMap++) {
		if (itrMap->first == curRegNum) continue;
		if (maxNeib >= itrMap->second.size()) continue;
		maxNeib = itrMap->second.size();
		maxReg = itrMap;
	}

	//find valid neighboring points
	if (maxNeib>0) 
		return maxReg->first;
	//only with one neighboring region, which has same tag with current region
	//this happens when the neighboring points have changed their tags in the iteration process
	else if(mapSegPnts.size()==1 && mapSegPnts[curRegNum].size()>0)
		return -2;
	//invalid neighboring points
	else 
		return -1;
}

std::vector<RoofTopGraphSearcher::LASITR> 
	RoofTopGraphSearcher::GetNeibPntWithSameTag(LASITR inPnt, LaserPointTag tag)
{
	assert(inPnt->HasAttribute(tag));
	
	int value = inPnt->Attribute(tag);
	std::vector<LASITR> outNeibs;
	LASITR neibPnt;
	TINEdgeSet& neibEdges = m_TINEdges[inPnt-m_Cloud.begin()];

	for (unsigned int i=0; i<neibEdges.size(); i++) {
		neibPnt = m_Cloud.begin()+neibEdges[i].Number();
		if (neibPnt->HasAttribute(tag) && neibPnt->Attribute(tag)==value) 
			outNeibs.push_back(neibPnt);
	}

	return outNeibs;
}

int RoofTopGraphSearcher::GetMaxFrequentNeibTag(LASITR inPnt, LaserPointTag tag)
{
	assert(inPnt->HasAttribute(tag));

	std::vector<int> vecNeiTags;
	LASITR neibPnt;
	TINEdgeSet& neibEdges = m_TINEdges[inPnt-m_Cloud.begin()];

	for (unsigned int i=0; i<neibEdges.size(); i++) {
		neibPnt = m_Cloud.begin()+neibEdges[i].Number();
		if (neibPnt->HasAttribute(tag)) 
			vecNeiTags.push_back(neibPnt->Attribute(tag));
	}

	int maxFreValue = MaxFrequentValue(vecNeiTags);
	return maxFreValue;
}

RoofTopGraphSearcher::LASITR RoofTopGraphSearcher::FindNeastContPntOnSameSurface(RoofTopGraphSearcher::LASITR inPnt)
{
	int curPntNum = inPnt - m_Cloud.begin();
	int curSurfNum = inPnt->Attribute(m_surfaceTag);
	
	//input point is on contour
	if (m_pnlContour.end()!=find(m_pnlContour.begin(), m_pnlContour.end(), PointNumber(curPntNum)))
		return inPnt;
	
	double minDist = 99999999.9, temDist;
	LASITR temPnt, outPnt = m_Cloud.end();
	for (unsigned int i=0; i<m_pnlContour.size(); i++) {
		temPnt = m_Cloud.begin() + m_pnlContour[i].Number();
		if (temPnt->Attribute(m_surfaceTag) != curSurfNum) continue;//not on same surface
		temDist = outPnt->Distance(temPnt->Position3DRef());

		if (temDist<minDist) {
			minDist = temDist;
			outPnt = temPnt;
		}
	}

	//must find it!
	assert(outPnt != m_Cloud.end());
	return outPnt;
}

#include "LineSegments2D.h"

bool RoofTopGraphSearcher::DeriveOutlines()
{
	/*
	if (m_vecAncBouns.empty()) return true;
	ObjectPoints& contourObjPnt = m_contourObjPnts;
	LineTopology& contourTop = m_contourTop;
//	LineSegments2D	m_contourSegs = LineSegments2D(contourObjPnt, contourTop);
	for (unsigned int i=0; i<m_contourSegs.size(); i++)
		m_contourSegs[i].Label(i);

	//std::vector<AnchorBoundary*> m_vecAncBouns;
	std::vector<AnchorBoundary*>::iterator itrABound;
	AnchorPoint *aPnt0, *aPnt1;
	Position3D point0, point1;
	LineSegment2D curARidge;
	vector <double> scalars;
	Positions2D intPnts;
	Position2D  intPntCur;
	LineSegments2D::iterator curContSeg;
	vector<AnchorPoint*> vecAPntsOnCurRidge;
	typedef vector<AnchorPoint*> VECANCPNTS;
	std::map<int, VECANCPNTS> mapSegIndAncPnt;
	int segNum0, segNum1;
	std::map<int, VECANCPNTS>::iterator itrMapCur, itrMap0, itrMap1;
	Line3D ridge, plumb;
	//LineSegments2D allRidges;//one double-open ridge will be stored twice
	std::map<AnchorPoint*, AnchorBoundary*> mapAPntRidge;
	AnchorBoundary *aBound0, *aBound1;

	//////////////////////////////////////////////////////////////////////////
	//step 0
	//shrink all open ridges
	//return false;
	for (itrABound=m_vecAncBouns.begin(); itrABound!=m_vecAncBouns.end(); itrABound++) {
		if (IsClosedAncBound(*itrABound)) continue;//only tackle open anchor boundary
		if ((*itrABound)->HasChildren() || (*itrABound)->IsStepBound()) continue;

		aPnt0 = (*itrABound)->GetAttAncPnt(0);
		aPnt1 = (*itrABound)->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()==1) ShrinkRidge(aPnt1, aPnt0);
		if(aPnt1->AttBoundCount()==1) ShrinkRidge(aPnt0, aPnt1);
	}

	//return false;
	//////////////////////////////////////////////////////////////////////////
	//step 1
	//intersect all possible ridges with contour
	for (itrABound=m_vecAncBouns.begin(); itrABound!=m_vecAncBouns.end(); itrABound++) {
		if (IsClosedAncBound(*itrABound)) continue;//only tackle open anchor boundary
		if ((*itrABound)->HasChildren() || (*itrABound)->IsStepBound()) continue;
		GetAttachSegNums(**itrABound, segNum0, segNum1);

		aPnt0 = (*itrABound)->GetAttAncPnt(0);
		aPnt1 = (*itrABound)->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()>1 && aPnt1->AttBoundCount()>1) continue;//both end points are stable

		vecAPntsOnCurRidge.clear();
		if (aPnt0->AttBoundCount()==1) {//point 1 --> point 0
			vecAPntsOnCurRidge.push_back(aPnt1);
			vecAPntsOnCurRidge.push_back(aPnt0);
		}
		if(aPnt1->AttBoundCount()==1) {//point 0 --> point 1
			vecAPntsOnCurRidge.push_back(aPnt0);
			vecAPntsOnCurRidge.push_back(aPnt1);
		}
		
		//two directions
		for (int iAPnt=0; iAPnt<vecAPntsOnCurRidge.size()/2; iAPnt++) {
			aPnt0 = vecAPntsOnCurRidge[2*iAPnt+0];
			aPnt1 = vecAPntsOnCurRidge[2*iAPnt+1];
			intPnts.clear();
			scalars.clear();
			curARidge = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
			//intersect ridge with all contour segments
			for (curContSeg=m_contourSegs.begin(); curContSeg!=m_contourSegs.end(); curContSeg++) {
				double	scalar0, dist0;
				//get intersection point, if the two lines are parallel, take the projection point
				if(Angle2Lines(curContSeg->Line2DReference(), curARidge.Line2DReference()) > 0.10*PI/180 ) {
					Intersection2NonParallelLines(curContSeg->Line2DReference(), curARidge.Line2DReference(), intPntCur);
					scalar0 = curContSeg->Scalar(intPntCur);
					dist0 = curARidge.Scalar(intPntCur)-curARidge.ScalarEnd();
				}
				else {
					intPntCur = curContSeg->Project(aPnt1->GetPosition().vect2D());
					scalar0 = curContSeg->Scalar(intPntCur);
					dist0 = 1.5*intPntCur.Distance(aPnt1->GetPosition().vect2D());
				}

				//the intersection should be on the contour segment
				//and on the ray of ridge line
				if (scalar0>curContSeg->ScalarBegin() && scalar0<curContSeg->ScalarEnd() ){
					int aaaa = 00;
				}

				if (scalar0>=curContSeg->ScalarBegin() && scalar0<curContSeg->ScalarEnd() 
					&& dist0>-0.00001 ) {
						intPnts.push_back(intPntCur);
						scalars.push_back(dist0);
				}
				else {
					intPnts.push_back(Position2D(0.0,0.0));
					scalars.push_back(9999999.9);
				}
			}

			//get the nearest intersection point
			vector<double>::iterator itrMinScalar = std::min_element(scalars.begin(), scalars.end());
			int indIntersectingSeg = itrMinScalar-scalars.begin();
			intPntCur = intPnts[indIntersectingSeg];
			plumb = Line3D(Position3D(intPntCur.X(), intPntCur.Y(), 0.0), Vector3D(0.0, 0.0, 1.0));
			ridge = Line3D(Position3D(aPnt0->GetPosition()), Position3D(aPnt1->GetPosition()));
			MyIntersect2Lines(ridge, plumb, point1);
			aPnt1->SetPosition(point1);
			
			//map between anchor point and ridge segment
			curARidge = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
			mapAPntRidge.insert(make_pair(aPnt1, *itrABound));
		}
	}

	//return false;
	//////////////////////////////////////////////////////////////////////////
	//step 2
	//check self-crossing of inner boundaries
	std::map<AnchorPoint*, AnchorBoundary*>::iterator iItrMapAR, jItrMapAR;
	std::map<AnchorPoint*, vector<AnchorPoint*> > mapAPCross;
	std::map<AnchorPoint*, vector<AnchorPoint*> >::iterator itrMapAC0, itrMapAC1;
	LineSegment2D ridgeSeg0, ridgeSeg1;

	for (iItrMapAR=mapAPntRidge.begin(); iItrMapAR!=mapAPntRidge.end(); iItrMapAR++) 
		mapAPCross.insert(make_pair(iItrMapAR->first, vector<AnchorPoint*>()));

	for (iItrMapAR=mapAPntRidge.begin(); iItrMapAR!=mapAPntRidge.end(); iItrMapAR++) {
		aBound0 = iItrMapAR->second;
		//start anchor point for anchor boundary 0
		aPnt0 = iItrMapAR->first!=aBound0->GetAttAncPnt(0)?aBound0->GetAttAncPnt(0):aBound0->GetAttAncPnt(1);
		ridgeSeg0 = LineSegment2D(aPnt0->GetPosition().vect2D(), iItrMapAR->first->GetPosition().vect2D());
		
		for (++(jItrMapAR=iItrMapAR); jItrMapAR!=mapAPntRidge.end(); jItrMapAR++) {
			//start from same point
			aBound1 = jItrMapAR->second;
			aPnt1 = jItrMapAR->first!=aBound1->GetAttAncPnt(0)?aBound1->GetAttAncPnt(0):aBound1->GetAttAncPnt(1);
			//anchor points start from same point
			if (aPnt0==aPnt1 || aPnt0->GetPosition()==aPnt1->GetPosition()) continue;
			if (aBound0==aBound1) continue;//same anchor boundary

			//do not intersect
			ridgeSeg1 = LineSegment2D(aPnt1->GetPosition().vect2D(), jItrMapAR->first->GetPosition().vect2D());
			if(!MyIsCrossing(ridgeSeg0, ridgeSeg1)) continue;
			if(IsStoredCrosing(mapAPCross, iItrMapAR->first, jItrMapAR->first)) continue;

			//store two times
			//itrMapAC0 = mapAPCross.find(iItrMapAR->first);
			//itrMapAC0->second.push_back(jItrMapAR->first);
			//itrMapAC0 = mapAPCross.find(jItrMapAR->first);
			//itrMapAC0->second.push_back(iItrMapAR->first);
			mapAPCross[iItrMapAR->first].push_back(jItrMapAR->first);
			mapAPCross[jItrMapAR->first].push_back(iItrMapAR->first);
		}
	}
	//return false;

	//////////////////////////////////////////////////////////////////////////
	//setp 3
	//rank the crossing ridges, relocate the most unreliable one
	//make sure the first ridge is more stable
	for (itrMapAC0=mapAPCross.begin(); itrMapAC0!=mapAPCross.end(); itrMapAC0++) {
		if (itrMapAC0->second.size()==0) continue;
		aPnt0 = itrMapAC0->first;
		aBound0 = aPnt0->GetAttBoundary();
		//todo
		//the situation that several lines cross several ones is not treated

		//one to one crossing situation
		bool bFind = false;
		bool bHasFather = true;
		if (itrMapAC0->second.size()==1) {
			aPnt1 = itrMapAC0->second[0];
			aBound1 = aPnt1->GetAttBoundary();
			assert(aPnt0->AttBoundCount()==1 && aPnt1->AttBoundCount()==1);

			//current ridge crosses one ridge, however, that one crosses several ones
			//then current one is sure to be stable, skip it
			itrMapAC1 = mapAPCross.find(aPnt1);
			if (itrMapAC1->second.size()>1) continue;

			//aBound0 and aBound1 may both have no father
			//assert(aBound0->HasFather()!=aBound1->HasFather());
			if (aBound0->HasFather()!=aBound1->HasFather()) {
				if (aBound0->HasFather()) {//make sure the first ridge is more stable
					std::swap(aPnt0, aPnt1);
					std::swap(aBound0, aBound1);
				}			
			}
			else if(!aBound0->HasFather() && !aBound1->HasFather()) {
				bHasFather = false;
			}
			else {
				//should not reach here
				assert(true);
				continue;
			}
			
			bFind = true;

			//relocate unstable anchor point
			//Line2D line = Line2D(aPnt0->GetPosition(), aPnt1->GetPosition());
		//	Position2D begPos = Position2D(aPnt0->GetPosition().X(), aPnt0->GetPosition().Y());
		//	Position2D temPos = line.Position(line.Scalar(begPos)-0.01);
		//	aPnt1->SetPosition(Vector3D(temPos.X(), temPos.Y(), 0.0));

			//clean the map of anchor point and crossing ridges
		//	vector<AnchorPoint*>::iterator itrTem;
		//	itrTem = std::find(itrMapAC1->second.begin(), itrMapAC1->second.end(), itrMapAC0->first);
		//	if (itrTem!=itrMapAC1->second.end()) itrMapAC1->second.erase(itrTem);
		//	itrMapAC0->second.clear();
		}
		//one to several crossing situation
		else if(itrMapAC0->second.size()>1) {
			int segNum2, segNum3;
			GetAttachSegNums(*aBound0, segNum0, segNum1);

			//find the most stable ridge to intersect
			for (unsigned int i=0; i<itrMapAC0->second.size(); i++) {
				aPnt1 = itrMapAC0->second[i];
				aBound1 = aPnt1->GetAttBoundary();
				GetAttachSegNums(*aBound1, segNum2, segNum3);
				if (segNum0==segNum2 || segNum0==segNum3 ||
					segNum1==segNum2 || segNum1==segNum3) { 
					bFind = true;
					break;
				}
			}
			
			assert(bFind);
			if (bFind) {//make sure the first ridge is more stable
				std::swap(aPnt0, aPnt1);
				std::swap(aBound0, aBound1);
			}
		}

		if (bFind){
			//relocate unstable anchor point
			if (bHasFather) {
				Line2D line = Line2D(aPnt0->GetPosition(), aPnt1->GetPosition());
				Position2D begPos = Position2D(aPnt0->GetPosition().X(), aPnt0->GetPosition().Y());
				Position2D temPos = line.Position(line.Scalar(begPos)-0.1);
				aPnt1->SetPosition(Vector3D(temPos.X(), temPos.Y(), 0.00));

				//the new position maybe outside of contour
				aBound1 = aPnt1->GetAttBoundary();
				aPnt0 = aPnt1!=aBound1->GetAttAncPnt(0)? aBound1->GetAttAncPnt(0):aBound1->GetAttAncPnt(1);
				ShrinkRidge(aPnt0, aPnt1);
			}
			else {//Oct 23, 2014
				//if both anchor boundaries have no father, both of them are not stable
				//and we cannot find which end points are stable, and which end points should be adjusted
				//Now just pick the two end points close to the cross, and switch their positions
				AnchorPoint *aPnt00 = aPnt0->GetAttBoundary()->GetAttAncPnt(0);
				AnchorPoint *aPnt01 = aPnt0->GetAttBoundary()->GetAttAncPnt(1);
				AnchorPoint *aPnt10 = aPnt1->GetAttBoundary()->GetAttAncPnt(0);
				AnchorPoint *aPnt11 = aPnt1->GetAttBoundary()->GetAttAncPnt(1);
				LineSegment2D ridge0 = LineSegment2D(aPnt00->GetPosition().vect2D(), aPnt01->GetPosition().vect2D());
				LineSegment2D ridge1 = LineSegment2D(aPnt10->GetPosition().vect2D(), aPnt11->GetPosition().vect2D());
				Position2D cross;
				Vector3D temPos;
				if(Intersect2LineSegments2D(ridge0, ridge1, cross, 0.5)) {
					if (cross.Distance(aPnt00->GetPosition().vect2D())>cross.Distance(aPnt01->GetPosition().vect2D()))
						std::swap(aPnt00, aPnt01);// make sure aPnt00 is closer to the cross than aPnt01
					if (cross.Distance(aPnt10->GetPosition().vect2D())>cross.Distance(aPnt11->GetPosition().vect2D()))
						std::swap(aPnt10, aPnt11);// make sure aPnt10 is closer to the cross than aPnt11
					temPos = aPnt00->GetPosition();
					aPnt00->SetPosition(aPnt10->GetPosition());
					aPnt10->SetPosition(temPos);
				}
			}

			//clean the map of anchor point and crossing ridges
			vector<AnchorPoint*>::iterator itrTem;
			for (unsigned int i=0; i<itrMapAC0->second.size(); i++) {
				aPnt1 = itrMapAC0->second[i];
				itrMapAC1 = mapAPCross.find(aPnt1);
				itrTem = std::find(itrMapAC1->second.begin(), itrMapAC1->second.end(), itrMapAC0->first);
				if (itrTem!=itrMapAC1->second.end()) 
					itrMapAC1->second.erase(itrTem);
			}

			itrMapAC0->second.clear();
		}
	}

	//return false;
	//////////////////////////////////////////////////////////////////////////
	//setp 4
	//re-intersect all possible ridges with contour
	//the intersection with contour may be changed when relocating the unstable ridges
	for (itrABound=m_vecAncBouns.begin(); itrABound!=m_vecAncBouns.end(); itrABound++) {
		if (IsClosedAncBound(*itrABound)) continue;//only tackle open anchor boundary
		if ((*itrABound)->HasChildren() || (*itrABound)->IsStepBound()) continue;

		aPnt0 = (*itrABound)->GetAttAncPnt(0);
		aPnt1 = (*itrABound)->GetAttAncPnt(1);
		GetAttachSegNums(**itrABound, segNum0, segNum1);
		if(aPnt0->AttBoundCount()>1 && aPnt1->AttBoundCount()>1) continue;//both end points are stable
		if (aPnt0->GetPosition() == aPnt1->GetPosition()) continue;
		ridge = Line3D(Position3D(aPnt0->GetPosition()), Position3D(aPnt1->GetPosition()));

		vecAPntsOnCurRidge.clear();
		if (aPnt0->AttBoundCount()==1) {//point 1 --> point 0
			vecAPntsOnCurRidge.push_back(aPnt1);
			vecAPntsOnCurRidge.push_back(aPnt0);
		}
		if(aPnt1->AttBoundCount()==1) {//point 0 --> point 1
			vecAPntsOnCurRidge.push_back(aPnt0);
			vecAPntsOnCurRidge.push_back(aPnt1);
		}

		//two directions
		for (int iAPnt=0; iAPnt<vecAPntsOnCurRidge.size()/2; iAPnt++) {
			aPnt0 = vecAPntsOnCurRidge[2*iAPnt+0];
			aPnt1 = vecAPntsOnCurRidge[2*iAPnt+1];
			intPnts.clear();
			scalars.clear();
			curARidge = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
			//intersect ridge with all contour segments
			for (curContSeg=m_contourSegs.begin(); curContSeg!=m_contourSegs.end(); curContSeg++) {
	//			if(curContSeg->Intersect(curARidge, intPntCur, 0.0)) {
	//				intPnts.push_back(intPntCur);
	//				scalars.push_back(fabs(curARidge.Scalar(intPntCur)-curARidge.ScalarBegin()));
	//			}
	//			else {
	//				intPnts.push_back(Position2D(0.0,0.0));
	//				scalars.push_back(9999999.9);
	//			}

				double	scalar0 = curContSeg->ScalarBegin()-1.0, dist0=9999999.9;
				//get intersection point, if the two lines are parallel, take the projection point
				if(Angle2Lines(curContSeg->Line2DReference(), curARidge.Line2DReference()) > 0.10*PI/180 ) {
					Intersection2NonParallelLines(curContSeg->Line2DReference(), curARidge.Line2DReference(), intPntCur);
					scalar0 = curContSeg->Scalar(intPntCur);
					dist0 = fabs(curARidge.Scalar(intPntCur)-curARidge.ScalarEnd());
				}

				if (scalar0>=curContSeg->ScalarBegin()&&scalar0<curContSeg->ScalarEnd()) {
					intPnts.push_back(intPntCur);
					scalars.push_back(dist0);
				}
				else {
					intPnts.push_back(Position2D(0.0,0.0));
					scalars.push_back(9999999.9);
				}
			}

			//get the nearest intersection point
			vector<double>::iterator itrMinScalar = std::min_element(scalars.begin(), scalars.end());
			int indIntersectingSeg = itrMinScalar-scalars.begin();
			intPntCur = intPnts[indIntersectingSeg];
			plumb = Line3D(Position3D(intPntCur.X(), intPntCur.Y(), 0.0), Vector3D(0.0, 0.0, 1.0));
			MyIntersect2Lines(ridge, plumb, point0);
			aPnt1->SetPosition(point0);

			//map between anchor point and contour line index
			itrMapCur = mapSegIndAncPnt.find(indIntersectingSeg);
			if (itrMapCur==mapSegIndAncPnt.end())
				mapSegIndAncPnt.insert(make_pair(indIntersectingSeg, VECANCPNTS(1,aPnt1)));
			else
				itrMapCur->second.push_back(aPnt1);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//setp 5
	//sort intersecting points on a same contour segment
	VECANCPNTS temVecAncPntS;
	Line3D line;
	//double scale0, scale1, scaleBeg;
	double dist0, dist1;
	for (itrMapCur=mapSegIndAncPnt.begin(); itrMapCur!=mapSegIndAncPnt.end(); itrMapCur++) {
		if(itrMapCur->second.size()==1) continue;
		point0 = contourObjPnt[contourTop[itrMapCur->first+0].Number()];
		point1 = contourObjPnt[contourTop[itrMapCur->first+1].Number()];
		//line = Line3D(point0, point1);
		//scaleBeg = line.Scalar(point0);
		temVecAncPntS = itrMapCur->second;
		
		for (unsigned int i=0; i<temVecAncPntS.size(); i++) {
			for (int j=i+1; j<temVecAncPntS.size(); j++) {
				//scale0 = line.Scalar(temVecAncPntS[i]->GetPosition())-scaleBeg;
				//scale1 = line.Scalar(temVecAncPntS[j]->GetPosition())-scaleBeg;
				//assert(scale0>=0 || scale1>=0);
				//if (point0.Distance(temVecAncPntS[i]->GetPosition())>point0.Distance(temVecAncPntS[j]->GetPosition()))
				//if (scale0 > scale1)//first anchor point should nearer to point0 than second anchor point
				//	std::swap(temVecAncPntS[i], temVecAncPntS[j]);
				dist0 = (point0-temVecAncPntS[i]->GetPosition()).Length2D();
				dist1 = (point0-temVecAncPntS[j]->GetPosition()).Length2D();
				if (dist0 > dist1)//first anchor point should nearer to point0 than second anchor point
					std::swap(temVecAncPntS[i], temVecAncPntS[j]);
			}
		}
		itrMapCur->second = temVecAncPntS;
	}

	VECANCPNTS vecBreakAPnts;
	vector<int> vecSegInds;
	for (itrMapCur=mapSegIndAncPnt.begin(); itrMapCur!=mapSegIndAncPnt.end(); itrMapCur++) {
		for (unsigned int i=0; i<itrMapCur->second.size(); i++) {
			vecBreakAPnts.push_back(itrMapCur->second[i]);
			vecSegInds.push_back(itrMapCur->first);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//setp 6
	//construct a top line for each two breaking anchor points
	m_objPntOutLines = contourObjPnt;
	m_lineTopOutLines.clear();
	LineTopology temBound;
	ObjectPoint temObjPnt;
	int ind0, ind1;

	for (unsigned int i=0; i<vecBreakAPnts.size(); i++) {
		ind0 = i;
		ind1 = (i!=vecBreakAPnts.size()-1) ? i+1:0;
		aPnt0 = vecBreakAPnts[ind0];
		aPnt1 = vecBreakAPnts[ind1];

		int surfaceNum = GetAttachingSurfaceSegNum(aPnt0, aPnt1);

		//point 0
		temObjPnt.Position3DRef() = aPnt0->GetPosition();
		temObjPnt.Number() = m_objPntOutLines.size();
		m_objPntOutLines.push_back(temObjPnt);

		//point 1
		temObjPnt.Position3DRef() = aPnt1->GetPosition();
		temObjPnt.Number() = m_objPntOutLines.size();
		m_objPntOutLines.push_back(temObjPnt);

		//line top
		temBound.clear();
		temBound.SetAttribute(SegmentLabel, -1);
		temBound.SetAttribute(RightPlaneTag, surfaceNum);
		
		
		//the situation that vecSegInds[ind0] == vecSegInds[ind1] is skipped todo
		//point 0 --> point 1
		if (vecSegInds[ind0] < vecSegInds[ind1]) {
			for (int iOPnt=vecSegInds[ind0]+1; iOPnt < vecSegInds[ind1]+1; iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
		}
		//point 0 --> 0 --> point 1
		else if(vecSegInds[ind0] > vecSegInds[ind1]) {
			for (int iOPnt=vecSegInds[ind0]+1; iOPnt<contourTop.size(); iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
			for (int iOPnt=1; iOPnt < vecSegInds[ind1]+1; iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
		}
		
		//no surface is found or more than 1 surfaces are found
		if (surfaceNum<0 && !temBound.empty()) {
			surfaceNum = GetAttachingSurfaceSegNum(m_objPntOutLines, temBound, aPnt0, aPnt1);
			temBound.SetAttribute(RightPlaneTag, surfaceNum);
		}
		
		//add start and end point
		temBound.insert(temBound.begin(), PointNumber(m_objPntOutLines.size()-2));
		temBound.push_back(PointNumber(m_objPntOutLines.size()-1));
		temBound.RemoveCollinearNodes(m_objPntOutLines, false);

		m_lineTopOutLines.push_back(temBound);
	}

	//step 7
	//no break point, export the contour
	if (vecBreakAPnts.empty()) {
		m_objPntOutLines.clear();
		ExportContour(m_objPntOutLines, m_lineTopOutLines);
	}
	*/
	return true;
}


bool RoofTopGraphSearcher::Project2DLineToPlane()
{
	Vector3D plumbDir(0.0, 0.0, 1.0);
	Line3D plumbLine;
	int surfSegNum;
	Plane destPlane;
	int indDestPlane;
	Position3D crossPos, temPos;

	//////////////////////////////////////////////////////////////////////////
	//outlines
	for (LineTopologies::iterator top=m_lineTopOutLines.begin(); top!=m_lineTopOutLines.end(); top++) {
		surfSegNum = top->Attribute(SegmentLabel);
		if (surfSegNum==-1) surfSegNum = top->Attribute(SecondSegmentLabel);

		if (surfSegNum==2) {
			int aaa = 0;
		}

		indDestPlane = IndexPlaneBySegNumber(m_vecSurPlanes, surfSegNum);
		if (indDestPlane==-1) continue;
		destPlane = (m_vecSurPlanes)[indDestPlane];

		for (int iPos=0; iPos<top->size(); iPos++) {
			ObjectPoint& curObjPnt = m_objPntOutLines[(*top)[iPos].Number()];
			if(fabs(curObjPnt.X()-258898.90)<0.01 && fabs(fabs(curObjPnt.Y()-258898.90))<0.01) {
				int aaa = 0;
			}

			plumbLine = Line3D(curObjPnt.Position3DRef(), plumbDir);
			if(!IntersectLine3DPlane(plumbLine, destPlane, crossPos)) continue;
			curObjPnt.Z() = crossPos.Z();
		}
	}


	return true;
}



bool RoofTopGraphSearcher::OutPutPolygons(ObjectPoints& destPolObjPnt, LineTopologies& destPolTops, bool onlyInner)
{
	/*
	//////////////////////////////////////////////////////////////////////////
	//anchor ridges
	destPolObjPnt.clear();
	destPolTops.clear();
	int segNum1, segNum2;
	AnchorBoundary* curBound;

	//////////////////////////////////////////////////////////////////////////
	//other ridges
	for (int i=0; i<m_vecAncBouns.size(); i++) {
		curBound = m_vecAncBouns[i];
		assert(curBound);
		if (curBound->HasChildren() || curBound->IsStepBound()) continue;

		AnchorPoint* aPnt0 = curBound->GetAttAncPnt(0);
		AnchorPoint* aPnt1 = curBound->GetAttAncPnt(1);
		GetAttachSegNums(*curBound, segNum1, segNum2);
		AddTopLine(destPolObjPnt, destPolTops, aPnt0->GetPosition(), aPnt1->GetPosition(), segNum1, segNum2);
	}

			
#ifdef _DEBUG
		if (segNum1==58||segNum2==58) {
			LineTopologies debugTops;
			debugTops.push_back(temTop);
			debugTops.Write("debug.top");
			destPolObjPnt.Write("debug.objpts");
		}
#endif

	//////////////////////////////////////////////////////////////////////////
	//outlines
	for (LineTopologies::iterator itrTop=m_lineTopOutLines.begin(); itrTop!=m_lineTopOutLines.end(); itrTop++) {
		segNum1 = itrTop->Attribute(SegmentLabel);
		segNum2 = itrTop->Attribute(SecondSegmentLabel);

		for (int iPos=0; iPos<itrTop->size()-1; iPos++) {
			AddTopLine(destPolObjPnt, destPolTops, m_objPntOutLines[(*itrTop)[iPos].Number()],
				m_objPntOutLines[(*itrTop)[iPos+1].Number()], segNum1, segNum2);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//only export contour when no inner ridges
	if (m_lineTopOutLines.empty() && !onlyInner)
		ExportContour(destPolObjPnt, destPolTops);
		*/
	return true;
}
