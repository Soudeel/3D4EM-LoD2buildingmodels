/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 12-03-2014
Description: 
Revise:
----------------------------------------------------------*/

#include <assert.h>
#include "AnchorPoints.h"
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
//#include "PolyhendronRepair/PolyhedronRepair.h"
#include "PolygonFunctions.h"

#include <algorithm>
#include <vector>
#include <stack>
#include <queue>
#include <set>
#include <map>

using namespace std;


#define CW(ind) ((ind+1)%3)
#define CCW(ind) ((ind+2)%3) //3+ind-1
#define Sign(x) (x>=0?1:-1)

#define AncTagNone -1
#define AncTagBound -2
#define AncTagPnt -3
#define AncTagContour -4

//bool ShouldKeepOldBoundary(const std::vector<PolyhedronRepair::CornerPoint>& newCrns, AnchorBoundary* oldBound);

PointNumberList GetAdjPnts(const TINEdges& edges, PointNumber v1, PointNumber v2)
{
	PointNumberList rst;
	if (v1.Number()<0||v1.Number()>edges.size()||v2.Number()<0||v2.Number()>edges.size())
		return rst;

	TINEdges::const_iterator itrSet1, itrSet2, itrSet3;
	itrSet1 = edges.begin() + v1.Number();
	itrSet2 = edges.begin() + v2.Number();
	if (itrSet1->end() == std::find(itrSet1->begin(), itrSet1->end(), v2))
		return rst;//v1 & v2 are not on one edge

	for (int i=0; i<itrSet1->size(); i++) {
		const PointNumber& v3 = (*itrSet1)[i];
		if (v3 == v2) continue;
		//v2 is adjacent to v3
		if (itrSet2->end() != std::find(itrSet2->begin(), itrSet2->end(), v3))
			rst.push_back(v3);
	}

	return rst;
}

AnchorPoint::AnchorPoint(LaserPointTag surfaceTag)
{
	m_surfaceTag = surfaceTag;
}

AnchorPoint::AnchorPoint(LASITR pnt1, LASITR pnt2,LaserPointTag surfaceTag)
{
	Initialize();
	m_surfaceTag = surfaceTag;
	AttachePoint(pnt1, NULL);
	AttachePoint(pnt2, NULL);
	ComputePosition();
}

AnchorPoint::AnchorPoint(LASITR pnt1, LASITR pnt2, LASITR pnt3,LaserPointTag surfaceTag)
{
	Initialize();
	m_surfaceTag = surfaceTag;
	AttachePoint(pnt1, NULL);
	AttachePoint(pnt2, NULL);
	AttachePoint(pnt3, NULL);
	ComputePosition();
}

AnchorPoint::AnchorPoint(LASITR pnt1, Plane* plane1, LASITR pnt2, Plane* plane2,LaserPointTag surfaceTag)
{
	Initialize();
	m_surfaceTag = surfaceTag;
	AttachePoint(pnt1, plane1);
	AttachePoint(pnt2, plane2);
	ComputePosition();
}

AnchorPoint::AnchorPoint(LASITR pnt1, Plane* plane1, LASITR pnt2, Plane* plane2, LASITR pnt3, Plane* plane3,LaserPointTag surfaceTag)
{
	Initialize();
	m_surfaceTag = surfaceTag;
	AttachePoint(pnt1, plane1);
	AttachePoint(pnt2, plane2);
	AttachePoint(pnt3, plane3);
	ComputePosition();
}

AnchorPoint::~AnchorPoint()
{

}

void AnchorPoint::Initialize()
{
	m_attBonds[0] = NULL;
	m_attBonds[1] = NULL;
	m_attBonds[2] = NULL;
	m_bCent = false;
}

AnchorPoint& AnchorPoint::operator=(const AnchorPoint& right)
{
	if (this == &right) return *this;

	m_pos = right.m_pos;
	m_attPnts = right.m_attPnts;
	m_bCent = right.m_bCent;
	return *this;
}

IEDll_API bool operator==(const AnchorPoint& left, const AnchorPoint& right)
{
	for (int i=0; i<right.m_attPnts.size(); i++) {
		//cannot find
		if(left.m_attPnts.end() == std::find(left.m_attPnts.begin(),left.m_attPnts.end(),right.m_attPnts[i]))
			return false;
	}
	return true;
}


void AnchorPoint::AttachePoint(LASITR pnt, Plane* plane)
{
	//assert(pnt);
	m_attPnts.push_back(PntPlanePair(pnt, plane));
}

void AnchorPoint::ChangeAttPnt(LASITR orgPnt, LASITR dstPnt, Plane* dstPlane)
{
	int ind = GetAttPntInd(orgPnt);
	if (ind!=-1) {
		m_attPnts[ind].first = dstPnt;
		m_attPnts[ind].second = dstPlane;
	}
}

void AnchorPoint::ComputePosition()
{
	//need 2 or 3 points attached
	if (m_attPnts.size()!=2 && m_attPnts.size()!=3) return;

	//check has plane attached
	bool hasPlane = true;
	for (int i=0; i<m_attPnts.size(); i++) {
		if (!m_attPnts[i].second) {
			hasPlane = false;
			break;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	if (hasPlane) {
		if (m_attPnts.size()==2) 
			ComputeCentOf2Pnts();
		else
			ComputeCentOf3Pnts();
	}
	else {//just use gravity points
		m_pos = Vector3D(0.0, 0.0, 0.0);
		for (int i=0; i<m_attPnts.size(); i++) 
			m_pos += Vector3D(*m_attPnts[i].first);
		m_pos /=m_attPnts.size();
		m_bCent = true;
	}
}

//project the points onto the intersection of the two planes
//and average the two projected points
bool AnchorPoint::ComputeCentOf2Pnts()
{
	double bufferSize = 3.0;
	Plane wall;
	Line3D line;
	m_pos = Vector3D(0.0, 0.0, 0.0);
	Position3D p2, p3;
	LineSegment2D bufferLineSeg;
	Vector3D vec01;
	bool b2D = true;
	Position3D cross;
	Vector3D ridgeDir;

	int segNum0 = m_attPnts[0].second->Number();
	int segNum1 = m_attPnts[1].second->Number();
	if ((segNum0==86&&segNum1==490) || (segNum0==490&&segNum1==86)) 
	{
		int aaa = 0;
	}

	if(MyIsParaller(m_attPnts[0].second->Normal(), m_attPnts[1].second->Normal(), 10*3.1415926/180))
		{
	m_bCent = true;
	m_pos = Vector3D(*m_attPnts[0].first) + Vector3D(*m_attPnts[1].first);
	m_pos /= 2.0;

	return true;
	}

	Intersect2Planes(*m_attPnts[0].second, *m_attPnts[1].second, line);
	ridgeDir = line.Direction();
	if (ridgeDir.Z()<0.0) ridgeDir *= -1.0;
	double zenith = MyAngle(ridgeDir,Vector3D(0.0, 0.0, 1.0));
	bufferSize = line.IsHorizontal(10*3.1415926/180)? 8.0 : 4.0;
	
	const Position3D& p0 = m_attPnts[0].first->Position3DRef();
	const Position3D& p1 = m_attPnts[1].first->Position3DRef();
	vec01 = Vector3D(p1-p0);
	vec01 = vec01.Normalize();
	wall = Plane(p0, vec01.VectorProduct(Vector3D(0.0, 0.0, 1.0)));

	if (MyIsParaller(wall, line, 10*3.1415926/180)) 
		{
	m_bCent = true;
	m_pos = Vector3D(*m_attPnts[0].first) + Vector3D(*m_attPnts[1].first);
	m_pos /= 2.0;

	return true;
	}

	//p2= (a+b)/2 + alpha(b-a)/2
	//p3 = (a+b)/2 + alpha(a-b)/2
	
	p2 = 0.5*((1-bufferSize)*p0 + (1+bufferSize)*p1);
	p3 = 0.5*((1+bufferSize)*p0 + (1-bufferSize)*p1);
	bufferLineSeg = LineSegment2D(p2.Position2DOnly(), p3.Position2DOnly());

	IntersectLine3DPlane(line, wall, cross);
	if (bufferLineSeg.DistanceToPoint(cross.Position2DOnly())>0.0001)
		{
	m_bCent = true;
	m_pos = Vector3D(*m_attPnts[0].first) + Vector3D(*m_attPnts[1].first);
	m_pos /= 2.0;

	return true;
	}

	m_pos = cross;
	m_bCent = false;
	return true;
}


bool AnchorPoint::ComputeCentOf3Pnts()
{
	double distThr = 1.0;
	Line3D line0, line1, line2;
	Vector3D cent01, cent02, cent12;
	Vector3D cross01, cross02, cross12;
	Vector3D proj01, proj02, proj12;
	bool bProj01=false, bProj02=false, bProj12=false;
	double dif01, dif02, dif12;//difference between cent point and projection point

	cent01 = (*m_attPnts[0].first+*m_attPnts[1].first)/2.0;
	if(Intersect2Planes(*m_attPnts[0].second, *m_attPnts[1].second, line2)) {//projection
		proj01  = (line2.Project(*m_attPnts[0].first)+line2.Project(*m_attPnts[1].first))/2;
		dif01 = (proj01-cent01).Length();
		if (dif01<distThr) bProj01 = true;
	}


	cent02 = (*m_attPnts[0].first+*m_attPnts[2].first)/2.0;
	if(Intersect2Planes(*m_attPnts[0].second, *m_attPnts[2].second, line1)) {//projection
		proj02  = (line1.Project(*m_attPnts[0].first)+line1.Project(*m_attPnts[2].first))/2;
		dif02 = (proj02-cent02).Length();
		if (dif02<distThr) bProj02 = true;
	}


	cent12 = (*m_attPnts[1].first+*m_attPnts[2].first)/2.0;
	if(Intersect2Planes(*m_attPnts[1].second, *m_attPnts[2].second, line0)) {//projection
		proj12  = (line0.Project(*m_attPnts[1].first)+line0.Project(*m_attPnts[2].first))/2;
		dif12 = (proj12-cent12).Length();
		if (dif12<distThr) bProj12 = true;
	}

	m_bCent = false;
	if (bProj01 && bProj02 && bProj12) {
	
		if (MyIsParaller(line0.Direction(), line1.Direction(), 15*3.1415926/320) &&
			MyIsParaller(line0.Direction(), line2.Direction(), 15*3.1415926/320) &&
			MyIsParaller(line1.Direction(), line2.Direction(), 15*3.1415926/320)) {
				m_pos = (*m_attPnts[0].first+*m_attPnts[1].first+*m_attPnts[2].first)/3.0;
		}	
		else {
			MyIntersect2Lines(line0, line1, cross01);
			MyIntersect2Lines(line0, line2, cross02);
			MyIntersect2Lines(line1, line2, cross12);
			m_pos = (cross01+cross02+cross12)/3.0;
			m_bCent = true;
		}		
	}
	else if (bProj01 && dif01<dif02 && dif01<dif12)//projection point 01
		m_pos  = 2/3.0*proj01 + 1/3.0*line2.Project(*m_attPnts[2].first);
	else if (bProj02 && dif02<dif01 && dif02<dif12)//projection point 02
		m_pos  = 2/3.0*proj02 + 1/3.0*line1.Project(*m_attPnts[1].first);
	else if (bProj12 && dif12<dif01 && dif12<dif02)//projection point 12
		m_pos  = 2/3.0*proj12 + 1/3.0*line0.Project(*m_attPnts[0].first);
	else//cent point
		m_pos = (*m_attPnts[0].first+*m_attPnts[1].first+*m_attPnts[2].first)/3.0;

	return true;
}


AnchorPoint::LASITR AnchorPoint::GetAttPntBySegNum(int surTagNum)
{
	for (vector<PntPlanePair>::iterator itr=m_attPnts.begin(); itr!=m_attPnts.end(); itr++) {
		if (itr->first->HasAttribute(m_surfaceTag) && itr->first->Attribute(m_surfaceTag)==surTagNum) {
			return itr->first;
		}
	}
	return AnchorPoint::LASITR();
}

AnchorPoint::LASITR AnchorPoint::GetAttPnt(int ind)
{
	assert(ind>=0 && ind<AttPntCount());
	return m_attPnts[ind].first;
}

Plane* AnchorPoint::GetAttPlaneBySegNum(int surTagNum)
{
	for (vector<PntPlanePair>::iterator itr=m_attPnts.begin(); itr!=m_attPnts.end(); itr++) {
		if (itr->first->HasAttribute(m_surfaceTag) && itr->first->Attribute(m_surfaceTag)==surTagNum) {
			return itr->second;
		}
	}
	return NULL;
}

Plane* AnchorPoint::GetAttPlane(int ind)
{
	assert(ind>=0 && ind<AttPntCount());
	return m_attPnts[ind].second;
}

int AnchorPoint::GetAncBoundInd(int vInd1, int vInd2)
{
	assert(vInd1!=vInd2);
	assert(vInd1>=0 && vInd1<=2);
	assert(vInd2>=0 && vInd2<=2);

	return 3-vInd1-vInd2;//sum of ind1, ind2, ind3 is 3
}

int AnchorPoint::GetAncBoundInd(const AnchorBoundary* pAncBnd)
{
	int ind = -1;
	for (int i=0; i<3; i++)	{
		if (pAncBnd == m_attBonds[i]) {
			ind = i;
			break;
		}
	}

	return ind;
}

int AnchorPoint::GetAncBoundInd(LASITR lasPnt0, LASITR lasPnt1)
{
	int indPnt0 = GetAttPntInd(lasPnt0);
	int indPnt1 = GetAttPntInd(lasPnt1);
	if (indPnt0==-1 || indPnt1==-1) return -1;

	return GetAncBoundInd(indPnt0, indPnt1);
}

void AnchorPoint::AttachBoundary(int indBound, AnchorBoundary* attBound)
{
	if (!attBound) return;
	assert(indBound>=0 && indBound<=2);
	m_attBonds[indBound] = attBound;
}

AnchorBoundary* AnchorPoint::GetAttBoundary(int indBound) 
{
	assert(indBound>=0 && indBound<=2);
	return m_attBonds[indBound];
}

//get the first valid boundary
AnchorBoundary* AnchorPoint::GetAttBoundary()
{
	if (m_attBonds[0]) return m_attBonds[0];
	else if (m_attBonds[1]) return m_attBonds[1];
	else return m_attBonds[2];
}

int AnchorPoint::AttBoundCount()
{
	int count = 0;
	if(m_attBonds[0] != NULL)  count++;
	if(m_attBonds[1] != NULL)  count++;
	if(m_attBonds[2] != NULL)  count++;
	return count;
}

int AnchorPoint::GetAttPntInd(AnchorPoint::LASITR lasPnt)
{
	int ind = -1;
	for (int i=0; i<m_attPnts.size(); i++) {
		if(m_attPnts[i].first == lasPnt) {
			ind = i;
			break;
		}
	}

	return ind;
}


std::vector<std::pair<AnchorPoint::LASITR, AnchorPoint::LASITR> > AnchorPoint::GetEmptyAnchorEdges()
{
	std::vector<std::pair<LASITR, LASITR> > rst;
	LASITR lasPnt0, lasPnt1;
	if (AttPntCount()==2 || AttBoundCount()==3) return rst;

	for (int i=0; i<3; i++) {
		if (m_attBonds[i]==NULL) {
			lasPnt0 = m_attPnts[CW(i)].first;
			lasPnt1 = m_attPnts[CCW(i)].first;
			if (lasPnt0>lasPnt1) std::swap(lasPnt0, lasPnt1);

			rst.push_back(std::make_pair(lasPnt0, lasPnt1));
		}
	}

	return rst;
}



AnchorBoundary::AnchorBoundary()
{
	m_APnts[0] = NULL;
	m_APnts[1] = NULL;
	m_pFather = NULL;
	m_bStepBound = false;
}

AnchorBoundary::~AnchorBoundary()
{
	m_vecChildren.clear();
}

void AnchorBoundary::Initialize()
{
	m_APnts[0] = NULL;
	m_APnts[1] = NULL;
	m_pFather = NULL;
	m_bStepBound = false;
}

void AnchorBoundary::AttachAncPnt(int ind, AnchorPoint* ancPnt)
{
	if (!ancPnt) return;
	assert(ind>=0 && ind<=2);
	m_APnts[ind] = ancPnt;
}

AnchorPoint* AnchorBoundary::GetAttAncPnt(int ind)
{
	assert(ind>=0 && ind<=2);
	return m_APnts[ind];
}

void AnchorBoundary::AddChild(AnchorBoundary* pChild) 
{
	assert(pChild);
	pChild->SetFather(this);

	if (std::find(m_vecChildren.begin(), m_vecChildren.end(), pChild)==m_vecChildren.end())
		m_vecChildren.push_back(pChild);
}

bool AnchorBoundary::GetAttachSegNums(int& segNum0, int& segNum1) const
{
	if (!m_APnts[0]) return false;
	LASITR p0, p1;

	//only two attach points
	if(m_APnts[0]->AttPntCount()==2) {
		p0 = m_APnts[0]->GetAttPnt(0);
		p1 = m_APnts[0]->GetAttPnt(1);
	} else {//three attach points
		int indABound = m_APnts[0]->GetAncBoundInd(this);
		if (indABound==-1) return false;
		p0 = m_APnts[0]->GetAttPnt(CW(indABound));
		p1 = m_APnts[0]->GetAttPnt(CCW(indABound));
	}
	
	segNum0 = p0->Attribute(m_APnts[0]->GetSurfaceTag());
	segNum1 = p1->Attribute(m_APnts[0]->GetSurfaceTag());

	return true;
}

bool AnchorBoundary::GetRidgeLine(Line3D& ridge)
{
	if (!m_APnts[0] ) return false;
	Plane* plane0, *plane1;

	//only two attach points
	if(m_APnts[0]->AttPntCount()==2) {
		plane0 = m_APnts[0]->GetAttPlane(0);
		plane1 = m_APnts[0]->GetAttPlane(1);
	} else {//three attach points
		int indABound = m_APnts[0]->GetAncBoundInd(this);
		if (indABound==-1) return false;
		plane0 = m_APnts[0]->GetAttPlane(CW(indABound));
		plane1 = m_APnts[0]->GetAttPlane(CCW(indABound));
	}
	
	Line3D line;
	if(MyIsParaller(plane0->Normal(), plane1->Normal(), 10*3.1415926/180)) return false;
	if(!Intersect2Planes(*plane0, *plane1, line)) return false;
	
	ridge = line;
	return true;
}

AnchorEDGE::AnchorEDGE (int num1, int num2)
{
	assert(num1!=num2);
	pntNum0 = num1;
	pntNum1 = num2;
	if (pntNum0 > pntNum1) std::swap(pntNum0, pntNum1);
}

AnchorEDGE& AnchorEDGE::operator = (const AnchorEDGE& right)
{
	if (this == &right) return *this;
	this->pntNum0 = right.pntNum0;
	this->pntNum1 = right.pntNum1;
	return *this;
}

IEDll_API bool operator==(const AnchorEDGE& left, const AnchorEDGE& right)
{
	bool bE = (left.pntNum0==right.pntNum0 && left.pntNum1==right.pntNum1);
	return left.pntNum0==right.pntNum0 && left.pntNum1==right.pntNum1;
}

IEDll_API bool operator!=(const AnchorEDGE& left, const AnchorEDGE& right)
{
	return !(left==right);
}

IEDll_API bool operator<(const AnchorEDGE& left, const AnchorEDGE& right)
{
	if (left.pntNum0<right.pntNum0) {
		return true;
	}
	else if (left.pntNum0 == right.pntNum0) {
		return (left.pntNum1<right.pntNum1);
	}
	else {
		return false;
	}
}

AnchorPointSearcher::AnchorPointSearcher(
	const LaserPoints& cloud, const std::vector<Plane>& vecSurPlanes, 
	const ObjectPoints& pContourObjPnt, const LineTopology& pContourTop,
	LaserPointTag surfaceTag, LaserPointTag anchorTag)
{
	m_surfaceTag = surfaceTag;
	m_anchorTag = anchorTag;
	m_Cloud = cloud;
	m_vecSurPlanes = vecSurPlanes;

	//if (!m_pTin) m_pTin = m_pCloud->DeriveTIN();
	//assert(m_pTin);
	
	m_contourObjPnts = pContourObjPnt;
	m_contourTop = pContourTop;


}


bool AnchorPointSearcher::CleanAllData()
{
	m_Cloud.clear();
	m_Tin.clear();
	m_TINEdges.clear();
	m_contourObjPnts.clear();
	m_contourTop.clear();
	m_pnlContour.clear();

	return true;
}

bool AnchorPointSearcher::Initialize()
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
	m_vecAncPnts.reserve(1000);
	SurfaceGrowContourPnts();
	RepairSegmentTag();
	MyDeriveSegPNL(m_Cloud, m_pnlSegs, m_surfaceTag);

	return true;
}

AnchorPointSearcher::~AnchorPointSearcher()
{
	for (int i=0; i<m_vecAncPnts.size(); ++i) {
		if (m_vecAncPnts[i] != NULL)
			delete m_vecAncPnts[i];
	}

}

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include "PolygonFunctions.h"

bool AnchorPointSearcher::CombineLasAndContour()
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


	
	//#ifdef _DEBUG
	LineTopologies debugTops1;
	debugTops1.push_back(m_contourTop);
	debugTops1.Write("debug.top");
	m_contourObjPnts.Write("debug.objpts");
	m_Cloud.Write("debug.laser", false, false);
	//#endif
	

	std::vector<bool> vecValid = std::vector<bool> (m_Cloud.size(), true);

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

	
//#ifdef _DEBUG
	LineTopologies debugTops;
	debugTops.clear();
	debugTops.push_back(m_contourTop);
	debugTops.Write("debug.top");
	m_contourObjPnts.Write("debug.objpts");
	m_Cloud.Write("debug.laser", false, false);
//#endif
	
	//TIN tin = localBndPnts.Triangulate(localBndTops, holeCents);
	return true;
}

bool AnchorPointSearcher::MakeTin()
{
	if(m_contourTop.empty()) {
		m_Tin = TIN();
		return false;
	}
	
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

bool AnchorPointSearcher::IsOnContour(LASITR pnt)
{
	assert(pnt!=m_Cloud.end());
	return IsOnContour(pnt-m_Cloud.begin());
}

bool AnchorPointSearcher::IsOnContour(int pntNum)
{
	assert(pntNum>=0 && pntNum<m_Cloud.size());
	return m_pnlContour.end()!=find(m_pnlContour.begin(), m_pnlContour.end(), pntNum);
}

bool AnchorPointSearcher::DeriveAnchorPoints()
{
	LASITR pnt0, pnt1, pnt2;
	//vector<AnchorPoint> vecAncPnts;
	int indP0, indP1, indP2;
	int surNum0, surNum1, surNum2;
	AnchorPoint* newAPnt = NULL;

	//check faces with node from 3 difference surfaces/segments
	for (int f=0; f<m_Tin.size(); f++) {
		pnt0 = m_Cloud.begin() + m_Tin[f].Nodes()[0].Number();
		pnt1 = m_Cloud.begin() + m_Tin[f].Nodes()[1].Number();
		pnt2 = m_Cloud.begin() + m_Tin[f].Nodes()[2].Number();
		if (!pnt0->HasAttribute(m_surfaceTag)||
			!pnt1->HasAttribute(m_surfaceTag)||!
			pnt2->HasAttribute(m_surfaceTag))
			continue;

		surNum0 = pnt0->Attribute(m_surfaceTag);
		surNum1 = pnt1->Attribute(m_surfaceTag);
		surNum2 = pnt2->Attribute(m_surfaceTag);

		if (surNum0==surNum1 || surNum0==surNum2 || surNum0==surNum2)
			continue;
		indP0 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum0);
		indP1 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum1);
		indP2 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum2);
		if (indP0==-1 || indP1==-1 || indP2==-1) continue;

		newAPnt = new AnchorPoint(pnt0, &((m_vecSurPlanes)[indP0]), pnt1, 
			&((m_vecSurPlanes)[indP1]), pnt2, &((m_vecSurPlanes)[indP2]));
		m_vecAncPnts.push_back(newAPnt);
	}

	return true;
}

AnchorPoint* AnchorPointSearcher::SearchNextAncPnt(AnchorPoint* curAPnt, int surfTag1, int surfTag2)
{
	assert(curAPnt);
	LASITR lasPnt1 = curAPnt->GetAttPntBySegNum(surfTag1);
	LASITR lasPnt2 = curAPnt->GetAttPntBySegNum(surfTag2);
	//if (!lasPnt1 || !lasPnt2) return NULL;
	
	AnchorEDGE curEdge, nexEdge;
	vector<AnchorEDGE> vecEdges;
	vecEdges.push_back(AnchorEDGE(lasPnt1-m_Cloud.begin(), lasPnt2-m_Cloud.begin()));
	PointNumberList adjPnts;
	int newEdge;//0, no new edge, 1 one new edge, 2 find next anchor point
	LASITR lasPnt3;
	AnchorPoint* nextAPnt=NULL;

	do {
		newEdge = 0;
		AnchorEDGE& curEdge = vecEdges[vecEdges.size()-1];
		adjPnts = GetAdjPnts(m_TINEdges, curEdge.pntNum0, curEdge.pntNum1);
		lasPnt1 = m_Cloud.begin() + curEdge.pntNum0;
		lasPnt2 = m_Cloud.begin() + curEdge.pntNum1;
		
		assert(adjPnts.size()<3);
		for (int i=0; i<adjPnts.size(); i++) {
			LASITR lasPnt3 = m_Cloud.begin()+adjPnts[i].Number();
			
			if (lasPnt3->Attribute(m_surfaceTag) == lasPnt1->Attribute(m_surfaceTag)) {
				nexEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum1);
				if (vecEdges.end()==find(vecEdges.begin(), vecEdges.end(), nexEdge)) continue;
				vecEdges.push_back(nexEdge);
				newEdge = 1;
			}

			else if (lasPnt3->Attribute(m_surfaceTag) == lasPnt2->Attribute(m_surfaceTag)) {
				nexEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum0);
				if (vecEdges.end()==find(vecEdges.begin(), vecEdges.end(), nexEdge)) continue;
				vecEdges.push_back(nexEdge);
				newEdge = 1;
			}

			else {
				int ind1 = IndexPlaneBySegNumber(m_vecSurPlanes, lasPnt1->Attribute(m_surfaceTag));
				int ind2 = IndexPlaneBySegNumber(m_vecSurPlanes, lasPnt2->Attribute(m_surfaceTag));
				int ind3 = IndexPlaneBySegNumber(m_vecSurPlanes, lasPnt3->Attribute(m_surfaceTag));
				if (ind1==-1 || ind2==-1 || ind3==-1) continue;
				nextAPnt = new AnchorPoint(
					lasPnt1, &(*(m_vecSurPlanes.begin()+ind1)), 
					lasPnt2, &(*(m_vecSurPlanes.begin()+ind1)),
					lasPnt3, &(*(m_vecSurPlanes.begin()+ind1)));
				if (*nextAPnt == *curAPnt) {
					delete nextAPnt;
					nextAPnt = NULL;
				}
				else
					newEdge = 2;
			}
		}
	} while (newEdge==1);
	

	if (newEdge==0 && vecEdges.size()>1) {
		//make sure it is outer boundary points
		bool bOnContour1 = false, bOnContour2 = false;
		for (int iEdge=0; iEdge<vecEdges.size(); iEdge++) {
			bOnContour1 = IsOnContour(vecEdges[iEdge].pntNum0);
			bOnContour2 = IsOnContour(vecEdges[iEdge].pntNum1);
			//both points are on contour
			if (bOnContour1 && bOnContour2) {
				vecEdges.erase(vecEdges.begin()+iEdge, vecEdges.end());
				break;
			}
		}


		lasPnt1 = m_Cloud.begin() + vecEdges[vecEdges.size()-1].pntNum0;
		if (!bOnContour1) {
			lasPnt1 = FindNeastContPntOnSameSurface(lasPnt1);
			vecEdges[vecEdges.size()-1].pntNum0 = lasPnt1 - m_Cloud.begin();
		}

		lasPnt2 = m_Cloud.begin() + vecEdges[vecEdges.size()-1].pntNum1;	
		if (!bOnContour2) {
			lasPnt2 = FindNeastContPntOnSameSurface(lasPnt2);
			vecEdges[vecEdges.size()-1].pntNum1 = lasPnt2 - m_Cloud.begin();
		}

		int ind1 = IndexPlaneBySegNumber(m_vecSurPlanes, lasPnt1->Attribute(m_surfaceTag));
		int ind2 = IndexPlaneBySegNumber(m_vecSurPlanes, lasPnt2->Attribute(m_surfaceTag));

		if (ind1!=-1 && ind2!=-1) {
			nextAPnt = new AnchorPoint(
				lasPnt1, &(*(m_vecSurPlanes.begin()+ind1)), 
				lasPnt2, &(*(m_vecSurPlanes.begin()+ind1)));
			if (*nextAPnt == *curAPnt) {
				delete nextAPnt;
				nextAPnt = NULL;
			}
		}
	}

	return NULL;
}

AnchorPoint* AnchorPointSearcher::NewAnchorPnt(LASITR inPnt1, LASITR inPnt2)
{
	if (inPnt1==m_Cloud.end()||inPnt2==m_Cloud.end()) return NULL;

	int AncPntInd = m_vecAncPnts.size();
	if (!inPnt1->HasAttribute(m_surfaceTag)||!inPnt2->HasAttribute(m_surfaceTag))
		return NULL;

	int surNum0 = inPnt1->Attribute(m_surfaceTag);
	int surNum1 = inPnt2->Attribute(m_surfaceTag);
	if (surNum0==surNum1) return NULL;

	int indP0 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum0);
	int indP1 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum1);
	if (indP0==-1 || indP1==-1) return NULL;

	AnchorPoint* pAncPnt = new AnchorPoint(inPnt1, &((m_vecSurPlanes)[indP0]), inPnt2, &((m_vecSurPlanes)[indP1]));
	pAncPnt->SetID(AncPntInd);
	inPnt1->SetAttribute(m_anchorTag, AncPntInd);
	inPnt2->SetAttribute(m_anchorTag, AncPntInd);
	m_vecAncPnts.push_back(pAncPnt);


	return pAncPnt;
}

AnchorPoint* AnchorPointSearcher::NewAnchorPnt(LASITR inPnt1, LASITR inPnt2, LASITR inPnt3)
{
	if (inPnt1==m_Cloud.end()||inPnt2==m_Cloud.end()||inPnt3==m_Cloud.end()) return NULL;

	int AncPntInd = m_vecAncPnts.size();
	if (!inPnt1->HasAttribute(m_surfaceTag)||
		!inPnt2->HasAttribute(m_surfaceTag)||
		!inPnt3->HasAttribute(m_surfaceTag))
		return NULL;

	int surNum1 = inPnt1->Attribute(m_surfaceTag);
	int surNum2 = inPnt2->Attribute(m_surfaceTag);
	int surNum3 = inPnt3->Attribute(m_surfaceTag);
	if (surNum1==surNum2||surNum1==surNum3||surNum2==surNum3) return NULL;

	int indP1 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum1);
	int indP2 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum2);
	int indP3 = IndexPlaneBySegNumber(m_vecSurPlanes, surNum3);
	if (indP1==-1 || indP2==-1 || indP3==-1) return NULL;

	AnchorPoint* pAncPnt = new AnchorPoint(inPnt1, &((m_vecSurPlanes)[indP1]), 
		inPnt2, &((m_vecSurPlanes)[indP2]), inPnt3, &((m_vecSurPlanes)[indP3]));
	
	pAncPnt->SetID(AncPntInd);
	inPnt1->SetAttribute(m_anchorTag, AncPntInd);
	inPnt2->SetAttribute(m_anchorTag, AncPntInd);
	inPnt3->SetAttribute(m_anchorTag, AncPntInd);
	m_vecAncPnts.push_back(pAncPnt);

	LaserPoint temPnt;
	temPnt.Position3DRef() = pAncPnt->GetPosition();
	if (!temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) {
		temPnt.X() = (inPnt1->X() + inPnt2->X() + inPnt3->X())/3.0;
		temPnt.Y() = (inPnt1->Y() + inPnt2->Y() + inPnt3->Y())/3.0;
		temPnt.Z() = (inPnt1->Z() + inPnt2->Z() + inPnt3->Z())/3.0;
		pAncPnt->SetPosition(temPnt.Position3DRef());
		pAncPnt->IsStable(true);
	}

	
	return pAncPnt;
}


AnchorPoint* AnchorPointSearcher::NewAnchorPnt(int segNum0, int segNum1, int segNum2, const Vector3D& pos)
{
	Position3D position(pos);
	LASITR lasPnt0, lasPnt1, lasPnt2, lasPntTem;
	double minDist, temDist;
	
	int indSeg = IndexSegBySegNumber(m_Cloud, m_pnlSegs, segNum0, m_surfaceTag);
	minDist = std::numeric_limits<double>::max();
	for (int i=0; i<m_pnlSegs[indSeg].size(); i++) {
		lasPntTem = m_Cloud.begin()+m_pnlSegs[indSeg][i].Number();
		temDist = lasPntTem->Distance(position);

		if (temDist < minDist) {
			minDist = temDist;
			lasPnt0 = lasPntTem;
		}
	}


	indSeg = IndexSegBySegNumber(m_Cloud, m_pnlSegs, segNum1, m_surfaceTag);
	minDist = std::numeric_limits<double>::max();
	for (int i=0; i<m_pnlSegs[indSeg].size(); i++) {
		lasPntTem = m_Cloud.begin()+m_pnlSegs[indSeg][i].Number();
		temDist = lasPntTem->Distance(position);

		if (temDist < minDist) {
			minDist = temDist;
			lasPnt1 = lasPntTem;
		}
	}

	indSeg = IndexSegBySegNumber(m_Cloud, m_pnlSegs, segNum2, m_surfaceTag);
	minDist = std::numeric_limits<double>::max();
	for (int i=0; i<m_pnlSegs[indSeg].size(); i++) {
		lasPntTem = m_Cloud.begin()+m_pnlSegs[indSeg][i].Number();
		temDist = lasPntTem->Distance(position);

		if (temDist < minDist) {
			minDist = temDist;
			lasPnt2 = lasPntTem;
		}
	}

	AnchorPoint* temAPnt = NewAnchorPnt(lasPnt0, lasPnt1, lasPnt2);
	temAPnt->SetPosition(pos);
	temAPnt->IsStable(true);
	
	return temAPnt;
}

AnchorBoundary* AnchorPointSearcher::NewAnchorBound(AnchorPoint* aPnt0, AnchorPoint* aPnt1,
	const std::vector<AnchorEDGE>& vecAncEdges, int indBnd2Pnt0, int indBnd2Pnt1)
{
	assert(aPnt0&&aPnt1);
	assert(indBnd2Pnt0>=0&&indBnd2Pnt0<=2&&indBnd2Pnt1>=0&&indBnd2Pnt1<=2);

	AnchorBoundary* ridgeAncBound = new AnchorBoundary();
	ridgeAncBound->AttachAncPnt(0, aPnt0);
	ridgeAncBound->AttachAncPnt(1, aPnt1);
	ridgeAncBound->SetAncEdges(vecAncEdges);
	aPnt0->AttachBoundary(indBnd2Pnt0, ridgeAncBound);
	aPnt1->AttachBoundary(indBnd2Pnt1, ridgeAncBound);
	m_vecAncBouns.push_back(ridgeAncBound);

	return ridgeAncBound;
}

AnchorPoint* AnchorPointSearcher::FindAnchorPnt(int segNum0, int segNum1, int segNum2) const
{
	std::vector<int> vecSegNum;
	vecSegNum.push_back(segNum0);
	vecSegNum.push_back(segNum1);
	vecSegNum.push_back(segNum2);
	std::sort(vecSegNum.begin(), vecSegNum.end());
	
	std::vector<int> vecLSegNum;
	AnchorPoint* rst = NULL;
	int temNum;

	for (int i=0; i<m_vecAncPnts.size(); i++) {
		if (m_vecAncPnts[i]->AttPntCount()!=3) continue;

		vecLSegNum.clear();
		for (int j=0; j<3; j++) {
			temNum = m_vecAncPnts[i]->GetAttPnt(j)->Attribute(m_surfaceTag);
			vecLSegNum.push_back(temNum);
		}

		std::sort(vecLSegNum.begin(), vecLSegNum.end());
		if (vecSegNum == vecLSegNum) {
			rst = m_vecAncPnts[i];
			break;
		}
	}

	return rst;
}

AnchorBoundary* AnchorPointSearcher::FindAnchorBound(int segNum0, int segNum1) const
{
	AnchorBoundary* rst = NULL;
	if (segNum0>segNum1) std::swap(segNum0, segNum1);

	int lSegNum0, lSegNum1;
	for (int i=0; i<m_vecAncBouns.size(); i++) {
		m_vecAncBouns[i]->GetAttachSegNums(lSegNum0, lSegNum1);
		if (lSegNum0>lSegNum1) std::swap(lSegNum0, lSegNum1);

		if (lSegNum0==segNum0 && lSegNum1==segNum1) {
			rst = m_vecAncBouns[i];
			break;
		}
	}

	return rst;
}

void AnchorPointSearcher::DeleteAnchorPnt(AnchorPoint* pAncPnt)
{
	for (int i=0; i<m_vecAncPnts.size(); i++) {
		if (m_vecAncPnts[i] == pAncPnt) {
			m_vecAncPnts.erase(m_vecAncPnts.begin()+i);
			break;
		}
	}
}

void AnchorPointSearcher::DeleteAnchorBound(AnchorBoundary* pAncBound)
{
	for (int i=0; i<m_vecAncBouns.size(); i++) {
		if (m_vecAncBouns[i] == pAncBound) {
			m_vecAncBouns.erase(m_vecAncBouns.begin()+i);
			break;
		}
	}
}

bool AnchorPointSearcher::TagBoundEdges(std::vector<AnchorEDGE>& vecAEdges)
{
	LASITR pnt1, pnt2;
	for (int i=0; i<vecAEdges.size(); i++) {
		pnt1 = m_Cloud.begin()+vecAEdges[i].pntNum0;
		pnt2 = m_Cloud.begin()+vecAEdges[i].pntNum1;
		if(pnt1->Attribute(m_anchorTag) == AncTagNone) 
			pnt1->SetAttribute(m_anchorTag, AncTagBound);

		if(pnt2->Attribute(m_anchorTag) == AncTagNone)
			pnt2->SetAttribute(m_anchorTag, AncTagBound);
	}

	return true;
}


AnchorBoundary* AnchorPointSearcher::SearchAncBoundary(AnchorPoint* curAPnt, int indVertex1, int indVertex2)
{
	assert(curAPnt);
	LASITR lasPnt1 = curAPnt->GetAttPnt(indVertex1);
	LASITR lasPnt2 = curAPnt->GetAttPnt(indVertex2);
	LASITR lasPnt3;// = curAPnt->GetAttPnt(3-indVertex1-indVertex2);
	int segNum1 = lasPnt1->Attribute(m_surfaceTag);
	int segNum2 = lasPnt2->Attribute(m_surfaceTag);
	int segNum3;// = lasPnt3->Attribute(m_surfaceTag);
	//if (!lasPnt1 || !lasPnt2) return NULL;

	AnchorEDGE curEdge, nextEdge;
	vector<AnchorEDGE> vecEdges;
	vector<AnchorEDGE> vecPossEdges;//possible edges
	vecEdges.reserve(500);
	vecEdges.push_back(AnchorEDGE(lasPnt1-m_Cloud.begin(), lasPnt2-m_Cloud.begin()));
	PointNumberList adjPnts;
	int newEdge;//0, no new edge, 1 one new edge, 2 find next anchor point
	//int indAnc1, indAnc2;

	int curAPntInd = lasPnt1->Attribute(m_anchorTag);
	std::set<AnchorEDGE> storedEdges;
	storedEdges.insert(vecEdges[0]);

	//search boundary edges
	int nLoop = 0;
	do {
		assert(nLoop++<100000);

		newEdge = 0;
		curEdge = vecEdges[vecEdges.size()-1];
		nextEdge = curEdge;
		adjPnts = GetAdjPnts(m_TINEdges, curEdge.pntNum0, curEdge.pntNum1);
		if (adjPnts.size()>2) adjPnts = CleanNeibPnts(curEdge, adjPnts);
		lasPnt1 = m_Cloud.begin() + curEdge.pntNum0;
		lasPnt2 = m_Cloud.begin() + curEdge.pntNum1;

		vecPossEdges.clear();
		//check points either one segment 1 or segment 2
		for (int i=0; i<adjPnts.size(); i++) {
			LASITR lasPnt3 = m_Cloud.begin()+adjPnts[i].Number();

			//edge 3-2
			if (lasPnt3->Attribute(m_surfaceTag) == lasPnt1->Attribute(m_surfaceTag)) {
				nextEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum1);
				if (storedEdges.count(nextEdge)!=0) continue;
				vecPossEdges.push_back(nextEdge);
			}
			//edge 3-1
			else if (lasPnt3->Attribute(m_surfaceTag) == lasPnt2->Attribute(m_surfaceTag)) {
				nextEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum0);
				if (storedEdges.count(nextEdge)!=0) continue;
				vecPossEdges.push_back(nextEdge);
			}
		}

		SortPossibleEdges(curEdge, vecPossEdges);
		for (int i=0; i<vecPossEdges.size(); i++) {
			nextEdge = vecPossEdges[i];
			storedEdges.insert(nextEdge);
			vecEdges.push_back(nextEdge);
			newEdge = 1;
		}


	} while (newEdge==1 && !IsOnSameAPnt(nextEdge.pntNum0,nextEdge.pntNum1));

#ifdef _DEBUG
	if ((segNum1==1 && segNum2==3) || (segNum1==3 && segNum2==1)) {
		WriteBoundaryEdges(vecEdges);
	}
#endif


	if (newEdge==0)
		nextEdge = vecEdges[vecEdges.size()-1];
	lasPnt1 = m_Cloud.begin() + nextEdge.pntNum0;
	lasPnt2 = m_Cloud.begin() + nextEdge.pntNum1;

	TagBoundEdges(vecEdges);
	AnchorPoint* nextAPnt = GetAnchorPnt(lasPnt1, lasPnt2);

	if (!nextAPnt) {
		ShrinkAEdges1SideByContour(vecEdges);
		nextEdge = vecEdges[vecEdges.size()-1];
		lasPnt1 = m_Cloud.begin() + nextEdge.pntNum0;
		lasPnt2 = m_Cloud.begin() + nextEdge.pntNum1;
		nextAPnt = NewAnchorPnt(lasPnt1, lasPnt2);
		assert(nextAPnt);

	}
	else if (nextAPnt->AttPntCount()==2) {
		AnchorEDGE temEdge = AnchorEDGE(nextAPnt->GetAttPnt(0)-m_Cloud.begin(),nextAPnt->GetAttPnt(1)-m_Cloud.begin());
		if (temEdge!=nextEdge) {//not exact the same edge of anchor point
			nextEdge = temEdge;
			vecEdges.push_back(nextEdge);
		}
	}


	TagBoundEdges(vecEdges);
	int indBnd2Pnt0 = curAPnt->GetAncBoundInd(indVertex1, indVertex2);
	indVertex1 = nextAPnt->GetAttPntInd(m_Cloud.begin()+nextEdge.pntNum0);
	indVertex2 = nextAPnt->GetAttPntInd(m_Cloud.begin()+nextEdge.pntNum1);
	int indBnd2Pnt1 = nextAPnt->GetAncBoundInd(indVertex1, indVertex2);
	AnchorBoundary* ridgeAncBound = NewAnchorBound(curAPnt, nextAPnt, vecEdges, indBnd2Pnt0, indBnd2Pnt1);

	return ridgeAncBound;
}



//////////////////////////////////////////////////////////////////////////
//search anchor boundary with no triple-anchor-point
AnchorBoundary* AnchorPointSearcher::SearchAncBoundary(const AnchorEDGE& inEdge)
{
	LASITR lasPnt0 = m_Cloud.begin()+inEdge.pntNum0;
	LASITR lasPnt1 = m_Cloud.begin()+inEdge.pntNum1;
	LASITR lasPnt2;
	int segNum0 = lasPnt0->Attribute(m_surfaceTag);
	int segNum1 = lasPnt1->Attribute(m_surfaceTag);

	bool bDebug = (inEdge.pntNum0==43&&inEdge.pntNum1==61) || (inEdge.pntNum0==61&&inEdge.pntNum1==43);
	if (bDebug) {
		int aaaaa = 0;
	}

	AnchorEDGE curEdge, nextEdge;
	vector<AnchorEDGE> vecEdges, vecAllEdges;
	vector<AnchorEDGE> vecPossEdges;//possible edges
	vecEdges.reserve(500);
	PointNumberList adjPnts,startAdjPnts;
	int newEdge;//0, no new edge, 1 one new edge, 2 find next anchor point

	std::set<AnchorEDGE> storedEdges;
	storedEdges.insert(inEdge);

	startAdjPnts = GetAdjPnts(m_TINEdges, inEdge.pntNum0, inEdge.pntNum1);
	if (startAdjPnts.size()>2) startAdjPnts = CleanNeibPnts(inEdge, startAdjPnts);

	//two searching directions
	for (int i=0; i<startAdjPnts.size(); i++) {
		lasPnt2 = m_Cloud.begin()+startAdjPnts[i].Number();

		//edge 3-1
		if (lasPnt2->Attribute(m_surfaceTag) == segNum0)
			curEdge = AnchorEDGE(startAdjPnts[i].Number(), inEdge.pntNum1);
		//edge 3-0
		else if (lasPnt2->Attribute(m_surfaceTag) == segNum1)
			curEdge = AnchorEDGE(startAdjPnts[i].Number(), inEdge.pntNum0);
		else
			continue;//no suitable node. ???

		storedEdges.insert(curEdge);
		vecEdges.clear();
		vecEdges.push_back(curEdge);

		//search boundary edges
		int nLoop = 0;
		do {
			assert(nLoop++<100000);

			newEdge = 0;
			curEdge = vecEdges[vecEdges.size()-1];
			nextEdge = curEdge;
			adjPnts = GetAdjPnts(m_TINEdges, curEdge.pntNum0, curEdge.pntNum1);
			if (adjPnts.size()>2) adjPnts = CleanNeibPnts(curEdge, adjPnts);
			lasPnt0 = m_Cloud.begin() + curEdge.pntNum0;
			lasPnt1 = m_Cloud.begin() + curEdge.pntNum1;

			vecPossEdges.clear();
			//check points either one segment 1 or segment 2
			for (int i=0; i<adjPnts.size(); i++) {
				lasPnt2 = m_Cloud.begin()+adjPnts[i].Number();

				//edge 3-2
				if (lasPnt2->Attribute(m_surfaceTag) == lasPnt0->Attribute(m_surfaceTag)) {
					nextEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum1);
					if (storedEdges.count(nextEdge)!=0) continue;
					vecPossEdges.push_back(nextEdge);
				}
				//edge 3-1
				else if (lasPnt2->Attribute(m_surfaceTag) == lasPnt1->Attribute(m_surfaceTag)) {
					nextEdge = AnchorEDGE(adjPnts[i].Number(), curEdge.pntNum0);
					if (storedEdges.count(nextEdge)!=0) continue;
					vecPossEdges.push_back(nextEdge);
				}
			}

			for (int i=0; i<vecPossEdges.size(); i++) {
				nextEdge = vecPossEdges[i];
				storedEdges.insert(nextEdge);
				vecEdges.push_back(nextEdge);
				newEdge = 1;
			}

#ifdef _DEBUG
			if (((segNum0==359&&segNum1==360)||(segNum1==360&&segNum0==359))&&nLoop>17) {
				WriteBoundaryEdges(vecEdges);

				for (LaserPoints::iterator pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++)
					pnt->SetAttribute(ComponentNumberTag, pnt->Attribute(m_anchorTag));
				for (int i=0; i<adjPnts.size(); i++) {
					lasPnt2 = m_Cloud.begin()+adjPnts[i].Number();
					lasPnt2->SetAttribute(ComponentNumberTag, 100);
				}
				m_Cloud.Write("debug.laser");
			}
#endif

		} while (newEdge==1 && !IsOnSameAPnt(nextEdge.pntNum0,nextEdge.pntNum1));

		if (i==0) {
			std::reverse(vecEdges.begin(),vecEdges.end());
			vecAllEdges.insert(vecAllEdges.end(), vecEdges.begin(), vecEdges.end());
			vecAllEdges.push_back(inEdge);
		}
		else 
			vecAllEdges.insert(vecAllEdges.end(), vecEdges.begin(), vecEdges.end());
	}



#ifdef _DEBUG
	if ((segNum0==359&&segNum1==360)||(segNum1==360&&segNum0==359)) {
		WriteBoundaryEdges(vecAllEdges);
	}
#endif

	if (vecAllEdges.size()<2) return NULL;
	curEdge = vecAllEdges[0];
	nextEdge = vecAllEdges[vecAllEdges.size()-1];
	if(!IsClosedAncBound(vecAllEdges)) {
		if (curEdge.pntNum0==nextEdge.pntNum0||curEdge.pntNum0==nextEdge.pntNum1||
			curEdge.pntNum1==nextEdge.pntNum0||curEdge.pntNum1==nextEdge.pntNum1)
			vecAllEdges.push_back(curEdge);
	}

	TagBoundEdges(vecAllEdges);
	if(!IsClosedAncBound(vecAllEdges)) {

		ShrinkAEdges2SideByContour(vecAllEdges);
	}

	if (vecAllEdges.empty()) return NULL;


	AnchorPoint* aPnt0 = NewAnchorPnt(vecAllEdges[0].pntNum0, vecAllEdges[0].pntNum1);
	AnchorPoint* aPnt1 = NewAnchorPnt(vecAllEdges[vecAllEdges.size()-1].pntNum0, vecAllEdges[vecAllEdges.size()-1].pntNum1);
	AnchorBoundary* ridgeAncBound = NewAnchorBound(aPnt0, aPnt1, vecAllEdges, 0, 0);


	return ridgeAncBound;
}


#include <numeric>



bool AnchorPointSearcher::ShrinkAnchorEdges1Side(std::vector<AnchorEDGE>& edges)
{
	//////////////////////////////////////////////////////////////////////////
	//get lengths
	if (edges.size()<2) return true;
	LASITR pnt11, pnt12, pnt21, pnt22;
	double lastLengh;
	vector<double> vecLens;
	double dx, dy;
	
	for (int i=0; i<edges.size(); i++) {
		pnt11 = m_Cloud.begin()+edges[i].pntNum0;
		pnt12 = m_Cloud.begin()+edges[i].pntNum1;

		dx = pnt11->X()-pnt12->X();
		dy = pnt11->Y()-pnt12->Y();
		lastLengh = sqrt(dx*dx+dy*dy);
		vecLens.push_back(lastLengh);
	}
	double maxLen = *(std::max_element(vecLens.begin(), vecLens.end()));
	double minLen = *(std::min_element(vecLens.begin(), vecLens.end()));

	//////////////////////////////////////////////////////////////////////////
	//edge length histogram
	double binLen = (maxLen-minLen)/(vecLens.size());
	if (binLen<0.1) binLen = 0.1;
	vector<int> vecHist = vector<int>(int((maxLen-minLen)/binLen)+1, 0);

	int indHist;
	for(int i=0; i<vecLens.size(); i++) {
		indHist = (vecLens[i]-minLen)/binLen;
		vecHist[indHist] += 1;
	}

	vector<int>::iterator maxEle = std::max_element(vecHist.begin(), vecHist.end());
	int maxFrequent = *maxEle;
	int indMaxFreq = maxEle - vecHist.begin();
	double freqLen = minLen+(indMaxFreq+0.5)*binLen;
	double LengThresh = freqLen*3.5;

	int indEdge=edges.size();
	for (int i=vecLens.size()/3; i<vecLens.size(); i++) {
		if (vecLens[i]>LengThresh) {
			indEdge = i;
			break;
		}
	}

	edges.erase(edges.begin()+indEdge, edges.end());

	return true;
}


bool AnchorPointSearcher::ShrinkAEdges1SideByContour(std::vector<AnchorEDGE>& edges)
{
	if (edges.size()<2) return true;
	LASITR pnt1, pnt2;
	int indEdge = edges.size()-1;
	LaserPoint midPnt;

	for (unsigned int i=edges.size()-1; i>0; i--) {
		pnt1 = m_Cloud.begin()+edges[i].pntNum0;
		pnt2 = m_Cloud.begin()+edges[i].pntNum1;
		midPnt = (pnt1->Position3DRef()+pnt2->Position3DRef())/2;

		//stop when one point is fully inside the contour
		if (pnt1->InsidePolygonJordan(m_contourObjPnts,m_contourTop) &&
			pnt2->InsidePolygonJordan(m_contourObjPnts,m_contourTop) &&
			midPnt.InsidePolygonJordan(m_contourObjPnts,m_contourTop)) {
				indEdge = i;
				break;
		}
	}

	if(indEdge!=edges.size()-1)
		edges.erase(edges.begin()+indEdge, edges.end());

	return true;
}


bool AnchorPointSearcher::ShrinkAnchorEdges2Side(std::vector<AnchorEDGE>& edges)
{
	//////////////////////////////////////////////////////////////////////////
	//get lengths
	if (edges.size()<2) return true;
	LASITR pnt11, pnt12, pnt21, pnt22;
	double lastLengh;
	vector<double> vecLens;
	double dx, dy;
	
	for (int i=0; i<edges.size(); i++) {
		pnt11 = m_Cloud.begin()+edges[i].pntNum0;
		pnt12 = m_Cloud.begin()+edges[i].pntNum1;

		dx = pnt11->X()-pnt12->X();
		dy = pnt11->Y()-pnt12->Y();
		lastLengh = sqrt(dx*dx+dy*dy);
		vecLens.push_back(lastLengh);
	}
	double maxLen = *(std::max_element(vecLens.begin(), vecLens.end()));
	double minLen = *(std::min_element(vecLens.begin(), vecLens.end()));



	//////////////////////////////////////////////////////////////////////////
	//edge length histogram
	double binLen = 2*(maxLen-minLen)/(vecLens.size());
	if (binLen<0.1) binLen = 0.1;
	vector<int> vecHist = vector<int>(int((maxLen-minLen)/binLen)+1, 0);

	int indHist;
	for(int i=0; i<vecLens.size(); i++) {
		indHist = (vecLens[i]-minLen)/binLen;
		vecHist[indHist] += 1;
	}

	vector<int>::iterator maxEle = std::max_element(vecHist.begin(), vecHist.end());
	int maxFrequent = *maxEle;
	int indMaxFreq = maxEle - vecHist.begin();
	double freqLen = minLen+(indMaxFreq+0.5)*binLen;
	double LengThresh = freqLen*3.5;

	//////////////////////////////////////////////////////////////////////////
	//get break points
	int indEdgeMid = vecLens.size()/2;
	for (int i=vecLens.size()/2; i<vecLens.size(); i++) {
		if (vecLens[i]<freqLen) {
			indEdgeMid = i;
			break;
		}
	}
	if (indEdgeMid==vecLens.size()) {
		for (int i=vecLens.size()/2; i>=0; i--) {
			if (vecLens[i]<freqLen) {
				indEdgeMid = i;
				break;
			}
		}
	}
	assert(indEdgeMid>=0);

	int indEdge0=0, indEdge1=edges.size();
	//for (int i=vecLens.size()/2; i>=0; i--) {
	for (int i=indEdgeMid; i>=0; i--) {
		if (vecLens[i]>LengThresh) {
			indEdge0 = i+1;
			break;
		}
	}

	//for (int i=vecLens.size()/2; i<vecLens.size(); i++) {
	for (int i=indEdgeMid; i<vecLens.size(); i++) {
		if (vecLens[i]>LengThresh) {
			indEdge1 = i;
			break;
		}
	}

	if (indEdge0<indEdge1)
		edges = std::vector<AnchorEDGE>(edges.begin()+indEdge0, edges.begin()+indEdge1);

	return true;
}


bool AnchorPointSearcher::ShrinkAEdges2SideByContour(std::vector<AnchorEDGE>& edges)
{
	if (edges.size()<2) return true;
	int indStart = 0;
	int indEnd = edges.size()-1;

	//search for start point
	for (int i=edges.size()/2; i>=0; --i) {
		if (!IsInsideContour(edges[i]))	{
			indStart = i+1;
			break;
		}
	}

	//search for end point
	for (int i=edges.size()/2; i<edges.size(); ++i) {
		if (!IsInsideContour(edges[i]))	{
			indEnd = i-1;
			break;
		}
	}
	
	if (indStart>=indEnd)
		edges.clear();
	else if(indStart!=0 || indEnd!=edges.size()-1)
		edges = vector<AnchorEDGE>(edges.begin()+indStart, edges.begin()+indEnd+1);

	return true;
}

bool AnchorPointSearcher::ShrinkRidge(AnchorPoint* aPnt0, AnchorPoint* aPnt1)
{
	assert(aPnt0 && aPnt1);

	LaserPoint temLasPnt;
	temLasPnt.SetX(aPnt1->GetPosition().X());
	temLasPnt.SetY(aPnt1->GetPosition().Y());
	//only shrink endpoint outside the contour
	if(temLasPnt.InsidePolygonJordan(m_contourObjPnts,m_contourTop)) return false;

	LineSegment2D line = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
	Position2D inter_pos;
	vector<double> vecDits;

	double	scalar0, dist0;
	for (int i=0; i<m_contourSegs.size(); i++) {
		//get intersection point, if the two lines are parallel, take the projection point
		if(Angle2Lines(line.Line2DReference(), m_contourSegs[i].Line2DReference()) > 0.01*PI/180 ) {
			Intersection2NonParallelLines(line.Line2DReference(), m_contourSegs[i].Line2DReference(), inter_pos);
			scalar0 = m_contourSegs[i].Scalar(inter_pos);
			//dist0 = line.Scalar(inter_pos)-line.ScalarBegin();
			//Oct 23, 2014		
			dist0 = line.ScalarEnd()-line.Scalar(inter_pos);
		}
		else {
			scalar0 = m_contourSegs[i].ScalarEnd()+10.0;
			dist0 = 9999999.9;
		}

		//the intersection should be on the contour segment
		//and on the ray of ridge line
		if (scalar0>=m_contourSegs[i].ScalarBegin()&&scalar0<m_contourSegs[i].ScalarEnd()&& 
			dist0>-0.00001)
			vecDits.push_back(dist0);
		else
			vecDits.push_back(9999999.9);
	}

	vector<double>::iterator itrMin = std::min_element(vecDits.begin(), vecDits.end());
	if (*itrMin==9999999.9) return false;

	Intersection2NonParallelLines(line.Line2DReference(), 
		m_contourSegs[itrMin-vecDits.begin()].Line2DReference(), inter_pos);
	//line.Intersect(m_contourSegs[itrMin-vecDits.begin()], inter_pos, 0.001);
	Line3D plumb = Line3D(Position3D(inter_pos.X(), inter_pos.Y(), 0.0), Vector3D(0.0, 0.0, 1.0));
	Line3D ridge = Line3D(Position3D(aPnt0->GetPosition()), Position3D(aPnt1->GetPosition()));
	Position3D newPos;
	if(!MyIntersect2Lines(plumb, ridge, newPos)) return false;
//	if(!Intersection2Lines(plumb, ridge, newPos)) return false;

#ifdef _DEBUG
	//line = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
	//line.Intersect(m_contourSegs[itrMin-vecDits.begin()], inter_pos, 0.001);
	double dist1 = m_contourSegs[itrMin-vecDits.begin()].DistanceToPoint(newPos.Position2DOnly());
	double dist2 = m_contourSegs[itrMin-vecDits.begin()].DistanceToPoint(inter_pos);
#endif

	aPnt1->SetPosition(newPos);
	return true;
}


bool AnchorPointSearcher::ExportContour(ObjectPoints& destObjPnts, LineTopologies& destLineTops) 
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


bool AnchorPointSearcher::WriteBoundaryEdges(const std::vector<AnchorEDGE>& edges, char* path)
{
	ObjectPoints objPnts;
	LineTopologies lineTops;
	ObjectPoint temPnt;
	LineTopology temTop;
	int pntNum = 0;

	for (unsigned int i=0; i<edges.size(); i++) {
		temTop.clear();

		temPnt.Position3DRef() = m_Cloud[edges[i].pntNum0].Position3DRef();
		temPnt.Number() = pntNum++;
		objPnts.push_back(temPnt);
		temTop.push_back(temPnt.NumberRef());

		temPnt.Position3DRef() = m_Cloud[edges[i].pntNum1].Position3DRef();
		temPnt.Number() = pntNum++;
		objPnts.push_back(temPnt);
		temTop.push_back(temPnt.NumberRef());

		lineTops.push_back(temTop);		
	}

	objPnts.Write("debug.objpts");
	lineTops.Write("debug.top");

	return true;
}


AnchorPoint* AnchorPointSearcher::GetAnchorPnt(LASITR inPnt1, LASITR inPnt2)
{
	assert(inPnt1!=m_Cloud.end() && inPnt2!=m_Cloud.end());
	assert(inPnt1!=inPnt2);

	std::map<AnchorPoint*, int> aPntCounts;
	std::map<AnchorPoint*, int>::iterator itrMap;

	//initiate the map
	for (unsigned int i=0; i<m_vecAncPnts.size(); i++)
		aPntCounts.insert(std::pair<AnchorPoint*, int>(m_vecAncPnts[i], 0));

	//count the anchor point
	for (unsigned int i=0; i<m_vecAncPnts.size(); i++) {
		if (m_vecAncPnts[i]->GetAttPntInd(inPnt1)!=-1){//docked laser point 1
			itrMap = aPntCounts.find(m_vecAncPnts[i]);
			itrMap->second += 1;
		}

		if (m_vecAncPnts[i]->GetAttPntInd(inPnt2)!=-1){//docked laser point 2
			itrMap = aPntCounts.find(m_vecAncPnts[i]);
			itrMap->second += 1;
		}
	}

	//get the most often anchor point
	int maxTime = 0;
	AnchorPoint* rstAPnt = NULL;
	for (itrMap=aPntCounts.begin(); itrMap!=aPntCounts.end(); itrMap++) {
		if (itrMap->second > maxTime) {
			maxTime = itrMap->second;
			rstAPnt = itrMap->first;
		}
	}


	if (maxTime<2) return NULL;
	else return rstAPnt;
}

bool AnchorPointSearcher::IsOnSameAPnt(int indLasPnt1, int indLasPnt2)
{
	return IsOnSameAPnt(m_Cloud.begin()+indLasPnt1,m_Cloud.begin()+indLasPnt2);
}

bool AnchorPointSearcher::IsOnSameAPnt(LASITR inPnt0, LASITR inPnt1)
{
	assert(inPnt0!=m_Cloud.end() && inPnt1!=m_Cloud.end());
	assert(inPnt0!=inPnt1);

	if (!inPnt0->HasAttribute(m_anchorTag) || !inPnt1->HasAttribute(m_anchorTag)) return false;
	if (inPnt0->Attribute(m_anchorTag)==AncTagNone || 
		inPnt1->Attribute(m_anchorTag)==AncTagNone) 
		return false;

	for (unsigned int i=0; i<m_vecAncPnts.size(); i++) {
		if (m_vecAncPnts[i]->GetAttPntInd(inPnt0)!=-1 &&
			m_vecAncPnts[i]->GetAttPntInd(inPnt1)!=-1) {
				return true;
		}
	}

	return false;
}

bool AnchorPointSearcher::IsInsideContour(LASITR lasPnt0)
{
	return lasPnt0->InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool AnchorPointSearcher::IsInsideContour(int indLasPnt0)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size());
	return IsInsideContour(m_Cloud.begin()+indLasPnt0);
}

bool AnchorPointSearcher::IsInsideContour(LASITR lasPnt0, LASITR lasPnt1)
{
	LaserPoint temPnt;
	temPnt.Position3DRef() = (lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2;
	return temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool AnchorPointSearcher::IsInsideContour(int indLasPnt0, int indLasPnt1)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size() &&
		indLasPnt1>=0 && indLasPnt1<m_Cloud.size());

	return IsInsideContour(m_Cloud.begin()+indLasPnt0,
		m_Cloud.begin()+indLasPnt1);
}

bool AnchorPointSearcher::IsInsideContour(AnchorEDGE aEdge)
{
	return IsInsideContour(aEdge.pntNum0, aEdge.pntNum1);
}

bool AnchorPointSearcher::IsInsideContour(LASITR lasPnt0, LASITR lasPnt1, LASITR lasPnt2)
{
	LaserPoint temPnt;
	temPnt.Position3DRef() = (lasPnt0->Position3DRef()+
		lasPnt1->Position3DRef()+lasPnt2->Position3DRef())/3;
	return temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop);
}

bool AnchorPointSearcher::IsInsideContour(int indLasPnt0, int indLasPnt1, int indLasPnt2)
{
	assert(indLasPnt0>=0 && indLasPnt0<m_Cloud.size() &&
		indLasPnt1>=0 && indLasPnt1<m_Cloud.size() &&
		indLasPnt2>=0 && indLasPnt2<m_Cloud.size());

	return IsInsideContour(m_Cloud.begin()+indLasPnt0,
		m_Cloud.begin()+indLasPnt1,
		m_Cloud.begin()+indLasPnt2);
}

bool AnchorPointSearcher::IsInsideContour(const AnchorBoundary& aBound, const Line3D& ridge) const
{
	LaserPoint temLasPnt;
	const std::vector<AnchorEDGE>& vecAncEdges = aBound.GetAncEdges();
	LaserPoints::const_iterator curLasPnt;
	
	curLasPnt = m_Cloud.begin()+vecAncEdges[0].pntNum0;
	temLasPnt.Position3DRef() = ridge.Project(*curLasPnt);
	if(!temLasPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return false;

	curLasPnt = m_Cloud.begin()+vecAncEdges[0].pntNum1;
	temLasPnt.Position3DRef() = ridge.Project(*curLasPnt);
	if(!temLasPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return false;

	curLasPnt = m_Cloud.begin()+vecAncEdges[vecAncEdges.size()-1].pntNum0;
	temLasPnt.Position3DRef() = ridge.Project(*curLasPnt);
	if(!temLasPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return false;

	curLasPnt = m_Cloud.begin()+vecAncEdges[vecAncEdges.size()-1].pntNum0;
	temLasPnt.Position3DRef() = ridge.Project(*curLasPnt);
	if(!temLasPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return false;
	
	return true;
}

//[0-1] the higher, the better
double AnchorPointSearcher::ComputeRidgeCoef(const AnchorBoundary& aBound, const Line3D& ridge) const
{
	double distThreshold = 1.2;
	LaserPoint temLasPnt;
	const std::vector<AnchorEDGE>& vecAncEdges = aBound.GetAncEdges();
	LaserPoints::const_iterator lasPnt0, lasPnt1;
	double dist0, dist1;
	if (vecAncEdges.size()<=3) return 0.0;

	std::vector<double> vecDists;
	for (int i=0; i<vecAncEdges.size(); i++) {
		lasPnt0 = m_Cloud.begin()+vecAncEdges[i].pntNum0;
		lasPnt1 = m_Cloud.begin()+vecAncEdges[i].pntNum1;
		dist0 = ridge.DistanceToPoint(*lasPnt0);
		dist1 = ridge.DistanceToPoint(*lasPnt1);

		vecDists.push_back(dist0<dist1 ? dist0:dist1);
	}


	int nClosePnt=0;
	for (int i=0; i<vecDists.size(); i++) {
		if (vecDists[i]<distThreshold) nClosePnt++;
	}

	double coef=nClosePnt/vecDists.size();
	return coef;
}

bool AnchorPointSearcher::IsNearContour(Vector3D pos, double errDist)
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

bool AnchorPointSearcher::IsClosedAncBound(AnchorBoundary* pABound)
{
	AnchorPoint* aPnt0 = pABound->GetAttAncPnt(0);
	AnchorPoint* aPnt1 = pABound->GetAttAncPnt(1);
	
	//if only one edge, check the geometry
	if (pABound->GetAncEdges().size()==1) {
		if (aPnt0->GetPosition().X()!=aPnt1->GetPosition().X() 
		||  aPnt0->GetPosition().Y()!=aPnt1->GetPosition().Y()
		||  aPnt0->GetPosition().Z()!=aPnt1->GetPosition().Z())
			return false;
	}

	if(aPnt0->AttPntCount()==2 && aPnt1->AttPntCount()==2) {
		LASITR p00 = aPnt0->GetAttPnt(0);
		LASITR p01 = aPnt0->GetAttPnt(1);
		LASITR p10 = aPnt1->GetAttPnt(0);
		LASITR p11 = aPnt1->GetAttPnt(1);

		return (p00==p10&&p01==p11) || (p00==p11&&p01==p10);
	}

	return false;
}

bool AnchorPointSearcher::CloseAncBound(AnchorBoundary* pABound)
{
	if (!pABound) return false;

	AnchorPoint* aPnt0 = pABound->GetAttAncPnt(0);
	AnchorPoint* aPnt1 = pABound->GetAttAncPnt(1);

	if (aPnt0->AttBoundCount()!=1 || aPnt0->AttBoundCount()!=1)
		return false;

	aPnt1->ChangeAttPnt(aPnt1->GetAttPnt(0), aPnt0->GetAttPnt(0), aPnt0->GetAttPlane(0));
	aPnt1->ChangeAttPnt(aPnt1->GetAttPnt(1), aPnt0->GetAttPnt(1), aPnt0->GetAttPlane(1));
	aPnt1->SetPosition(aPnt0->GetPosition());

	std::vector<AnchorEDGE> allEdges = pABound->GetAncEdges();
	allEdges.push_back(*allEdges.begin());

	return false;
}

bool AnchorPointSearcher::IsClosedAncBound(std::vector<AnchorEDGE>& vecAEdges)
{
	if (vecAEdges.size()<2) return false;

	AnchorEDGE aEdge0 = vecAEdges[0];
	AnchorEDGE aEdge1 = vecAEdges[vecAEdges.size()-1];

	return  aEdge0.pntNum0==aEdge1.pntNum0&&aEdge0.pntNum1==aEdge1.pntNum1;
}

//use angle to reference edge to sort possible edge
void AnchorPointSearcher::SortPossibleEdges(const AnchorEDGE& refEdge, std::vector<AnchorEDGE>& vecPossEdges)
{
	if (vecPossEdges.size()<2) return;//no need to sort
	
	LineSegment2D refLine;
	int startPntNum;
	if (refEdge.pntNum0==vecPossEdges[0].pntNum0 || refEdge.pntNum0==vecPossEdges[0].pntNum1) {
		startPntNum = refEdge.pntNum0;
		refLine = LineSegment2D(m_Cloud[refEdge.pntNum0].Position2DOnly(), m_Cloud[refEdge.pntNum1].Position2DOnly());
	}
	else {
		startPntNum = refEdge.pntNum1;
		refLine = LineSegment2D(m_Cloud[refEdge.pntNum1].Position2DOnly(), m_Cloud[refEdge.pntNum0].Position2DOnly());
	}

	LineSegments2D vecPossLines;
	for (unsigned int i=0; i<vecPossEdges.size(); i++) {
		if (startPntNum == vecPossEdges[i].pntNum0)
			vecPossLines.push_back(LineSegment2D(m_Cloud[vecPossEdges[i].pntNum0].Position2DOnly(), m_Cloud[vecPossEdges[i].pntNum1].Position2DOnly()));
		else
			vecPossLines.push_back(LineSegment2D(m_Cloud[vecPossEdges[i].pntNum1].Position2DOnly(), m_Cloud[vecPossEdges[i].pntNum0].Position2DOnly()));
	}
	
	//compute angles
	vector<double> vecAngles;
	for (unsigned int i=0; i<vecPossLines.size(); i++) {
		double angle = Angle2D(refLine.Normal(), vecPossLines[i].Normal());
		vecAngles.push_back(fabs(angle));//the angle should be in the range of 0-pi
	}

	//sort the edges by increasing sort of angles
	for (unsigned int i=0; i<vecAngles.size(); i++) {
		for (int j=i+1; j<vecAngles.size(); j++) {
			if (vecAngles[i]<vecAngles[j]) continue;
			std::swap(vecAngles[i], vecAngles[j]);
			std::swap(vecPossEdges[i],vecPossEdges[j]);
		}
	}

}


double AnchorPointSearcher::EvaluateBoundaryStabality(const VecAnchorEdges& edges, const Position3D& begNode, const Position3D& endNode) const
{
	int nEdges = edges.size();
	if (nEdges<2) return 1.0;
	double scale = 1.0/(nEdges-1);
	Vector3D vecBeg2End(endNode-begNode);
	Position3D temPosPnt;
	
	double dist0, dist1, dist2;
	LaserPoints::const_iterator lasPnt0, lasPnt1;
	LaserPoint lasPntMid;
	std::vector<double> vecDists(nEdges, 99999999.9);
	
	for (unsigned int i=0; i<edges.size(); i++)	{
		temPosPnt = begNode + scale*vecBeg2End;

		lasPnt0 = m_Cloud.begin()+edges[i].pntNum0;
		lasPnt1 = m_Cloud.begin()+edges[i].pntNum1;
		lasPntMid.Position3DRef() = (lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2.0;
		
		dist0 = temPosPnt.Position2DOnly().Distance(lasPnt0->Position2DOnly());
		dist1 = temPosPnt.Position2DOnly().Distance(lasPnt1->Position2DOnly());
		dist2 = temPosPnt.Position2DOnly().Distance(lasPntMid.Position2DOnly());

		if (dist0<=dist1 && dist0<=dist2)
			vecDists[i] = dist0;
		else if (dist1<=dist0 && dist1<=dist2)
			vecDists[i] = dist1;
		else
			vecDists[i] = dist1;
	}

	double aveDist = 0.0;
	for (unsigned int i=0; i<vecDists.size(); i++)
		aveDist += vecDists[i]/nEdges;
	
	double score = exp(-aveDist*aveDist/9.0 );
	return score;
}

PointNumberList AnchorPointSearcher::CleanNeibPnts(const AnchorEDGE& curEdge, const PointNumberList& neibPnts)
{
	if (neibPnts.size()<3) return neibPnts;

	LASITR pnt1 = m_Cloud.begin()+curEdge.pntNum0;
	LASITR pnt2 = m_Cloud.begin()+curEdge.pntNum1;
	LASITR pnt3;
	Line2D curLine = Line2D(pnt1->Position2DOnly(), pnt2->Position2DOnly());
	Vector2D auxDir = curLine.Direction();
	Vector2D normal = curLine.Normal();
	Vector2D temDir;
	
	
	vector<double> vecDists;
	double temDist;

	for (unsigned int i=0; i<neibPnts.size(); i++) {
		pnt3 = m_Cloud.begin()+neibPnts[i].Number();

		temDist = curLine.DistanceToPointSigned(pnt3->Position2DOnly());
		vecDists.push_back(temDist);
	}

	double minNegDist = -9999999999999.0, minPosDist = 9999999999999.0;
	int minNegPntInd=-1, minPosPntInd=-1;
	for (unsigned int i=0; i<vecDists.size(); i++) {
		temDist = vecDists[i];
		switch (Sign(temDist))//
		{
		case 1://positive
			if (temDist<minPosDist) {
				minPosDist = temDist;
				minPosPntInd = i;
			}
			break;
		default://negative
			if (temDist>minNegDist)	{
				minNegDist = temDist;
				minNegPntInd = i;
			}
			break;
		}
	}


	if (minNegPntInd==-1 || minPosPntInd==-1)
		return neibPnts;

	PointNumberList rst;
	rst.push_back(neibPnts[minNegPntInd]);
	rst.push_back(neibPnts[minPosPntInd]);
	return rst;
}


void AnchorPointSearcher::SurfaceGrowContourPnts()
{
	std::vector<LASITR> unSegPnts;
	for (LASITR pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
		if (pnt->Attribute(m_surfaceTag) == -1)
			unSegPnts.push_back(pnt);
	}
	std::vector<LASITR> unProjectedPnts = unSegPnts;


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

#ifdef _DEBUG
	m_Cloud.Write("debug.laser");
#endif


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


void AnchorPointSearcher::RepairSegmentTag()
{
	m_Cloud.SetAttribute(AveragePulseLengthTag, -1);
	int regionNum = 0;
	LASITR curPnt, nextPnt;
	std::stack<LASITR> regStack;


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


	Plane destPlane;
	Line3D plumbLine;
	Position3D destPos;
	int indMaxReg;
	int maxRegSize;
	std::queue<PointNumberList> unRepairedRegs;
	std::vector<LASITR> vecUnProjPnts;
	std::vector<bool> vecStableStatus = std::vector<bool>(m_Cloud.size(), false);
	for (ITRMAP itrMap=mapRegions.begin(); itrMap!=mapRegions.end(); itrMap++) {

		maxRegSize = 0;
		for (unsigned int i=0; i<itrMap->second.size(); i++) {
			if (maxRegSize<(itrMap->second)[i].size()) {
				maxRegSize = (itrMap->second)[i].size();
				indMaxReg = i;
			}
		}


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


int AnchorPointSearcher::GetMaximumNeibRegionTag(const PointNumberList& curRegion, const std::vector<bool>& vecStabality)
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

	else if(mapSegPnts.size()==1 && mapSegPnts[curRegNum].size()>0)
		return -2;
	//invalid neighboring points
	else 
		return -1;
}

std::vector<AnchorPointSearcher::LASITR> 
	AnchorPointSearcher::GetNeibPntWithSameTag(LASITR inPnt, LaserPointTag tag)
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

int AnchorPointSearcher::GetMaxFrequentNeibTag(LASITR inPnt, LaserPointTag tag)
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

AnchorPointSearcher::LASITR AnchorPointSearcher::FindNeastContPntOnSameSurface(AnchorPointSearcher::LASITR inPnt)
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

void AnchorPointSearcher::SearchTripleAncPnts()
{
	LASITR pnt0, pnt1, pnt2;
	//vector<AnchorPoint> vecAncPnts;
	//int indP0, indP1, indP2;
	//int surNum0, surNum1, surNum2;
	int AncPntInd = m_vecAncPnts.size();
	AnchorPoint* pAncPnt;
	double dist0, dist1, dist2;
	bool bOnCtour0, bOnCtour1, bOnCtour2;//belonging to contour
	int surNum1, surNum2, surNum3;

	//check faces with node from 3 difference surfaces/segments
	for (int f=0; f<m_Tin.size(); f++) {
		pnt0 = m_Cloud.begin() + m_Tin[f].Nodes()[0].Number();
		pnt1 = m_Cloud.begin() + m_Tin[f].Nodes()[1].Number();
		pnt2 = m_Cloud.begin() + m_Tin[f].Nodes()[2].Number();
		
		if (!pnt0->HasAttribute(m_surfaceTag)||!pnt1->HasAttribute(m_surfaceTag)||!pnt2->HasAttribute(m_surfaceTag))
			continue;

		surNum1 = pnt0->Attribute(m_surfaceTag);
		surNum2 = pnt1->Attribute(m_surfaceTag);
		surNum3 = pnt2->Attribute(m_surfaceTag);
		if (surNum1==surNum2||surNum1==surNum3||surNum2==surNum3) continue;

		bOnCtour0 = IsOnContour(m_Tin[f].Nodes()[0].Number());
		bOnCtour1 = IsOnContour(m_Tin[f].Nodes()[1].Number());
		bOnCtour2 = IsOnContour(m_Tin[f].Nodes()[2].Number());
		if (bOnCtour0||bOnCtour1||bOnCtour2) continue;//skip border point


		pAncPnt = NewAnchorPnt(pnt0, pnt1, pnt2);
	}
}

void AnchorPointSearcher::SearchDoubleAncPnts()
{
	if (m_pnlContour.empty()) return;
	LASITR pnt0, pnt1;
	int surNum0, surNum1;
	AnchorPoint* pAncPnt;

	//check faces with node from 2 difference surfaces/segments
	for (unsigned int i=0; i<m_pnlContour.size()-1; ++i) {
		pnt0 = m_Cloud.begin() + m_pnlContour[i].Number();
		pnt1 = m_Cloud.begin() + m_pnlContour[i+1].Number();
		//tow anchor points may be adjacent
		if (pnt0->Attribute(m_anchorTag)!=AncTagNone && pnt1->Attribute(m_anchorTag)!=AncTagNone) 
			continue;//already have anchor point

		if (!pnt0->HasAttribute(m_surfaceTag) || !pnt1->HasAttribute(m_surfaceTag)) continue;
		surNum0 = pnt0->Attribute(m_surfaceTag);
		surNum1 = pnt1->Attribute(m_surfaceTag);
		if (surNum0 == surNum1) continue;

		pAncPnt = NewAnchorPnt(pnt0, pnt1);
	}
}

void AnchorPointSearcher::SearchAncBounds()
{
	//vector<int> vecTripleAPntInds;
	AnchorPoint* curAncPnt = NULL;
	AnchorBoundary* ancBound = NULL;

	for (unsigned int i=0; i<m_vecAncPnts.size(); ++i) {
		curAncPnt = m_vecAncPnts[i];
		assert(curAncPnt);
		
		if(curAncPnt->AttPntCount() == 3) {
			for (int j=0; j<3; j++) {
				if(curAncPnt->GetAttBoundary(j)) continue;//already has anchor boundary
				ancBound = SearchAncBoundary(curAncPnt, CW(j), CCW(j));
			}
		}
		else {
			if(!curAncPnt->GetAttBoundary(2)) //does not attached to one boundary
				ancBound = SearchAncBoundary(curAncPnt, CW(2), CCW(2));
		}		
	}

	int indPnt0, indPnt1;
	LASITR pnt0, pnt1;
	int segNum0, segNum1;
	int aPntNum0, aPntNum1;
	for (int f=0; f<m_Tin.size(); f++) {
		for (unsigned int i=0; i<3; i++)	{
			indPnt0 = m_Tin[f].Nodes()[CW(i)].Number();
			indPnt1 = m_Tin[f].Nodes()[CCW(i)].Number();
			pnt0 = m_Cloud.begin() + indPnt0;
			pnt1 = m_Cloud.begin() + indPnt1;
			if (!pnt0->HasAttribute(m_surfaceTag)||!pnt1->HasAttribute(m_surfaceTag))
				continue;
			if (!pnt0->HasAttribute(m_anchorTag)||!pnt1->HasAttribute(m_anchorTag))
				continue;
			if (pnt0->Distance(*pnt1)>2.0) continue;

			aPntNum0 = pnt0->Attribute(m_anchorTag);
			aPntNum1 = pnt1->Attribute(m_anchorTag);
			segNum0 = pnt0->Attribute(m_surfaceTag);
			segNum1 = pnt1->Attribute(m_surfaceTag);
			if (aPntNum0!=AncTagNone || aPntNum1!=AncTagNone || segNum0==segNum1) continue;
			//if (!IsInsideContour(indPnt0, indPnt1)) continue;

			ancBound = SearchAncBoundary(AnchorEDGE(indPnt0, indPnt1));
		}
	}
}

bool AnchorPointSearcher::DeriveAnchorPointAndBoundaries()
{
	//order is very important
	Initialize();
	m_Cloud.SetAttribute(m_anchorTag, int(AncTagNone));
	SearchTripleAncPnts();
	SearchDoubleAncPnts();
	SearchAncBounds();
	
#ifdef _DEBUG
	int att;
	for (LaserPoints::iterator pnt=m_Cloud.begin(); pnt!=m_Cloud.end(); pnt++) {
		att = pnt->Attribute(m_anchorTag);
		pnt->SetAttribute(ComponentNumberTag, 1000+att);
	}
	m_Cloud.Write("debug.laser");
#endif
	
	return true;
}

#include "LineSegments2D.h"

bool AnchorPointSearcher::DeriveOutlines()
{
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

	for (itrABound=m_vecAncBouns.begin(); itrABound!=m_vecAncBouns.end(); itrABound++) {
		if (IsClosedAncBound(*itrABound)) continue;//only tackle open anchor boundary
		if ((*itrABound)->HasChildren() || (*itrABound)->IsStepBound()) continue;

		aPnt0 = (*itrABound)->GetAttAncPnt(0);
		aPnt1 = (*itrABound)->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()==1) ShrinkRidge(aPnt1, aPnt0);
		if(aPnt1->AttBoundCount()==1) ShrinkRidge(aPnt0, aPnt1);
	}

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
				double angle = Angle2Lines(curContSeg->Line2DReference(), curARidge.Line2DReference());
				if(Angle2Lines(curContSeg->Line2DReference(), curARidge.Line2DReference()) > 0.010*PI/180 ) {
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
			
			if(MyIntersect2Lines(ridge, plumb, point1))
				aPnt1->SetPosition(point1);
			else {
				aPnt1->GetPosition().X() = intPntCur.X();
				aPnt1->GetPosition().Y() = intPntCur.Y();
			}
			
			//map between anchor point and ridge segment
			curARidge = LineSegment2D(aPnt0->GetPosition().vect2D(), aPnt1->GetPosition().vect2D());
			mapAPntRidge.insert(make_pair(aPnt1, *itrABound));
		}
	}


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


		}

		else if(itrMapAC0->second.size()>1) {
			int segNum2, segNum3;
			GetAttachSegNums(*aBound0, segNum0, segNum1);


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
			else {
				AnchorPoint *aPnt00 = aPnt0->GetAttBoundary()->GetAttAncPnt(0);
				AnchorPoint *aPnt01 = aPnt0->GetAttBoundary()->GetAttAncPnt(1);
				AnchorPoint *aPnt10 = aPnt1->GetAttBoundary()->GetAttAncPnt(0);
				AnchorPoint *aPnt11 = aPnt1->GetAttBoundary()->GetAttAncPnt(1);
				LineSegment2D ridge0 = LineSegment2D(aPnt00->GetPosition().vect2D(), aPnt01->GetPosition().vect2D());
				LineSegment2D ridge1 = LineSegment2D(aPnt10->GetPosition().vect2D(), aPnt11->GetPosition().vect2D());
				Position2D cross;
				Vector3D temPos;
				if(Intersect2LineSegments2D(ridge0, ridge1, cross, 0.8)) {
					if (cross.Distance(aPnt00->GetPosition().vect2D())>cross.Distance(aPnt01->GetPosition().vect2D()))
						std::swap(aPnt00, aPnt01);// make sure aPnt00 is closer to the cross than aPnt01
					if (cross.Distance(aPnt10->GetPosition().vect2D())>cross.Distance(aPnt11->GetPosition().vect2D()))
						std::swap(aPnt10, aPnt11);// make sure aPnt10 is closer to the cross than aPnt11
					temPos = aPnt00->GetPosition();
					aPnt00->SetPosition(aPnt10->GetPosition());
					aPnt10->SetPosition(temPos);
				}
			}


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


	VECANCPNTS temVecAncPntS;
	Line3D line;

	double dist0, dist1;
	for (itrMapCur=mapSegIndAncPnt.begin(); itrMapCur!=mapSegIndAncPnt.end(); itrMapCur++) {
		if(itrMapCur->second.size()==1) continue;
		point0 = contourObjPnt[contourTop[itrMapCur->first+0].Number()];
		point1 = contourObjPnt[contourTop[itrMapCur->first+1].Number()];

		temVecAncPntS = itrMapCur->second;
		
		for (unsigned int i=0; i<temVecAncPntS.size(); i++) {
			for (int j=i+1; j<temVecAncPntS.size(); j++) {

				dist0 = (point0-temVecAncPntS[i]->GetPosition()).Length2D();
				dist1 = (point0-temVecAncPntS[j]->GetPosition()).Length2D();
				if (dist0 > dist1)
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


		temObjPnt.Position3DRef() = aPnt0->GetPosition();
		temObjPnt.Number() = m_objPntOutLines.size();
		m_objPntOutLines.push_back(temObjPnt);


		temObjPnt.Position3DRef() = aPnt1->GetPosition();
		temObjPnt.Number() = m_objPntOutLines.size();
		m_objPntOutLines.push_back(temObjPnt);


		temBound.clear();
		temBound.SetAttribute(SegmentLabel, -1);
		temBound.SetAttribute(SecondSegmentLabel, surfaceNum);
		
		

		if (vecSegInds[ind0] < vecSegInds[ind1]) {
			for (int iOPnt=vecSegInds[ind0]+1; iOPnt < vecSegInds[ind1]+1; iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
		}

		else if(vecSegInds[ind0] > vecSegInds[ind1]) {
			for (int iOPnt=vecSegInds[ind0]+1; iOPnt<contourTop.size(); iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
			for (int iOPnt=1; iOPnt < vecSegInds[ind1]+1; iOPnt++)
				temBound.push_back(contourTop[iOPnt]);
		}
		

		if (surfaceNum<0 && !temBound.empty()) {
			surfaceNum = GetAttachingSurfaceSegNum(m_objPntOutLines, temBound, aPnt0, aPnt1);
			temBound.SetAttribute(SecondSegmentLabel, surfaceNum);
		}
		

		temBound.insert(temBound.begin(), PointNumber(m_objPntOutLines.size()-2));
		temBound.push_back(PointNumber(m_objPntOutLines.size()-1));
		temBound.RemoveCollinearNodes(m_objPntOutLines, false);

		m_lineTopOutLines.push_back(temBound);
	}


	if (vecBreakAPnts.empty()) {
		m_objPntOutLines.clear();
		ExportContour(m_objPntOutLines, m_lineTopOutLines);
	}

	return true;
}


bool AnchorPointSearcher::IsStoredCrosing(const std::map<AnchorPoint*, vector<AnchorPoint*> >& mapAPCross, 
	AnchorPoint* aPnt0, AnchorPoint* aPnt1)
{
	if (!aPnt0 || !aPnt1) return false;
	if (aPnt0->AttBoundCount()!=1 || aPnt1->AttBoundCount()!=1) return false;
	
	AnchorBoundary* aBound0 = aPnt0->GetAttBoundary();
	AnchorBoundary* aBound1 = aPnt1->GetAttBoundary();
	AnchorPoint* aPnt00 = aBound0->GetAttAncPnt(0);
	AnchorPoint* aPnt01 = aBound0->GetAttAncPnt(1);
	AnchorPoint* aPnt10 = aBound1->GetAttAncPnt(0);
	AnchorPoint* aPnt11 = aBound1->GetAttAncPnt(1);

	std::map<AnchorPoint*, vector<AnchorPoint*> >::const_iterator itrMapAR;
	std::vector<AnchorPoint*> vecAPnts;


	itrMapAR = mapAPCross.find(aPnt00);	
	if (itrMapAR!=mapAPCross.end()) {
		vecAPnts = itrMapAR->second;
		if (vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt10) ||
			vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt11))
			return true;
	}

	itrMapAR = mapAPCross.find(aPnt01);	
	if (itrMapAR!=mapAPCross.end()) {
		vecAPnts = itrMapAR->second;
		if (vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt10) ||
			vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt11))
			return true;
	}


	itrMapAR = mapAPCross.find(aPnt10);
	if (itrMapAR!=mapAPCross.end()) {
		vecAPnts = itrMapAR->second;
		if (vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt00) ||
			vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt01))
			return true;
	}

	itrMapAR = mapAPCross.find(aPnt11);
	if (itrMapAR!=mapAPCross.end()) {
		vecAPnts = itrMapAR->second;
		if (vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt00) ||
			vecAPnts.end() != std::find(vecAPnts.begin(), vecAPnts.end(), aPnt01))
			return true;
	}



	return false;
}

bool AnchorPointSearcher::Project2DLineToPlane()
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


int AnchorPointSearcher::GetAttachingSurfaceSegNum(AnchorPoint* aPnt0, AnchorPoint* aPnt1)
{
	vector<int> vecSegNums;
	int temSegNum;

	for (unsigned int i=0; i<aPnt0->AttPntCount(); i++) {
		temSegNum = aPnt0->GetAttPnt(i)->Attribute(m_surfaceTag);
		if (temSegNum!=-1) vecSegNums.push_back(temSegNum);
	}
	for (unsigned int i=0; i<aPnt1->AttPntCount(); i++) {
		temSegNum = aPnt1->GetAttPnt(i)->Attribute(m_surfaceTag);
		if (temSegNum!=-1) vecSegNums.push_back(temSegNum);
	}

	std::sort(vecSegNums.begin(), vecSegNums.end());
	
	int nRepeatTimes = 0;
	int repeatSegNum;
	for (unsigned int i=0; i<vecSegNums.size()-1; i++) {
		if (vecSegNums[i]==vecSegNums[i+1]) {
			repeatSegNum = vecSegNums[i];
			nRepeatTimes++;
		}
	}

	switch (nRepeatTimes) {
	case 0: return -1; break;
	case 1: return repeatSegNum; break;
	default: return -2;
	}
}



int AnchorPointSearcher::GetAttachingSurfaceSegNum(
	const ObjectPoints& objPnts, const LineTopology& lineTop, AnchorPoint* aPnt0, AnchorPoint* aPnt1)
{
	std::vector<int> vecSegNums;
	int indPnt;
	LASITR curPnt;
	for (unsigned int i=0; i<lineTop.size(); i++) {
		indPnt = lineTop[i].Number();
		assert(indPnt>=0 && indPnt<m_Cloud.size());
		curPnt = m_Cloud.begin()+indPnt;
		assert(curPnt->HasAttribute(m_surfaceTag));
		vecSegNums.push_back(curPnt->Attribute(m_surfaceTag));
	}

	int fqtValue = MaxFrequentValue(vecSegNums);
	//assert(fqtValue>=0);
	return fqtValue;
}


std::vector<bool> AnchorPointSearcher::DetectMessyRidges(LineSegments2D& allRidges)
{
	std::vector<bool> vecMessyRides = std::vector<bool>(allRidges.size(), false);
	vector<int> temCLineInds0, temCLineInds1;//cross line
	vector<vector<int> > allCLineInds = vector<vector<int> >(allRidges.size(), temCLineInds0);//cross line
	Position2D crossPnt;
	double scalar;

	//crossing times
	for (unsigned int i=0; i<allRidges.size(); i++)	{
		const LineSegment2D& seg0 = allRidges[i];

		for (int j=i+1; j<allRidges.size(); j++) {
			const LineSegment2D& seg1 = allRidges[j];
			if (!Intersection2Lines(seg0.Line2DReference(), seg1.Line2DReference(), crossPnt))
				continue;//parallel
			scalar = seg0.Scalar(crossPnt);
			if (scalar<=seg0.ScalarBegin() || scalar>=seg0.ScalarEnd())
				continue;//out of segment0, end points are ignored
			scalar = seg1.Scalar(crossPnt);
			if (scalar<=seg1.ScalarBegin() || scalar>=seg1.ScalarEnd())
				continue;//out of segment 1, end points are ignored
			allCLineInds[i].push_back(j);
			allCLineInds[j].push_back(i);
		}
	}

	//check crossing same line
	for (unsigned int i=0; i<allCLineInds.size(); i++) {
		temCLineInds0 = allCLineInds[i];

		for (int j=0; j<temCLineInds0.size(); j++) {
			LineSegment2D& seg0 = allRidges[temCLineInds0[j]];
			for (int k=j+1; k<temCLineInds0.size(); k++) {
				LineSegment2D& seg1 = allRidges[temCLineInds0[k]];
				if (!(seg0.Line2DReference()==seg1.Line2DReference())) continue;
				allCLineInds[i].erase(allCLineInds[i].begin()+k);
				k--;
			}
		}
	}


	LineSegment2D newRidge;
	for (unsigned int i=0; i<allCLineInds.size(); i++) {
		if (vecMessyRides[i]) continue;//is already assigned to be messy
		temCLineInds0 = allCLineInds[i];

		//one line crosses several ones, then this line is messy
		if (temCLineInds0.size()>1)
			vecMessyRides[i] = true;
		else if (temCLineInds0.size()==1) {
			temCLineInds1 = allCLineInds[temCLineInds0[0]];
			//only cross one line, but that line crosses several ones
			//then this line is clean
			if (temCLineInds1.size()>1) continue;
			const LineSegment2D& seg0 = allRidges[i];
			const LineSegment2D& seg1 = allRidges[temCLineInds0[0]];

			Intersection2Lines(seg0.Line2DReference(), seg1.Line2DReference(), crossPnt);
			double dist0 = crossPnt.Distance(seg0.EndPoint());
			double dist1 = crossPnt.Distance(seg1.EndPoint());

			if (dist0<dist1) {//seg0 is clean
				vecMessyRides[temCLineInds0[0]] = true;
				//change the end point, to output it
				newRidge = LineSegment2D(seg1.BeginPoint(), seg0.EndPoint());
				allRidges[temCLineInds0[0]] = newRidge;
			}
			else {//seg1 is clean
				vecMessyRides[i] = true;
				//change the end point, to output it
				newRidge = LineSegment2D(seg0.BeginPoint(), seg1.EndPoint());
				allRidges[i] = newRidge;
			}
		}
	}

	return vecMessyRides;
}



//fix all planes
bool AnchorPointSearcher::RepairShortRidge10()
{

	return true;
}


bool AnchorPointSearcher::RepairShortRidge1()
{
	AnchorBoundary * curABound, *temABound;
	vector<AnchorBoundary*> vecNeibABound;
	AnchorPoint *aPnt0, *aPnt1;
	LineSegment2D lineSeg0, lineSeg1;
	bool bIsCrossing;
	int segNum0, segNum1;
	double ridgeLen;

	std::vector< std::list<AnchorPoint*> > vecUnstableAPntSets;
	typedef std::vector< std::list<AnchorPoint*> >::iterator ITRAPntSet;


	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;//this is a sub boundary, already split
		if (IsClosedAncBound(curABound)) continue;
		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		ridgeLen = (aPnt0->GetPosition()-aPnt1->GetPosition()).Length2D();
		GetAttachSegNums(*curABound, segNum0, segNum1);

		if (ridgeLen>0.5) continue;
		if (aPnt0->AttBoundCount()!=3 || aPnt1->AttBoundCount()!=3) continue;
		
		vecNeibABound.clear();
		for (unsigned int i=0; i<3; i++) {
			temABound = aPnt0->GetAttBoundary(i);
			if (temABound == curABound) continue;
			vecNeibABound.push_back(temABound);
		}


		for (unsigned int i=0; i<3; i++) {
			temABound = aPnt1->GetAttBoundary(i);
			if (temABound == curABound) continue;
			vecNeibABound.push_back(temABound);
		}

		if(vecNeibABound.size()!=4) continue;


		bIsCrossing = false;
		int segNum00, segNum01, segNum10, segNum11;
		AnchorPoint *aPnt00, *aPnt01, *aPnt10, *aPnt11;
		for (unsigned int i=0; i<4; i++) {
			GetAttachSegNums(*vecNeibABound[i], segNum00, segNum01);
			aPnt00 = vecNeibABound[i]->GetAttAncPnt(0);
			aPnt01 = vecNeibABound[i]->GetAttAncPnt(1);
			if (aPnt00->GetPosition()==aPnt01->GetPosition()) continue;//zero length ridge
			lineSeg0 = LineSegment2D(aPnt00->GetPosition().vect2D(), aPnt01->GetPosition().vect2D());

			for (int j=i+1; j<4; j++) {
				if ((i==0&&j==1) || (i==2&&j==3)) continue;
				GetAttachSegNums(*vecNeibABound[j], segNum10, segNum11);
				aPnt10 = vecNeibABound[j]->GetAttAncPnt(0);
				aPnt11 = vecNeibABound[j]->GetAttAncPnt(1);
				if (aPnt10->GetPosition()==aPnt11->GetPosition()) continue;//zero length ridge
			
				if (aPnt00==aPnt10||aPnt00==aPnt11||aPnt01==aPnt10||aPnt01==aPnt11) continue;

				lineSeg1 = LineSegment2D(aPnt10->GetPosition().vect2D(), aPnt11->GetPosition().vect2D());
				if(!MyIsCrossing(lineSeg0, lineSeg1)) continue;
				bIsCrossing = true;
				break;
			}

			if (bIsCrossing) break;
		}

		if (!bIsCrossing) continue;//only check the situation that neighboring boundaries cross


		ITRAPntSet itrAPntSet0, itrAPntSet1;
		itrAPntSet0 = itrAPntSet1 = vecUnstableAPntSets.end();
		for (ITRAPntSet itrSet=vecUnstableAPntSets.begin(); itrSet!=vecUnstableAPntSets.end(); itrSet++)	{
			if (std::find(itrSet->begin(), itrSet->end(), aPnt0)!=itrSet->end())
				itrAPntSet0 = itrSet;

			if (std::find(itrSet->begin(), itrSet->end(), aPnt1)!=itrSet->end())
				itrAPntSet1 = itrSet;
		}

		if (itrAPntSet0!=vecUnstableAPntSets.end() && itrAPntSet1==vecUnstableAPntSets.end())
			itrAPntSet0->push_back(aPnt1);
		else if (itrAPntSet0==vecUnstableAPntSets.end() && itrAPntSet1!=vecUnstableAPntSets.end())
			itrAPntSet1->push_back(aPnt0);
		else if(itrAPntSet0==vecUnstableAPntSets.end() && itrAPntSet1==vecUnstableAPntSets.end()) {
			std::list<AnchorPoint*> temAPntSet;
			temAPntSet.push_back(aPnt0);
			temAPntSet.push_back(aPnt1);
			vecUnstableAPntSets.push_back(temAPntSet);
		}
	}


	for (ITRAPntSet itrSet=vecUnstableAPntSets.begin(); itrSet!=vecUnstableAPntSets.end(); itrSet++) {
	
		Position3D newPos(0.0, 0.0, 0.0);
		int nPnt = itrSet->size();
		for (std::list<AnchorPoint*>::iterator itr=itrSet->begin(); itr!=itrSet->end(); itr++)
			newPos += (*itr)->GetPosition()/nPnt;

	
		for (std::list<AnchorPoint*>::iterator itr=itrSet->begin(); itr!=itrSet->end(); itr++)
			(*itr)->SetPosition(newPos);
	}

	return true;
}


bool AnchorPointSearcher::RepairShortRidge()
{
	AnchorBoundary * curABound, *temABound;
	vector<AnchorBoundary*> vecNeibABound;
	AnchorPoint *aPnt0, *aPnt1;
	LineSegment2D lineSeg0, lineSeg1;
	bool bIsCrossing;
	int segNum0, segNum1;
	std::multimap<AnchorPoint*, AnchorPoint*> mapMergingAPntPairs;
	std::map<AnchorPoint*, bool> mapUnstableAPnts;


	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;
		if (IsClosedAncBound(curABound)) continue;
		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		GetAttachSegNums(*curABound, segNum0, segNum1);
		if (aPnt0->AttBoundCount()!=3 || aPnt1->AttBoundCount()!=3) continue;
		
		vecNeibABound.clear();
		for (unsigned int i=0; i<3; i++) {
			temABound = aPnt0->GetAttBoundary(i);
			if (temABound == curABound) continue;
			vecNeibABound.push_back(temABound);
		}

		
		for (unsigned int i=0; i<3; i++) {
			temABound = aPnt1->GetAttBoundary(i);
			if (temABound == curABound) continue;
			vecNeibABound.push_back(temABound);
		}

		if(vecNeibABound.size()!=4) continue;

	
		bIsCrossing = false;
		int segNum00, segNum01, segNum10, segNum11;
		AnchorPoint *aPnt00, *aPnt01, *aPnt10, *aPnt11;
		for (unsigned int i=0; i<4; i++) {
			GetAttachSegNums(*vecNeibABound[i], segNum00, segNum01);
			aPnt00 = vecNeibABound[i]->GetAttAncPnt(0);
			aPnt01 = vecNeibABound[i]->GetAttAncPnt(1);
			if (aPnt00->GetPosition()==aPnt01->GetPosition()) continue;//zero length ridge
			lineSeg0 = LineSegment2D(aPnt00->GetPosition().vect2D(), aPnt01->GetPosition().vect2D());

			for (int j=i+1; j<4; j++) {
				if ((i==0&&j==1) || (i==2&&j==3)) continue;
				GetAttachSegNums(*vecNeibABound[j], segNum10, segNum11);
				aPnt10 = vecNeibABound[j]->GetAttAncPnt(0);
				aPnt11 = vecNeibABound[j]->GetAttAncPnt(1);
				if (aPnt10->GetPosition()==aPnt11->GetPosition()) continue;//zero length ridge
				//same point
				if (aPnt00==aPnt10||aPnt00==aPnt11||aPnt01==aPnt10||aPnt01==aPnt11) continue;

				lineSeg1 = LineSegment2D(aPnt10->GetPosition().vect2D(), aPnt11->GetPosition().vect2D());
				if(!MyIsCrossing(lineSeg0, lineSeg1)) continue;
				bIsCrossing = true;
				break;//stop searching
			}

			if (bIsCrossing) break;//stop searching
		}

		if (!bIsCrossing) continue;//only check the situation that neighboring boundaries cross

		mapMergingAPntPairs.insert(make_pair(aPnt0, aPnt1));
		mapMergingAPntPairs.insert(make_pair(aPnt1, aPnt0));
		mapUnstableAPnts.insert(make_pair(aPnt0, false));
		mapUnstableAPnts.insert(make_pair(aPnt1, false));

	}


	std::set<AnchorPoint*> temAPntGroup;
	std::map<AnchorPoint*, bool>::iterator itrUnAPnt;
	typedef std::multimap<AnchorPoint*, AnchorPoint*>::iterator PntPairMapItr;
	std::pair<PntPairMapItr, PntPairMapItr> PosPair;
	std::stack<AnchorPoint*> stackAPnts;

	for (itrUnAPnt=mapUnstableAPnts.begin(); itrUnAPnt!=mapUnstableAPnts.end(); itrUnAPnt++) {
		if (itrUnAPnt->second == true) continue;
		temAPntGroup.clear();
		stackAPnts.push(itrUnAPnt->first);


		while (!stackAPnts.empty()) {
			aPnt0 = stackAPnts.top();
			stackAPnts.pop();
			temAPntGroup.insert(aPnt0);

			PosPair = mapMergingAPntPairs.equal_range(aPnt0);
			while (PosPair.first != PosPair.second) {
				std::pair<std::set<AnchorPoint*>::iterator, bool> insRst;
				insRst = temAPntGroup.insert(PosPair.first->second);
				if (insRst.second == true) //new point
					stackAPnts.push(PosPair.first->second);
				PosPair.first++;
			}
		}
		
	
		Position3D newPos(0.0, 0.0, 0.0);
		for (std::set<AnchorPoint*>::iterator itr=temAPntGroup.begin(); itr!=temAPntGroup.end(); itr++)
			newPos += (*itr)->GetPosition()/temAPntGroup.size();
		
	
		for (std::set<AnchorPoint*>::iterator itr=temAPntGroup.begin(); itr!=temAPntGroup.end(); itr++) {
			(*itr)->SetPosition(newPos);
			mapUnstableAPnts[*itr] = true;
		}
	}

	return true;
}



bool AnchorPointSearcher::SplitAncBounds()
{
	////SplitAncBoundsZeroLenth();
	SplitAncBoundsOO();
	SplitAncBoundsCO();
	SplitAncBoundsCC();
	return true;
}


void AnchorPointSearcher::SplitAncBoundsCCOblique()
{
	double dist_threshold = 1.8;
	AnchorBoundary *curABound=NULL, *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	int segNum0, segNum1;
	Line3D ridge, auxLine, plumb;
	LASITR lasPnt0, lasPnt1;
	vector<AnchorEDGE> allEdges, subAEdges0, subAEdges1;
	vector<AnchorPoint*> vecNewAPnts;
	Position3D startPnt, endPnt, midPnt, endPntAux0, endPntAux1;
	Plane plane0, plane1;
	Vector3D auxDir;
	int indABound;

	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;//this is a sub boundary, already split
		if (IsClosedAncBound(curABound)) continue;

		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()!=3 || aPnt1->AttBoundCount()!=3) continue;//only both end nodes are closed
		if (aPnt0->IsStable() && aPnt1->IsStable()) continue;//both end points are stable, no need to split
		if(!curABound->GetRidgeLine(ridge)) continue;
		if (ridge.IsHorizontal(15*3.1415926/180)) continue;//only oblique ridge

		GetAttachSegNums(*curABound, segNum0, segNum1);
		allEdges = curABound->GetAncEdges();
		if (allEdges.size()<=3) continue;//no need to split

		startPnt = aPnt0->GetPosition();
		endPnt = aPnt1->GetPosition();
		//////////////////////////////////////////////////////////////////////////
		//choose the nearest point to the ridge as start point
		if (ridge.DistanceToPoint(startPnt) > ridge.DistanceToPoint(endPnt)) {
			std::swap(startPnt, endPnt);
			std::swap(aPnt0, aPnt1);
			std::reverse(allEdges.begin(), allEdges.end());
		}
		
		//////////////////////////////////////////////////////////////////////////
		//choose the support plane
		if (!ridge.PointOnLine(startPnt, 0.0001)) ridge = Line3D(startPnt, ridge.Direction());
		plumb = Line3D(endPnt, Vector3D(0.0, 0.0, 1.0));
		plane0 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum0)];
		plane1 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum1)];
		IntersectLine3DPlane(plumb, plane0, endPntAux0);
		IntersectLine3DPlane(plumb, plane1, endPntAux1);
		if (endPntAux0.Z()>endPntAux1.Z()) {//choose the lowest plane
			std::swap(plane0, plane1);
			std::swap(endPntAux0, endPntAux1);
		}

		//////////////////////////////////////////////////////////////////////////
		//get the position of the mid point
		auxDir = plane0.Normal().VectorProduct(Vector3D(0.0, 0.0, 1.0)).VectorProduct(plane0.Normal());
		auxLine = Line3D(endPntAux0, auxDir);
		Intersection2Lines(ridge, auxLine, midPnt);
		
		
		double dist = midPnt.Position2DOnly().Distance(startPnt.Position2DOnly());
		if (dist > 0.3)
		{
			subAEdges0 = vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+allEdges.size()/2);
			subAEdges1 = vector<AnchorEDGE>(allEdges.begin()+allEdges.size()/2, allEdges.end());
			double score0 = EvaluateBoundaryStabality(subAEdges0, aPnt0->GetPosition(), midPnt);
			double score1 = EvaluateBoundaryStabality(subAEdges1, midPnt, aPnt1->GetPosition());
			if (score0<0.5 || score1<0.5) continue;

			//////////////////////////////////////////////////////////////////////////
			//new anchor point
			lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum0;
			lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum1;
			aPnt2 = NewAnchorPnt(lasPnt0, lasPnt1);
			aPnt2->SetPosition(midPnt);

			//////////////////////////////////////////////////////////////////////////
			//new sub-anchor boundary 0
			indABound = aPnt0->GetAncBoundInd(curABound);
			subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges0, indABound, 0);
			curABound->AddChild(subABound);

			//new sub-anchor boundary 1
			indABound = aPnt1->GetAncBoundInd(curABound);
			subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges1, 1, indABound);
			curABound->AddChild(subABound);
		}
	
		else {
			//////////////////////////////////////////////////////////////////////////
			//get the two points on the upper plane
			auxDir = plane1.Normal().VectorProduct(Vector3D(0.0, 0.0, 1.0)).VectorProduct(plane1.Normal());
			auxLine = Line3D(endPnt, auxDir);
			segNum1 = plane1.Number();

			//get the lowest point one plane1
			std::vector<double> vecHeights;
			for (unsigned int i=0; i<allEdges.size(); i++) {
				lasPnt0 = m_Cloud.begin()+allEdges[i].pntNum0;
				lasPnt1 = m_Cloud.begin()+allEdges[i].pntNum1;
				if (lasPnt0->Attribute(m_surfaceTag)==segNum1)
					vecHeights.push_back(lasPnt0->Z());
				else
					vecHeights.push_back(lasPnt1->Z());
			}

			std::sort(vecHeights.begin(), vecHeights.end());
			double minHei = vecHeights[int(0.05*vecHeights.size())];
			Position3D midPnt0 = ridge.DetPositionZ(minHei);
			Position3D midPnt1 = auxLine.DetPositionZ(minHei);
			
			int pos0 = allEdges.size()/3;
			int pos1 = allEdges.size()*2/3;

			//new anchor point mid0
			lasPnt0 = m_Cloud.begin()+allEdges[pos0].pntNum0;
			lasPnt1 = m_Cloud.begin()+allEdges[pos0].pntNum1;
			AnchorPoint* aPntMid0 = NewAnchorPnt(lasPnt0, lasPnt1);
			aPntMid0->SetPosition(midPnt0);

			//new anchor point mid1
			lasPnt0 = m_Cloud.begin()+allEdges[pos1].pntNum0;
			lasPnt1 = m_Cloud.begin()+allEdges[pos1].pntNum1;
			AnchorPoint* aPntMid1 = NewAnchorPnt(lasPnt0, lasPnt1);
			aPntMid1->SetPosition(midPnt1);

			
			subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+pos0);
			indABound = aPnt0->GetAncBoundInd(curABound);
			subABound = NewAnchorBound(aPnt0, aPntMid0, subAEdges0, indABound, 0);
			curABound->AddChild(subABound);

			
			subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin()+pos0, allEdges.begin()+pos1);
			subABound = NewAnchorBound(aPntMid0, aPntMid1, subAEdges0, 1, 0);
			curABound->AddChild(subABound);

			
			subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin()+pos1, allEdges.end());
			indABound = aPnt1->GetAncBoundInd(curABound);
			subABound = NewAnchorBound(aPntMid1, aPnt1, subAEdges0, 1, indABound);
			curABound->AddChild(subABound);
		}
	}
}

void AnchorPointSearcher::SplitAncBoundsCCOblique(AnchorBoundary* inBound)
{
	if (!inBound) return;


	AnchorBoundary *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	int segNum0, segNum1;
	Line3D ridge, auxLine, plumb;
	LASITR lasPnt0, lasPnt1;
	vector<AnchorEDGE> allEdges, subAEdges0, subAEdges1;

	Position3D startPnt, endPnt, midPnt, endPntAux0, endPntAux1;
	Plane plane0, plane1;
	Vector3D auxDir;
	int indABound;


	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0->AttBoundCount()==3 || aPnt1->AttBoundCount()==3);//only both end nodes are closed
	assert(!aPnt0->IsStable() || !aPnt1->IsStable());//both end points are stable, no need to split
	inBound->GetRidgeLine(ridge);
	assert (!ridge.IsHorizontal(15*3.1415926/180));//only oblique ridge

	GetAttachSegNums(*inBound, segNum0, segNum1);
	allEdges = inBound->GetAncEdges();
	if (allEdges.size()<=3) return;//no need to split

	startPnt = aPnt0->GetPosition();
	endPnt = aPnt1->GetPosition();
	if (ridge.DistanceToPoint(startPnt) > ridge.DistanceToPoint(endPnt)) {
		std::swap(startPnt, endPnt);
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}

	if (!ridge.PointOnLine(startPnt, 0.0001)) ridge = Line3D(startPnt, ridge.Direction());
	plumb = Line3D(endPnt, Vector3D(0.0, 0.0, 1.0));
	plane0 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum0)];
	plane1 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum1)];
	IntersectLine3DPlane(plumb, plane0, endPntAux0);
	IntersectLine3DPlane(plumb, plane1, endPntAux1);
	if (endPntAux0.Z()>endPntAux1.Z()) {//choose the lowest plane
		std::swap(plane0, plane1);
		std::swap(endPntAux0, endPntAux1);
	}

	//////////////////////////////////////////////////////////////////////////
	//get the position of the mid point
	auxDir = plane0.Normal().VectorProduct(Vector3D(0.0, 0.0, 1.0)).VectorProduct(plane0.Normal());
	auxLine = Line3D(endPntAux0, auxDir);
	Intersection2Lines(ridge, auxLine, midPnt);


	double dist = midPnt.Position2DOnly().Distance(startPnt.Position2DOnly());
	if (dist > 0.3)
	{
		subAEdges0 = vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+allEdges.size()/2);
		subAEdges1 = vector<AnchorEDGE>(allEdges.begin()+allEdges.size()/2, allEdges.end());
		double score0 = EvaluateBoundaryStabality(subAEdges0, aPnt0->GetPosition(), midPnt);
		double score1 = EvaluateBoundaryStabality(subAEdges1, midPnt, aPnt1->GetPosition());
		if (score0<0.5 || score1<0.5) return;

	
		lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum1;
		aPnt2 = NewAnchorPnt(lasPnt0, lasPnt1);
		aPnt2->SetPosition(midPnt);

	
		indABound = aPnt0->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges0, indABound, 0);
		inBound->AddChild(subABound);

		//new sub-anchor boundary 1
		indABound = aPnt1->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges1, 1, indABound);
		inBound->AddChild(subABound);
	}

	else {
		
		auxDir = plane1.Normal().VectorProduct(Vector3D(0.0, 0.0, 1.0)).VectorProduct(plane1.Normal());
		auxLine = Line3D(endPnt, auxDir);
		segNum1 = plane1.Number();

		
		std::vector<double> vecHeights;
		for (unsigned int i=0; i<allEdges.size(); i++) {
			lasPnt0 = m_Cloud.begin()+allEdges[i].pntNum0;
			lasPnt1 = m_Cloud.begin()+allEdges[i].pntNum1;
			if (lasPnt0->Attribute(m_surfaceTag)==segNum1)
				vecHeights.push_back(lasPnt0->Z());
			else
				vecHeights.push_back(lasPnt1->Z());
		}

		std::sort(vecHeights.begin(), vecHeights.end());
		double minHei = vecHeights[int(0.05*vecHeights.size())];
		Position3D midPnt0 = ridge.DetPositionZ(minHei);
		Position3D midPnt1 = auxLine.DetPositionZ(minHei);

		int pos0 = allEdges.size()/3;
		int pos1 = allEdges.size()*2/3;

		//new anchor point mid0
		lasPnt0 = m_Cloud.begin()+allEdges[pos0].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[pos0].pntNum1;
		AnchorPoint* aPntMid0 = NewAnchorPnt(lasPnt0, lasPnt1);
		aPntMid0->SetPosition(midPnt0);

		
		lasPnt0 = m_Cloud.begin()+allEdges[pos1].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[pos1].pntNum1;
		AnchorPoint* aPntMid1 = NewAnchorPnt(lasPnt0, lasPnt1);
		aPntMid1->SetPosition(midPnt1);

	
		subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+pos0);
		indABound = aPnt0->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt0, aPntMid0, subAEdges0, indABound, 0);
		inBound->AddChild(subABound);

		//new sub-anchor boundary 1
		subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin()+pos0, allEdges.begin()+pos1);
		subABound = NewAnchorBound(aPntMid0, aPntMid1, subAEdges0, 1, 0);
		inBound->AddChild(subABound);

		//new sub-anchor boundary 2
		subAEdges0 = std::vector<AnchorEDGE>(allEdges.begin()+pos1, allEdges.end());
		indABound = aPnt1->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPntMid1, aPnt1, subAEdges0, 1, indABound);
		inBound->AddChild(subABound);
	}
	
}


void AnchorPointSearcher::SplitAncBoundsCO()
{
	double dist_threshold = 1.8;
	AnchorBoundary *curABound=NULL, *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	int segNum0, segNum1;
	Line3D ridge, auxLine, plumb;
	LASITR lasPnt0, lasPnt1;
	vector<AnchorEDGE> allEdges, subAEdges;
	vector<AnchorPoint*> vecNewAPnts;
	Position3D startPnt, endPnt, midPnt, endPntAux0, endPntAux1;
	Plane plane0, plane1, refPlane;
	Vector3D auxDir;
	int indABound;

	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;//this is a sub boundary, already split
		if (IsClosedAncBound(curABound)) continue;

		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		//one end point is close and the other one is open
		if(aPnt0->AttBoundCount()==aPnt1->AttBoundCount()) continue;
		

		GetAttachSegNums(*curABound, segNum0, segNum1);
		allEdges = curABound->GetAncEdges();
		if (allEdges.size()<=3) continue;//no need to split
		if (aPnt0->AttPntCount()!=3) {
			std::swap(aPnt0, aPnt1);
			std::reverse(allEdges.begin(), allEdges.end());
		}
		startPnt = aPnt0->GetPosition();
		endPnt = aPnt1->GetPosition();
		plane0 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum0)];
		plane1 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum1)];

	
		if(!curABound->GetRidgeLine(ridge)) {

		}
		//has ridge
		else {
			//////////////////////////////////////////////////////////////////////////
			if (!aPnt0->IsStable()) {
			
				if (!ridge.IsHorizontal(15*3.14159/180)) continue;
			
				if (ridge.PointOnLine(aPnt0->GetPosition(), 0.301)) continue;
				//anchor point should be on the ridge
				if (ridge.PointOnLine(aPnt1->GetPosition(), 0.301)) {
					
					SplitAncBoundsCOUnstableTo2(curABound);
				}
				else {
					SplitAncBoundsCOUnstableTo3(curABound);
				}
			}
	
			else {
				
				if (!ridge.IsHorizontal(15*3.14159/180))
					SplitAncBoundsCOOblique(curABound);
			}
		}

	}//all boundaries
}


void AnchorPointSearcher::SplitAncBoundsCC()
{
	AnchorBoundary *curABound=NULL;//, *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1;// *aPnt2, *aPnt3;
	int segNum0, segNum1;
	Line3D ridge, plumb;
	bool bHasRidge;
	vector<AnchorEDGE> allEdges, subAEdges;

	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;//this is a sub boundary, already split
		if (IsClosedAncBound(curABound)) continue;

		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()!=3 || aPnt1->AttBoundCount()!=3) continue;

		GetAttachSegNums(*curABound, segNum0, segNum1);
		if ((segNum0==697&&segNum1==14) || (segNum0==14&&segNum1==697))
		{
			int iii = 0;
		}
		allEdges = curABound->GetAncEdges();
		if (allEdges.size()<=3) continue;//no need to split

		if(curABound->GetRidgeLine(ridge)) {//has ridge
			double dist0 = ridge.DistanceToPoint(aPnt0->GetPosition());
			double dist1 = ridge.DistanceToPoint(aPnt1->GetPosition());

			if (aPnt0->IsStable() && aPnt1->IsStable() ) continue;
			
			if (!aPnt0->IsStable() && !aPnt1->IsStable() &&
				dist0>0.2 && dist1>0.2 && ridge.IsHorizontal(15*3.1415926/180)) {
					SplitAncBoundsCCTo3Horizontal(curABound);
			}
			else if(aPnt0->IsStable()!=aPnt1->IsStable() && !ridge.IsHorizontal(15*3.1415926/180)) {
				SplitAncBoundsCCOblique(curABound);
			}
		}
	}//all boundaries
}

void AnchorPointSearcher::SplitAncBoundsOO()
{
	double dist_threshold = 1.8;
	AnchorBoundary *curABound=NULL, *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
	int segNum0, segNum1;
	Plane plane0, plane1;
//	int indPlane0, indPlane1;
	Line3D ridge, plumb;
	bool bHasRidge;
	LASITR lasPnt0, lasPnt1;
	vector<AnchorEDGE> allEdges, subAEdges;
	vector<Position3D> vecPnts;
	Position3D temPnt, temPntProj;
	temPnt.Z() = 0.0;
	vector<int> vecContour;
	int pos0, pos1, posTem;
	Line3D line;
	vector<double> vecDists;
	vector<AnchorPoint*> vecNewAPnts;

	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (curABound->HasFather()) continue;//this is a sub boundary, already split
		if (IsClosedAncBound(curABound)) continue;

		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		if(aPnt0->AttBoundCount()>1 || aPnt1->AttBoundCount()>1) continue;

		GetAttachSegNums(*curABound, segNum0, segNum1);
		allEdges = curABound->GetAncEdges();
		if (allEdges.size()<=3) continue;//no need to split

		if(curABound->GetRidgeLine(ridge)) {//has ridge
			double length0 = (aPnt0->GetPosition()-aPnt1->GetPosition()).Length();
			double length1 = fabs(ridge.Scalar(aPnt0->GetPosition())-ridge.Scalar(aPnt1->GetPosition()));

			double dist0 = ridge.DistanceToPoint(aPnt0->GetPosition());
			double dist1 = ridge.DistanceToPoint(aPnt1->GetPosition());

			 if(length0<0.05 || length1<0.05)
				CloseAncBound(curABound);
			else
				SplitAncBoundsOOToX(curABound);

		
		}
		else {//parallel planes
			vecPnts.clear();
			for (int iEdge=0; iEdge<allEdges.size(); iEdge++) {
				lasPnt0 = m_Cloud.begin()+allEdges[iEdge].pntNum0;
				lasPnt1 = m_Cloud.begin()+allEdges[iEdge].pntNum1;
				temPnt.X() = (lasPnt0->X() + lasPnt1->X());
				temPnt.Y() = (lasPnt0->Y() + lasPnt1->Y());
				vecPnts.push_back(temPnt);
			}

			//find far points to be the nodes of the new contour
			vecContour.clear();
			vecContour.push_back(0);
			vecContour.push_back(allEdges.size()-1);
			for (int iPos=0; iPos<vecContour.size()-1; iPos++) {
				pos0 = vecContour[iPos];
				pos1 = vecContour[iPos+1];
				line = Line3D(vecPnts[pos0], vecPnts[pos1]);

				vecDists.clear();
				for (posTem=pos0; posTem<=pos1; posTem++)
					vecDists.push_back(line.DistanceToPoint(vecPnts[posTem]));

				vector<double>::iterator itrMax = std::max_element(vecDists.begin(), vecDists.end());
				if (*itrMax < dist_threshold) continue;
				posTem = pos0+(itrMax-vecDists.begin());
				vecContour.insert(vecContour.begin()+iPos+1, posTem);
				iPos--;
			}

			
			if (vecContour.size()==2) continue;//no new anchor points
#ifdef _DEBUG
			if ((segNum0==430&&segNum1==431) || (segNum0==431&&segNum1==430)) {
				WriteBoundaryEdges(allEdges);
				int aaa = 0;
			}
#endif
			vecNewAPnts.clear();
			bHasRidge = curABound->GetRidgeLine(ridge);
			//construct all anchor points of this boundary
			for (int iPos=0; iPos<vecContour.size(); iPos++) {
				if (iPos==0 && aPnt0->AttBoundCount()==3) {
					aPnt2 = aPnt0;
				}
				else if (iPos==vecContour.size()-1 && aPnt1->AttBoundCount()==3) {
					aPnt2 = aPnt1;
				}
				else {
					posTem = vecContour[iPos];
					lasPnt0 = m_Cloud.begin()+allEdges[posTem].pntNum0;
					lasPnt1 = m_Cloud.begin()+allEdges[posTem].pntNum1;
					aPnt2 = NewAnchorPnt(lasPnt0, lasPnt1);

					temPnt = (lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2.0;
					//set the position to be the projection
					if (bHasRidge && ridge.DistanceToPoint(temPnt)<dist_threshold/3.0) {
						aPnt2->SetPosition(ridge.Project(temPnt));
						aPnt2->IsStable(false);
					}
					else {
						//set the position to be the center
						aPnt2->SetPosition(temPnt);
						aPnt2->IsStable(true);
					}				
				}

				vecNewAPnts.push_back(aPnt2);
			}

		
			for (int iPos=0; iPos<vecContour.size()-1; iPos++) {
				//posTem = vecContour[iPos];
				aPnt2 = vecNewAPnts[iPos];
				aPnt3 = vecNewAPnts[iPos+1];
				subAEdges=vector<AnchorEDGE>(allEdges.begin()+vecContour[iPos], allEdges.begin()+vecContour[iPos+1]+1);

				//attach first node to old APoint
				if (aPnt2->AttBoundCount()==3)	{
					int indABound = aPnt2->GetAncBoundInd(curABound);
					assert(indABound!=-1);
					subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, indABound, iPos%2);
				}
				//attach second node to old APoint
				else if (aPnt3->AttBoundCount()==3) {
					int indABound = aPnt3->GetAncBoundInd(curABound);
					assert(indABound!=-1);
					subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, iPos%2, indABound);
				}
				else//both are new APoints
					subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, iPos%2, iPos%2);

				curABound->AddChild(subABound);
			}
		}//end parallel planes
	}//all boundaries
}


void AnchorPointSearcher::SplitAncBoundsOOTo3(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(aPnt0->IsStable() && aPnt1->IsStable());

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);

	//point 2
	lasPnt0 = m_Cloud.begin()+allEdges[0].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[0].pntNum1;
	Position3D posPnt2 = (ridge.Project(*lasPnt0)+ridge.Project(*lasPnt1))/2;
	int pos2 = int(allEdges.size()/3);
	aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
	aPnt2->SetPosition(posPnt2);
	aPnt2->IsStable(false);

	//point 3
	lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum1;
	Position3D posPnt3 = (ridge.Project(*lasPnt0)+ridge.Project(*lasPnt1))/2;
	int pos3 = int(allEdges.size()*2/3);	
	aPnt3 = NewAnchorPnt(allEdges[pos3].pntNum0, allEdges[pos3].pntNum1);
	aPnt3->SetPosition(posPnt3);
	aPnt3->IsStable(false);

	int indABound;
	AnchorBoundary* subABound;
	//boundary 0-2
	subAEdges = vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
	indABound = aPnt0->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
	inBound->AddChild(subABound);

	
	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.begin()+pos3+1);
	subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 0, 1);
	inBound->AddChild(subABound);

	//boundary 3-0
	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos3+1, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 0, indABound);
	inBound->AddChild(subABound);
}


void AnchorPointSearcher::SplitAncBoundsOOTo3SameEnde(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(!aPnt0->IsStable() && !aPnt1->IsStable());
	double dist = (aPnt0->GetPosition()-aPnt1->GetPosition()).Length();
	assert(dist<0.05);

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);

	
	Position3D pos3D0 = (aPnt0->GetAttPnt(0)->Position3DRef()+aPnt0->GetAttPnt(1)->Position3DRef())/2;
	Position3D pos3D1 = (aPnt1->GetAttPnt(0)->Position3DRef()+aPnt1->GetAttPnt(1)->Position3DRef())/2;
	double dist0 = ridge.DistanceToPoint(pos3D0);
	double dist1 = ridge.DistanceToPoint(pos3D1);
	if(dist0>dist1) {
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}

	Line2D line02 = Line2D(ridge);
	Position2D pos2D(aPnt0->GetPosition().X(), aPnt0->GetPosition().Y());
	Line2D line01 = line02.PerpendicularLine(pos2D);

	double maxDist2L02(-999.0), maxDist2L01(-999.0), temDist;
	int pos2, pos3;
	pos3D0 = aPnt0->GetPosition();
	Position3D pos3DTem;
	Position3D farPnt2L02, farPnt2L01;

	
	for (unsigned int i=0; i<allEdges.size(); i++) {
		lasPnt0 = m_Cloud.begin()+allEdges[i].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[i].pntNum1;
		pos3DTem = (lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2;
	
		temDist = line01.DistanceToPoint(pos3DTem.Position2DOnly());
		if (temDist > maxDist2L01) {
			maxDist2L01 = temDist;
			pos2 = i;
			farPnt2L01 = pos3DTem;
		}

		//line02
		temDist = line02.DistanceToPoint(pos3DTem.Position2DOnly());
		if (temDist > maxDist2L02) {
			maxDist2L02 = temDist;
			pos2 = i;
			farPnt2L02 = pos3DTem;
		}
	}


	Line2D line23 = line02.PerpendicularLine(farPnt2L01.Position2DOnly());
	Line2D line13 = line01.PerpendicularLine(farPnt2L02.Position2DOnly());
	Position2D pos2D1, pos2D2,pos2D3;
	Intersection2Lines(line02, line23, pos2D2);
	Intersection2Lines(line23, line13, pos2D3);
	Intersection2Lines(line13, line01, pos2D1);
	pos3D1 = Position3D(pos2D1.X(), pos2D1.Y(), pos3D0.Z());
	Position3D pos3D2 = Position3D(pos2D2.X(), pos2D2.Y(), pos3D0.Z());
	Position3D pos3D3 = Position3D(pos2D3.X(), pos2D3.Y(), pos3D0.Z());

	if (pos2<allEdges.size()/2)
		pos3 = (pos2+allEdges.size())/2;
	else {
		pos3 = pos2;
		pos2 /= 2;
	}

	
	aPnt1->SetPosition(pos3D1);


	aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
	aPnt2->SetPosition(pos3D2);
	aPnt2->IsStable(false);


	aPnt3 = NewAnchorPnt(allEdges[pos3].pntNum0, allEdges[pos3].pntNum1);
	aPnt3->SetPosition(pos3D3);
	aPnt3->IsStable(false);


	int indABound;
	AnchorBoundary* subABound;

	subAEdges = vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
	indABound = aPnt0->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
	inBound->AddChild(subABound);


	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.begin()+pos3+1);
	subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 0, 1);
	inBound->AddChild(subABound);


	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos3+1, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 0, indABound);
	inBound->AddChild(subABound);
}

void AnchorPointSearcher::SplitAncBoundsOOToX(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
//	if(aPnt0->IsStable() && aPnt1->IsStable()) return;

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);


	double dist0 = ridge.DistanceToPoint(aPnt0->GetPosition());
	double dist1 = ridge.DistanceToPoint(aPnt1->GetPosition());
	if (dist0>dist1) {
		std::swap(aPnt0, aPnt1);
		std::swap(dist0, dist1);
	}


	double scalar0 = ridge.Scalar(aPnt0->GetPosition());
	double scalar1 = ridge.Scalar(aPnt1->GetPosition());
	if (scalar0>scalar1) {
		ridge.Direction(-1.0*ridge.Direction());
		scalar0 *= -1.0;
		scalar1 *= -1.0;
	}
	
	//get min and max scalar along ridge
	std::vector<double> vecScales(allEdges.size(), 0.0);
	for (int i=0; i<allEdges.size(); i++) {
		Position3D temPos = m_Cloud[allEdges[i].pntNum0].Position3DRef()/2;
		temPos += m_Cloud[allEdges[i].pntNum1].Position3DRef()/2;
		vecScales[i] = ridge.Scalar(temPos);
	}

	std::vector<double>::iterator scalarMin = std::min_element(vecScales.begin(), vecScales.end());
	std::vector<double>::iterator scalarMax = std::max_element(vecScales.begin(), vecScales.end());

	int indABound;
	AnchorBoundary* subABound;
	int pos2, pos3, pos4;

	double distThresh = 0.3;


	if (fabs(scalar0-scalar1)<0.3) {
		double dist0 = ridge.DistanceToPoint(aPnt0->GetPosition());
		double dist1 = ridge.DistanceToPoint(aPnt1->GetPosition());
		if (dist0>dist1) {
			std::swap(aPnt0, aPnt1);
			std::swap(scalar0, scalar1);
		}

		//re-project aPnt0
		aPnt0->SetPosition(ridge.Project(aPnt0->GetPosition()));

		//position2: get farthest point to aPnt0
		std::vector<double> vecScalar2=vecScales;		
		for (int i=0; i<vecScalar2.size(); i++)
			vecScalar2[i] = fabs(vecScalar2[i]-scalar0);

		std::vector<double>::iterator itrScalarMaxTem = std::max_element(vecScalar2.begin(), vecScalar2.end());
		Position3D position2 = ridge.Position(vecScales[itrScalarMaxTem-vecScalar2.begin()]);
		
		//position3: get farthest point to aPnt0
		Line2D ridge2D = ridge.ProjectOntoXOYPlane();
		Line2D auxLine2D = ridge2D.PerpendicularLine(position2.Position2DOnly());
		Position2D pos02D(aPnt0->GetPosition().X(), aPnt0->GetPosition().Y());
		Position2D pos12D(aPnt1->GetPosition().X(), aPnt1->GetPosition().Y());
		if (auxLine2D.Scalar(pos02D) > auxLine2D.Scalar(pos12D))
			auxLine2D.SwapNormal();

		for (int i=0; i<allEdges.size(); i++) {
			Position3D temPos = m_Cloud[allEdges[i].pntNum0].Position3DRef()/2;
			temPos += m_Cloud[allEdges[i].pntNum1].Position3DRef()/2;
			vecScalar2[i] = auxLine2D.Scalar(temPos.Position2DOnly());
		}
		itrScalarMaxTem = std::max_element(vecScalar2.begin(), vecScalar2.end());

		Position3D position3 = position2;//use the height of position2
		position3.X() = auxLine2D.Position(*itrScalarMaxTem).X();
		position3.Y() = auxLine2D.Position(*itrScalarMaxTem).Y();

		//re-project aPnt1
		Line2D ridge2DBottom = auxLine2D.PerpendicularLine(position3.Position2DOnly());
		aPnt1->SetPosition(Vector3D(ridge2DBottom.Project(pos12D).X(), ridge2DBottom.Project(pos12D).Y(), 0.0));

		//new anchor point
		pos2 = allEdges.size()/3;
		aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
		aPnt2->SetPosition(position2);
		aPnt2->IsStable(false);

		pos3 = allEdges.size()*2/3;
		aPnt3 = NewAnchorPnt(allEdges[pos3].pntNum0, allEdges[pos3].pntNum1);
		aPnt3->SetPosition(position3);
		aPnt3->IsStable(false);

		//construct boundary line
		//boundary 0-2
		subAEdges = std::vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
		indABound = aPnt0->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
		inBound->AddChild(subABound);

		//boundary 2-3
		subAEdges = std::vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.begin()+pos3+1);
		subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 0, 1);
		inBound->AddChild(subABound);

		//boundary 3-1
		subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos3, allEdges.end());
		indABound = aPnt1->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 0, indABound);
		inBound->AddChild(subABound);
	}

	else if (dist0<=distThresh && dist1>distThresh) {//0-----2-1
		Position3D position2 = ridge.Position(*scalarMax);
		Line2D ridge2D = ridge.ProjectOntoXOYPlane();
		Line2D auxLine2D = ridge2D.PerpendicularLine(position2.Position2DOnly());
		Position2D posTem(aPnt1->GetPosition().X(), aPnt1->GetPosition().Y());
		posTem = auxLine2D.Project(posTem);
		aPnt1->SetPosition(Vector3D(posTem.X(), posTem.Y(), aPnt1->GetPosition().Z()));

		//new anchor point
		pos2 = allEdges.size()/2;
		aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
		aPnt2->SetPosition(position2);
		aPnt2->IsStable(false);

		//construct boundary line
		//boundary 0-2
		subAEdges = std::vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
		indABound = aPnt0->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
		inBound->AddChild(subABound);

		//boundary 2-1
		subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.end());
		indABound = aPnt1->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges, 0, indABound);
		inBound->AddChild(subABound);
	}

	else if (dist0>distThresh && dist1>distThresh) {
		Position3D position2 = ridge.Position(*scalarMin);
		Line2D ridge2D = ridge.ProjectOntoXOYPlane();
		Line2D auxLine2D = ridge2D.PerpendicularLine(position2.Position2DOnly());
		Position2D posTem(aPnt0->GetPosition().X(), aPnt0->GetPosition().Y());
		posTem = auxLine2D.Project(posTem);
		aPnt0->SetPosition(Vector3D(posTem.X(), posTem.Y(), aPnt0->GetPosition().Z()));

		Position3D position3 = ridge.Position(*scalarMax);
		auxLine2D = ridge2D.PerpendicularLine(position3.Position2DOnly());
		posTem = Position2D(aPnt1->GetPosition().X(), aPnt1->GetPosition().Y());
		posTem = auxLine2D.Project(posTem);
		aPnt1->SetPosition(Vector3D(posTem.X(), posTem.Y(), aPnt1->GetPosition().Z()));

		//new anchor point
		pos2 = allEdges.size()/3;
		aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
		aPnt2->SetPosition(position2);
		aPnt2->IsStable(false);

		pos3 = allEdges.size()*2/3;
		aPnt3 = NewAnchorPnt(allEdges[pos3].pntNum0, allEdges[pos3].pntNum1);
		aPnt3->SetPosition(position3);
		aPnt3->IsStable(false);

	
		subAEdges = std::vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
		indABound = aPnt0->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
		inBound->AddChild(subABound);

	
		subAEdges = std::vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.begin()+pos3+1);
		subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 0, 1);
		inBound->AddChild(subABound);

	
		subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos3, allEdges.end());
		indABound = aPnt1->GetAncBoundInd(inBound);
		subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 0, indABound);
		inBound->AddChild(subABound);
	}

}


void AnchorPointSearcher::SplitAncBoundsOOTo2(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(aPnt0->IsStable() || aPnt1->IsStable());

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);

	//point 2
	if (aPnt0->IsStable()) {
		lasPnt0 = m_Cloud.begin()+allEdges[0].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[0].pntNum1;
	}
	else {
		lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum1;
	}

	Position3D posPnt2 = (ridge.Project(*lasPnt0)+ridge.Project(*lasPnt1))/2;
	int pos2 = int(allEdges.size()/2);
	aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
	aPnt2->SetPosition(posPnt2);
	aPnt2->IsStable(false);


	int indABound;
	AnchorBoundary* subABound;
	
	subAEdges = vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
	indABound = aPnt0->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
	inBound->AddChild(subABound);

	
	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges, 0, indABound);
	inBound->AddChild(subABound);
}

     
void AnchorPointSearcher::SplitAncBoundsCOHorizontal(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(aPnt0->AttBoundCount() != aPnt1->AttBoundCount());

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);

	double dist = (aPnt0->GetPosition()-aPnt1->GetPosition()).Length();
	if (dist<3.0 || allEdges.size() < 50)
		return;

	//make sure aPnt0 is closed
	if (aPnt0->AttBoundCount() != 3) {
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}
	if (!aPnt1->IsStable())
		return;

	lasPnt0 = aPnt1->GetAttPnt(0);
	lasPnt1 = aPnt1->GetAttPnt(1);

	Position3D posPnt2 = (ridge.Project(*lasPnt0)+ridge.Project(*lasPnt1))/2;
	LaserPoint temLaspnt(posPnt2);
	if (!temLaspnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return;

	int pos2 = int(allEdges.size()/2);
	aPnt2 = NewAnchorPnt(allEdges[pos2].pntNum0, allEdges[pos2].pntNum1);
	aPnt2->SetPosition(posPnt2);
	aPnt2->IsStable(false);

	int indABound;
	AnchorBoundary* subABound;
	//boundary 0-2
	subAEdges = vector<AnchorEDGE> (allEdges.begin(), allEdges.begin()+pos2+1);
	indABound = aPnt0->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 1);
	inBound->AddChild(subABound);

	//boundary 2-1
	subAEdges = vector<AnchorEDGE> (allEdges.begin()+pos2, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges, 0, indABound);
	inBound->AddChild(subABound);
}



void AnchorPointSearcher::SplitAncBoundsCOOblique(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(aPnt0->AttBoundCount()!=aPnt1->AttBoundCount());

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);

	int segNum0, segNum1;
	inBound->GetAttachSegNums(segNum0, segNum1);
	int indSeg0 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum0);
	int indSeg1 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum1);
	if (indSeg0==-1 || indSeg1==-1) return;
	Plane& plane0 = m_vecSurPlanes[indSeg0];
	Plane& plane1 = m_vecSurPlanes[indSeg1];


	//make sure the aPnt0 is stable
	if (aPnt0->AttBoundCount()==2) {
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}

	if (!ridge.PointOnLine(aPnt0->GetPosition(), 0.0001)) 
		ridge = Line3D(aPnt0->GetPosition(), ridge.Direction());
	assert(!ridge.IsHorizontal(15*3.14159/180));

	//check lines that do not need split
	lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum1;
	double dist0 = ridge.DistanceToPoint(*lasPnt0);
	double dist1 = ridge.DistanceToPoint(*lasPnt1);
	double dist01 = ridge.DistanceToPoint((lasPnt0->Position3DRef()+lasPnt1->Position3DRef())/2);
	if ((dist01<0.5 || (dist0<1.0&&dist1<1.0))) return;


	Line3D plumb = Line3D(aPnt1->GetPosition(), Vector3D(0.0, 0.0, 1.0));
	Position3D endPntAux0, endPntAux1, midPnt;
	IntersectLine3DPlane(plumb, plane0, endPntAux0);
	IntersectLine3DPlane(plumb, plane1, endPntAux1);
	Plane refPlane = endPntAux0.Z()<endPntAux1.Z() ? plane0:plane1;

#ifdef _DEBUG
	if (segNum0 == 170&& segNum1==171)
		WriteBoundaryEdges(allEdges);
#endif

	//search for mid point index
	vector<double> vecHei, vecHeiTem;
	for (unsigned int i=0; i<allEdges.size(); i++) {
		lasPnt0 = m_Cloud.begin()+allEdges[i].pntNum0;
		lasPnt1 = m_Cloud.begin()+allEdges[i].pntNum1;
		if (lasPnt0->Attribute(m_surfaceTag)!=refPlane.Number())
			vecHei.push_back(lasPnt0->Z());
		else
			vecHei.push_back(lasPnt1->Z());
	}
	vecHeiTem = vecHei;
	std::sort(vecHeiTem.begin(), vecHeiTem.end());
	//double midPntHei = vecHeiTem[vecHeiTem.size()*0.1];
	double midPntHei = vecHeiTem[2];
	int indMidPnt = std::find(vecHei.begin(), vecHei.end(), midPntHei)-vecHei.begin();
	if (indMidPnt==0) indMidPnt = 2;

	//mid point
	Plane auxPlane = Plane(Vector3D(0.0, 0.0, midPntHei), Vector3D(0.0, 0.0, 1.0));
	IntersectLine3DPlane(ridge, auxPlane, midPnt);

	//the mid point is outside of the contour, is invalid
	LaserPoint temLasPnt(midPnt);
	if(!temLasPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return;

	//auxiliary line
	Vector3D auxDir = refPlane.Normal().VectorProduct(Vector3D(0.0, 0.0, 1.0)).VectorProduct(refPlane.Normal());
	if (auxDir.Z()>0.0) auxDir *= -1.0;
	Line3D auxLine = Line3D(midPnt, auxDir);

	//new anchor point
	lasPnt0 = m_Cloud.begin()+allEdges[indMidPnt].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[indMidPnt].pntNum1;
	aPnt2 = NewAnchorPnt(lasPnt0, lasPnt1);
	aPnt2->SetPosition(midPnt);

	//set the end point lower than the mid point
	double scalar = 0.0;
	lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()-1].pntNum1;
	if (auxLine.Scalar(lasPnt0->Position3DRef())>0.0) 
		scalar += auxLine.Scalar(lasPnt0->Position3DRef())/2;
	if (auxLine.Scalar(lasPnt1->Position3DRef())>0.0) 
		scalar += auxLine.Scalar(lasPnt1->Position3DRef())/2;
	if (scalar==0.0) scalar += 0.01;
	Position3D endPnt = auxLine.Position(scalar);
	//biao July 3, 2015
	//keep the original position so that no conflicts with outlines
	//	aPnt1->SetPosition(endPnt);
	aPnt1->IsStable(false);

	//new sub-anchor boundary 0
	subAEdges=vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+indMidPnt);
	int indABound = aPnt0->GetAncBoundInd(inBound);
	AnchorBoundary* subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 0);
	inBound->AddChild(subABound);

	//new sub-anchor boundary 1
	subAEdges=vector<AnchorEDGE>(allEdges.begin()+indMidPnt, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges, 1, indABound);
	inBound->AddChild(subABound);
}



void AnchorPointSearcher::SplitAncBoundsCOUnstableTo2(AnchorBoundary* inBound)
{
	AnchorPoint *aPnt0, *aPnt1, *aPnt2;
	int segNum0, segNum1;
	Line3D ridge, auxLine, plumb;
	LASITR lasPnt0, lasPnt1;
//	vector<AnchorEDGE> allEdges, subAEdges;
	Position3D midPnt;
	Vector3D auxDir;

	assert (!inBound->HasFather());//this is a sub boundary, already split
	assert (!IsClosedAncBound(inBound));

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	//one end point is close and the other one is open
	assert(aPnt0->AttBoundCount()!=aPnt1->AttBoundCount()) ;



	GetAttachSegNums(*inBound, segNum0, segNum1);
	vector<AnchorEDGE> allEdges = inBound->GetAncEdges();
	//	if (allEdges.size()<=3) continue;//no need to split
	if (aPnt0->AttPntCount()!=3) {
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}

	Position3D startPnt = aPnt0->GetPosition();
	Position3D endPnt = aPnt1->GetPosition();
	const Plane& plane0 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum0)];
	const Plane& plane1 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum1)];

	inBound->GetRidgeLine(ridge);

	if (!ridge.IsHorizontal(15*3.14159/180)) return;

	//anchor point 0 should not be on the ridge, but anchor point 1 is.
	assert (!ridge.PointOnLine(aPnt0->GetPosition(), 0.301));
	assert (ridge.PointOnLine(aPnt1->GetPosition(), 0.301));

	//get reference plane
	double dist0 = fabs(plane0.Distance(aPnt0->GetPosition()));
	double dist1 = fabs(plane1.Distance(aPnt0->GetPosition()));
	const Plane& refPlane = dist0<dist1 ? plane0:plane1;
	
	//new anchor point position
	auxDir = ridge.Direction().VectorProduct(refPlane.Normal());
	auxLine = Line3D(aPnt0->GetPosition(), auxDir);
	bool hasMidPnt = Intersection2Lines(ridge, auxLine, midPnt);
	assert(hasMidPnt);
	LaserPoint temPnt(midPnt);
	if (!temPnt.InsidePolygonJordan(m_contourObjPnts, m_contourTop)) return;

	//new anchor point
	lasPnt0 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum0;
	lasPnt1 = m_Cloud.begin()+allEdges[allEdges.size()/2].pntNum1;
	aPnt2 = NewAnchorPnt(lasPnt0, lasPnt1);
	aPnt2->SetPosition(midPnt);

	//new sub-anchor boundary 0
	vector<AnchorEDGE> subAEdges=vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+allEdges.size()/2);
	int indABound = aPnt0->GetAncBoundInd(inBound);
	AnchorBoundary* subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 0);
	inBound->AddChild(subABound);

	//new sub-anchor boundary 1
	subAEdges=vector<AnchorEDGE>(allEdges.begin()+allEdges.size()/2, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt2, aPnt1, subAEdges, 1, indABound);
	inBound->AddChild(subABound);
}



void AnchorPointSearcher::SplitAncBoundsCOUnstableTo3(AnchorBoundary* inBound)
{
	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
	int segNum0, segNum1;
	Line3D ridge;//, auxLine, plumb;
//	LASITR lasPnt0, lasPnt1;
//	vector<AnchorEDGE> allEdges, subAEdges;
	Position3D midPnt;
//	Vector3D auxDir;

	assert (!inBound->HasFather());//this is a sub boundary, already split
	assert (!IsClosedAncBound(inBound));

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	//one end point is close and the other one is open
	assert(aPnt0->AttBoundCount()!=aPnt1->AttBoundCount()) ;


	GetAttachSegNums(*inBound, segNum0, segNum1);
	vector<AnchorEDGE> allEdges = inBound->GetAncEdges();
	//	if (allEdges.size()<=3) continue;//no need to split
	if (aPnt0->AttPntCount()!=3) {
		std::swap(aPnt0, aPnt1);
		std::reverse(allEdges.begin(), allEdges.end());
	}

	assert(!aPnt0->IsStable());
	Position3D startPnt = aPnt0->GetPosition();
	Position3D endPnt = aPnt1->GetPosition();
	const Plane& plane0 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum0)];
	const Plane& plane1 = (m_vecSurPlanes)[IndexPlaneBySegNumber(m_vecSurPlanes, segNum1)];

	inBound->GetRidgeLine(ridge);

	if (!ridge.IsHorizontal(15*3.14159/180)) return;
	if (!IsInsideContour(*inBound, ridge)) return;
	if (ComputeRidgeCoef(*inBound, ridge)<0.5) return;

	//anchor point 0 should not be on the ridge, but anchor point 1 is.
	assert (!ridge.PointOnLine(aPnt0->GetPosition(), 0.001));
	assert (!ridge.PointOnLine(aPnt1->GetPosition(), 0.001));

	//get reference plane
	double dist0 = fabs(plane0.Distance(aPnt0->GetPosition()));
	double dist1 = fabs(plane1.Distance(aPnt0->GetPosition()));
	const Plane& refPlane = dist0<dist1 ? plane0:plane1;

	//new anchor points
	int indPnt2 = allEdges.size()/3;
	aPnt2 = NewAnchorPnt(allEdges[indPnt2].pntNum0, allEdges[indPnt2].pntNum1);
	aPnt2->SetPosition(ridge.Project(aPnt0->GetPosition()));

	int indPnt3 = allEdges.size()*2/3;
	aPnt3 = NewAnchorPnt(allEdges[indPnt3].pntNum0, allEdges[indPnt3].pntNum1);
	aPnt3->SetPosition(ridge.Project(aPnt1->GetPosition()));

	//new sub-anchor boundaries
	vector<AnchorEDGE>subAEdges=vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+indPnt2+1);
	int indABound = aPnt0->GetAncBoundInd(inBound);
	AnchorBoundary* subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 0);
	inBound->AddChild(subABound);

	subAEdges=vector<AnchorEDGE>(allEdges.begin()+indPnt2, allEdges.begin()+indPnt3+1);
	subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 1, 0);
	inBound->AddChild(subABound);

	subAEdges=vector<AnchorEDGE>(allEdges.begin()+indPnt3+1, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 1, indABound);
	inBound->AddChild(subABound);
}


void AnchorPointSearcher::SplitAncBoundsCCTo3Horizontal(AnchorBoundary* inBound)
{
	assert(inBound);

	AnchorPoint *aPnt0, *aPnt1, *aPnt2, *aPnt3;
//	LASITR lasPnt0, lasPnt1;
	Line3D ridge;

	aPnt0 = inBound->GetAttAncPnt(0);
	aPnt1 = inBound->GetAttAncPnt(1);
	assert(aPnt0 && aPnt1);
	assert(aPnt0->AttBoundCount()==3 && aPnt1->AttBoundCount()==3);

	std::vector<AnchorEDGE> allEdges, subAEdges;
	allEdges = inBound->GetAncEdges();
	bool bHasRdige = inBound->GetRidgeLine(ridge);
	assert(bHasRdige);
	assert(ridge.IsHorizontal(15*3.14159/180));


	//new anchor points
	int indPnt2 = allEdges.size()/3;
	aPnt2 = NewAnchorPnt(allEdges[indPnt2].pntNum0, allEdges[indPnt2].pntNum1);
	aPnt2->SetPosition(ridge.Project(aPnt0->GetPosition()));

	int indPnt3 = allEdges.size()*2/3;
	aPnt3 = NewAnchorPnt(allEdges[indPnt3].pntNum0, allEdges[indPnt3].pntNum1);
	aPnt3->SetPosition(ridge.Project(aPnt1->GetPosition()));

	//new sub-anchor boundaries
	subAEdges=vector<AnchorEDGE>(allEdges.begin(), allEdges.begin()+indPnt2+1);
	int indABound = aPnt0->GetAncBoundInd(inBound);
	AnchorBoundary* subABound = NewAnchorBound(aPnt0, aPnt2, subAEdges, indABound, 0);
	inBound->AddChild(subABound);

	subAEdges=vector<AnchorEDGE>(allEdges.begin()+indPnt2, allEdges.begin()+indPnt3+1);
	subABound = NewAnchorBound(aPnt2, aPnt3, subAEdges, 1, 0);
	inBound->AddChild(subABound);

	subAEdges=vector<AnchorEDGE>(allEdges.begin()+indPnt3+1, allEdges.end());
	indABound = aPnt1->GetAncBoundInd(inBound);
	subABound = NewAnchorBound(aPnt3, aPnt1, subAEdges, 1, indABound);
	inBound->AddChild(subABound);
}


void AnchorPointSearcher::SplitAncBoundsStepEdge()
{
	AnchorBoundary *curABound=NULL, *subABound=NULL;
	AnchorPoint *aPnt0, *aPnt1;
//	AnchorPoint *pAPnt00, *pAPnt01;
	int segNum0, segNum1;
	Plane plane0, plane1;
	int indPlane0, indPlane1;
	Line3D plumb;
	//LASITR lasPnt0, lasPnt1;
	Position3D posProj00, posProj01, posProj10, posProj11, posCur;
	bool bOnRidge0, bOnRidge1;

	for (int iABound=0; iABound<m_vecAncBouns.size(); iABound++) {
		curABound = m_vecAncBouns[iABound];
		if (IsClosedAncBound(curABound)) continue;
		if (curABound->HasChildren()) continue;//father boundary does not need to split
		if (curABound->IsStepBound()) continue;//already step edge. slit in unstable aPoints partition

		aPnt0 = curABound->GetAttAncPnt(0);
		aPnt1 = curABound->GetAttAncPnt(1);
		//at least one point is open
	//	if (!aPnt0->IsStable() && !aPnt1->IsStable()) continue;

		//planes
		GetAttachSegNums(*curABound, segNum0, segNum1);
		indPlane0 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum0);
		indPlane1 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum1);
		if (indPlane0==-1 || indPlane1==-1) continue;
		plane0 = (m_vecSurPlanes)[indPlane0];
		plane1 = (m_vecSurPlanes)[indPlane1];

		posProj00 = posProj01 = aPnt0->GetPosition();
		posProj10 = posProj11 = aPnt1->GetPosition();
		bOnRidge0 = bOnRidge1 = true;

		//aPnt0 
		posCur = aPnt0->GetPosition();
		plumb = Line3D(posCur, Vector3D(0.0, 0.0, 1.0));
		if(!IntersectLine3DPlane(plumb, plane0, posProj00)) continue;
		if(!IntersectLine3DPlane(plumb, plane1, posProj01)) continue;
		//APnt0 is on the ridge line of plane0 and plane1
		bOnRidge0 = (fabs(posProj00.Z()-posProj01.Z())<0.00001);


		//aPnt1 
		posCur = aPnt1->GetPosition();
		plumb = Line3D(posCur, Vector3D(0.0, 0.0, 1.0));
		if(!IntersectLine3DPlane(plumb, plane0, posProj10)) continue;
		if(!IntersectLine3DPlane(plumb, plane1, posProj11)) continue;
		//APnt0 is on the ridge line of plane0 and plane1
		bOnRidge1 = (fabs(posProj10.Z()-posProj11.Z())<0.00001);


		if (bOnRidge0 && bOnRidge1) continue;

		curABound->IsStepBound(true);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, posProj00, posProj10, segNum0, -1);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, posProj01, posProj11, segNum1, -1);
	}//all boundaries
}


void AnchorPointSearcher::SplitAncBoundsZeroLenth()
{
	AnchorBoundary* curABound = NULL;
	AnchorPoint *aPnt0, *aPnt1;
	int segNum0, segNum1;
	Plane plane0, plane1;
	int indPlane0, indPlane1;
	Line3D ridge, plumb, line1, line2, line3;
	LASITR lasPnt0, lasPnt1;
	double meanHei0, meanHei1;
	vector<double> vecScales;
	vector<double>::iterator itrScaleMin, itrScaleMax;
	Position3D pnt00, pnt01, pnt10, pnt11, pnt20, pnt21;

	for (int i=0; i<m_vecAncBouns.size(); i++) {
		aPnt0 = m_vecAncBouns[i]->GetAttAncPnt(0);
		aPnt1 = m_vecAncBouns[i]->GetAttAncPnt(1);
		//the two anchor point locate at same position
		//if(aPnt0->GetPosition()!=aPnt1->GetPosition()) continue;
		if(!IsClosedAncBound(m_vecAncBouns[i])) continue;

		curABound = static_cast<AnchorBoundary*>(m_vecAncBouns[i]);
		if (!curABound) continue;
		GetAttachSegNums(*curABound, segNum0, segNum1);
#ifdef _DEBUG
		if ((segNum0==0&&segNum1==1) || (segNum0==0&&segNum1==1)) {
			int aaa = 0;
		}
#endif
		if (segNum0==-1 || segNum1==-1) continue;
		indPlane0 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum0);
		indPlane1 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum1);
		if (indPlane0==-1 || indPlane1==-1) continue;
		plane0 = (m_vecSurPlanes)[indPlane0];
		plane1 = (m_vecSurPlanes)[indPlane1];

		//nonparallel, one is dormer
		//parallel, one segment is located inside of another one
		if (MyIsParaller(plane0.Normal(), plane1.Normal()))
			continue;

		Intersect2Planes(plane0, plane1, ridge);
		std::vector<AnchorEDGE> vecAEdges = curABound->GetAncEdges();
		

		meanHei0 = meanHei1 = 0.0;
		for (int iEdge=0; iEdge<vecAEdges.size(); iEdge++) {
			lasPnt0 = m_Cloud.begin() + vecAEdges[iEdge].pntNum0;
			lasPnt1 = m_Cloud.begin() + vecAEdges[iEdge].pntNum1;
			if (lasPnt0->Attribute(SegmentNumberTag)!=segNum0)
				std::swap(lasPnt0, lasPnt1);
			meanHei0 += lasPnt0->Z()/vecAEdges.size();
			meanHei1 += lasPnt1->Z()/vecAEdges.size();			
		}

		//dormer should be higher than the main roof segment
		//make sure segment 0 is the dormer
		if (meanHei0<meanHei1) {
			std::swap(segNum0, segNum1);
			std::swap(plane0, plane1);
		}

		vecScales.clear();
		for (int iEdge=0; iEdge<vecAEdges.size(); iEdge++) {
			lasPnt0 = m_Cloud.begin() + vecAEdges[iEdge].pntNum0;
			lasPnt1 = m_Cloud.begin() + vecAEdges[iEdge].pntNum1;
			if (lasPnt0->Attribute(SegmentNumberTag)!=segNum0)
				std::swap(lasPnt0, lasPnt1);
			vecScales.push_back(ridge.Scalar(lasPnt0->Position3DRef()));
		}
		itrScaleMin = std::min_element(vecScales.begin(), vecScales.end());
		itrScaleMax = std::max_element(vecScales.begin(), vecScales.end());
		pnt00 = ridge.Position(*itrScaleMin);
		pnt01 = ridge.Position(*itrScaleMax);
		//make sure point01 is higher than point00
		if (pnt00.Z() > pnt01.Z()) std::swap(pnt00, pnt01);

		Vector3D dir2;
		if (ridge.IsHorizontal(10*PI/180))
			dir2 = plane0.Normal().VectorProduct(ridge.Direction());
		else
			dir2 = plane0.Normal().VectorProduct(Vector3D(0,0,1));

		line1 = Line3D(pnt00, dir2);
		line2 = Line3D(pnt01, dir2);

		double maxDist = -99999.0, temDist;
		Position3D temPnt;
		for (int iEdge=0; iEdge<vecAEdges.size(); iEdge++) {
			lasPnt0 = m_Cloud.begin() + vecAEdges[iEdge].pntNum0;
			lasPnt1 = m_Cloud.begin() + vecAEdges[iEdge].pntNum1;
			if (lasPnt0->Attribute(SegmentNumberTag) != segNum0)
				std::swap(lasPnt0, lasPnt1);

			temPnt = line1.Project(lasPnt0->Position3DRef());
			temDist = temPnt.Distance(pnt00);
			if(temDist > maxDist )	{
				maxDist = temDist;
				pnt10 = temPnt;
			}
		}

		pnt11 = line2.Project(pnt10);

		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pnt00, pnt01, segNum0, segNum1, -1);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pnt01, pnt11, segNum0, -1, -1);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pnt11, pnt10, segNum0, -1, -1);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pnt10, pnt00, segNum0, -1, -1);
	}//end search anchor boundary
}

void AnchorPointSearcher::SplitAncBoundsTripleTagPnt()
{
	AnchorBoundary* curBound;
	int segNum0, segNum1;
	Position3D pos00, pos01, pos10, pos11, cent;
	Line3D plumb;
	Plane plane0, plane1;
	int indPlane0, indPlane1;
//	bool isCent0, isCent1;
	AnchorPoint *aPnt0, *aPnt1;

	for (int i=0; i<m_vecAncBouns.size(); i++) {
		curBound = m_vecAncBouns[i];
		assert(curBound);
		if (curBound->HasChildren()||curBound->IsStepBound()) continue;
		if (IsClosedAncBound(curBound)) continue;

		aPnt0 = curBound->GetAttAncPnt(0);
		aPnt1 = curBound->GetAttAncPnt(1);
		if (!aPnt0->IsStable() && !aPnt1->IsStable()) continue;//at least one is centre

		GetAttachSegNums(*curBound, segNum0, segNum1);
		indPlane0 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum0);
		indPlane1 = IndexPlaneBySegNumber(m_vecSurPlanes, segNum1);
		if (indPlane0==-1 || indPlane1==-1) continue;
		plane0 = (m_vecSurPlanes)[indPlane0];
		plane1 = (m_vecSurPlanes)[indPlane1];

		//start point
		cent = aPnt0->GetPosition();
		pos00 = pos01 = cent;
		plumb = Line3D(cent, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(plumb, plane0, pos00);
		IntersectLine3DPlane(plumb, plane1, pos01);

		//end point
		cent = aPnt1->GetPosition();
		plumb = Line3D(cent, Vector3D(0.0, 0.0, 1.0));
		IntersectLine3DPlane(plumb, plane0, pos10);
		IntersectLine3DPlane(plumb, plane1, pos11);

		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pos00, pos10, segNum0, -1);
		AddTopLine(m_objPntOutLines, m_lineTopOutLines, pos01, pos11, segNum1, -1);
		curBound->IsStepBound(true);
	}
}

bool AnchorPointSearcher::GetAttachSegNums(const AnchorBoundary& aBndRidge,int& segNum0, int& segNum1)
{
	vector<AnchorEDGE> vecAncEdges = aBndRidge.GetAncEdges();
	segNum0 = -1;
	segNum1 = -1;
	assert(!vecAncEdges.empty());
	LASITR pnt0 = m_Cloud.begin()+vecAncEdges[0].pntNum0;
	LASITR pnt1 = m_Cloud.begin()+vecAncEdges[0].pntNum1;

	segNum0 = pnt0->Attribute(m_surfaceTag);
	segNum1 = pnt1->Attribute(m_surfaceTag);

	return true;
}

bool AnchorPointSearcher::OutPutPolygons(ObjectPoints& destPolObjPnt, LineTopologies& destPolTops, bool onlyInner)
{
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


	for (LineTopologies::iterator itrTop=m_lineTopOutLines.begin(); itrTop!=m_lineTopOutLines.end(); itrTop++) {
		segNum1 = itrTop->Attribute(SegmentLabel);
		segNum2 = itrTop->Attribute(SecondSegmentLabel);

		for (int iPos=0; iPos<itrTop->size()-1; iPos++) {
			AddTopLine(destPolObjPnt, destPolTops, m_objPntOutLines[(*itrTop)[iPos].Number()],
				m_objPntOutLines[(*itrTop)[iPos+1].Number()], segNum1, segNum2);
		}
	}


	if (m_lineTopOutLines.empty() && !onlyInner)
		ExportContour(destPolObjPnt, destPolTops);

	return true;
}
