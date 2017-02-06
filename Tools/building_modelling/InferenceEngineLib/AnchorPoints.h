#ifndef _ANCHOR_POINTS_H_
#define _ANCHOR_POINTS_H_
/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 12-03-2015
Description: 
Revise:
----------------------------------------------------------*/
#include "InferenceEngine.h"
#include "LaserPoint.h"
#include "Vector3D.h"
#include "Plane.h"
#include "TIN.h"
#include "LaserPoints.h"
#include "LineSegments2D.h"

#include <vector>
#include <map>

class TINEdges;
class PointNumberList;
#define ANCPNTINDNUM 1000

class AnchorBoundary;

struct IEDll_API AnchorEDGE {
public:
	AnchorEDGE () {};
	AnchorEDGE (int num1, int num2);
	AnchorEDGE& operator= (const AnchorEDGE& right);
	IEDll_API friend bool operator==(const AnchorEDGE& left, const AnchorEDGE& right);
	IEDll_API friend bool operator!=(const AnchorEDGE& left, const AnchorEDGE& right);
	IEDll_API friend bool operator<(const AnchorEDGE& left, const AnchorEDGE& right);

	int pntNum0;
	int pntNum1;
};

class IEDll_API AnchorPoint
{
	typedef LaserPoints::iterator LASITR;
	typedef std::pair<LASITR, Plane*> PntPlanePair;
public:
	AnchorPoint(LaserPointTag surfaceTag=SegmentNumberTag);
	AnchorPoint(LASITR pnt1, LASITR pnt2, LaserPointTag surfaceTag=SegmentNumberTag);
	AnchorPoint(LASITR pnt1, LASITR pnt2, LASITR pnt3, LaserPointTag surfaceTag=SegmentNumberTag);
	AnchorPoint(LASITR pnt1, Plane* plane1, LASITR pnt2, Plane* plane2, LaserPointTag surfaceTag=SegmentNumberTag);
	AnchorPoint(LASITR pnt1, Plane* plane1, LASITR pnt2, Plane* plane2, 
		LASITR pnt3, Plane* plane3, LaserPointTag surfaceTag=SegmentNumberTag);
	
	~AnchorPoint();
	AnchorPoint& operator=(const AnchorPoint& right);
	friend IEDll_API bool operator==(const AnchorPoint& left, const AnchorPoint& right);

	LASITR GetAttPntBySegNum(int surTagNum);
	LASITR GetAttPnt(int ind);
	Plane* GetAttPlaneBySegNum(int surTagNum);
	Plane* GetAttPlane(int ind);
	int GetAttPntInd(LASITR lasPnt);

	int AttPntCount() {return m_attPnts.size();}
	void AttachePoint(LASITR pnt, Plane* plane=NULL);
	void ChangeAttPnt(LASITR orgPnt, LASITR dstPnt, Plane* dstPlane=NULL);
	//void AttachePoint(std::pair<LASITR, Plane*> pnt=);
	void ComputePosition();
//	bool IsMergeAble(const AnchorPoint& right);
//	void Merge(const AnchorPoint& right);

	int GetAncBoundInd(int vInd1, int vInd2);
	int GetAncBoundInd(const AnchorBoundary* pAncBnd);
	int GetAncBoundInd(LASITR lasPnt0, LASITR lasPnt1);
	
	void AttachBoundary(int indBound, AnchorBoundary* attBound);
	AnchorBoundary* GetAttBoundary(int indBound);
	AnchorBoundary* GetAttBoundary();
	int AttBoundCount();

	Vector3D GetPosition() {return m_pos;}
	void SetPosition(const Vector3D& pos) {m_pos=pos;}
	void SetID(int id) {m_ID =id;}
	int GetID() {return m_ID;}

//	bool IsCent() {return m_bCent;}
//	void IsCent(bool isCent) {m_bCent = isCent;}
	bool IsStable() {return m_bCent;}
	void IsStable(bool isCent) {m_bCent = isCent;}

	LaserPointTag GetSurfaceTag() {return m_surfaceTag;}

//	int GetEmptyAnchorEdgeNumber();
//	int IndexEmptyAnchorBoundary(int );
	std::vector<std::pair<LASITR, LASITR> > GetEmptyAnchorEdges();
protected:
	bool ComputeCentOf2Pnts();
	bool ComputeCentOf3Pnts();
	void Initialize();

protected:
	int m_ID;
	std::vector<PntPlanePair> m_attPnts;//attached laser point
	AnchorBoundary* m_attBonds[3];
	LaserPointTag m_surfaceTag;
	Vector3D m_pos;//position
	bool m_bCent;//center of the points or the intersection of planes
	//std::vector<LASITR> m_attachedPnt;
};

class IEDll_API AnchorBoundary
{
	typedef LaserPoints::iterator LASITR;
public:
	AnchorBoundary();
	~AnchorBoundary();

	void AttachAncPnt(int ind, AnchorPoint* ancPnt);
	AnchorPoint* GetAttAncPnt(int ind);

	void SetAncEdges(const std::vector<AnchorEDGE>& inBounds) {m_Bounds = inBounds;}
	std::vector<AnchorEDGE> GetAncEdges() const {return m_Bounds;}
	
	bool HasFather() {return m_pFather != NULL;} 
	void SetFather(AnchorBoundary* pFather) {m_pFather = pFather;}
	AnchorBoundary* GetFather() {return m_pFather;}
	bool HasChildren() {return !m_vecChildren.empty();}
	void AddChild(AnchorBoundary* pChild);
	std::vector<AnchorBoundary*> GetChildren() {return m_vecChildren;}

	bool IsStepBound() {return m_bStepBound;}
	void IsStepBound(bool bStepBound) {m_bStepBound = bStepBound;}

	bool GetRidgeLine(Line3D& ridge);
	bool GetAttachSegNums(int& segNum0, int& segNum1) const;
protected:
	void Initialize();

protected:
	AnchorPoint* m_APnts[2];
	std::vector<AnchorEDGE> m_Bounds;
	AnchorBoundary* m_pFather;
	std::vector<AnchorBoundary*> m_vecChildren;
	bool m_bStepBound;
};


class IEDll_API AnchorPointSearcher
{
public:
	typedef LaserPoints::iterator LASITR;
	typedef std::vector<AnchorEDGE> VecAnchorEdges;

	AnchorPointSearcher(const LaserPoints& cloud, const std::vector<Plane>& vecSurPlanes, 
		const ObjectPoints& pContourObjPnt, const LineTopology& pContourTop,
		LaserPointTag surfaceTag=SegmentNumberTag, LaserPointTag anchorTag=NumRaysTag);
	~AnchorPointSearcher();

	bool DeriveAnchorPoints();
	bool DeriveAnchorPointAndBoundaries();
	bool DeriveOutlines();
	bool OutPutPolygons(ObjectPoints& PolObjPnt, LineTopologies& polTops, bool onlyInnder=false);
	
	bool RepairShortRidge();
	bool RepairShortRidge1();
	bool RepairShortRidge10();
	bool SplitAncBounds();
	void SplitAncBoundsStepEdge();//with at least one open anchor point
	void SplitAncBoundsZeroLenth();
	void SplitAncBoundsTripleTagPnt();//with at least one unstable anchor point
	bool Project2DLineToPlane();

private:
	AnchorPointSearcher(){};
	bool CombineLasAndContour();
	bool MakeTin();
	bool CleanAllData();
	bool Initialize();
	bool IsOnContour(LASITR pnt);
	bool IsOnContour(int pntNum);

	AnchorPoint* SearchNextAncPnt(AnchorPoint* curAPnt, int surfTag1, int surfTag2);
	AnchorPoint* GetAnchorPnt(LASITR inPnt1, LASITR inPnt2);
	void SurfaceGrowContourPnts();
	void RepairSegmentTag();
	int GetMaximumNeibRegionTag(const PointNumberList& curRegion, const std::vector<bool>& vecStabality);
	std::vector<LASITR> GetNeibPntWithSameTag(LASITR inPnt, LaserPointTag tag);
	int GetMaxFrequentNeibTag(LASITR inPnt, LaserPointTag tag);
	LASITR FindNeastContPntOnSameSurface(LASITR inPnt);
	void SearchTripleAncPnts();
	void SearchDoubleAncPnts();
	void SearchAncBounds();
	AnchorBoundary* SearchAncBoundary(AnchorPoint* curAPnt, int indVertex1, int indVertex2);
	AnchorBoundary* SearchAncBoundary(const AnchorEDGE& inEdge);

	double EvaluateBoundaryStabality(const VecAnchorEdges& edges, const Position3D& begNode, const Position3D& endNode) const;
	void SortPossibleEdges(const AnchorEDGE& refEdge, std::vector<AnchorEDGE>& vecPossEdges);
	PointNumberList CleanNeibPnts(const AnchorEDGE& curEdge, const PointNumberList& neibPnts);
	bool ShrinkAnchorEdges1Side(std::vector<AnchorEDGE>& edges);
	bool ShrinkAnchorEdges2Side(std::vector<AnchorEDGE>& edges);
	bool ShrinkAEdges1SideByContour(std::vector<AnchorEDGE>& edges);
	bool ShrinkAEdges2SideByContour(std::vector<AnchorEDGE>& edges);
	bool ShrinkRidge(AnchorPoint* aPnt0, AnchorPoint* aPnt1);
	
	bool ExportContour(ObjectPoints& destObjPnts, LineTopologies& destLineTops);

	AnchorPoint* NewAnchorPnt(LASITR inPnt0, LASITR inPnt1);
	inline AnchorPoint* NewAnchorPnt(int indLasPnt0, int indLasPnt1) 
	{return NewAnchorPnt(m_Cloud.begin()+indLasPnt0, m_Cloud.begin()+indLasPnt1);}
	AnchorPoint* NewAnchorPnt(LASITR inPnt0, LASITR inPnt1, LASITR inPnt2);
	inline AnchorPoint* NewAnchorPnt(int indLasPnt0, int indLasPnt1, int indLasPnt2)
	{return NewAnchorPnt(m_Cloud.begin()+indLasPnt0,m_Cloud.begin()+indLasPnt1,m_Cloud.begin()+indLasPnt2);}
	AnchorPoint* NewAnchorPnt(int segNum0, int segNum1, int segNum2, const Vector3D& pos);
	AnchorBoundary* NewAnchorBound(AnchorPoint* pAPnt0, AnchorPoint* pAPnt1,
		const std::vector<AnchorEDGE>& vecAncEdges, int indBnd2Pnt0, int indBnd2Pnt1);

	AnchorPoint* FindAnchorPnt(int segNum0, int segNum1, int segNum2) const;
	AnchorBoundary* FindAnchorBound(int segNum0, int segNum1) const;
	void DeleteAnchorPnt(AnchorPoint* pAncPnt);
	void DeleteAnchorBound(AnchorBoundary* pAncBound);
	
	bool TagBoundEdges(std::vector<AnchorEDGE>& vecAEdges);
	bool GetAttachSegNums(const AnchorBoundary& aBndRidge, int& segNum0, int& segNum1);

	void SplitAncBoundsOO();
	void SplitAncBoundsCO();//ridge with one open end point and one close end point	
	void SplitAncBoundsCC();

	void SplitAncBoundsOOTo2(AnchorBoundary* inBound);
	void SplitAncBoundsOOTo3(AnchorBoundary* inBound);
	void SplitAncBoundsOOTo3SameEnde(AnchorBoundary* inBound);
	void SplitAncBoundsOOToX(AnchorBoundary* inBound);

	void SplitAncBoundsCOHorizontal(AnchorBoundary* inBound);
	void SplitAncBoundsCOOblique(AnchorBoundary* inBound);
	void SplitAncBoundsCOUnstableTo2(AnchorBoundary* inBound);
	void SplitAncBoundsCOUnstableTo3(AnchorBoundary* inBound);
	

	void SplitAncBoundsCCTo3Horizontal(AnchorBoundary* inBound);
	void SplitAncBoundsCCOblique();//oblique ridge, both endnotes are closed
	void SplitAncBoundsCCOblique(AnchorBoundary* inBound);

	bool WriteBoundaryEdges(const std::vector<AnchorEDGE>& edges, char* path=NULL);
	bool IsOnSameAPnt(LASITR inPnt0, LASITR inPnt1);
	bool IsOnSameAPnt(int indLasPnt1, int indLasPnt2);
	
	bool IsInsideContour(LASITR lasPnt0);
	bool IsInsideContour(int indLasPnt0);
	bool IsInsideContour(LASITR lasPnt0, LASITR lasPnt1);
	bool IsInsideContour(int indLasPnt0, int indLasPnt1);
	bool IsInsideContour(AnchorEDGE aEdge);
	bool IsInsideContour(LASITR lasPnt0, LASITR lasPnt1, LASITR lasPnt2);
	bool IsInsideContour(int indLasPnt0, int indLasPnt1, int indLasPnt2);
	bool IsInsideContour(const AnchorBoundary& aBound, const Line3D& ridge) const;
	bool IsNearContour(Vector3D pos, double errDist=0.3);

	bool IsClosedAncBound(AnchorBoundary* pABound);
	bool IsClosedAncBound(std::vector<AnchorEDGE>& vecAEdges);
	bool CloseAncBound(AnchorBoundary* pABound);

	bool IsStoredCrosing(const std::map<AnchorPoint*, std::vector<AnchorPoint*> >& mapAPCross, 
		AnchorPoint* aPnt0, AnchorPoint* aPnt1);

	int GetAttachingSurfaceSegNum(AnchorPoint* aPnt0, AnchorPoint* aPnt1);
	int GetAttachingSurfaceSegNum(const ObjectPoints& objPnts, const LineTopology& lineTop, AnchorPoint* aPnt0, AnchorPoint* aPnt1);

	double ComputeRidgeCoef(const AnchorBoundary& aBound, const Line3D& ridge) const;

	std::vector<bool> DetectMessyRidges(LineSegments2D& allRidges);
//	bool ShouldKeepOldBoundary(const std::vector<PolyhedronRepair::CornerPoint>& newCrns, AnchorBoundary* oldBound);
private:
	LaserPoints m_Cloud;
	std::vector<PointNumberList> m_pnlSegs;// point number lists for all segments in m_pCloud;
	std::vector<Plane> m_vecSurPlanes;
	TIN m_Tin;
	TINEdges m_TINEdges;
	LaserPointTag m_surfaceTag;
	LaserPointTag m_anchorTag;
	std::vector<AnchorPoint*> m_vecAncPnts;
	std::vector<AnchorBoundary*> m_vecAncBouns;
	PointNumberList m_pnlContour;
	ObjectPoints m_contourObjPnts;
	LineTopology m_contourTop;
	LineSegments2D	m_contourSegs;

	//for outlining
	ObjectPoints m_objPntOutLines;
	LineTopologies m_lineTopOutLines;
};

PointNumberList GetAdjPnts(const TINEdges& edges, PointNumber v1, PointNumber v2);

#endif
