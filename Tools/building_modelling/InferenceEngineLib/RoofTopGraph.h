#ifndef _ROOF_TOPOLOGY_GRAPH_H_
#define _ROOF_TOPOLOGY_GRAPH_H_

#include "InferenceEngine.h"
#include "LaserPoint.h"
#include "Vector3D.h"
#include "Plane.h"
#include "TIN.h"
#include "LaserPoints.h"
#include "LineSegments2D.h"

#include <vector>


//only for one building
class IEDll_API RoofTopGraphSearcher
{
public:
	typedef LaserPoints::iterator LASITR;

	RoofTopGraphSearcher(const LaserPoints& cloud, const std::vector<Plane>& vecSurPlanes, 
		const ObjectPoints& pContourObjPnt, const LineTopology& pContourTop,
		LaserPointTag surfaceTag=SegmentNumberTag);
	~RoofTopGraphSearcher();

	bool DoCompute();
	bool DeriveRidges();

	bool DeriveOutlines();
	bool OutPutPolygons(ObjectPoints& PolObjPnt, LineTopologies& polTops, bool onlyInnder=false);
	
	bool RepairShortRidge();

	bool Project2DLineToPlane();

private:
	RoofTopGraphSearcher(){};

	void ExtendRidgeOrCreateAuxilityLine();


	bool CombineLasAndContour();
	bool MakeTin();
	bool CleanAllData();
	bool Initialize();
	bool IsOnContour(LASITR pnt);
	bool IsOnContour(int pntNum);

	void SurfaceGrowContourPnts();
	void RepairSegmentTag();
	int GetMaximumNeibRegionTag(const PointNumberList& curRegion, const std::vector<bool>& vecStabality);
	std::vector<LASITR> GetNeibPntWithSameTag(LASITR inPnt, LaserPointTag tag);
	int GetMaxFrequentNeibTag(LASITR inPnt, LaserPointTag tag);
	LASITR FindNeastContPntOnSameSurface(LASITR inPnt);
	
	bool ExportContour(ObjectPoints& destObjPnts, LineTopologies& destLineTops);

	
	bool IsInsideContour(LASITR lasPnt0);
	bool IsInsideContour(int indLasPnt0);
	bool IsInsideContour(LASITR lasPnt0, LASITR lasPnt1);
	bool IsInsideContour(int indLasPnt0, int indLasPnt1);
	bool IsInsideContour(LASITR lasPnt0, LASITR lasPnt1, LASITR lasPnt2);
	bool IsInsideContour(int indLasPnt0, int indLasPnt1, int indLasPnt2);
	bool IsNearContour(Vector3D pos, double errDist=0.3);

private:
	LaserPoints m_Cloud;
	std::vector<PointNumberList> m_pnlSegs;// point number lists for all segments in m_pCloud;
	std::vector<Plane> m_vecSurPlanes;
	TIN m_Tin;
	TINEdges m_TINEdges;
	LaserPointTag m_surfaceTag;
	LaserPointTag m_anchorTag;
	PointNumberList m_pnlContour;
	ObjectPoints m_contourObjPnts;
	LineTopology m_contourTop;
	LineSegments2D	m_contourSegs;

	//for outlining
	ObjectPoints m_objPntOutLines;
	LineTopologies m_lineTopOutLines;
};

#endif
