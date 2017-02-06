#include <assert.h>
#include "InferenceEngine.h"
#include "OccupyMap.h"
#include "LaserPoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

#include "Position2D.h"

using namespace std;

bool OccupyMap::Grid::AddPnt(int segId, int pntId)
{
	GridMap::iterator itr = m_mapPnts.find(segId);
	if (itr==m_mapPnts.end()) 
		m_mapPnts.insert(GridPnt(segId, vector<int>(1, pntId)));
	else
		itr->second.push_back(pntId);

	return true;
}

bool OccupyMap::Grid::IsEmpty() 
{
	if( m_mapPnts.size()==0) return true;

	if (m_mapPnts.size()==1 && m_mapPnts.begin()->first==-1) {
		return true;
	}

	return false;
};

OccupyMap::OccupyMap(LaserPoints* pPntCloud, const PointNumberList pnlLocalReg, double precision)
{
	assert(pPntCloud);

	m_pricision = precision;
	m_pnlLocalReg = pnlLocalReg;
	m_pPntCloud = pPntCloud;
	m_segNumThresh = 20;

	DeriveBoundBox();
	MappingGrids();
}

OccupyMap::OccupyMap(LaserPoints* pPntCloud, double precision)
{
	assert(pPntCloud);
	PointNumberList pnlLocalReg;

	for (int i=0; i<pPntCloud->size(); i++)
		pnlLocalReg.push_back(PointNumber(i));
	
	new(this) OccupyMap(pPntCloud, pnlLocalReg, precision);
}

OccupyMap::~OccupyMap()
{

}

bool OccupyMap::DeriveBoundBox()
{
	m_BoundBox.Initialise();
	LaserPoints::iterator pnt;

	for (int i=0; i<m_pnlLocalReg.size(); i++) {
		pnt = m_pPntCloud->begin()+m_pnlLocalReg[i].Number();
		assert(pnt!=m_pPntCloud->end());
		m_BoundBox.Update(pnt->Position2DOnly());
	}

	//make the top left and bottom right point have have integer position
	Position2D pos = m_BoundBox.Minimum();
	m_BoundBox.SetMinimumX((int)pos.X());
	m_BoundBox.SetMinimumY((int)pos.Y());

	pos = m_BoundBox.Maximum();
	m_wid = (pos.X()-m_BoundBox.Minimum().X())/m_pricision+0.5;
	m_hei = (pos.Y()-m_BoundBox.Minimum().Y())/m_pricision+0.5;
	m_BoundBox.SetMaximumX(m_BoundBox.Minimum().X()+m_pricision*m_wid);
	m_BoundBox.SetMaximumY(m_BoundBox.Minimum().Y()+m_pricision*m_hei);

	//////////////////////////////////////////////////////////////////////////
	//boundary polygon
	PointNumberList contour;
	MyDeriveContour(*m_pPntCloud, m_pnlLocalReg, contour);
	m_objBound.clear();
	m_topBound.clear();
	ObjectPoint temObjPnt;
	for (int i=0; i<contour.size()-1; i++) {
		temObjPnt.Number() = i;
		temObjPnt.Position3DRef() = (*m_pPntCloud)[contour[i].Number()].Position3DRef();
		m_objBound.push_back(temObjPnt);
		m_topBound.push_back(PointNumber(i));
	}
	m_topBound.push_back(m_topBound[0]);

#ifdef _DEBUG
	m_objBound.Write("debug.objpts");
	LineTopologies onelinetops;
	onelinetops.push_back(m_topBound);
	onelinetops.Write("debug.top");
#endif

	return true;
}

bool OccupyMap::MappingGrids()
{
	m_vecGrids = std::vector<Grid>(m_wid*m_hei, Grid());
	MapBigSegments();

	return false;
}

int OccupyMap::IndexGrid(double x, double y)
{
	if (!m_BoundBox.Inside(Position2D(x,y))) return -1;

	int irow = (y-m_BoundBox.Minimum().Y())/m_pricision;
	int icol = (x-m_BoundBox.Minimum().X())/m_pricision;
	return irow*m_wid + icol;
	//return -1;
}

LaserPoint OccupyMap::GridPos(int indGrid)
{
	int col = indGrid%m_wid;
	int row = indGrid/m_wid;
	double x = m_BoundBox.Minimum().X() + (col+0.5)*m_pricision;
	double y = m_BoundBox.Minimum().Y() + (row+0.5)*m_pricision;
	
	LaserPoint pnt;
	pnt.X() = x;
	pnt.Y() = y;
	pnt.Z() = 0.0;
	int segNum = GridMainSeg(indGrid);
	pnt.SetAttribute(SegmentNumberTag, segNum);
	
	if (segNum!=-1)	{
		GridMap::iterator itr = m_vecGrids[indGrid].m_mapPnts.find(segNum);
		pnt.SetAttribute(PulseCountTag, (int)(itr->second).size());
	}
	
	return pnt;
}

int OccupyMap::GridMainSeg(int indGrid)
{
	GridMap& gridPnts = m_vecGrids[indGrid].m_mapPnts;
	int maxPntCount = 0;
	int maxSegNum = -1;
	for (GridMap::iterator itr=gridPnts.begin(); itr!=gridPnts.end(); itr++) {
		if (maxPntCount<(itr->second).size()) {
			maxPntCount = (itr->second).size();
			maxSegNum = itr->first;
		}
	}

	return maxSegNum;
}

bool OccupyMap::WriteOutAsLaser(char* filePath)
{
	LaserPoints gridCloud;
	LaserPoint temGridPnt;
	for (int i=0; i<m_vecGrids.size(); i++) 
		gridCloud.push_back(GridPos(i));
	
	gridCloud.Write(filePath);
	return true;
}

bool CompareSegSizeFun(PointNumberList seg1, PointNumberList seg2)
{
	return seg1.size()>seg2.size();
}

bool OccupyMap::MapBigSegments()
{
	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(*m_pPntCloud, vecPnlSegs,SegmentNumberTag);
	
	//sort the segments according to their point numbers
	std::sort(vecPnlSegs.begin(), vecPnlSegs.end(), CompareSegSizeFun);
	
	vector<PointNumberList>::iterator curSeg;
	int iGrid, pntNum;
	LaserPoints::iterator itrPnt;

	for (curSeg=vecPnlSegs.begin(); curSeg!=vecPnlSegs.end(); curSeg++) {
		if (curSeg->size()<m_segNumThresh) continue;

		for (int iPnt=0; iPnt<curSeg->size(); iPnt++) {
			pntNum = (*curSeg)[iPnt].Number();
			itrPnt = m_pPntCloud->begin()+pntNum;
			iGrid = IndexGrid(*itrPnt);

			if (iGrid==-1) continue;
			if (m_vecGrids[iGrid].IsEmpty() 
				|| m_vecGrids[iGrid].m_mapPnts.begin()->first==itrPnt->Attribute(SegmentNumberTag))
				m_vecGrids[iGrid].AddPnt(itrPnt->Attribute(SegmentNumberTag), pntNum);
			//else
			//	itrPnt->RemoveAttribute(SegmentNumberTag);//invalid this point
			//not precise
		}
	}
	return true;
}