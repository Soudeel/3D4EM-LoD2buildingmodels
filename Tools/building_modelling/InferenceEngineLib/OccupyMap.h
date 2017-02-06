/* -----------------------------------------------------------------
 |Map Point onto DEM, a building region is occupied by several continue segments
 |1) Segments should be continues,2) the building region have no gap.
 |
 |Initial Creation: Biao Xiong
 |Data: May 27, 2013
------------------------------------------------------------------- */
#ifndef _OCCUPY_MAP_H_
#define _OCCUPY_MAP_H_

#include "InferenceEngine_DLL.h"
#include "DataBounds2D.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

#include <map>
#include <vector>

class LaserPoints;
class DataBounds2D;
class PointNumberList;
class LaserPoint;
class LineTopology;
class ObjectPoints;

class IEDll_API OccupyMap
{
public:
	typedef std::map<int, std::vector<int> > GridMap;
	typedef GridMap::value_type GridPnt;

	class Grid	{
		enum GridType {
			UnKnown = 0,
			Empty,
			Bound,
			Inside
		};
	public:
		Grid():m_Type(UnKnown),m_row(0), m_col(0),m_segID(-1){};

		GridType m_Type;
		int m_row;
		int m_col;
		int m_segID;
		GridMap m_mapPnts;
		bool AddPnt(int segId, int pntId);
		bool IsEmpty();// {return m_mapPnts.size()==0 || };
	};

public:
	OccupyMap(LaserPoints* pPntCloud, const PointNumberList pnlLocalReg, double precision=0.5);
	OccupyMap(LaserPoints* pPntCloud, double precision=0.5);

	~OccupyMap();

	bool WriteOutAsLaser(char* filePath);

private:
	bool DeriveBoundBox();
	bool MappingGrids();
	bool MapBigSegments();
	inline int IndexGrid(const LaserPoint & pnt) {return IndexGrid(pnt.X(), pnt.Y());}
	inline int IndexGrid(double x, double y);
	inline LaserPoint GridPos(int indGrid);
	inline int GridMainSeg(int indGrid);

private:
	double m_pricision;
	int m_wid;
	int m_hei;
	PointNumberList m_pnlLocalReg;
	DataBounds2D m_BoundBox;
	LaserPoints* m_pPntCloud;
	std::vector<Grid> m_vecGrids;
	LineTopology m_topBound;
	ObjectPoints m_objBound;
	int m_segNumThresh;
};

#endif