/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Nov 21, 2014
 |
 |
------------------------------------------------------------------- */
#ifndef __INDEX_POINT_IN_POLYGON_H_
#define __INDEX_POINT_IN_POLYGON_H_

#include "InferenceEngine.h"
#include "LaserPoint.h"

class LaserPoints;
class LineTopologies;
class ObjectPoints;


class IEDll_API IndexPointInPolygon
{
public:
	IndexPointInPolygon(LaserPoints* inLasCloud, LineTopologies* inPolygonTops, 
		ObjectPoints* inPolygonPnts, LaserPointTag tag=PolygonNumberTag);
	~IndexPointInPolygon(void);
	IndexPointInPolygon& operator= (const IndexPointInPolygon& rhs);

	bool Index();

private:
	IndexPointInPolygon();

private:
	LaserPoints* m_LasCloud;
	LineTopologies* m_PolygonTops;
	ObjectPoints* m_PolygonPnts;
	LaserPointTag m_tag;
};

#endif
