/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Nov 21, 2014
 |
 |
------------------------------------------------------------------- */
#include "InferenceEngine.h"
#include "IndexPointInPolygon.h"
#include "LaserPoints.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "DataBounds2D.h"
#include "LineTopologiesComplex.h"

#include <time.h>

IndexPointInPolygon::IndexPointInPolygon(LaserPoints* inLasCloud, LineTopologies* inPolygonTops,
						ObjectPoints* inPolygonPnts, LaserPointTag tag)
{
	m_LasCloud = inLasCloud;
	m_PolygonTops = inPolygonTops;
	m_PolygonPnts = inPolygonPnts;
	m_tag = tag;
}

IndexPointInPolygon::~IndexPointInPolygon(void)
{

}

IndexPointInPolygon& IndexPointInPolygon::operator= (const IndexPointInPolygon& rhs)
{
	if (this == &rhs)
		return *this;

	m_LasCloud = rhs.m_LasCloud;
	m_PolygonTops = rhs.m_PolygonTops;
	m_PolygonPnts = rhs.m_PolygonPnts;
	m_tag = rhs.m_tag;
}

bool IndexPointInPolygon::Index()
{
	if (!m_LasCloud || !m_PolygonPnts || !m_PolygonTops)
		return false;

	printf("Matching points with maps\n");
	LaserPoints::iterator itrCloud;
	LineTopologies::iterator poly;

	//////////////////////////////////////////////////////////////////////////
	//use bonding box for coarsely searching
	LineTopologiesComplex m_mapTopsComp (*m_PolygonTops);
	vector<DataBounds2D> polyBounds;
	for (int i=0; i<m_mapTopsComp.size(); i++)
		polyBounds.push_back((m_mapTopsComp)[i].DeriveBound(*m_PolygonPnts));

	int mapTopCounts = m_mapTopsComp.size();
	(*m_LasCloud).RemoveAttribute(m_tag);
	(*m_LasCloud).RemoveAttribute(ComponentNumberTag);
	//#pragma omp parallel for reduction(+:sum) private(x)

	time_t start;
	time(&start);
	//int nPnts=0;
	int polAtt;
	//#pragma omp parallel for private(poly, itrCloud)
	//for (itrCloud=(*m_LasCloud).begin(); itrCloud!=(*m_LasCloud).end(); ++itrCloud) {
	for (int i=0; i<(*m_LasCloud).size();++i) {
		//printf("  Match %7d point\r", i);
		for (int iLine=0; iLine<mapTopCounts; ++iLine) {
			//coarsely check
			if (!polyBounds[iLine].Inside((*m_LasCloud)[i])) continue;
				
			//nPnts++;
			if(m_mapTopsComp[iLine].CoverPoint((*m_PolygonPnts), (*m_LasCloud)[i])) {
				if (m_mapTopsComp[iLine].OuterRing().HasAttribute(IDNumberTag))
					polAtt = m_mapTopsComp[iLine].OuterRing().Attribute(IDNumberTag);
				else
					polAtt = m_mapTopsComp[iLine].OuterRing().Attribute(LineNumberTag);
				
				(*m_LasCloud)[i].SetAttribute(m_tag, polAtt);

				break;
			}
		}
	}

	time_t end;
	time(&end);
	printf("  Time used for match data: %.2f second\n", difftime(end, start));

#ifdef _DEBUG
	(*m_LasCloud).Write("matchcloud.laser");
#endif
//	MyDeriveSegPNL((*m_LasCloud), m_PnlPolyTag, m_tag);

	return true;
}
