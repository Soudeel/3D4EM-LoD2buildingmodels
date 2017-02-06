
#ifndef _MODEL_BLD_WITH_ROOF_TOP_GRAPH_H_
#define _MODEL_BLD_WITH_ROOF_TOP_GRAPH_H_

#include "InferenceEngine.h"
#include "LaserPoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "PointNumberList.h"
#include "LaserPoint.h"
#include "Buildings.h"
#include "ModelBldWithOccMap.h"

#include <map>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////////
//Tags used in this class
//line topology tags:
//BuildingPartNumberTag: roof layer

//laser point tags:
//ComponentNumberTag: roof layer
//////////////////////////////////////////////////////////////////////////////////////

class IEDll_API ModelBldWithRoofTopGraph : public ModelBldWithOccMap
{
public:
	ModelBldWithRoofTopGraph(LaserPoints* inLasPnts, const LineTopologies footTops, 
		const ObjectPoints& footPnts, bool bOnlyInnerRidge=false);
	ModelBldWithRoofTopGraph(LaserPoints* inLasPnts, bool bOnlyInnerRidge=false);
	~ModelBldWithRoofTopGraph() {};

protected:
	bool ModelOneBld();
	bool ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers);
	bool ContourRoofLayers();
	bool EncloseRoofPolygons(ObjectPoints& roofObjPnts, LineTopologies& roofLineTops);
	bool ReconstructRidges();
};

#endif
