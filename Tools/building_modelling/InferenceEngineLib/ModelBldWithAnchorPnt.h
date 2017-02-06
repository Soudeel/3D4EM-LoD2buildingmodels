
#ifndef _MODEL_BLD_WITH_ANCHOR_POINT_BOUNDARY_H_
#define _MODEL_BLD_WITH_ANCHOR_POINT_BOUNDARY_H_

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

class IEDll_API ModelBldWithAnchorPnt : public ModelBldWithOccMap
{
public:
	ModelBldWithAnchorPnt(LaserPoints* inLasPnts, const LineTopologies footTops, 
		const ObjectPoints& footPnts, bool bOnlyInnerRidge=false, 
		const OutliningParameters& outLinePara=OutliningParameters());
	ModelBldWithAnchorPnt(LaserPoints* inLasPnts, bool bOnlyInnerRidge=false,
		const OutliningParameters& outLinePara=OutliningParameters());
	~ModelBldWithAnchorPnt(){};

protected:
	bool ModelOneBld();
	bool ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers);
	
	bool ContourRoofLayers();
	//bool DeriveOutlines();
	bool EncloseRoofPolygons(ObjectPoints& roofObjPnts, LineTopologies& roofLineTops);
};

#endif
