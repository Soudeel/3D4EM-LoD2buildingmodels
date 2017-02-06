
#ifndef _MODEL_BLD_LOD_1_H_
#define _MODEL_BLD_LOD_1_H_

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

class IEDll_API ModelBldLoD1 : public ModelBldWithOccMap
{
public:
	ModelBldLoD1(LaserPoints* inLasPnts, const LineTopologies footTops, 
		const ObjectPoints& footPnts, bool bOnlyInnerRidge=false, 
		const OutliningParameters& outLinePara=OutliningParameters());
	ModelBldLoD1(LaserPoints* inLasPnts, bool bOnlyInnerRidge=false,
		const OutliningParameters& outLinePara=OutliningParameters());
	~ModelBldLoD1(){};

protected:
	bool ModelOneBld();
	bool ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers);
	
	
	//bool ContourRoofLayers();
	//bool DeriveOutlines();
	bool EncloseRoofPolygons(ObjectPoints& roofObjPnts, LineTopologies& roofLineTops);
};

#endif
