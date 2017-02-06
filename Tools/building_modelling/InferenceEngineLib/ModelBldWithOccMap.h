
#ifndef _MODEL_BLD_WITH_OCCUPY_MAP_H_
#define _MODEL_BLD_WITH_OCCUPY_MAP_H_


#include "InferenceEngine.h"
#include "LaserPoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "PointNumberList.h"
#include "LaserPoint.h"
#include "Buildings.h"

#include <map>
#include <vector>
#define FOOT_PRINT_NUM -1


class IEDll_API ModelBldWithOccMap
{
public:
	ModelBldWithOccMap(LaserPoints* inLasPnts, const LineTopologies footTops, 
		const ObjectPoints& footPnts, bool bOnlyInnerRidge=false, 
		const OutliningParameters& outLinePara=OutliningParameters());
	ModelBldWithOccMap(LaserPoints* inLasPnts, bool bOnlyInnerRidge=false,
		const OutliningParameters& outLinePara=OutliningParameters());
	~ModelBldWithOccMap(){};

	bool DoModel();
	bool WritePcmModel(char* pcmModelPath);
	bool GetPcmModel(Buildings& outPcmBlds, ObjectPoints& outPcmObjPnts);

private:
	ModelBldWithOccMap(){};

protected:
	virtual bool ModelOneBld();
	virtual bool ModelRoofLayers(Building& curBld, std::vector<PointNumberList>& vecLayers);

	bool RemoveNoiseAndWallPnts();
	bool IsHigher(const PointNumberList& L1, const PointNumberList& L2);
	bool SimplifyContour(const LaserPoints& lasPnts, const PointNumberList& contour,
		ObjectPoints& localObjPnts, LineTopology& localTop);
	bool IntersectWithRoofLayer(const LaserPoints& lasPnts, const PointNumberList& layer,
		ObjectPoints& localObjPnts, LineTopology& localTop);
	bool ComputeContourHeight(std::vector<PointNumberList> vecLayers);
	bool DeriveWholeContour(const LaserPoints& lasPnts, ObjectPoints& localObjPnts, LineTopology& localTop);
	bool SortRoofLayers(const LaserPoints& lasPnts, std::vector<PointNumberList>& vecLayers);
	bool SnapToOldContours(ObjectPoints& localObjPnts, LineTopology& localTop, bool bLastCompt=false);
	bool SnapAllContoursToFoot();
	bool InvalidOverlapedPnts(ObjectPoints& localObjPnts, LineTopology& localTop,
		std::vector<PointNumberList>& vecLayers, int curLayInd);
	bool RemoveOverlapedPnts(const LaserPoints& curLasPnts, PointNumberList& pnlCurCom);
	std::vector<PointNumberList> DeriveRoofLayers(LaserPoints& inPnts, const LaserPointTag& tag);

	double DeriveGroundHeight(bool bFPHeight=true);

	bool CreateDummyBlockBuilding(Building& curBld);
	bool CreateWalls(Building& curBld);
	bool CreateBottomFloor(Building& curBld, double height=0.0);
	bool MatchData(LaserPoints& lasPnts, ObjectPoints& mapObjPts, LineTopologies& mapTops, LaserPointTag tag);
	
protected:
	OutliningParameters m_outLinePara;

	bool m_bOnlyInnerRidge;
	bool m_bHasFootPrint;
	ObjectPoints m_footPts;
	LineTopologies m_footTops;
	LaserPoints* m_pPntCloud;
	Buildings m_pcmBlds;
	ObjectPoints m_modelObjPnts;
	double m_gGrndHight;

	//PointNumberList m_curCompnent;
	//temperate data
	LaserPoints m_curLasPnts;
	LineTopology m_curFootPrint;
	LineTopologies m_curContours;
	ObjectPoints m_curObjPnts;
};

#endif
