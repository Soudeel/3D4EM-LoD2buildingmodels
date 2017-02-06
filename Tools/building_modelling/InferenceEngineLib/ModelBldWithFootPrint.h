#ifndef _MODEL_BLD_WITH_FOOT_PRINT_
#define _MODEL_BLD_WITH_FOOT_PRINT_

#include <string>
#include <vector>

#include "InferenceEngine.h"
#include "LaserPoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "PointNumberList.h"
#include "LaserPoint.h"
#include "Buildings.h"

class LaserPoints;

class IEDll_API ModelBldWithFootPrint
{
public:
	ModelBldWithFootPrint();
	~ModelBldWithFootPrint();

	void Initialize();
	void DoModeling();
	bool WriteOutModels(const char* modelObjPath, const char* modelTopPath);

	void SetInLaserFile(const char* path) {m_CloudPath = path;}
	void SetInMapTopFile(const char* path) {m_mapTopPath = path;}
	void SetInMapPntFile(const char* path) {m_mapPntsPath = path;}

private:
	void Modeling();
	void MergePolygons();
	bool MatchData();
//	bool MatchData2();
	bool MatchData3();
	bool GetGroupPnts(LaserPoints& cloud, int tagNum, LaserPointTag tag);
	bool IntersectWallsWithRoofs(LaserPoints& cloud, LineTopology& footprintTop, ObjectPoints& footprintPnts, 
		std::vector<Plane>& vecLocalPlanes, std::vector<PointNumberList>& vecLocalSegPnts, 
		ObjectPoints& localObjPnts, LineTopologies& localLineTops);
	bool IntersectWallsWithRoofs2(LaserPoints& cloud, LineTopology& footprintTop, ObjectPoints& footprintPnts, 
		std::vector<Plane>& vecLocalPlanes, std::vector<PointNumberList>& vecLocalSegPnts, 
		ObjectPoints& localObjPnts, LineTopologies& localLineTops);

private:
	LaserPoints m_Cloud;
	LineTopologies m_mapTops;
	ObjectPoints m_mapPnts;

	std::string m_CloudPath;
	std::string m_mapTopPath;
	std::string m_mapPntsPath;

	std::vector<PointNumberList> m_PnlPolyTag;
	Buildings m_pcmBlds;
	ObjectPoints m_modelObjPnts;
};

#endif
