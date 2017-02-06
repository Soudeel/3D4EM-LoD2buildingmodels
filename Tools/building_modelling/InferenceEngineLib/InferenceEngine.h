/*--------------------------------------------------
 Initial creation:
 Author : Biao Xiong
 Date   : 26-01-2011
 Description: 
 Revise:
----------------------------------------------------------*/
#ifndef _Inference_Engine_H_
#define _Inference_Engine_H_

//#ifdef InfeEngine_EXPORTS
//#define IEDll_API __declspec(dllexport)
//#else
//#define IEDll_API __declspec(dllimport)
//#endif

#include "InferenceEngine_DLL.h"
#include "LaserPoint.h"

//#include "Buildings.h"
#include <vector>
#include <set>
#include "PointNumberList.h"

#define PI 4.0*atan(1.0)
#define USING_CONSTRAINT 0
//#define INTERSECT_FACE_MIN_DIST 1.0
//#define SEGMENT_MIN_PNTS 30
//#define ALPPHA_DIAMETER 1.2
//linetop Attribute 0: left plane num; Attribute 1: right plane num
//Attribute 2: is refined or not
extern double ALPPHA_DIAMETER;//gBldRecPar.alphaDia
extern double MIN_RIDGE_LEN;//gBldRecPar.minRidgeLen
extern int MIN_SEG_PNT_NUM;//gBldRecPar.minSegNum
extern int MIN_RIDGE_PNT_NUM;//gBldRecPar.minRidgeNum

struct BldRecPar {
public:
	BldRecPar() {
		bAdaptiveAlpha = true; 
		alphaDia = 1.2;
		nDorDirs = 2;
		minRidgeLen = 0.5;  
		minPntDist = 0.5; 
		minSegNum = 30; 
		minRidgeNum = 7;
		bIsLoD1 = false;
		bStepEdges = false; 
		bHasWalls = true; 
		bFootPrintHei = true;
		dicPath.reserve(60);
		dicPath = "F:\\data\\dicionary\\graph_dic.txt";
	}
	bool bAdaptiveAlpha;//use adaptively derived alpha
	double alphaDia;//alpha diameter for detect boundary points
	int nDorDirs;
	double minRidgeLen;//minimum ridge length
	double minPntDist;//minimum point distance when searching adjacent segment
	int minSegNum;//minimum segment number
	int minRidgeNum;//minimum ridge point number
	bool bIsLoD1;
	bool bStepEdges;
	bool bHasWalls;
	bool bFootPrintHei;//adaptive wall height
	std::string dicPath;
};

extern BldRecPar gBldRecPar;

class LaserPoints;
class PointNumber;
class ObjectPoints; 
class LineTopologies;
class LineTopology;
class Plane;
class Position3D;
class Position2D;
class Line3D;
class Vector3D;
class TINEdges;
class Line2D;
class Buildings;
class Building;
//class Rotation3D;
class Positions3D;
class LineSegment2D;
class OutliningParameters;

//global variables
//to store point number list of all roof segments in one building
extern std::vector<PointNumberList> gVecLocalSegPnts;

IEDll_API void UpdataIEGlobelVari(BldRecPar& updataPar);
IEDll_API BldRecPar& GetIEGlobelVari();

IEDll_API bool ReconstructBuilding(char* strInLaserPntPath, char* strOutObjPntsPath, char* strOutTopLinesPath);
IEDll_API bool StatisticRoofInfo(char* strInLaserPntPath, char* strOutObjPntsPath, char* strOutTopLinesPath);
IEDll_API void StatistBuildingInfo(const char* strObjPntsPath, const char* strTopLinesPath, const char* strInfoPath);
IEDll_API void StatisticRoofSlop(const char* strLasPntPath, const char* strSlopPath);

IEDll_API bool ReconstructLocalSegments(LaserPoints& gLaserPnts, TINEdges& gEdge, 
	std::vector<PointNumberList>& gVecPnlSegs, std::vector<Plane>& vecLocalPlanes, 
	PointNumberList pnlCurSegPnts,
	ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool MergeLocalSegments(LaserPoints& gLaserPnts, std::vector<PointNumberList>& vecLocalSegPnts,
	std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineLocalBuilding(const std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
//bool RefineSimpleDormer(const LaserPoints& gLaserPnts, const std::vector<Plane>& vecLocalPlanes, 
//	ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineSingleRoof(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineSimpleDormer(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineCornerOf3Faces(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineCornerOf4Faces(std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineCornerOfXFaces(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineOutBoundaryPlnae(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine1(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine2(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine2(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	PointNumberList& vecCurSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine3(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine4(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine5(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
bool RefineOutBoundaryLine8(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	PointNumberList& vecCurSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
//bool RefineOutBoundaryLine9(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
//	PointNumberList& vecCurSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops, const OutliningParameters& para);
IEDll_API bool RefineOutBoundaryLine6(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API bool RefineOutBoundaryLine7(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API void RefineRoofTopGrap(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);

IEDll_API bool SearchNeiborSegments(const LaserPoints* pLaserPnts, std::vector<LaserPoints>& vecCurSegPnts);
IEDll_API bool IsAdjacent(LaserPoints* pLLaserPnts, LaserPoints* pRLaserPnts, double nMinDistance = 1.0, int nMinAdjPnts=10);
IEDll_API bool IsAdjacent(const LaserPoints& gLaserPnts, const TINEdges& gEdges, const PointNumberList& pnlSeg1, 
	const PointNumberList& pnlSeg2, double nMinDistance = 1.0, int nMinAdjPnts=10);
IEDll_API bool IsAdjacent2D(const LaserPoints& gLaserPnts, const TINEdges& gEdges, const PointNumberList& pnlSeg1, 
	const PointNumberList& pnlSeg2, double fMinDistance, int nMinAdjPnts);
IEDll_API bool SearchAdjacentSegments(const LaserPoints& gLaserPnts, TINEdges& gEdges,
	std::vector<PointNumberList>& gVecPnlSegs, PointNumberList pnlOrgSeg, 
	std::vector<PointNumberList>& vecAdjSegPnts, int nMinSegPntNum = 10, int nMinAdjPntNum = 10);
IEDll_API bool SearchNeigborNodes(ObjectPoints& localObjPnts, std::vector<bool>& vecIsSearched, 
	PointNumber numCurObjPnt, std::vector<PointNumber>& vecAdjPntNum, double fMinDist = 0.5);
IEDll_API bool SearchNodesOnNeigborLine(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops, 
	const int numCurLine, const int curNode, std::vector<int>& vecNeibNodes, double minDist);
IEDll_API bool SearchLinesOnPlane(const LineTopologies& localLineTops, const int curPlaneNum, std::vector<int>& vecOutTopInds);
IEDll_API bool SearchAdjacentFaces(const LineTopologies& localLineTops, const int curPlaneNum, std::vector<int>& vecAdjFaceNums);
IEDll_API bool SearchLineBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, int& outLineInd);
IEDll_API bool SearchFaceBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, std::vector<int>& vecFaceNums);
IEDll_API bool Search2AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, std::vector<std::vector<int> >& vecOutAdjFaces);
IEDll_API bool Search3AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, std::vector<std::vector<int> >& vecOutAdjFaces);
IEDll_API bool Search4AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, std::vector<std::vector<int> >& vecOutAdjFaces);

//bool Detect3DLineByHoughTrans(const LaserPoints& lLaserPnts, const Plane& refPlane, std::vector<Position3D>& vecLineNodes);

IEDll_API bool ConstraintPlane(const LaserPoints& gLaserPnts, const PointNumberList&, Plane& plane, double minAngle = 5*3.14159/180);
IEDll_API bool ConstraintIntersecLine(const LaserPoints& gLaserPnts, const PointNumberList& pnlLeftSeg, const PointNumberList& pnlRightSeg,
	Plane& leftPlane, Plane& rightPlane, Position3D& startPos, Position3D& endPos,	double minAngle = 5*3.14159/180);

IEDll_API int IndexPlaneBySegNumber(const std::vector<Plane>& vecPlanes, int segNumber);
IEDll_API int IndexSegBySegNumber(const LaserPoints& gLaserPnts, const std::vector<PointNumberList>& vecLocalSegPnts, int segNumber, LaserPointTag tag=SegmentNumberTag);
IEDll_API bool ReplaceLineNode(ObjectPoints& localObjPnts, const LineTopology& lineTop, const Position3D& point, double maxDistance = 5.0);
//attribute 1: left segment number; attribute 2: right segment number; 
//attribute 3: is stable or not; -1 -->no 1 --> yes
//attribute 4: line gradient [0-6]
IEDll_API inline bool AddTopLine(ObjectPoints& localObjPnts, LineTopologies& localLineTops, const Position3D& point1, 
	const Position3D& point2, int att1=-1, int att2 = -1, int att3 = -1, int att4 = -1);

IEDll_API bool MyIntersect2Lines(const Line3D &lin1, const Line3D &lin2, Vector3D &pos);
IEDll_API bool MyIsCrossing(const LineSegment2D& seg0, const LineSegment2D& seg1, double err_angle=5*3.1415926/180.0);
IEDll_API bool MyIntersectLine3DPlane(const Line3D &line, const Plane &plane, Position3D &point);
IEDll_API bool MyIsParaller(const Vector3D& lDir, const Vector3D& rDir, double maxAngle = 10*3.14159/180);
IEDll_API bool MyIsParaller(const Plane& plane, const Line3D& line, double maxAngle = 10*3.14159/180);
IEDll_API double MyAngle(const Vector3D& lDir, const Vector3D& rDir);
IEDll_API bool IsMergeAble(const LaserPoints& gLaserPnts, const PointNumberList& lSeg, 
	const PointNumberList& rSeg, const Plane& lPlane, const Plane& rPlane);
IEDll_API bool IsDeleteAble(const LaserPoints& gLaserPnts, const PointNumberList& lSeg, 
	const PointNumberList& rSeg, const Plane& lPlane, const Plane& rPlane);
IEDll_API bool MyDeriveSegPNL(const LaserPoints& laserPnts, std::vector<PointNumberList>& vecSegPnls, LaserPointTag tag=SegmentNumberTag);
IEDll_API int MyCleanRedupPnts( LaserPoints& laserPnts, int dimention=3);
IEDll_API int MyCleanOddPnts( LaserPoints& laserPnts);
IEDll_API bool MyIsSelfCross(const LineTopology& polygon, const ObjectPoints& objPnts);
IEDll_API bool MyIsInsidePoly(const LaserPoint& cand, const LineTopology& polygon, const ObjectPoints& polyPnts);
IEDll_API int GetDominaintSegNum(const LaserPoints& lasPnts, PointNumberList pnlGroup);

IEDll_API bool RefineSegmentIslands(LaserPoints& laserPnts, std::vector<PointNumberList>& vecSegPnls);

IEDll_API bool GetCountour(const LaserPoints& gLaserPnts, const std::vector<PointNumberList>& vecLocalSegPnts, 
    ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API LineTopology MyDeriveContour(const LaserPoints& gLaserPnts, const TINEdges& tinEdges, 
	const PointNumberList &pnlSeg, double minEdgeLen = 0.8, bool bSameTagVale = true);
IEDll_API bool MyDeriveContour(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg, PointNumberList& contour, 
	const double factor=4.0, const Plane* pSegPlane = NULL);
IEDll_API bool MySimplifyContour(const LaserPoints& gLaserPnts, const PointNumberList& pnlTemBoundPnts, 
	const Plane& curPlane, ObjectPoints& outObjPnts, LineTopology& outTop);
//LineTopology MyDeriveContour(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg, const double factor = 2.0);
IEDll_API double TINEdgeAngle(const LaserPoints& gLaserPoints, const PointNumber& startPntNum, 
	const PointNumber& endPntNum, const Plane& refPlane,const Vector3D& refDirection);

IEDll_API bool MyFitLine(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, Line2D& line);
IEDll_API bool MyFitLine(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, Line3D& line);
IEDll_API void MyComputeNormal(const LaserPoints& lsPnts, std::vector<Vector3D>& normals, int nNeibPnts);
IEDll_API bool MyFitRectangle(const LaserPoints& laserPnts, const PointNumberList& pnlLineSeg, 
	LineTopologies& localTops, ObjectPoints& localObjPts, double maxWid=2.0);

//transform 3d coordinate system to xy, xz, or yz reference plane
IEDll_API Position2D Project2RefPlane(const Plane& refPlane, const Position3D& inPos);
//transform back to 3d coordinate system to xy, xz, or yz reference plane coordinate system
IEDll_API Position3D Repoject2OrgPlane(const Plane& refPlane, const Position2D& inPos);
IEDll_API void ReProject2OrgPlane (const LaserPoints& inPnts, const Plane& refPlane, 
	const Position3D& center, LaserPoints& outPnts);
IEDll_API void Project2RefPlane(const LaserPoints& inLaserPnts, const PointNumberList& pnlSeg, 
	const Plane& refPlane, Position3D& center, LaserPoints& outPnts);

IEDll_API double ComputeEndPointScalar(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, const Line3D& line);
IEDll_API Position3D ComputeEndPnt(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, 
	const Plane& plane, const Line3D& line);
IEDll_API double ComputeMeanPointScalar(LaserPoints& gLaserPnts, PointNumberList& pnlSegPnts, const Line3D& Line);
IEDll_API double ComputeBoundLineCoef(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, 
	const Plane& plane, const Line3D& line, double bufferWidth = 3.0, double bufferLeng = 20.0);
IEDll_API bool IsOnSameLine(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops,
	const std::vector<int>& vecLTNums, int lPntNum, int rPntNum);
IEDll_API bool ComputeNextLineDir(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts,
	const Plane& plane, const Position3D& curPoint, const Vector3D& refDir, Vector3D& outDir);
//att1 att2: left and right neighbor face number; att3: is stable or not
//IEDll_API bool AddTopLine(ObjectPoints& localObjPnts, LineTopologies& localLineTops, const Position3D& point1, 
//	const Position3D& point2, int att1, int att2, int att3);

//////////////////////////////////////////////////////////////////////////
//PCM BUILDING FUCTION
IEDll_API bool AddLine2PcmBld(Building& bld, ObjectPoints& modelObjPnts, Position3D startPnt, 
	Position3D endPnt, int leftRoofNum, int rightRoofNum, int gradient=0);
IEDll_API bool ReconstructBuilding(LaserPoints& lasPnts, std::vector<Plane>& vecPlanes, 
	Buildings& pcmBlds, ObjectPoints& modelObjPnts);
IEDll_API bool ReconstructBldWithRidges(LaserPoints& gLasPnts, Building& bld, ObjectPoints& modelObjPnts);
IEDll_API bool ReconstructBuildingLOD1(LaserPoints& gLasPnts, Buildings& pcmBlds, 
	ObjectPoints& modelObjPnts, const OutliningParameters& para);
IEDll_API bool ReconstructBuildingLOD1(LaserPoints& gLasPnts, int segNum, Building& bld, 
	ObjectPoints& modelObjPnts,  const OutliningParameters& para);
IEDll_API bool ConstructPCMModel(Buildings& buildings, ObjectPoints& map_points);
IEDll_API bool ScatterBuilding(const LaserPoints& gLasPnts, Building& bld, ObjectPoints& objPnts);
IEDll_API int IndPcmBldBySegNum(Buildings& blds, int segNum);
IEDll_API int IndPcmBldByPolygonID(Buildings& blds, int polID);
IEDll_API void SearchSegNumsInBld(Building& bld, std::vector<int>& vecSegNums);
IEDll_API void SearchSegNumsInBld(LineTopologies& topLines, std::vector<int>& vecSegNums );
IEDll_API void SearchSegNumsInBlds(Buildings& blds, std::vector<int>& vecSegNums);
IEDll_API LineTopologies SearchRidgesInBld(Building& bld);
IEDll_API LineTopologies SearchRidgesInBld(LineTopologies& topLines);
IEDll_API bool SearchLineBy2FacesPcm(const LineTopologies& localLineTops, const int faceNum1, 
	const int faceNum2, int& indLine);
IEDll_API bool SearchLinesOnPlanePcm(const LineTopologies& localLineTops, 
	const int curPlaneNum, std::vector<int>& vecOutTopNums);
IEDll_API std::vector<int> CollectRoofFacesByLines(const LineTopologies& localLineTops);
IEDll_API std::vector<std::set<int> > CollectConnectedRoofFacesByLines(const LineTopologies& localLineTops);
IEDll_API void ClosePolygon(const LineTopologies& orgLineTops, const ObjectPoints& ObjPnts, LineTopologies& destLineTops);
IEDll_API void ComputeBoundLineCoef(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API void ComputeRoofFaceConf(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API void ComputeRoofFaceConf2(LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops);
IEDll_API double Distance2Polygons(const LineTopologies& orgLineTops, 
	const ObjectPoints& ObjPnts, const Position3D& inPnt);
IEDll_API double Distance2Polygons(const std::vector<Line3D>& vecPolyLines, const Position3D& inPnt);
IEDll_API bool RemoveMapUnusedPnts(Buildings& buildings, ObjectPoints& map_points);
IEDll_API bool RemoveModelUnusedPnts(Buildings& buildings, ObjectPoints& model_points);
IEDll_API std::vector<int> Histgram(const std::vector<double>& inValues, double min, double max, double binWid);
IEDll_API std::vector<int> Histgram(const std::vector<double>& inValues, int binSize, double& binWid);
IEDll_API int MaxFrequentValue(const std::vector<int>& inValues);

IEDll_API bool DeriveContour_AlphaShape(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
	PointNumberList& contour, const double factor, const Plane* pSegPlane = NULL);
IEDll_API bool DeriveContour_AlphaShape2D(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
	PointNumberList& contour, double inAlpha=1.2, bool bAdaptiveAlpha=true);
IEDll_API bool DeriveContour_AlphaShape3D(LaserPoints& gLaserPnts, const PointNumberList &pnlSeg,
	PointNumberList& contour, const double factor=3);
//IEDll_API void ComputeResidential(LaserPoints& gLaserPnts, std::vector<Plane>& vecPlanes,
//	std::vector<PointNumberList>& vecPnlSegs, LineTopologies& roofPolygons, Buildings::iterator curBld );

IEDll_API  bool RepairMapData(ObjectPoints& mapPnts, LineTopologies& mapTops);
IEDll_API  bool MatchPointsWithMap(LaserPoints& lasPnts, ObjectPoints& mapObjPts, LineTopologies& mapTops, LaserPointTag tag=PolygonNumberTag);
IEDll_API  bool MyMergeMapPolygons(ObjectPoints& mapObjPts, LineTopologies& mapTops);


//void TestCLSF();
//void TestCLSF_perpend();
IEDll_API void TestCLSF_Vertical();
IEDll_API void TestCLSF_perpend();
IEDll_API void TestCLSF_EqualSlop();
IEDll_API void TestCLSF_concurrent();
IEDll_API void TestCLSF_concurrent2();
IEDll_API void TestCLSF_Hori_Intersect();
IEDll_API void TestCLSF_Mixture();
IEDll_API void TestCLSF_Mixture2();
IEDll_API void TestCLSF_Mixture3();
IEDll_API void TestCLSF_Mixture4();
IEDll_API void TestCLSF_Mixture5();

IEDll_API void CompareImprovement();
IEDll_API void TestGraphDiction();


#endif
