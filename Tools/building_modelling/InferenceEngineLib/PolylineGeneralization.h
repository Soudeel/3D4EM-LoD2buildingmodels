/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Jan, 17, 2014
 |
 |
------------------------------------------------------------------- */
#ifndef _POLYLINE_GENERALIZATION_H_
#define _POLYLINE_GENERALIZATION_H_
#include "InferenceEngine.h"
#include "OutliningParameters.h"
#include <vector>
#include <map>

class PointNumberList;
class LaserPoints;
class ObjectPoints;
class LineTopology;
class Line2D;
class OutliningParameters;

IEDll_API 
	void PolylineGeneralizeByHoughTransform(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const OutliningParameters& outlining_par,
	const std::vector<double>& constraintDirections=std::vector<double>());
IEDll_API 
	void PolylineGeneralizeByLineGrow(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const OutliningParameters& outlining_par=OutliningParameters(),
	const std::vector<double>& constraintDirections=std::vector<double>());
IEDll_API 
	void PolylineGeneralizeByFitPCARectangle(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop,
	const std::vector<double>& constraintDirections=std::vector<double>());
IEDll_API 
	void PolylineGeneralizeByMinAreaRectangle(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop);
IEDll_API 
	void PolylineGeneralizeByRectangleStrip(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	ObjectPoints& outObjPnts,
	LineTopology& outLineTop);

//dominate directions in radian
IEDll_API 
	std::vector<double> DeriveDominateDirections(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	int knn=2);
//dominate directions in radian
IEDll_API 
	std::vector<double> DeriveDominateDirByLineGrow(
	const LaserPoints& lasPnts, 
	const PointNumberList& pnlPolyline,
	int k = 2);

class IEDll_API LineSegmentGrow
{
public:
	LineSegmentGrow();
	LineSegmentGrow(const OutliningParameters& para);
	~LineSegmentGrow();

	std::vector<PointNumberList> DoGrow(const LaserPoints& lasPnts, const PointNumberList& pnlCtr);
	void SetParameters(const OutliningParameters& para);
	OutliningParameters GetParameters() {return m_para;}

private:
	void CleanData();
	void CopyToLocalData(const LaserPoints& lasPnts, const PointNumberList& pnlCtr);
	std::vector<PointNumberList> CopyToOutData(const LaserPoints& lasPnts, const PointNumberList& pnlCtr);

	void TagLineNumber(const PointNumber& curPnt, int lineNum);
	void TagLineNumber(const PointNumberList& inPnts, int lineNum);
	int GetLineNumber(const PointNumber& curPnt);
	bool IsUsedPoint(const PointNumber& curPnt);

	void ScoreInitialisingLine(int nNeibs);
	void SetScoreAsLine(const PointNumber& curPnt, double score);
	double GetScoreAsLine(const PointNumber& curPnt);

	PointNumberList GetNeibsWithItself(const PointNumber& curPnt, int nNeibs=2);
	PointNumberList GetUnusedPoints(const PointNumberList& inPnts);
	PointNumberList CreateSeedPoints();
	PointNumberList ExpandLineSegment(const PointNumberList& seedPnl, int lineNum);
	void CleanSmallSegments();
	Line2D CreatLine2D(const LaserPoints& lasPnts, const PointNumberList& pnl, double& maxDistPnt2Line);
	Line2D CreatLine2D(const LaserPoints& lasPnts, const PointNumberList& pnl);

	void WriteDebugLaserData();

private:
	LaserPoints* m_lasPnts;
	PointNumberList* m_pnlCtr;
	bool m_bClosed;//closed polygon
	OutliningParameters m_para;
	std::map<int, Line2D*> m_mapLines;
};

IEDll_API void TestPolyGeneByHoughTransf(const LaserPoints& inLasPnts, ObjectPoints& outPnts, LineTopologies& outTops);
IEDll_API void TestPolyGeneByHoughTransf0();
IEDll_API void TestPolyGeneByHoughTransf1();

IEDll_API void TestPolyGeneByLineGrow(const LaserPoints& inLasPnts, ObjectPoints& outPnts, LineTopologies& outTops);
IEDll_API void TestPolyGeneByLineGrow0();

IEDll_API void TestLineSegmentGrow(LaserPoints& inLasPnts);
IEDll_API void TestLineSegmentGrow0();
IEDll_API void TestLineSegmentGrow1();
IEDll_API void TestLineSegmentGrow2();

IEDll_API void TestBoundedSidePolygon();
#endif