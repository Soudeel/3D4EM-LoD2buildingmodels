#ifndef _LineTopology_Complex_h_
#define _LineTopology_Complex_h_

#include "LineTopology.h"
#include "LineTopologies.h"
class ObjectPoints;
class Position2D;
class Vector3D;
class Vector2D;
class Position3D;

class LineTopologyComplex
{
public:
	LineTopologyComplex() {};
	LineTopologyComplex(const LineTopology& outerRing, const LineTopologies& innerRinges)
		:m_outerRing(outerRing), m_innerRinges(innerRinges) {};
	~LineTopologyComplex() {};

public:
	LineTopology& OuterRing(){return m_outerRing;}
	LineTopologies& InnerRings(){return m_innerRinges;}
	LineTopologies ToLineTopologies();

	bool ReplaceOuterRing(const LineTopology& outerRing);
	bool AddInnerRing(const LineTopology& innerRing);

	bool CoverPoint(const ObjectPoints& objPnts, const Position3D& pnt) const;
	bool CoverPoint(const ObjectPoints& objPnts, const Position2D& pnt) const;
	bool CoverPoint(const ObjectPoints& objPnts, const Vector3D& pnt) const;
	bool CoverPoint(const ObjectPoints& objPnts, const Vector2D& pnt) const;
	DataBounds2D DeriveBound(const ObjectPoints& objPnts);
	double Area(ObjectPoints objPnts);
	double Distance2Pnt(const ObjectPoints& objPnts, const Vector3D& inPnt) const;

	TIN DeriveTIN(const ObjectPoints& boundPnts, const ObjectPoints& otherPoints);

	//Dong
	void SetAttribute(const LineTopologyTag tag, const int value);
	int GetAttribute(const LineTopologyTag tag);

private:
	bool InsidePolygonJordan(const Vector2D& curPnt, const ObjectPoints& pts, const PointNumberList& top) const;

protected:
	LineTopology m_outerRing;
	LineTopologies m_innerRinges;
};

#endif
