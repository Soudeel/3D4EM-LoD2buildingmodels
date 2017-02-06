#include "LineTopologyComplex.h"
#include "Plane.h"
#include "TIN.h"
#include "DataBounds2D.h"
#include "ObjectPoints.h"

#include <assert.h>
#include <map>

#define MAP_PAR_LABEL 2

//Dong
LineTopologies LineTopologyComplex::ToLineTopologies() 
{
	LineTopologies tempTopLines;
	LineTopologies::const_iterator itrinnerRing;

	for(itrinnerRing=m_innerRinges.begin(); itrinnerRing!=m_innerRinges.end(); ++itrinnerRing)
		tempTopLines.push_back(*itrinnerRing);
	tempTopLines.push_back(m_outerRing);

	return tempTopLines;
}

//Dong
void LineTopologyComplex::SetAttribute(const LineTopologyTag tag, const int value)
{
	this->OuterRing().SetAttribute(tag, value);
}

//Dong
int LineTopologyComplex::GetAttribute(const LineTopologyTag tag)
{
	if(!this->OuterRing().HasAttribute(tag))
		return -1;
	return this->OuterRing().Attribute(tag); 
}

bool LineTopologyComplex::AddInnerRing(const LineTopology& innerRing) 
{
	if(std::find(m_innerRinges.begin(), m_innerRinges.end(), innerRing)==m_innerRinges.end())
		m_innerRinges.push_back(innerRing);
	return true;
}


bool LineTopologyComplex::ReplaceOuterRing(const LineTopology& outerRing) 
{
	m_outerRing=outerRing; 
	return true;
}

bool LineTopologyComplex::CoverPoint(const ObjectPoints& objPnts, const Position3D& pnt) const 
{
	return CoverPoint(objPnts, (Vector2D)pnt.Position2DOnly());
}

bool LineTopologyComplex::CoverPoint(const ObjectPoints& objPnts, const Position2D& pnt) const 
{
	return CoverPoint(objPnts, (Vector2D)pnt);
}

bool LineTopologyComplex::CoverPoint(const ObjectPoints& objPnts, const Vector3D& pnt) const 
{
	return CoverPoint(objPnts, (Vector2D)pnt);
}
bool LineTopologyComplex::CoverPoint(const ObjectPoints& objPnts, const Vector2D& pnt) const 
{
	if (!InsidePolygonJordan(pnt, objPnts, m_outerRing)) return false;
	LineTopologies::const_iterator inRing;
	for (inRing=m_innerRinges.begin(); inRing!=m_innerRinges.end(); inRing++ ) 
		if (InsidePolygonJordan(pnt, objPnts, *inRing)) return false;

	return true;
}

DataBounds2D LineTopologyComplex::DeriveBound(const ObjectPoints& objPnts)
{
	DataBounds2D temBound;

	for (unsigned int i=0; i<m_outerRing.size(); ++i)
		temBound.Update(objPnts[m_outerRing[i].Number()].Position2DOnly());

	for (unsigned int i=0; i<m_innerRinges.size(); ++i) {
		for (unsigned int j=0; j<m_innerRinges[i].size(); ++j)
			temBound.Update(objPnts[m_innerRinges[i][j].Number()].Position2DOnly());
	}

	return temBound;
}

double LineTopologyComplex::Area(ObjectPoints objPnts)
{
	double area=m_outerRing.CalculateArea(objPnts);
	for (unsigned int i=0; i<m_innerRinges.size(); i++)
		area -= m_innerRinges[i].CalculateArea(objPnts);

	return area;
}

#include "LineSegments3D.h"
double LineTopologyComplex::Distance2Pnt(const ObjectPoints& objPnts, const Vector3D& inPnt) const
{
	double	minDist = 10000.0, temDist;
	LineTopologies temLineTops = m_innerRinges;
	temLineTops.push_back(m_outerRing);
	LineSegments3D lineSegs = LineSegments3D(objPnts, temLineTops);

	for (unsigned int i=0; i<lineSegs.size(); i++) {
		temDist = lineSegs[i].DistanceToPoint(inPnt);
		if (minDist>temDist) minDist = temDist;
	}

	return minDist;
}

bool LineTopologyComplex::InsidePolygonJordan(const Vector2D& curPnt, const ObjectPoints& pts, const PointNumberList& top) const 
{
	ObjectPoints::const_iterator begPnt, pt0, pt1;
	PointNumberList::const_iterator node;
	int   num_intersections;
	double  Y_test;

	//Check if the polygon is closed and has at least three points
	if (top.begin()->Number()!=(top.end()-1)->Number()) return false;
	if (top.size() < 4) return false;

	begPnt = pts.begin();
	pt0 = begPnt+top[0].Number();
	num_intersections = 0;
	const double candX = curPnt.X();
	const double candY = curPnt.Y();
	double dx1, dx2;

	//#pragma omp parallel for private(pt1, dx1, dx2)
	for (int i=1; i<top.size(); ++i) {
		pt1 = begPnt + top[i].Number();
		dx1 = candX - pt0->X();
		dx2 = pt1->X() - candX;
		if (pt1->X() != candX && dx1*dx2>=0) {
			Y_test = (dx2*pt0->Y() + dx1*pt1->Y())/(pt1->X()-pt0->X());
			if (Y_test > candY) num_intersections++;
		}

		pt0 = pt1;
	}

	//return !(num_intersections%2);
	if ((num_intersections/2)*2 == num_intersections) return(0);
	else return(1);

	/*
	int c = 0;
	ObjectPoint *pt0, *pt1;
	PointNumberList::const_iterator node;

	// Get the first polygon point
	pt0 = pts.GetPoint(*(top.begin()));
	if (!pt0) {
		fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints3D list\n", top.begin()->Number());
		return(0);
	}

	for (node=top.begin()+1; node!=top.end(); node++) {
		pt1 = pts.GetPoint(*node);
		if( ((pt1->Y()>curPnt.Y()) != (pt0->Y()>curPnt.Y())) &&
			(curPnt.X() < (pt0->X()-pt1->X())*(curPnt.Y()-pt1->Y())/(pt0->Y()-pt1->Y()) + pt1->X()) )
			c = !c;   
		pt0 = pt1;
	}

	return(bool(c));*/
}
