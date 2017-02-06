#include "LineTopologiesComplex.h"
#include "LineTopologyComplex.h"

#include "Plane.h"
#include "TIN.h"
#include "DataBounds2D.h"
#include "ObjectPoints.h"

#include <assert.h>
#include <map>

#define MAP_PAR_LABEL 2

LineTopologiesComplex::LineTopologiesComplex(const LineTopologies& inLineTops) 
	:std::vector<LineTopologyComplex>()
{
#ifdef _DEBUG
	inLineTops.Write("debug.top");
#endif

	LineTopologies::const_iterator itrLineTop;
	LineTopologiesComplex::iterator itrComPol;
	std::multimap<int, int> mapComPolID;//polygon ID number --> complex polygon index
	typedef std::multimap<int, int>::iterator MAPITR;
	//MAPITR itrMap;
	std::pair<MAPITR, MAPITR> rangeMap;
	this->reserve(inLineTops.size());

	//outer rings
	for (itrLineTop=inLineTops.begin(); itrLineTop!=inLineTops.end(); itrLineTop++) {
		if (!itrLineTop->HasAttribute(IDNumberTag)) continue;
		if (itrLineTop->HasAttribute(HoleTag) && itrLineTop->Attribute(HoleTag)!=0)
			continue;

		int ID = itrLineTop->Attribute(IDNumberTag);
		this->push_back(LineTopologyComplex());
		itrComPol = this->begin() + this->size()-1;
		itrComPol->ReplaceOuterRing(*itrLineTop);
		mapComPolID.insert(std::make_pair(ID,this->size()-1));
	}

	//inner rings
	for (itrLineTop=inLineTops.begin(); itrLineTop!=inLineTops.end(); itrLineTop++) {
		if (!itrLineTop->HasAttribute(IDNumberTag)) continue;
		if (itrLineTop->HasAttribute(HoleTag) && itrLineTop->Attribute(HoleTag)!=1) 
			continue;

		int ID = itrLineTop->Attribute(IDNumberTag);
		rangeMap = mapComPolID.equal_range(ID);
		while (rangeMap.first != rangeMap.second)
		{
			itrComPol = this->begin() + rangeMap.first->second;
			rangeMap.first++;

			LineTopology& outRing = itrComPol->OuterRing();
			if (*itrLineTop == outRing) continue;//same polygon

			//only same building part
			//bool bIsPart0 = outRing.Label()==MAP_PAR_LABEL;
			//bool bIsPart1 = itrLineTop->Label()==MAP_PAR_LABEL;
			//if (bIsPart0!=bIsPart1) continue;
			bool bHasPart0 = outRing.HasAttribute(BuildingPartNumberTag);
			bool bHasPart1 = itrLineTop->HasAttribute(BuildingPartNumberTag);
			if ((bHasPart0 && !bHasPart1) || (!bHasPart0 && bHasPart1)) continue;
			int bldPart0 = outRing.Attribute(BuildingPartNumberTag);
			int bldPart1 = itrLineTop->Attribute(BuildingPartNumberTag);
			if (bldPart0 != bldPart1) continue;

			itrComPol->AddInnerRing(*itrLineTop);
		}
	}

	//this->shrink_to_fit();
}

LineTopologies LineTopologiesComplex::ToLineTopologies() 
{
	LineTopologies outTopLines, temTopLines;
	//tempTopLines = inComplxTopLine.ToLineTopologies();

	for (int i=0; i<this->size();i++) {
		temTopLines = (*this)[i].ToLineTopologies();
		outTopLines.insert(outTopLines.end(), temTopLines.begin(), temTopLines.end());
	}
	
	return outTopLines;
}

//Dong
void LineTopologiesComplex::SetAttribute(const LineTopologyTag tag, const int value)
{
	for (int i=0; i<this->size();i++) {
		(*this)[i].SetAttribute(tag, value);
	}
}

TIN LineTopologyComplex::DeriveTIN(const ObjectPoints& boundPnts, const ObjectPoints& otherPoints)
{
	double *coordinate_list = 0;
	double *coordinate = 0;
	double *hole_list = 0;

	int *edge_list, *edge_point, num_edges;
	ObjectPoints::const_iterator point;
	LineTopologies::const_iterator polygon;
	LineTopology::const_iterator node;
	TIN tin;

	//////////////////////////////////////////////////////////////////////////
	//step 1
	// Create the coordinate list
	int coordinateNum =  m_outerRing.size()-1+otherPoints.size();
	coordinate_list = (double*) malloc(2*coordinateNum*sizeof(double));
	coordinate = coordinate_list;
	//outer ring
	for (unsigned int i=0; i<m_outerRing.size()-1; i++) {
		point = boundPnts.begin()+m_outerRing[i].Number();
		*coordinate++ = point->X();
		*coordinate++ = point->Y();
	}

	//other points
	for (point=otherPoints.begin(); point!=otherPoints.end(); point++) {
		*coordinate++ = point->X();
		*coordinate++ = point->Y();
	}

	//////////////////////////////////////////////////////////////////////////
	//step 2
	//Create the edge list of outer ring
	num_edges = m_outerRing.size();
	if (num_edges) {
		edge_list = (int*) malloc(2*num_edges*sizeof(int));
		edge_point = edge_list;
		for (int i=0; i<num_edges; i++) {
			*edge_list++ = i;
			*edge_list++ = i+1;
		}
		//last edge
		*edge_list++ = num_edges-1;
		*edge_list = 0;
	}

	return tin;
}
