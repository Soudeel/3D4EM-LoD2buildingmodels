#ifndef _LineTopologies_Complex_h_
#define _LineTopologies_Complex_h_

#include "LineTopology.h"
#include "LineTopologies.h"
#include "LineTopologyComplex.h"

class ObjectPoints;
class Position2D;
class Vector3D;
class Vector2D;
class Position3D;

class LineTopologiesComplex : public std::vector<LineTopologyComplex>
{
public:
	LineTopologiesComplex() {}
	LineTopologiesComplex(const LineTopologies& inLineTops);

	//Dong
	LineTopologies ToLineTopologies();
	void SetAttribute(const LineTopologyTag tag, const int value);
	
private:

};

#endif
