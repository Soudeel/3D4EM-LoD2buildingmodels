#include <math.h>

#include "PrimitiveGeometry.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "Vector3D.h"
#include "Position3D.h"
#include "LineTopology.h"
#include "ObjectPoint.h"

//const double PI = 3.141592653589793238462643383279502884197;

//
void  MakeCone(ObjectPoints& outObjPnts, 
						LineTopologies& outLineTops, 
						double height,
						double radius,
						const Vector3D& cent,
						const Vector3D& normal)
{
	outObjPnts.clear();
	outLineTops.clear();

	if (height <= 0.0) height = 1.0;
	if (radius <= 0.0) radius = 1.0;

	ObjectPoint peak, pos0, pos1;
	peak.Position3DRef() = cent + height*normal;
	pos0.Z() = pos1.Z() = cent.Z();
	
	LineTopology temLine, botLine;
	int nPnts=0;
	double step = PI/6.0;

	peak.Number() = nPnts++;
	outObjPnts.push_back(peak);
//	botLine.push_back(pos0.Number());

	for (double phi=0.0; phi<2*PI; phi+=step) {
		pos0.X() = cent.X() + radius*cos(phi);
		pos0.Y() = cent.Y() + radius*sin(phi);
		pos0.Number() = nPnts++;
		outObjPnts.push_back(pos0);

		pos1.X() = cent.X() + radius*cos(phi+step);
		pos1.Y() = cent.Y() + radius*sin(phi+step);
		pos1.Number() = nPnts++;
		outObjPnts.push_back(pos1);

		temLine.clear();
		temLine.push_back(pos0.Number());
		temLine.push_back(pos1.Number());
		temLine.push_back(peak.Number());
		temLine.push_back(pos0.Number());
		outLineTops.push_back(temLine);

		botLine.push_back(pos0.Number());
	}

	botLine.push_back(botLine[0]);
	outLineTops.push_back(botLine);
//	outObjPnts.RemoveDoublePoints(outLineTops, 0.0);
//	outObjPnts.RemoveUnusedPnts(outLineTops);
}

Vector3D MakePointOnSphare(double phi, double theta)
{
	Vector3D rst;
	rst.X() = cos(phi)*sin(theta);
	rst.X() = sin(phi)*sin(theta);
	rst.X() = cos(theta);

	return rst;
}

// 3--------2
// |       /|
// |      / | 
// |     /  |
// |    /   |
// |   /    |
// |  /     |
// | /      |
// 0--------1
void  MakeSphare(ObjectPoints& outObjPnts, 
						  LineTopologies& outLineTops, 
						  double radius,
						  const Vector3D& cent)
{
	outObjPnts.clear();
	outLineTops.clear();

	if (radius <= 0.0) radius = 1.0;

	ObjectPoint pos0, pos1, pos2, pos3;
	LineTopology temLine;
	int nPnts=0;
	double step = PI/12.0;

	// Iterate through phi, theta then convert r,theta,phi to  XYZ
	for (double phi = step; phi < 2*PI; phi += step) {// Azimuth [0, 2PI]
		for (double theta = step; theta < PI; theta += step) {// Elevation [0, PI]
			pos0.Position3DRef() = MakePointOnSphare(phi, theta)*radius+cent;
			pos1.Position3DRef() = MakePointOnSphare(phi, theta+step)*radius+cent;
			pos2.Position3DRef() = MakePointOnSphare(phi+step, theta+step)*radius+cent;
			pos3.Position3DRef() = MakePointOnSphare(phi+step, theta)*radius+cent;

			pos0.Number() = nPnts++;
			pos1.Number() = nPnts++;
			pos2.Number() = nPnts++;
			pos3.Number() = nPnts++;
			outObjPnts.push_back(pos0);
			outObjPnts.push_back(pos1);
			outObjPnts.push_back(pos2);
			outObjPnts.push_back(pos3);

			//triangle 0
			temLine.clear();
			temLine.push_back(pos0.Number());
			temLine.push_back(pos1.Number());
			temLine.push_back(pos2.Number());
			temLine.push_back(pos0.Number());
			outLineTops.push_back(temLine);

			//triangle 1
			temLine.clear();
			temLine.push_back(pos0.Number());
			temLine.push_back(pos2.Number());
			temLine.push_back(pos3.Number());
			temLine.push_back(pos0.Number());
			outLineTops.push_back(temLine);
		}
	}

//	outObjPnts.RemoveDoublePoints(outLineTops, 0.0);
//	outObjPnts.RemoveUnusedPnts(outLineTops);

	return;
}

// 3--------2
// |        |
// |        | 
// |        |
// |        |
// |        |
// |        |
// |        |
// 0--------1  
void  MakeCylinder(ObjectPoints& outObjPnts, 
							LineTopologies& outLineTops,
							double height,
							double radius,
							const Vector3D& botCent)
{
	outObjPnts.clear();
	outLineTops.clear();

	double step = PI/6.0;
	if (radius <= 0.0) radius = 1.0;
	ObjectPoint pos0, pos1, pos2, pos3;
	LineTopology temLine, topLine, botLine;
	int nPnts=0;	

	for (double phi=0.0; phi<2*PI; phi+=step) 
	{
		pos0.X() = botCent.X()+radius*cos(phi);
		pos0.Y() = botCent.Y()+radius*sin(phi);
		pos0.Z() = botCent.Z();
		pos0.Number() = nPnts++;
		outObjPnts.push_back(pos0);

		pos1.X() = botCent.X()+radius*cos(phi+step);
		pos1.Y() = botCent.Y()+radius*sin(phi+step);
		pos1.Z() = botCent.Z();
		pos1.Number() = nPnts++;
		outObjPnts.push_back(pos1);

		pos2 = pos1;
		pos2.Z() += height;
		pos2.Number() = nPnts++;
		outObjPnts.push_back(pos2);

		pos3 = pos0;
		pos3.Z() += height;
		pos3.Number() = nPnts++;
		outObjPnts.push_back(pos3);

		temLine.clear();
		temLine.push_back(pos0.Number());
		temLine.push_back(pos1.Number());
		temLine.push_back(pos2.Number());
		temLine.push_back(pos3.Number());
		temLine.push_back(pos0.Number());
		outLineTops.push_back(temLine);

		botLine.push_back(pos0.Number());
		topLine.push_back(pos3.Number());
	}

	botLine.push_back(botLine[0]);
	topLine.push_back(topLine[0]);
	outLineTops.push_back(botLine);
	outLineTops.push_back(topLine);

	outObjPnts.RemoveDoublePoints(outLineTops, 0.0);
	outObjPnts.RemoveUnusedPnts(outLineTops);
}
