/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Sep 11, 2015
 |
 |
------------------------------------------------------------------- */
#ifndef __PRIMITIVE_GEOMETRY_H__
#define __PRIMITIVE_GEOMETRY_H__

#include "InferenceEngine.h"
#include "Vector3D.h"

class ObjectPoints;
class LineTopologies;
class Position3D;
class Vector3D;
class LaserPoints;

void  MakeCone(ObjectPoints& outObjPnts, 
						LineTopologies& outLineTops, 
						double height = 1.0,
						double radius = 1.0,//radius of bottom circle
						const Vector3D& cent = Vector3D(0.0, 0.0, 0.0),//position of bottom circle
						const Vector3D& normal = Vector3D(0.0, 0.0, 1.0));

void  MakeSphare(ObjectPoints& outObjPnts, 
						  LineTopologies& outLineTops, 
						  double radius = 1.0,
						  const Vector3D& pos = Vector3D(0.0, 0.0, 0.0));

void  MakeCylinder(ObjectPoints& outObjPnts, 
							LineTopologies& outLineTops,
							double height = 1.0,
							double radius = 1.0,
							const Vector3D& botCent = Vector3D(0.0, 0.0, 0.0));//bottom center


#endif
