
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : July 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Defines simple objects fitted to laser points
*
*--------------------------------------------------------------------*/
#include "OpenGLUtility.h"
#include "LaserObjects.h"
#include "LaserPointsFitting.h"
#include "VrmlUtility.h"
#include "StlUtilities.h"
#include "LaserTransform3D.h"
#include <time.h>

static void 
BeginModel(ofstream& file,char* name)
{
	file<<"measured_model:"<<endl; 
	file<<"name: \""<<name<<"\" "<<endl;
}

static void 
WriteColor(ofstream& file,Vector3D color)
{
	file<<"color: "<<endl;
	file<<"red: "<<color.X()<<endl;
	file<<"green: "<<color.Y()<<endl;
	file<<"blue: "<<color.Z()<<endl;
	file<<"endcolor: "<<endl;
}

static void 
BeginCsgNode(ofstream& file,int id)
{
	file<<"csg_node: "<<endl;
	file<<"id: "<<id<<endl;

}

static void 
WriteOrientation(ofstream& file,QuaternionRotation q,Position3D t)
{
	file<<"orientation: "<<endl;
	file<<"position: "<<endl;
	file<<"x: "<<t.X()<<endl;
	file<<"y: "<<t.Y()<<endl;
	file<<"z: "<<t.Z()<<endl;
	file<<"endposition: "<<endl;
	file<<"rotation: "<<endl;
	file<<"q0: "<<q[0]<<endl;
	file<<"q1: "<<q[1]<<endl;
	file<<"q2: "<<q[2]<<endl;
	file<<"q3: "<<q[3]<<endl;
	file<<"endrotation: "<<endl;
	file<<"endorientation: "<<endl;

}

static void 
EndCsgNode(ofstream& file)
{
	file<<"endcsg_node: "<<endl;
}

static void 
EndModel(ofstream& file)
{
	file<<"endmeasured_model: "<<endl;
}

static void 
EndMeasurements(ofstream& file)
{
	file<<"endmeasurements:" <<endl;
}

static void 
BeginMeasurements(ofstream& file)
{
	file<<"measurements:"<<endl;
}

static QuaternionRotation 
FindQuaternion(Vector3D destination,Vector3D original)
{
	destination = destination.Normalize();
	original = original.Normalize();
	Vector3D rotAxis = original.VectorProduct(destination);
	rotAxis = rotAxis.Normalize();
	
	double angle = Angle(destination,original);
	
	QuaternionRotation q(AngleAxisRotation(rotAxis,angle));
	
	Rotation3D r = q.to_matrix();
	Vector3D final = r*original;
	
	return q;
}

static void 
WriteBnfCylinder(ofstream& file,int id,const LaserCylinder& cyl)
{
	BeginModel(file,"cylinder");
	WriteColor(file,cyl.color);
	BeginCsgNode(file,id);
	
	QuaternionRotation q = FindQuaternion(cyl.pointEnd-cyl.pointBegin,Vector3D(0,0,1));
	
	Position3D t = cyl.pointBegin;
	WriteOrientation(file,q,t);
	
	//Write cylinder
	file<<"cylinder: "<<endl;
	file<<"length: "<<(cyl.pointBegin - cyl.pointEnd).Length()<<endl;
	file<<"radius: "<<cyl.radius<<endl;
	file<<"connection: 1"<<endl;
	file<<"connection: 2"<<endl;
	file<<"endcylinder: "<<endl;
	EndCsgNode(file);
	EndModel(file);
	file<<endl;
}

static void 
WriteBnfSphere(ofstream& file,int id,const LaserSphere& sph)
{
	BeginModel(file,"sphere");
	WriteColor(file,sph.color);
	BeginCsgNode(file,id);
	QuaternionRotation q;
	Position3D t = sph.centre;
	
	WriteOrientation(file,q,t);

	file<<"sphere: "<<endl;
	file<<"radius: "<<sph.radius<<endl;
	file<<"endsphere: "<<endl;
	EndCsgNode(file);
	EndModel(file); 
}

static void 
WriteBnfTorus(ofstream& file,int id, const LaserTorus& torus)
{
	BeginModel(file,"curve");
	WriteColor(file,torus.color);
	BeginCsgNode(file,id);
	
	QuaternionRotation q = FindQuaternion(Vector3D(torus.axis),Vector3D(1,0,0));
	Vector3D s(0,-torus.R,0);
	Position3D t = torus.position +q.to_matrix()*s ;

	WriteOrientation(file,q,t);
	file<<"torus: "<<endl;
	file<<"angle: 90"<<endl;
	file<<"radius: "<<torus.R<<endl;
	file<<"radius2: "<<torus.r<<endl;
	file<<"connection: 1"<<endl;
	file<<"connection: 2"<<endl;
	file<<"endtorus: "<<endl;
	EndCsgNode(file);
	EndModel(file);
	file<<endl;
}


///Constructor
LaserCylinder::LaserCylinder(Vector3D a,Vector3D b,double c,Vector3D cc)
:pointBegin(a),pointEnd(b),radius(c),LaserObject(cc)
{

}
/// Project given points to the axis and get the minimum and maximum points.
///a static local function
static int GetCylinderExtents(const LaserPoints& pts,
				Vector3D axis,
				Vector3D position,
				Vector3D& point1, Vector3D& point2)
{
	double minDot=1e30, maxDot=-1e30;
	axis = axis.Normalize();
		
	vector<double> dots(pts.size());
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = Vector3D(pts[i]);
		double dot = axis.DotProduct(v);
		dots.push_back(dot);
		minDot = MIN(minDot,dot);
		maxDot = MAX(maxDot,dot);
	}
	//cylLine must be made perperndicular to the axis.
	Vector3D cylLine = position-axis*(axis.DotProduct(position));
	point1 = cylLine+axis*minDot;
	point2 = cylLine+axis*maxDot;
	
	return 0;
}


LaserCylinder::LaserCylinder(const LaserPoints& pts, Vector3D axis, Vector3D position,double r,Vector3D c)
:radius(r),LaserObject(c)
{
	GetCylinderExtents(pts,axis,position,pointBegin, pointEnd);
}

///Print to screen.
void 
LaserCylinder::Print()
{
	Vector3D a = AxisDirection();
	Vector3D p = PointClosestToOrigin();
	printf("LaserCylinder: (axis: %g %g %g) (closest: %g %g %g) (%g,%g,%g) (%g,%g,%g) %g\n",
		a.X(),a.Y(),a.Z(),p.X(), p.Y(),p.Z(),
		pointBegin.X(),pointBegin.Y(),pointBegin.Z(),
		pointEnd.X(),pointEnd.Y(),pointEnd.Z(),radius);
}

LaserObject* LaserCylinder::operator *=(const LaserTransform3D& transform)
{
	pointBegin = transform*pointBegin;
	pointEnd = transform*pointEnd;
	
	return this;
}

//return parameter count.
int LaserCylinder::ParamCount() const
{
	return 7;
}

//Load from vector double.
void LaserCylinder::FromVector(const vector<double>& v)
{
	if(v.size()<7)
		cerr<<"size mismatch in LaserCylinder::FromVector\n";
	else
	{
		//This might be wrong but will do for the time being.
		//Just tries to preserve the length of cylinder model.
		double oldLength = (pointBegin-pointEnd).Length();
		Vector3D axis(v[0],v[1],v[2]);
		axis = axis.Normalize();
		Vector3D ptClosest(v[3],v[4],v[5]);
		ptClosest = ptClosest - axis*(axis.DotProduct(ptClosest));
		radius = v[6];
		pointBegin = ptClosest;
		pointEnd = pointBegin + axis*oldLength;
	}
		
}

//Save to vector
vector<double> LaserCylinder::ToVector()const
{
	vector<double> ans;
	Vector3D a = AxisDirection();
	Vector3D p = PointClosestToOrigin();
	
	ans.push_back(a[0]);
	ans.push_back(a[1]);
	ans.push_back(a[2]);
	
	ans.push_back(p[0]);
	ans.push_back(p[1]);
	ans.push_back(p[2]);
	
	ans.push_back(radius);
	return ans;
}

//Returns similarity with another object.
double LaserCylinder::Similarity(LaserObject* query)
{
	if(Name()!=query->Name())
		return DBL_MAX;
		
	Vector3D axis = AxisDirection();
	
	LaserCylinder* cyl = (LaserCylinder*)query;
	if(fabs(axis.DotProduct(cyl->AxisDirection()))<0.95)
	{
		return ((PointClosestToOrigin()-cyl->PointClosestToOrigin()).Length() + fabs(radius-cyl->radius))/4.00;
	}
	else
		return DBL_MAX;
}
	


///Draw the object.
void 
LaserCylinder::Draw()
{
	DrawCylinder(this->pointBegin,this->pointEnd,this->radius,this->color,true);
}

///Copy constructor.
LaserCylinder::LaserCylinder(const LaserCylinder& c)
:pointBegin(c.pointBegin),pointEnd(c.pointEnd),radius(c.radius),LaserObject(c)
{
}
	
///Make a clone.
LaserObject* 
LaserCylinder::Clone()
{
	return new LaserCylinder(*this);
}

///Copy all parameters to a vector and return.
vector<double> LaserCylinder::ParamVector()const
{
	Vector3D axis = AxisDirection();
	Vector3D pt = PointClosestToOrigin();
	vector<double> params(7);
	
	for(int i=0;i<3;i++)
	{
		params[i] = axis[i];
		params[i+3] = pt[i];
	}
	params[6] = radius;
	
	return params;
}

///Return the axis direction.
Vector3D LaserCylinder::AxisDirection()const
{
	Vector3D dir = (pointEnd-pointBegin);
	return dir.Normalize();
}
	
//Return the point closest to origin.
Vector3D LaserCylinder::PointClosestToOrigin()const
{
	Vector3D axis = AxisDirection();
	Vector3D pt = pointBegin - axis*(axis.DotProduct(pointBegin));
	return pt;
}

///Update bounds by using points in a given laser points.
///Donot change axis or radius.
void LaserCylinder::UpdateBounds(const LaserPoints& laserPoints)
{
	Vector3D axis = AxisDirection();
	Vector3D pointOnAxis = PointClosestToOrigin();

	//Project to axis and find min and max point to get point1 and point2 on cylinder.
	double dMin = 1e10;
	double dMax = -1e10;
	for(int i=0;i<laserPoints.size();i++)
	{
		double factor;
		factor = axis.DotProduct(laserPoints[i]-pointOnAxis);
		dMin = std::min(dMin,factor);
		dMax = std::max(dMax,factor);
	}
	
	pointBegin = pointOnAxis + axis*dMin;
	pointEnd = pointOnAxis + axis*dMax;
}
	
///Make angular histogram using the specified number of bins.
vector<int> LaserCylinder::AngularHistogram(const LaserPoints& pts, int count)
{
	//Get ideal normals.
	vector<Vector3D> cylNormals = this->Normal(pts);
	
	return this->AngularHistogram(cylNormals,count);
}

///Make angular histogram using the specified number of bins.
///Uses provided normals..
vector<int> LaserCylinder::AngularHistogram(const std::vector<Vector3D>& cylNormals, int count)
{
	//Make orthonormal basis.
	Vector3D xv,yv;
	this->AxisDirection().PerpendicularVectors(xv, yv);
		
	double factor = 180.00/count;
	
	vector<int> hist(count + 1);
	
	for(int i=0;i<cylNormals.size();i++)
	{
		//double angle = std::min(acos(fabs(xv.DotProduct(cylNormals[i])))*180.00/M_PI,180.);
		double xproj = xv.DotProduct(cylNormals[i]);
		double yproj = yv.DotProduct(cylNormals[i]);
		
		double angle = min(fabs((atan2(yproj,xproj))*180.00/M_PI),180.00);
		
		//cerr<<" xproj: "<<xproj;
		//cerr<<" yproj: "<<yproj <<endl;
		//cerr<<" angle: "<<angle <<endl;
		int index =	(int)(angle/factor);
		hist[index] = hist[index]+1;
	}

	return hist;
}	
		
///Calculate distance from a single point.
double LaserCylinder::DistanceFromPoint(Vector3D v)const
{
	Vector3D axis = (pointEnd-pointBegin);
	if(!axis.Length())
	{
		cerr<<"LaserCylinder::DistanceFromPoint Axis in Null vector\n";
		return 0;
	}
	axis = axis.Normalize();
	return fabs(((v-pointBegin).VectorProduct(axis)).Length() - radius);
}

///Give ideal normal direction from the surface at a given point.
Vector3D LaserCylinder::NormalAtPoint(Vector3D v)const
{
	Vector3D axis = AxisDirection();
	
	Vector3D line = v - pointBegin;
	
	line = line - axis*(axis.DotProduct(line));
	
	return line.Normalize();
}


///Write VRML.
void 
LaserCylinder::WriteVRML(ofstream& file)
{
	WriteVRMLCylinder(file);
}

///Read from ascii format.
void
LaserCylinder::ReadAscii(ifstream& file)
{
	file>>pointBegin.X()>>pointBegin.Y()>>pointBegin.Z();
	file>>pointEnd.X()>>pointEnd.Y()>>pointEnd.Z();
	file>>radius;
	file>>color.X()>>color.Y()>>color.Z();
	ReadIndices(file);
}

///write to ascii format.
void 
LaserCylinder::WriteAscii(ofstream& file)
{
	string header = "Cylinder";
	file<<header<<" ";
	file<<pointBegin.X()<<" "<<pointBegin.Y()<<" "<<pointBegin.Z()<<" ";
	file<<pointEnd.X()<<" "<<pointEnd.Y()<<" "<<pointEnd.Z()<<" ";
	file<<radius<<" ";
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	WriteIndices(file);
	file<<endl;
}

//Write to bnf file
void
LaserCylinder::WriteBnf(ofstream& file,int id)
{
	WriteBnfCylinder(file,id,*this);
}

	
void
LaserCylinder::DrawCylinder(Vector3D point1, Vector3D point2, double radius, Vector3D color,bool drawEndCaps)
{
	const int stackCount = 20;
	double angle;
	//Direction of cylinder axis.(required)
	Vector3D direction = point2 - point1;
	if(direction.Length())
		direction = direction.Normalize();
	else
		return;

	//Direction of cylinder given by gluCylinder. (z-axis)
	Vector3D zAxis(0,0,1);

	//Rotation axis is normal to the plane defined by zAxis and direction.
	Vector3D rotAxis = zAxis.VectorProduct(direction);

	if(rotAxis.Length()==0)
	{
		//printf("zero length shifting to z\n");
		rotAxis = zAxis;
		angle = 0;
	}
	else
	{

		//Find the angle between the two vectors.
		angle = Angle(zAxis,direction);
		angle = RADIAN_TO_DEGREE*angle; 
	}
	//Find height of cylinder.
	double height = (point2-point1).Length();


	//Draw the cylinder. (The last transformations are applied first in openGL)
	GLUquadricObj* cylinder = gluNewQuadric();
	gluQuadricNormals(cylinder,GLU_SMOOTH);
	gluQuadricTexture(cylinder,GL_TRUE);
	gluQuadricDrawStyle(cylinder,GLU_FILL);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef(point1.X(),point1.Y(),point1.Z());
		glRotatef(angle,rotAxis.X(),rotAxis.Y(),rotAxis.Z());
		glColor3f(color.X(),color.Y(),color.Z());
		gluCylinder(cylinder,radius,radius,height,stackCount,stackCount);
	//	glPopMatrix();

		gluDeleteQuadric(cylinder);

	if(drawEndCaps)
	{
		//Draw bottom.
		GLUquadricObj* bottom = gluNewQuadric();
		gluQuadricNormals(bottom,GLU_SMOOTH);
		gluQuadricTexture(bottom,GL_TRUE);
		gluQuadricDrawStyle(bottom,GLU_FILL);
		gluDisk(bottom,0.0, radius,stackCount,1);
		gluDeleteQuadric(bottom);
		glPopMatrix();

		glPushMatrix();
			glTranslatef(point2.X(),point2.Y(),point2.Z());
			glRotatef(angle,rotAxis.X(),rotAxis.Y(),rotAxis.Z());
			GLUquadricObj* top = gluNewQuadric();
			gluQuadricNormals(top,GLU_SMOOTH);
			gluQuadricTexture(top,GL_TRUE);
			gluQuadricDrawStyle(top,GLU_FILL);
			gluDisk(top,0.0, radius,stackCount,1);
			gluDeleteQuadric(top);
		glPopMatrix();
	}
	else
	{
		glPopMatrix();
	}
}

void
LaserCylinder::WriteVRMLCylinder(ofstream& file)
{
	double angle;
	Vector3D point1 = pointBegin;
	Vector3D point2 = pointEnd;

	//Find height of cylinder.
	double height = (point2-point1).Length();

	//Find direction of symmetry axis.
	
	Vector3D direction;
	direction = (point2-point1);
	
	if(direction.Length())
		direction = direction.Normalize();
	else
		return;

	//Direction of cylinder given by gluCylinder. (z-axis)
	Vector3D yAxis(0,1,0);

	//Rotation axis is normal to the plane defined by zAxis and direction.
	Vector3D rotAxis = yAxis.VectorProduct(direction);

	if(rotAxis.Length()==0)
	{
		if(direction.Y()>0)
			rotAxis = yAxis;
		else
			rotAxis = Vector3D(0,-1,0);
		angle = 0;
	}
	else
	{

		//Find the angle between the two vectors.
		angle = Angle(yAxis,direction);		
	}

	file << "Transform {\n";
	file << "translation " << (point1.X()+point2.X())/2 << " " << (point1.Y()+point2.Y())/2 << " " << (point1.Z()+point2.Z())/2 << " " << endl;
	file << "rotation " << rotAxis.X() << " " << rotAxis.Y() << " " << rotAxis.Z() << " " << angle << " " << endl;


	file << "children Shape {\n";
	Material2Vrml(file,color);
	
	file <<  "\t geometry Cylinder {radius "<< radius << " height " << height << " }\n}\n}";

}


///Constructor.
LaserSphere::LaserSphere(Vector3D a,double c, Vector3D cc)
:centre(a),radius(c),LaserObject(cc)
{

}

///Print to console.
void 
LaserSphere::Print()
{
	printf("LaserSphere: (%g,%g,%g) %g\n",centre.X(),centre.Y(),centre.Z(),radius);
}

///Draw in OpenGL.
void 
LaserSphere::Draw()
{
	//cerr<<"LaserSphere::Draw() for "<<this<<flush<<endl;
	DrawSphere(centre,radius,color);
	//cerr<<"~~~LaserSphere::Draw() for "<<this<<flush<<endl;
	
}

///Copy constructor.
LaserSphere::LaserSphere(const LaserSphere& s)
:centre(s.centre),radius(s.radius),LaserObject(s)
{
}

LaserObject* LaserSphere::operator *=(const LaserTransform3D& transform)
{
	centre = transform*centre;
	return this;
}

//return parameter count.
int LaserSphere::ParamCount() const
{
	return 4;
}

//Load from vector double.
void LaserSphere::FromVector(const vector<double>& v)
{
	if(v.size()<4)
		cerr<<"size mismatch in LaserSphere::FromVector\n";
	else
	{
		centre = Vector3D(v[0],v[1],v[2]);
		radius = v[3];
	}
		
}

//Save to vector
vector<double> LaserSphere::ToVector()const
{
	vector<double> ans;
		
	ans.push_back(centre[0]);
	ans.push_back(centre[1]);
	ans.push_back(centre[2]);
	
	ans.push_back(radius);
	return ans;
}

//Returns similarity with another object.
double LaserSphere::Similarity(LaserObject* q)
{
	if(Name()!=q->Name())
		return DBL_MAX;
	else
	{
		LaserSphere * query = (LaserSphere*)q;
		return ((centre-query->centre).Length()+ fabs(radius-query->radius))/4.00;
	}
}


///Calculate distance from a single point.
double
LaserSphere::DistanceFromPoint(Vector3D v)const
{
	return (v-centre).Length();
}
	
///Give ideal normal direction from the surface at a given point.
Vector3D LaserSphere::NormalAtPoint(Vector3D v)const
{
	return (v-centre).Normalize();
}

///Make a clone.
LaserObject* 
LaserSphere::Clone()
{
	return new LaserSphere(*this);
}
	
///Copy all parameters to a vector and return.
vector<double> LaserSphere::ParamVector()const
{
	vector<double> params(4);
	
	for(int i=0;i<3;i++)
	{
		params[i] = centre[i];
	}
	params[3] = radius;
	
	return params;
}
///Write to vrml file
void 
LaserSphere::WriteVRML(ofstream& file)
{
	WriteVRMLSphere(file);
}

///Read from ascii format.
void 
LaserSphere::ReadAscii(ifstream& file)
{
	file>>centre.X()>>centre.Y()>>centre.Z();
	file>>radius;
	file>>color.X()>>color.Y()>>color.Z();
	ReadIndices(file);
}

///Write to ascii format.
void
LaserSphere::WriteAscii(ofstream& file)
{
	string header = "Sphere";
	file<<header<<" ";
	file<<centre.X()<<" "<<centre.Y()<<" "<<centre.Z()<<" ";
	file<<radius<<" ";
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	WriteIndices(file);
	file<<endl;
}

//Write to bnf file
void
LaserSphere::WriteBnf(ofstream& file,int id)
{
	WriteBnfSphere(file,id,*this);
}

void 
LaserSphere::DrawSphere(Vector3D centre, double radius, Vector3D color)
{
	glColor3f(color.X(),color.Y(),color.Z());

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef(centre.X(),centre.Y(),centre.Z());
		glutSolidSphere(radius,50,50);
	glPopMatrix();
}

void
LaserSphere::WriteVRMLSphere(ofstream& file)
{
	file << "Transform {\n";
	file << "translation " << centre.X()<<" "<<centre.Y()<<" "<<centre.Z()<<endl;
	
	file << "children Shape {\n";
	Material2Vrml(file,color);
	file <<  "\t geometry Sphere {radius "<< radius << "\n}\n}\n}\n";

}


///Constructor.
LaserPlane::LaserPlane(Plane p,Positions3D ps, Vector3D cc)
:plane(p), poss(ps), LaserObject(cc)
{

}

///Render in Opengl.
void 
LaserPlane::Draw()
{
	DrawPlane(plane,color);
}

void LaserPlane::Print()
{
	Vector3D n = plane.Normal();
	printf("LaserPlane: (%g %g %g) %g \n",n[0],n[1],n[2],plane.Distance());
}

LaserObject* LaserPlane::operator *=(const LaserTransform3D& transform)
{
	plane.Normal() = transform.Rotation()* plane.Normal();
	plane.Distance() = plane.Distance() + transform.Translation().DotProduct(plane.Normal());
	
	//The convex hull needs to be rotated too.
	for(int i=0;i<poss.size();i++)
	{
		poss[i] = transform*poss[i];
	};
	return this;
}

///Update convex hull from given laser points.
void LaserPlane::UpdateHull(const LaserPoints& pts)
{
	GetConvexHull(pts,plane.Normal(),poss);
}

//return parameter count.
int LaserPlane::ParamCount() const
{
	return 4;
}

//Load from vector double.
void LaserPlane::FromVector(const vector<double>& v)
{
	if(v.size()<4)
		cerr<<"size mismatch in LaserPlane::FromVector\n";
	else
	{
		plane.Normal() = Vector3D(v[0],v[1],v[2]).Normalize();
		plane.Distance() = v[3];
	}
		
}

//Save to vector
vector<double> LaserPlane::ToVector()const
{
	vector<double> ans;
	Vector3D n = plane.Normal();
	ans.push_back(n[0]);
	ans.push_back(n[1]);
	ans.push_back(n[2]);
	ans.push_back(plane.Distance());
	return ans;
}

//Returns similarity with another object.
double LaserPlane::Similarity(LaserObject* q)
{
	if(q->Name()!=Name())
		return DBL_MAX;
	
	LaserPlane* query = (LaserPlane*)q;
	double dot = NormalDirection().DotProduct(query->NormalDirection());
	if(fabs(dot)>0.95)
	{
		return (fabs(DistanceFromOrigin()-query->DistanceFromOrigin()*dot));
	}
	else
		return DBL_MAX;
}


///Copy constructor.
LaserPlane::LaserPlane(const LaserPlane& p)
:plane(p.plane), poss(p.poss), LaserObject(p) 
{

}

///Make a clone.
LaserObject* 
LaserPlane::Clone()
{
	return new LaserPlane(*this);
}

///Copy all parameters to a vector and return.
vector<double> LaserPlane::ParamVector()const
{
	Vector3D normal = plane.Normal();
	vector<double> params(4);
	
	for(int i=0;i<3;i++)
	{
		params[i] = normal[i];
	}
	params[3] = plane.Distance();
	
	return params;
}

///Calculate distance from a single point.
double LaserPlane::DistanceFromPoint(Vector3D v)const
{
	Vector3D n = plane.Normal();
	if(!n.Length())
	{
		cerr<<"LaserPlane::DistanceFromPoint Axis in Null vector\n";
		return 0;
	}
	n = n.Normalize();
	return fabs(n.DotProduct(v)-plane.Distance());
}


///Give ideal normal direction from the surface at a given point.
Vector3D LaserPlane::NormalAtPoint(Vector3D v)const
{
	return plane.Normal();
}

///Write to vrml.
void 
LaserPlane::WriteVRML(ofstream& file)
{
	WriteVRMLPlane(file);
}

///Read from ascii format.
void 
LaserPlane::ReadAscii(ifstream& file)
{
	file>>plane.Normal().X()>>plane.Normal().Y()>>plane.Normal().Z()>>plane.Distance();
	file>>color.X()>>color.Y()>>color.Z();
	int size;
	file>>size;
	poss.resize(size);
	for(int i=0;i<size;i++)
	file>>poss[i].X()>>poss[i].Y()>>poss[i].Z();
	ReadIndices(file);
	
}

///Write to ascii format.
void 
LaserPlane::WriteAscii(ofstream& file)
{
	string header = "Plane";
	file<<header<<" ";
	file<<plane.Normal().X()<<" "<<plane.Normal().Y()<<" "<<plane.Normal().Z()<<" "<<plane.Distance()<<" ";
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	file<<poss.size()<<" ";
	for(int i=0;i<poss.size();i++)
		file<<poss[i].X()<<" "<<poss[i].Y()<<" "<<poss[i].Z()<<" ";
	WriteIndices(file);
	file<<endl;
}

//Write to bnf file (doesn't make sense currently)
void
LaserPlane::WriteBnf(ofstream& file,int id)
{

}






void 
LaserPlane::DrawPlane(Plane p,Vector3D color)
{
	//We have the convex hull in the form of a polygon.
	//But OpenGL doesn't support drawing polygons as such (at least the results are really crappy)
	//So we will tesselate the polygon, and draw it as TRIANGLE_FAN.
	GLdouble* vertex;
	int i;
	int count = poss.size();
	static GLUtriangulatorObj *tobj = NULL;
	GLdouble *data = new GLdouble[count*3];

	if (tobj == NULL) {
		tobj = gluNewTess();
#ifdef windows		
		gluTessCallback(tobj, (GLenum) GLU_BEGIN,reinterpret_cast<GLvoid (CALLBACK *)()>(&glBegin));
		gluTessCallback(tobj, (GLenum) GLU_VERTEX,reinterpret_cast<GLvoid (CALLBACK *)()>(&glVertex3dv));
		gluTessCallback(tobj, (GLenum) GLU_END,reinterpret_cast<GLvoid (CALLBACK *)()>(&glEnd));
#else
		gluTessCallback(tobj, (GLenum) GLU_BEGIN,reinterpret_cast<GLvoid ( *)()>(&glBegin));
		gluTessCallback(tobj, (GLenum) GLU_VERTEX,reinterpret_cast<GLvoid ( *)()>(&glVertex3dv));
		gluTessCallback(tobj, (GLenum) GLU_END,reinterpret_cast<GLvoid ( *)()>(&glEnd));
#endif		
		
	}
	glColor3f(color.X(),color.Y(),color.Z());
	glNormal3f(p.Normal().X(),p.Normal().Y(),p.Normal().Z());
	gluBeginPolygon(tobj);
	vertex = data;
	for(i = 0; i < poss.size(); i++) 
	{
		vertex[0] = poss[i].X();
		vertex[1] = poss[i].Y();
		vertex[2] = poss[i].Z();
		gluTessVertex(tobj, vertex, vertex);
		vertex += 3;
	}
	gluEndPolygon(tobj);
	
	delete[] data;
}

void
LaserPlane::WriteVRMLPlane(ofstream& file)
{
	file << "Transform {\n";
	file << "children Shape {\n";
	Material2Vrml(file,color);
	file <<  "\t geometry IndexedFaceSet {\n";
	file << "\t coord Coordinate {\n";
	file << "point[\n";
	vector<int> indices;
	
	int count = 0;
	for (Positions3D::iterator pos = poss.begin(); pos != poss.end(); pos++)
	{
		file<<"\t\t "<<pos->X()<<" "<<pos->Y()<<" "<< pos->Z()<<"\n";
		indices.push_back(count++);
	}
	file <<"]}\n";
	file <<"coordIndex [\n";
	
	for(int i=0;i<indices.size();i++)
	{
		file<<indices[i]<<"  ";
	}
	file<<"]\n";
	file<<"}\n";
	file<<"}\n}\n";
}		


///Constructor.
LaserTorus::LaserTorus(Vector3D a,Vector3D b,double tR,double tr,Vector3D cc)
:axis(a),position(b),R(tR),r(tr),LaserObject(cc){}

///Print to screen.
void 
LaserTorus::Print()
{
	printf("LaserTorus: (%g,%g,%g) (%g,%g,%g) %g  %g\n",axis.X(),axis.Y(),axis.Z(),
							position.X(),position.Y(),position.Z(),R,r);
}

///Render in Opengl.
void 
LaserTorus::Draw()
{
	DrawTorus(axis,position,R,r,color);;
}

///Copy constructor.
LaserTorus::LaserTorus(const LaserTorus& t)
:axis(t.axis),position(t.position),R(t.R),r(t.r),LaserObject(t)
{

}

LaserObject* LaserTorus::operator *=(const LaserTransform3D& transform)
{
	axis = transform.Rotation()*axis;
	position = transform*position;
	return this;
}

//return parameter count.
int LaserTorus::ParamCount() const
{
	return 8;
}

//Load from vector double.
void LaserTorus::FromVector(const vector<double>& v)
{
	if(v.size()<8)
		cerr<<"size mismatch in LaserTorus::FromVector\n";
	else
	{
		axis = Vector3D(v[0],v[1],v[2]).Normalize();
		position = Vector3D(v[3],v[4],v[5]);
		R = v[6];
		r = v[7];
	}
		
}

//Save to vector
vector<double> LaserTorus::ToVector()const
{
	vector<double> ans;
		
	ans.push_back(axis[0]);
	ans.push_back(axis[1]);
	ans.push_back(axis[2]);
	
	ans.push_back(position[0]);
	ans.push_back(position[1]);
	ans.push_back(position[2]);
	
	ans.push_back(R);
	ans.push_back(r);
	
	return ans;
}

//Returns similarity with another object.
double LaserTorus::Similarity(LaserObject* q)
{
	if(Name()!=q->Name())
		return DBL_MAX;
		
	LaserTorus* query = (LaserTorus*)q;
	
	if(fabs(axis.DotProduct(query->axis))>0.95)
	{
		return ((position-query->position).Length()+R+r)/5.00;
	}
}


///Clone this object.
LaserObject* 
LaserTorus::Clone()
{
	return new LaserTorus(*this);
}

///Copy all parameters to a vector and return.
vector<double> LaserTorus::ParamVector()const
{
	vector<double> params(8);
	
	for(int i=0;i<3;i++)
	{
		params[i] = axis[i];
		params[i+3] = position[i];
	}
	params[6] = R;
	params[7] = r;
	
	return params;
}

///Write to vrml file.
void
LaserTorus::WriteVRML(ofstream& file)
{
	WriteVRMLTorus(file);
}

///Read from ascii format.
void
LaserTorus::ReadAscii(ifstream& file)
{
	file>>axis.X()>>axis.Y()>>axis.Z();
	file>>position.X()>>position.Y()>>position.Z();
	file>>R;
	file>>r;
	file>>color.X()>>color.Y()>>color.Z();
	ReadIndices(file);
}

///Calculate distance from a single point.
double 
LaserTorus::DistanceFromPoint(Vector3D v)const
{
	Vector3D diff = v - position;
	
	double d1 = R - axis.VectorProduct(diff).Length();
	double d2 = axis.DotProduct(diff);
	
	return (sqrt(d1*d1 + d2*d2) - r);
}
	
///Give ideal normal direction from the surface at a given point.
Vector3D LaserTorus::NormalAtPoint(Vector3D v)const
{
	Vector3D proj = v - axis*(axis.DotProduct(v));

    Vector3D circlePt = (proj-position).Normalize()*R;

	return (v-circlePt).Normalize();
}

///Write to ascii format.
void
LaserTorus::WriteAscii(ofstream& file)
{
	string header = "Torus";
	file<<header<<" ";
	file<<axis.X()<<" "<<axis.Y()<<" "<<axis.Z()<<" ";
	file<<position.X()<<" "<<position.Y()<<" "<<position.Z()<<" ";
	file<<R<<" ";
	file<<r<<" ";
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	WriteIndices(file);
	file<<endl;
}

//Write to bnf file
void
LaserTorus::WriteBnf(ofstream& file,int id)
{
	WriteBnfTorus(file,id,*this);
}

///Draw torus.
void
LaserTorus::DrawTorus(Vector3D axis, Vector3D centre,double outerRadius,double innerRadius,Vector3D color)
{
	Vector3D zAxis(0,0,1);

	Vector3D rotationAxis = zAxis.VectorProduct(axis);
	if(rotationAxis.Length())
		rotationAxis = rotationAxis.Normalize();
	if(axis.Length())
		axis = axis.Normalize();
	double angle;

	if(rotationAxis.Length()==0)
	{
		angle = 0;
		rotationAxis = zAxis;
	}
	else
	{
		angle = Angle(zAxis,axis);
		angle = RADIAN_TO_DEGREE*angle;
	}
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		glTranslatef(centre.X(),centre.Y(),centre.Z());
		glRotatef(angle,rotationAxis.X(),rotationAxis.Y(),rotationAxis.Z());
		glColor3f(color.X(),color.Y(),color.Z());
		glutSolidTorus(innerRadius,outerRadius,30,30);
	glPopMatrix();

}

///Write trus to Vrml.
void
LaserTorus::WriteVRMLTorus(ofstream& file)
{
	file<<"\n#Torus not yet implemented\n";
}


///Plane fitted to laser points. 
///This one keeps a triangulate representation instead of a convex hull.
//is written as a list of all triangles.
///Keeps info about convex hull too needed for drawing.
	
///Constructor.
LaserPlaneTR::LaserPlaneTR(Plane p,const LaserPoints& pts,Vector3D cc)
:plane(p),LaserObject(cc)
{
	//Don't use copy constructor it just copies pointers, and causes problems
	if(pts.size())
	{
		for(int i=0;i<pts.size();i++)
			points.push_back(pts[i]);
		points.DeriveTIN();
		CopyTIN((TIN&)pts.TINReference(),points.TINReference());
	}
	//points.TINReference() = pts.TINReference();
}
///Render in Opengl.
void LaserPlaneTR::Draw()
{
	//cerr<<"LaserPlaneTR::Draw()...";
	//cerr<<"size: "<<points.TINReference().size()<<"  ";
	DrawLaserPointsTINGL(points,points.TINReference(),color);
	//cerr<<"Done\n";
}
	
///Copy constructor.
LaserPlaneTR::LaserPlaneTR(const LaserPlaneTR& p):LaserObject(p)
{
	if(&p!=(this))
	{
		plane = p.plane;
		points = p.points;
		color = p.color;
	}
}
	
///Make a clone.
LaserObject* LaserPlaneTR::Clone()
{
	return new LaserPlaneTR(*this);
}

LaserObject* LaserPlaneTR::operator *=(const LaserTransform3D& transform)
{
	return this;
}

//return parameter count.
int LaserPlaneTR::ParamCount() const
{
	return 0;
}

//Load from vector double.
void LaserPlaneTR::FromVector(const vector<double>& v)
{
			
}

//Save to vector
vector<double> LaserPlaneTR::ToVector()const
{
	vector<double> ans;
	return ans;
}


//Returns similarity with another object.
double LaserPlaneTR::Similarity(LaserObject* query)
{

}



///Copy all parameters to a vector and return.
vector<double> LaserPlaneTR::ParamVector()const
{
	Vector3D normal = plane.Normal();
	vector<double> params(4);
	
	for(int i=0;i<3;i++)
	{
		params[i] = normal[i];
	}
	params[3] = plane.Distance();
	
	return params;
}

///Calculate distance from a single point.
double LaserPlaneTR::DistanceFromPoint(Vector3D v)const
{
	Vector3D n = plane.Normal();
	if(!n.Length())
	{
		cerr<<"LaserPlaneTR::DistanceFromPoint Axis in Null vector\n";
		return 0;
	}
	n = n.Normalize();
	return fabs(n.DotProduct(v)-plane.Distance());
}

///Give ideal normal direction from the surface at a given point.
Vector3D LaserPlaneTR::NormalAtPoint(Vector3D v)const
{
	return Vector3D(0,0,1);
}

	
///Write to vrml.
void LaserPlaneTR::WriteVRML(ofstream& file)
{
	LaserPointsTIN2Vrml(file,points,points.TINReference(),color);
}
	
///Read from ascii format.
void LaserPlaneTR::ReadAscii(ifstream& file)
{
	int pointCount;
	file>>pointCount;
	
	points.resize(pointCount);
	double d;
	int v;
	for(int i=0;i<points.size();i++)
	{
		file>>d; points[i].X() = d;
		file>>d; points[i].Y() = d;
		file>>d; points[i].Z() = d;
	}
	
	points.DeriveTIN();
	TIN& tin = points.TINReference();
	
	int tinSize;
	file>>tinSize;
	tin.resize(tinSize);
	
	for(int i=0;i<tin.size();i++)
	{
		TINMesh currentMesh = tin[i];
		PointNumber *points = tin[i].Nodes ();

		if(points)
		{
			file>>v; points[0].Number() = v;
			file>>v; points[1].Number() = v;
			file>>v; points[2].Number() = v;
		}
	}
	file>>d; color.X()=d;
	file>>d; color.Y()=d;
	file>>d; color.Z()=d;
	ReadIndices(file);
}
	
///Write to ascii format.
void LaserPlaneTR::WriteAscii(ofstream& file)
{
	string header = "PlaneTR";
	file<<header<<" ";
	file<<points.size()<<" ";
	
	for(int i=0;i<points.size();i++)
		file<<points[i].X()<<" "<<points[i].Y()<<" "<<points[i].Z()<<" ";
	
	TIN& tin = points.TINReference();
	file<<tin.size()<<" ";
	for(int i=0;i<tin.size();i++)
	{
		TINMesh currentMesh = tin[i];
		PointNumber *points = tin[i].Nodes ();

		if(points)
		{
			file<<points[0].Number()<<" "
				<<points[1].Number()<<" "
				<<points[2].Number()<<" ";
		}
	}
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	WriteIndices(file);
	file<<endl;
}
	
//Write to bnf file (doesn't make sense currently)
void LaserPlaneTR::WriteBnf(ofstream& file,int id)
{

}


	
///Constructor.
LaserControlPoint::LaserControlPoint(Vector3D a,double c,Vector3D cc,int64 _id, string _name)
:LaserSphere(a,c,cc)
{
	id = _id;
	name = _name;
}

///Print to console.
void LaserControlPoint::Print()
{
	cerr<<"LaserControlPoint: ";
	//(*((LaserSphere*)(this))).Print();
	LaserSphere::Print();
}

///Draw in OpenGL.
void LaserControlPoint::Draw()
{
	DrawSphere(centre, radius, color);
}	


LaserObject* LaserControlPoint::operator *=(const LaserTransform3D& transform)
{
	return this;
}

//return parameter count.
int LaserControlPoint::ParamCount() const
{
	return 0;
}

//Load from vector double.
void LaserControlPoint::FromVector(const vector<double>& v)
{
			
}

//Save to vector
vector<double> LaserControlPoint::ToVector()const
{
	vector<double> ans;
	return ans;
}

//Returns similarity with another object.
double LaserControlPoint::Similarity(LaserObject* query)
{

}



///Draw on a given context.
void LaserControlPoint::Draw(QGLWidget* widget)
{
	this->Draw();
	if(widget)
	{	
		GLboolean oldLight,oldDepthTest;
		glGetBooleanv(GL_LIGHTING,&oldLight);
		glGetBooleanv(GL_DEPTH_TEST,&oldDepthTest);
        
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH_TEST);
		
		//Draw a small sphere at the centre.
		LaserSphere::DrawSphere(centre,radius*0.1,Vector3D(1,1,1)-color);
		
		//Draw text to mark the control point
		//Turned off for the time being.
/*		
		glColor3f(0.3,0.3,1);
		QFont serifFont( "Arial", 20, QFont::Bold );
		widget->renderText(centre.X()-0.5*radius,centre.Y()-0.9*radius,centre.Z(),QString("%1%2").arg(name.c_str()).arg(id),serifFont);
*/		
		
		if(oldLight)
			glEnable(GL_LIGHTING);
		if(oldDepthTest)
			glEnable(GL_DEPTH_TEST);
		
	}
}
///Copy constructor.
LaserControlPoint::LaserControlPoint(const LaserControlPoint& s):LaserSphere(s)
{

}

///Make a clone.
LaserObject* LaserControlPoint::Clone()
{
	return new LaserControlPoint(*this);	
}

///Copy all parameters to a vector and return.
vector<double> LaserControlPoint::ParamVector()const
{
	return ((LaserSphere*)this)->ParamVector();
}

///Write to vrml file
void LaserControlPoint::WriteVRML(ofstream& file)
{

}

///Read from ascii format.
void LaserControlPoint::ReadAscii(ifstream& file)
{
	file>>centre.X()>>centre.Y()>>centre.Z();
	file>>radius;
	file>>color.X()>>color.Y()>>color.Z();
}

///Write to ascii format.
void LaserControlPoint::WriteAscii(ofstream& file)
{
	string header = "ControlPoint";
	file<<header<<" ";
	file<<centre.X()<<" "<<centre.Y()<<" "<<centre.Z()<<" ";
	file<<radius<<" ";
	file<<color.X()<<" "<<color.Y()<<" "<<color.Z()<<" ";
	file<<endl;
}

//Write to bnf file
void LaserControlPoint::WriteBnf(ofstream& file,int id)
{

}

void WriteVRMLObjects(string fileName,vector<LaserObject*>& laserObjects)
{
	ofstream file(fileName.c_str());
	int i;
	file<<"#VRML V2.0 utf8\n";
	for(i=0;i<laserObjects.size();i++)
	{
		cerr<<"WriteVRMLObjects: "<<i<<"/"<<laserObjects.size()<<"\r";
		
		file<<"\n\n#Saving Object number: "<<i
			<<" which is of type: "<<laserObjects[i]->Name()<<endl;
		laserObjects[i]->WriteVRML(file);
	}
	cerr<<"\rWriteVRMLObjects: Done!!!\n\n";
	file.close();
}

void WriteAsciiFile(string fileName,vector<LaserObject*> &objects)
{
	string header = "LaserObjectsFile";
	
	ofstream file(fileName.c_str());
	file<<header<<endl;
	for(int i=0;i<objects.size();i++)
	{
		objects[i]->WriteAscii(file);
	}
}

	
void WriteAsciiFile(char* fileName,vector<LaserObject*> &objects)
{
	string header = "LaserObjectsFile";
	
	ofstream file(fileName);
	file<<header<<endl;
	for(int i=0;i<objects.size();i++)
	{
		objects[i]->WriteAscii(file);
	}
}

void ReadAsciiFile(const char* fileName,vector<LaserObject*>& objects)
{
	string header = "LaserObjectsFile";
	string buff;
	
	ifstream file(fileName);
	file>>buff;
	if(buff==header)
	{
		
		while(1)
		{
			if(file.peek()==EOF) return;
			file>>buff;
			//cerr<<buff<<endl;
			if(file.peek()==EOF) return;
			if(buff=="Cylinder")
			{
				LaserCylinder* cyl = new LaserCylinder();
				cyl->ReadAscii(file);
				objects.push_back(cyl);
			}
			else if(buff=="Plane")
			{
				LaserPlane* plane = new LaserPlane();
				plane->ReadAscii(file);
				objects.push_back(plane);
			}
			else if(buff=="PlaneTR")
			{
				LaserPlaneTR* plane = new LaserPlaneTR();
				plane->ReadAscii(file);
				objects.push_back(plane);
			}
			else if(buff== "Sphere")
			{
				LaserSphere* sph = new LaserSphere();
				sph->ReadAscii(file);
				objects.push_back(sph);
			}
			else if(buff== "ControlPoint")
			{
				LaserControlPoint* obj = new LaserControlPoint();
				obj->ReadAscii(file);
				objects.push_back(obj);
			}
			else if(buff == "Torus")
			{
				LaserTorus* torus = new LaserTorus();
				torus->ReadAscii(file);
				objects.push_back(torus);
			}
			else if( buff == header)
			{
			 	//We can safely skip this one.
			}
			else
			{
				cerr<<"Unknown type ["<<buff<<"] encountered "<<__FILE__<<": "<<__LINE__<<"\n";
				file.close();
				return;
			}
		}
	}
	else
	{
		cerr<<"The file header is corrupt\n";
	}
}


static void PVector3D(char* msg=NULL,Vector3D v=Vector3D())
{
	cerr<<msg<<": ("<<v.X()<<" "<<v.Y()<<" "<<v.Z()<<endl;
}

	

void WriteBnfFile(char* fileName,vector<LaserObject*> &objects)
{
	ofstream file(fileName);

	//Measurements start.
	
	file<<"measurements:"<<endl;

	const int id_start = 10000;
	for(int i=0;i<objects.size();i++)
	{
		objects[i]->WriteBnf(file,id_start+i);		 
 	}
	file<<"endmeasurements:" <<endl;

	file.close();
	cerr<<fileName<<"  saved!!!\n";
}


//Save to file.
void LaserObjectsVector::Save(std::string fileName)const
{
	WriteAsciiFile(fileName.c_str(),*((vector<LaserObject*>*)this));
}

//Load from file.
void LaserObjectsVector::Load(std::string fileName)const
{
	ReadAsciiFile(fileName.c_str(),*((vector<LaserObject*>*)this));
}

//Transform all objects.
LaserObjectsVector& LaserObjectsVector::operator *=(const LaserTransform3D& t)
{
	for(int i=0;i<size();i++)
		(*(*this)[i]) *= t;
		
	return *this;
}

//Draw all objects.
void LaserObjectsVector::Draw()const
{
	for(int i=0;i<size();i++)
		(*this)[i]->Draw();
}

//Clear the vector.
void LaserObjectsVector::Clear()
{
	Delete(*this);
}

//Destroy the objects.
LaserObjectsVector::~LaserObjectsVector()
{
	//Clear();
}

//Sets the color for all objects.
LaserObjectsVector& LaserObjectsVector::SetColor(Vector3D v)
{
	for(int i=0;i<size();i++)
		(*this)[i]->SetColor(v);
		
	return *this;
}

//Find index of the closest object to query in this vector.
int LaserObjectsVector::FindClosest(LaserObject* query,double & minDiff)
{
	int ans = -1;
	minDiff = 1e20;
	for(int i=0;i<size();i++)
	{
		LaserObject *current = (*this)[i];
		
		if(current->Name() == query->Name())
		{
			double d = Mean(Abs(Diff(current->ParamVector(),query->ParamVector())));
			
			if(d<minDiff)
			{
				ans = i;
				minDiff = d;
			}
		}
	}
	return ans;
}
			
		
	
vector<int64> LaserObject::SelectPoints(const LaserPoints& pts,
			const vector<Vector3D>& normals,
			double normalSimilarity,
			double distanceThreshold)
{
	double dotThreshold = normalSimilarity;
	vector<int64> indices;
	for(int i=0;i<pts.size();i++)
	{
		if(fabs(DistanceFromPoint(pts[i]))<=distanceThreshold
		&& fabs(NormalAtPoint(pts[i]).DotProduct(normals[i]))>=dotThreshold)
			indices.push_back(i);
	}
	return indices;
}

ifstream& LaserObject::ReadIndices(ifstream& file)
{
	streampos original = file.tellg();	 //save current position
	
	//Read a string.
	string s;
	file>>s;
	
	if(s=="LaserObjectsIndices")
	{
		int count;
		file>>count;
		indices.resize(count);
		for(int i=0;i<indices.size();i++)
			file>>indices[i];
	}
	else
		file.seekg(original);
	return file;
}

ofstream& LaserObject::WriteIndices(ofstream& file)const
{
	if(indices.size())
	{
		file<<" LaserObjectsIndices "<<indices.size()<<" ";
		for(int i=0;i<indices.size();i++)
			file<<indices[i]<<" ";
	}
	return file;
}
	

vector<int64> LaserObject::SelectPointsDistance(const LaserPoints& pts,
			double distanceThreshold)
{
	vector<int64> indices;
	for(int i=0;i<pts.size();i++)
	{
		if(fabs(DistanceFromPoint(pts[i]))<=distanceThreshold)
			indices.push_back(i);
	}
	return indices;
} 

//Prints the objects.
void LaserObjectsVector::Print()const
{
	cerr<<"LaserObjectsVector with "<<size()<<" pointers\n";
	for(int i=0;i<size();i++)
	{
		cerr<<i<<"/"<<size()<<" "<<(*this)[i]->Name()<<": ";
		(*this)[i]->Print();
		cerr<<"\n";
	}
}
	
double LaserObject::SimilarityMeasure(LaserObject* query)
{
	if(Name()!=query->Name())
		return DBL_MAX;
		
	vector<double> my = ParamVector();
	vector<double> yours = query->ParamVector();
	
	return Mean(Abs(Diff(my,yours)));
}

void LaserObjectsVector::SaveVrml(string fileName)const
{
	ofstream file(fileName.c_str());
	
	file<<"#VRML V2.0 utf8\n";

	time_t currentTime;
	time ( &currentTime );
	file<<"\n# created on "<<ctime(&currentTime)<<endl;
	
	for(int i=0;i<size();i++)
		(*this)[i]->WriteVRML(file);
		
}

