
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
#ifndef _LASER_OBJECTS_H____
#define _LASER_OBJECTS_H____

#include "LaserPoints.h"
#include "Positions3D.h"
#include "Plane.h"
#include "MyTypeDefinitions.h"
#include "VrmlUtility.h"
#include "StlUtilities.h"

class QGLWidget;
class LaserTransform3D;
class GLObject;

//Interface class. Defines all necessary virtual functions.
class LaserObject
{
	public:
	
	LaserObject(Vector3D c = Vector3D(1,1,1),int __id=-1):color(c),id(__id)
	{
	}
	
	///copy constructor.
	LaserObject(const LaserObject& obj)
	{
		if(this==&obj)
			return;
		
		color = obj.color;
		indices = obj.indices;
		name = obj.name;
		description = obj.description;
		id = obj.id;
	}
	
	///Prints to console the contents of object
	virtual void Print(){printf("%s: \n",this->Name().c_str());};
	
	///Draws object in opengl.
	virtual void Draw(){};
	
	///Draws object in opengl.
	virtual void Draw(QGLWidget* widget)
	{
		this->Draw();
	};
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform)
	{
		cerr<<"operator * (LaserTransform3D) not implemented for LaserObject\n";
		return this;
	}
	
	//return parameter count.
	virtual int ParamCount() const
	{
		return 0;
	}
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values)
	{
	
	}
	
	//Save to vector
	virtual vector<double> ToVector()const
	{
		return vector<double>();
	}
	
	//Return similarity measure with another object.
	virtual double Similarity(LaserObject* query)
	{
		cerr<<"LaserObject::Similarity is not meaningfull\n";
		return DBL_MAX;
	}
	
	
	
	///Draws in specified color.
	void Draw(Vector3D newColor,QGLWidget* widget)
	{
		Vector3D old = color;
		color = newColor;
		this->Draw(widget);
		color = old;
	}
	
	///Clones current object and return the memory (memory management is callers headache)
	virtual LaserObject* Clone(){};
	
	///virtual destructor is necessary.
	virtual ~LaserObject(){};
	
	///Return name.
	virtual string Name(){return "LaserObject";};
	
	///Write to VRML file.
	virtual void WriteVRML(ofstream& file){file<<"\n#Error: LaserObject cannot be saved\n";}
	
	///Write to simple ascii format.
	virtual void WriteAscii(ofstream& file){}
	
	///Read indices from Ascii file.
	ifstream& ReadIndices(ifstream& file);

	///Write indices
	ofstream& WriteIndices(ofstream& file)const;

	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;}
	
	///Calculate distance from a single point.
	virtual double DistanceFromPoint(Vector3D v)const{return 0;}
	
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const{return Vector3D(0,0,1);};
	
	///Write to bnf format (for sharing with piper)
	virtual void WriteBnf(ofstream& file,int id){}
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const{return vector<double>();}
	
	void SetColor(Vector3D v)
	{
		color = v;
	}
	
	Vector3D GetColor() const
	{
		return color;
	}
	
	///Calculate distance from a vector of something convertible to a point.
	template<class T>
	std::vector<double> Distance(const std::vector<T>& pts)const
	{
		std::vector<double> distances(pts.size());
		
		for(int i=0;i<pts.size();i++)
			distances[i] = this->DistanceFromPoint(Vector3D(pts[i]));
		return distances;
	}
	
	///Calculate ideal normal at a vector of anything convertible to point.
	template<class T>
	std::vector<Vector3D> Normal(const std::vector<T>& pts)const
	{
		std::vector<Vector3D> normals(pts.size());
		
		for(int i=0;i<pts.size();i++)
		{
			normals[i] = this->NormalAtPoint(Vector3D(pts[i]));
		}
		return normals;
	}
	
	
	void SetDescription(string str)
	{
		description = str;
	}
	
	string GetDescription() const
	{
		return description;
	}
	
	void SetName(string str)
	{
		name = str;
	}
	
	string GetName() const
	{
		return name;
	}
	
	void SetId(int64 i)
	{
		id = i;
	}
	
	int64 GetId() const
	{
		return id;
	}
	
	void SetIndices(const vector<int64>& v)
	{
		indices = v;
	}
	
	void SetIndices(const vector<int>& v)
	{
		Copy(v,indices);
	}
	
	vector<int64>& GetIndices()
	{
		return indices;
	}
	
	//Select the points from a given scan.
	vector<int64> SelectPoints(const LaserPoints& pts,
			const vector<Vector3D>& normals,
			double normalSimilarity = 10,
			double distanceThreshold = 0.1);
			
	vector<int64> SelectPointsDistance(const LaserPoints& pts,double distanceThreshold);
	
	double SimilarityMeasure(LaserObject* query);
	

public: 
	//Color to draw the object with.
	Vector3D color;		
	
	//indices of points used to model this object.
	vector<int64> indices;
	
	//name of the object.
	string name;
	
	//description of the object.
	string description;
	
	//unique (id) of the object.
	int64 id;
};


///Class to represent a cylinder.
class LaserCylinder:public LaserObject
{
public:
	Vector3D pointBegin,pointEnd;
	double radius;
	
	///Constructor
	LaserCylinder(Vector3D a=Vector3D(0,0,0),Vector3D b=Vector3D(1,0,0),double c=5,Vector3D cc=Vector3D(1,1,1));
	
	LaserCylinder(const LaserPoints& pts, Vector3D axis,Vector3D position,double radius,Vector3D color=Vector3D(0.8,0.8,8.8));
		
	///Print to screen.
	void Print();

	///Draw the object.
	void Draw();
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
		
	///Copy constructor.
	LaserCylinder(const LaserCylinder& c);
		
	///Make a clone.
	LaserObject* Clone();
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
	
	///Calculate distance from a single point.
	virtual double DistanceFromPoint(Vector3D v)const;
	
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const;
	
	///Return name.
	string Name(){return "LaserCylinder";};
	
	///double length of cylinder.
	double Length(){return (pointBegin-pointEnd).Length();};
	
	///Write VRML.
	void WriteVRML(ofstream& file);

	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///write to ascii format.
	void WriteAscii(ofstream& file);

	///Write to bnf file
	void WriteBnf(ofstream& file,int id);
	
	///Return the axis direction.
	Vector3D AxisDirection()const;
	
	///Update bounds by using points in a given laser points.
	///Donot change axis or radius.
	void UpdateBounds(const LaserPoints& pts);
	
	///Make angular histogram using the specified number of bins.
	///Uses ideal normals.
	vector<int> AngularHistogram(const LaserPoints& pts, int count = 45);
	
	///Make angular histogram using the specified number of bins.
	///Uses provided normals..
	vector<int> AngularHistogram(const std::vector<Vector3D>& normals, int count = 45);
	
	//Return the point closest to origin.
	Vector3D PointClosestToOrigin()const;
	
	///Return the radius of the object.
	double Radius()const 
	{
		return radius;
	};
		
private:
	///Private function to render.
	void DrawCylinder(Vector3D point1, Vector3D point2, double radius, Vector3D color,bool drawEndCaps);
	
	///Private function to write VRML.
	void WriteVRMLCylinder(ofstream& file);
};


///Class to represent a sphere object.
class LaserSphere:public LaserObject
{
public:
	Vector3D centre;
	double radius;
	
	///Constructor.
	LaserSphere(Vector3D a=Vector3D(0,0,0),double c=1,Vector3D cc=Vector3D(1,1,1));
	
	///Print to console.
	void Print();
	
	///Draw in OpenGL.
	void Draw();
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
		
	///Copy constructor.
	LaserSphere(const LaserSphere& s);
	
	///Calculate distance from a single point.
	virtual double DistanceFromPoint(Vector3D v)const;
	
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const;
	
	///Make a clone.
	LaserObject* Clone();
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
		
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
		
	///Write to vrml file
	void WriteVRML(ofstream& file);
	
	///Return name.
	string Name(){return "LaserSphere";};

	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///Write to ascii format.
	void WriteAscii(ofstream& file);
	
	//Write to bnf file
	void WriteBnf(ofstream& file,int id);
	
protected:
	///Private function to render.
	void DrawSphere(Vector3D centre, double radius, Vector3D color);
	
	///Private function to write sphere.
	void WriteVRMLSphere(ofstream& file);
};


///Plane fitted to laser points. 
///Keeps info about convex hull too needed for drawing.
class LaserPlane:public LaserObject
{
	public:
	Plane plane;
	Positions3D poss;
	
	///Constructor.
	LaserPlane(Plane p=Plane(),Positions3D ps=Positions3D(), Vector3D cc=Vector3D(1,1,1));
	
	///Render in Opengl.
	void Draw();
	
	virtual void Print();
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
		
	///Copy constructor.
	LaserPlane(const LaserPlane& p);
	
	///Make a clone.
	LaserObject* Clone();
	
	///Update convex hull from given laser points.
	void UpdateHull(const LaserPoints& pts);
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
	
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
	
	///Write to vrml.
	void WriteVRML(ofstream& file);
	
	///Return name.
	string Name(){return "LaserPlane";};
	
	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///Write to ascii format.
	void WriteAscii(ofstream& file);
	
	double DistanceFromPoint(Vector3D v)const;
	
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const;
	
	///Return the normal direction.
	Vector3D NormalDirection()const
	{
		return plane.Normal();
	}
	
	///Return the normal.
	Vector3D Normal()const
	{
		return plane.Normal();
	}
	
	///Return Rho.
	double Rho()const
	{
		return plane.Distance();
	}
	
	///set Rho.
	void SetRho(double newDist)
	{
		plane.Distance() = newDist;	
	}
	
	///set Normal
	void SetNormal(Vector3D n)
	{
		plane.Normal() = n;	
	}
	
	///Returns distance from origin.
	double DistanceFromOrigin()const
	{
		return plane.Distance();
	}
	
	//Write to bnf file (doesn't make sense currently)
	void WriteBnf(ofstream& file,int id);
	
	

private:
	///Private rendering function.	
	void DrawPlane(Plane p,Vector3D color);
	
	///Write plane to VRML file.
	void WriteVRMLPlane(ofstream& file);
};

///Plane fitted to laser points. 
///This one keeps a triangulate representation instead of a convex hull.
//is written as a list of all triangles.
///Keeps info about convex hull too needed for drawing.
class LaserPlaneTR:public LaserObject
{
	public:
	Plane plane;
	LaserPoints points;
	
	///Constructor.
	LaserPlaneTR(Plane pl=Plane(),const LaserPoints &p=LaserPoints(),Vector3D cc=Vector3D(1,1,1));
	
	///Render in Opengl.
	void Draw();
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
		
	///Copy constructor.
	LaserPlaneTR(const LaserPlaneTR& p);
	
	///Make a clone.
	LaserObject* Clone();
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
	
	///Write to vrml.
	void WriteVRML(ofstream& file);
	
	///Calculate distance from a single point.
	virtual double DistanceFromPoint(Vector3D v)const;
	
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const;
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
	
	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///Return name.
	string Name(){return "LaserPlaneTR";};
	
	///Write to ascii format.
	void WriteAscii(ofstream& file);
	
	//Write to bnf file (doesn't make sense currently)
	void WriteBnf(ofstream& file,int id);
	
	//Get bounds.
	void GetBoundingBox(Vector3D& minimum,Vector3D& maximum)
	{
		DataBoundsLaser b = points.DeriveDataBounds(0);
		minimum = b.Minimum();
		maximum = b.Maximum();
	}

private:
	///Private rendering function.	
	void DrawPlaneTR(Plane p,Vector3D color);
	
	///Write plane to VRML file.
	void WriteVRMLPlaneTR(ofstream& file);
};


///Class representing torus modeled from laser points.
class LaserTorus:public LaserObject
{
public:
	Vector3D axis,position;
	double R;
	double r;
	
	///Constructor.
	LaserTorus(
			Vector3D a=Vector3D(0,0,0),
			Vector3D b=Vector3D(0,0,1),
			double tR=2,
			double tr=1,
			Vector3D cc=Vector3D(1,1,1));

	
	///Print to screen.
	void Print();
	
	///Render in Opengl.
	void Draw();
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
		
	///Copy constructor.
	LaserTorus(const LaserTorus& t);
	
	///Clone this object.
	LaserObject* Clone();
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
	
	///Calculate distance from a single point.
	virtual double DistanceFromPoint(Vector3D v)const;
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
	
			
	///Give ideal normal direction from the surface at a given point.
	virtual Vector3D NormalAtPoint(Vector3D v)const;
	
	///Write to vrml file.
	void WriteVRML(ofstream& file);
	
	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///Write to ascii format.
	void WriteAscii(ofstream& file);
	
	//Write to bnf file
	void WriteBnf(ofstream& file,int id);
	
private:
	///Draw torus.
	void DrawTorus(Vector3D axis, Vector3D centre,double outerRadius,double innerRadius,Vector3D color);
	
	///Write torus to Vrml.
	void WriteVRMLTorus(ofstream& file);
};


///Class to represent a control point, actually its quite similar to sphere.
///Though we can't fit it.
class LaserControlPoint:public LaserSphere
{
public:
	
	///Constructor.
	LaserControlPoint(Vector3D a=Vector3D(0,0,0),double c=1,Vector3D cc=Vector3D(1,1,1),int64 id=-1,string name="ControlPoint");
	
	///Print to console.
	void Print();
	
	///Draw in OpenGL.
	virtual void Draw();
	
	///Draw on a given context.
	void Draw(QGLWidget* widget);
	
	///Convert to a GLObject.Its the job of the calling routine to free the pointer.
	virtual GLObject* ToGLObject(){return NULL;};
		
	///Copy constructor.
	LaserControlPoint(const LaserControlPoint& s);
	
	///Make a clone.
	LaserObject* Clone();
	
	///Copy all parameters to a vector and return.
	virtual vector<double> ParamVector()const;
	
	//Multiply with the transform.
	virtual LaserObject* operator *=(const LaserTransform3D& transform);
	
	
	//Return number of paramameters.
	virtual int ParamCount() const;
	
	//Load from vector double.
	virtual void FromVector(const vector<double>& values);
		
	//Save to vector
	virtual vector<double> ToVector()const;
	
	//Returns similarity with another object.
	virtual double Similarity(LaserObject* query);
	
	
			
	///Write to vrml file
	void WriteVRML(ofstream& file);
	
	///Return name.
	string Name(){return "LaserControlPoint";};

	///Read from ascii format.
	void ReadAscii(ifstream& file);
	
	///Write to ascii format.
	void WriteAscii(ofstream& file);
	
	//Write to bnf file
	void WriteBnf(ofstream& file,int id);
	
private:
	
};

//Read write functions for laser objects.
//Declaration of functions defined in another file.
void WriteAsciiFile(char* fileName,vector<LaserObject*> &objects);
void WriteBnfFile(char* fileName,vector<LaserObject*> &objects);
void ReadAsciiFile(char* fileName,vector<LaserObject*>& objects);

//A vector of laser objects and the function to operate on them.
class LaserObjectsVector:public std::vector<LaserObject*>
{
public:
	//Default constructor.
	LaserObjectsVector()
	{
	}
	
	//constructor from vector.
	LaserObjectsVector(const vector<LaserObject*>& v)
	{
		*((vector<LaserObject*>*)this) = v;
	}
	
	//Save to file.
	void Save(std::string fileName)const;
	
	//Load from file.
	void Load(std::string fileName)const;
	
	//Save vrml file.
	void SaveVrml(std::string fileName)const;
	
	//Transform all objects.
	LaserObjectsVector& operator *=(const LaserTransform3D& t);
	
	//Find index of the closest object to query in this vector.
	int FindClosest(LaserObject* query,double& minDiff);
	
	//Draw all objects.
	void Draw()const;
	
	//Clear the vector.
	void Clear();
	
	//Prints the objects.
	void Print()const;
	
	//Sets the color for all objects.
	LaserObjectsVector& SetColor(Vector3D v);
	
	//Destroy the objects.
	~LaserObjectsVector();
};



#endif //_LASER_OBJECTS_H_


