
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
*   File made : August 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Fitting of geometric objects using ENLSIP
*
*--------------------------------------------------------------------*/
#include "GeometricFittingENLSIP.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "KNNFinder.h"
#include "Vector3D.h"
#include "RotationParameter.h"
#include "LaserPoints.h"
#include "LaserPointsUtility.h"
#include "LaserPointsProcessing.h"
#include "ENLSIPFitting.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"

#define SHOW_DEBUG 0


//Constructor.
ENLSIPCylinder::ENLSIPCylinder(const LaserPoints* pPts):ENLSIPFitting(7,20,2,1)
{
	pLaserPoints = pPts;
	if(pPts)
		obsCount = pPts->size();
	functionCounter = constraintCounter = 0;
}

///This function calculates the function values for all points.
bool ENLSIPCylinder::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPCylinder::EvaluateFunction \n";

	Vector3D axis(x[0],x[1],x[2]);
	axis = axis.Normalize();

	Vector3D pointOnAxis(x[3],x[4],x[5]);

	double radius = x[6];

	//convert point on axis to point on axis closest to origin.
	Vector3D p = pointOnAxis - axis*axis.DotProduct(pointOnAxis);

	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		Vector3D v = (Vector3D((*pLaserPoints)[i])-p);
		f[i] = ((axis.VectorProduct(v)).Length()-radius);

		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
		printf("Sumsq: %f \n",sumSq);
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPCylinder::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPCylinder::EvaluateConstraint with count: "<< count <<"\n";
	//Length of the axis should be one.
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - 1;
	
	//axis should be perpendicular to the closest point.
	f[1] = x[0]*x[3] + x[1]*x[4] + x[2]*x[5];
	
	//A bound constraint on the radius of the cylinder.
	const int ineqStart = 2;
	f[2+0] = x[6] + 1e6;
	f[2+1] = -x[6] + 1e6;

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

///A static function for executing the test.
void ENLSIPCylinder::ExecuteTest()
{
	cerr<<"Calling the ENLSIPCylinder::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test cylinder...";
	LaserPoints pts;
	double r = 10;
	for(double h=0;h<10;h+=1)
	{
		for(double t=0;t<M_PI;t+=0.1)
		{
			pts.push_back(LaserPoint(r*sin(t)+100,r*cos(t)+200,h+300));
		}
	}
	cerr<<"Done!!!\n";

	cerr<<"Making optimizer...";
	ENLSIPCylinder test(&pts);
	cerr<<"Done!!\n";

	//Initialize.
	vector<double> initial(7,0.9);
	initial[2] = 1;
	initial[6] = 0.1;

	Vector3D mean = pts.Mean();
	initial[3] = mean.X(); initial[4] = mean.Y(); initial[5] = mean.Z();

	//Print final result.
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
}


//Constructor.
ENLSIPTorus::ENLSIPTorus(const LaserPoints* pPts):ENLSIPFitting(8,200,1,0)
{
	pLaserPoints = pPts;
	if(pPts)
		obsCount = pPts->size();
	functionCounter = constraintCounter = 0;
}

#define SHOW_DEBUG 0
///This function calculates the function values for all points.
bool ENLSIPTorus::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPTorus::EvaluateFunction \n";

	Vector3D axis(x[0],x[1],x[2]);
	axis = axis.Normalize();

	Vector3D centre(x[3],x[4],x[5]);

	double R = x[6];
	double r = x[7];

	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		Vector3D point = (*pLaserPoints)[i];
		Vector3D v = (point-centre);

		if(v.Length()<1e-10 || axis.Length() < 1e-10)
		{
			f[i] = 0;
			continue;
		}
		Vector3D vav = axis.VectorProduct(v);
		double mf = sqrt(vav.DotProduct(vav));
		double mp = axis.DotProduct(v);
		f[i] = sqrt(pow(R-mf,2) + pow(mp,2)) - r;
		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
		printf("Sumsq: %f \n",sumSq);
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPTorus::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPTorus::EvaluateConstraint with count: "<< count <<"\n";
	
	//Only one constraint on the orientation of the torus.
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - 1;

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

///A static function for executing the test.
void ENLSIPTorus::ExecuteTest()
{
	cerr<<"Calling the ENLSIPTorus::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test torus...";
	LaserPoints pts = MakeTorus();;
	cerr<<"Done!! with "<<pts.size()<<"   points \n";


	cerr<<"Making optimizer...";
	ENLSIPTorus test(&pts);
	cerr<<"Done!!!\n";

	//Initialize.
	vector<double> initial(8,0.3);
	initial[2] = 1;

	Vector3D mean = pts.Mean();
	initial[3] = mean.X(); initial[4] = mean.Y(); initial[5] = mean.Z();
	initial[6] = 10;
	initial[7] = 2.5;
	
	//Print final result.
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
}


//Constructor.
ENLSIPPlane::ENLSIPPlane(const LaserPoints* pPts):ENLSIPFitting(4,200,1,0)
{
	pLaserPoints = pPts;
	if(pPts)
		obsCount = pPts->size();
	functionCounter = constraintCounter = 0;
}

///This function calculates the function values for all points.
bool ENLSIPPlane::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPPlane::EvaluateFunction \n";

	Vector3D axis(x[0],x[1],x[2]);
	double rho = x[3];

	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		f[i] = axis.DotProduct((Vector3D)((*pLaserPoints)[i])) - rho;
		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
		printf("Sumsq: %f \n",sumSq);
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPPlane::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPPlane::EvaluateConstraint with count: "<< count <<"\n";
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - 1;

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

///A static function for executing the test.
void ENLSIPPlane::ExecuteTest()
{
	cerr<<"Calling the ENLSIPPlane::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test plane...";
	LaserPoints pts = MakePlane();;
	cerr<<"Done!! with "<<pts.size()<<"   points \n";


	cerr<<"Making optimizer...";
	ENLSIPPlane test(&pts);
	cerr<<"Done!!!\n";

	//Initialize.
	vector<double> initial(4,1);

	Vector3D mean = pts.Mean();
	initial[3] = mean.DotProduct(Vector3D(initial[0],initial[1],initial[2]));

	//Print final result.
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
}

//Constructor.
ENLSIPSphere::ENLSIPSphere(const LaserPoints* pPts):ENLSIPFitting(4,200,0,0)
{
	pLaserPoints = pPts;
	if(pPts)
		obsCount = pPts->size();
	functionCounter = constraintCounter = 0;
}

///This function calculates the function values for all points.
bool ENLSIPSphere::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPSphere::EvaluateFunction \n";

	Vector3D centre(x[0],x[1],x[2]);
	double rho = x[3];

	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		Vector3D diff = (*pLaserPoints)[i] - centre;
		f[i] = diff.Length() - rho;
		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
		printf("Sumsq: %f \n",sumSq);
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPSphere::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPSphere::EvaluateConstraint with count: "<< count <<"\n";

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

///A static function for executing the test.
void ENLSIPSphere::ExecuteTest()
{
	cerr<<"Calling the ENLSIPSphere::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test sphere...";
	LaserPoints pts = MakeSphere(100,1000);;
	cerr<<"Done!! with "<<pts.size()<<"   points \n";


	cerr<<"Making optimizer...";
	ENLSIPSphere test(&pts);
	cerr<<"Done!!!\n";

	//Initialize.
	vector<double> initial(4,1);

	Vector3D mean = pts.Mean();
	for(int k=0;k<3;k++)
		initial[k] = mean[k];

	//Print final result.
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
}


///Fixed registration class.
///Each point in scan1 is known to be the correspond to the point in the scan2.
///This problem has closed form solutions but here we solve it iteratively.

ENLSIPRegistrationFixed::ENLSIPRegistrationFixed(const LaserPoints* _pPtsFixed , const LaserPoints* _pPtsMoving)
			:pPtsFixed(_pPtsFixed), pPtsMoving(_pPtsMoving),
			ENLSIPFitting(7,20,4,0,700,1)
{
	if(pPtsMoving)
		obsCount = pPtsMoving->size();
	functionCounter = constraintCounter = 0;	
}				

///This function calculates the function values for all points.
bool ENLSIPRegistrationFixed::EvaluateFunction(double* x,double* f,int count)
{
	if(!pPtsFixed || !pPtsMoving)
	{
		cerr<<"ENLSIPRegistrationFixed::EvaluateFunction invalid pointer "
			<<" pPtsFixed: "<<pPtsFixed
			<<" pPtsMoving: "<<pPtsMoving<<"\n";
			
		return true;
	}
			
	//Make transformation matrices.
	LaserPoints ptsTransformed = (*pPtsMoving) + Vector3D(x[4],x[5],x[6]);
	ptsTransformed = ptsTransformed*QuaternionRotation(x[0],x[1],x[2],x[3]);
	
	
	//Get the distances.
	for(int i=0;i<ptsTransformed.size();i++)
	{
		Vector3D diff = (*pPtsFixed)[i] - ptsTransformed[i];
		f[i] = diff.Length();
	}
	
	//Calculate chi-square and print it.
	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		sumSq += pow(f[i],2);
	}
	
	if(SHOW_DEBUG)
		printf("Fixed Registration Sumsq: %f with count: %d fixed_size: %d\n",sumSq,count,pPtsFixed->size());

	//return with status.
	return 0;
}

static Vector3D FixedMean;
///This function calculates residual value for the constraints.
bool ENLSIPRegistrationFixed::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationFixed::EvaluateConstraint with count: "<< count <<"\n";

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3] - 1;
	f[1] = x[4] - FixedMean.X();
	f[2] = x[5] - FixedMean.Y();
	f[3] = x[6] - FixedMean.Z();
	
	return 0;
}


///A static function for executing the test.
void ENLSIPRegistrationFixed::ExecuteTest()
{
	double noiseLevel = 150;
	cerr<<"Calling the ENLSIPRegistrationFixed::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test data...";
	
	double x[7] = {1,1,1,1,0,0,0};
	
	LaserPoints ptsFixed = MakeSinePoints();
	
	
	if(1)
	{
		ptsFixed.Read("/home/star/reg0.laser.gz");
	}
	
	int dataSize = 400;
	ptsFixed.resize(dataSize);
	ptsFixed[0] = LaserPoint(0,0,0);
	ptsFixed[1] = LaserPoint(1000,0,0);
	ptsFixed[2] = LaserPoint(0,1000,0);
	for(int i=3;i<dataSize;i++)
		ptsFixed[i] = LaserPoint(1000,1000,i*10);
		
	ptsFixed = ptsFixed + (ptsFixed.Mean()*-1);
	
	LaserPoints ptsMoving = ptsFixed.AddNoise(noiseLevel);
	
	ptsMoving = ptsMoving + (ptsMoving.Mean()*-1);

	//Make transformation matrices.
	QuaternionRotation quat(x[0],x[1],x[2],x[3]);
	
	Rotation3D rot(quat);
	Vector3D trans(x[4],x[5],x[6]);
	
	//Apply the transformation.
	ptsFixed = ptsFixed+trans;
	ptsFixed = ptsFixed*rot;
	
	
	cerr<<"Done\n";
	
	cerr<<"Making optimizer...";
	ENLSIPRegistrationFixed test(&ptsFixed,&ptsMoving);
	cerr<<"Done!!!\n";
	
	
	//Initialize.
	vector<double> initial(7,1);

	
	for(int k=0;k<4;k++)
		initial[k] = x[k]*1.1 + GenerateRandom(-0.5,0.5);
		
	Vector3D mean = ptsFixed.Mean() - ptsMoving.Mean();
	for(int k=4;k<7;k++)
		initial[k] = mean[k-4];//x[k]*1.00005;
		
	::FixedMean = mean;

	//Print initial values
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;
		
	LaserPoints before = ptsFixed;
	before.SetReflectance(10);
	LaserPoints tranformed = ptsMoving + Vector3D(initial[4],initial[5],initial[6]);
	tranformed = tranformed*QuaternionRotation(initial[0],initial[1],initial[2],initial[3]);
	before = before + tranformed.SetReflectance(20);
	before.SaveToAscii("before.pts");

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
		
	
	before = ptsFixed;
	before.SetReflectance(10);
	ptsMoving = ptsMoving +Vector3D(final[4],final[5],final[6]);
	ptsMoving = ptsMoving*Rotation3D(QuaternionRotation(final[0],final[1],final[2],final[3]));
	ptsMoving.SetReflectance(20);
	before = before + ptsMoving;
	before.SaveToAscii("after.pts");
	
	
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
	
	cerr<<"ENLSIPRegistrationFixed::ExecuteTest() is Done!!!\n";

}



//Class for ICP based registration
//The parameters are in the order:
//	0-3 quaternion
//	4-6 translation
#define SHOW_DEBUG 1
ENLSIPRegistrationICP::ENLSIPRegistrationICP(const LaserPoints* _pPtsFixed , const LaserPoints* _pPtsMoving,
				double _percentageThreshold,double _distanceThreshold,bool _useNormals)
:pPtsFixed(_pPtsFixed), pPtsMoving(_pPtsMoving),percentageThreshold(_percentageThreshold),
distanceThreshold(_distanceThreshold), useNormals(_useNormals),
ENLSIPFitting(7,20,1,0,7,0)
{
	if(pPtsMoving)
		obsCount = pPtsMoving->size();
	functionCounter = constraintCounter = 0;	
}				


///This function calculates the function values for all points.
bool ENLSIPRegistrationICP::EvaluateFunction(double* x,double* f,int count)
{
	if(!pPtsFixed || !pPtsMoving)
	{
		cerr<<"ENLSIPRegistrationICP::EvaluateFunction invalid pointer "
			<<" pPtsFixed: "<<pPtsFixed
			<<" pPtsMoving: "<<pPtsMoving<<"\n";
			
		return true;
	}
			

	LaserPoints ptsTransformed;
	ptsTransformed.resize(pPtsMoving->size());

	//Make transformation matrices.
	QuaternionRotation quat(x[0],x[1],x[2],x[3]);
	Rotation3D rot(quat);
	Vector3D trans(x[4],x[5],x[6]);

	//Apply the transformation.
	for(int i=0;i<pPtsMoving->size();i++)
	{
		Vector3D v = Vector3D((*pPtsMoving)[i]);
		v = rot*v;
		v = v+trans;
		ptsTransformed[i] = LaserPoint(v.X(),v.Y(),v.Z());
	}

	//Get the closest neighbours and distances.
	vector<int> indices;
	vector<double> distances;

	KNNFinder<LaserPoint> finder(*pPtsFixed);
	indices = finder.FindIndices(ptsTransformed,1,&distances);
	
	for(int i=0;i<distances.size();i++)
	{
		Vector3D diff = (*pPtsFixed)[i] - ptsTransformed[i];
		distances[i] = diff.Length();
	}
	
	//copy to output variables.
	copy(distances.begin(),distances.end(),f);

	//Calculate chi-square and print it.
	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		sumSq += pow(f[i],2);
	}
	
	if(SHOW_DEBUG)
		printf("ICP Sumsq: %f with count: %d fixed_size: %d\n",sumSq,count,pPtsFixed->size());

	//return with status.
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPRegistrationICP::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationICP::EvaluateConstraint with count: "<< count <<"\n";

	if(SHOW_DEBUG)
	{
		for(int i=0;i<count;i++)
			cerr<<"Constraint "<<i<<": "<<f[i]<<endl;
	}
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3] - 1;
	
	return 0;
}


///A static function for executing the test.
void ENLSIPRegistrationICP::ExecuteTest()
{
	cerr<<"Calling the ENLSIPRegistrationICP::ExecuteTest()\n";

	//make a temporary optimizer object
	cerr<<"Making test data...";
	
	double x[7] = {1,1,1,1,2000,3000,4000};
	
	LaserPoints ptsFixed = MakeSinePoints();
	
	if(1)
	{
		ptsFixed.Read("/home/star/reg0.laser.gz");
	}
	LaserPoints ptsMoving = ptsFixed.AddNoise(50);

	//Make transformation matrices.
	QuaternionRotation quat(x[0],x[1],x[2],x[3]);
	
	for(int i=0;i<4;i++)
		 ;//x[i] = quat[i]; 
	
	Rotation3D rot(quat);
	Vector3D trans(x[4],x[5],x[6]);
	
	//Apply the transformation.
	ptsFixed = ptsFixed*rot;
	ptsFixed = ptsFixed+trans;
	
	cerr<<"Done\n";
	
	cerr<<"Making optimizer...";
	ENLSIPRegistrationICP test(&ptsFixed,&ptsMoving);
	cerr<<"Done!!!\n";
	
	
	//Initialize.
	vector<double> initial(7,1);

	for(int k=0;k<4;k++)
		initial[k] = x[k]*1.1 + GenerateRandom(-0.5,0.5);
		
	for(int k=4;k<7;k++)
		initial[k] = x[k]*1.00005;

	//Print final result.
	for(int i=0;i<initial.size();i++)
		cerr<<"Initial: "<<i<<": "<<initial[i]<<endl;
		
	LaserPoints before = ptsFixed;
	before.SetReflectance(10);
	LaserPoints tranformed = ptsMoving*QuaternionRotation(initial[0],initial[1],initial[2],initial[3]);
	tranformed = tranformed + Vector3D(initial[4],initial[5],initial[6]);
	
	before = before + tranformed.SetReflectance(20);
	before.SaveToAscii("before.pts");

	//Opitmize.
	cerr<<"Now optimizing\n";
	vector<double> final = test.Optimize(initial);

	//Print final result.
	for(int i=0;i<final.size();i++)
		cerr<<"Final: "<<i<<": "<<final[i]<<endl;
		
	
	before = ptsFixed;
	before.SetReflectance(10);
	ptsMoving = ptsMoving*Rotation3D(QuaternionRotation(final[0],final[1],final[2],final[3]));
	ptsMoving = ptsMoving +Vector3D(final[4],final[5],final[6]);
	ptsMoving.SetReflectance(20);
	before = before + ptsMoving;
	before.SaveToAscii("after.pts");
	
	
	cerr<<"FunctionCounter: "<<test.FunctionCounter()<<endl;
	cerr<<"ConstraintCounter: "<<test.ConstraintCounter()<<endl;
	
	cerr<<"ENLSIPRegistrationICP::ExecuteTest() is Done!!!\n";

}

double DistanceCone(const Vector3D& p, const Vector3D& a, const Vector3D& t, double theta)
{
	//calculate distance of point p from the cone.
	//cone is given by [axis apex theta]

    double h = (p-t).Length();
    
	if(fabs(h)<1e-12)
		return 0;
		
    Vector3D a1 = (p-t).Normalize();
    
    double dd = a1.DotProduct(a);
    
    if (dd>0)
	{
        double phi = acos(dd) - theta;
        return h*sin(phi);
	}
    else
        return h;
}
