
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
*   Purpose   : Provides functions for fitting different geometric entities to LaserPoints.
*
*--------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <iostream>
#include <ANN.h>
#include "LaserObjects.h"
#include "Orientation3D.h"
#include "GeometricFittingENLSIP.h"
#include "LaserPointsFitting.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"
#include "VisualizationUtility.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
#include "KNNFinder.h"
#include "ShMacro.h"
#include "Macros.h"
#include "IOFunctions.h"

#include "newmatap.h"                // need matrix applications
#include "LaserPoints.h"
#include "newmatio.h"                // need matrix output routines

using namespace NEWMAT;              // access NEWMAT namespace
using namespace std;




/** Approximate cylinder using plane fitting on Gaussian sphere.
	The steps of algorithm are as follows:
	-Estimate normals using kNN
	-Fit plane to normals plotted in 3D (Find normal direction of great circle on gaussian sphere)
	-normal of plane approximates cylinder axis.
	-point closest to origin is approximated by laser point mean
	-radius is given a constant value (its linear if other six parameter are approximately right)
	
*/	
LaserCylinder 
ApproximateCylinder(
			LaserPoints& laserPoints, int kNN )
{
	if(laserPoints.size()<10)
	{
		cerr<<"Too few points in ApproximateCylinder\n";
		return LaserCylinder();	
	}

	 LaserPoints normalPoints = laserPoints.Normals(kNN);

	//Add extra points on origin.
	int oldSize = normalPoints.size();
	normalPoints.reserve(normalPoints.size()*2);
	
	for(int i=0; i<oldSize;i++)
		normalPoints.push_back(LaserPoint(0,0,0));
		
	Vector3D closestPoint, axis;
	double radius;
	
	//Get the normal to the plane fitted on this Gaussian sphere.
	axis = normalPoints.Normal();
	
	//Give radius any value.
	radius = 10;
	
	//Closest point should be well approximated by laserPoints mean.
	closestPoint = laserPoints.Mean();
	
	//Return with the approximated cylinder
	return LaserCylinder(laserPoints,axis,closestPoint,radius);
}				

///Fit a cylinder to laser points using the given approximation.
LaserCylinder FitCylinder(LaserPoints& laserPoints, const LaserCylinder& approximate)
{
	//Check for valid sizes.
	if(laserPoints.size()<10)
	{
		cerr<<"Invalid size in FitCylinder\n";
		return LaserCylinder();
	}
	DEBUG("FitCylinder");
	
	ENLSIPCylinder fitter(&laserPoints);
	
	
	cerr<<"*** Calling ENLSIPCylinder::Optimize ***\n";
	vector<double> final = fitter.Optimize(approximate.ToVector());
	
	NEWMAT::Matrix covF, covO, covC;
	//cerr<<"Finding covariance...";
	fitter.Covariance(final,1,&covF,&covO,&covC);
	//cerr<<"Done\n";
	cerr<<"Full: "<<covF;
	cerr<<"Obs: "<<covO;
	cerr<<"Cons: "<<covC;
	
	LaserCylinder ans;
	ans.FromVector(final);
	ans.UpdateBounds(laserPoints);
	
	return ans;
}



//Calculate 2D convex hull.
// Assume that a class is already given for the object:
//    Point with coordinates {float x, y;}
//===================================================================
 
typedef struct Point{float x;float y;float z;};

// isLeft(): tests if a point is Left|On|Right of an infinite line.
 //    Input:  three points P0, P1, and P2
 //    Return: >0 for P2 left of the line through P0 and P1
 //            =0 for P2 on the line
 //            <0 for P2 right of the line
 //    See: the January 2001 Algorithm on Area of Triangles
 static float
 isLeft( Point P0, Point P1, Point P2 )
 {
     return (P1.x - P0.x)*(P2.y - P0.y) - (P2.x - P0.x)*(P1.y - P0.y);
 }
 //===================================================================
 static bool ComparePointOld(const Point &a, const Point &b) 
{
    if(a.x!=b.x)
		return (a.x > b.x);
	else
		return (a.y > b.y);
}
 static bool ComparePoint(const Point &a, const Point &b) 
{
    if(a.y>b.y)
		return true;
	else if(a.y<b.y)
		return false;
	else 
		return (a.x > b.x);
} 

 

// chainHull_2D(): Andrew's monotone chain 2D convex hull algorithm
 //     Input:  P[] = an array of 2D points 
 //                   presorted by increasing x- and y-coordinates
 //             n = the number of points in P[]
 //     Output: H[] = an array of the convex hull vertices (max is n)
 //     Return: the number of points in H[]
 static int
 chainHull_2D( Point* P, int n, Point* H)
 {
     sort(P,P+n,ComparePoint);
	 // the output array H[] will be used as the stack
     int    bot=0, top=(-1);  // indices for bottom and top of the stack
     int    i;                // array scan index
 
     // Get the indices of points with min x-coord and min|max y-coord
     int minmin = 0, minmax;
     float xmin = P[0].x;
     for (i=1; i<n; i++)
         if (P[i].x != xmin) break;
     minmax = i-1;
     if (minmax == n-1) {       // degenerate case: all x-coords == xmin
         H[++top] = P[minmin];
		 if (P[minmax].y != P[minmin].y) // a nontrivial segment
		 {
             H[++top] = P[minmax];
		 }
         H[++top] = P[minmin];           // add polygon endpoint

         return top+1;
     }
 
     // Get the indices of points with max x-coord and min|max y-coord
     int maxmin, maxmax = n-1;
     float xmax = P[n-1].x;
     for (i=n-2; i>=0; i--)
         if (P[i].x != xmax) break;
     maxmin = i+1;
 
     // Compute the lower hull on the stack H
     H[++top] = P[minmin];      // push minmin point onto stack
     i = minmax;
     while (++i <= maxmin)
     {
         // the lower line joins P[minmin] with P[maxmin]
         if (isLeft( P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
             continue;          // ignore P[i] above or on the lower line
 
         while (top > 0)        // there are at least 2 points on the stack
         {
             // test if P[i] is left of the line at the stack top
             if (isLeft( H[top-1], H[top], P[i]) > 0)
                 break;         // P[i] is a new hull vertex
             else
                 top--;         // pop top point off stack
         }
         H[++top] = P[i];       // push P[i] onto stack
     }
 
     // Next, compute the upper hull on the stack H above the bottom hull
     if (maxmax != maxmin)      // if distinct xmax points
	 {
         H[++top] = P[maxmax];  // push maxmax point onto stack
	}
     bot = top;                 // the bottom point of the upper hull stack
     i = maxmin;
     while (--i >= minmax)
     {
         // the upper line joins P[maxmax] with P[minmax]
         if (isLeft( P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
             continue;          // ignore P[i] below or on the upper line
 
         while (top > bot)    // at least 2 points on the upper stack
         {
             // test if P[i] is left of the line at the stack top
             if (isLeft( H[top-1], H[top], P[i]) > 0)
                 break;         // P[i] is a new hull vertex
             else
                 top--;         // pop top point off stack
         }
         H[++top] = P[i];       // push P[i] onto stack
     }
     if (minmax != minmin)
	 {
         H[++top] = P[minmin];  // push joining endpoint onto stack
	 }
 
     return top+1;
 }

///Calculate and return 2D-convex hull of points projected to given plane normal.
int
GetConvexHull(const LaserPoints& laserPoints,
			Vector3D& normal,
			Positions3D& positions)
{

	//reize positions to zero. will fill with hull points later on.
	positions.resize(0);

	//local variables for calculating the convex hull.
	bool closeToVertical = false;
	Rotation3D rot, invRot;
	int pointSize = laserPoints.size();

	//In rare cases the hull may contain one more point than input data. So allocate more.
	Point* rotatedPoints = new Point[pointSize+10];
	Point* hullPoints = new Point[pointSize+10];
	if (fabs(normal.Z()) != 1)
	{
		closeToVertical = true;
		Vector3D v(0,0,1);
		Vector3D rotAxis = v.VectorProduct(normal);
		rotAxis = rotAxis.Normalize();
		Rotation3D tempRot(rotAxis, Angle(v,normal));
		rot = tempRot;
		invRot = rot.Transpose();
		for(int i=0;i<pointSize;i++)
		{
			Vector3D v = rot*Vector3D(laserPoints[i]);
			rotatedPoints[i].x = v.X();
			rotatedPoints[i].y = v.Y();	
			rotatedPoints[i].z = v.Z();
		}
	}
	else
	{
		for(int i=0;i<pointSize;i++)
		{
			Vector3D v = Vector3D(laserPoints[i]);
			rotatedPoints[i].x = v.X();
			rotatedPoints[i].y = v.Y();
			rotatedPoints[i].z = v.Z();
		}
	}
	int hullCount = chainHull_2D( rotatedPoints, pointSize, hullPoints);
	//cerr<<"hullCount: "<<hullCount<<endl;
	for(int i=0;i<hullCount;i++)
	{
		Vector3D v;
		if(closeToVertical)
		{
			v = Vector3D(hullPoints[i].x,hullPoints[i].y,hullPoints[i].z);
			v = invRot*v ;
			positions.push_back(Position3D(v));
		}
		else
		{
			v = Vector3D(hullPoints[i].x,hullPoints[i].y,hullPoints[i].z);
			positions.push_back(Position3D(v));
		}	
	}
	delete[] hullPoints;
	delete[] rotatedPoints;

	return 0;
}			

///Fit plane to laser points.
LaserPlane 
FitPlane(LaserPoints& laserPoints)
{
	//Check for valid sizes.
	if(laserPoints.size()<4)
	{
		cerr<<"Invalid size in FitPlane\n";
		return LaserPlane();
	}

	//local variables.
	double rho, residual;

	Vector3D normal = laserPoints.Normal(&residual, &rho);
	
	Positions3D convexHull;
	GetConvexHull(laserPoints,normal,convexHull);
	
	Plane plane;
	plane.SetNormal(normal);
	plane.SetDistance(rho);
	return LaserPlane(plane,convexHull);
}


///Approximate LaserSphere.
LaserSphere ApproximateSphere(const LaserPoints& laserPoints)
{
	vector<double> extents = laserPoints.Extents();
	
	return LaserSphere(laserPoints.Mean(),Max(extents));	
}

///Fit sphere using least squares on orhthogonal distance.
LaserSphere FitSphere(LaserPoints& laserPoints, const LaserSphere& approximate)
{
	ENLSIPSphere fitter(&laserPoints);
	
	vector<double> final = fitter.Optimize(approximate.ToVector());
	
	LaserSphere ans;
	ans.FromVector(final);
	return ans;
}


///Approximate torus.	
LaserTorus ApproximateTorus(LaserPoints& laserPoints)
{
	//TODO: implement multiple cylinder based algorithm described in the thesis.
	return LaserTorus();	
}		

///Fit torus to laser points using the given approximate values.
LaserTorus FitTorus(LaserPoints& laserPoints, const LaserTorus& approximate)
{
	//Check for valid sizes.
	if(laserPoints.size()<20)
	{
		cerr<<"Invalid size in FitTorus\n";
		return LaserTorus();
	}
	
	ENLSIPTorus fitter(&laserPoints);
	
	vector<double> out = fitter.Optimize(approximate.ToVector());
	
	LaserTorus ans;
	ans.FromVector(out);
	return ans;

}


/*
mx = mean(X);
my = mean(Y);

for k=1:3
    X(:,k) = X(:,k) - mx(k);
    Y(:,k) = Y(:,k) - my(k);
end;

M = X'*Y;

N = [trace(M) M(2,3)-M(3,2) M(3,1)-M(1,3) M(1,2)-M(2,1);...
    M(2,3)-M(3,2)  M(1,1)-M(2,2)-M(3,3) M(1,2)+M(2,1) M(3,1)+M(1,3);...
    M(3,1)-M(1,3) M(1,2)+M(2,1) -M(1,1)+M(2,2)-M(3,3) M(2,3)+M(3,2);...
    M(1,2)-M(2,1) M(3,1)+M(1,3) M(2,3)+M(3,2) -M(1,1)-M(2,2)+M(3,3)];

[a b] = eig(N);

R = quat2rot((a(:,4)'));

Diff = Y - (R*X')'	
*/

//Find R and T so that:
//Y = R*X + T
void FindTransformation(const LaserPoints& X, const LaserPoints& Y,
			Rotation3D& rot, Vector3D& trans)
{
	Tracer tr("FindTransformation");
	//Find mean of points.
	Vector3D mx = X.Mean();
	Vector3D my = Y.Mean();

	//Allocate variance matrix.
	NEWMAT::Matrix M(3,3);
	M = 0.0;
	
	//fill the M matrix
	for(int i=1;i<=X.size();i++)
	{
		Vector3D vx = X[i-1] - mx;
		Vector3D vy = Y[i-1] - my;
		for(int x=1;x<=3;x++)
			for(int y=1;y<=3;y++)
				M(x,y) = M(x,y) + vx[x-1]*vy[y-1];
	}
	
	if(M.NormFrobenius()<1e-26)
	{
		rot = Rotation3D();
		trans = Vector3D();
		
		return;
	}
		
	NEWMAT::Matrix N(4,4);
	
	
	//1st row.
	N(1,1) = M(1,1) + M(2,2) + M(3,3);
	N(1,2) = M(2,3)-M(3,2);
	N(1,3) = M(3,1)-M(1,3); 
	N(1,4) = M(1,2)-M(2,1);
	
	//2nd row.
    N(2,1) = M(2,3)-M(3,2);
	N(2,2) = M(1,1)-M(2,2)-M(3,3);
	N(2,3) = M(1,2)+M(2,1);
	N(2,4) = M(3,1)+M(1,3);
	
	//3rd row.
    N(3,1) = M(3,1)-M(1,3);
	N(3,2) = M(1,2)+M(2,1);
	N(3,3) = -M(1,1)+M(2,2)-M(3,3);
	N(3,4) = M(2,3)+M(3,2);
	
	//4th row.
    N(4,1) = M(1,2)-M(2,1);
	N(4,2) = M(3,1)+M(1,3);
	N(4,3) = M(2,3)+M(3,2);
	N(4,4) = -M(1,1)-M(2,2)+M(3,3);
	
	SymmetricMatrix Ns;
	Ns<< N;
	
	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(4);
	NEWMAT::Matrix V(4,4);
	
	Jacobi(Ns,D,V);
	
	//The eigenvector with maximum eigenvalues is the one we want.
	NEWMAT::Matrix q = V.Column(4);
	
	rot = Rotation3D(QuaternionRotation(q(1,1),q(2,1),q(3,1),q(4,1)));	
					
	//Now the translation.
	trans = my - rot*mx;	
	
}


//Y is fixed and X is being transformed
void RegisterUsingICP(const LaserPoints& X, const LaserPoints& Y,
			Rotation3D& rot, Vector3D& trans,int maxIter,
			double percentageThreshold,bool useNormals,int kNN)
{

	vector<int> indices;
	vector<double> distances;
	KNNFinder<LaserPoint> finder;
	LaserPoints normalsY, normals;
	
	LaserPoints selectedX, selectedY, YReordered, selectedXTransformed;
	
	 if(useNormals)
	 {
		normalsY = Y.Normals(kNN);
	 }
		
	for(int iter = 0; iter <maxIter;iter++)
	{
		Rotation3D deltaRot;
		Vector3D deltaTrans;
	
		if(percentageThreshold<100)
		{	
			//don't update the selected point in each iteration.
			if(selectedX.empty() || selectedY.empty())
			{
				LaserPoints transformedX = X.Transform(rot,trans);
				
				finder.SetData(Y);
				finder.FindKnn(transformedX,1,distances,indices);
			
				vector<double> sorted = distances;
				sort(sorted.begin(),sorted.end());
				
				double maxDistance = sorted[(int)((sorted.size()-1)*percentageThreshold/100.00)];
			#if 0
				cerr<<"minDist "<<(*sorted.begin())<<endl;
				cerr<<"maxDist "<<(*(sorted.end()-1))<<endl;
				cerr<<"threshold "<<maxDistance<<endl;
			#endif
				normals.resize(0);
				for(int i=0;i<distances.size();i++)
				{
					if(distances[i]<maxDistance)
					{
						selectedX.push_back(X[i]);
						selectedY.push_back(Y[indices[i]]);
						
						if(useNormals)
							normals.push_back(normalsY[indices[i]]);
					}
				}
				finder.SetData(selectedY);
			}
			
			//cerr<<"selectedX size "<<selectedX.size()<<endl;
			//cerr<<"selectedY size "<<selectedY.size()<<endl;
			selectedXTransformed = selectedX.Transform(rot,trans);
			YReordered.resize(selectedXTransformed.size());
			
			if(useNormals)
			{
				int counter = 0;
				for(int i=0; i<selectedXTransformed.size();i++)
				{
					vector<int> ind;
					vector<double> dist;
					finder.FindKnn(selectedXTransformed[i],kNN,dist,ind);
					
					double minDistance = -1;
					for(int j=0;j<kNN;j++)
					{
						Vector3D n = normals[ind[j]];
						double distance = (n.DotProduct(selectedY[ind[j]])-n.DotProduct(Vector3D(selectedXTransformed[i])));
						if(fabs(distance)<minDistance || minDistance<0)
						{
							minDistance = fabs(distance);
							YReordered[i] = selectedXTransformed[i] + n*(distance);
						}
					}
					
					if(minDistance>dist[0])
					{
						YReordered[i] = selectedY[ind[0]];
						counter++;
					}
				}
				cerr<<"normals distance discarded for "<<counter<<" points out of "<<YReordered.size()<<endl;
			}
			else
			{

				finder.FindKnn(selectedXTransformed,1,distances,indices);

				YReordered.resize(indices.size());
			
				for(int i=0;i<indices.size();i++)
					YReordered[i] = (selectedY[indices[i]]);
			}
			
			FindTransformation(selectedXTransformed,YReordered,deltaRot,deltaTrans);
		}
		else
		{
			LaserPoints transformedX = X.Transform(rot,trans);

			//Find closest points.
			KNNFinder<LaserPoint> finder(transformedX);
			indices = finder.FindIndices(Y,1);

			FindTransformation(transformedX,Y,deltaRot,deltaTrans);
		}
		cerr<<"iter: "<<iter<<endl;
		Print(deltaRot,"deltaRot");
		rot = deltaRot*rot;
		trans = deltaTrans+trans;
	}
	
}	


///Register two point clouds using ICP. Y is fixed and X is being transformed
void RegisterUsingICP(const LaserPoints& X, const LaserPoints& Y,
			Orientation3D *orient,int maxIter,double percentageThreshold,bool useNormals,int kNN)
{
	Rotation3D rot(orient->rotation());
	Vector3D trans(*orient);
	
	RegisterUsingICP(X,Y,rot,trans,maxIter,percentageThreshold,useNormals,kNN);
	
	*orient = Orientation3D(trans,rot);	
}

///Find the transformation between two point clouds, assuming each point in X corresponds to the same point in Y.
void FindTransformation(const LaserPoints& X, const LaserPoints& Y,
			Orientation3D* orient)
{
	Rotation3D rot(*orient);
	Vector3D trans(*orient);
	
	FindTransformation(X,Y,rot,trans);
	
	*orient = Orientation3D(trans,rot);

}	

///Fit plane using eigen analysis of precomputed summation values.
Vector3D FindPlane(const double x2,const double y2,const double z2,
				const double xy, const double xz, const double yz,
				const double x, const double y, const double z,
				const int n, double &rho,double* pResidual )
{
	
	//subtract the mean
	double divisor = 1.00/(double)n;
	
	double mx = x*divisor;
	double my = y*divisor;
	double mz = z*divisor;
	
	//Allocate variance matrix.
	SymmetricMatrix M(3);
	M = 0;
	
	M(1,1) = x2 - 2*mx*x + n*mx*mx;
	M(1,2) = xy - mx*y -my*x +mx*my*n;
	M(1,3) = xz - mz*x - mx*z + mx*mz*n;
	M(2,2) = y2 - 2*my*y + n*my*my;
	M(2,3) = yz - mz*y - my*z + my*mz*n;
	M(3,3) = z2 - 2*mz*z + n*mz*mz;

	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	
	Jacobi(M,D,V);
	
	//The residual equal eigen value with minimum value.
	if(pResidual)
		*pResidual = D(1);
	
	//The eigenvector with minimum eigenvalues is the one we want.
	NEWMAT::Matrix q = V.Column(1);		
	Vector3D normal(q(1,1),q(2,1),q(3,1));
	
	Vector3D mean(mx,my,mz);
	rho = normal.DotProduct(mean);
	

	return normal;
}
	
		

///Fit cylinder using RANSAC.
int FitCylinderRANSAC(
			LaserPoints& laserPointsIn,
			IndicesVector& selectedIndices,
			Vector3D& in_point1,
			Vector3D& in_point2,
			double& in_dRadius,
			Vector3D& in_axis,
			Vector3D& in_pointOnAxis,
			double &in_dChiSquare, 
			bool doApproximation,
			int kNN,
			int nMaxIterations,
			double percentagePoints,
			double ransacIterations)
{
	const bool saveDebugOutput = 1;	
	
	LaserPoints laserPoints = laserPointsIn.Select(selectedIndices);
	for(int i=0;i<laserPointsIn.size();i++)
		laserPointsIn[i].Reflectance() = i*10;
	
	if(selectedIndices.empty())
		laserPoints = laserPointsIn;
		
	double minDistance = -1;
	for(int iter=0;iter<ransacIterations;iter++)
	{
		cerr<<"iter: "<<iter<<endl;
		IndicesVector ransacIndices;
		set<int> usedIndices;
		int count = 0;
		
		for(int i=0;i<laserPoints.size();i++)
		{
			int index = GenerateRandom(0,laserPoints.size()-1);
			
			if(!usedIndices.count(index))
			{
				usedIndices.insert(index);
				count++;
			}
			if(count>= (percentagePoints*0.01*laserPoints.size()))
				break;
		}
				
		ransacIndices.resize(usedIndices.size());
		copy(usedIndices.begin(),usedIndices.end(),ransacIndices.begin());
		
		
		//ransacIndices = SampleRandomly(laserPoints,percentagePoints*0.01*laserPoints.size());
		
		cerr<<"Selected points "<<ransacIndices.size()<<" from "<<laserPoints.size()<<endl;
		
		if(saveDebugOutput)
		{
			char buff[4096];
			sprintf(buff,"ransac_%d.pts",iter);
			laserPoints.Select(ransacIndices).SaveToAscii(buff);
		}
		
		Vector3D point1, point2;
		double dRadius;
		Vector3D axis, pointOnAxis;
		double dChiSquare;
		LaserPoints sel = laserPoints.Select(ransacIndices);
		LaserCylinder cyl = FitCylinder(sel,ApproximateCylinder(sel));
			
		vector<double> distances = cyl.Distance(laserPoints);
		
		sort(distances.begin(),distances.end());
		double currentDistance = 0;
		
		for(int k=0;k<(distances.size()/2);k++)
		{
			currentDistance += distances[k];
		}
		cerr<<"currentDistance: "<<currentDistance<<endl;
		if(currentDistance<minDistance || minDistance<0)
		{
			minDistance = currentDistance;
			in_point1 = point1;
			in_point2 = point2;
			in_dRadius = dRadius;
			in_axis = axis;
			in_pointOnAxis = pointOnAxis;
			in_dChiSquare = dChiSquare;
		}
		cerr<<"minDistance: "<<minDistance<<endl<<endl<<endl;
		cerr<<"axis: "<<in_axis;
	}
		
}
