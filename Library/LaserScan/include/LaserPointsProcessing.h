
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
*   Purpose   : Provides various functions to do processing on LaserPoints.
*
*--------------------------------------------------------------------*/
#ifndef _LASERPOINTS_PROCESSING_H_
#define _LASERPOINTS_PROCESSING_H_
#include "LaserPoints.h"
#include "LaserObjects.h"
#include "MyTypeDefinitions.h"
#include "BoxFilters.h"





/** 
Calculates normals for given laser points using specified distance neighbours.
The plane is fitted using Principal component analysis of covariance matrix of kNN.

	@param laserPoints laserPoints whose normals are to be calculated
    @param normalPoints (OUT) calculated normals
    @param distances (OUT) constant parameter in plane fitting D in Ax+By+C+D=0
    @param averageDistance (OUT) average distance of k-nearest neighbours.
    @param errors (OUT) residual for plane fitting.
	@param distanceThreshold - distance threshold to select neighbours.
    @param kNN-Min (IN) minimum number of k nearest neighbours to use
	@param kNN-Max (IN) maximum number of k nearest neighbours to use
    @param showProgress if true show progress of normal calculation, else use quiet mode
    
    @return void
*/
void
CalculateConstantDNormals(
				const LaserPoints& laserPoints,
				LaserPoints& normalPoints,
				double distanceThreshold = 0,
				int kNNMin = 5,
				int kNNMax = 200,
				vector<double>* pErrors = NULL,
				const bool showProgress = true);
				
				
/** Make LaserPoints consisting of random data.

	@param nSize count of generated points.
	@return generated laser points.
*/
LaserPoints 
MakeRandomPoints(int nSize);

/** Multiply given laser point with a matrix.
	can be used directly with the matrix read from OpenGL ModelView stack.
	
	@param laserPoint 
	@param pMatrix
	
	@return transformed laser point.
*/
LaserPoint & 
MultiplyLaserPointWithMatrix (double *pMatrix, LaserPoint & laserPoint);

/**	Generate data for an ideal sphere.

	@param r radius of sphere
	@param samples numbers of points in theta and phi.
	
	@return Generated laserPoints 
*/	
LaserPoints 
MakeSphere(double r,int samples,bool halfSphere=false);

/**	Generate data for an ideal sphere using cartesian sampling.
	@return Generated laserPoints 
*/	
LaserPoints 
MakeSphereXY(double r,int samples,bool halfSphere=false);


/** Generate data for an ideal torus

	@param dRMajor major radius
	@param dRMinor Minor radius
	@param nCirclesCount how many circles
	@params nPointsCount how many points in each circle.
	
	@return Generated laser points.
*/
LaserPoints 
MakeTorus(double dRMajor=10, double dRMinor=3, int nCirclesCount=100, int nPointsPerCircle=50);

/**	Generate data for an ideal cylinder.
Cylinder will be starting from orign and have user specified length and point density.

	@param radius radius of cylinder
	@param circleCount number of points along the circle.
	@param stackCount number of stacks along the circle's axis.
	@param zLength the length of cylinder.
	
	@return Generated laserPoints 
*/	

LaserPoints
MakeCylinder(double radius ,
			unsigned int circleCount, 
			unsigned int stackCount,
			double zLength);
			
LaserPoints 
MakePlane(int count=100,double x1=-1,double x2=1,double y1=-1,double y2=1,double xCoeff=1,double yCoeff=1);			
/** Calculate Rotataion3d that rotates v1 into v2.
*/

Rotation3D 
GetRotation(Vector3D v1, Vector3D v2);

/**	Generate data for an ideal cylinder.
Points p1 and p2 give starting and end points of the cylinder.

	@param radius radius of cylinder
	@param p1-p2 start and end point of the cylinder.
	@param circleCount number of samples along circular boundry.
	@param stackCount number of stacks along the circle's axis.
	
	@return Generated laserPoints 
*/	
LaserPoints 
MakeCylinder(
			double radius = 1,
			Vector3D p1 = Vector3D(0,0,0), 
			Vector3D p2 = Vector3D(0,0,10),
			int circleCount = 100, 
			int stackCount = 100);

/** Generate points uniformly distribute over sphere.

	@param count number of point in phi, theta steps are adaptively calculated.
	
	@return Generated laser points.
*/
LaserPoints
MakeUniformSphere(int count );

/** Generate data for ideal circle

	@param r radius of circle
	@param points, number of samples on circle.
	
	@return Generated laser points.
*/
LaserPoints 
MakeCirclePoints(double r, int points);

/** Cluster normals on uniform tesselation of gaussian sphere.
	
	@param normalPoints points to be culstered.
	@param clusteredPoints x,y,z values of cluster centres.
	@param clusterVotes votes for each centre.
	
	@return void
*/
void 
ClusterNormalsUniform(const LaserPoints& normalPoints,
				LaserPoints& clusteredPoints,
				vector<int>& clusteredVotes,
    			const int count,
				Vector3D& maxNormal,
				Vector3D& maxThetaPhiRho,
				double& maxValue,
				bool bMissZeros = true);
			

				

/** Helper class for estimation of TIN 3D. For more details looks at next function
*/
class TriangleIndices
{
public:
	int indices[3];
	TriangleIndices(){indices[0]=indices[1]=indices[2]=-1;};
	TriangleIndices(int a,int b,int c)
	{
		if(a>b && a>c)
		{
			if(b>c)
			{
				indices[0] = a; indices[1] = b; indices[2] = c;
			}
			else
			{
				indices[0] = a; indices[1] = c; indices[2] = b;
			}
		}
		else if(b>c && b>a)
		{
			indices[0] = b;
			if(c>a)
			{
				indices[1] = c; indices[2] = a;
			}
			else
			{
				indices[1] = a; indices[2] = c;
			}
		}
		else
		{
			indices[0] = c;
			if(b>a)
			{
				indices[1] = b; indices[2] = a;
			}
			else
			{
				indices[1] = a; indices[2] = b;
			}
		}
	};
	bool operator <(const TriangleIndices& b)const
	{
		return ((indices[0]*100.00 + indices[1]*10.00 + indices[2])>(b.indices[0]*100.00 + b.indices[1]*10.00 + b.indices[2]));
	}
	bool operator ==(const TriangleIndices& b)const
	{
		if(b.indices[0]==indices[0] && b.indices[1]==indices[1] && b.indices[2]==indices[2])
			return true;
		else
			return false;
	}
	int operator[](const int index)const
	{
		return indices[index];
	}
};
typedef vector<TriangleIndices> TIN3D;

/** Approximates 3D TIN by 
	- estimating local planes.
	- projecting points to these planes,
	- fitting 2D TIN to these projections. 

	The integrated TIN is formed by joining these local TIN's
	
	@param laserPoints input laser data
	@param tin3D final tin out put
	@kNN for plane estimation.
	
	@return void.
*/	
int
ApproximateTIN3DANN(
				const LaserPoints& laserPoints,
				TIN3D& tin3D,
				const int kNN = 5);
				
				
/** Segment point cloud using normal estimation in kNN and by region growing on basis of 
	normal similarity. Should result in segmentation of smooth patches.
	TO DO: Add internal constants for sort-filter etc as options to the function
	
	@param laserPoints  input point cloud
	@param segmentedIndices   segmentation result
	@param kNN    number of neighbours used.
	
	@return void
	
*/	
void
SegmentUsingSmoothnessConstraint(
			const LaserPoints& laserPoints,
			SegmentsVector& segmentedIndices,				
			const int kNN=5,
			const bool useStrictNormalSimilarity=false);
			
void
SegmentUsingSmoothnessConstraintWithConstantDistance(
				const LaserPoints& laserPoints,
				SegmentsVector& segmentedIndices,
				double distanceThreshold = 0.01,			
				const int kNNMin=20,
				int kNNMax=200,				
				const bool useStrictNormalSimilarity=false);			

/** Converts polar to rectangular

*/
Vector3D
Polar2Rect(const double theta, const double phi);

/** Converts from rectangular to plar
*/
void
Rect2Polar(Vector3D pt, double& theta, double& phi);

				
				
LaserPoints
GetNormalIntersectionPoints(
			const LaserPoints& laserPoints,
			vector<Vector3D>& normals,
			double dataExtents,
			double distanceThreshold);
			
LaserPoints
GetNormalIntersectionPoints(
			const LaserPoints& laserPoints,
			vector<vector<int> > &segments,
			vector<Vector3D>& normals,
			double dataExtents,
			double distanceThreshold);
			
///Find cylinders using RANSAC and segmentation.
class LaserCylinder;
vector<LaserCylinder*> GrowCylinders(LaserPoints& laserPoints,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndices,
				const int maxIterations = 10,
				const int maxRansacIteration = 100);	
				
//Grow more than one planes if possible.
class LaserPlane;
vector<LaserPlane*> GrowPlanes(LaserPoints& laserPoints,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndices,
				const int maxIterations=-1);
				
//Grow both, and select the best model.
class LaserObject;
vector<LaserObject*> GrowCylindersAndPlanes(
				LaserPoints& laserPointsIn,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndicesIn,
				const int maxIterations=-1,
				const int maxRansacIteration=100);	
				
//Make normals consistent.
vector<Vector3D> MakeNormalsConsistent(const LaserPoints& laserPoints,vector<double>&rhos,int kNN);
					
//Analyze angular histogram to see if we have a good cylinder.				
bool AnalyzeAngularHistogram(const vector<Vector3D>& nv, Vector3D axis, const double percentage);
				

#endif //_LASERPOINTS_PROCESSING_H_


