
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
* 	Purpose   : Provides various functions to do processing on LaserPoints.
*
*--------------------------------------------------------------------*/
#include "LaserPoints.h"
#include "LaserPointsProcessing.h"
#include "LaserPointsFitting.h"
#include "GeneralUtility.h"
#include "ProgressDisplay.h"
#include "VisualizationUtility.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
#include "Rotation3D.h"
#include "Orientation3D.h"
#include "VrmlUtility.h"
#include "Vector3Df.h"
#include "KNNFinder.h"
#include "StdHeader.h"
#include "Histo1D.h"
#include "Histo3D.h"
#include "ShMacro.h"
#include <set>
#include <map>
#include <algorithm>
#include "LaserPoints.h"
#include "LaserSegments.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"
#include "StlUtilities.h"
#include "VisualizationUtility.h"
#include "SegmentFinder.h"
#include "Macros.h"


/** Calculates normals for given laser points using specified distance neighbours.
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
vector<Vector3D>
CalculateConstantDNormals(
				const LaserPoints& laserPoints,
				double distanceThreshold,
				int kNNMin,
				int kNNMax,
				vector<double>* pErrors,
				const bool showProgress)
{

	double residual;
	vector<Vector3D> normals;
	if(kNNMax<kNNMin*5 || kNNMin<5 || laserPoints.size()<2*kNNMin || laserPoints.empty())
	{
		cerr<<"Normal calculation failed\n";
		return normals;
	}
	
	//Resize all containers to proper size.
	normals.resize(laserPoints.size());
	
	//User might not be interested in the errors.
	if(pErrors)
		pErrors->resize(laserPoints.size());
	
	//We need KNNFinder for doing knn search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	//Show the progress if so requrested.
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
	vector<int> indices;
    for(int i=0;i<laserPoints.size();i++)
    {
		if(showProgress)
			progress.Step("Calculating normals FDN: ",i);
		
		int kNN;
		int lastKnn = kNN;
		for(kNN=kNNMin;kNN<kNNMax;kNN+=kNNMin)
		{
			indices = finder.FindIndices(laserPoints[i],kNN);
			lastKnn = kNN;
			if((laserPoints[i]-laserPoints[indices[kNN-1]]).Length()>distanceThreshold)
			{
				for(int k=(kNN);k>kNNMin;k--)
				{
					if((laserPoints[i]-laserPoints[indices[k-1]]).Length()<=distanceThreshold)
					{
						lastKnn = k;
						break;
					}
				}
				break;
			}
		}
		//Note that lastKnn will be valid one in all cases, while kNN will get 
		//incremented in the last iteration before it gets checked againts limit.
		kNN = lastKnn;
		indices.resize(kNN);
		
		normals[i] = laserPoints.Select(indices).Normal(&residual);
		
		if(pErrors)
			(*pErrors)[i] = residual;
	}
	progress.End("Calculating normals FDN");
	
	return normals;	
}	

/** Make LaserPoints consisting of random data.

	@param nSize count of generated points.
	@return generated laser points.
*/
LaserPoints 
MakeRandomPoints(int nSize)
{
	LaserPoints pts;
	pts.resize(nSize);
	
	for(int i=0;i<nSize;i++)
	{
		pts[i]=LaserPoint(GenerateRandom(),GenerateRandom(),GenerateRandom());
	}
	return pts;
}

/** Multiply given laser point with a matrix.
	can be used directly with the matrix read from OpenGL ModelView stack.
	
	@param laserPoint 
	@param pMatrix
	
	@return transformed laser point.
*/
LaserPoint & 
MultiplyLaserPointWithMatrix (double *pMatrix, LaserPoint & laserPoint)
{
	double dX, dY, dZ;

	dX = laserPoint.X ();
	dY = laserPoint.Y ();
	dZ = laserPoint.Z ();

	laserPoint.X () =
		pMatrix[0] * dX + pMatrix[4] * dY + pMatrix[8] * dZ + pMatrix[12];
	laserPoint.Y () =
		pMatrix[1] * dX + pMatrix[5] * dY + pMatrix[9] * dZ + pMatrix[13];
	laserPoint.Z () =
		pMatrix[2] * dX + pMatrix[6] * dY + pMatrix[10] * dZ + pMatrix[14];

	return laserPoint;
}


/**	Generate data for an ideal sphere.

	@param r radius of sphere
	@param samples numbers of points in theta and phi.
	
	@return Generated laserPoints 
*/	
LaserPoints 
MakeSphere(double r,int samples,bool halfSphere)

{
	double theta = 0; // theta the angle from the z axis (0 <= theta <= pi)
	double phi = 0; //  phi the angle from the x axis (0 <= phi <= 2pi)
	LaserPoints laserPoints;
	laserPoints.reserve(samples*samples);
	
	int samplesTheta = halfSphere?(samples/4):(samples/2);
	int samplesPhi = samples;
	double factor = 2*M_PI/(double)samples;
	
	for(int i=0;i<samplesTheta;i++)
	{
		for(int j=0;j<samplesPhi;j++)
		{
			theta = i*factor;
			phi = j*factor;
			laserPoints.push_back(LaserPoint(r*sin(theta)*cos(phi),r*sin(theta)*sin(phi),r*cos(theta)));
		}
	}
	return laserPoints;
}	

/**	Generate data for an ideal sphere using cartesian sampling.
	@return Generated laserPoints 
*/	
LaserPoints 
MakeSphereXY(double r,int samples,bool halfSphere)
{
	LaserPoints laserPoints;
	laserPoints.reserve(samples*samples);
	
	double factor = 2.00/(double)samples;
	
	for(int i=0;i<=samples;i++)
	{
		double x = i*factor-1;
		for(int j=0;j<=samples;j++)
		{
			double y = j*factor-1;
			double z = 1 - x*x - y*y;
			
			if(z>=0)
			{
				z = sqrt((double)z);
				laserPoints.push_back(LaserPoint(r*x,r*y,r*z,r*z*10));
				
				if(!halfSphere)
				{
					z = -z;
					laserPoints.push_back(LaserPoint(r*x,r*y,r*z,r*z*10));
				}
			}
		}
	}
	return laserPoints;
}	

/**	Generate data for an ideal cylinder.
Cylinder will be starting from orign and have user specified length and point density.

	@param radius radius of cylinder
	@param circleCount number of points along the circle.
	@param stackCount number of stacks along the circle's axis.
	@param zLength the length of cylinder.
	
	@return Generated laserPoints 
*/	

LaserPoints
MakeCylinder(double radius,
			unsigned int circleCount, 
			unsigned int stackCount,
			double zLength)
{
    //Verify steps.
    BOUND_MIN(circleCount,2);
    BOUND_MIN(stackCount,1);
    BOUND_NONZERO(zLength,1);
    BOUND_NONZERO(radius,1);
    
    //Calculate factors.
    double stepCircle = M_PI*2.00/(double)circleCount;
    double stepZ = (zLength)/(double)stackCount;
    
    LaserPoints points;
    points.reserve(circleCount*stackCount);
    
    for(int i=0;i<circleCount;i++)
    {
    	for(int j=0;j<stackCount;j++)
		{
			points.push_back(LaserPoint(radius*sin(i*stepCircle),radius*cos(i*stepCircle),stepZ*j,stepZ*j));
		}
	}
	
	return points;
}	

LaserPoints 
MakePlane(int count,double x1,double x2,
	double y1,double y2,double xCoeff,double yCoeff)
{
	LaserPoints pts;
	for(double x=x1;x<=x2;x+=((x2-x1)/(double)count))
		for(double y=y1;y<=y2;y+=((y2-y1)/(double)count))
			pts.push_back(LaserPoint(x,y,xCoeff*x+yCoeff*y,(xCoeff*x+yCoeff*y)*100));
	return pts;
}				
		

/** Calculate Rotataion3d that rotates v1 into v2.
*/

Rotation3D 
GetRotation(Vector3D v1, Vector3D v2)
{
	double angle = Angle(v1,v2);
	
	if(fabs(angle)<1e-5)
		return Rotation3D();
	
	Vector3D axis = v1.VectorProduct(v2);
	axis = axis.Normalize();
	AngleAxisRotation ar(axis,angle);
	Rotation3D rot = ar.to_matrix();
	return rot;
}

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
			double radius,
			Vector3D p1, 
			Vector3D p2,
			int circleCount, 
			int stackCount)
{

	//Generate points along z-axis with appropriate length.
	//We will need to shift and rotate after wards.
	LaserPoints points = MakeCylinder(radius,circleCount,stackCount,(p1-p2).Length());
	
	//Calculate necessary rotation and translation.
	Rotation3D rot = GetRotation(Vector3D(0,0,1),p2-p1);
	Orientation3D orient(p1,rot);
	
	//Apply the rotation and translation to laser points.
	for(int i=0;i<points.size();i++)
	{
		Vector3D v = points[i];
		v = orient*v;
		points[i] = LaserPoint(v.X(),v.Y(),v.Z(),points[i].Reflectance());
	}
	
	return points;			
}	


/** Generate data for an ideal torus

	@param dRMajor major radius
	@param dRMinor Minor radius
	@param nCirclesCount how many circles
	@params nPointsCount how many points in each circle.
	
	@return Generated laser points.
*/
LaserPoints 
MakeTorus(double dRMajor, double dRMinor, int nCirclesCount, int nPointsPerCircle)
{
    double dUStep = 2*M_PI/(double)nCirclesCount;
    double dVStep = 2*M_PI/(double)nPointsPerCircle;
    int i,j;
    double dX, dY , dZ, dU, dV;
    LaserPoints pts;
    LaserPoint temp;
    pts.reserve(nCirclesCount*nPointsPerCircle);
    
    for(i=0;i<nCirclesCount;i++)
    {
        dU = i*dUStep;
        for(j=0;j<nPointsPerCircle;j++)
        {
                dV = j*dVStep;			
                dX = (dRMajor+dRMinor*cos(dV))*cos(dU);
                dY = (dRMajor+dRMinor*cos(dV))*sin(dU);
                dZ = dRMinor*sin(dV);
				temp = LaserPoint(dX,dY,dZ);
				temp.Reflectance() = dZ/dRMinor*200;
				pts.push_back(temp);
        }
    }
    return pts;
}

/** Generate points uniformly distribute over sphere.

	@param count number of point in phi, theta steps are adaptively calculated.
	
	@return Generated laser points.
*/
LaserPoints
MakeUniformSphere(int count )
{
	LaserPoints pts;
	int thetaCount = count;
	int phiCount = count;
	pts.reserve(thetaCount*phiCount);

	double thetaFactor = 2*M_PI/(double)thetaCount;
	double phiFactor = 0.5*M_PI/(double)(phiCount-1);
	double adjThetaFactor;
	double phi,theta;
	
	int i,j,k;
	for(j=0;j<phiCount;j++)
	{
		phi = j*phiFactor;
		if(phi==0)
		{
			pts.push_back(LaserPoint(0,0,1));
		}
				
		else if(0)
		{			
			int adjThetaCount = thetaCount*sin(phi);
			adjThetaFactor = 2*M_PI/(double)(adjThetaCount);;
			
			for(i=0;i<adjThetaCount;i++)
			{
				theta = i*adjThetaFactor;
				pts.push_back(LaserPoint(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi),cos(phi)*100));
			}
		}
		
		else if(10)
		{			
			int adjThetaCount = 2*M_PI*sin(phi)/(phiFactor);
			adjThetaFactor = 2*M_PI/(double)(adjThetaCount);;
			
			for(i=0;i<adjThetaCount;i++)
			{
				theta = i*adjThetaFactor;
				pts.push_back(LaserPoint(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi),cos(phi)*100));
			}
		}

	}
	return pts;
}

/** Generate data for ideal circle

	@param r radius of circle
	@param points, number of samples on circle.
	
	@return Generated laser points.
*/
LaserPoints MakeCirclePoints(double r, int points)
{
	LaserPoints pts;
	pts.reserve(10*10);
	double thetaFactor = 2*M_PI/(double)points;
	for(int i=0;i<points;i++)
	{
		pts.push_back(LaserPoint(r*cos(thetaFactor*i),r*sin(thetaFactor*i),0,0));
	}
	return pts;
}


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
				LaserPoints& laserPoints,
				TIN3D& tin3D,
				const int kNN)
{
	KNNFinder<LaserPoint> finder(laserPoints);
	ProgressDisplay<int> &progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
    int i,j,k,l;
  	double chiSquare;
	    
	//Allocate vectors.
	LaserPoints projectedPoints;
	projectedPoints.resize(kNN);
	tin3D.clear();
	tin3D.resize(laserPoints.size());
	
	set<TriangleIndices> triangleSet;
	vector<double> residuals(laserPoints.size());
	vector<Vector3D> normals(laserPoints.size());
	
	Vector3D u,v,w;
	
	//In the first pass just calculate the residuals.
	normals = laserPoints.Normals(kNN,true,&residuals);
	
	//Apply a sort filter to residuals. Just calculate the threshold in this step.
	//Apply threshold while processing.
	vector<double> sorted;
	sorted.resize(laserPoints.size());
	copy(residuals.begin(),residuals.end(),sorted.begin());
	sort(sorted.begin(),sorted.end());
	
	double threshold = sorted[(int)(0.9*laserPoints.size())];

    for(i=0;i<laserPoints.size();i++)
    {
		progress.Step("Generating TIN3D",i);
		//If current residual exceeds the threshold skip the point.
		if(residuals[i]>threshold)
			continue;
		
		vector<int> indices = finder.FindIndices(laserPoints[i],kNN);
		
		//We have the plane equation , Project points to it.
		w = normals[i].Normalize();
		w.PerpendicularVectors(u,v);
	
		for(j=0;j<kNN;j++)
		{
			int index = indices[j];
			projectedPoints[j].X() = u.DotProduct(laserPoints[index]);
			projectedPoints[j].Y() = v.DotProduct(laserPoints[index]);
			projectedPoints[j].Z() = w.DotProduct(laserPoints[index]);
		}

		//Generate TIN for projected Points.
		projectedPoints.DeriveTIN();
		TIN& tin = projectedPoints.TINReference();
		
		//Populate TIN3D.
		for(j=0;j<tin.size();j++)
		{
			TINMesh currentMesh = tin[j];
			PointNumber *points = tin[j].Nodes ();
					
			triangleSet.insert(TriangleIndices(indices[points[0].Number()],indices[points[1].Number()],indices[points[2].Number()]));
		}			
	}
	
	
	tin3D.resize(triangleSet.size());
	copy(triangleSet.begin(),triangleSet.end(),tin3D.begin());
	
	progress.End("Generating TIN3D");
}	



/** Segment point cloud using normal estimation in kNN and by region growing on basis of 
	normal similarity. Should result in segmentation of smooth patches.
	TO DO: Add internal constants for sort-filter etc as options to the function
	
	@param laserPoints  input point cloud
	@param segmentedIndices   segmentation result
	@param kNN    number of neighbours used.
	
	@return void
	
*/	

const double normalSimilarityThreshold = 0.97;
const double strictNormalSimilarityThreshold = 0.96;
//Some helper functions for segmentation.
static bool 
IsNormalSimilar(const Vector3D& n1,const Vector3D& n2,const double threshold=normalSimilarityThreshold)
{
	
	if(fabs(n1.X()*n2.X()+n1.Y()*n2.Y()+n1.Z()*n2.Z())>threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static bool 
IsNormalSimilar(const Vector3Df &n1,const Vector3Df& n2,const double threshold=normalSimilarityThreshold)
{
	
	if(fabs(n1.X()*n2.X()+n1.Y()*n2.Y()+n1.Z()*n2.Z())>threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static bool 
IsNormalSimilarStrict(const Vector3D& n1,
				const Vector3D& avg,
				const double threshold=strictNormalSimilarityThreshold)
{
	
	Vector3D n2 = avg.Normalize();
	
	if(fabs(n1.X()*n2.X()+n1.Y()*n2.Y()+n1.Z()*n2.Z())>threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static bool 
IsNormalSimilarStrict(const Vector3Df& n1,
				const Vector3Df& avg,
				const double threshold=strictNormalSimilarityThreshold)
{
	
	Vector3Df n2 = avg.Normalize();
	
	if(fabs(n1.X()*n2.X()+n1.Y()*n2.Y()+n1.Z()*n2.Z())>threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}


static bool 
CompareIndicesVector(const vector<int> &a, const vector<int> &b) 
{
    return a.size() > b.size();
}

template <class T>
struct ValuePlusIndexSort
{
     ValuePlusIndexSort(std::vector<T>* pRes=NULL)
	 :pResiduals(pRes)
	 {
	 }
	 bool operator()(int a, int b)
     {
          return ((*pResiduals)[a])< ((*pResiduals)[b]);
     }
private:
	std::vector<T>* pResiduals;
};

static bool
UpdateAvg(Vector3Df& avg, const Vector3Df normal)
{
	//just return don't use any averaging, just use the normal of the selected point.
	return true;
}

static bool
UpdateAvg(Vector3D& avg, const Vector3D normal)
{
	//just return don't use any averaging, just use the normal of the selected point.
	return true;
/*	
	#define POLARITY_SENSITIVE 0	//if true check for plarity reversal of normals too.
	if(	POLARITY_SENSITIVE)
	{
		double dot = avg.DotProduct(normal);
		if(fabs(dot)>0.8 || avg.Length()<0.1)
		{
			if(dot>0)
				avg = avg + normal;
			else
				avg = avg + (normal*-1);
			avg = avg.Normalize();
		}
		else
			cerr<<"Failed to update avg normal with dot "<<dot<<"\n";
	}
	else
	{
		avg = avg + normal;
		avg = avg.Normalize();
	}	
*/	
}

template <class T>
static bool 
ErasePoint(int index,set<int>& unusedSet,vector<T>& residuals,double maxRes)
{
	unusedSet.erase(index);
	residuals[index] = maxRes;
	// (not necessary as for update we select the residuals of unused points) residuals[index] = maxRes;
	return true;
}

template <class T>
static bool 
ErasePoint(int index,vector<bool>& unusedVector,int& unusedCount,vector<T>& residuals,double maxRes)
{
	unusedVector[index] = false;
	residuals[index] = maxRes;
	unusedCount--;
	// (not necessary as for update we select the residuals of unused points) residuals[index] = maxRes;
	return true;
}

template <class T,class U>
static bool 
ErasePoint(int index,vector<bool>& unusedVector,int& unusedCount,vector<T>& residuals,double maxRes, U& histo)
{
	histo.Remove(residuals[index]);
	unusedVector[index] = false;
	residuals[index] = maxRes;
	unusedCount--;
	histo.Remove(residuals[index]);
	// (not necessary as for update we select the residuals of unused points) residuals[index] = maxRes;
	return true;
}

#define SHOW_PROGRESS(current,total,frequency,msg) \
	if((current)%(MAX(total/frequency,1))==0) \
	{ \
		fprintf(stderr,"\r%s: %4.1f%%",msg,100.00*current/(double)total);\
	};
	
#define SHOW_PROGRESS_B() \
	{ showProgressCounter++;\
		if(showProgressCounter%MAX(pointCount/reportCount,1)==0){fprintf(stderr,"\r%s: %4.1f%%  region count: %d   ","Growing Regions",100.00*(pointCount - unusedPoints.size()+currentRegion.size())/(double)pointCount,segmentedIndices.size());}}
		
#define SHOW_PROGRESS_C() \
	{ showProgressCounter++;\
		if(showProgressCounter%MAX(pointCount/reportCount,1)==0){fprintf(stderr,"\r%s: %4.1f%%  region count: %d   ","Growing Regions",100.00*(pointCount - unUsedPointCount)/(double)pointCount,segmentedIndices.size());}}
		

void
SegmentUsingKConnectivity(
				const LaserPoints& laserPoints,
				SegmentsVector& segmentedIndices,				
				const int kNN)
{
	
	const int kNNFactor = 1;
	const int kNNRegionGrowing = kNNFactor*kNN; 
	const int maxRegionSize = 2*kNNRegionGrowing;
	ANNpointArray	dataPts;		// data points
    ANNpoint		queryPt;		// query point
    ANNidxArray		nnIndices;		// near neighbor indices
    ANNdistArray	dists;			// near neighbor distances
    ANNkd_tree		*pANNTree;		// search structure
    int i,j,k,l;
        
    int dim = 3;
    double eps = 0;
    int pointCount = laserPoints.size();
        
    queryPt = annAllocPt(dim);			// allocate query point
    dataPts = annAllocPts(pointCount, dim);		// allocate data points
    nnIndices = new ANNidx[kNNRegionGrowing];			// allocate near neigh indices
    dists = new ANNdist[kNNRegionGrowing];			// allocate near neighbor dists
 
	//Copy laser Points from the vector to ANN data structures.
	for(i=0;i<pointCount;i++)
	{
		dataPts[i][0]=laserPoints[i].X();
		dataPts[i][1]=laserPoints[i].Y();
		dataPts[i][2]=laserPoints[i].Z();
	}
	
	//Get the ANN tree.
	pANNTree = new ANNkd_tree(			// build search structure
		    		dataPts,			// the data points
		    		pointCount,			// number of points
		    		dim);			// dimension of space

    
    //Iterate over all laserPoints, finding normal for every point.
    int report = pointCount/25;
   	
	//Allocate temporary sets.
	typedef IndicesSet::iterator IndicesSetIterator;
    IndicesSet currentRegion;
    IndicesSet unusedSet;
    vector<int> potentialSeedPoints,unusedPoints;
    IndicesSet resultPoints;
    int currentIndex = 0;
    int counter = 0;
    
    unusedPoints.reserve(pointCount);
    for(i=0;i<pointCount;i++)
    {
    	unusedSet.insert(i);
		unusedPoints.push_back(i);
	}
		
	//Start Region Growing.
	while(1)
	{
		if(unusedPoints.empty()|| unusedPoints.size()<kNNRegionGrowing ||currentIndex==unusedPoints.size())			
			break;	
	
		currentIndex = unusedPoints[currentIndex];
		
		//clear scratch pads.
		currentRegion.clear();
		potentialSeedPoints.clear();
				
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		currentRegion.insert(currentIndex);
		unusedSet.erase(currentIndex);		
		
		for(i=0;i<potentialSeedPoints.size();i++)
		{
			currentIndex = potentialSeedPoints[i];
		
			//Find k-nearest neighbours.
			queryPt[0]=laserPoints[currentIndex].X();
			queryPt[1]=laserPoints[currentIndex].Y();
			queryPt[2]=laserPoints[currentIndex].Z();

			pANNTree->annkSearch(queryPt,kNNRegionGrowing,nnIndices,dists,eps/kNNFactor);

			for(j=0;j<kNNRegionGrowing;j++)
			{
				bool isSimilar = 1;
				
				if(isSimilar)
				{
					if(unusedSet.count(nnIndices[j]))
					{
						currentRegion.insert(nnIndices[j]);
						unusedSet.erase(nnIndices[j]);	
						potentialSeedPoints.push_back(nnIndices[j]);
					}
				}
			}
			
			if(currentRegion.size()>maxRegionSize)
				break;					
		}
		//This new region is finished.
		//Add current Region to global list.
		vector<int> temp;
		temp.resize(currentRegion.size());
		copy(currentRegion.begin(),currentRegion.end(),temp.begin());
		temp.reserve(currentRegion.size());
		segmentedIndices.push_back(temp);
	
		DEBUG("SegmentUsingSmoothnessConstraint: Region %d with %d points finished",segmentedIndices.size(),temp.size());
		
		unusedPoints.resize(unusedSet.size());
		copy(unusedSet.begin(),unusedSet.end(),unusedPoints.begin());
		currentIndex = 0;
		
		fprintf(stderr,"\rGrowing Regions: %4.1f%%",100.00*(double)(pointCount - unusedPoints.size())/(double)pointCount);
	}
	//sort the resultant segments with respect to size.
	sort(segmentedIndices.begin(),segmentedIndices.end(),CompareIndicesVector);
	
	fprintf(stderr,"\rSegmentUsingKConnectivity: Finished !!!!!\n");
	
	//free resources.
	delete[] nnIndices;
	delete[] dists;
	delete(pANNTree);
	annDeallocPts(dataPts);
	annDeallocPt(queryPt);

}



double radian2degree(double a)
{
	return a*180.00/M_PI;
}

double degree2radian(double a)
{
	return a*M_PI/180.00;
}


struct DistanceBasedSorter
{
     DistanceBasedSorter(const LaserPoints* __pPts,Vector3D __centroid)
	 :pPts(__pPts),centroid(__centroid)
	 {
	 }
	 
	double distance(int i)
	{
		return ((*pPts)[i]-centroid).Length();
	}
	 bool operator()(int a, int b)
     {
          return (distance(a)< distance(b));
     }
private:
	const LaserPoints* pPts;
	Vector3D centroid;
};


vector<int>
GrowFromSeed(const LaserPoints& laserPoints,
			const LaserPoints& normals,
			int seedPointIndex,
			double maxAngleInDegrees,
			const int kNN,
			bool breakOnMaxAngle=true)
{

	
	//Make the object for doing KNN search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	double dotThreshold = cos(degree2radian(maxAngleInDegrees));
	//cerr<<"dotThreshold: "<<dotThreshold<<endl;
	bool bThresholdCrossed = false;
	
	//Scratch pads for region indices.
	set<int> region;
	set<int> usedSeeds;
	
	vector<int> currentSeeds;
	
	//Insert starting point into seed index.
	usedSeeds.insert(seedPointIndex);
	currentSeeds.push_back(seedPointIndex);
		
	Vector3D startNormal = normals[seedPointIndex];
	
	while((!bThresholdCrossed || !breakOnMaxAngle)  
			&& !currentSeeds.empty())
	{		
		#if 1
		if(region.size()>kNN)
		{
			//Lets get the next seed which is closest to the centroid.
			//This we will accompalish by sorting w.r.t distance.
			Vector3D centroid = laserPoints.Select(vector<int>(region.begin(),region.end())).Mean();
			
			startNormal = normals[finder.FindIndex(centroid)];
					
			//cerr<<"sort..."<<flush;
			sort(currentSeeds.begin(), currentSeeds.end(),DistanceBasedSorter(&laserPoints,centroid));
		}
		#endif
				
		vector<int> newSeeds;
		for(int j=0;j<currentSeeds.size();j++)
		{
			int currentSeed = currentSeeds[j];
			usedSeeds.insert(currentSeed);
			vector<int> neighbours = finder.FindIndices(laserPoints[currentSeed],kNN);
		
			for(int i=0;i<neighbours.size();i++)
			{
				if(!region.count(neighbours[i])) 
				{
					if(!breakOnMaxAngle ||
					fabs(normals[neighbours[i]].DotProduct(startNormal))>dotThreshold)
					{
						if(!usedSeeds.count(neighbours[i]))
							newSeeds.push_back(neighbours[i]);
						region.insert(neighbours[i]);
					}
					else
						bThresholdCrossed = true;
				}				
			}
		}
		currentSeeds = newSeeds;
	}
	vector<int> regionVector(region.size());
	copy(region.begin(),region.end(),regionVector.begin());
	
	return regionVector;
}


vector<vector<int> >
SplitIntoRegions(const LaserPoints& laserPoints,
			double maxAngleInDegrees,
			const int kNN,
			bool breakOnMaxAngle=true)
{

	
	//Make the object for doing KNN search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	double dotThreshold = cos(degree2radian(maxAngleInDegrees));
		
	vector<Vector3D> normals(laserPoints.size());
	vector<double> residuals(laserPoints.size());
	
	//Calculate normals for all points.
	for(int i=0;i<laserPoints.size();i++)
	{
		double r;
		normals[i] = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Normal(&r);
		residuals[i] = r;
	}

	set<int> usedPoints;
	vector<vector<int> > segments;
	
	while(1)
	{
		if(usedPoints.size()==laserPoints.size())
			break;
			
		cerr<<"\n*****************************************\n";
		cerr<<"Region "<<segments.size()
			<<" remaining points"<<(laserPoints.size()-usedPoints.size())
			<<endl;
		
		//Find next seed as the point with lowest residual.
		int seedPointIndex = min_element(residuals.begin(),residuals.end())-residuals.begin();
		
		//Scratch pads for region indices.
		set<int> region;
		set<int> usedSeeds;
		bool bThresholdCrossed = false;
		
		//We cannot trust ANN to find self-similarity always.
		//If a point is repeated more than kNN times, we may have an infinite loop
		//To avoid that add current start explicitly.
		region.insert(seedPointIndex);
		usedPoints.insert(seedPointIndex);
		residuals[seedPointIndex] = DBL_MAX;
	
		vector<int> currentSeeds;
	
		//Insert starting point into seed index.
		currentSeeds.push_back(seedPointIndex);
		
		Vector3D startNormal = normals[seedPointIndex];
		
	
		while((!bThresholdCrossed || !breakOnMaxAngle)  
				&& !currentSeeds.empty())
		{		
			#if 1
			if(region.size()>kNN && currentSeeds.size()>5)
			{
				//Lets get the next seed which is closest to the centroid.
				//This we will accompalish by sorting w.r.t distance.
				Vector3D centroid = laserPoints.Select(vector<int>(region.begin(),region.end())).Mean();
				startNormal = normals[finder.FindIndex(centroid)];

				sort(currentSeeds.begin(), currentSeeds.end(),DistanceBasedSorter(&laserPoints,centroid));
			}
			#endif
			vector<int> newSeeds;
			for(int j=0;j<currentSeeds.size();j++)
			{
				int currentSeed = currentSeeds[j];
				usedSeeds.insert(currentSeed);
				vector<int> neighbours = finder.FindIndices(laserPoints[currentSeed],kNN);

LaserPoints *pts = ((LaserPoints*)(&laserPoints));
				for(int i=0;i<neighbours.size();i++)
				{
					if(!region.count(neighbours[i]) && !usedPoints.count(neighbours[i])) 
					{
						if(!breakOnMaxAngle ||
						fabs(normals[neighbours[i]].DotProduct(startNormal))>dotThreshold)
						{
							region.insert(neighbours[i]);
(*pts)[neighbours[i]].Reflectance() = region.size()*10;
							usedPoints.insert(neighbours[i]);
							residuals[neighbours[i]] = DBL_MAX;
							if(!usedSeeds.count(neighbours[i]))
								newSeeds.push_back(neighbours[i]);
						}
						else
							bThresholdCrossed = true;
					}				
				}
			}
			currentSeeds = newSeeds;
		}
		
		vector<int> regionVector(region.size());
		copy(region.begin(),region.end(),regionVector.begin());
		cerr<<"Finished with "<<regionVector.size()<<endl;
		segments.push_back(regionVector);
	}
	
	return segments;
}




vector<int>
GrowFromSeed(const LaserPoints& laserPoints,
			int seedPointIndex,
			double maxAngleInDegrees,
			const int kNN,
			bool breakOnMaxAngle=true)
{

	/// We need to calculate normals ourselves.
	//Make the object for doing KNN search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	LaserPoints normals;
	normals.resize(laserPoints.size());
	//Calculate normals for all points.
	for(int i=0;i<laserPoints.size();i++)
		normals[i] = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Normal();
	
	//Now call the function that takes calculated normals.
	return GrowFromSeed(laserPoints,normals,seedPointIndex,maxAngleInDegrees,kNN,breakOnMaxAngle);
}



/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return FALSE if no solution exists.
*/
int LineLineIntersect(
   Vector3D p1,Vector3D p2,Vector3D p3,Vector3D p4,Vector3D &pa,Vector3D &pb,
   double &mua, double &mub)
{
   const double EPS = 1e-10;
   #define ABS(a) fabs(a)
   Vector3D p13,p43,p21;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom;

   p13.X() = p1.X() - p3.X();
   p13.Y() = p1.Y() - p3.Y();
   p13.Z() = p1.Z() - p3.Z();
   p43.X() = p4.X() - p3.X();
   p43.Y() = p4.Y() - p3.Y();
   p43.Z() = p4.Z() - p3.Z();
   if (ABS(p43.X())  < EPS && ABS(p43.Y())  < EPS && ABS(p43.Z())  < EPS)
      return(FALSE);
   p21.X() = p2.X() - p1.X();
   p21.Y() = p2.Y() - p1.Y();
   p21.Z() = p2.Z() - p1.Z();
   if (ABS(p21.X())  < EPS && ABS(p21.Y())  < EPS && ABS(p21.Z())  < EPS)
      return(FALSE);

   d1343 = p13.X() * p43.X() + p13.Y() * p43.Y() + p13.Z() * p43.Z();
   d4321 = p43.X() * p21.X() + p43.Y() * p21.Y() + p43.Z() * p21.Z();
   d1321 = p13.X() * p21.X() + p13.Y() * p21.Y() + p13.Z() * p21.Z();
   d4343 = p43.X() * p43.X() + p43.Y() * p43.Y() + p43.Z() * p43.Z();
   d2121 = p21.X() * p21.X() + p21.Y() * p21.Y() + p21.Z() * p21.Z();

   denom = d2121 * d4343 - d4321 * d4321;
   if (ABS(denom) < EPS)
      return(FALSE);
   numer = d1343 * d4321 - d1321 * d4343;

   mua = numer / denom;
   mub = (d1343 + d4321 * (mua)) / d4343;

   pa.X() = p1.X() + mua * p21.X();
   pa.Y() = p1.Y() + mua * p21.Y();
   pa.Z() = p1.Z() + mua * p21.Z();
   pb.X() = p3.X() + mub * p43.X();
   pb.Y() = p3.Y() + mub * p43.Y();
   pb.Z() = p3.Z() + mub * p43.Z();

   return(TRUE);
}

LaserPoints
GetNormalIntersectionPoints(
			const LaserPoints& laserPoints,
			vector<Vector3D>& normals,
			double dataExtents,
			double distanceThreshold)
{
	LaserPoints interPoints;
	
	for(int i=0;i<laserPoints.size();i++)
	{
		if((i%(laserPoints.size()/10))==0)
			fprintf(stderr,"\rGetNormalIntersectionPoints: %4.1f%%",100.00*(double)(i)/(double)laserPoints.size());
		Vector3D la1,la2,lb1,lb2,p1,p2;
		double ma,mb;
		la1 = normals[i]*(-dataExtents)+Vector3D(laserPoints[i]);
		la2 = normals[i]*(dataExtents)+Vector3D(laserPoints[i]);
		
		DataBoundsLaser bounds = ((LaserPoints&)laserPoints).DeriveDataBounds(0);
		double maxDim = max(bounds.XRange(),max(bounds.YRange(),bounds.ZRange())); 
		double maxDistance = 0.1*maxDim;
		distanceThreshold = .005*maxDim;
		
		
		for(int j=i;j<laserPoints.size();j++)
		{
			if((Vector3D(laserPoints[i])-Vector3D(laserPoints[j])).Length()<maxDistance
				 && fabs(normals[i].DotProduct(normals[j]))<0.95)
			{
				lb1 = normals[j]*(-dataExtents)+Vector3D(laserPoints[j]);
				lb2 = normals[j]*(dataExtents)+Vector3D(laserPoints[j]);
			
				if(LineLineIntersect(la1,la2,lb1,lb2,p1,p2,ma,mb))
				{
					if((p1-p2).Length()<distanceThreshold)
					{
						Vector3D mid = (p1+p2)*0.5;
						interPoints.push_back(LaserPoint(mid.X(),mid.Y(),mid.Z(),100));
					}
				}
			}
		}			
	}
	fprintf(stderr,"\rGetNormalIntersectionPoints: %4.1f%%\n",100.00);
	//return with results.
	return interPoints;
}

//This one uses a segmented point cloud.
LaserPoints
GetNormalIntersectionPoints(
			const LaserPoints& laserPoints,
			vector<vector<int> > &segments,
			vector<Vector3D>& normals,
			double dataExtents,
			double distanceThreshold)
{
	LaserPoints interPoints;
	
	DataBoundsLaser bounds = ((LaserPoints&)laserPoints).DeriveDataBounds(0);
	double maxDim = max(bounds.XRange(),max(bounds.YRange(),bounds.ZRange())); 
	double maxDistance = 0.1*maxDim;
	distanceThreshold = .005*maxDim;

	
	for(int s=0;s<segments.size();s++)
	{
		if((s%(segments.size()/10))==0)
			fprintf(stderr,"\rGetNormalIntersectionPoints: %4.1f%%",100.00*(double)(s)/(double)segments.size());
		for(int t=s;t<segments.size();t++)
		{
			vector<int>& seg1 = segments[s];
			vector<int>& seg2 = segments[t];
			for(int i=0;i<seg1.size();i++)
			{
	
				Vector3D la1,la2,lb1,lb2,p1,p2;
				double ma,mb;
				la1 = normals[seg1[i]]*(-dataExtents)+Vector3D(laserPoints[seg1[i]]);
				la2 = normals[seg1[i]]*(dataExtents)+Vector3D(laserPoints[seg1[i]]);
		
				for(int j=0;j<seg2.size();j++)
				{
					if((Vector3D(laserPoints[seg1[i]])-Vector3D(laserPoints[seg2[j]])).Length()<maxDistance
						&& fabs(normals[seg1[i]].DotProduct(normals[seg2[j]]))<0.9)
					{
						lb1 = normals[seg2[j]]*(-dataExtents)+Vector3D(laserPoints[seg2[j]]);
						lb2 = normals[seg2[j]]*(dataExtents)+Vector3D(laserPoints[seg2[j]]);
			
						if(LineLineIntersect(la1,la2,lb1,lb2,p1,p2,ma,mb))
						{
							if((p1-p2).Length()<distanceThreshold)
							{
								Vector3D mid = (p1+p2)*0.5;
								interPoints.push_back(LaserPoint(mid.X(),mid.Y(),mid.Z(),100));
							}
						}
					}
				}
			}
		}			
	}
	fprintf(stderr,"\rGetNormalIntersectionPoints: %4.1f%%\n",100.00);
	//return with results.
	return interPoints;
}


/** Filters LaserPoints using kNN. Finds k-nearest neighbours and replaces by
	their mean
	
	@param laserPoints the point cloud to be processed.
	@param kNN number of k nearest neighbours
	@param allAxes if false, only reflectance is processed else x,y,z,pulseCount are also filtered.
	
	@return the filtered data.
*/	
	
LaserPoints
FilterKNNLaserPoints(
				const LaserPoints& laserPoints,
				const int kNN,
				bool allAxes)
{

	ANNpointArray	dataPts;		// data points
    ANNpoint		queryPt;		// query point
    ANNidxArray		nnIndices;		// near neighbor indices
    ANNdistArray	dists;			// near neighbor distances
    ANNkd_tree		*pANNTree;		// search structure
    int i,j,k;

    int dim = 3;
    double eps = 0;
    int pointCount = laserPoints.size();
    queryPt = annAllocPt(dim);			// allocate query point
    dataPts = annAllocPts(pointCount, dim);		// allocate data points
    nnIndices = new ANNidx[kNN];			// allocate near neigh indices
    dists = new ANNdist[kNN];			// allocate near neighbor dists
 
	//Copy laser Points from the vector to our ANN data structures.
	for(i=0;i<pointCount;i++)
	{
		dataPts[i][0]=laserPoints[i].X();
		dataPts[i][1]=laserPoints[i].Y();
		dataPts[i][2]=laserPoints[i].Z();
	}
	
	//Get the ANN tree.
	pANNTree = new ANNkd_tree(			// build search structure
		    		dataPts,			// the data points
		    		pointCount,			// number of points
		    		dim);			// dimension of space

    
    LaserPoints resultPoints = laserPoints;
    for(i=0;i<pointCount;i++)
    {
    	queryPt[0]=dataPts[i][0];
		queryPt[1]=dataPts[i][1];
		queryPt[2]=dataPts[i][2];
	
		pANNTree->annkSearch(queryPt,kNN,nnIndices,dists,eps);
		
		double factor = 1.00/dists[kNN-1];
		double weight ;
		double x=0,y=0,z=0,r=0,p=0;
		double w = 0;
		for(j=0;j<kNN;j++)
		{
			weight = 1.00/(0.001+dists[j]*factor);
			w += weight;
			r += laserPoints[nnIndices[j]].Reflectance()*weight;
			if(allAxes)
			{
				
				x += laserPoints[nnIndices[j]].X()*weight;
				y += laserPoints[nnIndices[j]].Y()*weight;
				z += laserPoints[nnIndices[j]].Z()*weight;
				p += laserPoints[nnIndices[j]].PulseCount()*weight;
			}
		}
		if(w<=0)
			w = 1;
		if(allAxes)
		{
			resultPoints[i]=LaserPoint(x/w,y/w,z/w,r/w,p/w);
		}
		else
			resultPoints[i].Reflectance() = r/w;
	}
	
	//free resources.
	delete(pANNTree);
	delete[] nnIndices;
	delete[] dists;
	annDeallocPts(dataPts);
	annDeallocPt(queryPt);
	
	//return with the result.
	return resultPoints;
}				

																																																																																																																																
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
				bool bMissZeros)
{
	//For reverse mapping from x,y,z coordinate of normals to (theta,phi) coordinates of
	//unifrom sphere, first calculate the phi value, use to get theta Factor needed for unifrom
	//tesselation and use this thetaFactore to get thetaIndex. The number of thetaSamples will vary
	//with sin(phi).
	
	int phiCount = count;
	int maxThetaCount = count*4;
	double phiFactor = 0.5*M_PI/(double)(phiCount-1);
	double *thetaFactors = new double[phiCount];
	int *thetaCounts = new int[phiCount];
	double phi,theta;
	int i,j,k;

	//Make look-up tables for reverse mapping.
	for(j=0;j<phiCount;j++)
	{
		phi = j*phiFactor;
		int adjThetaCount = 2*M_PI*sin(phi)/(phiFactor);
		if(adjThetaCount==0)
		{
			//Set a very large Factor, so that every value is mapped to zero.
			thetaFactors[j] = 1e30;
			thetaCounts[j] = 1;
		}
		else
		{			
			
			thetaFactors[j] = 2*M_PI/(double)(adjThetaCount);
			thetaCounts[j] = adjThetaCount;
		}
	}
	
	//Allocate space for accumulator.
	//The 2-D array allocated for the cluster-accumulator will be (phiCount*maxThetaCount)
	#define TYPE int
	TYPE* pTableMemory = new TYPE[phiCount*maxThetaCount];
	memset(pTableMemory,0,sizeof(TYPE)*phiCount*maxThetaCount);
	TYPE** ppTable = new TYPE*[count];
	for(i=0;i<phiCount;i++)
		ppTable[i] = pTableMemory+i*maxThetaCount;
		
	//Populate cluster table.
	int phiIndex,thetaIndex;
	for(i=0;i<normalPoints.size();i++)
	{
		LaserPoint pt = normalPoints[i];
		phiIndex = (acos(pt.Z()))/phiFactor;
		thetaIndex = (atan2(pt.Y(),pt.X())+M_PI)/thetaFactors[phiIndex];
		ppTable[phiIndex][thetaIndex]++;
	}
	#undef TYPE
	
	//convert the cluster table to laser Points.
	clusteredPoints.reserve(count*count);
	clusteredVotes.reserve(count*count);
	double x,y,z;
	int max = -1e10;
	LaserPoint maxPoint;
	for(i=0;i<phiCount;i++)
	{
		z = cos(i*phiFactor);
		
		for(j=0;j<thetaCounts[i];j++)
		{
			if(bMissZeros && ppTable[i][j]<1)
				continue;
			theta = thetaFactors[i]*j - M_PI; 
			x = cos(theta)*sin(i*phiFactor);
			y = sin(theta)*sin(i*phiFactor);
			clusteredPoints.push_back(LaserPoint(x,y,z,ppTable[i][j]));
			clusteredVotes.push_back(ppTable[i][j]);
			
			if(ppTable[i][j]>max)
			{
				max = ppTable[i][j];
				maxPoint = LaserPoint(2*x,2*y,2*z,ppTable[i][j]);
				maxNormal = Vector3D(x,y,z);
				maxThetaPhiRho = Vector3D(theta,i*phiFactor,1);
			}
		}
	}
	maxValue = max;
	
#if 0
	//For Debugging only.
	LaserPoints debugPoints = clusteredPoints;	
	debugPoints.push_back(maxPoint);
	ShowLaserPoints(debugPoints,"Normals Clustered on Unifrom sphere");
#endif	
	
	//Free allocated resources.
	delete[] pTableMemory;
	delete[] ppTable;
	delete[] thetaCounts;
	delete[] thetaFactors;		
}																					


/** Find nearest neighbours on uniform sphere without using kNN
Gets an approximation by getting points around given start point.
*/
LaserPoints
GetKnnOnUniformSphere(double theta0, double phi0,int phiCount,int kNN )
{
	LaserPoints pts;

	double phiFactor = 0.5*M_PI/(double)(phiCount-1);
	double phi,theta;
	
	int thetaNN = (sqrt((double)kNN) + 1);
		
	for(int j=0;j<=(2*phiCount);j++)
	{
		//Alternately take one step along phi, in positive or negative direction.
		if(j%2)
			phi = phi0-((int)((j+1)/2))*phiFactor;
		else
			phi = phi0+((int)(j/2))*phiFactor;
				
		//If we cross the valid bound just, skip this iteration.
		if(phi<0 || phi>M_PI*0.5)
			continue;
			
		//If phi is zero, we have only one point.
		if(phi==0)
		{
			pts.push_back(LaserPoint(0,0,1));
		}				
		else
		{			
			int adjThetaCount = 2*M_PI*sin(phi)/(phiFactor);
			double adjThetaFactor = 2*M_PI/(double)(adjThetaCount);;
						
			if(adjThetaCount <= thetaNN)
			{
				//Push all theta points.
				for(int i=0;i<adjThetaCount;i++)
				{
					theta = i*adjThetaFactor;
					pts.push_back(LaserPoint(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi),cos(phi)*100));
				}
			}
			else
			{
				int thetasAdded = 0;
				for(int k=0;k<=adjThetaCount;k++)
				{
					if(k%2)
						theta = theta0 - ((int)((k+1)/2))*adjThetaFactor;
					else
						theta = theta0 + ((int)(k/2))*adjThetaFactor;
						
					pts.push_back(LaserPoint(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi),10));
					thetasAdded++;
					
					if(thetasAdded>=thetaNN)
						break;
				}
			}
		}
		if(pts.size()>kNN)
			break;					
	}

	//For debugging only.
	if(0)
	{
		LaserPoints p ;//= MakeUniformSphere(25);
		
		for(int i=0;i<p.size();i++)
			p[i].Reflectance() = 1;
		for(int i=0;i<pts.size();i++)
		{
			p.push_back(LaserPoint(pts[i].X(),pts[i].Y(),pts[i].Z(),25,25));
		}
		p.push_back(LaserPoint(cos(theta0)*sin(phi0),sin(theta0)*sin(phi0),cos(phi0),50,50));
		
		char fileName[] = "SelectionFullOnUnformSphere";
		void HalfSphere2Vrml(FILE* pFile);
		FILE* pFile = OpenVrmlFile(fileName);
		LaserPoints2Vrml(pFile,p,(GLColorScheme)1);
		HalfSphere2Vrml(pFile);
		CloseVrmlFile(pFile);
		ShowInVrmlview(fileName);
	}
	
	return pts;
}
LaserPoints
GetKnnOnUniformSphere(Vector3D v,int phiCount,int kNN )
{
	double theta0, phi0;
	Rect2Polar(v,theta0,phi0);
	return GetKnnOnUniformSphere(theta0,phi0,phiCount,kNN);
}

/** Converts polar to rectangular

*/
Vector3D
Polar2Rect(const double theta, const double phi)
{
	return Vector3D(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi));
}

/** Converts from rectangular to plar
*/
void
Rect2Polar(Vector3D pt, double& theta, double& phi)
{
	theta = atan2(pt.Y(),pt.X());
	phi = acos(pt.Z());
}

void
SegmentUsingSmoothnessConstraint(
				const LaserPoints& laserPoints,
				SegmentsVector& segmentedIndices,				
				const int kNN,
				const bool useStrictNormalSimilarity)
{
SH	
	const bool assignProgressToReflectance = true;
	const double eps = 1e-12;
	//LaserPoints debuggingPoints = laserPoints;
	int showProgressCounter = 0;
	
	//Check the size of points, if too small return.
	if(laserPoints.size()<2*kNN)
	{
		cerr<<"SegmentUsingSmoothnessConstraint: too few points \n";
		return;
	}
	
	//Set default values for different seeds.
	double seedFactor1 = 0.9950;
	double seedFactor2 = 0.9975;
	double mergeFactor = 0.9999;
	
	if(useStrictNormalSimilarity)
	{
		seedFactor1 = 1;
		seedFactor2 = 1;
		mergeFactor = 1;	
	}
	else
	{
		seedFactor1 = 0.50;
		seedFactor2 = 0.75;
		mergeFactor = 1;	
	}
	
	//We may search for next-level seeds in bigger neighbourhoods.
	//kNNFactor controls that.
	const int kNNFactor = 1;
	const int kNNRegionGrowing = kNNFactor*kNN;
	
	//Declare some local variables.
    int i,j,k,l, pointCount = laserPoints.size();

	KNNFinder<LaserPoint> finder(laserPoints);
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),200);
	double chiSquare;
	
	//In the first pass just calculate the normals and their residuals.
	vector<double> residuals(laserPoints.size());
	vector<Vector3D> normals(laserPoints.size());
	for(int i=0;i<laserPoints.size();i++)
	{
		cerr<<i<<endl;
		double temp;
		normals[i] = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Normal(&temp);
		residuals[i] = temp;
		progress.Step("Calculating normals",i);
	}
	progress.End();
	progress.Reset();

	//Get max Residual.
	double maxRes = Max(residuals);
	
	//Apply a sort filter to residuals. Just calculate the threshold in this step.
	//Apply threshold while processing.
	double thresholdSeed1;
	double thresholdSeed2;
	double thresholdMerge;
	
	//update threshold through histogramming.
	int lastThresholdUpdate = pointCount;

SH	//Making histogram for seed calculation and updating the seeds.
	Histo1D<int> histogram(residuals,Range(residuals)/(1024*20));
	thresholdSeed1 = histogram.PercentValue(seedFactor1*100);
	thresholdSeed2 = histogram.PercentValue(seedFactor2*100);
	thresholdMerge = histogram.PercentValue(mergeFactor*100);
SH 	
	//Allocate temporary sets.
	typedef IndicesSet::iterator IndicesSetIterator;
    IndicesSet currentRegion;
    vector<int> potentialSeedPoints;
    vector<bool> pointStateVector(pointCount,true);
    IndicesSet resultPoints;
    int currentIndex = 0;
    int unUsedPointCount = pointCount;
    int counter = 0;
	
	segmentedIndices.reserve(4096);
    
    int iterationCounter = 0;	
	fprintf(stderr,"\nStarting segmentation loop...\n");
	//Start Region Growing.
	while(1)
	{
		if(unUsedPointCount == 0)			
			break;	
			
		if(unUsedPointCount<kNNRegionGrowing && !useStrictNormalSimilarity)
			break;
	
		//Select the point with minimum residual as the next seed.
		double minResidual = +1e30;
		for(int k=0;k<pointCount;k++)
		{
			if(pointStateVector[k] && residuals[k]<minResidual)	
			{
				currentIndex = k;
				minResidual = residuals[k];
			}
		}
		
		//We have found the new main seed, so continue.
	
		
		DEBUG("SegmentUsingSmoothnessConstraint: Starting a new region with seed %d",currentIndex);
		
		//clear scratch pads.
		currentRegion.clear();
		potentialSeedPoints.clear();
		Vector3Df avg(0,0,0);
		
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		currentRegion.insert(currentIndex);
		if(assignProgressToReflectance)
			(*((LaserPoints*)(&laserPoints)))[currentIndex].Reflectance() = currentRegion.size();
		avg = normals[currentIndex];
		
		//Erase this point, we don't need it any more.
		ErasePoint(currentIndex,pointStateVector,unUsedPointCount,residuals,maxRes,histogram);
		//debuggingPoints[currentIndex].Reflectance() = iterationCounter++;
		
		
		for(i=0;i<potentialSeedPoints.size();i++)
		{
			currentIndex = potentialSeedPoints[i];

			vector<int> indices = finder.FindIndices(laserPoints[currentIndex],kNNRegionGrowing);

			for(j=0;j<kNNRegionGrowing;j++)
			{
				if(pointStateVector[indices[j]])
				{
					bool isSimilar;

					if(useStrictNormalSimilarity)
					{
						isSimilar = IsNormalSimilarStrict(normals[indices[j]],avg);
					}
					else
					{
						isSimilar = IsNormalSimilar(normals[currentIndex],normals[indices[j]]);
					}
					
					#if ENABLE_DEBUGGING
					{
						#define SH(a) cerr<<#a<<": "<<a<<endl;
						SH(residuals[indices[j]]);
						SH(thresholdMerge);
						SH(thresholdSeed2);
						SH(isSimilar);
						SH(currentRegion.size());
						SH(pointStateVector[indices[j]]);
					}
					#endif
					
					
					if(residuals[indices[j]]<(thresholdMerge+eps)
						&& isSimilar)
					{
						if(pointStateVector[indices[j]])
						{
							currentRegion.insert(indices[j]);
							if(assignProgressToReflectance)
								(*((LaserPoints*)(&laserPoints)))[indices[j]].Reflectance() = currentRegion.size();
							if(useStrictNormalSimilarity)
								UpdateAvg(avg,normals[indices[j]]);
							if(residuals[indices[j]]<(thresholdSeed2+eps))
							{
								potentialSeedPoints.push_back(indices[j]);
							}
							ErasePoint(indices[j],pointStateVector,unUsedPointCount,residuals,maxRes,histogram);
							progress.Step("Growing regions ",pointCount - unUsedPointCount + currentRegion.size());
						}
					}
					#if ENABLE_DEBUGGING
						SH(currentRegion.size());
					#endif
				}
			}					
		}
		
		//This new region is finished.
		//Add current Region to global list.
		if(currentRegion.size() > kNNRegionGrowing || useStrictNormalSimilarity)
		{
			vector<int> temp;
			temp.resize(currentRegion.size());
			copy(currentRegion.begin(),currentRegion.end(),temp.begin());
			temp.reserve(currentRegion.size());
			segmentedIndices.push_back(temp);
		
			DEBUG("SegmentUsingSmoothnessConstraint: Region %d with %d points finished",segmentedIndices.size(),temp.size());
		}
		else
		{
			#if ENABLE_DEBUGGING
				cerr<<"Region is too small. We better not add it to segmented list."<<endl;
			#endif
		}
		
		currentIndex = 0;
		
		if(unUsedPointCount>kNNRegionGrowing)
		{
			//We can either keep a fixed number of threshold-updates or we can do so after every region is added.
			if(std::abs(lastThresholdUpdate-unUsedPointCount)>=(pointCount/500))
			{
				thresholdSeed1 = histogram.PercentValue(seedFactor1*100);
				thresholdSeed2 = histogram.PercentValue(seedFactor2*100);
				thresholdMerge = histogram.PercentValue(mergeFactor*100);
			
				#if ENABLE_DEBUGGING
					cerr<<"Unused points: "<<pointCount<<"  "<<unUsedPointCount<<endl;
					cerr<<"New thresholds: "<<thresholdSeed1<<"   "<<thresholdSeed2<<"   "<<thresholdMerge<<endl;
				#endif
			}			
		}				
		//fprintf(stderr,"\rGrowing Regions: %4.1f%% with %d Regions",100.00*(double)(pointCount-unUsedPointCount)/(double)pointCount,segmentedIndices.size());
	}
	//sort the resultant segments with respect to size.
	sort(segmentedIndices.begin(),segmentedIndices.end(),CompareIndicesVector);
	
	fprintf(stderr,"\rSegmentUsingSmoothnessConstraint: Finished !!!!!                      \n");
	
	//Just chekc the difference in sizes.
	int segmentedCount = 0;
	for(int i=0;i<segmentedIndices.size();i++)
		segmentedCount += segmentedIndices[i].size();
	cerr<<"Segmentation size "<<segmentedCount<<"/"<<laserPoints.size()<<" with "<<segmentedIndices.size()<<" regions"<<endl;
	
	//ShowLaserPoints(debuggingPoints,"DebuggingTheSegmentationProcess");
}



double
CalculateDistanceFromLineSegment(Vector3D point,
					Vector3D startPoint,
					Vector3D endPoint,
					Vector3D *pPosition)
{
	Vector3D position;
  	//Normalize the axis.
	const Vector3D axis = (endPoint-startPoint);
	Vector3D a = axis.Normalize();
		
	double dot = (point-startPoint).DotProduct(a);
	double distance = -1;
	
	if(dot<0)
	{
		distance = (point-startPoint).Length();
		position = startPoint;
	}
	else
	{
		if(dot>axis.Length())
		{
			distance =  (point-endPoint).Length();
			position = endPoint;
		}
		else
		{
			distance = (a.VectorProduct(point-startPoint)).Length();
			position = startPoint + a*dot;
		}
	}
	if(pPosition)
		*pPosition = position;
	
	return distance;
	
}



static double
dist3D_Line_to_Line(Vector3D L1P0, Vector3D L1P1, Vector3D L2P0, Vector3D L2P1,
	Vector3D* midPoint=NULL)
{
    const double SMALL_NUM = 1e-6;
	Vector3D   u = L1P1 - L1P0;
    Vector3D   v = L2P1 - L2P0;
    Vector3D   w = L1P0 - L2P0;
    float    a = u.DotProduct(u);        // always >= 0
    float    b = u.DotProduct(v);
    float    c = v.DotProduct(v);        // always >= 0
    float    d = u.DotProduct(w);
    float    e = v.DotProduct(w);
    float    D = a*c - b*b;       // always >= 0
    float    sc, tc;

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) {         // the lines are almost parallel
        sc = 0.0;
        tc = (b>c ? d/b : e/c);   // use the largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // get the difference of the two closest points
    Vector3D   dP = w + (u*sc) - (v*tc);  // = L1(sc) - L2(tc)
	
	if(midPoint)
	{
		Vector3D p1 = L1P0 + u*sc;
		Vector3D p2 = L2P0 + v*tc;
		
		*midPoint = (p1+p2)*0.5;
	}
    
	return dP.Length();   // return the closest distance
}


LaserPoints GetNormalIntersections(const LaserPoints& laserPoints,
						const vector<Vector3D>& normals,
						const int kNNBegin, const int kNNEnd, 
						const double distanceThreshold,
						const double intersectionThreshold)
{

	//Make a finder object for knn search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	LaserPoints intersectionPoints;
	intersectionPoints.reserve(laserPoints.size()*(kNNEnd-kNNBegin+1));
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
	double factor = distanceThreshold*10;
	
	for(int i=0;i<laserPoints.size();i++)
	{
		vector<int> indices = finder.FindIndices(laserPoints[i],kNNEnd);
		progress.Step("Calculating intersection: ",i);		
		for(int k=kNNBegin;k<kNNEnd;k++)
		{
			int index = indices[k];
			Vector3D mid;
			
			double dist = dist3D_Line_to_Line(
				Vector3D(laserPoints[i]), Vector3D(laserPoints[i])+normals[i]*factor,
				Vector3D(laserPoints[index]), Vector3D(laserPoints[index])+normals[index]*factor,&mid);
			
			if(dist< intersectionThreshold && (laserPoints[i]-mid).Length()<distanceThreshold)
				intersectionPoints.push_back(mid);
		}
	}
	return intersectionPoints;	
}	



vector<Vector3D> MakeNormalsConsistent(const LaserPoints& laserPoints, vector<double>&rhos, int kNN)
{		
	//Make a finder object for knn search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	vector<Vector3D> consistent = laserPoints.Normals(kNN,rhos,true,NULL);
	
	set<int> used;
	vector<int> seeds;
	seeds.reserve(laserPoints.size());	
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
	seeds.push_back(0);
	used.insert(0);
	
	int count = 0;
	for(int i=0;i<seeds.size();i++)
	{
		int seed = seeds[i];
		vector<int> indices = finder.FindIndices(laserPoints[seed],kNN);
		progress.Step("Calculating consistent normals: ",i);
		
		(*((LaserPoints*)(&laserPoints)))[seed].Reflectance() = i;
		
		for(int j=0;j<indices.size();j++)
		{
			if(consistent[seed].DotProduct(consistent[indices[j]])<0.3)
			{
				consistent[indices[j]] = consistent[indices[j]]*(-1.00);
				rhos[indices[j]]*=-1;
				count++;
			}
				
			if(!used.count(indices[j]))
			{
				used.insert(indices[j]);
				seeds.push_back(indices[j]);
			}
		}
	}
	cerr<<"\n\n Number of normals reversed is "<<count<<"\n\n";
	return consistent;
}		

vector<int> GenerateUniqueRandom(int low, int high, int count)
{
	vector<int> indices;
	set<int> usedIndices;
	for(int i=0;i<high;i++)
	{
		int index = GenerateRandom(low,high);

		if(!usedIndices.count(index))
		{
			usedIndices.insert(index);
			indices.push_back(index);
			
			if(indices.size()>=count)
				break;
		}
	}
	return indices;
}

vector<vector<int> > FindConnectedComponents(const LaserPoints& laserPoints, int kNN, bool showProgress)
{		
	LaserConnectedSegments segments(const_cast<LaserPoints*>(&laserPoints), kNN, showProgress);
	return segments;
}		

	
	

LaserCylinder* GrowCylinder(LaserPoints& laserPoints,
				LaserPoints& segLaserPoints,
				SegmentsVector& planarSegments,
				vector<Vector3D> normals,
				int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& selectedIndices,
				const int ransacIterations)
{		
	//NOTE CAREFULLY:
	//The segmentation must reference into segLaserPoints, as the other laserPoints is being
	//changed externally, and the segmentation indices would become invalid after the first call.
	
	//Number of planar segments, used to approximate cylinder axis.
	//Minimum is two.
	const int segSelSize = 2;
	
	
	//At least 2*knn points.
	if(laserPoints.size()<2*kNN)
		return NULL;
	
	//Filter the segments to select only those which have more than kNN points.
	planarSegments = FilterOnSize(planarSegments,kNN);
	
	//Make a finder for segments.
	SegmentFinder finder(segLaserPoints,planarSegments,kNN);
	
	
	///Too few segments.	
	if(planarSegments.size()<segSelSize)
		return NULL;
		
	
	int maxValid = -1;
	//Start the iterations.
	LaserCylinder* maxCylPre = NULL, *maxCyl = NULL;
	for(int iter=0;iter<ransacIterations;iter++)
	{
		vector<int> selSeg = GenerateUniqueRandom(0,planarSegments.size()-1,segSelSize);
		
////////////////////////////////
        //Get all neighbors
		vector<int> nn = finder.FindAllNeighbors(selSeg[0],1);

			
		if(nn.size() && selSeg.size()>1)
		{
			selSeg[1] = nn[nn.size()-1];
		}
		else 
			continue;
	
		//cerr<<"iter: "<<iter<<" : ("<<selSeg[0]<<", "<<selSeg[1]<<")"<<endl;
		//finder.ToLaserPoints(nn).ShowVrml();
///////////////////////////////		
		vector<int> selIndices;
		
#if 0		
		for(int i=0;i<selSeg.size();i++)
			selIndices = selIndices + planarSegments[selSeg[i]];
#endif			
			
		//Get all neighbours
		selIndices = Combine(Select(planarSegments,nn));

		///To few points.
		if(selIndices.size()<2*kNN)
			continue;
			
		LaserPoints selPts = segLaserPoints.Select(selIndices);
		
		//Approximate cylinder orientation.
		Vector3D axis(0,0,1);			
		if(selSeg.size()>=2)
		{
			Vector3D n1 =  segLaserPoints.Select(planarSegments[selSeg[0]]).Normal();
			Vector3D n2 = segLaserPoints.Select(planarSegments[selSeg[1]]).Normal();
			
			//SHV(planarSegments[selSeg[0]].size());SHV(selSeg[0]);SHV(planarSegments[selSeg[1]].size());SHV(selSeg[1]);
			//n1.Print("n1");n2.Print("n2");
			
			if(acos(fabs(n1.DotProduct(n2)))*180./M_PI < 10.)
			{
				//cerr<<"Angle is bad\r";
				continue;
			}
			///Take anytwo normals, their vector product should give us the 
			//axis direction of the cylinder.
			axis = n1.VectorProduct(n2);
			
			if(axis.Length()>1e-12)
				axis = axis.Normalize();
			else
				continue;
		}
		
		LaserCylinder* cyl = new LaserCylinder(FitCylinder(selPts,LaserCylinder(selPts,axis,selPts.Mean(),1)));
		
//		cerr<<"cyl->AxisDirection(): "<<cyl->AxisDirection()<<endl;
//		cerr<<"axis approx: "<<axis<<endl;
		
		//We don't want null vector posing as valid.
		if(cyl->AxisDirection().Length()<0.5)
			continue;
			
		vector<int64> selCyl = cyl->SelectPoints(laserPoints,normals,
						cos(angleThresholdDegrees*M_PI/180.),distanceThreshold);
		
		if((selCyl.size()>maxValid || maxValid<0))
		{
//			cerr<<"max valid is : "<<maxValid<<endl;
			maxValid = selCyl.size();
			Copy(selCyl,selectedIndices);
			cerr<<"iter: "<<iter<<"  maxSel: "<<maxValid<<"  remaining laser points: "<<laserPoints.size()<<"        \r";
			
			delete maxCylPre;
			maxCylPre = cyl;
			
		}
		else
		{
			delete cyl;
		}
	}
	
	
	if(maxValid>0)
	{
		//Refit to update bounds.
		LaserPoints selPts = laserPoints.Select(selectedIndices);
		
		maxCyl = new LaserCylinder(FitCylinder(selPts,*maxCylPre));
		delete maxCylPre; maxCylPre = NULL; 		
		maxCyl->Print();
		
		if(!AnalyzeAngularHistogram(Select(normals,selectedIndices),maxCyl->AxisDirection(),40))
		{
			delete maxCyl;
			maxCyl = NULL;
		}
		else
		{
			//To remove the points select again but with a lenient threshold.
			double leniencyFactor = 2;
			vector<int64> selCyl = maxCyl->SelectPoints(laserPoints,normals,
						cos(leniencyFactor*angleThresholdDegrees*M_PI/180.),leniencyFactor*distanceThreshold);
			
			//vector<int64> selCyl = maxCyl->SelectPointsDistance(laserPoints,distanceThreshold*2);
			Copy(selCyl,selectedIndices);
			
			//Lets find connected components of selected points.
			vector<vector<int> > components = FindConnectedComponents(laserPoints.Select(selectedIndices),kNN,false);

			//Update bounds. Use the biggest segment to do that.
			if(components.size())
			{
				maxCyl->UpdateBounds(laserPoints.Select(Select(selectedIndices,components[0])));
				//Also the selected Indices should mark our selection of the largest component.
				selectedIndices = Select(selectedIndices,components[0]);
			}
		}
	}
	
	return maxCyl;
}


//Grow more than one cylinder if possible.
vector<LaserCylinder*> GrowCylinders(LaserPoints& laserPoints,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndices,
				const int maxIterations,
				const int maxRansacIteration)
{	

	int iterCount = 0;
	vector<LaserCylinder*> cyls;
	LaserPoints usedPoints;
	
	///Calculate normals for points.
	vector<Vector3D> normals = laserPoints.Normals(kNN);
	
	//Keep original indices to update segmentation.
	vector<int> originalIndices = FillSequence(0,(int)laserPoints.size(),1);
	
	LaserPoints segLaserPoints = laserPoints;
	
	LaserPlanarSegments planarSegments(&segLaserPoints,kNN,angleThresholdDegrees,distanceThreshold);
				
	planarSegments.FilterFixedArea(distanceThreshold*10, kNN, true);
	
	finalSelectedIndices.resize(0);
	
	while(1) 
	{
		cerr<<"Iter: "<<iterCount<<endl;
						
		iterCount++;
		
		if(maxIterations>0 && iterCount > maxIterations)
		{
			break;
		}
		
		
		vector<int> selectedIndices;
		LaserCylinder* cyl = GrowCylinder(laserPoints, segLaserPoints, 
								planarSegments, normals, kNN, 
								angleThresholdDegrees,distanceThreshold, selectedIndices,maxRansacIteration);
		
		//if we failed, or too few points belong to this cylinder.
		if(selectedIndices.size()<2*kNN)
		{
			if(cyl) 
			{
				delete cyl;
				cyl = NULL;
			}
			continue;
		}
		vector<int> currentSel = Select(originalIndices,selectedIndices);
		//We have enough points but still the operation failed. So remove the points and continue.
		if(cyl)
		{
			cyl->SetIndices(currentSel);
			cyls.push_back(cyl);
			finalSelectedIndices = finalSelectedIndices + currentSel;
			
			usedPoints = usedPoints + segLaserPoints.Select(currentSel).SetReflectance(cyls.size());
		}
		
		FilterOutUsed(planarSegments,currentSel);
				
		//Select unused points and continue in the next iteration.
		laserPoints = SelectInverse(laserPoints,selectedIndices);
		normals = SelectInverse(normals,selectedIndices);
		originalIndices = SelectInverse(originalIndices,selectedIndices);
		
	}
	usedPoints.ShowVrml("GrowCylindersUsedPoints");
	cerr<<"\n\nWe have found "<<cyls.size()<<" cylinders\n";
	
	return cyls;
}


//Grow one plane.				
LaserPlane* GrowPlane(LaserPoints& laserPoints,
				LaserPoints& segLaserPoints,
				SegmentsVector& planarSegments,
				vector<Vector3D> normals,
				int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& selectedIndices)
{		
	//At least 2*knn points.
	if(laserPoints.size()<2*kNN)
		return NULL;
	
	//Filter the segments to select only those which have more than kNN points.
	planarSegments = FilterOnSize(planarSegments,kNN);
		
	///Too few segments.	
	if(planarSegments.size()<1)
		return NULL;
		
	
	int maxValid = -1;
		
	//Take the biggest segment.
	vector<int> selIndices = planarSegments[0];
	
	///To few points.
	if(selIndices.size()<2*kNN)
		return NULL;
			
	//Select the points.
	LaserPoints selPts = segLaserPoints.Select(selIndices);

	//Fit a plane to it.
	LaserPlane* plane = new LaserPlane(FitPlane(selPts));

	vector<int64> selPlane = plane->SelectPoints(laserPoints,normals,
					cos(angleThresholdDegrees*M_PI/180.),distanceThreshold);

	if(selPlane.size()>0)
	{
		
		//To remove the points select again but with a lenient threshold.
		double leniencyFactor = 2;
		vector<int64> selPlane = plane->SelectPoints(laserPoints,normals,
						cos(leniencyFactor*angleThresholdDegrees*M_PI/180.),leniencyFactor*distanceThreshold);
			
		Copy(selPlane,selectedIndices);
			
		//Lets find connected components of selected points.
		vector<vector<int> > components = FindConnectedComponents(laserPoints.Select(selectedIndices),10,false);

		//Update bounds. Use the biggest segment to do that.
		if(components.size())
		{
			selectedIndices = Select(selectedIndices,components[0]);
			LaserPoints selPts = laserPoints.Select(selectedIndices);
			
			//Lets check the aspect ratio to get rid of very elongated kind of planes.
			vector<double> exts = selPts.Extents();
			double aspect = exts[0]/exts[1];
//			cerr<<"aspect ratio is "<<aspect<<endl;
//			cerr<<"selection size is "<<selectedIndices.size()<<endl;
//			cerr<<"laser points is "<<laserPoints.size()<<endl;
			//Not a good one.
			if(aspect > 4 || selPts.size()<2*kNN)
			{
				delete plane;
				return NULL;
			}
			
//			static int count =0;
//			char buff[4096]; sprintf(buff,"/home/star/dump/seg_%d.laser",count);
//			selPts.Save(buff);count++;
			
			plane->UpdateHull(selPts);
			plane->Print();
			return plane;
			
		}
	}
	return NULL;
}
				

//Grow more than one planes if possible.
vector<LaserPlane*> GrowPlanes(LaserPoints& laserPoints,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndices,
				const int maxIterations)
{	

	int iterCount = 0;
	vector<LaserPlane*> planes;
	
	///Calculate normals for points.
	vector<Vector3D> normals = laserPoints.Normals(kNN);
	
	//Keep original indices to update segmentation.
	vector<int> originalIndices = FillSequence(0,(int)laserPoints.size(),1);
	
	LaserPoints segLaserPoints = laserPoints;
	LaserPlanarSegments planarSegments(&segLaserPoints,kNN,angleThresholdDegrees,distanceThreshold);
	
	//Filter on size.
	planarSegments.FilterOnSize(kNN);
	
	finalSelectedIndices.resize(0);
	
	while(1) 
	{
		iterCount++;
		
		if(maxIterations>0 && iterCount > maxIterations)
		{
			break;
		}
		
		
		vector<int> selectedIndices;
		LaserPlane* plane = GrowPlane(laserPoints, segLaserPoints, 
								planarSegments, normals, kNN, 
								angleThresholdDegrees,distanceThreshold, selectedIndices);
		
		//if we failed, or too few points belong to this plane.
		if(selectedIndices.size()<2*kNN)
		{
			if(plane) 
			{
				delete plane;
				plane = NULL;
			}
			break;
		}
		vector<int> currentSel = Select(originalIndices,selectedIndices);
		
		if(plane)
		{
			plane->SetIndices(currentSel);
			planes.push_back(plane);
			finalSelectedIndices = finalSelectedIndices + currentSel;
		}
		
		FilterOutUsed(planarSegments,currentSel);
		
		//Select unused points and continue in the next iteration.
		laserPoints = SelectInverse(laserPoints,selectedIndices);
		normals = SelectInverse(normals,selectedIndices);
		originalIndices = SelectInverse(originalIndices,selectedIndices);
		
	}
	
	cerr<<"\n\nWe have found "<<planes.size()<<" planes\n";
	
	return planes;
}


//Grow both, and select the best model.
vector<LaserObject*> GrowCylindersAndPlanes(
				LaserPoints& laserPointsIn,int kNN,
				const double angleThresholdDegrees,
				const double distanceThreshold,
				vector<int>& finalSelectedIndicesIn,
				const int maxIterations,
				const int maxRansacIteration)
{
	//First grow only cylinders.
	LaserPoints laserPoints = laserPointsIn;
	vector<int> finalSelectedIndicesCyls;
	vector<LaserCylinder*> cyls = GrowCylinders(laserPoints,kNN,
					angleThresholdDegrees,distanceThreshold,
					finalSelectedIndicesCyls,
					maxIterations,maxRansacIteration);
	
	//Then only planes.
	laserPoints = laserPointsIn;					
	vector<int> finalSelectedIndicesPlanes;
	vector<LaserPlane*> planes = GrowPlanes(laserPoints,kNN,
					angleThresholdDegrees,distanceThreshold,
					finalSelectedIndicesPlanes);
	
	finalSelectedIndicesIn = finalSelectedIndicesCyls;		
	
	//Filter out the planes which share more than 10% of its vertices with any cylinder.
	set<int> cylPoints(finalSelectedIndicesCyls.begin(),finalSelectedIndicesCyls.end());
	
	vector<LaserObject*> finalObjects;
	for(int i=0;i<planes.size();i++)
	{
		LaserPlane* plane = planes[i];
		vector<int>	indices;
		Copy(plane->GetIndices(),indices);
		
		vector<int> disjoint;
		
		int common = 0;
		
		for(int j=0;j<indices.size();j++)
			if(cylPoints.count(indices[j]))
				common++;
			else
				disjoint.push_back(indices[j]);
				
		if((double)common/(double)indices.size() > 0.1)
		{
			cerr<<"Rejecting "<<i<<"-th plane  with common "<<common<<" and total "<<indices.size()<<endl;
			delete plane;
		}
		else
		{
			finalObjects.push_back(plane);
			finalSelectedIndicesIn = finalSelectedIndicesIn + disjoint;
		}
	}
	finalObjects = finalObjects + cyls;
	
	cerr<<"Returning found objects "<<finalObjects.size()<<endl;
	
	return finalObjects;

}		


/*
				
///Searching in terms of segments. Two segments are neighbours if the points in one have points in the other as 
///its knn neighours. Each segment must know its neighours beforehand, so that we can search quickly.
void SearchUsingSegments(LaserPoints& laserPoints,SegmentsVector& segments, int kNN)
{	

	///Calculate normals for points.
	vector<Vector3D> normals = laserPoints.Normals(kNN);
				
	SegmentFinder finder(laserPoints, segments, kNN);
	
#if 0	
	int test[] = {segments.size()-1, segments.size()/2, segments.size()/3, segments.size()/7};
	for(int i=0;i<4;i++)
		finder.ToLaserPointsColored(finder.FindNeighbors(test[i],3,true)).ShowVrml("test");
#endif		
	
	Histo3D<int> hist(0.01);
	
	double intersectionThreshold = 0.005;
	//double distanceThresholdInt
	
	ProgressDisplay<int> progress(segments.size(),"Progress");
	
	//Keep a track of combinations already processed.
	typedef pair<int,int> IdCouple;
	set<IdCouple> done;
	
	//Lets calculate the intersection points.
	for(int i=0;i<segments.size();i++)
	{		
		progress.Step("Calculating intersection: ",i);		
		
		vector<int>& srcIndices = segments[i];
		
		///Search for points suitable for intersection.
		//vector<int> segInd = finder.FindBestNeighborsB(i,5,false); 
		
		//vector<int> segInd = finder.FindConstrainedNeighbors(i,30,false); 
		
		vector<int> segInd = finder.FindNeighbors(i,4,false); 
		
		if(segInd.empty())
		{
			//cerr<<"\nNo neighors found for segment: "<<i<<endl;
			continue;
		}
		
		//cerr<<"For "<<i<<" the first neighbor is "<<segInd[0]<<" total are "<<segInd.size()<<endl;
			
		for(int s=0;s<segInd.size();s++)
		{
			if(done.count(IdCouple(i,segInd[s])))
				continue;
			
			done.insert(IdCouple(i,segInd[s]));
			done.insert(IdCouple(segInd[s],i));
			
			double angle = finder.Angle(i,segInd[s]);
			
			if(angle <7.5 || angle>30)
				continue;
				
			vector<int>& destIndices = segments[segInd[s]];

			for(int j=0;j<srcIndices.size();j++)
			{
				int src = srcIndices[j];
				for(int k=0;k<destIndices.size();k++)
				{
					int dest = destIndices[k];

					Vector3D mid;
					double factor = 1;

					double dist = dist3D_Line_to_Line(
						Vector3D(laserPoints[src]), Vector3D(laserPoints[src])+normals[src]*factor,
						Vector3D(laserPoints[dest]), Vector3D(laserPoints[dest])+normals[dest]*factor,&mid);

					if(dist< intersectionThreshold && (laserPoints[src]-mid).Length()<0.5)
						hist.Add(mid);
				}
			}
		}
	}
	((laserPoints.SetReflectance(0) + hist.ToPointsFiltered(1)).Histoequalize(256)).ShowVrml("SearchUsingSegments");
	
}
	
*/	

vector<vector<int> > FindFixedAreaComponents(const LaserPoints& laserPoints, int kNN, double maxRadius, bool showProgress)
{		
	LaserFixedAreaSegments segments(const_cast<LaserPoints*>(&laserPoints), kNN, 
							maxRadius,showProgress);
	
	return segments;
	
}		

///Apply fixed area filter to any segmentation. Just divide each segment using fixed area component breakdown.
vector<vector<int> > FixedAreaFilter(const LaserPoints& laserPoints, 
					vector<vector<int> > & segmentsIn,
					int knn, double maxRadius, bool showProgress)
{					
	LaserSegments segments(const_cast<LaserPoints*>(&laserPoints));
	segments = segmentsIn;
	
	segments.FilterFixedArea(maxRadius,knn,showProgress);
	
	return segments;
}



bool AnalyzeAngularHistogram(const vector<Vector3D>& nv, Vector3D axis, const double percentage)
{
	//Make a temporary cylinder.
	LaserCylinder cyl(axis*(-2), axis*2, 1);
	
	vector<int> hist = cyl.AngularHistogram(nv);
	
	//Print(hist);
		
	//int threshold = max(Max(hist)*0.25+1,(double)selPts.size()/(hist.size()));
	int threshold = 0.5*max((double)Median(hist),(double)nv.size()/(hist.size()));
	int histSelSize = FindGE(hist,threshold).size();
	
	//cerr<<"threshold "<<threshold<<endl;
	//cerr<<"median "<<Median(hist)<<endl;
	cerr<<"histSelSize: "<<histSelSize<<" from total of "<<hist.size()<<" ->  "<<histSelSize*100.00/hist.size()<<"%"<<endl;
	cerr<<flush;
	bool result = histSelSize>= (percentage/100.00*hist.size());
	
	return  result;
}

