
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
*   File made : October 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Defines a space partitioning scheme for 3D points.
*				Uses a simple cubic partition, but keeps data in a map
*				to exploit the sparsity of data. Each box in space partition
*				is represented by PartitionBox, and its data is given by
*				PartitioinBoxData.
*
*--------------------------------------------------------------------*/
#ifndef ___POINTS___SPACE_PARTITION__H__
#define ___POINTS___SPACE_PARTITION__H__

#include <vector>
#include <iomanip>
#include <map>
#include <ext/hash_set>
#include <ext/hash_map>
#include <algorithm>
#include <iostream>
#include <Vector3D.h>
#include "LaserPoints.h"
#include "GeneralUtility.h"
#include "VrmlUtility.h"
#include "VisualizationUtility.h"
#include "PointsSpacePartition.h"
#include "FunctionTimer.h"
#include "KNNFinder.h"
#include "ProgressDisplay.h"
#include "PartitionBox.h"

using namespace std;
// Namespace alias to catch hash_set classes
namespace stdext = ::__gnu_cxx; 
using stdext::hash_set;
using stdext::hash_map;


static bool 
CompareIndicesVector(const vector<int> &a, const vector<int> &b) 
{
    return a.size() > b.size();
}

///Keeps data for one box in space partitioning.
class PartitionBoxData
{
public:
	vector<int> indicesVector;
	Vector3D mean;
	Vector3D normal;
	double rho;
	double error;
	
	void Print()const
	{
		cerr<<"PartitionBoxData:"<<endl;
		cerr<<"\t mean: ("<<mean[0]<<", "<<mean[1]<<", "<<mean[2]<<")"<<endl;
		cerr<<"\t normal: ("<<normal[0]<<", "<<normal[1]<<", "<<normal[2]<<")"<<endl;
		cerr<<"\t rho: "<<rho<<"   error: "<<error<<endl;
	}
};

		
//class T is for the data-type of PartitionBox (short-int or int whatever is big enough)
//PointVector is a container of points, with each point supporting operator[] with 0,1,2
//and PointVector having functions size(), Max()[], Min()[] and Range()[]
template <class T,class PointVector>	
class PointsSpacePartition
{
public:
	typedef PartitionBox<T> PBox;
	typedef PartitionBoxCompare<T> PBoxCompare;

#define USE_HASHING 0
#if USE_HASHING
	typedef hash_map<PBox,PartitionBoxData,HashPartitionBox<T>,HashPartitionBoxEqual<T> > PartitionMap;
	typedef hash_set<PBox,HashPartitionBox<T>,HashPartitionBoxEqual<T> > PBoxSet;
#else	
	typedef map<PBox,PartitionBoxData,PBoxCompare> PartitionMap;
	typedef std::set<PBox,PBoxCompare> PBoxSet;
#endif	
	typedef PartitionMap::iterator MapIterator;
			typedef typename PointVector::value_type PointType;
	typedef std::vector<PBox> PBoxVector;
	
	
	///default constructor.
	PointsSpacePartition()
	:pPointVector(NULL),xCount(10),yCount(10),zCount(10)
	{
		UpdateConstants();
	}
	
	///Constructor from a given pointer. with specified box counts along x,y,and z
	///The final box-count will be adjusted so that all boxes are square.
	PointsSpacePartition(PointVector* p ,T xC,T yC,T zC)
	:pPointVector(p),xCount(xC),yCount(yC),zCount(zC)
	{
		UpdateConstants();
	}
	
	///Constuructor using a given pointer and a specified step size.
	PointsSpacePartition(PointVector* p,double stepSize)
	:pPointVector(p)
	{
		UpdateConstants(stepSize);
	}
	
	//Set up data pointer.
	void SetPointVector(PointVector* p)
	{
		pPointVector = p;
	}
	
	//Get Point vector.
	PointVector* GetPointVector()
	{
		return pPointVector;
	}
	
	//Set step sizes.
	void SetStepSize(double stepSize)
	{
		UpdateConstants(stepSize);
	}		
	
	//Set box counts.
	void SetBoxCounts(int x=10,int y=0,int z=0)
	{
		x = (x>0)?x:10;
		xCount = x;
		yCount = (y>0)?y:x;
		zCount = (z>0)?z:x;
		
		UpdateConstants();	
	}
	
	///Updates constants using fixed step size.
	void UpdateConstants(double stepSize=0)
	{
		if(!pPointVector)
			return;
		
		double xMin,yMin,zMin,xMax,yMax,zMax;

#if 1
		xMin=yMin=zMin=1e40;
		xMax=yMax=zMax=-1e40;
		PointVector& pointVector = *pPointVector;
		for(int i=0;i<pointVector.size();i++)
		{
			xMin = std::min(xMin,pointVector[i][0]);
			xMax = std::max(xMax,pointVector[i][0]);
			yMin = std::min(yMin,pointVector[i][1]);
			yMax = std::max(yMax,pointVector[i][1]);
			zMin = std::min(zMin,pointVector[i][2]);
			zMax = std::max(zMax,pointVector[i][2]);
		}
#endif 

#if 0
		PointType valMin = pPointVector->Min();
		PointType valMax = pPointVector->Max();
		xMin = valMin[0]; yMin = valMin[1]; zMin = valMin[2];
		xMax = valMax[0]; yMax = valMax[1]; zMax = valMax[2];
#endif		
		xOffset = xMin; yOffset = yMin; zOffset = zMin;
		
		if(stepSize==0)
		{
			xFactor = (xMax-xMin)?((double)xCount/(xMax-xMin)):1e20;
			yFactor = (yMax-yMin)?((double)yCount/(yMax-yMin)):1e20;
			zFactor = (zMax-zMin)?((double)zCount/(zMax-zMin)):1e20;

			//Try same size as max-step size in all directions.
			//This will give us true cubic cells.
			double maxFactor = MAX(zFactor,MAX(xFactor,yFactor));
			double minFactor = MIN(zFactor,MIN(xFactor,yFactor));
			xFactor = yFactor = zFactor = minFactor;
		}
		else
			xFactor = yFactor = zFactor = 1.00/stepSize;
		
		xCount = (xMax-xMin)*xFactor + 1;
		yCount = (yMax-yMin)*yFactor + 1;
		zCount = (zMax-zMin)*zFactor + 1;
		
	}
	
	///Convert a given point to box index
	PBox Point2Box(PointType& pt)
	{
		return PBox((pt[0]-xOffset)*xFactor,(pt[1]-yOffset)*yFactor,(pt[2]-zOffset)*zFactor);
	}
	
	///Convert a given box to point
	PointType Box2Point(PBox b)
	{
		//fprintf(stderr,"Box2Point: %d %d %d xFactor: %f xOffset: %f yFactor: %f yOffset: %f, zFactor: %f zOffset: %f\n",
		//b.x,b.y,b.z,xFactor,xOffset,yFactor,yOffset,zFactor,zOffset);
		return PointType(b.x/xFactor+xOffset,b.y/yFactor+yOffset,b.z/zFactor+zOffset);
	}
	
	///Convert a given box to point
	vector<PointType> Box2CornerPoints(PBox b)
	{
		vector<PointType> corners;
		for(int k=0;k<=1;k++)		
		{
			corners.push_back(Box2Point(PBox(b.x,b.y,b.z+k)));
			corners.push_back(Box2Point(PBox(b.x+1,b.y,b.z+k)));
			corners.push_back(Box2Point(PBox(b.x,b.y+1,b.z+k)));
			corners.push_back(Box2Point(PBox(b.x+1,b.y+1,b.z+k)));
		}
		return corners;
	}
	
	///Convert a box-centre to point.
	PointType BoxCentre2Point(PBox b)
	{
		//fprintf(stderr,"Box2Point: %d %d %d xFactor: %f xOffset: %f yFactor: %f yOffset: %f, zFactor: %f zOffset: %f\n",
		//b.x,b.y,b.z,xFactor,xOffset,yFactor,yOffset,zFactor,zOffset);
		return PointType((b.x+0.5)/xFactor+xOffset,(b.y+0.5)/yFactor+yOffset,(b.z+0.5)/zFactor+zOffset);
	}		
	
	///Check if the partitioning is valid.
	bool IsValid()
	{
		if(!pPointVector)
			return false;
			
		//counts shouldn't be zero.
		if(xCount<=0 || yCount<=0 || zCount <=0 )
			return false;
			
		//add further checks.
			
		return true;
	}

	///Updates the partition. Must be explicitly called.
	///Constructor doesn't update automatically.
	int UpdatePartition(double stepSize = 0)
	{
		if(!IsValid())
			return 1;
			
		if(stepSize>0)
		{
			//Update the constants 
			UpdateConstants(stepSize);
		}
		
		//clear the map.
		partitionMap.clear();
		
		//Update the partition.
		PointVector& pointVector = *pPointVector;

		//Buffer size gives the amount of overlap in box extents.
		const double bufferSize = 0;
		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pointVector.size()/1000,pointVector.size());
		
		if(bufferSize==0)
		{
			for(int i=0;i<pointVector.size();i++)
			{
				progress.Step("Updating partition ",i);
				partitionMap[Point2Box(pointVector[i])].indicesVector.push_back(i);
			}
		}
		else
		{
			double low = bufferSize;
			double high = 1 - bufferSize;
			for(int i=0;i<pointVector.size();i++)
			{
				PointType pt = pointVector[i];
				double x = (pt[0]-xOffset)*xFactor;
				double y = (pt[1]-yOffset)*yFactor;
				double z = (pt[2]-zOffset)*zFactor;
				
				double fracX = x - int(x);
				double fracY = y - int(y);
				double fracZ = z - int(z);
				
				partitionMap[PBox(x,y,z)].indicesVector.push_back(i);
				
				if(fracX<low)
					partitionMap[PBox(x-1,y,z)].indicesVector.push_back(i);
				else if(fracX>high)
					partitionMap[PBox(x+1,y,z)].indicesVector.push_back(i);
					
				if(fracY<low)
					partitionMap[PBox(x,y-1,z)].indicesVector.push_back(i);
				else if(fracY>high)
					partitionMap[PBox(x,y+1,z)].indicesVector.push_back(i);
				
				if(fracZ<low)
					partitionMap[PBox(x,y,z-1)].indicesVector.push_back(i);
				else if(fracZ>high)
					partitionMap[PBox(x,y,z+1)].indicesVector.push_back(i);
			}
		}	
		
		progress.End("Finished updating partition. ");
		
		return 0;	
	}
	
	//Subsamples and return is a PointVector.
	//Just calculates the mean, and puts one point for each box.
	LaserPoints SubSample()
	{
		LaserPoints subSampled;
		subSampled.reserve(partitionMap.size());
		
		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pPointVector->size()/1000,pPointVector->size());
		int total = 0;		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			vector<int> iv = iter->second.indicesVector;
			LaserPoints sel = pPointVector->Select(iv);
			subSampled.push_back(sel.Mean());
			
			progress.Step("Subsampling ",total+=iv.size());
		}
		progress.End("Done subsampling ");
		return subSampled;
	}
	
	///Calculates density. Works only if function Reflectance() is provided for PointType
	PointVector CalculateDensity()
	{
		PointVector& pointVector = *pPointVector;
		PointVector densityPoints;
		
		//Copy the points and just change the reflectance.
		densityPoints = pointVector;
		for(int i=0;i<pointVector.size();i++)
		{
			PartitionBoxData box = partitionMap[Point2Box(pointVector[i])];
			densityPoints[i].Reflectance() = box.indicesVector.size();
		}
		
		return densityPoints;
	}
	
	//Reorder the points and return the results.
	void Reorder(PointVector& ordered)
	{
		PointVector& pointVector = *pPointVector;
		ordered.resize(0);
		ordered.reserve(pointVector.size());
		
	
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			IndicesVector &iv = iter->second.indicesVector;
			for(int i=0;i<iv.size();i++)
			{
				ordered.push_back(pointVector[iv[i]]);
			}
		}
	}
	
	
	///Calculate normals using k-nearest neighbours.
	///Currently uses functions which are implemented in LaserPoints.
	///In future convert internally all types to LaserPoints, this way
	///if PointVector supports conversion to LaserPoints we can still use
	///the functionality provided by LaserPoints.
	vector<Vector3D> CalculateNormals(int kNN,bool showProgress=true)
	{
		vector<Vector3D> normals;
		CalculateNormals(normals,kNN,showProgress);
		return normals;
	}
	
	
	template <class NormalVector>
	void CalculateNormals(NormalVector& normals,int kNN,bool showProgress=true)
	{
		CalculateNormals< NormalVector, vector<double> >(normals,kNN,NULL,showProgress);
	}
	
	template <class NormalVector,class ResidualVector>
	void CalculateNormals(NormalVector& normals,int kNN,ResidualVector* pResiduals = NULL,bool showProgress=true)
	{
		const int progressCount = 200;
		PointVector& pointVector = *pPointVector;
		if(pResiduals)
			pResiduals->resize(pointVector.size());
		normals.resize(pointVector.size());
		
		int64 processedCount = 0;
		int64 insufficientKnn = 0;
		KNNFinder<PointType> finder;
		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			IndicesVector &iv = iter->second.indicesVector;
			
			IndicesVector neighbours = GetNeighbourIndices(iter->first,kNN);
			LaserPoints neighbourPoints = pointVector.Select(neighbours);
					
			//We have tried all possible ways. Lets fit one normal and assign to all.
			if(neighbourPoints.size()<kNN)
			{
				Vector3D n(1,1,1);
				 
				if(kNN>10)
					n = neighbourPoints.Normal();
				
				for(int i=0;i<iv.size();i++)
				{
					normals[iv[i]] = n;
					processedCount++;
					insufficientKnn++;
				}
			}
			else
			{
				//Only if more than kNN points can we fit a normal per vertex
				finder.SetData(neighbourPoints);

				for(int i=0;i<iv.size();i++)
				{
					normals[iv[i]] = neighbourPoints.Normal(finder.FindIndices(pointVector[iv[i]],kNN),
															pResiduals?(&((*pResiduals)[iv[i]])):NULL);
					processedCount++;
					
					//show progress
					if(showProgress && processedCount%(pointVector.size()/progressCount)==0)
					{
						cerr<<"\rPointsSpacePartition::CalculateNormals "
							<<setprecision(3)<<setw(5)
							<<(double)processedCount/(double)pointVector.size()*100<<" % "
							<<"  "<<processedCount<<"/"<<pointVector.size()
							<<"  insufficientKnn: "<<insufficientKnn<<"          ";
					}
				}
			}
			
			
		}
		
		if(showProgress)
				cerr<<"\nPointsSpacePartition::CalculateNormals done!!!\n";
	}
	
	//Find length squared of a point3D
	double LengthSquared(const Vector3D& a)
	{
		return (a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}
	
	//Calculate normals using Fixed Distance Neighbourhood.
	template <class NormalVector,class ResidualVector>
	void CalculateNormalsFDN(NormalVector& normals,double fdnDistance,ResidualVector* pResiduals = NULL,bool showProgress=true)
	{
		this->Print();
				
		//Note this function will only work for laser points or one of its variants.
		LaserPoints& pointVector = *((LaserPoints*)(pPointVector));
		if(pResiduals)
			pResiduals->resize(pointVector.size());
		normals.resize(pointVector.size());
		
		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pointVector.size()/200,pointVector.size());
		
		int64 processedCount = 0;
		int64 insufficientKnn = 0;
		
		//Get how many boxes we might need to search.
		//Get box count for checking.
		int levelCount = fdnDistance/MinBoxSize();
		
		//One level below level Count is safe.
		int safeLevel = levelCount - 1;
		
		//For level Count we must increment it by 1.
		levelCount = levelCount+1;
		int maxNeighbours = -1e20;
		int minNeighbours = 1e20;
		double avgNeighbours = 0;
		
		//cerr<<endl<<"LevelCount: "<<levelCount<<" safeLevels: "<<safeLevel<<endl<<endl;
		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Indices for this box.
			IndicesVector &iv = iter->second.indicesVector;
			
			
			
		#if 0
			//Old method, doesn't seem to work reliably.			
			vector<int> safeIndices = Boxes2Indices(GetNLevelNeighbours(iter->first,safeLevel));
			vector<int> unsafeIndices = Boxes2Indices(GetM2NLevelNeighbours(iter->first,safeLevel+1,levelCount));
		#else
			//Corner based sub-selection.
			//Get the 8-corner points of the current box.
			PointVector corners = Box2CornerPoints(iter->first);
			vector<int> all = Boxes2Indices(GetNLevelNeighbours(iter->first,levelCount));
			
			//Assume the indices of current box are safe.
			//This would be true if fdnDistance > boxSize
			vector<int> safeIndices;
			
			vector<int> unsafeIndices;
			
			double safeLimit = fdnDistance ;//- 0*MaxBoxSize();
			double fdnDistanceSqr = fdnDistance*fdnDistance;
			double safeLimitSqr = safeLimit*safeLimit;
			
			for(int k=0;k<all.size();k++)
			{
				bool bSafe = true;
				PointType current = pointVector[all[k]];
				
				for(int l=0;l<corners.size();l++)
				{
					if(!(LengthSquared(corners[l]-current) < safeLimitSqr))
					{
						bSafe = false;
						break;
					}
				}
				
				if(bSafe)
				{
					safeIndices.push_back(all[k]);
				}
				else
				{
					bool bUnsafe = false;
					for(int l=0;l<corners.size();l++)
					{
						if((LengthSquared(corners[l]-current) <= fdnDistanceSqr))
						{
							bUnsafe = true;
							break;
						}
					}
					
					if(bUnsafe)
						unsafeIndices.push_back(all[k]);
				}
										
			}
		
		#endif
			
			
			for(int i=0;i<iv.size();i++)
			{
				vector<int> neighbours = safeIndices;
				PointType queryPt = pointVector[iv[i]];
				
				for(int j=0;j<unsafeIndices.size();j++)
				{
					if(LengthSquared(pointVector[unsafeIndices[j]]-queryPt)<=fdnDistanceSqr)
					{
						neighbours.push_back(unsafeIndices[j]);
					}
				}
				
				if(neighbours.size()>10)
					normals[iv[i]] = pointVector.Select(neighbours).Normal();
				else
					normals[iv[i]] = Vector3D(0,0,1);
				maxNeighbours = max(maxNeighbours,(int)neighbours.size());
				minNeighbours = min(minNeighbours,(int)neighbours.size());
				avgNeighbours += neighbours.size()/(double)pointVector.size();
				progress.Step("Calcuating normals using FDN ",processedCount++);
			}
		}
		
		if(showProgress)
		{
			progress.End("\nPointsSpacePartition::CalculateNormalsFDN");
			//cerr<<"\nPointsSpacePartition::CalculateNormalsFDN done!!!\n";
			cerr<<"maxNN = "<<maxNeighbours<<"  minNN = "<<minNeighbours<<"  avgNN = "<<avgNeighbours<<endl;
			
		}
	}
	
	///Calculate curvatures.
	template <class CurvatureVector,class DirectionVector>
	void CalculateCurvatures(double fdnDistance,CurvatureVector* pMajorCurvatures=NULL,
		CurvatureVector* pMinorCurvatures=NULL, DirectionVector* pMajorDirections=NULL, DirectionVector* pMinorDirections=NULL,
		bool showProgress = true)
	{
		//Note this function will only work for laser points or one of its variants.
		LaserPoints& pointVector = *((LaserPoints*)(pPointVector));
		
		if(pMajorCurvatures)
			pMajorCurvatures->resize(pointVector.size());
			
		if(pMinorCurvatures)
			pMinorCurvatures->resize(pointVector.size());
			
		if(pMajorDirections)
			pMajorDirections->resize(pointVector.size());
			
		if(pMinorDirections)
			pMinorDirections->resize(pointVector.size());
		
		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pointVector.size()/200,pointVector.size());
		
		int64 processedCount = 0;
		int64 insufficientKnn = 0;
		
		//Get how many boxes we might need to search.
		//Get box count for checking.
		int levelCount = fdnDistance/MinBoxSize();
		
		//One level below level Count is safe.
		int safeLevel = levelCount - 1;
		
		//For level Count we must increment it by 1.
		levelCount = levelCount+1;
		int maxNeighbours = -1e20;
		int minNeighbours = 1e20;
		double avgNeighbours = 0;
		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Indices for this box.
			IndicesVector &iv = iter->second.indicesVector;
			
			//Corner based sub-selection.
			//Get the 8-corner points of the current box.
			PointVector corners = Box2CornerPoints(iter->first);
			vector<int> all = Boxes2Indices(GetNLevelNeighbours(iter->first,levelCount));
			
			//Assume the indices of current box are safe.
			//This would be true if fdnDistance > boxSize
			vector<int> safeIndices;
			
			vector<int> unsafeIndices;
			
			double safeLimit = fdnDistance ;//- 0*MaxBoxSize();
			double fdnDistanceSqr = fdnDistance*fdnDistance;
			double safeLimitSqr = safeLimit*safeLimit;
			
			for(int k=0;k<all.size();k++)
			{
				bool bSafe = true;
				PointType current = pointVector[all[k]];
				
				for(int l=0;l<corners.size();l++)
				{
					if(!(LengthSquared(corners[l]-current) < safeLimitSqr))
					{
						bSafe = false;
						break;
					}
				}
				
				if(bSafe)
				{
					safeIndices.push_back(all[k]);
				}
				else
				{
					bool bUnsafe = false;
					for(int l=0;l<corners.size();l++)
					{
						if((LengthSquared(corners[l]-current) <= fdnDistanceSqr))
						{
							bUnsafe = true;
							break;
						}
					}
					
					if(bUnsafe)
						unsafeIndices.push_back(all[k]);
				}
										
			}
		
					
			for(int i=0;i<iv.size();i++)
			{
				vector<int> neighbours = safeIndices;
				PointType queryPt = pointVector[iv[i]];
				
				for(int j=0;j<unsafeIndices.size();j++)
				{
					if(LengthSquared(pointVector[unsafeIndices[j]]-queryPt)<=fdnDistanceSqr)
					{
						neighbours.push_back(unsafeIndices[j]);
					}
				}
				
				double majorCurvature = 0;
				double minorCurvature = 0;
				Vector3D majorDirection, minorDirection;
				if(neighbours.size()>10)
				{
					pointVector.Select(neighbours).Curvature(&majorCurvature,&minorCurvature,&majorDirection,&minorDirection);
				}
				
				//Copy the results back.
				if(pMajorCurvatures)
					(*pMajorCurvatures)[iv[i]] = majorCurvature;
			
				if(pMinorCurvatures)
					(*pMinorCurvatures)[iv[i]] = minorCurvature;

				if(pMajorDirections)
					(*pMajorDirections)[iv[i]] = majorDirection;

				if(pMinorDirections)
					(*pMinorDirections)[iv[i]] = minorDirection;
				
				//Update status variables.
				maxNeighbours = max(maxNeighbours,(int)neighbours.size());
				minNeighbours = min(minNeighbours,(int)neighbours.size());
				avgNeighbours += neighbours.size()/(double)pointVector.size();
				progress.Step("Calcuating curvature using FDN ",processedCount++);
			}
		}
		
		if(showProgress)
		{
			progress.End("\nPointsSpacePartition::CalculateCurvatureFDN");
			//cerr<<"\nPointsSpacePartition::CalculateNormalsFDN done!!!\n";
			cerr<<"maxNN = "<<maxNeighbours<<"  minNN = "<<minNeighbours<<"  avgNN = "<<avgNeighbours<<endl;
			
		}
	
	
	}	
	
	//For each point get the neighbour-count Fdn can find.
	vector<int> GetFdnCount(double fdnDistance, bool showProgress=true)
	{
		//Note this function will only work for laser points or one of its variants.
		LaserPoints& pointVector = *((LaserPoints*)(pPointVector));
		vector<int> nnCount(pointVector.size());

		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pointVector.size()/200,pointVector.size());
		
		int64 processedCount = 0;
		int64 insufficientKnn = 0;
		
		//Get how many boxes we might need to search.
		//Get box count for checking.
		int levelCount = fdnDistance/MinBoxSize();
		
		//One level below level Count is safe.
		int safeLevel = levelCount - 1;
		
		//For level Count we must increment it by 1.
		levelCount = levelCount+1;
		
		int maxNeighbours = -1e20;
		int minNeighbours = 1e20;
		double avgNeighbours = 0;
				
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Indices for this box.
			IndicesVector &iv = iter->second.indicesVector;
			
			//Corner based sub-selection.
			//Get the 8-corner points of the current box.
			PointVector corners = Box2CornerPoints(iter->first);
			vector<int> all = Boxes2Indices(GetNLevelNeighbours(iter->first,levelCount));
			
			//Assume the indices of current box are safe.
			//This would be true if fdnDistance > boxSize
			vector<int> safeIndices;
			
			vector<int> unsafeIndices;
			
			double safeLimit = fdnDistance ;//- 0*MaxBoxSize();
			
			double fdnDistanceSqr = fdnDistance*fdnDistance;
			double safeLimitSqr = safeLimit*safeLimit;
			
			for(int k=0;k<all.size();k++)
			{
				bool bSafe = true;
				PointType current = pointVector[all[k]];
				
				for(int l=0;l<corners.size();l++)
				{
					if(!(LengthSquared(corners[l]-current) < safeLimitSqr))
					{
						bSafe = false;
						break;
					}
				}
				
				if(bSafe)
				{
					safeIndices.push_back(all[k]);
				}
				else
				{
					bool bUnsafe = false;
					for(int l=0;l<corners.size();l++)
					{
						if((LengthSquared(corners[l]-current) <= fdnDistanceSqr))
						{
							bUnsafe = true;
							break;
						}
					}
					
					if(bUnsafe)
						unsafeIndices.push_back(all[k]);
				}
										
			}
		
					
			for(int i=0;i<iv.size();i++)
			{
				vector<int> neighbours = safeIndices;
				PointType queryPt = pointVector[iv[i]];
				
				for(int j=0;j<unsafeIndices.size();j++)
				{
					if(LengthSquared(pointVector[unsafeIndices[j]]-queryPt)<=fdnDistanceSqr)
					{
						neighbours.push_back(unsafeIndices[j]);
					}
				}
				nnCount[iv[i]] = neighbours.size();
				
				//Update status variables.
				maxNeighbours = max(maxNeighbours,(int)neighbours.size());
				minNeighbours = min(minNeighbours,(int)neighbours.size());
				avgNeighbours += neighbours.size()/(double)pointVector.size();
		
				
				progress.Step("GetFdnCount using FDN ",processedCount++);
			}
		}
		
		if(showProgress)
		{
			progress.End("\nPointsSpacePartition::GetFdnCount");
			cerr<<"maxNN = "<<maxNeighbours<<"  minNN = "<<minNeighbours<<"  avgNN = "<<avgNeighbours<<endl;
		}
		return nnCount;
	}	
	
	//For each point get the neighbour-count Fdn can find.
	//Try a new method.
	vector<int> GetFdnCountIndirect(double fdnDistance, vector<int>& countVector,bool showProgress=true)
	{
		//Note this function will only work for laser points or one of its variants.
		LaserPoints& pointVector = *((LaserPoints*)(pPointVector));
		vector<int> nnCount(pointVector.size());

		ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
		progress.Initialize(pointVector.size()/200,pointVector.size());
		
		KNNFinder<LaserPoint> finder(pointVector);
		
		int64 processedCount = 0;
		int64 insufficientKnn = 0;
		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Indices for this box.
			IndicesVector &iv = iter->second.indicesVector;
			
			for(int i=0;i<iv.size();i++)
			{
				vector<int> neighbours = finder.FindIndices(pointVector[iv[i]],countVector[iv[i]]);
				
				progress.Step("GetFdnCountIndirect using FDN ",processedCount++);
			}
		}
		
		if(showProgress)
		{
			progress.End("\nPointsSpacePartition::GetFdnCountIndirec");
		}
		return nnCount;
	}	
	
	//Get all boxes in form of a vector.
	PBoxVector GetAllBoxesVector()const
	{
		PBoxVector boxes;
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			boxes.push_back(iter->first);;
		}
		return boxes;
	}
	
	//Get neighbourhood indices for knn
	vector<int> GetNeighbourIndices(PBox box, int kNN,
							int strictConnectivityLevels=4,int relaxedConnectivityLevels=10)
	{
		vector<PBox > currentBoxes = Get27Neighbours(box);
		vector<int> neighbourPoints = Boxes2Indices(currentBoxes);
			
		//If the points are not enough, try going 3-levels up.
		//would work if the boxes are somehow connected.
		if(neighbourPoints.size()<kNN)
		{
			for(int i=0;i<strictConnectivityLevels;i++)
			{
				//Get neighbours of a next level.
				currentBoxes = Get27Neighbours(currentBoxes);
				neighbourPoints = Boxes2Indices(currentBoxes);
										
				//if we have enough points lets break the chase.
				if(neighbourPoints.size()>kNN)
					break;
			}
		}
			
		//This means we have really separated points, lets try relaxed
		//connectivity.
		if(neighbourPoints.size()<kNN)
		{
			for(int i=2;i<relaxedConnectivityLevels;i++)
			{
				//Get neighbours of a next level.
				currentBoxes = GetNLevelNeighbours(box,i);
				neighbourPoints = Boxes2Indices(currentBoxes);

				//if we have enough points lets break the chase.
				if(neighbourPoints.size()>kNN)
				{
					break;
				}
			}
		}
		
		//return the found box indices.
		return neighbourPoints;
	}
	
	//Get all boxes in form of a set.
	PBoxSet GetAllBoxesSet()const
	{
		PBoxSet boxes;
		for(PartitionMap::const_iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			boxes.insert(iter->first);;
		}
		return boxes;
	}
	
//Try to find smooth regions based on connectivity and normal similarity.
//A given upper percentile of high residual points are classified as dirty ones.
//They cannot be used as seeds for next generation except if the parent it self is dirty.
//Also the points below alwaysValid percent are considered fit for first level parenthood.
void
SegmentIntoSmoothRegions(
				vector<vector<int> >& segmentedIndices,				
				const int kNN = 20,
				const double angleThresholdDegrees=10,
				const double distanceThreshold=1,
				const double invalidPercentage = 10,
				const double alwaysValidPercentage = 30)
				
{
#if 1
	//Data for KNNFinder.
	const int expandLevel = 3;
	//should be less than expand level.
	const int validLevel = 2; 

	PointVector& pointVector = *pPointVector;
	//Check the size of points, if too small return.
	if(pointVector.size()<2*kNN)
	{
		cerr<<"SegmentIntoSmoothRegions: too few points \n";
		return;
	}
	
	//Progress display related variables.
	bool bShowProgress= true;
	int showProgressCounter = 0;
	int reportCount = 1000;
	int reportInterval = pointVector.size()/reportCount;
	
		
	//Iterate over all laserPoints, finding normal for every point.
    //Allocate vectors for normals.
	vectorMM<double> residuals;
	residuals.setFileName("segmentation.residuals.mmap",true);
	
	vectorMM<Vector3D> normals;
	normals.setFileName("segmentation.normals.mmap",true);
	
	CalculateNormals(normals,kNN,&residuals,true);
	
	//We need a state vector to keep track of which points have been already used.
	vector<bool> stateVector(pointVector.size(),true);	
	
	//Get all available boxes in the form of a set.
	PBoxSet availableBoxes = GetAllBoxesSet();
	
	//Calculate thresholds and preallocate.
	int unUsedPointCount = pointVector.size();
	double angleThreshold = cos(angleThresholdDegrees*M_PI/180.00);
	segmentedIndices.reserve(4096);
	reportCount = 0;
	
	cerr<<"distanceThreshold: "<<distanceThreshold<<endl;
	cerr<<"angleThreshold: "<<angleThreshold<<endl;
	cerr<<"invalidPercentage: "<<invalidPercentage<<endl;
	cerr<<"alwaysValidPercentage: "<<alwaysValidPercentage<<endl;
	
	//Calculating the thresholds based on residuals.
	vector<double> residualsCopy = residuals.ToVector();
	sort(residualsCopy.begin(),residualsCopy.end());
	
	double invalidThreshold = residualsCopy[(int)(residualsCopy.size()*((100 - invalidPercentage)*0.01))];
	double alwaysValidThreshold = residualsCopy[(int)(residualsCopy.size()*(alwaysValidPercentage*0.01))];
	
	//Data for KNN tree.
	vector<int> finderIndices;
	LaserPoints finderPoints;
	KNNFinder<LaserPoint> finder;
	finder.Reserve(100000);
	PBoxSet finderBoxes;
	PBoxVector finderUsedBoxes;
	int knnTreeCount = 0;
	
	//scratch pads.
	vector<int> currentRegion;
	vector<int> potentialSeedPoints;
	const int reservedSize = 10000;
	currentRegion.reserve(reservedSize);
	potentialSeedPoints.reserve(reservedSize);
	
#define IS_UNUSED(index) (stateVector[index])
#define ERASE(index) {if(IS_UNUSED(index)){stateVector[index]= false; unUsedPointCount--;}}
#define ADD_TO_REGION(index)	{currentRegion.push_back(index); \
						Vector3D pt = (*pPointVector)[index]; \
							sumX2 += pt.X()*pt.X(); sumY2 += pt.Y()*pt.Y(); sumZ2 += pt.Z()*pt.Z(); \
							sumX += pt.X(); sumY += pt.Y(); sumZ += pt.Z();\
							sumXY += pt.X()*pt.Y(); sumXZ += pt.X()*pt.Z(); sumYZ+=pt.Y()*pt.Z();}
							
    int iterationCounter = 0;	
	//Start Region Growing.
	while(1)
	{
#define SH(a) //cerr<<#a<<(a)<<endl<<" at line "<<__LINE__<<endl<<flush;		
		if(unUsedPointCount == 0)			
			break;	
		
SH(unUsedPointCount);
SH(availableBoxes.size());
		//Get the box which is free.
		PBox currentBox;
		bool boxFound = false;
				
		for(PBoxSet::iterator iter=finderBoxes.begin();iter!=finderBoxes.end();iter++)
		{
			//Check to see if we have some free point.
			vector<int> indices = Box2Indices(*iter);
			
			for(int i=0;i<indices.size();i++)
			{
				if(stateVector[indices[i]])
				{
					//we have at least one point.
					currentBox = *iter;
					boxFound = true;
					break;
				}
			}
			
			if(boxFound)
				break;
		}
		
		
		if(!boxFound)
		for(PBoxSet::iterator iter=availableBoxes.begin();iter!=availableBoxes.end();iter++)
		{
			//Check to see if we have some free point.
			vector<int> indices = Box2Indices(*iter);
			
			bool boxIsUsed = true;
			for(int i=0;i<indices.size();i++)
			{
				if(stateVector[indices[i]])
				{
					//we have at least one point.
					boxIsUsed = false;
					currentBox = *iter;
					boxFound = true;
					break;
				}
			}
			
			//If the box was completely used mark it we don't need it any more.
			if(boxIsUsed)
			{
				availableBoxes.erase(*iter);
			}
			else if(boxFound)
				break;
		}
		
		if(!boxFound)
		{
			cerr<<"\nFailed to find a box something is wrong\n";
			break;
		}
		
			
SH("Found the new box");		
		if(finderBoxes.find(currentBox)==finderBoxes.end())
		{
			finderUsedBoxes = GetNLevelNeighbours(currentBox,expandLevel);
			finderIndices = Boxes2Indices(finderUsedBoxes);
		
			finderPoints = pPointVector->Select(finderIndices);
			finder.SetData(finderPoints);
			SH(finderPoints.size());	
		
			//Make a set for checking the valid boxes.
			PBoxVector tempVector = GetNLevelNeighbours(currentBox,validLevel);
			finderBoxes = PBoxSet(tempVector.begin(),tempVector.end());
			knnTreeCount++;
		}
		
		if(finderIndices.size()<kNN)
		{
			//We haven't been able to find at least kNN neighbours.
			//Lets just put the points in one segment and proceed with other boxes.
			segmentedIndices.push_back(finderIndices);
			
			//Mark them as used.
			for(int i=0;i<finderIndices.size();i++)
			{
				stateVector[finderIndices[i]] = false;
				unUsedPointCount--;
			}
			
			//Now continue with the next box and its neighbours.
			continue;
		}
		
		//We have more than Knn points so we must proceed.
		//Select the point with minimum residual as the next seed.
		double minResidual = DBL_MAX;
		int currentIndex = -1;
		for(int i=0;i<finderIndices.size();i++)
		{
			if(stateVector[finderIndices[i]] && residuals[finderIndices[i]]<minResidual)
			{
				//Not that currentIndex is pointing into pPointVector.
				currentIndex = finderIndices[i];
				minResidual = residuals[currentIndex];
			}
		}
SH(currentIndex);
SH(minResidual);
		if(currentIndex<0)
		{
			cerr<<"It cannot be. current index is "<<currentIndex
				<<" something somewhere is going wroing\n";
			break;
		}
		
		
		//We have found the new main seed, so continue from there.
				
		//make scratch pads.
		currentRegion.resize(0);
		potentialSeedPoints.resize(0);
		double sumX2=0,sumY2=0,sumZ2=0,sumX=0,sumY=0,sumZ=0,sumXY=0,sumXZ=0,sumYZ=0;
		
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		ADD_TO_REGION(currentIndex);
		ERASE(currentIndex);
		
		for(int i=0;i<potentialSeedPoints.size();i++)
		{
SH(0);			
			currentIndex = potentialSeedPoints[i];
			
			PBox currentBox = Point2Box((*pPointVector)[currentIndex]);
			
			//If the finder is not valid for the current box, lets make it so
			if(finderBoxes.find(currentBox)==finderBoxes.end())
			{
				//KNN Finder is invalid must update for the currentIndex.
				
				//Get all neighbour boxes and filter out the ones which are empty.
				PBoxVector allBoxes = GetNLevelNeighbours(currentBox,expandLevel);
				finderUsedBoxes.resize(0);
				finderIndices.resize(0);
				
				//Make a set for checking the valid boxes.
				PBoxVector tempVector = GetNLevelNeighbours(currentBox,validLevel);
				finderBoxes = PBoxSet(tempVector.begin(),tempVector.end());
SH(100);			
				//Filter allBoxes against availability. Get rid of the boxes which are empty.
				for(int i=0;i<allBoxes.size();i++)
				{
SH(101);
					if(availableBoxes.find(allBoxes[i])!=availableBoxes.end())
					{
						vector<int> currentIndices = Box2Indices(allBoxes[i]);
						int indicesSelected = 0;
						
						for(int j=0;j<currentIndices.size();j++)
						{
							if(stateVector[currentIndices[j]])
							{
								finderIndices.push_back(currentIndices[j]);
								indicesSelected++;
							}
						} 
						if(indicesSelected)
						{
							//We have some points, so keep it in the used set.
							finderUsedBoxes.push_back(allBoxes[i]);
						}
						else
						{	
							//This box is empty, get rid of it.
							availableBoxes.erase(allBoxes[i]);
						}
					}
				}
								
				//Select the points and make the tree.
				finderPoints = pPointVector->Select(finderIndices);
				finder.SetData(finderPoints);
		
				knnTreeCount++;
			}
			
			//Find neighbouring points
			vector<int> neighbours;
			 
			if(finderIndices.size()>kNN)
			 	neighbours = finder.FindIndices((*pPointVector)[currentIndex],kNN);
			else
			{
				neighbours.resize(finderIndices.size());
				for(int i=0;i<neighbours.size();i++)
					neighbours[i] = i;
			}
SH(1);				
			for(int j=0;j<neighbours.size();j++)
			{
SH("processing");
SH(j);				
				if(IS_UNUSED(finderIndices[neighbours[j]]))
				{
					double thisIndex = finderIndices[neighbours[j]];
					double distance = ((*pPointVector)[currentIndex] - (*pPointVector)[thisIndex]).Length();
					double angle = fabs(normals[currentIndex].DotProduct(normals[thisIndex]));
SH(thisIndex);					
					if(distance<distanceThreshold  &&  angle>angleThreshold)
					{
SH("adding");						
						ADD_TO_REGION(thisIndex);
						
						if(residuals[thisIndex]<invalidThreshold 
							|| residuals[currentIndex]>invalidThreshold)
							potentialSeedPoints.push_back(thisIndex);
SH("erasing");						
						ERASE(thisIndex);
						
						//Comment it out in the final version.
						//Just assigns reflectance to show progress of region growing.
					#if 0
						LaserPoints* pPts = (LaserPoints*)(&laserPoints);
						(*pPts)[neighbours[j]].Reflectance() = currentRegion.size();
					#endif
					}
				}
			}
SH(2);			
			if(bShowProgress && ((pPointVector->size()-unUsedPointCount) - reportCount*reportInterval)>reportInterval)
			{
				fprintf(stderr,"Growing regions %5.1f%%  regions: %d    current_size: %d  trees_made: %d   tree_points: %d  available_boxes: %d   valid_boxes: %d usedBoxes: %d \r",
							(double)(pPointVector->size()-unUsedPointCount)/(double)pPointVector->size()*100.00,
							segmentedIndices.size()+1,currentRegion.size(),
							knnTreeCount,finderIndices.size(),availableBoxes.size(),finderBoxes.size(),finderUsedBoxes.size());
				reportCount++;
			}
		}
SH(3);		
		//This new region is finished.
		//Add current Region to global list.
		segmentedIndices.push_back(currentRegion);
	}
	
	fprintf(stderr,"\nRegion growing finished\n");
	fprintf(stderr,"\nSorting %d regions with %d points...",segmentedIndices.size(),pointVector.size());
	fprintf(stderr,"Finished!!!\n");
	//sort the resultant segments with respect to size.
	sort(segmentedIndices.begin(),segmentedIndices.end(),CompareIndicesVector);
	
	fprintf(stderr,"\SegmentIntoPlanarRegions: Finished !!!!!                      \n");
	
	//Just chekc the difference in sizes.
	int segmentedCount = 0;
	for(int i=0;i<segmentedIndices.size();i++)
		segmentedCount += segmentedIndices[i].size();
	cerr<<"Segmentation size "<<segmentedCount<<"/"<<pPointVector->size()<<" with "<<segmentedIndices.size()<<" regions"<<endl;
#endif	
}


void
SegmentIntoPlanarRegions(
				vector<vector<int> >& segmentedIndices,				
				const int kNN = 20,
				const double angleThresholdDegrees=10,
				const double distanceThreshold=1)
				
{
#if 1
	//Data for KNNFinder.
	const int expandLevel = 3;
	//should be less than expand level.
	const int validLevel = 2; 

	PointVector& pointVector = *pPointVector;
	//Check the size of points, if too small return.
	if(pointVector.size()<2*kNN)
	{
		cerr<<"SegmentIntoPlanarRegions: too few points \n";
		return;
	}
	
	//Progress display related variables.
	bool bShowProgress= true;
	int showProgressCounter = 0;
	int reportCount = 1000;
	int reportInterval = pointVector.size()/reportCount;
	
		
	//Iterate over all laserPoints, finding normal for every point.
    //Allocate vectors for normals.
	vectorMM<double> residuals;
	residuals.setFileName("segmentation.residuals.mmap",true);
	
	vectorMM<Vector3D> normals;
	normals.setFileName("segmentation.normals.mmap",true);
	
	CalculateNormals(normals,kNN,&residuals,true);
	
	//We need a state vector to keep track of which points have been already used.
	vector<bool> stateVector(pointVector.size(),true);	
	
	//Get all available boxes in the form of a set.
	PBoxSet availableBoxes = GetAllBoxesSet();
	
	//declaration of the function defined in LaserPointsFItting.cc
	Vector3D FindNormal(const LaserPoints& X, const vector<int>& indices,double* pResidual = 0);
	Vector3D FindPlane(const LaserPoints& X, const vector<int>& indices,double &rho,double* pResidual = 0);
		
	//Calculate thresholds and preallocate.
	int unUsedPointCount = pointVector.size();
	double angleThreshold = cos(angleThresholdDegrees*M_PI/180.00);
	segmentedIndices.reserve(4096);
	reportCount = 0;
	
	cerr<<"distanceThreshold: "<<distanceThreshold<<endl;
	cerr<<"angleThreshold: "<<angleThreshold<<endl;
	
	//Data for KNN tree.
	vector<int> finderIndices;
	LaserPoints finderPoints;
	KNNFinder<LaserPoint> finder;
	finder.Reserve(100000);
	PBoxSet finderBoxes;
	PBoxVector finderUsedBoxes;
	int knnTreeCount = 0;
	
	//scratch pads.
	vector<int> currentRegion;
	vector<int> potentialSeedPoints;
	const int reservedSize = 10000;
	currentRegion.reserve(reservedSize);
	potentialSeedPoints.reserve(reservedSize);
	
#define IS_UNUSED(index) (stateVector[index])
#define ERASE(index) {if(IS_UNUSED(index)){stateVector[index]= false; unUsedPointCount--;}}
#define ADD_TO_REGION(index)	{currentRegion.push_back(index); \
						Vector3D pt = (*pPointVector)[index]; \
							sumX2 += pt.X()*pt.X(); sumY2 += pt.Y()*pt.Y(); sumZ2 += pt.Z()*pt.Z(); \
							sumX += pt.X(); sumY += pt.Y(); sumZ += pt.Z();\
							sumXY += pt.X()*pt.Y(); sumXZ += pt.X()*pt.Z(); sumYZ+=pt.Y()*pt.Z();}
							
    int iterationCounter = 0;	
	//Start Region Growing.
	while(1)
	{
#define SH(a) //cerr<<#a<<(a)<<endl<<" at line "<<__LINE__<<endl<<flush;		
		if(unUsedPointCount == 0)			
			break;	
		
SH(unUsedPointCount);
SH(availableBoxes.size());
		//Get the box which is free.
		PBox currentBox;
		bool boxFound = false;
				
		for(PBoxSet::iterator iter=finderBoxes.begin();iter!=finderBoxes.end();iter++)
		{
			//Check to see if we have some free point.
			vector<int> indices = Box2Indices(*iter);
			
			for(int i=0;i<indices.size();i++)
			{
				if(stateVector[indices[i]])
				{
					//we have at least one point.
					currentBox = *iter;
					boxFound = true;
					break;
				}
			}
			
			if(boxFound)
				break;
		}
		
		
		if(!boxFound)
		for(PBoxSet::iterator iter=availableBoxes.begin();iter!=availableBoxes.end();iter++)
		{
			//Check to see if we have some free point.
			vector<int> indices = Box2Indices(*iter);
			
			bool boxIsUsed = true;
			for(int i=0;i<indices.size();i++)
			{
				if(stateVector[indices[i]])
				{
					//we have at least one point.
					boxIsUsed = false;
					currentBox = *iter;
					boxFound = true;
					break;
				}
			}
			
			//If the box was completely used mark it we don't need it any more.
			if(boxIsUsed)
			{
				availableBoxes.erase(*iter);
			}
			else if(boxFound)
				break;
		}
		
		if(!boxFound)
		{
			cerr<<"\nFailed to find a box something is wrong\n";
			break;
		}
		
			
SH("Found the new box");		
		if(finderBoxes.find(currentBox)==finderBoxes.end())
		{
			finderUsedBoxes = GetNLevelNeighbours(currentBox,expandLevel);
			finderIndices = Boxes2Indices(finderUsedBoxes);
		
			finderPoints = pPointVector->Select(finderIndices);
			finder.SetData(finderPoints);
			SH(finderPoints.size());	
		
			//Make a set for checking the valid boxes.
			PBoxVector tempVector = GetNLevelNeighbours(currentBox,validLevel);
			finderBoxes = PBoxSet(tempVector.begin(),tempVector.end());
			knnTreeCount++;
		}
		
		if(finderIndices.size()<kNN)
		{
			//We haven't been able to find at least kNN neighbours.
			//Lets just put the points in one segment and proceed with other boxes.
			segmentedIndices.push_back(finderIndices);
			
			//Mark them as used.
			for(int i=0;i<finderIndices.size();i++)
			{
				stateVector[finderIndices[i]] = false;
				unUsedPointCount--;
			}
			
			//Now continue with the next box and its neighbours.
			continue;
		}
		
		//We have more than Knn points so we must proceed.
		//Select the point with minimum residual as the next seed.
		double minResidual = DBL_MAX;
		int currentIndex = -1;
		for(int i=0;i<finderIndices.size();i++)
		{
			if(stateVector[finderIndices[i]] && residuals[finderIndices[i]]<minResidual)
			{
				//Not that currentIndex is pointing into pPointVector.
				currentIndex = finderIndices[i];
				minResidual = residuals[currentIndex];
			}
		}
SH(currentIndex);
SH(minResidual);
		if(currentIndex<0)
		{
			cerr<<"It cannot be. current index is "<<currentIndex
				<<" something somewhere is going wroing\n";
			break;
		}
		
		
		//We have found the new main seed, so continue from there.
				
		//make scratch pads.
		currentRegion.resize(0);
		potentialSeedPoints.resize(0);
		double sumX2=0,sumY2=0,sumZ2=0,sumX=0,sumY=0,sumZ=0,sumXY=0,sumXZ=0,sumYZ=0;
		
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		ADD_TO_REGION(currentIndex);
		ERASE(currentIndex);
		
		//temporary locals.
		Vector3D planeNormal;
		double planeRho=0;
		int lastUpdate = 0;
				
		for(int i=0;i<potentialSeedPoints.size();i++)
		{
SH(0);			
			currentIndex = potentialSeedPoints[i];
			
			PBox currentBox = Point2Box((*pPointVector)[currentIndex]);
			
			//If the finder is not valid for the current box, lets make it so
			if(finderBoxes.find(currentBox)==finderBoxes.end())
			{
				//KNN Finder is invalid must update for the currentIndex.
				
				//Get all neighbour boxes and filter out the ones which are empty.
				PBoxVector allBoxes = GetNLevelNeighbours(currentBox,expandLevel);
				finderUsedBoxes.resize(0);
				finderIndices.resize(0);
				
				//Make a set for checking the valid boxes.
				PBoxVector tempVector = GetNLevelNeighbours(currentBox,validLevel);
				finderBoxes = PBoxSet(tempVector.begin(),tempVector.end());
SH(100);			
				//Filter allBoxes against availability. Get rid of the boxes which are empty.
				for(int i=0;i<allBoxes.size();i++)
				{
SH(101);
					if(availableBoxes.find(allBoxes[i])!=availableBoxes.end())
					{
						vector<int> currentIndices = Box2Indices(allBoxes[i]);
						int indicesSelected = 0;
						
						for(int j=0;j<currentIndices.size();j++)
						{
							if(stateVector[currentIndices[j]])
							{
								finderIndices.push_back(currentIndices[j]);
								indicesSelected++;
							}
						} 
						if(indicesSelected)
						{
							//We have some points, so keep it in the used set.
							finderUsedBoxes.push_back(allBoxes[i]);
						}
						else
						{	
							//This box is empty, get rid of it.
							availableBoxes.erase(allBoxes[i]);
						}
					}
				}
								
				//Select the points and make the tree.
				finderPoints = pPointVector->Select(finderIndices);
				finder.SetData(finderPoints);
		
				knnTreeCount++;
			}
			
			//Update plane equation if we have sufficient data.
			if(currentRegion.size()>kNN)
			{
				//Update only if 5% new points have been added to the region.
				if((currentRegion.size() - lastUpdate)>=(lastUpdate/20))
				{
					//planeNormal = FindPlane(pPointVector->Select(currentRegion),vector<int>(),planeRho);
					lastUpdate = currentRegion.size();
										
					Vector3D FindPlane(const double x2,const double y2,const double z2,
				const double xy, const double xz, const double yz,
				const double x, const double y, const double z,
				const int n, double &rho,double* pResidual = 0);
SH("Fitting Plane ");
					planeNormal = FindPlane(sumX2,sumY2,sumZ2,sumXY,sumXZ,sumYZ,sumX,sumY,sumZ,currentRegion.size(),planeRho);
SH("Fitting Plane Done");					
				}
			}
			else
			{
				planeNormal = normals[currentIndex];
				planeRho = planeNormal.DotProduct(pPointVector->Select(currentRegion).Mean());
			}
						
			vector<int> neighbours;
			 
			if(finderIndices.size()>kNN)
			 	neighbours = finder.FindIndices((*pPointVector)[currentIndex],kNN);
			else
			{
				neighbours.resize(finderIndices.size());
				for(int i=0;i<neighbours.size();i++)
					neighbours[i] = i;
			}
SH(1);				
			for(int j=0;j<neighbours.size();j++)
			{
SH("processing");
SH(j);				
				if(IS_UNUSED(finderIndices[neighbours[j]]))
				{
					double thisIndex = finderIndices[neighbours[j]];
					double distance = fabs(planeNormal.DotProduct((*pPointVector)[thisIndex])-planeRho);
					double angle = fabs(planeNormal.DotProduct(normals[thisIndex]));
SH(thisIndex);					
					if(distance<distanceThreshold  &&  angle>angleThreshold)
					{
SH("adding");						
						ADD_TO_REGION(thisIndex);
						potentialSeedPoints.push_back(thisIndex);
SH("erasing");						
						ERASE(thisIndex);
						
						//Comment it out in the final version.
						//Just assigns reflectance to show progress of region growing.
					#if 0
						LaserPoints* pPts = (LaserPoints*)(&laserPoints);
						(*pPts)[neighbours[j]].Reflectance() = currentRegion.size();
					#endif
					}
				}
			}
SH(2);			
			if(bShowProgress && ((pPointVector->size()-unUsedPointCount) - reportCount*reportInterval)>reportInterval)
			{
				fprintf(stderr,"Growing regions %5.1f%%  regions: %d    current_size: %d  trees_made: %d   tree_points: %d  available_boxes: %d   valid_boxes: %d usedBoxes: %d \r",
							(double)(pPointVector->size()-unUsedPointCount)/(double)pPointVector->size()*100.00,
							segmentedIndices.size()+1,currentRegion.size(),
							knnTreeCount,finderIndices.size(),availableBoxes.size(),finderBoxes.size(),finderUsedBoxes.size());
				reportCount++;
			}
		}
SH(3);		
		//This new region is finished.
		//Add current Region to global list.
		segmentedIndices.push_back(currentRegion);
	}
	
	fprintf(stderr,"\nRegion growing finished\n");
	fprintf(stderr,"\nSorting %d regions with %d points...",segmentedIndices.size(),pointVector.size());
	fprintf(stderr,"Finished!!!\n");
	//sort the resultant segments with respect to size.
	sort(segmentedIndices.begin(),segmentedIndices.end(),CompareIndicesVector);
	
	fprintf(stderr,"\SegmentIntoPlanarRegions: Finished !!!!!                      \n");
	
	//Just chekc the difference in sizes.
	int segmentedCount = 0;
	for(int i=0;i<segmentedIndices.size();i++)
		segmentedCount += segmentedIndices[i].size();
	cerr<<"Segmentation size "<<segmentedCount<<"/"<<pPointVector->size()<<" with "<<segmentedIndices.size()<<" regions"<<endl;
#endif	
}

	
	///Draw in OpenGL as a solid box.
	void DrawGLSolid()
	{	
		glMatrixMode(GL_MODELVIEW);
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			Vector3D shift = BoxCentre2Point(iter->first);
			
			glPushMatrix();
				glTranslatef(shift[0],shift[1],shift[2]);
				glScalef(1.00/xFactor,1.00/yFactor,1.00/zFactor);
				glutSolidCube(1);
			glPopMatrix();
		}
	}
		
	//Update data for each box.
	//Will work only for LaserPoints.
	int UpdateData()
	{
		if(!IsValid())
			return 1;
			
		PartitionMap::iterator iter;
		PointVector& pointVector = *pPointVector;
		
		for(iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Some references.
			PartitionBoxData &pbd = iter->second;
			IndicesVector& iv = pbd.indicesVector;
			
			FitPlaneToPointVector(pointVector,iv,pbd.normal,pbd.rho,&pbd.error);
					
			//temp local variables.
			Vector3D mean(0,0,0);
			for(int i=0;i<iv.size();i++)
			{
				mean = mean + Vector3D(pointVector[iv[i]]);
			}
			pbd.mean = mean*(1.00/(double)iv.size());
		}
		return 0;
	}
	
	
	///Just update the mean don't do anything for normal
	int UpdateMean()
	{
		if(!IsValid())
			return 1;
			
		PointVector& pointVector = *pPointVector;
		
		for(PartitionMap::iterator iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//Some references.
			PartitionBoxData &pbd = iter->second.mean;
			IndicesVector& iv = pbd.indicesVector;
			
			Vector3D mean(0,0,0);
			for(int i=0;i<iv.size();i++)
			{
				mean = mean + Vector3D(pointVector[iv[i]]);
			}
			pbd.mean = mean*(1.00/(double)iv.size());
		}
		return 0;
	}

	//Get 6-Neighbours which share a face.
	vector<PBox > Get6Neighbours(PBox b)
	{
		PartitionMap::iterator iter;
		vector<PBox > nv;
		if((iter = partitionMap.find(PBox(b.x-1,b.y,b.z)))!=partitionMap.end())
			nv.push_back(iter->first);
		if((iter = partitionMap.find(PBox(b.x+1,b.y,b.z)))!=partitionMap.end())
			nv.push_back(iter->first);
		if((iter = partitionMap.find(PBox(b.x,b.y-1,b.z)))!=partitionMap.end())
			nv.push_back(iter->first);
		if((iter = partitionMap.find(PBox(b.x,b.y+1,b.z)))!=partitionMap.end())
			nv.push_back(iter->first);
		if((iter = partitionMap.find(PBox(b.x,b.y,b.z-1)))!=partitionMap.end())
			nv.push_back(iter->first);
		if((iter = partitionMap.find(PBox(b.x,b.y,b.z+1)))!=partitionMap.end())
			nv.push_back(iter->first);
			
		//Don't forget to return the vector.	
		return nv;
	}
	
	//Get 7-Neighbours which share a face including self.
	vector<PBox > Get7Neighbours(PBox b)
	{
		vector<PBox> nv = Get6Neighbours(b);
		
		//Add the self box too.
		if(BoxExists(b))
			nv.push_back(b);
		return nv;
	}
	
	//Get 27 Neighours. all including self
	vector<PBox > Get27Neighbours(PBox b)
	{
		return GetNLevelNeighbours(b,1);
	}
	
	///Get all neighours upto level n. If n==1 we have 27-neighours case.
	vector<PBox > GetNLevelNeighbours(PBox b,int levels = 1)
	{
		PartitionMap::iterator iter;
		vector<PBox > nv;
		
		levels = std::max(1,levels);
		
		for(int x=-levels;x<=levels;x++)
			for(int y=-levels;y<=levels;y++)
				for(int z=-levels;z<=levels;z++)
		{
			//if(1)
			if((b.x+x)>=0 && (b.y+y)>=0 && (b.z+z)>=0 
			&& (b.x+x)<=xCount && (b.y+y)<=yCount && (b.z+z)<=zCount)
			{
				if((iter = partitionMap.find(PBox(b.x+x,b.y+y,b.z+z)))!=partitionMap.end())
					nv.push_back(iter->first);
			}
		}
		
		//Don't forget to return the vector.	
		return nv;
	}
	
	///Get all neighours upto level n. If n==1 we have 27-neighours case.
	vector<PBox > GetNLevelNeighboursExceptSelf(PBox b,int levels = 1)
	{
		PartitionMap::iterator iter;
		vector<PBox > nv;
		
		levels = std::max(1,levels);
		
		for(int x=-levels;x<=levels;x++)
			for(int y=-levels;y<=levels;y++)
				for(int z=-levels;z<=levels;z++)
		{
			//if(1)
			if((b.x+x)>=0 && (b.y+y)>=0 && (b.z+z)>=0 
			&& (b.x+x)<=xCount && (b.y+y)<=yCount && (b.z+z)<=zCount)
			{
				if((iter = partitionMap.find(PBox(b.x+x,b.y+y,b.z+z)))!=partitionMap.end())
				{
					if((iter->first)==b)
						continue;
					
					nv.push_back(iter->first);
				}
			}
		}
		
		//Don't forget to return the vector.	
		return nv;
	}
	
	
	///Get all neighours between m and n levels.
	vector<PBox > GetM2NLevelNeighbours(PBox box,int m,int n)
	{
		vector<PBox > nv;
		
		vector<PBox> bigger = GetNLevelNeighbours(box,n);
		
		vector<PBox> smaller = GetNLevelNeighbours(box,m);
		
		PBoxSet smallSet(smaller.begin(),smaller.end());
		
		for(int i=0;i<bigger.size();i++)
		{
			if(smallSet.find(bigger[i])==smallSet.end())
				nv.push_back(bigger[i]);
		}
		
		//Don't forget to return the vector.	
		return nv;
	}
	
	///Get 27-neighours for all boxes, and combine them in a set.
	///Return the result in a vector.
	vector<PBox > Get27Neighbours(const vector<PBox >& b)
	{
		PBoxSet twoLevelSet;
		
		for(int i=0;i<b.size();i++)
		{
			vector<PBox > current = Get27Neighbours(b[i]);
			
			for(int j=0;j<current.size();j++)
				twoLevelSet.insert(current[j]);
		}
		return vector<PBox > (twoLevelSet.begin(),twoLevelSet.end());
	}
	
	//Check if a given box exists in a map or not.
	bool BoxExists(const PBox& b)
	{
		PartitionMap::iterator iter;
		return ((iter = partitionMap.find(b))!=partitionMap.end());
	}
	
	///Convert a box to the points it contains.
	PointVector Box2Points(const PBox& b)
	{
		PointVector pts;
		PartitionMap::iterator iter;
		if((iter = partitionMap.find(b))!=partitionMap.end())
			pts = pPointVector->Select(iter->second.indicesVector);
			
		return pts;
	}
	
	///Convert a set of given boxes to points they contain.
	PointVector Boxes2Points(const vector<PBox >& b)
	{
		PointVector pts;
		
		for(int i=0;i<b.size();i++)
			pts = pts + Box2Points(b[i]);
			
		return pts;
	}
		
	///Convert a box to indices of the points it contains.
	vector<int> Box2Indices(const PBox& b)
	{
		PartitionMap::iterator iter;
		if((iter = partitionMap.find(b))!=partitionMap.end())
		{
			return iter->second.indicesVector;
		}	
		else		
			return vector<int>();
	}
	
	///Convert a set of given boxes to indices of the points they contain.
	vector<int> Boxes2Indices(const vector<PBox >& b)
	{
		vector<int> pts;
		
		for(int i=0;i<b.size();i++)
		{
			vector<int> current = Box2Indices(b[i]);
			int oldSize = pts.size();
			pts.resize(pts.size()+current.size());
			copy(current.begin(),current.end(),pts.begin()+oldSize);
		}			
		return pts;
	}


	//Prints some statistics to console.
	void Print()
	{
		cerr<<"PointsSpacePartition of "<<pPointVector->size()<<" points\n";
		cerr<<"Box counts are: ("<<xCount<<", "<<yCount<<", "<<zCount<<")"<<endl;
		cerr<<"Box sizes are: ("<<1.00/xFactor<<", "<<1.0/yFactor<<", "<<1.0/zFactor<<")"<<endl;
		cerr<<"Map entries are: "<<partitionMap.size()<<endl;
		cerr<<"The data is "<<(1.00 - (double)partitionMap.size()/(double)(1.000*xCount*yCount*zCount))*100<<"% sparse"<<endl;
		cerr<<"Only "<<partitionMap.size()<<" boxes used instead of "<<xCount*yCount*zCount<<endl;
		
		
		PartitionMap::iterator iter;
		for(iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			//cerr<<"Number of 6-neighbours: "<<Get6Neighbours(iter->first).size()<<endl;
			//iter->first.Print();	
			//iter->second.Print();
			if(iter->second.indicesVector.size()<=0)
				cerr<<"!!!!!!!!!!!!!!!!  Invalid box of size: "<<iter->second.indicesVector.size()<<endl;
		}
		
	}
	
	///Check how many neighbours each box has. (The one which really exists in the map)
	void CheckNeighbours(int count)
	{
		//Check neighbour function.
		PartitionMap::iterator iter;
		int i = 0;
		for(iter=partitionMap.begin();iter!=partitionMap.end();iter++)
		{
			vector<PBox > hits ;
			
			hits = (count<=6)?Get6Neighbours(iter->first):Get27Neighbours(iter->first);
			cerr<<"Box "<<i++<<" has    "<<hits.size()<<"/"<<((count<=6)?6:27)<<"    neighbours\n";
		}
	}
	
	///Get the neighbours within a given threshold distance and write them to a file.
	void WriteFdn(FILE* pResultFile,double fdn_distance,int neighbour_boxes=3)
	{
		PointVector& laserPoints = *pPointVector;
		int minCount = laserPoints.size()*10;
		int maxCount = -laserPoints.size()*10;
		double avgCount = 0;
		for(int i=0;i<laserPoints.size();i++)
		{
			vector<PBox> boxes = GetNLevelNeighbours(Point2Box(laserPoints[i]));
			vector<int> indices = Boxes2Indices(boxes);
			vector<int> neighbours;
			//printf("Now %d with %d boxes %d indices\n",i,boxes.size(),indices.size());
			for(int j=0;j<indices.size();j++)
			{
				if((laserPoints[i]-laserPoints[indices[j]]).Length()<=fdn_distance)
					neighbours.push_back(indices[j]);
			}	
			//printf("%d: Found %d Selected %d\n",i,indices.size(),neighbours.size());		
			for(int j=0;j<neighbours.size();j++)
				fprintf(pResultFile, "%d  ",neighbours[j]);
			fprintf(pResultFile,"\n");
			minCount = min(minCount,(int)neighbours.size());
			maxCount = max(minCount,(int)neighbours.size());
			avgCount += (neighbours.size()/(double)laserPoints.size());
		}
		printf("\n min: %d max: %d avg: %f\n",minCount,maxCount,avgCount);
	}
	
	///Return minimum box size.
	double MinBoxSize()const
	{
		return min(1.00/xFactor,min(1.00/yFactor,1.00/zFactor));
	}
	
	///Return maximum box size.
	double MaxBoxSize()const
	{
		return max(1.00/xFactor,max(1.00/yFactor,1.00/zFactor));
	}
	
	
	///Get the neighbours within a given threshold distance.
	vector<int> GetFdn(PointType& queryPt,double fdn_distance,vector<double>* pDistances = NULL, int max_neighbour_boxes=-1)
	{
		PointVector& laserPoints = *pPointVector;
		
		//Get box count for checking.
		int levelCount = fdn_distance/MinBoxSize() + 1;
		
		if(max_neighbour_boxes>0)
			levelCount = min(levelCount,max_neighbour_boxes);
			
			
		vector<PBox> boxes = GetNLevelNeighbours(Point2Box(queryPt), levelCount);
		vector<int> indices = Boxes2Indices(boxes);
		vector<int> neighbours;
		
		if(pDistances)
		{
			pDistances->resize(0);
			pDistances->reserve(indices.size());
		}
		
		double dist;
		for(int j=0;j<indices.size();j++)
		{
			
			if((dist=(queryPt-laserPoints[indices[j]]).Length())<=fdn_distance)
			{
				neighbours.push_back(indices[j]);
				
				if(pDistances)
				{
					pDistances->push_back(dist);
				}
			}
		}	
		
		return neighbours;
	}
	
				
private:
	PointVector* pPointVector;
	PartitionMap partitionMap;
	T xCount,yCount,zCount;
	double xFactor,yFactor,zFactor;
	double xOffset,yOffset,zOffset;
};

#endif //___POINTS___SPACE_PARTITION__H__
