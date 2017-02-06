
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
*   File made : Sep 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : class for managing laser segments.
*
*--------------------------------------------------------------------*/
#include <LaserSegments.h>
#include <fstream>
#include <set>
#include <algorithm>
#include <cmath>
#include "ProgressDisplay.h"
#include "StlUtilities.h"
#include "KNNFinder.h"
#include "Histo1D.h"
#include "ProgressDisplay.h"
#include "VrmlUtility.h"
#include "GeneralUtility.h"

using std::fabs;
///Default constructor.
LaserSegments::LaserSegments(LaserPoints* pL):pLaserPoints(pL)
{

}

///Constructor with arguments.
LaserSegments::LaserSegments(LaserPoints *pL,const vector<vector<int> >& v)
:pLaserPoints(pL),vector<vector<int> >(v)
{

}

///Constructor only from vector of vectors
LaserSegments::LaserSegments(const vector<vector<int> >& v)
:vector<vector<int> >(v)
{

}

///Equality operator for assigning vector<vector> to this object
LaserSegments& LaserSegments::operator = (const vector<vector<int> >& v)
{
    if (this == &v) return *this;  // Check for self assignment
	this->resize(v.size());
	copy(v.begin(),v.end(),this->begin());
	return *this;
}

///Destructor.
LaserSegments::~LaserSegments()
{
	
}

///Check if both objects are the same.
bool LaserSegments::operator == (const LaserSegments& s)
{
	if(pLaserPoints == s.pLaserPoints && size()==s.size())
	{
		for(int i=0;i<size();i++)
			if(s[i].size()!=(*this)[i].size())
			{
				cerr<<i<<"-th segment this is "<<(*this)[i].size()
					<<" the other is "<<s[i].size()<<endl;
				return false;
			}
		return true;
	}
	return false;
}

///Save to a stream.
ostream& LaserSegments::Save(ostream& os)
{
	os<<"LaserSegments\n";
	os<<pLaserPoints->size()<<endl;
	os<<size()<<endl;

	for(int i=0;i<size();i++)
	{
		vector<int>& v = (*this)[i];

		os<<v.size()<<" ";
		for(int j=0;j<v.size();j++)
			os<<v[j]<<" ";
		os<<endl;		
	}
	return os;	
}

///Load from an input stream
bool LaserSegments::Load(istream& is)
{
	string buff;

	is>>buff;

	if(buff != "LaserSegments")
	{
		cerr<<"Laser Segments: File header is wrong\n";
		return true;
	}

	int count;
	is >> count;

	if(count != pLaserPoints->size())
	{
		cerr<<"Laser Segments: Laser points size mismatch\n";
		return true;
	}

	is >> count;
	resize(count);

	for(int i=0;i<size();i++)
	{
		vector<int>& v = (*this)[i];

		//Get the size of this vector.
		is >> count;

		v.resize(count);

		for(int i=0;i<v.size();i++)
			is >> v[i];			
	}
	return false;	
}


///Save to a given file.
bool LaserSegments::Save(string fileName)
{
	ofstream file(fileName.c_str());

	if(file.is_open())
	{
		this->Save(file);
		file.close();
		return false;
	}
	else
	{
		cerr<<"Failed to save LaserSegments in "<<fileName<<endl;
		return true;
	}
}

///Print sizes on the output stream.
void LaserSegments::PrintSizes(ostream& os)
{
	os<<"["<<size()<<"]->";
	for(int i=0;i<size();i++)
		os<<(*this)[i].size()<<" ";
	os<<endl;
}

///Load from file.
bool LaserSegments::Load(string fileName)
{
	ifstream file(fileName.c_str());

	if(file.is_open())
	{
		if(this->Load(file))
		{
			cerr<<"Error in reading LaserSegments from "<<fileName<<endl;
		}
	}
	else
	{
		cerr<<"Failed to open LaserSegments from "<<fileName<<endl;
		return true;
	}
}


///Save to vrml file.
VrmlFile LaserSegments::SaveVrml(string fileName,bool useReflectance)const
{
	VrmlFile file(fileName.c_str());
	file.Write(*pLaserPoints,*this,useReflectance);
	return file;
}


///Show vrml
void LaserSegments::ShowVrml(bool useReflectance)const
{
	static int count=0;count++;
	string buff;
	sprintf(buff,"LaserSegments_%d.wrl",count);
	SaveVrml(buff,useReflectance).Show();
}


///Sort on size
void LaserSegments::Sort()
{
	SortOnSize(*this);
}

///Filter based on size.
void LaserSegments::Filter(int minSize)
{
	(*this) = ::FilterOnSize(*((vector<vector<int> >*)this),minSize);
}


///Check for validity.
bool LaserSegments::IsValid() const
{
	if(!pLaserPoints)
		return false;
		
	for(int i=0;i<size();i++)
	{
		const vector<int>& v = (*this)[i];
		for(int j=0;j<v.size();j++)
		{
			if(v[j]<0 || v[j]>=pLaserPoints->size())
				return false;
		}
	}
	return true;
}

///make valid, for bounds etc.
void LaserSegments::MakeValid()
{
	if(!pLaserPoints)
		return ;
		
	for(int i=0;i<size();i++)
	{
		vector<int>& v = (*this)[i];
		vector<int> nv;
		nv.reserve(v.size());
		for(int j=0;j<v.size();j++)
		{
			if(v[j]<0 || v[j]>=pLaserPoints->size())
				continue;
			nv.push_back(v[j]);
		}
		v = nv;
	}
}

///Get laser points.
LaserPoints* LaserSegments::GetLaserPoints()const
{
	return pLaserPoints;
}

///Set laser points.
LaserPoints* LaserSegments::SetLaserPoints(LaserPoints* pNewLaser)
{
	LaserPoints* pOld = pLaserPoints;
	pLaserPoints = 	pNewLaser;
	return pOld;
}

///Get the point of a given segment.
LaserPoints LaserSegments::operator() (int index)const
{
	if(!pLaserPoints || index <0 || index>= size())
		return LaserPoints();
	return pLaserPoints->Select((*this)[index]);
}

///Filter so that each segment is subdivided into patches of fixed area.
void LaserSegments::FilterFixedArea(double maxRadius, int kNN, bool showProgress)
{
	const LaserPoints& laserPoints = *pLaserPoints; 
	vector<vector<int> > & segments = *this;
	
	LaserSegments totalSeg(pLaserPoints);
		
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(segments.size(),"Progress");

	for(int i=0;i<segments.size();i++)
	{		
		if(showProgress)
			progress.Step("FixedAreaFilter ",i);

		LaserPoints currPts = laserPoints.Select(segments[i]);
		LaserFixedAreaSegments currSegments(&currPts, kNN, maxRadius, false);
		
		for(int j=0;j<currSegments.size();j++)
			totalSeg.push_back(Select(segments[i],currSegments[j]));
	}
	SortOnSize(totalSeg);

	if(showProgress)
		progress.End("FixedAreaFilter ");

	*this = totalSeg;
}

///Find connected components
void LaserConnectedSegments::DoSegmentation()
{		
	vector<vector<int> >& components = *this;
	LaserPoints& laserPoints = *pLaserPoints;
	
	if(laserPoints.size()<2*kNN)
	{
		//Too few points, return one component with all points.
		components.push_back(FillSequence(0,laserPoints.size(),1));
		return ;
		
	}
	//Make a finder object for knn search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	set<int> unused;
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
	//We want to select best seeds. based on minimum distance to neighbour points.
	vector<double> distances = finder.FindDistance(laserPoints,kNN);
	
	//Initially all points are unused.
	for(int i=0;i<laserPoints.size();i++)
		unused.insert(i);
		
	components.reserve(4096);
	
	int count = 0;
	while(1)
	{
		if(unused.empty())
			break;
		
		vector<int> seeds;
		
		//We want to find seed of very good quality.
		int seed = MinIndex(distances);
				
		//Get the first seed.
		seeds.push_back(seed);
				
		//Grow regions from seeds.
		for(int i=0;i<seeds.size();i++)
		{
			int seed = seeds[i];
			
			//The region will at least contain this seed.
			unused.erase(seed); distances[seed] = DBL_MAX;
						
			//Find knn
			vector<int> indices = finder.FindIndices(laserPoints[seed],kNN);
			
			//show progress
//			if(showProgress)
//				progress.Step("Finding connected components: ",i, "  Region count: %d",components.size()+1);

			for(int j=0;j<indices.size();j++)
			{
				if(unused.count(indices[j]))
				{
					unused.erase(indices[j]);distances[seed] = DBL_MAX;
					seeds.push_back(indices[j]);
				}
			}
		}
		//Add this component to our list.
		components.push_back(seeds);
	}
	SortOnSize(components);
	
	//show progress
	if(showProgress)
		progress.End("Finding connected components: ");
}		

	
void LaserPlanarSegments::DoSegmentation()
{
	
	const LaserPoints& laserPoints = *pLaserPoints;
	LaserSegments& segmentation = *this;
	
	//Fit plane to region and get the reference normal from it.
	bool bFitPlaneToRegion = true;
	
	//Check the size of points, if too small return.
	if(laserPoints.size()<regionMinSize)
	{
		cerr<<"SegmentIntoPlanarRegions: too few points \n";
		return;
	}
	
	//Progress display related variables.
	bool bShowProgress= true;
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
			
	//Make a Finder for doing K-nearest neighbour search.
	KNNFinder<LaserPoint> finder(laserPoints);
		
	//Iterate over all laserPoints, finding normal for every point.
    	
	//Allocate vectors.
	vector<double> residuals(laserPoints.size());
	vector<Vector3D> normals(laserPoints.size());
	
	//In the first pass just calculate the normals and their residuals.
	for(int i=0;i<laserPoints.size();i++)
    {
		if(bShowProgress)
			progress.Step("Calculating normals: ",i);
				
		//Calculate the normal.
		normals[i] = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Normal(&residuals[i]);
		
		//Keep only half side of Gaussian sphere.
		if(normals[i][2]<0)
			normals[i] = normals[i]*(-1);
	}
	progress.End("Calculating normals ");
		
	//Calculate min, max and range for residuals.
	double min_residual = *(min_element(residuals.begin(),residuals.end()));
	double max_residual = *(max_element(residuals.begin(),residuals.end()));
	double range_residual = (max_residual-min_residual);
	range_residual = (range_residual>1e-10)?range_residual:1;
	
	//Make a histogram of residuals.
	const double resMapFactor = std::min(1e4/range_residual,1e4);
	typedef std::map<int,std::set<int> > ResMap;
	ResMap resMap;
	
	for(int i=0;i<residuals.size();i++)
	{
		resMap[(int)(residuals[i]*resMapFactor)].insert(i);
	}
	
	cerr<<"Created resMap with "<<resMap.size()<<" entries\n"; //HIT_ENTER();
	
	//Allocate temporary sets.
	int unUsedPointCount = laserPoints.size();
	double angleThreshold = cos(angleThresholdDegrees*M_PI/180.00);
	segmentation.reserve(min(1024*20,maxRegionCount));
	
	const int reservedSize = laserPoints.size()/20 + 1;
	vector<int> currentRegion; currentRegion.reserve(reservedSize);
	vector<double> currentRegionResiduals; currentRegionResiduals.reserve(reservedSize);
	vector<int> potentialSeedPoints; potentialSeedPoints.reserve(reservedSize);
	
	//cerr<<"distanceThreshold: "<<distanceThreshold<<endl;
	//cerr<<"angleThreshold: "<<angleThreshold<<endl;
	//cerr<<"DBL_MAX: "<<DBL_MAX<<endl;
	//cerr<<"max_residual: "<<max_residual<<endl;
	//cerr<<"min_residual: "<<min_residual<<endl;

#define IS_UNUSED(index) (residuals[index] < (DBL_MAX))
#define ERASE(index) {if(IS_UNUSED(index)){int localIndex = (int)(residuals[index]*resMapFactor); resMap[localIndex].erase(index); if(resMap[localIndex].empty()) resMap.erase(localIndex); residuals[index]= DBL_MAX; unUsedPointCount--; }}

	//Make a map based histogram of the residuals
    
    int iterationCounter = 0;	
	//Start Region Growing.
	while(1)
	{
		if(unUsedPointCount == 0)			
			break;	
			
		if(segmentation.size()>maxRegionCount)
			break;
			
		//Select the point with minimum residual as the next seed.
		//This step takes the maximum time in the full segmentation loop, especially if 
		//there are lots of small regions, then after starting each new segment we need to
		//walk through the whole vector. Can be excruciatingly slow. 
		//A histogram based solution can be much faster and thats what we try to implement through a map.
		
#if 1		
		int currentIndex = 0;		
		if(!resMap.empty() && !(resMap.begin()->second.empty()))
		{
			set<int>& topSet = resMap.begin()->second;
			
			double minVal = DBL_MAX;
			for(set<int>::iterator iter = topSet.begin();iter!=topSet.end();iter++)
			{
				if(residuals[*iter]<minVal)
				{
					currentIndex = *iter;
					minVal = residuals[currentIndex];
				}
			}
			
			if(!IS_UNUSED(currentIndex))
			{
				cerr<<"Current index cannot be used, serious error\n";
			}
		}
		else
			break;
#endif		
				
		//We have found the new main seed, so continue from there.
				
		//clear scratch pads.
		currentRegion.clear();
		currentRegionResiduals.clear();
		potentialSeedPoints.clear();
		
		double acceptableResidualSeed = DBL_MAX;
		double acceptableResidualAdd = DBL_MAX;
		    	
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		currentRegion.push_back(currentIndex);
		ERASE(currentIndex);
		
		//temporary locals.
		Vector3D planeNormal;
		double planeRho=0;
		int lastUpdate = 0;
				
		for(int i=0;i<potentialSeedPoints.size();i++)
		{
			currentIndex = potentialSeedPoints[i];
			
			//Update plane equation if we have sufficient data.
			int updateInterval = std::max(lastUpdate/40,kNN);
			//updateInterval = kNN;
			if(currentRegion.size()>kNN)
			{
				if((currentRegion.size() - lastUpdate)>=updateInterval)
				{
					if(bFitPlaneToRegion)
					{
						double planeResidual;
						planeNormal = laserPoints.Select(currentRegion).Normal(&planeResidual,&planeRho);
					}
					else
					{
						planeNormal = normals[currentIndex];
						planeRho = planeNormal.DotProduct(laserPoints.Select(currentRegion).Mean());
						
						//Update acceptable residual limit.
						double mx = *(max_element(currentRegionResiduals.begin(),currentRegionResiduals.end()));
						double mn = *(min_element(currentRegionResiduals.begin(),currentRegionResiduals.end()));
												
						//based on std and mean.
						double mean = Mean(currentRegionResiduals);
						double std = StdDev(currentRegionResiduals);
						
						acceptableResidualSeed = mean + 0.1*std;
						acceptableResidualAdd = mean + 0.5*std;
						
						//cerr<<"size: "<<currentRegionResiduals.size()<<" Add: "<<acceptableResidualAdd<<"  seed: "<<acceptableResidualSeed
						//		<<" max global: "<<max_residual<<" min global: "<<min_residual<<"\n";
					}
						
					lastUpdate = currentRegion.size();
				}
				
				//If we are not fitting a plane update the normal for each point.
				if(!bFitPlaneToRegion)
				{
					planeNormal = normals[currentIndex];
				}
			}
			else
			{
				planeNormal = normals[currentIndex];
				planeRho = planeNormal.DotProduct(laserPoints.Select(currentRegion).Mean());
			}
						
			vector<int> neighbours = finder.FindIndices(laserPoints[currentIndex],kNN);
			for(int j=0;j<neighbours.size();j++)
			{
				if(IS_UNUSED(neighbours[j]))
				{
					double distance = fabs(planeNormal.DotProduct(laserPoints[neighbours[j]])-planeRho);
					double angle = fabs(planeNormal.DotProduct(normals[neighbours[j]]));
					
					if(distance<distanceThreshold  &&  angle>angleThreshold)
					{
						//Comment it out in the final version.
						//Just assigns reflectance to show progress of region growing.
					#if 0
						LaserPoints* pPts = (LaserPoints*)(&laserPoints);
						(*pPts)[neighbours[j]].Reflectance() = currentRegion.size();
						
						//Assign the residual to reflectance.
						//(*pPts)[neighbours[j]].Reflectance() = (residuals[neighbours[j]]-min_residual)/range_residual*1e6;
					#endif
						
						//Lets decide addition and seed selection.
						if(bFitPlaneToRegion)
						{
							//Add this point to region in progress.
							currentRegion.push_back(neighbours[j]);
							
							//If we are in a plane fitting mode, accept as parent always.
							potentialSeedPoints.push_back(neighbours[j]);
							
							//Erase this point from our list.
							ERASE(neighbours[j]);
						}
						else
						{
							//Add this point to region in progress.
							if(residuals[neighbours[j]]<acceptableResidualAdd)
							{
								currentRegion.push_back(neighbours[j]);
								
								//Do selection based on residual.
								if(residuals[neighbours[j]]<acceptableResidualSeed)
									potentialSeedPoints.push_back(neighbours[j]);
									
								currentRegionResiduals.push_back(neighbours[j]);
									
								//Erase this point from our list.
								ERASE(neighbours[j]);
							}
						}
						
					}
				}
			}
//			if(bShowProgress)
//				progress.Step("Growing regions: ",(laserPoints.size()-unUsedPointCount), " Region count: %d",segmentation.size()+1);
		}
		//This new region is finished.
		//Add current Region to global list.
		//Lets put a limit on the smallest size of the region.
		if(currentRegion.size()>=regionMinSize)
			segmentation.push_back(currentRegion);
			
		///From the resMap remove the entries with 0 points.
	}
	//sort the resultant segments with respect to size.
	SortOnSize(segmentation);
	progress.End("Growing regions ");
		
	//Just chekc the difference in sizes.
	int segmentedCount = 0;
	for(int i=0;i<segmentation.size();i++)
		segmentedCount += segmentation[i].size();
	cerr<<"Segmentation size "<<segmentedCount<<"/"<<laserPoints.size()<<" with "<<segmentation.size()<<" regions"<<endl;
}


void LaserFixedAreaSegments::DoSegmentation()
{
	LaserSegments& components = *this;
	LaserPoints& laserPoints = *pLaserPoints;
	
	if(laserPoints.size()<2*kNN)
	{
		//Too few points, return one component with all points.
		components.push_back(FillSequence(0,laserPoints.size(),1));
		return ;
		
	}
	//Make a finder object for knn search.
	KNNFinder<LaserPoint> finder(laserPoints);
	
	set<int> unused;
	
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Progress");
	
	//We want to select best seeds. based on minimum distance to neighbour points.
	vector<double> distances = finder.FindDistance(laserPoints,kNN);
	
	//Initially all points are unused.
	for(int i=0;i<laserPoints.size();i++)
		unused.insert(i);
		
	components.reserve(4096);
	
	int count = 0;
	while(1)
	{
		if(unused.empty())
			break;
		
		vector<int> seeds;
		
		//We want to find seed of very good quality.
		int seed = MinIndex(distances);
		LaserPoints currentPoints;
		currentPoints.push_back(laserPoints[seed]);
				
		//Get the first seed.
		seeds.push_back(seed);
				
		//Grow regions from seeds.
		for(int i=0;i<seeds.size();i++)
		{
			int seed = seeds[i];
			
			//The region will at least contain this seed.
			unused.erase(seed); distances[seed] = DBL_MAX;
						
			//Find knn
			vector<int> indices = finder.FindIndices(laserPoints[seed],kNN);
			
			//show progress
			if(showProgress)
				progress.Step("Finding connected components: ",i);

			for(int j=0;j<indices.size();j++)
			{
				if(unused.count(indices[j]))
				{
					unused.erase(indices[j]);distances[seed] = DBL_MAX;
					seeds.push_back(indices[j]);
					currentPoints.push_back(laserPoints[indices[j]]);
				}
			}
			
			//Apply the Area constraint
			DataBoundsLaser bounds = currentPoints.DeriveDataBounds(0);
			double mxRange = max(bounds.XRange(),max(bounds.YRange(),bounds.ZRange()));
			
			if(mxRange>maxRadius)
				break;
			
		}
		//Add this component to our list.
		components.push_back(seeds);
	}
	//Sort based on size
	SortOnSize(components);
	
	if(showProgress)
		progress.End("Finding connected components: ");
}


///Find the segments. Also called from constructor if LaserPoints was valid.
void LaserSmoothSegments:: DoSegmentation()
{
	const bool assignProgressToReflectance = false;
	const double eps = 1e-12;
	const double normalThreshold = cos(angleThresholdDegrees*M_PI/180.00);
	LaserSmoothSegments& segments = (*this);
	LaserPoints& laserPoints = *pLaserPoints;
		
	//Check the size of points, if too small return.
	if(laserPoints.size()<2*kNN)
	{
		cerr<<"LaserSmoothSegments:: DoSegmentation(): too few points \n";
		return;
	}
	
	//We may search for next-level seeds in bigger neighbourhoods.
	//kNNFactor controls that.
	const int kNNFactor = 1;
	const int kNNRegionGrowing = kNNFactor*kNN;
	
	//Declare some local variables.
    int i,j,k,l, pointCount = laserPoints.size();

	KNNFinder<LaserPoint> finder(laserPoints);

	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(laserPoints.size(),"Normals");
	
	double chiSquare;
	
	//In the first pass just calculate the normals and their residuals.
	vector<double> residuals(laserPoints.size());
	vector<Vector3D> normals(laserPoints.size());
	for(int i=0;i<laserPoints.size();i++)
	{
		double temp;
		normals[i] = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Normal(&temp);
		residuals[i] = temp;
		progress.Step("Calculating normals",i);
	}
	progress.End();
	progress.Reset("Region growing");

	//Get max Residual.
	double maxRes = Max(residuals);
	
	//Apply a sort filter to residuals. Just calculate the threshold in this step.
	//Apply threshold while processing.
	double thresholdSeed1, thresholdSeed2, thresholdMerge;
	
	//update threshold through histogramming.
	int lastThresholdUpdate = pointCount;

	//Making histogram for seed calculation and updating the seeds.
	Histo1D<int> histogram(residuals,Range(residuals)/(1024*20));
	thresholdSeed1 = histogram.PercentValue(seedFactor1*100);
	thresholdSeed2 = histogram.PercentValue(seedFactor2*100);
	thresholdMerge = histogram.PercentValue(mergeFactor*100);
 	
	//Allocate temporary sets.
	typedef std::set<int> IndicesSet;
	typedef IndicesSet::iterator IndicesSetIterator;
    IndicesSet currentRegion;
    vector<int> potentialSeedPoints;
    vector<bool> pointStateVector(pointCount,true);
    IndicesSet resultPoints;
    int currentIndex = 0;
    int unUsedPointCount = pointCount;
    int counter = 0;
	
	segments.reserve(4096);
    
    int iterationCounter = 0;	
	progress.ShowMessage("Starting segmentation loop...");
	progress.Initialize(laserPoints.size(),"Growing regions...");
	
	//Start Region Growing.
	while(1)
	{
		if(unUsedPointCount == 0)			
			break;	
			
		if(unUsedPointCount<kNNRegionGrowing)
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
		
		//clear scratch pads.
		currentRegion.clear();
		potentialSeedPoints.clear();
		
		//Add current seed to potentialSeedPoints.
		potentialSeedPoints.push_back(currentIndex);
		
		//current Region will contain at least the current seed.
		currentRegion.insert(currentIndex);
		if(assignProgressToReflectance)
			(*((LaserPoints*)(&laserPoints)))[currentIndex].Reflectance() = currentRegion.size();
		
		//Erase this point, we don't need it any more.
		ErasePoint(currentIndex,pointStateVector,unUsedPointCount,residuals,maxRes,histogram);
		
		for(i=0;i<potentialSeedPoints.size();i++)
		{
			currentIndex = potentialSeedPoints[i];
			progress.Step("Growing regions ",pointCount - unUsedPointCount);

			vector<int> indices = finder.FindIndices(laserPoints[currentIndex],kNNRegionGrowing);

			for(j=0;j<kNNRegionGrowing;j++)
			{
				if(pointStateVector[indices[j]])
				{
					bool isSimilar = IsNormalSimilar(normals[currentIndex],normals[indices[j]],normalThreshold);
									
					if(residuals[indices[j]]<(thresholdMerge+eps)
						&& isSimilar)
					{
						if(pointStateVector[indices[j]])
						{
							currentRegion.insert(indices[j]);
							if(assignProgressToReflectance)
								(*((LaserPoints*)(&laserPoints)))[indices[j]].Reflectance() = currentRegion.size();
	
							if(residuals[indices[j]]<(thresholdSeed2+eps))
							{
								potentialSeedPoints.push_back(indices[j]);
							}
							ErasePoint(indices[j],pointStateVector,unUsedPointCount,residuals,maxRes,histogram);
						}
					}
				}
			}					
		}
		
		//This new region is finished.
		//Add current Region to global list.
		if(currentRegion.size() > kNNRegionGrowing)
		{
			vector<int> temp;
			temp.resize(currentRegion.size());
			copy(currentRegion.begin(),currentRegion.end(),temp.begin());
			temp.reserve(currentRegion.size());
			segments.push_back(temp);
			
			progress.Step("Growing regions ",pointCount - unUsedPointCount);
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
	}
	SortOnSize(segments);
	 
	progress.End();
	
	//Just check the difference in sizes.
	int segmentedCount = 0;
	for(int i=0;i<segments.size();i++)
		segmentedCount += segments[i].size();
		
	char buffer[4096];
	sprintf(buffer,"LaserSmoothSegments:: DoSegmentation() finished \n"
		"Segmentatation has %d points, %d regions out of total %d points",segmentedCount,segments.size(),laserPoints.size());
	progress.ShowMessage("LaserSmoothSegments:: DoSegmentation() finished");
	progress.ShowMessage(buffer);
}

///Check if two normal vectors are similar.
bool LaserSmoothSegments::IsNormalSimilar(const Vector3D& n1,const Vector3D& n2,const double threshold)
{
	if(fabs(n1.DotProduct(n2))>threshold)
	{
		return true;
	}
	return false;
}
