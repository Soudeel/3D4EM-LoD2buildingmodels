
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
*   File made : August 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Provides a way to search for neighbors in segments.
*
*--------------------------------------------------------------------*/
#include "LaserPoints.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"
#include "StlUtilities.h"
#include "VisualizationUtility.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
#include "VrmlUtility.h"
#include "KNNFinder.h"
#include "SegmentFinder.h"
#include "ProgressDisplay.h"
#include "ShMacro.h"
#include "Histo3D.h"
#include "FunctionTimer.h"
#include<stack>

///Default constructor.
SegmentFinder::SegmentFinder()
{
	pSegments = NULL;
	pLaserPoints = NULL;
}

///Destructor.
SegmentFinder::~SegmentFinder()
{

}

///Constructor with arguments.
SegmentFinder::SegmentFinder(LaserPoints& laserPoints, SegmentsVector& segments, int knn)
{
	Update(laserPoints, segments, knn);
}

///Update data structure again.
void SegmentFinder::Update(LaserPoints& laserPointsIn, SegmentsVector& segments, int kNN)
{
	//Update internal pointers.
	pSegments = &segments;
	pLaserPoints = &laserPointsIn;
	
	//Its possible that there are points in laserPoints which do not belong to a segment.
	//They should not influence our data structure.
	//To account for that make a temporary laser points and set all unused points to a very high value.
	LaserPoints laserPoints;
	laserPoints.resize(laserPointsIn.size());
	std::fill(laserPoints.begin(),laserPoints.end(),LaserPoint(DBL_MAX,DBL_MAX,DBL_MAX));
	
	for(int i=0;i<segments.size();i++)
		for(int j=0;j<segments[i].size();j++)
			laserPoints[segments[i][j]] = laserPointsIn[segments[i][j]];

	//Lets find the normals for each segment.
	segNormals.resize(segments.size());

	for(int i=0;i<segments.size();i++)
	{
		segNormals[i] = laserPoints.Select(segments[i]).Normal();		
	}

	//Keep a list of segment ids for all points.
	//Default id is -1, meaning its invalid.
	segIds.resize(laserPoints.size(),-1);

	for(int i=0;i<segments.size();i++)
	{
		for(int j=0;j<segments[i].size();j++)
		{
			if(segments[i][j]>=0 && segments[i][j]<segIds.size())
				segIds[segments[i][j]] = i;
			else
				{cerr<<"Error in index: "; SH}
		}
	}

	// Make a finder for KNN search.
	KNNFinder<LaserPoint> finder(laserPoints);

	///Resize internal search data.
	searchInfo.resize(segments.size());

	//Update the neighbourhood relations information.
	for(int i=0;i<segments.size();i++)
	{
		//Map of segment id, with the number of neighours we have for it.
		typedef std::map<int,int> NeighborSegMap;
		NeighborSegMap segMap;

		for(int j=0;j<segments[i].size();j++)
		{
			vector<int> indices = finder.FindIndices(laserPoints[segments[i][j]],kNN);
			for(int k=0;k<indices.size();k++)
			{
				int segId = segIds[indices[k]];

				if(segId>=0 && segId!=i)
					segMap[segId]++;
			}
		}	

		vector<std::pair<int,int> > vec = ToVector(segMap);

		//Now sort based on the second value in the pair. That is the number of shared points.
		sort(vec.begin(),vec.end(),PairSorterB<int,int>);

		//Put the sorted vector in the search structure.
		searchInfo[i] = vec;
	}
	//The data structure is now updated.
}

///Find the segment that is next to this segment.
///Query is the index of the segment. The result is the index of found segment.
int SegmentFinder::FindNeighbor(int query)
{
	int ans = -1;

	if(searchInfo[query].size())
	{
		ans = searchInfo[query][0].first;
	}

	return ans;
}

///Find All neighbours upto a given level.
vector<int> SegmentFinder::FindAllNeighbors(int query,int nLevel, bool includeSelf)
{
	set<int> result;

	std::stack<int> seeds;
	seeds.push(query);
	
	//For recursive search we have to keep track of visited nodes to avoid infinite recursion.
	set<int> used;
	
	int levels = 0;
    
	while(seeds.size())
	{
		std::stack<int> nextSeeds;
		///Collect neighbors.
		while(seeds.size())
		{
			query = seeds.top();seeds.pop();
			
			if(query>=0 && !used.count(query))		
			{
				used.insert(query);
				
				for(int i=0;i<searchInfo[query].size();i++)
				{
					int t = searchInfo[query][i].first;
					result.insert(t);
					
					if(!used.count(t))
						nextSeeds.push(t);
				}
			}
		}
		
		levels++;
		
		if(levels >= nLevel)
			break;
			
		seeds = nextSeeds;
	}
	
	vector<int> final(result.size());
	copy(result.begin(),result.end(),final.begin());
	
	if(includeSelf)
		final.push_back(query);
	
	return final;
}			

	
///Find more than one neighors. All must be connected.
vector<int> SegmentFinder::FindNeighbors(int query, int nCount, bool includeSelf )
{
	vector<int> result;

	//include self too.
	if(includeSelf)
		result.push_back(query);

	//For recursive search we have to keep track of visited nodes to avoid infinite recursion.
	set<int> used;

	while(query>=0 && searchInfo[query].size())
	{
		///Update the set. Don't visit this node again.
		used.insert(query);

		///Get the search info vector for this segment.
		SearchInfoVector& info = searchInfo[query];

		//No neighbors, can't do much to help.
		if(info.empty())
			break;

		//Iterate to find unvisited node.
		int ans = -1;
		for(int i=0;i<info.size();i++)
		{
			if(!used.count(info[i].first))
			{
				ans = info[i].first;
				break;
			}
		}

		//All nodes are visited, so must break.
		if(ans<0)
			break;

		//push the result on the final vector.
		result.push_back(ans);

		//Enough neighbors must return.
		if(result.size()>=nCount)
			break;

		//Next iteration.
		query = ans;
	}

	return result;
}			

///Find more than one neighors. All must be connected.
///Also apply the angle constraint, angle must be less than a certain threshold.
vector<int> SegmentFinder::FindConstrainedNeighbors(int query,double angleThresholdDegrees,bool includeSelf,int nCount)
{
	vector<int> result;

	//include self too.
	if(includeSelf)
		result.push_back(query);

	//For recursive search we have to keep track of visited nodes to avoid infinite recursion.
	set<int> used;

	double dotThreshold = cos(angleThresholdDegrees*M_PI/180.);
	Vector3D n1 = segNormals[query];

	while(query>=0 && searchInfo[query].size())
	{
		///Update the set. Don't visit this node again.
		used.insert(query);

		///Get the search info vector for this segment.
		SearchInfoVector& info = searchInfo[query];

		//No neighbors, can't do much to help.
		if(info.empty())
			break;

		//Iterate to find unvisited node.
		int ans = -1;
		for(int i=0;i<info.size();i++)
		{
			if(!used.count(info[i].first) && n1.DotProduct(segNormals[info[i].first])>dotThreshold)
			{
				ans = info[i].first;
				break;
			}
		}

		//All nodes are visited, so must break.
		if(ans<0)
			break;

		//push the result on the final vector.
		result.push_back(ans);

		//Enough neighbors must return.
		if(nCount>0 && result.size()>=nCount)
			break;

		//Next iteration.
		query = ans;
	}

	return result;
}

///Find more than one neighors. All must be connected.
///Select the neighbors which have the best angle with the current one.
vector<int> SegmentFinder::FindBestNeighbors(int query,int nCount,bool includeSelf )
{
	vector<int> result;

	//include self too.
	if(includeSelf)
		result.push_back(query);

	//For recursive search we have to keep track of visited nodes to avoid infinite recursion.
	set<int> used;

	while(query>=0 && searchInfo[query].size())
	{
		///Update the set. Don't visit this node again.
		used.insert(query);

		//Get the ref normal.
		Vector3D n1 = segNormals[query];

		///Get the search info vector for this segment.
		SearchInfoVector& info = searchInfo[query];

		//No neighbors, can't do much to help.
		if(info.empty())
			break;

		//Iterate to find unvisited node.
		double bestDot = -1;
		int ans = -1;

		for(int i=0;i<info.size();i++)
		{
			if(!used.count(info[i].first) && fabs(n1.DotProduct(segNormals[info[i].first]))>bestDot)
			{
				ans = info[i].first;
				bestDot = fabs(n1.DotProduct(segNormals[info[i].first]));
			}
		}

		//All nodes are visited, so must break.
		if(ans<0)
			break;

		//push the result on the final vector.
		result.push_back(ans);

		//Enough neighbors must return.
		if(nCount>0 && result.size()>=nCount)
			break;

		//Next iteration.
		query = ans;
	}

	return result;
}

///Find more than one neighors. All must be connected.
///Uses a combined criterion, consisting of count and angle.
vector<int> SegmentFinder::FindBestNeighborsB(int query,int nCount,bool includeSelf )
{
	vector<int> result;

	//include self too.
	if(includeSelf)
		result.push_back(query);

	//For recursive search we have to keep track of visited nodes to avoid infinite recursion.
	set<int> used;

	while(query>=0 && searchInfo[query].size())
	{
		///Update the set. Don't visit this node again.
		used.insert(query);

		//Get the ref normal.
		Vector3D n1 = segNormals[query];

		///Get the search info vector for this segment.
		SearchInfoVector& info = searchInfo[query];

		//No neighbors, can't do much to help.
		if(info.empty())
			break;

		//Iterate to find unvisited node.
		double bestDot = -1;
		int ans = -1;

		for(int i=0;i<info.size();i++)
		{
			if(!used.count(info[i].first))
			{
				double thisDot = info[i].second* (0.1+fabs(n1.DotProduct(segNormals[info[i].first])));
				
				if(thisDot>bestDot)
				{
					ans = info[i].first;
					bestDot = thisDot;
				}
			}
		}

		//All nodes are visited, so must break.
		if(ans<0)
			break;

		//push the result on the final vector.
		result.push_back(ans);

		//Enough neighbors must return.
		if(nCount>0 && result.size()>=nCount)
			break;

		//Next iteration.
		query = ans;
	}

	return result;
}



///Convert segment indices to vector<int> of point indices into laser points.
vector<int> SegmentFinder::ToIndices(const vector<int>& segIndices)const
{
	vector<int> final;

	for(int i=0;i<segIndices.size();i++)
		final = final + (*pSegments)[segIndices[i]];

	return final;
}

///Convert segment indices to laser points.
LaserPoints SegmentFinder::ToLaserPoints(const vector<int>& segIndices)const
{
	return (*pLaserPoints).Select(this->ToIndices(segIndices));		
}

///To Colored LaserPoints with color.
LaserPoints SegmentFinder::ToLaserPointsColored(const vector<int>& segIndices)const
{
	LaserPoints pts;

	for(int i=0;i<segIndices.size();i++)
		pts = pts + pLaserPoints->Select((*pSegments)[segIndices[i]]).SetReflectance(-1*(int)pts.size());

	return pts;
}

///Is the given seg id valid.
bool SegmentFinder::IsSegIdValid(int id)const
{
	if(pSegments && id>=0 && id < pSegments->size())
		return true;
	
	return false;
}
	

///Angle between two segments in degrees.
double SegmentFinder::Angle(int segId1, int segId2)const
{
	double angle = 90;
	if(IsSegIdValid(segId1) && IsSegIdValid(segId2))
	{
		angle = acos(fabs(segNormals[segId1].DotProduct(segNormals[segId2])));
		
		return angle*180./M_PI;
	}
	return angle;
}
	
