
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
#ifndef _SEGMENTS_FINDER___H___
#define _SEGMENTS_FINDER___H___

//class to find close segments, through proximity search.
class SegmentFinder		
{
public:	
	///Default constructor.
	SegmentFinder();
		
	///Destructor.
	~SegmentFinder();
		
	///Constructor with arguments.
	SegmentFinder(LaserPoints& laserPoints, SegmentsVector& segments, int knn);
	
	
	///Update data structure again.
	void Update(LaserPoints& laserPoints, SegmentsVector& segments, int kNN);
		
	///Find the segment that is next to this segment.
	///Query is the index of the segment. The result is the index of found segment.
	int FindNeighbor(int query);
	
	///Find more than one neighors. All must be connected.
	vector<int> FindNeighbors(int query,int nCount = 1, bool includeSelf = false);
	
	///Find All neighbours upto a given level.
	vector<int> FindAllNeighbors(int query,int nLevel = 1, bool includeSelf = false);
	
	///Find more than one neighors. All must be connected.
	///Also apply the angle constraint, angle must be less than a certain threshold.
	vector<int> FindConstrainedNeighbors(int query,double angleThresholdDegrees,bool includeSelf = false,int nCount=-1);
	
	///Find more than one neighors. All must be connected.
	///Select the neighbors which have the best angle with the current one.
	vector<int> FindBestNeighbors(int query,int nCount,bool includeSelf = false);
	
	///Find more than one neighors. All must be connected.
	///Uses a combined criterion, consisting of count and angle.
	vector<int> FindBestNeighborsB(int query,int nCount,bool includeSelf=false);

	
	///Convert segment indices to vector<int> of point indices into laser points.
	vector<int> ToIndices(const vector<int>& segIndices)const;
	
	///Convert segment indices to laser points.
	LaserPoints ToLaserPoints(const vector<int>& segIndices)const;
	
	///To Colored LaserPoints with color.
	LaserPoints ToLaserPointsColored(const vector<int>& segIndices)const;
	
	///Angle between two segments in degrees.
	double Angle(int segId1, int segId2)const;
	
	///Is the given seg id valid.
	bool IsSegIdValid(int id)const;
	
protected:
	///Normals of all segments. Used for angle based search.
	vector<Vector3D> segNormals;
	
	///Define the types for internal data structures.
	typedef vector<std::pair<int,int> >  SearchInfoVector;	 
	typedef vector<SearchInfoVector> SearchInfoVectors;

	///An instance of the data structure.
	SearchInfoVectors searchInfo;
	
	///A vector mapping point index to segment index.
	vector<int> segIds;

	///A pointer to externally managed segmentation.
	SegmentsVector* pSegments;
	
	///A pointer to externally managed laser points
	LaserPoints* pLaserPoints;					
};

#endif //_SEGMENTS_FINDER___H___
