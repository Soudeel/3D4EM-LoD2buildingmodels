
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
#ifndef _LASER_SEGMENTS_____H_
#define _LASER_SEGMENTS_____H_

#include <LaserPoints.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include "ProgressDisplay.h"
#include "VrmlUtility.h"

using namespace std;

class LaserSegments:public vector<vector<int> >
{
public:
	///Default constructor.
	LaserSegments(LaserPoints* pL=NULL);
		
	///Constructor with arguments.
	LaserSegments(LaserPoints *pL,const vector<vector<int> >& v);
			
	///Constructor only from vector of vectors
	LaserSegments(const vector<vector<int> >& v);
	
	///Equality operator for assigning vector<vector> to this object
	LaserSegments& operator = (const vector<vector<int> >& v);
	
	///Destructor.
	~LaserSegments();
	
	///Check if both objects are the same.
	bool operator == (const LaserSegments& s);
				
	///Save to a stream.
	ostream& Save(ostream& os);
	
	///Load from an input stream
	bool Load(istream& is);
	
	///Save to a given file.
	bool Save(string fileName);
	
	///Print sizes on the output stream.
	void PrintSizes(ostream& os);
	
	///Load from file.
	bool Load(string fileName);
	
	///Get laser points.
	LaserPoints* GetLaserPoints()const;
	
	///Set laser points.
	LaserPoints* SetLaserPoints(LaserPoints* pNewLaser);
	
	///Return a new progress display.
	

	
	
	///Save to vrml file.
	VrmlFile SaveVrml(string fileName,bool useReflectance=false)const;
	
	///Show vrml
	void ShowVrml(bool useReflectance=false)const;

	
	///Sort on size
	void Sort();
	
	///Filter based on size.
	void Filter(int minSize);
	void FilterOnSize(int minSize)
	{
		Filter(minSize);
	}
	
	///Filter so that each segment is subdivided into patches of fixed area.
	void FilterFixedArea(double radius, int kNN, bool showProgress);
	
	///Check for validity.
	bool IsValid() const;
	
	///make valid, for bounds etc.
	void MakeValid();
	
	///Get the point of a given segment.
	LaserPoints operator() (int index)const;
			
public:	
	///Keep a pointer to laser points.
	LaserPoints* pLaserPoints;
	
};

///Class for connected components found using kNN
class LaserConnectedSegments : public LaserSegments
{
public:
	///Constructor. If LaserPoints is valid also creates the segmentation.
	LaserConnectedSegments(LaserPoints* pts=NULL, int _kNN = 20, bool _showProgress=true)
	:LaserSegments(pts), kNN(_kNN), showProgress(_showProgress)
	{
		if(pLaserPoints)
		{
			DoSegmentation();
		}
	}
	
	///Find the segments. Also called from constructor if LaserPoints was valid.
	void DoSegmentation();
	
protected:
	int kNN;
	bool showProgress;
};	

///Class for planar components found using kNN
class LaserPlanarSegments : public LaserSegments
{
public:
	///Constructor. If LaserPoints is valid also creates the segmentation.
	LaserPlanarSegments(LaserPoints* pts=NULL, int _kNN = 20, 
	double _angleThresholdDegrees=10, double _distanceThreshold=1, 
	int _maxRegionCount = 1000000, int _regionMinSize = -1, bool _showProgress=true)
	: LaserSegments(pts), kNN(_kNN), showProgress(_showProgress),
	angleThresholdDegrees(_angleThresholdDegrees), distanceThreshold(_distanceThreshold),
	maxRegionCount(_maxRegionCount), regionMinSize(_regionMinSize)
	{
		regionMinSize = std::max(regionMinSize,2*kNN);
		if(pLaserPoints)
		{
			DoSegmentation();
		}
	}
	
	///Find the segments. Also called from constructor if LaserPoints was valid.
	void DoSegmentation();
	
protected:
	int kNN;
	double distanceThreshold;
	double angleThresholdDegrees;
	bool showProgress;
	int maxRegionCount;
	int regionMinSize;
};	

///Class for smooth regions found using kNN
//Calculates thresholds automatically from the histogram of residuals. The percentiles used for 
//these threshold calculations are user customizable.
class LaserSmoothSegments : public LaserSegments
{
public:
	///Constructor. If LaserPoints is valid also creates the segmentation.
	LaserSmoothSegments(LaserPoints* pts=NULL, int _kNN = 20, bool _showProgress=true, 
	double _angleThresholdDegrees=15, double _seed1=0.5, double _seed2=0.75, double _merge=1,
	int _maxRegionCount = 1000000, int _regionMinSize = -1)
	: LaserSegments(pts), kNN(_kNN), showProgress(_showProgress),
	angleThresholdDegrees(_angleThresholdDegrees), seedFactor1(_seed1),seedFactor2(_seed2),
	mergeFactor(_merge), maxRegionCount(_maxRegionCount), regionMinSize(_regionMinSize)
	{
		regionMinSize = std::max(regionMinSize,2*kNN);
		if(pLaserPoints)
		{
			DoSegmentation();
		}
	}
	
	///Find the segments. Also called from constructor if LaserPoints was valid.
	void DoSegmentation();
	
	///Some helper functions.
	template <class T,class U>
	bool ErasePoint(int index,vector<bool>& unusedVector,int& unusedCount,vector<T>& residuals,double maxRes, U& histo)
	{
		histo.Remove(residuals[index]);
		unusedVector[index] = false;
		residuals[index] = maxRes;
		unusedCount--;
		histo.Remove(residuals[index]);
		return true;
	}
	
	///Check if two normal vectors are similar.
	bool IsNormalSimilar(const Vector3D& n1,const Vector3D& n2,const double threshold);
		
protected:
	int kNN;
	double angleThresholdDegrees;
	double seedFactor1;
	double seedFactor2;
	double mergeFactor;
	bool showProgress;
	int maxRegionCount;
	int regionMinSize;
	
};	
	
///Class for planar components found using kNN
class LaserFixedAreaSegments : public LaserSegments
{
public:
	///Constructor. If LaserPoints is valid also creates the segmentation.
	LaserFixedAreaSegments(LaserPoints* pts=NULL, int _kNN = 20, 
	double _maxRadius = 1, bool _showProgress=true)
	: LaserSegments(pts), kNN(_kNN), showProgress(_showProgress), maxRadius(_maxRadius)
	{
		if(pLaserPoints)
		{
			DoSegmentation();
		}
	}
	
	///Find the segments. Also called from constructor if LaserPoints was valid.
	void DoSegmentation();
	
protected:
	int kNN;
	double maxRadius;
	bool showProgress;
};	
	
	


#endif //_LASER_SEGMENTS_____H_


