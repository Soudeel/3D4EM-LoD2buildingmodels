
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
*   Purpose   : A 3D histogramming class. Uses a map to save space.
*
*--------------------------------------------------------------------*/
#ifndef __HISTO____3d__h____
#define __HISTO____3d__h____
#include"PartitionBox.h"
#include<map>
using std::map;
///Class for making mapped histograms.
template <class T>	
class Histo3D : public std::map<PartitionBox<T>,int,PartitionBoxCompare<T> >
{
public:
	//typedef hash_map<PartitionBox<T>,PartitionBoxData,HashPartitionBox<T>,HashPartitionBoxEqual<T> > PartitionMap;
	typedef  std::map<PartitionBox<T>,int,PartitionBoxCompare<T> > PartitionMap;
	
	Histo3D(double xC,double yC,double zC)
	:xScale(xC),yScale(yC),zScale(zC)
	{
		UpdateConstants();
	}
	
	Histo3D(double stepSize)
	:xScale(stepSize),yScale(stepSize),zScale(stepSize)
	{
		UpdateConstants();	
	}
	
	Histo3D(const LaserPoints& pts,double stepSize,bool useReflectance=false)
	:xScale(stepSize),yScale(stepSize),zScale(stepSize)
	{
		UpdateConstants();
		offset = pts.Mean();
		Update(pts,useReflectance);	
	}
	
	
	
	void UpdateConstants()
	{
		xFactor = 1.00/xScale;
		yFactor = 1.00/yScale;
		zFactor = 1.00/zScale;
	}
	
	PartitionBox<T> Point2Box(const LaserPoint& pta)const
	{
		Vector3D pt = pta - offset;
		return PartitionBox<T>((T)(pt.X()*xFactor),(T)(pt.Y()*yFactor),(T)(pt.Z()*zFactor));
	}
	
	PartitionBox<T> Point2Box(double x, double y, double z)const
	{
		return PartitionBox<T>((T)(x-offset[0])*xFactor,(T)(y-offset[1])*yFactor,(T)(z-offset[2])*zFactor);
	}
	
	LaserPoint Box2Point(const PartitionBox<T> b,int count=0)const
	{
		return LaserPoint(b.x/xFactor+offset[0],b.y/yFactor+offset[1],b.z/zFactor+offset[2],count);
	}
	
	int Update(const LaserPoints& laserPoints,bool useReflectance=false)
	{
		//Update the constants 
		UpdateConstants();
		
		//clear the map.
		this->clear();
		
		Add(laserPoints);
			
		return 0;
	}
	
	int Add(const LaserPoints& laserPoints)
	{
		for(int i=0;i<laserPoints.size();i++)
			Add(laserPoints[i]);
	}
		
	int Add(const LaserPoint& pt)
	{
		(*this)[Point2Box(pt)]++;
	}
	
	int Add(double x,double y, double z, int count=1)
	{
		(*this)[Point2Box(x,y,z)]+=count;
	}
	
	int Add(const Vector3D& v,int count=1)
	{
		(*this)[Point2Box(v[0],v[1],v[2])]+=count;
	}
	
	//Use Reflectance.
	int AddRef(const LaserPoint& pt)
	{
		(*this)[Point2Box(pt)]+=pt.Reflectance();
	}
	
	int AddRef(const LaserPoints& laserPoints)
	{
		for(int i=0;i<laserPoints.size();i++)
			AddRef(laserPoints[i]);
	}
	
	
	LaserPoints ToPoints()const
	{
		LaserPoints pts;
		
		typename Histo3D::const_iterator iter = this->begin();
		
		for(;iter!=this->end();iter++)
			pts.push_back(Box2Point(iter->first,iter->second));
			
		return pts;
	}
	
	///Collect the counts.
	std::vector<T> CollectCounts()const
	{
		std::vector<T> counts;
		
		typename Histo3D::const_iterator iter = this->begin();
		
		for(;iter!=this->end();iter++)
			counts.push_back(iter->second);
			
		return counts;
	}
	
	//Filter out the points whose count is less than given value
	LaserPoints ToPointsFiltered(int minCount=1)const
	{
		LaserPoints pts;
		
		typename Histo3D::const_iterator iter = this->begin();
		
		for(;iter!=this->end();iter++)
			if(iter->second>minCount)
				pts.push_back(Box2Point(iter->first,iter->second));
			
		return pts;
	}
		
	
	//Prints some statistics to console.
	void Print()const
	{
		cerr<<"Histo3D: "<<this<<"\n";
		cerr<<"Box scale are: ("<<xScale<<", "<<yScale<<", "<<zScale<<")"<<endl;
		cerr<<"Map entries are: "<<this->size()<<endl;
	}
private:
	double xScale,yScale,zScale;
	double xFactor,yFactor,zFactor;
	Vector3D offset;
};



#endif //__HISTO__3d__h____
