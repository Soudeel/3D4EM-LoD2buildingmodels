
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
*   File made : November 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A 1D histogramming class. Uses a map to save space.
*
*--------------------------------------------------------------------*/
#ifndef __HISTO__1d____h____
#define __HISTO__1d____h____
#include<map>
#include<vector>
using std::map;
using std::vector;
///Class for making mapped histograms.
template <class T>	
class Histo1D : public std::map<T,int>
{
public:
	typedef  typename std::map<T,int> HistoMap;
	
	
	Histo1D(double stepSize)
	:xScale(stepSize)
	{
		UpdateConstants();	
	}
	
	template <class U>
	Histo1D(const vector<U>& data,double stepSize)
	:xScale(stepSize)
	{
		UpdateConstants();
		Update(data);	
	}
	
	void UpdateConstants()
	{
		xFactor = 1.00/xScale;
	}
	
	T Value2Box(const double& val)const
	{
		return (T)(val*xFactor);
	}
	
	double Box2Value(const T& box)const
	{
		return box*xScale;
	}
	
	double Box2Val(int x)const
	{
		return x*xScale;
	}
	
	template<class U>
	void Add(const U& value)
	{
		(*this)[Value2Box(value)]++;
	}
	
	template<class U>
	void Add(const std::vector<U>& values)
	{
		for(int i=0;i<values.size();i++)
			(*this)[Value2Box(values[i])]++;
	}
	
	template<class U>
	void Remove(const U& value)
	{
		T box = Value2Box(value);
		(*this)[box]--;
		
		if((*this)[box]<=0)
			(*this).erase(box);
	}
	
	template<class U>
	void Remove(const std::vector<U>& values)
	{
		for(int i=0;i<values.size();i++)
			(*this)[Value2Box(values[i])]--;
	}
	
	template<class U>
	int Update(const std::vector<U>& values)
	{
		//Update the constants 
		UpdateConstants();
		
		//clear the map.
		this->clear();
		
		Add(values);
			
		return 0;
	}
	
	///Collect the counts.
	vector<T> CollectCounts()const
	{
		vector<T> counts;
		
		typename Histo1D::const_iterator iter = this->begin();
		
		for(;iter!=this->end();iter++)
			counts.push_back(iter->second);
			
		return counts;
	}
	
	///Get the total.
	int Total()const
	{
		int total = 0;
		typename Histo1D::const_iterator iter = this->begin();
		
		for(;iter!=this->end();iter++)
			total += (iter->second);
		
		return total;		
	}
	
	///Return the percentage value.
	double PercentValue(double percentage)const
	{
		int total = Total();
		int threshold = (int)(percentage/100.00*total);
		
		typename Histo1D::const_iterator iter = this->begin();
		
		int count = 0;
		for(;iter!=this->end();iter++)
		{
			count += (iter->second);	
			if(count>=threshold)
				return Box2Value(iter->first);
		}
		
		cerr<<"Histo1D::PercentValue failed for "<<percentage<<" percent\n";
	}
	
	//Prints some statistics to console.
	void Print()const
	{
		cerr<<"Histo1D: "<<this<<"\n";
		cerr<<"Scale is: ("<<xScale<<endl;
		cerr<<"Map entries are: "<<this->size()<<endl;
	}
private:
	double xScale, xFactor;
};



#endif //__HISTO__1d__h____
