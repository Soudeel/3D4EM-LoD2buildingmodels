
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
*   File made : March 2005
*   Author    : Tahir Rabbani
*	Modified  :
* 	Purpose   : Utilities for working with STL
*--------------------------------------------------------------------*/
#ifndef __STL__UTILITIES_H__
#define __STL__UTILITIES_H__
#include<vector>
#include<set>
#include<map>
#include <iostream>

using std::map;
using std::vector;
using std::set;

///Select element of vector at given indices.
template <class U,class T>
std::vector<T> Select(const std::vector<T>& container,const std::vector<U>& indices)
{
	vector<T> selection;

	for(int i=0;i<indices.size();i++)
	{
		if(indices[i]>=0 && indices[i]<container.size())
			selection.push_back(container[indices[i]]);
	}
	return selection;
}

///Select a range of indices.
template<class T>
std::vector<T> Select(const std::vector<T>& v,int start,int end, int step=1)
{
	start = std::max(0,start);
	end = std::min(end,(int)v.size());

	vector<T> sel;	
	for(int i=start; i<end; i+=step)
		sel.push_back(v[i]);
		
	return sel;
}

///Select element of vector other than at given indices.
template <class U,class T>
std::vector<T> SelectInverse(const std::vector<T>& container,const std::vector<U>& indices)
{
	vector<T> selection;
	
	std::set<U> selset(indices.begin(),indices.end());

	for(int i=0;i<container.size();i++)
	{
		if(!selset.count(i))
			selection.push_back(container[i]);
	}
	return selection;
}

///call delete for all elements of a vector.
template <class T>
void Delete(const std::vector<T*>& container)
{
	for(int i=0;i<container.size();i++)
	{
		delete container[i];
	}
}


///Append two vectors together.
template <class T,class U>
std::vector<T> operator + (const std::vector<T>& v1, const std::vector<U>& v2)
{
	vector<T> ans = v1;
	ans.reserve(v1.size()+v2.size());
	
	for(int i=0;i<v2.size();i++)
		ans.push_back(v2[i]);
		
	return ans;
}

///Return sum of all elements of a given vector, or its range.
template<class T>
double Sum(T begin,T end)
{
	double sum = 0;
	for(T t=begin;t!=end;t++)
		sum += (*t);
	return sum;
}

template<class T>
double Sum(const vector<T>& v)
{
	return Sum(v.begin(),v.end());
}

///Return sum of squares of all elements of a given vector, or its range.
template<class T>
double SumSquare(T begin,T end)
{
	double sum = 0;
	for(T t=begin;t!=end;t++)
		sum += (pow(*t,2));
	return sum;
}

///Return sum of squares of all elements of a given vector, or its range.
template<class T>
double SumSquare(const vector<T>& v)
{
	return SumSquare(v.begin(),v.end());
}

///Return rms
template<class T>
double RMS(T begin,T end)
{
	if(end<=begin)
		return 0;
	
	return sqrt(SumSquare(begin,end)/(double)(end-begin));
}

///Return rms
template<class T>
double RMS(const vector<T>& v)
{
	return RMS(v.begin(),v.end());
}

///Return median of a vector
template<class T>
T Median(const vector<T>& v)
{
	vector<T> temp = v;
	sort(temp.begin(),temp.end());
	
	return temp[temp.size()/2];
}

///Return mean using iterator range.
template<class T>
double Mean(T begin,T end)
{
	double sum = 0;
	int count = 0;
	for(T t=begin;t!=end;t++)
	{
		sum += (*t); count++;
	}
	return sum/count;
}

///Mean of a vector.
template<class T>
double Mean(const vector<T>& v)
{
	return Mean(v.begin(),v.end());
}

///Minimum value of a vector..
template<class T>
T Min(const vector<T>& v)
{
	return *(min_element(v.begin(),v.end()));
}

///Maximum value of a vector..
template<class T>
T Max(const vector<T>& v)
{
	return *(max_element(v.begin(),v.end()));
}

///Index of Maximum value of a vector..
template<class T>
int MaxIndex(const vector<T>& v)
{
	return (max_element(v.begin(),v.end()) - v.begin());
}

///Index of Min value of a vector..
template<class T>
int MinIndex(const vector<T>& v)
{
	return (min_element(v.begin(),v.end()) - v.begin());
}

///Range value of a vector..
template<class T>
T Range(const vector<T>& v)
{
	return Max(v)-Min(v);
}

///Return differnce of all elements.
template<class T>
std::vector<T> Diff(const std::vector<T>& a,const std::vector<T>& b)
{
	vector<T> ans(min(a.size(),b.size()));
	
	for(int i=0;i<ans.size();i++)
		ans[i] = a[i] - b[i];
	return ans;
}

///Return Abs value of all elements of a vector.
template <class T>
std::vector<T> Abs(const std::vector<T>& a)
{
	vector<T> ans(a.size());
	
	for(int i=0;i<ans.size();i++)
		ans[i] = fabs(a[i]);
	
	return ans;
}

///Return standard deviation of all elements of a vector.
template <class T>
double StdDev(const std::vector<T>& a)
{
	vector<T> cp = a;
	double mean = Mean(cp);
	
	double ans = 0;
	
	for(int i=0;i<cp.size();i++)
		ans += pow(cp[i]-mean,2);
		
	return sqrt(ans/(double)(cp.size()));
}

///Return Abs value of all elements of a vector.
template <class T>
std::vector<T> SubSample(const std::vector<T>& a,int skipInterval)
{
	vector<T> ans;
	ans.reserve(a.size()/skipInterval);
	
	for(int i=0;i<a.size();i+=skipInterval)
		ans.push_back(a[i]);
	
	return ans;
}

///Write the contents of a container to a file.
template <class T>
void WriteToFile(std::string fileName,T t1,T t2)
{
	std::ofstream file(fileName.c_str());
	
	for(T t=t1;t!=t2;t++)
		file<<(*t)<<"  ";
	file<<endl;
}

///copy from src to dest.
template <class U,class T>
void Copy(const std::vector<T>& src,std::vector<U>& dest)
{
	dest.resize(src.size());
	for(int i=0;i<src.size();i++)
	{
		dest[i] = (U)(src[i]);
	}
}
///Filter a container based on size.
template<class T>
vector<vector<T> > FilterOnSize(const vector<vector<T> >& src,int minSize)
{
	vector<vector<T> > filtered;
	filtered.reserve(src.size());
	
	for(int i=0;i<src.size();i++)
	{
		if(src[i].size()>minSize)
			filtered.push_back(src[i]);
	}
	return filtered;
}

///Find indices of elements of a vector which are greater than a given value.
template <class T,class U>
vector<int> FindG(const std::vector<T>& src,const U& value)
{
	vector<int> sel;
	for(int i=0;i<src.size();i++)
	{
		if(src[i]>value)
			sel.push_back(i);
	}
	return sel;
}

///Find indices of elements of a vector which are less than a given value.
template <class T,class U>
vector<int> FindL(const std::vector<T>& src,const U& value)
{
	vector<int> sel;
	for(int i=0;i<src.size();i++)
	{
		if(src[i]<value)
			sel.push_back(i);
	}
	return sel;
}

///Find indices of elements of a vector which are less than or equal to a given value.
template <class T,class U>
vector<int> FindLE(const std::vector<T>& src,const U& value)
{
	vector<int> sel;
	for(int i=0;i<src.size();i++)
	{
		if(src[i]<= value)
			sel.push_back(i);
	}
	return sel;
}

///Find indices of elements of a vector which are greater than or equal to a given value.
template <class T,class U>
vector<int> FindGE(const std::vector<T>& src,const U& value)
{
	vector<int> sel;
	for(int i=0;i<src.size();i++)
	{
		if(src[i]>= value)
			sel.push_back(i);
	}
	return sel;
}

///Scale with a factor.
template <class T,class U>
vector<T>& multiply(vector<T>& v,U factor)
{
	for(int i=0;i<v.size();i++)
			v[i] = factor*(v[i]);
	return v;
}

///Find indices of elements of a vector which are equal to a given value.
template <class T,class U>
vector<int> FindE(const std::vector<T>& src,const U& value)
{
	vector<int> sel;
	for(int i=0;i<src.size();i++)
	{
		if(src[i]==value)
			sel.push_back(i);
	}
	return sel;
}

///A class to sort based on indices.
template<class T>
bool VectorSizeSorter(const vector<T> &a, const vector<T> &b) 
{
    return a.size() > b.size();
}


///Sort based on size.
template<class T>
void SortOnSize(vector<vector<T> >& v)
{
	std::sort(v.begin(),v.end(),VectorSizeSorter<T>);
}

///Filter a vector of vectors, based on a vector of used T's.
template<class T>
void FilterOutUsed(vector<vector<T> >& seg, const vector<T>& used)
{
	std::set<T> usedSet(used.begin(),used.end());
	
	for(int i=0;i<seg.size();i++)
	{
	 	vector<T>& current = seg[i];
		
		vector<T> temp;
		temp.reserve(current.size());
		
		for(int j=0;j<current.size();j++)
		{
			if(usedSet.count(current[j]))
				usedSet.erase(current[j]);
			else
				temp.push_back(current[j]);
		}
		seg[i] = temp;
	}
	SortOnSize(seg);
		
}


///return total size of a vector of vector.
template<class T>
int TotalSize(const vector<vector<T> >& v)
{
	int total = 0;
	
	for(int j=0;j<v.size();j++)
		total += v[j].size();
		
	return total;
}

///Combine elements in one big vector.
template<class T>
vector<T> Combine(const vector<vector<T> >& v)
{
	vector<T> t;
	
	for(int i=0;i<v.size();i++)
		t = t + v[i];
	
	return t;
}
	
	

///Fill sequence.
template<class T,class U,class V>
vector<T> FillSequence(const T& a, const U& b, const V& inc)
{
	vector<T> vec;
	vec.reserve((b-a)/inc+1);
	
	for(T i=a;i<b;i+=inc)
		vec.push_back(i);
		
	return vec;
}

///A function to sort based on first value of pair
template<class T,class U>
bool PairSorterA(const std::pair<T,U> &a, const std::pair<T,U> &b) 
{
    return a.first > b.first;
}

///A function to sort based on 2nd value of pair
template<class T,class U>
bool PairSorterB(const std::pair<T,U> &a, const std::pair<T,U> &b) 
{
    return a.second > b.second;
}


///Convert a map to vector.
template<class T, class U>
vector<std::pair<T,U> > ToVector(const std::map<T,U>& mp)
{
	std::vector<std::pair<T,U> > vec;
	typedef typename std::map<T,U>::const_iterator IterType;
	
	IterType iter;
	
	for(iter = mp.begin();iter!=mp.end();iter++)
		vec.push_back(*iter);
		
	return vec;
}

//Print a vector to screen or to file.
template<class T>
std::ostream& Print(const std::vector<T>& v,std::ostream& os=std::cerr)
{
	for(int i=0;i<v.size();i++)
		os<<i<<": "<<v[i]<<endl;
}

///Fill a vector with a sequence.
template<class T>
vector<T>& Fill(vector<T>& vec, T start,T step, T stop)
{
	vec.resize(0);
	
	for(T i=start;i<stop;i+=step)
	{
		vec.push_back(i);	
	}
	return vec;
}

///Append v2 to the end of v1.
template<class T>
std::vector<T>& Append(vector<T>& dest, const vector<T>& src)
{
	dest.reserve(src.size()+dest.size());
	for(int i=0;i<src.size();i++)
		dest.push_back(src[i]);
		
	return dest;
}

#endif//__STL_UTILITIES_H__
