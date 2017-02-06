
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
*   Purpose   : A partition box for space partitioning and histogramming
*
*--------------------------------------------------------------------*/
#ifndef __PARTITION_BOX__h____
#define __PARTITION_BOX__h____
#include<iostream>
///This represents the key of our map for space partition. Represents one box.
template <class T >
class PartitionBox
{
public:
	///Constructor.
	PartitionBox(T ax=T(), T ay=T(), T az=T())
	:x(ax),y(ay),z(az)
	{
	}
	
	///From anything that supports [] operator from 0 to 2.
	template<class U>
	PartitionBox(const U& pt)
	:x(pt[0]),y(pt[1]),z(pt[2])
	{
	
	}
	
	void Print()const
	{
		cerr<<"PartitionBox: ("<<x<<", "<<y<<", "<<z<<") ";
	}
	
	std::ostream& Save(std::ostream& os)const
	{
		os<<"PartitionBox "<<x<<" "<<y<<" "<<z<<" ";
		return os;
	}
	
	std::istream& Load(std::istream& is)
	{
		std::string header;
		is>>header;
		
		if(header!=std::string("PartitionBox"))
		{
			cerr<<"ERROR: Header missing in stream for PartitionBox::Load\n";
			return is;
		}
		is >> x >> y >> z;
		return is;
	}
	
	T x,y,z;
};

///This class provides a compare function for keys in space-partition-map.
template <class T >
class PartitionBoxCompare
{
public:
	bool operator()(const PartitionBox<T>& a,const PartitionBox<T>& b)const
	{
		if(a.x<b.x)
			return true;
		else if(a.x>b.x)
			return false;
		else
		{
			if(a.y<b.y)
				return true;
			else if(a.y>b.y)
				return false;
			else
			{
				if(a.z<b.z)
					return true;
				else if(a.z>b.z)
					return false;
				else
					return false;
			}
		}
	}
};

template <class T >
class HashPartitionBox
{
public: 
	int operator()(const PartitionBox<T> &b) const
	{ 
		return int(((int)b.x) + (((int)b.y)<<10) + (((int)b.z)<<20));
		//return int(((int)b.x) + ((int)b.y)*1024.00 + ((int)b.z)*pow(1024,2));
	}
};

template <class T> 
class HashPartitionBoxEqual
{
public: 
	bool operator()(const PartitionBox<T>& s1, const PartitionBox<T>&  s2) const 
	{ 
		return s1.x == s2.x && s1.y == s2.y && s1.z==s2.z;
	}
};

#endif
