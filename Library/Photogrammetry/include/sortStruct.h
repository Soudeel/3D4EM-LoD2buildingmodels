
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



#ifndef _SORTSTRUCT_
#define _SORTSTRUCT_

//using namespace std;

template <class T>class sortStruct
{
public:
	sortStruct (T _item, int _index):item(_item), index(_index){}
	const T & Item() const {return item;}
	int   Index() const {return index;}
	bool operator == (const sortStruct & rhs) const 
		{return (item == rhs.Item())? true : false;}
	bool operator < (const sortStruct & rhs) const 
		{return (item < rhs.Item())? true : false;}
private:
	T   item;
	int index;
};

typedef sortStruct<double> sortDouble;

#endif //SORTSTRUCT_


