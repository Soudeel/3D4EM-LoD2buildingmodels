
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
*   File made : December 2003
*   Author    : Tahir Rabbani
*	Modified  :
* 	Purpose   : Utility functions for parsing text files.
*
*--------------------------------------------------------------------*/
#ifndef __PARSE_UTILITY__H__
#define __PARSE_UTILITY__H__
#include <string.h>
#include <map>
#include <vector>
#include <string>
#include<stdio.h>
#include<stdlib.h>
#include<fstream>
#include<strstream>

using namespace std;


typedef map<string,string> KeyValueMap;

//Utility function to parse a colon separated key-value pair file. 
//The result is returned as a map<string(key),string(value)>
//The components are separated by colon and each pair must be on a new line
int LoadKeyValueFile(string fileName,KeyValueMap& result);

//Prints the key value pairs to the stdout.
void PrintStringMap(const KeyValueMap& m);
void PrintKeyValueMap(const KeyValueMap& m);

//Parses given number of values from a string.
//returns the remaining string and parsed values.
vector<double> 
ParseNumbers(string str,int count = 1,string* pRemainingString=NULL);

//Parses a sigle value from a string.
//returns the remaining string and parsed values.
double
ParseNumber(string str,string* pRemainingString=NULL);

//Parses a string from a string having many delimited by token.
string
ParseString(string str,const char token=' ',string* pRemainingString=NULL);
//Parses a string from a string having many delimited by token.
vector<string>
ParseStrings(string str,int count = 1,const char token=' ',string* pRemainingString=NULL);

//Check whether a given string is constained by the map or not.
bool Contains(KeyValueMap& keyValueMap,string str);

#define SAVE_KV(stream,a) stream<<#a<<": "<<a<<endl;
#define LOAD_KV(KeyValueFile,a) if(KeyValueFile.Contains(#a)){std::strstream inout; inout<<KeyValueFile[#a]; inout>>a;}
///A class to encapsulate key-value file funcionality.
class KeyValueFile:public KeyValueMap
{
public:	
	///Default constructor.
	KeyValueFile(){}
	
	///From file name.
	KeyValueFile(string fileName)
	{
		Load(fileName);
	}
	
	///Check if a given token is contained.
	bool Contains(string str)
	{
		return ::Contains(GetMap(),str);	
	}
	
	///Convert to map.
	KeyValueMap& GetMap()
	{
		return *((KeyValueMap*)this);
	}
	
	///Load key value file.
	void Load(string fileName)
	{
		LoadKeyValueFile(fileName,GetMap());
	}
	
	///Save to file.
	void Save(string fileName)
	{
		ofstream file(fileName.c_str());
		
		KeyValueMap m = GetMap();
		KeyValueMap::const_iterator iter;
		for(iter=m.begin();iter!=m.end();iter++)
		{
			file<<iter->first<<": "<<iter->second<<endl;
		}
	}
	
	///Parse a double number.
	double DoubleVal(string key)
	{
		return ParseNumber((*this)[key]);
	}
	
	///Parse a double number.
	string StringVal(string key)
	{
		return ((*this)[key]);
	}
};


#endif //__PARSE_UTILITY__H__
