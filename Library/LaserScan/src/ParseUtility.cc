
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
#include <iostream>
#include"ParseUtility.h"

//Utility function to parse a colon separated key-value pair file. 
//The result is returned as a map<string(key),string(value)>
//The components are separated by colon and each pair must be on a new line
int LoadKeyValueFile(string fileName,map<string,string>& result)
{
	FILE* pFile = fopen(fileName.c_str(),"rt");

	if(!pFile)
	{
		cerr<<"Failed to open "<<fileName<<endl;
		return 1;
	}
	
	//local variables.
	const int buffsize = 4096;
	char buff[buffsize];
	string title,value;
	int count = 0;
	
	//Keep reading and parsing the file, line by line.
	while(fgets(buff,buffsize,pFile))
	{
		count++;
		if(strlen(buff)>0 && buff[strlen(buff)-1]=='\n')
			buff[strlen(buff)-1]=NULL;
			
		string str = buff;
		int n = str.find(":");
		
		if(n!=string::npos)
		{
			title = str.substr(0,n);
			value = str.substr(n+2);
			result[title] = value;
		}
		else
		{
			cerr<<"skipping as no token found: "<<buff<<" line: "<<count<<" of file "<<fileName<<endl;
		}
	}
	fclose(pFile);
	return 0;
}

//Prints the key value pairs to the stdout.
void PrintStringMap(const map<string,string>& m)
{
	map<string,string>::const_iterator iter;
	for(iter=m.begin();iter!=m.end();iter++)
	{
		cerr<<iter->first<<": "<<iter->second<<endl;
	}
}

//Parses given number of values from a string.
//returns the remaining string and parsed values.
vector<double> 
ParseNumbers(string str,int count ,string* pRemainingString)
{
	const int buffSize = 4096;
	char buff[buffSize];
	
	snprintf(buff,buffSize,"%s",str.c_str());
	
	char* lastEnd = buff;
	char* end = NULL;
	double value;
	vector<double> results;
	for(int i=0;i<count;i++)
	{
		value = strtod(lastEnd,&end);
		
		if(end!=lastEnd)
		{
			lastEnd = end;
			results.push_back(value);
		}
	}
	
	if(pRemainingString)
	{
		*pRemainingString = string(lastEnd);
	}
	
	return results;			
}

//Parses single value from a string.
//returns the remaining string and parsed values.
double 
ParseNumber(string str,string* pRemainingString)
{
	const int buffSize = 4096;
	char buff[buffSize];
	
	snprintf(buff,buffSize,"%s",str.c_str());
	
	char* lastEnd = buff;
	char* end = NULL;
	double value = 0;
	value = strtod(lastEnd,&end);
		
	if(end!=lastEnd)
	{
		lastEnd = end;
	}
	
	if(pRemainingString)
	{
		*pRemainingString = string(lastEnd);
	}
	
	return value;			
}

//Parses a string from a string having many delimited by token.
string
ParseString(string str,const char token,string* pRemainingString)
{
	const int buffSize = 4096;
	char buff[buffSize];
	
	snprintf(buff,buffSize,"%s",str.c_str());
	
	char* lastEnd = buff;
	char* end = NULL;
	double value = 0;
	
	//remove tokens from the very start
	while(*lastEnd==token && *lastEnd)
		lastEnd++;
	
	end = lastEnd;
	//now find the other end.
	while(*end!=token && *end)
		end++;
	if(end!=lastEnd && *end)
	{
		*end = 0;
		end++;
	}
	
	if(pRemainingString)
		*pRemainingString = string(end);
		
	return string(lastEnd);
	
}
//Parses a string from a string having many delimited by token.
vector<string>
ParseStrings(string str,int count ,const char token,string* pRemainingString)
{
	
	vector<string> results;
	string remaining = str;
	for(int i=0;i<count;i++)
		results.push_back(ParseString(remaining,token,&remaining));
		
	return results;
}







void PrintKeyValueMap(const KeyValueMap& m)
{
	return PrintStringMap(m);
}

//Check whether a given string is constained by the map or not.
bool Contains(KeyValueMap& keyValueMap,string str)
{
	return (keyValueMap.find(str)!=keyValueMap.end());
}
