
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
*   File made : Feb 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Some Utilities for reading and parsing the text files.
*
*--------------------------------------------------------------------*/
#ifndef _TEXT_FILE_UTILITIES__H_
#define _TEXT_FILE_UTILITIES__H_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <vector>



//Utility function to read in a line and parse into tokens
std::vector<std::string> ReadTokensFromLine(FILE* pFile,const char* delimiters = " \t\n,;:|");


//A function for testing.
//Read in a file and prints the token on the screen.
void PrintTokens(const char* fileName);

//Convert a container of strings to doubles.
template<class T>
std::vector<double> ConvertToDouble(T first,T last)
{
	std::vector<double> result;
	for(T a=first;a!=last;a++)
		result.push_back(atof(a->c_str()));
	return result;
}

//Convert a container of string to Ints.
template<class T>
std::vector<int> ConvertToInt(T first,T last)
{
	std::vector<int> result;
	for(T a=first;a!=last;a++)
		result.push_back(atoi(a->c_str()));
	return result;
}


#endif // _TEXT_FILE_UTILITIES__H_
