
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
#include "TextFileUtilities.h"
#include <string.h>
#include <iostream>
using namespace std;
//Utility function to read in a line and parse into tokens
vector<string> ReadTokensFromLine(FILE* pFile,const char* delimiters)
{
	vector<string> tokens;
	if(!pFile)
	{
		cerr<<"ReadTokensFromLine NULL pointer\n";
		return tokens;
	}
	
	//local variables.
	const int buffsize = 1024*20;
	char buff[buffsize];
	string title,value;
		
	//Read one line and parse it.
	if(fgets(buff,buffsize,pFile))
	{

	  char * pch;
	  pch = strtok (buff,delimiters);
	  while (pch != NULL)
	  {
		tokens.push_back(pch);
		pch = strtok (NULL, delimiters);
	  }
  
	}
	return tokens;
}

//A function for testing.
//Read in a file and prints the token on the screen.
void PrintTokens(const char* fileName)
{
	FILE* pFile = fopen(fileName,"rt");
	
	while(!feof(pFile))
	{
		vector<string> tokens = ReadTokensFromLine(pFile);
		for(int i=0;i<tokens.size();i++)
			cout<<tokens[i]<<"|";
		cout<<endl;
	}
	fclose(pFile);
}
