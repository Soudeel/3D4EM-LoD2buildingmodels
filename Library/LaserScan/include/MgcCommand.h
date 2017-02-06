
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



// Magic Software, Inc.
// http://www.magic-software.com
// Copyright (c) 2000-2002.  All Rights Reserved
//
// Source code from Magic Software is supplied under the terms of a license
// agreement and may not be copied or disclosed except in accordance with the
// terms of that agreement.  The various license agreements may be found at
// the Magic Software web site.  This file is subject to the license
//
// FREE SOURCE CODE
// http://www.magic-software.com/License/free.pdf

#ifndef MGCCOMMAND_H
#define MGCCOMMAND_H


#include <string>
using std::string;

class Command
{
public:
    Command (int iQuantity, char** apcArgument);
    Command (char* acCmdline);
    ~Command ();

    // return value is index of first excess argument
    int ExcessArguments ();

    // Set bounds for numerical arguments.  If bounds are required, they must
    // be set for each argument.
    Command& Min (double dValue);
    Command& Max (double dValue);
    Command& Inf (double dValue);
    Command& Sup (double dValue);

    // The return value of the following methods is the option index within
    // the argument array.

    // Use the boolean methods for options which take no argument, for
    // example in
    //           myprogram -debug -x 10 -y 20 filename
    // the option -debug has no argument.

    int Boolean (char* acName);  // returns existence of option
    int Boolean (char* acName, bool& rbValue);
    int Integer (char* acName, int& riValue);
    int Float (char* acName, float& rfValue);
    int Double (char* acName, double& rdValue);
    int String (char* acName, char*& racValue);
	int String(char* acName,string & racValue);
				
	int Filename (char*& racName);
	
	int Print()const;
		
	int Filename (std::string& fileName);
	
    // last error reporting
    const char* GetLastError ();

protected:
    // constructor support
    void Initialize ();

    // command line information
    int m_iQuantity;       // number of arguments
    char** m_apcArgument;  // argument list (array of strings)
    char* m_acCmdline;     // argument list (single string)
    bool* m_abUsed;        // keeps track of arguments already processed

    // parameters for bounds checking
    double m_dSmall;   // lower bound for numerical argument (min or inf)
    double m_dLarge;   // upper bound for numerical argument (max or sup)
    bool m_bMinSet;    // if true, compare:  small <= arg
    bool m_bMaxSet;    // if true, compare:  arg <= large
    bool m_bInfSet;    // if true, compare:  small < arg
    bool m_bSupSet;    // if true, compare:  arg < large

    // last error strings
    const char* m_acLastError;
    static char ms_acOptionNotFound[];
    static char ms_acArgumentRequired[];
    static char ms_acArgumentOutOfRange[];
    static char ms_acFilenameNotFound[];
};


#endif



