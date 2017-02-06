
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
*   File made : July 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A class for profiling functions. Constructor notes the start time
*				and the total time taken by the function is printed by the destructor.
*
*--------------------------------------------------------------------*/

#include <stdio.h>
#include <time.h>
#include "FunctionTimer.h"

//Constructor saves the time and function name. So that time taken by the function can be printed 
//at the end.
FunctionTimer::FunctionTimer(char* n)
{
	snprintf(name,256,"%s",n);
	startTime = clock();
}

//Destructor. Prints the time taken by the function.
FunctionTimer::~FunctionTimer()
{
	double endTime = clock();
	fprintf(stderr,"%s: time %4.3f ms\n",name,(endTime-startTime)/1000.00);
}


