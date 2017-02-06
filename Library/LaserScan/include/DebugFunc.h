
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
*   File made : June 2002
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Functions and macros for debugging.
*
*--------------------------------------------------------------------*/

#ifdef _DEBUG_FUNC_H_
#undef _DEBUG_FUNC_H_
#endif

#ifndef _DEBUG_FUNC_H_
#define _DEBUG_FUNC_H_

#include <stdio.h>

#ifndef ENABLE_DEBUGGING
	#define ENABLE_DEBUGGING 0
#endif	

#if ENABLE_DEBUGGING
	#define DEBUG(fmt, args...) fprintf(stderr, fmt , ## args);fprintf(stderr," :%s  on Line %d \n",__FILE__,__LINE__);
#else
	#define DEBUG(fmt, args...)
#endif

#endif //_DEBUG_FUNC_H_


