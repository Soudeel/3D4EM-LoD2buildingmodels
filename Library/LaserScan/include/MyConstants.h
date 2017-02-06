
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



#ifndef _CONSTANTS_H___
#define _CONSTANTS_H___
/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : June 2002
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Definition of various constants, macros and type definitions.
*
*--------------------------------------------------------------------*/

#define SUCCESS 0
#define CLIP(var,val) (var)=(var)>(val)?(val):(var);
#define CLAMP(var,val) (var)=(var)<(val)?(val):(var);
#define KEEP_BETWEEN(var,down,up) CLIP(var,up);CLAMP(var,down);
#define MIN(a,b) ((a)>(b)?(b):(a))
#define MAX(a,b) ((a)<(b)?(b):(a))
#define MY_PI 3.1415926536
#define FALSE 0
#define TRUE 1
#define RADIAN_TO_DEGREE 	(180.00/MY_PI)
#define DEGREE_TO_RADIAN	(MY_PI/180.00)

///Macros for bit manipulation.
#define TURN_ON(a,b) a = ((b)|(a))
#define TURN_OFF(a,b) (a) = (b) & ((a)^0xFFFFFFFF)
#define TOGGLE(a,b) (a) = (b) ^ (a)
#define IS_ON(a,b) ((a) & (b))
#define IS_OFF(a,b) (!((a) & (b)))

//Bound enforcing macros.
#define BOUND_MIN(a,m) (a) = MAX((a),(m));
#define BOUND_MAX(a,m) (a) = MIN((a),(m));
#define BOUND_NONZERO(a,m) (a) = (fabs(a)>0)?fabs(a):(m);

//For memory manipulation.
#define MEM_ZERO(pointer,size) {if(!pointer)cerr<<"NULL pointer at: "<<__LINE__<<"in file: "<<__FILE__<<endl;\
							else memset(pointer,0,sizeof(*(pointer))*(size));}

//X-Related macros.
#define KEYCODE_TO_STRING(s) (XKeysymToString(XKeycodeToKeysym(fl_display,(s),0)))

//Some type declarations.
typedef unsigned char BYTE;


#endif //_CONSTANTS_H___

