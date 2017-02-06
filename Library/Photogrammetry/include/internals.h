
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



/* 
  dummy internal.h file for the compilation 

  under KHOROS in the CANTATA environment 
  internals.h is a file in the source directory 
  (*.c, *.f, *.cc) with some internal definitions. 

  These definitions are not necessary under 
  the compilation in the CVS environment. 
  Therefore this file is empty. 

  However, some functions require NULL to be defined 
  or reference stderr, which is written in non of the
  include-files und CANTATA (or at least not obviously).
  These definitions are inserted here. 

*/
#ifndef __INTERNALS_H
#define __INTERNALS_H 

#include <stdio.h>

#ifndef NULL
#define NULL 0x0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#endif


