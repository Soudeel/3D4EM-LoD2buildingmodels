
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



/* This include file redefines the variable of the architecture, like
 * e.g. hpux. Unfortunately this variable is undef'ed in Khoros.macro.
 * However, we use these variables in several routines for machine specific
 * code. Currently switches for linux, windows, and hpux are used.
 *
 * George Vosselman
 */

#ifndef windows
//#define windows - Changed by Tahir, add -Dwindows to g++.exe for Qt projects add DEFINES += windows 
#endif
//Include definitions of limits like DBL_MAX, DBL_MIN
#include <climits>
#include <cfloat>
