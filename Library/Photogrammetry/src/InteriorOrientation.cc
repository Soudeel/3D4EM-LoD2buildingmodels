
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
--------------------------------------------------------------------------------
 Collection of functions for class InteriorOrientation          

 void InteriorOrientation::Cpp2C(CamPts **)      Conversion of C++ to C object
 void InteriorOrientation::C2Cpp(CamPts *)       Conversion of C to C++ object
 int  InteriorOrientation::Read(char *)          Read interior orientation from a database
 int  InteriorOrientation::Write(char *)         Write interior orientation to a database
 void InteriorOrientation::Print()               Print interior orientation to stdout

 Initial creation
 Author : Ildiko Suveg
 Date   : 03-12-1998

 Update #1
 Author : 
 Date   : 
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdlib.h>
#include "InteriorOrientation.h"

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void InteriorOrientation::Cpp2C(Interior **intoptr) const
{
  Interior *into;

/* Allocate space if this has not been done yet */

  into = *intoptr;
  if (into == NULL) {
    into = (Interior *) malloc(sizeof(Interior));
    *intoptr = into;
  }

/* Copy the data */

  into->cc = cc; 
  into->rh = rh; 
  into->ch = ch; 
  into->k1 = k1; 
  into->k2 = k2; 
  into->k3 = k3;
  into->p1 = p1;
  into->p2 = p2; 
  into->shear = shear;
  into->rot = rot; 
  into->spacing_r = spacing_r; 
  into->spacing_c = spacing_c;
  into->dim_r = dim_r; 
  into->dim_c = dim_c;
  // bingo param
  into->bingo_cc = 0;
  into->bingo_xh = 0;
  into->bingo_yh = 0;
  into->bingo[0] = 0;
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void InteriorOrientation::C2Cpp(Interior *into)
{
  cc = into->cc; 
  rh = into->rh; 
  ch = into->ch; 
  k1 = into->k1; 
  k2 = into->k2; 
  k3 = into->k3;
  p1 = into->p1; 
  p2 = into->p2; 
  shear = into->shear;
  rot = into->rot; 
  spacing_r = into->spacing_r; 
  spacing_c = into->spacing_c;
  dim_r = into->dim_r; 
  dim_c = into->dim_c;
}

/*
--------------------------------------------------------------------------------
                       Read the interior orientation from a database
--------------------------------------------------------------------------------
*/
int InteriorOrientation::Read(const char *filename)
{
  Interior *into;

  into = Get_Interior(filename);      /* Read the database into C structure */
  if (into == NULL) return(0);
  C2Cpp(into);                        /* Convert to C++ object */
  free(into);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write the interior orientation to a database
--------------------------------------------------------------------------------
*/
int InteriorOrientation::Write(const char *filename) const
{
  int    error;
  Interior *into;

  into = NULL;
  Cpp2C(&into);
  error = Put_Interior(into, filename);
  free(into);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print interior orientation to stdout
--------------------------------------------------------------------------------
*/
void InteriorOrientation::Print() const
{
  Interior *into;

  into = NULL;
  Cpp2C(&into);
  Print_Interior(into);
  free(into);
}
