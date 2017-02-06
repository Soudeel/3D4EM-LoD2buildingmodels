
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
 Collection of functions for class AbsoluteOrientation          

 void AbsoluteOrientation::Cpp2C(CamPts **)      Conversion of C++ to C object
 void AbsoluteOrientation::C2Cpp(CamPts *)       Conversion of C to C++ object
 int  AbsoluteOrientation::Read(char *)          Read Absolute orientation from a database
 int  AbsoluteOrientation::Write(char *)         Write Absolute orientation to a database
 void AbsoluteOrientation::Print()               Print Absolute orientation to stdout

 Initial creation
 Author : Ildiko Suveg
 Date   : 03-03-1998

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

#include "AbsoluteOrientation.h"

AbsoluteOrientation::AbsoluteOrientation(const AbsoluteOrientation &absor)	
  : Orientation3D()
{
   OrientRef() = absor.OrientRef();
   lambda = absor.lambda;
}


AbsoluteOrientation& AbsoluteOrientation::operator=(const AbsoluteOrientation &absor)	
{
  // Check for self assignment
  if (this == &absor) return *this;
  OrientRef() = absor.OrientRef();
  lambda = absor.lambda;
  return *this;
}



/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void AbsoluteOrientation::Cpp2C(Absolute **absorptr) const
{
  Absolute *absor;

/* Allocate space if this has not been done yet */

  absor = *absorptr;
  if (absor == NULL) {
    absor = (Absolute *) malloc(sizeof(Absolute));
    *absorptr = absor;
  }

/* Copy the data */

   absor->x = x[0]; 
   absor->y = x[1]; 
   absor->z = x[2];
   for (int j = 0; j < W_DIM; j++)
	for (int i = 0; i < W_DIM; i++)	
	   absor->rot[j][i] = r[j][i];
   absor->lambda = lambda;	   
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void AbsoluteOrientation::C2Cpp(Absolute *absor)
{
   x[0] = absor->x; 
   x[1] = absor->y; 
   x[2] = absor->z;
   for (int j = 0; j < W_DIM; j++)
    for (int i = 0; i < W_DIM; i++)	
    	r[j][i] = absor->rot[j][i];
   lambda = absor->lambda;
}

/*
--------------------------------------------------------------------------------
                       Read the Absolute orientation from a database
--------------------------------------------------------------------------------
*/
int AbsoluteOrientation::Read(char *filename)
{
  Absolute *absor;
  int err; 	
  absor = Get_Absolute(filename, &err);      /* Read the database into C structure */
  if (absor == NULL) return(0);
  C2Cpp(absor);                        /* Convert to C++ object */
  free(absor);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write the Absolute orientation to a database
--------------------------------------------------------------------------------
*/
int AbsoluteOrientation::Write(char *filename) const
{
  int    error;
  Absolute *absor;

  absor = NULL;
  Cpp2C(&absor);
  error = Put_Absolute(absor, filename);
  free(absor);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print Absolute orientation to stdout
--------------------------------------------------------------------------------
*/
void AbsoluteOrientation::Print() const
{
  Absolute *absor;

  absor = NULL;
  Cpp2C(&absor);
  Print_Absolute(absor);
  free(absor);
}

