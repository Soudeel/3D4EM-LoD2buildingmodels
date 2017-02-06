
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
 Collection of functions for class RelativeOrientation          

 int  RelativeOrientation::Read(char *)          Read Relative orientation from a database
 int  RelativeOrientation::Write(char *)         Write Relative orientation to a database

 Initial creation
 Author : Ildiko Suveg
 Date   : 10-12-1998

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

#include "RelativeOrientation.h"
#include "Database4Cpp.h"

/*
--------------------------------------------------------------------------------
                       Read the Relative orientation from a database
--------------------------------------------------------------------------------
*/
int RelativeOrientation::Read(char *filename, ExteriorOrientation *ext1,
					ExteriorOrientation *ext2)
{
  Exterior *extor1 = (Exterior *) malloc(sizeof(Exterior));	
  Exterior *extor2 = (Exterior *) malloc(sizeof(Exterior));	
  printf("File: %s\n", filename);
  int err = Get_Relative(filename, extor1, extor2);   /* Read the database into C structure */
  ext1->C2Cpp(extor1);                        /* Convert to C++ object */
  ext2->C2Cpp(extor2);
  if (err == 0) 
  {
    printf("Error reading relative orientation \n");	
    return(0);
  }  
  free(ext1);
  free(ext2);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write the Relative orientation to a database
--------------------------------------------------------------------------------
*/
int RelativeOrientation::Write(char *filename, const ExteriorOrientation *ext1,
					const ExteriorOrientation *ext2) const
{
  Exterior *extor1, *extor2;
  ext1->Cpp2C(&extor1);
  ext2->Cpp2C(&extor2);	
  int err = Put_Relative(extor1, extor2, filename);
  free(extor1);
  free(extor2);
  return(err);
}

