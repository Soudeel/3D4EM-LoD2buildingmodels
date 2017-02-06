
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
 Collection of functions for class TINMesh
 
 void TINMesh::Cpp2C(Triangle **)        Conversion of C++ class to C structure
 void TINMesh::C2Cpp(Triangle *)         Conversion of C structure to C++ class
 void TINMesh::VRML_Write(FILE *) const  Write mesh to VRML file

 Initial creation
 Author : George Vosselman
 Date   : 18-03-1999

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
#include "TINMesh.h"

/*
--------------------------------------------------------------------------------
                     		Constructor
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void TINMesh::Cpp2C(Triangle **triangleptr) const 
{
  Triangle *triangle;
  int      i;

/* Allocate space if this has not been done yet */

  triangle = *triangleptr;
  if (triangle == NULL) {
    triangle = (Triangle *) malloc(sizeof(Triangle));
    *triangleptr = triangle;
  }

/* Copy the data from the C++ to the C object */

  for (i=0; i<3; i++) triangle->node[i] = node[i].Number();
  for (i=0; i<3; i++) triangle->nb[i] = nb[i].Number();
}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void TINMesh::C2Cpp(Triangle *triangle)
{
  int i;

  for (i=0; i<3; i++) node[i] = PointNumber(triangle->node[i]);
  for (i=0; i<3; i++) nb[i] = MeshNumber(triangle->nb[i]);
}

/*
--------------------------------------------------------------------------------
                     Write mesh to VRML file
--------------------------------------------------------------------------------
*/

void TINMesh::VRML_Write(FILE *fd) const
{
  fprintf(fd, "IndexedFaceSet { coordIndex [%i,%i,%i,-1]}\n",
          node[0].Number(), node[1].Number(), node[2].Number());
}
