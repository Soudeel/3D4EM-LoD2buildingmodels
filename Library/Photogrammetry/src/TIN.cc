
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
 Collection of functions for class TIN          

          TIN::TIN(char *, int *)       Construction by reading from file
          TIN::TIN(Triangles *)         Construction by conversion of C object
          TIN::~TIN()                   Destructor
 TIN     &TIN::operator =(const TIN &)  Copy assignment
 void     TIN::Cpp2C(Triangles **)      Conversion of C++ to C object
 void     TIN::C2Cpp(Triangles *)       Conversion of C to C++ object
 int      TIN::Read(char *)             Read TIN from a database
 int      TIN::Write(char *)            Write TIN to a database
 int      TIN::Write(char *, int)       Write TIN to a compressed database
 void     TIN::Erase()                  Erase TIN and free capacity
 void     TIN::Derive(double *, int,    Derive TIN from a coordinate list
            int *, int, double *, int)
 void     TIN::VRML_Write(FILE *) const Write TIN meshes to a VRML file
 
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "TIN.h"
#include "triangle.h"

/*
--------------------------------------------------------------------------------
                     Declarations of functions in C
--------------------------------------------------------------------------------
*/

extern "C" void  triangulate(char *, struct triangulateio *,
				             struct triangulateio *,struct triangulateio *);

/*
--------------------------------------------------------------------------------
                     Constructors / Destructors 
--------------------------------------------------------------------------------
*/

/* Construct by reading the file */

TIN::TIN(char *filename, int *succes) : std::vector<TINMesh>()
{
  *succes = Read(filename);
}

/* Construct by converting the C object */

TIN::TIN(Triangles *triangles) : std::vector<TINMesh>()
{
  C2Cpp(triangles);
}

/* Destructor */

/* Note that the destructor does not release the capacity of the vector!
 * In order to free the capacity, first call TIN::Erase().
 */

TIN::~TIN()
{
  if (!empty()) erase(begin(), end());
}

/*
--------------------------------------------------------------------------------
                                 Copy assignment
--------------------------------------------------------------------------------
*/

TIN &TIN::operator = (const TIN &tin)
{
  // Check for self assignment
  if (this == &tin) return *this;
  if (!empty()) erase(begin(), end());
  if (!tin.empty()) insert(begin(), tin.begin(), tin.end());
  return(*this);
}
/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void TIN::Cpp2C(Triangles **trianglesptr) 
{
  TIN::const_iterator mesh;
  Triangles *triangles;   
  Triangle *triangle;

/* Allocate space */
 
  triangles = *trianglesptr;	
  if (triangles == NULL) {
    triangles = (Triangles *) malloc(sizeof(Triangles));
    if (triangles == NULL) printf("Error 1 malloc in TIN::Cpp2C\n");
    triangles->tri = (Triangle *) malloc(size()*sizeof(Triangle));
    if (triangles->tri == NULL) printf("Error 2 malloc in TIN::Cpp2C\n");
    *trianglesptr = triangles;
  }

/* Copy the data */

  triangles->num_tri = size();
  for (mesh=begin(), triangle=triangles->tri;
       mesh!=end();
       mesh++, triangle++) {
    mesh->Cpp2C(&triangle); 	
  }	
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void TIN::C2Cpp(Triangles *triangles)
{
  TINMesh  mesh; 	
  Triangle *triangle;
  int      i;

/* Delete old data and allocate new space */

  if (!empty()) erase(begin(), end());
  reserve(triangles->num_tri);

/* Convert all triangles */

  for (i=0, triangle=triangles->tri;
       i<triangles->num_tri;
       i++, triangle++) {
    mesh.C2Cpp(triangle);
    push_back(mesh);
  }
}

/*
--------------------------------------------------------------------------------
                       Read triangles from a file
--------------------------------------------------------------------------------
*/
int TIN::Read(char *filename)
{
  Triangles *triangles;

  triangles = Get_Triangles(filename);  /* Read the database into C structure */
  if (triangles == NULL) return(0);  	
  	
  C2Cpp(triangles);                     /* Convert to C++ object */    
  free(triangles->tri);
  free(triangles);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write triangles to a file
--------------------------------------------------------------------------------
*/

int TIN::Write(char *filename) 
{
  return(TIN::Write(filename, 0));
}

int TIN::Write(char *filename, int compress) 
{
  int       success;
  char      *command;
  Triangles *triangles; 
  
/* Convert to the C structure and write the file */

  triangles = NULL;  
  Cpp2C(&triangles);  
  success = Put_Triangles(triangles, filename);  
  free(triangles->tri);
  free(triangles);
  if (!success) return(success);
  
/* Compress the file if required */

  if (compress) {
    command = (char *) malloc((9+strlen(filename)) * sizeof(char));
    strcpy(command, "gzip -f ");
    strcat(command, filename);
    system(command);
    free(command);
  }
  return(success);
}

/*
--------------------------------------------------------------------------------
                Erase the TIN and free the capacity
--------------------------------------------------------------------------------
*/
void TIN::Erase()
{
  TIN dummy;

  if (!empty()) erase(begin(), end());
  swap(dummy);
}

/*
--------------------------------------------------------------------------------
                Derive the triangulation from a list of points
--------------------------------------------------------------------------------
*/
void TIN::Derive(double *coord, int num_pts, int *segm, int num_segm,
                 double *holecoord, int num_holes)
{
  int                  i, *node, *nb;
  struct triangulateio *in, *out;

/* Return an empty tin if the number of points is less than three. */

  if (num_pts < 3) return;

/* Initialise the input structure for the triangulation procedure */

  in = (struct triangulateio *) calloc(1, sizeof(struct triangulateio));
  in->numberofpoints = num_pts;
  in->pointlist = coord;
  if (num_segm) {
    in->numberofsegments = num_segm;
    in->segmentlist = segm;
  }
  if (num_holes) {
    in->numberofholes= num_holes;
    in->holelist = holecoord;
  }

/* Initialise the output structure for the triangulation procedure */

  out = (struct triangulateio *) calloc(1, sizeof(struct triangulateio));

/* Triangulate with options:
 *
 *   z - start numbering points from zero
 *   n - create list of neighbouring triangles
 *   N - do not copy the node list from the input to the output structure
 *   Q - quite mode
 *   p - use polygons to constrain triangulation
 */

  if (num_segm) triangulate((char *) "znNQp", in, out, NULL);
  else triangulate((char *) "znNQ", in, out, NULL);

/* Store the triangulation in the TIN class */

  if (!empty()) erase(begin(), end());
  reserve(out->numberoftriangles);
  for (i=0, node=out->trianglelist, nb=out->neighborlist;
       i<out->numberoftriangles;
       i++, node+=3, nb+=3)
    // Added check to delete wrong output of triangulate in case of
    // self intersecting boundary
    if (*node < num_pts && *(node+1) < num_pts && *(node+2) < num_pts)
      push_back(TINMesh(*node, *(node+1), *(node+2), *nb, *(nb+1), *(nb+2)));

/* Free all allocated memory, including that allocated by triangulate */

  free(in);
  free(out->trianglelist);
  free(out->neighborlist);
  free(out);
}

/*
--------------------------------------------------------------------------------
                   Write TIN meshes to a VRML file
--------------------------------------------------------------------------------
*/
void TIN::VRML_Write(FILE *fd) const
{
  TIN::const_iterator mesh;

  for (mesh=begin(); mesh!=end(); mesh++) mesh->VRML_Write(fd);
}

/*
--------------------------------------------------------------------------------
                   Derive the highest node number
--------------------------------------------------------------------------------
*/
PointNumber TIN::HighestNodeNumber() const
{
  TIN::const_iterator mesh;
  PointNumber         maximum = PointNumber(-1);
  const PointNumber   *nodes;
  int                 i;
  
  for (mesh=begin(); mesh!=end(); mesh++) {
    nodes = mesh->Nodes();
    for (i=0; i<3; i++)
      if (nodes[i] > maximum) maximum = nodes[i];
  }
  return maximum;
}
            
