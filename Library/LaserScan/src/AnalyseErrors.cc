
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
 Additional collection of functions for class LaserPoints

 void LaserPoints::ErrorHistogram              Analyse type 1 or 2 errors
   (TIN &, Image &)
 TINMesh *LaserPoints::SearchMesh              Search mesh around start mesh
   (LaserPoint *, TINMesh *, char *,
    vector <MeshNumber> &)
 TINMesh *LaserPoints::InitialSearchMesh       Path search from start mesh
   (LaserPoint *, TINMesh *)
 TINMesh *LaserPoints::SimpleSearchMesh        Linear search from start mesh
   (LaserPoint *, TINMesh *)

 Initial creation
 Author : George Vosselman
 Date   : 07-02-2000

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
#include "LaserDataTypes.h"
#include "LaserPoints.h"

/*
--------------------------------------------------------------------------------
                        Put errors in an histogram
--------------------------------------------------------------------------------
*/

void LaserPoints::ErrorHistogram(const LaserPoints &errors, Image &histogram,
                                 int error_type) const
{
  LaserPoints::const_iterator point;
  const TINMesh               *start_mesh, *found_mesh;
  double                      min_error, max_error, int_error, tin_height,
                              error;
  int                         dim_histo, ie, num;
  float                       *histo_pixel;
  std::vector <MeshNumber>    meshnumber_list;
  char                        *mesh_labels;

/* Retreive histogram properties */

  histogram.Get1DLocationData(&min_error, &max_error);
  dim_histo = histogram.NumColumns();
  int_error = (max_error - min_error) / dim_histo;

/* Allocation and initialisation of mesh search arrays */

  meshnumber_list.reserve(tin->size());
  mesh_labels = (char *) malloc(tin->size());

/* Loop over all errors */

  start_mesh = &*tin->begin();
  num = 0;
  for (point=errors.begin(); point!=errors.end(); point++) {

    found_mesh = SearchMesh(&*point, start_mesh, mesh_labels, meshnumber_list);

/* Calculate the error size */

    if (found_mesh) {
      tin_height = Inter_plane(found_mesh, point->X(), point->Y(), HeightData);
      if (error_type == 1) error = tin_height - point->Z();
      else error = point->Z() - tin_height;  /* Type 2 error */

/* Map the error into the histogram */

      ie = (int) ((error - min_error) / int_error);
      if (ie < 0) ie = 0;
      else if (ie >= dim_histo) ie = dim_histo;
      histo_pixel = histogram.FloatPixel(0, ie);
      *histo_pixel += 1.0;

/* Determine the start mesh for the next search */

      start_mesh = found_mesh;
      num++;
    }
  }
}

/*
--------------------------------------------------------------------------------
                        Search the mesh around a point
--------------------------------------------------------------------------------
*/

const TINMesh *LaserPoints::SearchMesh(const LaserPoint *point,
                               const TINMesh *start_mesh,
                               char *mesh_labels,
                               std::vector <MeshNumber> &meshnumber_list) const
{
  const TINMesh *next_mesh;
  TIN::iterator mesh;
  int           number, inb;
  MeshNumber    *nbs, *nb;
  std::vector <MeshNumber>::iterator meshnumber;

/* Initialise mesh labels and the neighbourhood */

  memset((void *) mesh_labels, 0, tin->size());
  if (!meshnumber_list.empty())
    meshnumber_list.erase(meshnumber_list.begin(), meshnumber_list.end());

/* Initial search for approximate location */

  next_mesh = InitialSearchMesh(point, start_mesh);

/* Start searching and give up after examining 1000 meshes */

  number = ((int) next_mesh - (int) (&*tin->begin())) / sizeof(TINMesh);
  mesh_labels[number] = 1;
  meshnumber_list.push_back(MeshNumber(number));
  for (meshnumber=meshnumber_list.begin();
       meshnumber!=meshnumber_list.end() && meshnumber_list.size() < 1000;
       meshnumber++) {
    mesh = tin->begin() + meshnumber->Number();
    if (InsideTriangle(&*mesh, point->X(), point->Y())) return(&*mesh);
    nbs = mesh->Neighbours();
    for (inb=0, nb=nbs; inb<3; inb++, nb++) {
      if (nb->Number() != -1) {
        if (!mesh_labels[nb->Number()]) {
          meshnumber_list.push_back(*nb);
          mesh_labels[nb->Number()] = 1;
        }
      }
    }
  }
  return(NULL);
}

const TINMesh *LaserPoints::InitialSearchMesh(const LaserPoint *point,
                                              const TINMesh *start_mesh) const
{
  int               looping, num_try, ipt, inb;
  const MeshNumber  *nbmeshnum;
  const PointNumber *nodes, *nbptnum;
  const TINMesh     *nbmesh, *newmesh, *old_mesh1, *old_mesh2, *old_mesh3;
  const TINMesh     *mesh;
  LaserPoints::const_iterator nbpoint;
  double            sqdist, minsqdist;

  looping = num_try = 0;
  mesh = start_mesh;
  old_mesh1 = old_mesh2 = old_mesh3 = NULL;
  while (num_try < 1000 && !looping) {
    num_try++;
    if (InsideTriangle(mesh, point->X(), point->Y())) return(mesh);
    nodes = mesh->Nodes(); /* Nodes of current mesh */
    minsqdist = 1.0e30;
    newmesh = NULL;
    for (nbmeshnum=mesh->Neighbours(), inb=0; inb<3; inb++, nbmeshnum++) {
      if (nbmeshnum->Number() != -1) { /* No TIN border */
        nbmesh = &*(tin->begin() + nbmeshnum->Number()); /* Neighbouring mesh */
        for (nbptnum=nbmesh->Nodes(), ipt=0; ipt<3; ipt++, nbptnum++) {
          if (*nbptnum != nodes[0] &&
              *nbptnum != nodes[1] &&
              *nbptnum != nodes[2]) {
            nbpoint = begin() + nbptnum->Number();
            sqdist = (*point - *nbpoint).SqLength2D();
            if (sqdist < minsqdist) {
              minsqdist = sqdist;
              newmesh = nbmesh;
            }
          }
        }
      }
    }
    if (!newmesh) {
      printf("ISM: NULL mesh after %d meshes!\n", num_try);
      return(mesh);
    }
    if (newmesh == old_mesh1 || newmesh == old_mesh2 || newmesh == old_mesh3) {
      looping = 1;
    }
    else {
      old_mesh3 = old_mesh2;
      old_mesh2 = old_mesh1;
      old_mesh1 = mesh;
      mesh = newmesh;
    }
  }
  return(mesh);
}

const TINMesh *LaserPoints::SimpleSearchMesh(const LaserPoint *point,
                                             const TINMesh *previous_mesh) const
{

  const TINMesh *start_mesh, *mesh, *found_mesh;
  int           num_try;

/* Set starting position */

  start_mesh = previous_mesh - 100;
  if (start_mesh < &*tin->begin()) start_mesh = &*tin->begin();

/* Search beyond starting position */

  for (mesh=start_mesh, found_mesh=NULL, num_try=0;
       mesh!=&*tin->end() && !found_mesh;
       mesh++, num_try++)
    if (InsideTriangle(mesh, point->X(), point->Y())) found_mesh = mesh;

/* Search before starting position */

  if (!found_mesh)
    for (mesh=&*tin->begin();
         mesh!=start_mesh && !found_mesh;
         mesh++, num_try++)
      if (InsideTriangle(mesh, point->X(), point->Y())) found_mesh = mesh;

/* Return the mesh */

  return(found_mesh);
}
