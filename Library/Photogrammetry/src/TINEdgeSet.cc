
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
 Collection of functions for class TINEdgeSet

 TINEdgeSet & TINEdgeSet::operator = (TINEdgeSet &)       Copy assignment
 void TINEdgeSet::Derive(TIN &, PointNumber, MeshNumber)  Derive edges of point
 
 Initial creation
 Author : George Vosselman
 Date   : 06-05-1999

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
#include <string.h>
#include "TINEdgeSet.h"
#include "TIN.h"

/*
--------------------------------------------------------------------------------
                           Copy assignment
--------------------------------------------------------------------------------
*/
TINEdgeSet & TINEdgeSet::operator = (const TINEdgeSet &set)
{
  // Check for self assignment
  if (this == &set) return *this;
  if (!empty()) erase(begin(), end());
  if (!set.empty()) insert(begin(), set.begin(), set.end());
  return(*this);
}

/*
--------------------------------------------------------------------------------
            Derive the edges of a point in a Delaunay triangulation
--------------------------------------------------------------------------------
*/
void TINEdgeSet::Derive(const TIN &tin, const PointNumber &ptno,
                        const MeshNumber &firstmesh)
{
  MeshNumber         nextmesh, outside=MeshNumber(-1);
  const MeshNumber   *neighbours;
  PointNumber        nextpoint, tmppoint;
  const PointNumber  *nodes;
  int                index_node, index_mesh, done, direction;
  TINEdgeSet::iterator point1, point2;

/* Delete old edges */

  if (!empty()) erase(begin(), end());

/* Collect the edges */

  nextmesh = firstmesh;
  done = 0;
  direction = 1;
  while (!done) {

/* Get the nodes and neighbour meshes of the current mesh */

    nodes      = (tin.begin() + nextmesh.Number())->Nodes();
    neighbours = (tin.begin() + nextmesh.Number())->Neighbours();

/* Find the current point */

    index_node = 0;
    while (index_node < 3 && nodes[index_node] != ptno) index_node++;
    if (index_node == 3) {
      fprintf(stderr, "Error: Inconsistancy detected in TIN.\n");
      fprintf(stderr, "       Can not derive edge set of point %d\n",
              ptno.Number());
      return;
    }

/* Get the next point and add it to the edge set */

    index_node -= direction;
    if (index_node == -1) index_node = 2;
    if (index_node == 3) index_node = 0;
    nextpoint = nodes[index_node];
    reserve(size()+1); /* Avoid allocation of large vector! */
    push_back(nextpoint);

/* Get the next adjacent mesh */

    index_mesh = index_node - direction;
    if (index_mesh == -1) index_mesh = 2;
    if (index_mesh == 3) index_mesh = 0;
    nextmesh = neighbours[index_mesh];

/* Check if the next mesh is the outside of the TIN. If it is, reverse the
 * order of the edges. If this is the first time the outside is encountered,
 * continue the search in the opposite direction. Otherwise, we're finished.
 */ 

    if (nextmesh == outside) {

      for (point1=begin(), point2=end()-1;
           point1<begin()+size()/2;
           point1++, point2--) {
        tmppoint = *point1;
        *point1 = *point2;
        *point2 = tmppoint;
      }

      if (direction == 1) {
        direction = -1;
        nextmesh = firstmesh;
      }
      else done = 1;
    }

/* We're finished if the next mesh is the first mesh */

    else {
      if (nextmesh == firstmesh) done = 1;
    }
  }
}

/*
--------------------------------------------------------------------------------
                    Add extra edges to this edge set
--------------------------------------------------------------------------------
*/
void TINEdgeSet::Add(const TINEdgeSet &extra_edges)
{
  TINEdgeSet::const_iterator extra_edge;

  for (extra_edge=extra_edges.begin(); extra_edge!=extra_edges.end();
       extra_edge++)
    if (!Contains(*extra_edge)) push_back(*extra_edge);
}
