
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
 Collection of functions for class TINEdges

 void TINEdges::Derive(const TIN &)         Derive the edges of a TIN
 void TINEdges::VRML_Write(FILE *) const    Write edges to a VRML file
 
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
#include <malloc.h>
#include "TIN.h"
#include "TINEdges.h"
#include "LineTopologies.h"

/*
void memory_usage_TIN(char *string)
{
  struct mallinfo info;

  info = mallinfo();
  printf("%s: %d bytes\n", string, info.uordblks);
}
*/

/*
--------------------------------------------------------------------------------
                                 Copy assignment
--------------------------------------------------------------------------------
*/

TINEdges &TINEdges::operator = (const TINEdges &edges)
{
  // Check for self assignment
  if (this == &edges) return *this;
  if (!empty()) erase(begin(), end());
  if (!edges.empty()) insert(begin(), edges.begin(), edges.end());
  return(*this);
}

/*
--------------------------------------------------------------------------------
                Derive the edges of a Delaunay triangulation
--------------------------------------------------------------------------------
*/
void TINEdges::Derive(const TIN &tin)
{
  TIN::const_iterator mesh;
  TINEdges::iterator  edgeset;
  PointNumber         maxnum;
  const PointNumber   *ptno;
  int                 i, index_mesh;

/* Determine the highest point number */

  maxnum = PointNumber(0);
  for (mesh=tin.begin(); mesh!=tin.end(); mesh++)
    for (i=0, ptno=mesh->Nodes(); i<3; i++, ptno++)
      if (*ptno > maxnum) maxnum = *ptno;

/* Allocate and initialise list of edges */

  if (!empty()) erase(begin(), end());
  reserve(maxnum.Number()+1);
  for (i=0; i<=maxnum.Number(); i++) push_back(TINEdgeSet());

/* Derive edge sets for all points */

  for (mesh=tin.begin(), index_mesh=0; mesh!=tin.end(); mesh++, index_mesh++) {
    for (i=0, ptno=mesh->Nodes(); i<3; i++, ptno++) {
      edgeset = begin() + ptno->Number();
      if (!edgeset->size()) { /* Point without edge set */
        edgeset->Derive(tin, *ptno, MeshNumber(index_mesh));
      }
    }
  }
}

/*
--------------------------------------------------------------------------------
                      Write edges to a VRML file
--------------------------------------------------------------------------------
*/
void TINEdges::VRML_Write(FILE *vrml) const 
{
  TINEdges::const_iterator        edgeset;
  int                             point_number;
  PointNumberList::const_iterator nbnode;

  for (point_number=0, edgeset=begin(); edgeset!=end();
       point_number++, edgeset++)
    for (nbnode=edgeset->begin(); nbnode!=edgeset->end(); nbnode++)
      if (point_number < nbnode->Number()) // Only write each edge once
        fprintf(vrml, "IndexedLineSet { coordIndex [%i,%i,-1] }\n",
                point_number, nbnode->Number());
}

/*
--------------------------------------------------------------------------------
                      Add more edges to the edge sets
--------------------------------------------------------------------------------
*/

void TINEdges::Add(const TINEdges &extra_edges)
{
  TINEdges::iterator       edgeset1;
  TINEdges::const_iterator edgeset2;
  int num_added=0;
  
  for (edgeset1=begin(), edgeset2=extra_edges.begin();
       edgeset1!=end(); edgeset1++, edgeset2++) {
    num_added -= edgeset1->size();
    edgeset1->Add(edgeset2->TINEdgeSetRef());
    num_added += edgeset1->size();
  }
  printf("%d edges added\n", num_added / 2);
}

/*
--------------------------------------------------------------------------------
                      Erase all edges
--------------------------------------------------------------------------------
*/

void TINEdges::Erase()
{
  TINEdges::iterator edgeset;
  TINEdges           no_edges;

  if (empty()) return;
  for (edgeset=begin(); edgeset!=end(); edgeset++) edgeset->Erase();
  erase(begin(), end());
  swap(no_edges);
}

/*
--------------------------------------------------------------------------------
                      Convert to line topologies
--------------------------------------------------------------------------------
*/

LineTopologies * TINEdges::EdgeTopologies() const
{
  LineTopologies             *lines;
  LineTopology               line;
  TINEdges::const_iterator   edgeset;
  TINEdgeSet::const_iterator nb;
  int                        number, line_number=1;
  
  lines = new LineTopologies();
  for (edgeset=begin(), number=0; edgeset!=end(); edgeset++, number++) {
    for (nb=edgeset->begin(); nb!=edgeset->end(); nb++) {
      line.Number() = line_number;
      line.push_back(PointNumber(number));
      line.push_back(*nb);
      lines->push_back(line);
      line.Erase();
    }
  }
  return lines;
}
