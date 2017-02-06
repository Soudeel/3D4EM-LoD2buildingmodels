
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
 Collection of functions for class LaserOctree

 Initial creation
 Author : George Vosselman
 Date   : 07-10-2004

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
#include "stdmath.h"
#include "LaserOctree.h"

/*
--------------------------------------------------------------------------------
                                 Initialisation
--------------------------------------------------------------------------------
*/

void LaserOctree::Initialise()
{
  Erase();
  bounds.Initialise();
  margin = 0.0;
  max_num_pts = 1000;
}

void LaserOctree::Initialise(const DataBounds3D &new_bounds, double new_margin,
                             int new_max_num_pts)
{
  Erase();
  bounds      = new_bounds;
  margin      = new_margin;
  max_num_pts = new_max_num_pts;
}

/*
--------------------------------------------------------------------------------
                                 Erase octree
--------------------------------------------------------------------------------
*/

void LaserOctree::Erase()
{
  std::vector <LaserOctree>::iterator child;

  // Erase children
  if (children.size())
    for (child=children.begin(); child!=children.end(); child++)
      child->Erase();

  // Erase point numbers
  if (numbers.size()) numbers.Erase();
}

/*
--------------------------------------------------------------------------------
                                 Insert a point
--------------------------------------------------------------------------------
*/

void LaserOctree::Insert(const LaserPoints &points, const PointNumber &number)
{
  std::vector <LaserOctree>::iterator child;
  double                              exclusive;
  LaserPoints::const_iterator         point = points.begin() + number.Number();

  // Insert in one the children
  if (children.size()) {
    for (child=children.begin(), exclusive=false;
         child!=children.end() && !exclusive; child++) {
      if (child->Bounds().Inside(point->Position3DRef(), margin)) {
        child->Insert(points, number);
        if (child->Bounds().Inside(point->Position3DRef(), -margin))
          exclusive = true;
      }
    }
  }

  // Insert here
  else {
    numbers.push_back(number);
    // Check if we need to split
    if (numbers.size() > max_num_pts) 
      if (bounds.XRange() > 2.0 * margin) Split(points);
  }
}

void LaserOctree::Insert(const LaserPoints &points)
{
  LaserPoints::const_iterator point;
  int                         number;
  
  for (point=points.begin(), number=0; point!=points.end(); point++, number++)
    Insert(points, PointNumber(number));
}

/*
--------------------------------------------------------------------------------
                                 Split into children
--------------------------------------------------------------------------------
*/

void LaserOctree::Split(const LaserPoints &points)
{
  DataBounds3D                child_bounds;
  LaserPoints::const_iterator point;
  PointNumberList::iterator   number;
  int                         x, y, z;

  // Create the children
  for (x=0; x<2; x++) {
    if (x == 0) {
      child_bounds.SetMinimumX(bounds.Minimum().X());
      child_bounds.SetMaximumX(bounds.Minimum().X() + bounds.XRange() / 2.0);
    }
    else {
      child_bounds.SetMinimumX(bounds.Minimum().X() + bounds.XRange() / 2.0);
      child_bounds.SetMaximumX(bounds.Maximum().X());
    }
    for (y=0; y<2; y++) {
      if (y == 0) {
        child_bounds.SetMinimumY(bounds.Minimum().Y());
        child_bounds.SetMaximumY(bounds.Minimum().Y() + bounds.YRange() / 2.0);
      }
      else {
        child_bounds.SetMinimumY(bounds.Minimum().Y() + bounds.YRange() / 2.0);
        child_bounds.SetMaximumY(bounds.Maximum().Y());
      }
      for (z=0; z<2; z++) {
        if (z == 0) {
          child_bounds.SetMinimumZ(bounds.Minimum().Z());
          child_bounds.SetMaximumZ(bounds.Minimum().Z() + bounds.ZRange() /2.0);
        }
        else {
          child_bounds.SetMinimumZ(bounds.Minimum().Z() + bounds.ZRange() /2.0);
          child_bounds.SetMaximumZ(bounds.Maximum().Z());
        }
        children.push_back(LaserOctree(child_bounds, margin, max_num_pts));
      }
    }
  }

  // Transfer the points to the children
  for (number=numbers.begin(); number!=numbers.end(); number++)
    Insert(points, *number);

  // Erase the point numbers here
  numbers.Erase();
}

/*
--------------------------------------------------------------------------------
                         Number of cubes in the octree
--------------------------------------------------------------------------------
*/

int LaserOctree::NumberOfCubes(bool leaves_only) const
{
  std::vector <LaserOctree>::const_iterator child;
  int                                       number;

  if (children.size()) {
    for (child=children.begin(), number=0; child!=children.end(); child++)
      number += child->NumberOfCubes(leaves_only);
    if (!leaves_only) number++;
  }
  else number = 1;
  return number;
}

/*
--------------------------------------------------------------------------------
                         Depth of the octree
--------------------------------------------------------------------------------
*/

int LaserOctree::Depth() const
{
  std::vector <LaserOctree>::const_iterator child;
  int                                       depth=1;

  if (children.size())
    for (child=children.begin(); child!=children.end(); child++)
      depth = MAX(depth, child->Depth() + 1);
  return depth;
}

/*
--------------------------------------------------------------------------------
                         Number of points in leaves
--------------------------------------------------------------------------------
*/

int LaserOctree::NumberOfPoints() const
{
  std::vector <LaserOctree>::const_iterator child;
  int                                       number;

  if (children.size())
    for (child=children.begin(), number=0; child!=children.end(); child++)
      number += child->NumberOfPoints();
  else
    number = numbers.size();
  return number;
}

/*
--------------------------------------------------------------------------------
                         Derive neighbourhood edges
--------------------------------------------------------------------------------
*/

TINEdges * LaserOctree::NeighbourhoodEdges3D(const LaserPoints &points,
                                             double range) const
{
  TINEdges                        *edges;
  LaserPoints::const_iterator     point, nb_point;
  const LaserOctree               *octree;
  int                             number;
  TINEdgeSet                      edgeset;
  double                          sqrange = range * range;
  PointNumberList::const_iterator nb_node;
  PointNumberList                 neighbourhood;

  edges = new TINEdges();

  // Search in multiple octree cubes
  if (range > margin) {
    printf("Search in multiple cubes\n");
    for (point=points.begin(); point!=points.end(); point++) {
      Neighbourhood(neighbourhood, point->Position3DRef(), points, range);
      edges->push_back(TINEdgeSet(neighbourhood));
    }
  }

  // Search in only one octree cube for each point
  else {
    for (point=points.begin(), number=0; point!=points.end();
         point++, number++) {
      octree = LeafCube(point->Position3DRef());
      if (!octree->PointNumbers().Contains(PointNumber(number)))
        printf("Point %d not inside cube", number);
      else {
        for (nb_node=octree->PointNumbers().begin();
             nb_node!=octree->PointNumbers().end(); nb_node++) {
          if (nb_node->Number() != number) {
            nb_point = points.begin() + nb_node->Number();
            if ((*point - *nb_point).SqLength() < sqrange)
              edgeset.push_back(*nb_node);
          }
        }
      }
      edges->push_back(edgeset);
      edgeset.Erase();
    }
  }

  return edges;
}

/*
--------------------------------------------------------------------------------
                    Locate the cube of a specified position
--------------------------------------------------------------------------------
*/

const LaserOctree * LaserOctree::LeafCube(const Position3D &pos) const
{
  std::vector <LaserOctree>::const_iterator child;
  
  if (children.size()) {
    child = children.begin();
    if (pos.X() > child->Bounds().Maximum().X()) child += 4;
    if (pos.Y() > child->Bounds().Maximum().Y()) child += 2;
    if (pos.Z() > child->Bounds().Maximum().Z()) child += 1;
    return child->LeafCube(pos);
  }
  return this;
}

/*
--------------------------------------------------------------------------------
                     Collect all points within some distance
--------------------------------------------------------------------------------
*/

void LaserOctree::Neighbourhood(PointNumberList &neighbourhood,
                                const Position3D &pos,
                                const LaserPoints &points, double range) const
{
  DataBounds3D bounding_box;

  // Start with an empty neighbourhood
  if (neighbourhood.size()) neighbourhood.Erase();

  // Derive the bounding box for the search area
  bounding_box.SetMinimumX(pos.X() - range);
  bounding_box.SetMinimumY(pos.Y() - range);
  bounding_box.SetMinimumZ(pos.Z() - range);
  bounding_box.SetMaximumX(pos.X() + range);
  bounding_box.SetMaximumY(pos.Y() + range);
  bounding_box.SetMaximumZ(pos.Z() + range);

  // Get the points
  AddNearPoints(neighbourhood, pos, bounding_box, points, range * range);
}

void LaserOctree::AddNearPoints(PointNumberList &neighbourhood,
                                const Position3D &pos,
                                const DataBounds3D &bounding_box,
                                const LaserPoints &points, double sqrange) const
{
  std::vector <LaserOctree>::const_iterator child;
  PointNumberList::const_iterator           number;
  LaserPoints::const_iterator               point;

  // Look for points in children
  if (children.size()) {
    for (child=children.begin(); child!=children.end(); child++)
      if (Overlap(child->Bounds(), bounding_box))
        child->AddNearPoints(neighbourhood, pos, bounding_box, points, sqrange);
  }
  // Look for points here
  else {
    for (number=numbers.begin(); number!=numbers.end(); number++) {
      point = points.begin() + number->Number();
      if (margin == 0.0) {
        if ((pos - point->Position3DRef()).SqLength() <= sqrange)
          neighbourhood.push_back(*number);
      }
      else {
        if (bounds.Inside(point->Position3DRef()) &&
            (pos - point->Position3DRef()).SqLength() <= sqrange)
          neighbourhood.push_back(*number);
      }
    }
  }
}
