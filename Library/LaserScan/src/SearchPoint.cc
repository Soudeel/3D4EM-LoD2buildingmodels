
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
 Additional collection of functions for searching a point in a set of points

 int LargerXYZ(const void *, const void *)    Order function for qsort
 void LaserPoints::SortOnCoordinates()        Sort all points on coordinates
 LaserPoint *LaserPoints::MatchXYZ            Find point with same coordinates
   (const LaserPoint *, LaserPoint *)
 

 Initial creation
 Author : George Vosselman
 Date   : 11-02-2000

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
#include "LaserPoints.h"

/*
--------------------------------------------------------------------------------
                        Sort all points after coordinates
--------------------------------------------------------------------------------
*/


int LargerXYZ(const LaserPoint *point1, const LaserPoint *point2)
{
  if (point1->X() > point2->X() + 0.0001) return(1);    /* First sort on X */
  if (point1->X() < point2->X() - 0.0001) return(-1);
  if (point1->Y() > point2->Y() + 0.0001) return(1);    /* Then on Y */
  if (point1->Y() < point2->Y() - 0.0001) return(-1);
  if (point1->Z() > point2->Z() + 0.0001) return(1);    /* Then on Z */
  if (point1->Z() < point2->Z() - 0.0001) return(-1);
  return(0);                                   /* Same coordinates */
}

int LargerXYZqsort(const void *pt1, const void *pt2)
{
  return(LargerXYZ((const LaserPoint *) pt1, (const LaserPoint *) pt2));
}

void LaserPoints::SortOnCoordinates()
{
  qsort((void *) &*begin(), (int) size(), sizeof(LaserPoint), LargerXYZqsort);
}
/*
--------------------------------------------------------------------------------
                        Sort all points after coordinates
--------------------------------------------------------------------------------
*/


int LargerScalar(const LaserPoint *point1, const LaserPoint *point2)
{
  if (point1->FloatAttribute(ScalarTag) > point2->FloatAttribute(ScalarTag))
    return(1); // Larger scalar
  if (point1->FloatAttribute(ScalarTag) < point2->FloatAttribute(ScalarTag))
    return(-1); // Smaller scalar
  // Same scalar: sort on coordinates
  return(LargerXYZ(point1, point2));
}

int LargerScalarqsort(const void *pt1, const void *pt2)
{
  return(LargerScalar((const LaserPoint *) pt1, (const LaserPoint *) pt2));
}

void LaserPoints::SortAlongLine(const Line2D &line, bool remove_scalars,
                                bool use_given_scalars)
{
  // Add scalars to the points, if needed
  if (!use_given_scalars || !(begin()->HasAttribute(ScalarTag)))
    for (LaserPoints::iterator point=begin(); point!=end(); point++)
      point->FloatAttribute(ScalarTag) = line.Scalar(point->Position2DOnly());
    
  // Sort on scalar
  qsort((void *) &*begin(), (int) size(), sizeof(LaserPoint),
        LargerScalarqsort);
        
  // Remove scalars if requested
  if (remove_scalars) RemoveAttribute(ScalarTag);
}

/*
--------------------------------------------------------------------------------
                           Find point at the same position
--------------------------------------------------------------------------------
*/

const LaserPoint *LaserPoints::MatchXYZ(const LaserPoint *point,
                                        const LaserPoint *proposed_start) const
{
  const LaserPoint *start, *same_point;
  int              match;

  if (proposed_start) start = proposed_start;
  else start = &*begin();

  switch (LargerXYZ(point, start)) {
    case -1: /* Point should be earlier, search backwards */
      for (same_point=start-1; same_point>=&*begin(); same_point--) {
        match = LargerXYZ(point, same_point);
        if (match == 0) return(same_point);
        if (match == 1) return(NULL);
      }
      return(NULL);

    case 1: /* Point should be later, search forward */
      for (same_point=start+1; same_point!=&*end(); same_point++) {
        match = LargerXYZ(point, same_point);
        if (match == 0) return(same_point);
        if (match == -1) return(NULL);
      }
      return(NULL);

    case 0: /* Point already found */
      return(start);
  }
/* Just to satisfy the compiler, the program should not come here */
  return(NULL);
}

LaserPoint *LaserPoints::MatchXYZ(LaserPoint *point, LaserPoint *proposed_start)
{
  LaserPoint *start, *same_point;
  int              match;

  if (proposed_start) start = proposed_start;
  else start = &*begin();

  switch (LargerXYZ(point, start)) {
    case -1: /* Point should be earlier, search backwards */
      for (same_point=start-1; same_point>=&*begin(); same_point--) {
        match = LargerXYZ(point, same_point);
        if (match == 0) return(same_point);
        if (match == 1) return(NULL);
      }
      return(NULL);

    case 1: /* Point should be later, search forward */
      for (same_point=start+1; same_point!=&*end(); same_point++) {
        match = LargerXYZ(point, same_point);
        if (match == 0) return(same_point);
        if (match == -1) return(NULL);
      }
      return(NULL);

    case 0: /* Point already found */
      return(start);
  }
/* Just to satisfy the compiler, the program should not come here */
  return(NULL);
}
