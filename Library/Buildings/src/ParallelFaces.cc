
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

 void LaserPoints::ParallelFaces                Simultaneously adjust         
   (PointNumberLists &, Planes &)               parallel faces 

 Initial creation
 Author : George Vosselman
 Date   : 27-04-2001

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
#include "digphot_arch.h"
#include <limits.h>
#include "Position2D.h"  // local
#include "LaserPoints.h" // local
#include "LineSegments2D.h"
#include "ObjectPoints2D.h"
#include "PointNumberLists.h"
#include "stdmath.h"
#include "VRML_io.h"

/*
--------------------------------------------------------------------------------
                    Estimate parallel and horizontal faces
--------------------------------------------------------------------------------
*/

void LaserPoints::ParallelFaces(PointNumberLists &faces,
                                Planes &planes,
                                double min_dist_planes,
                                double min_angle_planes,
                                double max_angle_horizontal)
{
  PointNumberLists::iterator face;
  Planes::iterator           plane, plane2;
  Plane                      new_plane;
  int                        i, i2, *plane_ref, *plane_size, plane_number;

  double degree = 4.0 * atan(1.0) / 180.0;

  for (plane=planes.begin(); plane!=planes.end(); plane++)
    printf("Plane %2d: %6.2f %6.2f %6.2f   %10.2f\n",
           plane->Number(), plane->Normal().X(),
           plane->Normal().Y(), plane->Normal().Z(), plane->Distance());

/* Estimate horizontal faces */

  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end();
       face++, plane++) {
    if (plane->IsHorizontal(max_angle_horizontal)) {
      *plane = FitPlane(face->PointNumberListReference(),
                        Vector3D(0, 0, 1), plane->Number());
      printf("Plane %d made horizontal\n", plane->Number());
    }
  }

/* Estimate parallel faces */

  /* Set up the pointers and size array */
  plane_ref  = (int *) malloc(planes.size() * sizeof(int));
  plane_size = (int *) malloc(planes.size() * sizeof(int));
  for (i=0, face=faces.begin(); face!=faces.end(); i++, face++) {
    plane_ref[i]  = i;
    plane_size[i] = face->size();
  }

  /* Adjust normal directions */
  for (i=0, plane=planes.begin(), face=faces.begin();
       plane!=planes.end(); i++, plane++, face++) {
    if (plane_ref[i] == i && !plane->IsHorizontal(0.01 * degree)) {
      for (i2=i+1, plane2=plane+1; plane2!=planes.end(); i2++, plane2++) {
        if (plane_ref[i2] == i2) {
          if (Angle(plane->Normal(), plane2->Normal()) < min_angle_planes) {
            plane->Normal() = ((plane->Normal() * plane_size[i] +
                                plane2->Normal() * plane_size[i2]) /
                               (plane_size[i] + plane_size[i2])).Normalize();
            printf("Common orientation plane %d and %d: %8.2f %8.2f %8.2f\n",
                   plane->Number(), plane2->Number(),
                   plane->Normal().X(), plane->Normal().Y(),
                   plane->Normal().Z());
            plane_size[i] += plane_size[i2];
            plane_ref[i2] = i;
          }
        }
      }
    }

    /* Fit plane with the adjusted normal direction */
    if (plane_ref[i] != i) {
      plane_number = plane->Number();
      *plane = planes[plane_ref[i]];
      plane->Number() = plane_number;
    }
    if (plane_ref[i] != i || plane_size[i] != face->size()) {
      new_plane = FitPlane(face->PointNumberListReference(), plane->Normal(),
                           plane->Number());
      *plane = new_plane;
    }
  }

/* Estimate collinear faces */

  /* Set up the pointers and size array */
  for (i=0, face=faces.begin(); face!=faces.end(); i++, face++) {
    plane_ref[i]  = i;
    plane_size[i] = face->size();
  }

  /* Adjust distances to origin */
  for (i=0, plane=planes.begin(), face=faces.begin();
       plane!=planes.end(); i++, plane++, face++) {
    if (plane_ref[i] == i) {
      for (i2=i+1, plane2=plane+1; plane2!=planes.end(); i2++, plane2++) {
        if (plane_ref[i2] == i2) {
          if (Angle(plane->Normal(), plane2->Normal()) < 0.01 * degree &&
              fabs(plane->Distance() - plane2->Distance()) < min_dist_planes) {
            plane->Distance() = (plane->Distance() * plane_size[i] +
                                 plane2->Distance() * plane_size[i2]) /
                                (plane_size[i] + plane_size[i2]);
            printf("Common distance plane %d and %d: %10.2f\n",
                   plane->Number(), plane2->Number(), plane->Distance());
            plane_size[i] += plane_size[i2];
            plane_ref[i2] = i;
          }
        }
      }
    }

    /* Resolve the reference to a plane with an adjusted distance */
    if (plane_ref[i] != i) {
      plane_number = plane->Number();
      *plane = planes[plane_ref[i]];
      plane->Number() = plane_number;
    }
  }
}

Plane LaserPoints::FitPlane(const PointNumberList &face,
                            const Vector3D &orientation,
                            int plane_number) const
{
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;
  double                          dist;
  Plane                           plane;

  dist = 0;
  for (node=face.begin(); node!=face.end(); node++) {
    point = begin() + node->Number();
    dist += orientation.DotProduct(point->Position3DRef());
  }

  plane.Normal() = orientation.Normalize();
  plane.Distance() = dist / face.size();
  plane.Number() = plane_number;
  return(plane);
}
