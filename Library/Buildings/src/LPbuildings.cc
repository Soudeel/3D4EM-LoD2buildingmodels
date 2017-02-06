
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
 Local functions for class LaserPoints

 int LaserPoints::ExtractFace(HoughSpace &,     Extract face from Hough space
   int, double, int, TINEdges &, int, Plane &, PointNumberList &)
 int LaserPoints::Adjacent                      Determine adjacency of points
   (PointNumberList &, int, TINEdges &)
 void LaserPoints::MergeFaces                   Merge faces
   (PointNumberLists &, Planes &, double,
    double, TINEdges &)
 void LaserPoints::FaceGrowing                  Grow all faces
   (PointNumberLists &, Planes &, double,
    TINEdges &)
 void LaserPoints::GrowFace(int,                Grow a face
    PointNumberLists &, Planes &, double,
    TINEdges &)
  double LaserPoints::FitPolynomial             Fit a polynomial
   (const Position2D &, const PointNumberList &,
    double &, int, const Vector2D &,
    double, double) const

 Initial creation
 Author : George Vosselman
 Date   : 10-07-2000

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
#include "Position2D.h" // local
#include "LineTopologies.h" // local
#include "Plane.h" // local
#include "DataBoundsLaser.h" // local
#include "LaserPoints.h" // local
#include "PointNumberLists.h"
#include "Adjustment.h"
#include "Rotation2D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                       Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void Update_Normal_Eq(double *, double, double, double *, double *,
                                 int);
extern "C" void Solve_Normal_Eq(double *, double *, int);

/*
--------------------------------------------------------------------------------
                    Extract the best face from the Hough space
--------------------------------------------------------------------------------
*/

void PrintPlane(char *string, Plane &plane, int numpts)
{
  printf("%s plane %d with %d points\n", string, plane.Number(), numpts);
  printf("  Slope X : %6.2f\n", plane.Normal().X() / plane.Normal().Z());
  printf("  Slope Y : %6.2f\n", plane.Normal().Y() / plane.Normal().Z());
  printf("  Distance: %6.2f\n", plane.Distance() / plane.Normal().Z());
}

int LaserPoints::ExtractFace(HoughSpace &houghspace, int local_max_window,
                             double max_dist_to_plane, int min_numpts,
                             TINEdges &edges, int best_face_number,
                             Plane &plane, PointNumberList &face_points)
{
  int                        numpts, face_number, largest_face_number,
                             point_number, found_large_face;
  double                     dist_bin;
  PointNumberLists           faces;
  PointNumberLists::iterator face, largest_face;
  LaserPoints::iterator      point;
  int                        print_plane = 0;

/* Some labels for internal usage.
 * Note: it is assumed that all unprocessed points have label 0.
 * Note: this function uses short labels only.
 */

  int INSIDE_PARTITION = 1;
  int NEAR_PLANE       = 2;
  int FIRST_FACE       = 3;
  int PROCESSED        = 255;

/* Get the plane with most points in the Hough space */

  plane = houghspace.BestPlane(&numpts, 0, local_max_window);
  if (numpts < 3) return(0);

/* Label all points within the bin of the approximate plane */

  dist_bin = houghspace.BinSize(3);
  numpts = Label(plane, dist_bin, INSIDE_PARTITION, INSIDE_PARTITION,
                 NEAR_PLANE, INSIDE_PARTITION);
  if (numpts < 3) {
    ReLabel(NEAR_PLANE, INSIDE_PARTITION);
    return(0);
  }
  if (print_plane) PrintPlane((char *) "Approximate", plane, numpts);

/* Fit a plane through the labeled points */

  plane = FitPlane(NEAR_PLANE, NEAR_PLANE);

/* Label all points within some distance of the adjusted plane */

  numpts = Label(plane, max_dist_to_plane, NEAR_PLANE, INSIDE_PARTITION,
                 NEAR_PLANE, INSIDE_PARTITION);
  if (print_plane) PrintPlane((char *) "Estimated", plane, numpts);

/* Determine the faces within the current plane */

  face_number = FIRST_FACE;
  for (point=begin(), point_number=0; point!=end(); point++, point_number++) {
    if (point->Label() == 2) {
      faces.push_back((Neighbourhood(PointNumber(point_number),
                                     edges, 1, 0, face_number)));
      face_number++;
    }
  }

/* Select the largest face */

  largest_face = faces.begin();
  largest_face_number = FIRST_FACE;
  for (face=faces.begin()+1, face_number = FIRST_FACE + 1;
       face!=faces.end();
       face++, face_number++) {
    if (face->size() > largest_face->size()) {
      largest_face = face;
      largest_face_number = face_number;
    }
  }

/* Fit a plane through the points of the largest face. */

  if (largest_face->size() >= min_numpts) {
    plane = FitPlane(largest_face_number, largest_face_number);
    plane.Number() = best_face_number;
    if (print_plane) PrintPlane((char *) "Face", plane, largest_face->size());
    found_large_face = 1;
  }
  else found_large_face = 0;

/* Reset the point labels to INSIDE_PARTITION of points that are not part of
 * the largest face.
 */

  for (face=faces.begin(), face_number=FIRST_FACE;
       face!=faces.end();
       face++, face_number++) {
    if (face != largest_face || !found_large_face)
      ReLabel(face_number, INSIDE_PARTITION);
  }

/* Transfer the largest face to the output variable */
  
  if (found_large_face) {
    face_points = *largest_face;

/* Remove all points of the largest face from the Hough space and relabel them
 * to PROCESSED.
 */

    for (point=begin(); point!=end(); point++) {
      if (point->Label() == largest_face_number) {
        houghspace.RemovePoint(point->X(), point->Y(), point->Z());
        point->Label(PROCESSED);
      }
    }
  }

/* Delete all faces */

  for (face=faces.begin(); face!=faces.end(); face++)
    face->erase(face->begin(), face->end());
  faces.erase(faces.begin(), faces.end());

  return(found_large_face);
}

/*
--------------------------------------------------------------------------------
                Determine adjacency of two connected components
--------------------------------------------------------------------------------
*/

int LaserPoints::Adjacent(PointNumberList &face, int label, TINEdges &edges)
{
  PointNumberList::iterator node, nb_node;
  TINEdgeSet                neighbours;
  LaserPoints::iterator     nb_point;

  for (node=face.begin(); node!=face.end(); node++) {
    neighbours = edges[node->Number()];
    for (nb_node=neighbours.begin(); nb_node!=neighbours.end(); nb_node++) {
      nb_point = begin() + nb_node->Number();
      if (nb_point->Label() == label) return(1);
    }
  }
  return(0);
}

/*
--------------------------------------------------------------------------------
            Merge faces if they are adjacent and in the same plane
--------------------------------------------------------------------------------
*/

/* This function is no longer used !! */

void LaserPoints::MergeFaces(PointNumberLists &faces, Planes &planes,
                             double max_dist, double min_perc,
                             TINEdges &edges)
{
  PointNumberLists::iterator face1, face2;
  Planes::iterator           plane1, plane2;
  Plane                      merged_plane;

  for (face1=faces.begin(), plane1=planes.begin();
       face1!=faces.end();
       face1++, plane1++) {
    if (plane1->Number() >= 0) { /* Plane not yet merged */
      for (face2=face1+1, plane2=plane1+1;
           face2!=faces.end();
           face2++, plane2++) {
        if (plane2->Number() >= 0) { /* Plane not yet merged */

          // Check adjacency of the faces
          if (Adjacent(face1->PointNumberListReference(), plane2->Number(),
                       edges)) {

            // Compute the merged plane
            merged_plane = FitPlane(plane1->Number(), plane2->Number());

            // See if points of face1 and face 2 lie in the merged plane
            if(PointsInPlane(face1->PointNumberListReference(), merged_plane,
                             max_dist, min_perc) &&
               PointsInPlane(face2->PointNumberListReference(), merged_plane,
                             max_dist, min_perc)) {
              printf("Merging faces %d and %d\n", plane1->Number(),
                     plane2->Number());
              Label(face2->PointNumberListReference(), plane1->Number());
              face1->insert(face1->end(), face2->begin(), face2->end());
              *plane1 = merged_plane;
              PrintPlane((char *) "Merged", *plane1, face1->size());
/* Iteration may be needed */
              plane2->Number() = -1; /* Mark plane as merged */
            }
          }
        }
      }
    }
  }

/* Remove the faces that have been merged with other faces */

  for (face1=faces.begin(), face2=face1, plane1=planes.begin(), plane2=plane1;
       face1!=faces.end();
       face1++, plane1++) {
    if (plane1->Number() >= 0) {
      if (face1 != face2) {
        *face2 = *face1;
        *plane2 = *plane1;
      }
      face2++;  plane2++;
    }
  }
  faces.erase(face2, faces.end());
  planes.erase(plane2, planes.end());

}

/*
--------------------------------------------------------------------------------
               Face growing using seeds from Hough transform
--------------------------------------------------------------------------------
*/

void LaserPoints::FaceGrowing(PointNumberLists &faces, Planes &planes,
                              double max_dist_to_plane, int min_numpts,
                              TINEdges &edges)
{
  int                        max_size, next_face_index;
  Planes::iterator           plane, large_plane;
  PointNumberLists::iterator face, large_face;

  printf("Faces before growing:\n");
  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end(); face++, plane++)
    printf("Face %d with %d points.\n", plane->Number(), face->size());

/* Set all plane labels to 0 (= not processed) */

  for (plane=planes.begin(); plane!=planes.end(); plane++) plane->Label() = 0;

/* Pick the largest face, until there are no more faces to process */

  do {
    max_size = 0;
    for (face=faces.begin(), plane=planes.begin();
         face!=faces.end();
         face++, plane++) {
      if (face->size() > max_size && plane->Label() == 0) {
        max_size = face->size();
        next_face_index = plane->Number();
      }
    }

/* Grow this face */

    if (max_size > min_numpts) {
      GrowFace(next_face_index, faces, planes, max_dist_to_plane, edges);
      plane = planes.begin() + next_face_index;
      plane->Label() = 1;
    }
  } while (max_size > min_numpts);

/* Delete all small remaining faces. Relabel all points in those faces
 * as unclassified.
 */

  for (face = large_face = faces.begin(), plane = large_plane = planes.begin();
       face!=faces.end();
       face++, plane++) {
    if (face->size() >= min_numpts) {
      if (face != large_face) {
        *large_face  = *face;
        *large_plane = *plane;
      }
      large_face++;  large_plane++;
    }
    else {
      printf("Deleting plane %d with %d points.\n", plane->Number(), face->size());
      // 65535 is used to indicate points have no label
      ReLabel(plane->Number(), 65535);
    }
  }
  faces.erase(large_face, faces.end());
  planes.erase(large_plane, planes.end());

  printf("Faces after growing:\n");
  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end(); face++, plane++)
    printf("Face %d with %d points.\n", plane->Number(), face->size());

/* Re-estimate the planes from the face points */

  for (face=faces.begin(), plane=planes.begin();
       face!=faces.end();
       face++, plane++)
    *plane = FitPlane(face->PointNumberListReference(), plane->Number());

}

void LaserPoints::GrowFace(int face_index, PointNumberLists &faces, 
                           Planes &planes, double max_dist_to_plane,
                           TINEdges &edges)
{
  PointNumberLists::iterator nb_face, current_face;
  PointNumberList            nb_nodes;
  Planes::iterator           nb_plane, current_plane;
  Vector3D                   local_normal;
  PointNumberList::iterator  node, nb_node;
  LaserPoints::iterator      nb_point;
  int                        nb_label, node_index, num_faces, add_point;
  double                     dist, nb_dist;
  
/* Some variables for quick referencing */

  current_face = faces.begin() + face_index;
  current_plane = planes.begin() + face_index;
  num_faces = faces.size();

/* Loop over all points of the face */

  for (node=current_face->begin(), node_index=0;
       node!=current_face->end();
       node++, node_index++) {

/* Try to extend the face to neighbouring points */

    nb_nodes = edges[node->Number()];
    for (nb_node=nb_nodes.begin(); nb_node!=nb_nodes.end(); nb_node++) {
      nb_point = begin() + nb_node->Number();
      nb_label = nb_point->Label();
      if (nb_label != face_index) {
        dist = ABS(nb_point->Distance(current_plane->PlaneReference()));
        if (dist < max_dist_to_plane) {
          add_point = (nb_label >= num_faces); // Add unlabeled points
          if (!add_point) {
            nb_plane = planes.begin() + nb_label;
            add_point = (nb_plane->Label() == 0); // Add from unprocessed face
            if (!add_point) {
              nb_dist = ABS(nb_point->Distance(nb_plane->PlaneReference()));
              local_normal = NormalAtPoint(*node, edges);
              // Add point if plane is nearest and normal fits better
              add_point = (dist < nb_dist &&
                           Angle(local_normal, current_plane->Normal()) <
                           Angle(local_normal, nb_plane->Normal()));
            }
            if (add_point) { // Remove point from neighbouring face
              nb_face = faces.begin() + nb_label;
              nb_face->DeletePoint(&*nb_node);
            }
          }
          if (add_point) {
            nb_point->Label(face_index);
            current_face->push_back(nb_node->NumberRef());
            node = current_face->begin() + node_index;
          }
        }
      }
    }
  }
}

/*
--------------------------------------------------------------------------------
             Robustly fit a polynomial through the points in a list
--------------------------------------------------------------------------------
*/

double LaserPoints::FitPolynomial(const Position2D &origin,
                                  const PointNumberList &list,
                                  double &coef, int method,
                                  const Vector2D &direction,
                                  double lambda1, double lambda2) const
{
  int numpar, iter;
  PointNumberList::const_iterator node;
  LaserPoints::const_iterator point;
  Adjustment adjustment;
  vMatrix a, y, Qyy, par;
  Rotation2D rot;
  Vector2D dv;
  double dx, dy, height, dist;

// Constrained polygnomial fitting is applied if method 3 is selected
// and the ratio of eigenvalues is above 3. In that case all points are rotated
// in the observation equations. If the ratio is below 3, this
// function is called with method 2.

  if (method == 3) {
    // Create the rotation matrix
    if (lambda1 > 3.0 * lambda2) {
      rot.R(0, 0) = direction.X();
      rot.R(0, 1) = direction.Y();
      rot.R(1, 0) = -direction.Y();
      rot.R(1, 1) = direction.X();
/*
      rot.R(0, 0) = -direction.Y();
      rot.R(0, 1) = direction.X();
      rot.R(1, 0) = -direction.X();
      rot.R(1, 1) = -direction.Y();
*/
    }
    // Fit a second order polynomial
    else
      return FitPolynomial(origin, list, coef, 2, direction, lambda1, lambda2);
  }

// Determine the number of surface parameters
  switch (method) {
    case 0: numpar = 1; break;
    case 3:
    case 1: numpar = 3; break;
    case 2: numpar = 6; break;
    default: fprintf(stderr, "Invalid fitting method %d\n", method);
             fprintf(stderr, "The method index should be 0, 1, 2, or 3.\n");
             return 0.0;
  }

  adjustment = Adjustment(numpar);
  a   = vMatrix(1, numpar);
  y   = vMatrix(1, 1);
  Qyy = vMatrix(1, 1); Qyy.Val(0) = 1.0;
  for (iter=0; iter<2; iter++) {
    if (iter == 1) par = adjustment.EstPar();
    for (node=list.begin(); node!=list.end(); node++) {
      point = begin() + node->Number();
      y.Val(0) = point->Z();
      if (method == 3) {
        dv = point->vect2D() - origin.vect();
        dx = rot.R(0,0) * dv.X() + rot.R(0,1) * dv.Y(); // Rotation not yet
        dy = rot.R(1,0) * dv.X() + rot.R(1,1) * dv.Y(); // defined in library!
      }
      else {
        dx = point->X() - origin.X();
        dy = point->Y() - origin.Y();
      }
      if (iter == 1) {
        height = 0;
        switch (method) {
          case 2: height += par.Val(3) * dx * dx + par.Val(4) * dx * dy +
                            par.Val(6) * dy * dy;
          case 1: height += par.Val(1) * dx + par.Val(2) * dy;
          case 0: height += par.Val(0);
                  break;
          case 3: height += par.Val(0) + par.Val(1) * dx + par.Val(2) * dx * dx;
                  break;
        }
        if (fabs(point->Z() - height) > 0.2) continue;
      }
      switch (method) {
        case 2: a.Val(5) = dy * dy;
                a.Val(4) = dx * dy;
                a.Val(3) = dx * dx;
        case 1: a.Val(2) = dy;
                a.Val(1) = dx;
        case 0: a.Val(0) = 1.0;
                break;
        case 3: a.Val(0) = 1.0;
                a.Val(1) = dx;
                a.Val(2) = dx * dx;
                break;
      }
/*
      dist = dx*dx +dy*dy;
      if (dist > 0.0) dist = sqrt(dist);
      if (dist > 0.5) Qyy.Val(0) = dist;
      else 
*/
        Qyy.Val(0) = 0.5;
      adjustment.AddObservations(a, y, Qyy);
    }
    adjustment.SolveEquationSystem();
  }
  height = adjustment.EstPar().Val(0);
  return (height);
}
