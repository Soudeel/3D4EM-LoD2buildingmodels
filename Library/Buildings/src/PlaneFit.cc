
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



#include "LaserPoints.h"

extern "C" void rg_(int *, int *, double *, double *, double *, int *,
                    double *, double *, double *, int *);

Position3D LaserPoints::CentreOfGravity(const PointNumberList &list) const
{
  Position3D                      centre;
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;

  for (node=list.begin(); node!=list.end(); node++) {
    point = begin() + node->Number();
    centre += point->Position3DRef();
  }
  centre /= (double) list.size();
  return centre;
}

Plane LaserPoints::FitPlane(const PointNumberList &list) const
{
  Position3D                      centre;
  Vector3D                        diff, normal;
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;
  double                          moments[3][3], tmp1[3], tmp2[3];
  double                          eigen_real[3], eigen_imag[3], eigen_vec[3][3];
  int                             dim = 3, matz=1, error, smallest, i, j;

  // Determine the centre of gravity
  centre = CentreOfGravity(list);

  // Calculate the moment matrix
  for (node=list.begin(); node!=list.end(); node++) {
    point = begin() + node->Number();
    diff = centre - point->Position3DRef();
    moments[0][0] += diff.X() * diff.X();
    moments[0][1] += diff.X() * diff.Y();
    moments[0][2] += diff.X() * diff.Z();
    moments[1][1] += diff.Y() * diff.Y();
    moments[1][2] += diff.Y() * diff.Z();
    moments[2][2] += diff.Z() * diff.Z();
  }
  moments[1][0] = moments[0][1];
  moments[2][0] = moments[0][2];
  moments[2][1] = moments[1][2];

  printf("Centralised moments\n");
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) printf("%20.4f", moments[i][j]);
    printf("\n");
  }

  // Determine the eigen values and eigen vectors with Eispack
  rg_(&dim, &dim, (double *) moments, eigen_real, eigen_imag, &matz, 
      (double *) eigen_vec, tmp1, tmp2, &error);

  // Select the smallest eigen value
  smallest = (eigen_real[0] < eigen_real[1]) ? 0 : 1;
  smallest = (eigen_real[smallest] < eigen_real[2]) ? smallest : 2;

  // Take the corresponding eigen vector as normal vector
  normal.X() = eigen_vec[smallest][0];
  normal.Y() = eigen_vec[smallest][1];
  normal.Z() = eigen_vec[smallest][2];
  normal = normal.Normalize();

  // Return the plane constructed from the centre of gravity and normal vector
  return Plane(centre, normal);
}

/*
Plane LaserPoints::FitPlane2(const PointNumberList &list) const
{
  Position3D                      centre;
  Vector3D                        normal;
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;
  double                          moments[3][3], tmp1[3], tmp2[3];
  double                          eigen_real[3], eigen_imag[3], eigen_vec[3][3];
  int                             dim = 3, matz=1, error, smallest, i, j;

  // Determine the centre of gravity
  centre = CentreOfGravity(list);

  // Calculate the moment matrix
  for (node=list.begin(); node!=list.end(); node++) {
    point = begin() + node->Number();
    moments[0][0] += point->X() * point->X();
    moments[0][1] += point->X() * point->Y();
    moments[0][2] += point->X() * point->Z();
    moments[1][1] += point->Y() * point->Y();
    moments[1][2] += point->Y() * point->Z();
    moments[2][2] += point->Z() * point->Z();
  }
  moments[1][0] = moments[0][1];
  moments[2][0] = moments[0][2];
  moments[2][1] = moments[1][2];

  printf("Moments\n");
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) printf("%20.4f", moments[i][j]);
    printf("\n");
  }

  // Centralised moments, corrected for centre of gravity
  for (i=0; i<3; i++)
    for (j=0; j<3; j++)
      moments[i][j] -= list.size() * centre.X(i) * centre.X(j);

  printf("Centralised moments\n");
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++) printf("%20.4f", moments[i][j]);
    printf("\n");
  }

  // Determine the eigen values and eigen vectors with Eispack
  rg_(&dim, &dim, (double *) moments, eigen_real, eigen_imag, &matz, 
      (double *) eigen_vec, tmp1, tmp2, &error);

  // Select the smallest eigen value
  smallest = (eigen_real[0] < eigen_real[1]) ? 0 : 1;
  smallest = (eigen_real[smallest] < eigen_real[2]) ? smallest : 2;

  // Take the corresponding eigen vector as normal vector
  normal.X() = eigen_vec[smallest][0];
  normal.Y() = eigen_vec[smallest][1];
  normal.Z() = eigen_vec[smallest][2];
  normal = normal.Normalize();

  // Return the plane constructed from the centre of gravity and normal vector
  return Plane(centre, normal);
}
*/
