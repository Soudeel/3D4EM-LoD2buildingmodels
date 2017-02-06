
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
 Collection of functions for class HoughSpace          

 HoughSpace(int, int, double, double,         Constructor with initialisation
            double, double)
 HoughSpace(int, int, int,                    Constructor with initialisation
            double, double, double,
            double, double, double)
 void HoughSpace::Initialise(int, int)        Initialisation
            double, double, double, double)
 void HoughSpace::Initialise(int, int, int,   Initialisation
            double, double, double,
            double, double, double)
 void HoughSpace::SetDimensions               Set the Hough space size
   (int, int, int)
 void HoughSpace::SetRange                    Set the ranges of the axes
   (double, double, double,
    double, double, double)
 void HoughSpace::Clear()                     Set all frequencies to zero
 void HoughSpace::ModifyPoint                 Add or remove a point
   (int, double, double, double)
 void HoughSpace::AddPoint                    Add a point
   (double, double, double)
 void HoughSpace::RemovePoint                 Remove a point
   (double, double, double)
 Plane HoughSpace::BestPlane(int *, int, int) Determine plane with most points
 double HoughSpace::BinSize(int)              Return bin size of variable
 void HoughSpace::ModifyGablePoint            Add or remove a gable roof point
   (int, const Position3D &, const Line2D &)
 
 Initial creation
 Author : George Vosselman
 Date   : 03-07-2000

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
#include <math.h>
#include "HoughSpace.h"
#include "stdmath.h"
#include "Line2D.h"

/*
--------------------------------------------------------------------------------
                     Constructors / Destructors 
--------------------------------------------------------------------------------
*/

HoughSpace::HoughSpace(int dimv1, int dimv2, double minv1, double minv2,
                       double maxv1, double maxv2, int equation)
{
  space = NULL;
  offX = offY = offZ = 0.0;
  sin1 = cos1 = sin2 = cos2 = NULL;
  Initialise(dimv1, dimv2, minv1, minv2, maxv1, maxv2, equation);
}

HoughSpace::HoughSpace(int dimv1, int dimv2, int dimv3,
                       double minv1, double minv2, double minv3,
                       double maxv1, double maxv2, double maxv3, int equation)
{
  space = NULL;
  offX = offY = offZ = 0.0;
  sin1 = cos1 = sin2 = cos2 = NULL;
  Initialise(dimv1, dimv2, dimv3, minv1, minv2, minv3, maxv1, maxv2, maxv3,
             equation);
}

/*
--------------------------------------------------------------------------------
                              Initialisation
--------------------------------------------------------------------------------
*/

void HoughSpace::Initialise(double *directions)
{
  int    i;
  double v;

  Erase();
  if (dim1 == 0 || dim2 == 0 || dim3 == 0) return;
  space = (int *) calloc(dim1 * dim2 * dim3, sizeof(int));
  if (!space) {
    fprintf(stderr,
            "Error allocating Hough space in HoughSpace::Initialise().\n");
    fprintf(stderr, "Requested dimension: %d x %d x %d = %d voxels\n",
            dim1, dim2, dim3, dim1 * dim2 * dim3);
    exit(0);
  }
  // Prepare sine and cosine tables
  if (eq == 1 || eq == 2) {
    if (sin1 != NULL) free(sin1);
    if (cos1 != NULL) free(cos1);
    sin1 = (double *) malloc(dim1 * sizeof(double));
    cos1 = (double *) malloc(dim1 * sizeof(double));
    if (eq == 1) {
      for (i=0, v=min1 + int1 / 2.0; i<dim1; i++, v+=int1)
        { sin1[i] = sin(v); cos1[i] = cos(v); }
      if (min3 != max3) { // Three dimensional case
        if (sin2 != NULL) free(sin2);
        if (cos2 != NULL) free(cos2);
        sin2 = (double *) malloc(dim2 * sizeof(double));
        cos2 = (double *) malloc(dim2 * sizeof(double));
        for (i=0, v=min2 + int2 / 2.0; i<dim2; i++, v+=int2)
          { sin2[i] = sin(v); cos2[i] = cos(v); }
      }
    }
    else { // eq == 2
      for (i=0; i<dim1; i++)
        { sin1[i] = sin(directions[i]); cos1[i] = cos(directions[i]); }
    }
  }
}

void HoughSpace::Initialise(int dimv1, int dimv2, double minv1, double minv2,
                            double maxv1, double maxv2, int equation)
{
  eq = equation;
  SetDimensions(dimv1, dimv2, 1);
  SetRange(minv1, minv2, 0.0, maxv1, maxv2, 0.0);
  Initialise();
}

void HoughSpace::Initialise(int dimv1, double *directions, int dimv2,
                            double minv2, double maxv2)
{
  eq = 2;
  SetDimensions(dimv1, dimv2, 1);
  SetRange(0.0, minv2, 0.0, 0.0, maxv2, 0.0);
  Initialise(directions);
}
  
void HoughSpace::Initialise(int dimv1, int dimv2, int dimv3,
                            double minv1, double minv2, double minv3,
                            double maxv1, double maxv2, double maxv3,
                            int equation)
{
  eq = equation;
  SetDimensions(dimv1, dimv2, dimv3);
  SetRange(minv1, minv2, minv3, maxv1, maxv2, maxv3);
  Initialise();
}

void HoughSpace::SetDimensions(int dimv1, int dimv2, int dimv3)
{
  dim1 = dimv1;  dim2 = dimv2;  dim3 = dimv3;
}

void HoughSpace::SetRange(double minv1, double minv2, double minv3,
                          double maxv1, double maxv2, double maxv3)
{
  min1 = minv1;  min2 = minv2;  min3 = minv3;
  max1 = maxv1;  max2 = maxv2;  max3 = maxv3;
  int1 = (max1 - min1) / dim1;
  int2 = (max2 - min2) / dim2;
  int3 = (max3 - min3) / dim3;
}

void HoughSpace::SetOffsets(double offX1, double offY1, double offZ1)
{
  offX = offX1;  offY = offY1;  offZ = offZ1; 
}

void HoughSpace::Clear()
{
  memset((void *) space, 0, dim1 * dim2 * dim3 * sizeof(int));
}

/*
--------------------------------------------------------------------------------
                         Add or remove a point in a 3D space
--------------------------------------------------------------------------------
*/

void HoughSpace::ModifyPoint(int factor, double X, double Y, double Z)
{
  int    i1, i2, i3, *sptr;
  double v1, v2, v3;

  for (i1=0, v1=min1 + int1 / 2.0, sptr=space; i1<dim1; i1++, v1+=int1) {
    for (i2=0, v2=min2 + int2 / 2.0; i2<dim2; i2++, v2+=int2, sptr+=dim3) {
      switch (eq) {
        case 0: // v1, v2 slope in X and Y, v3 distance to origin
          v3 = (Z-offZ) - (X-offX) * v1 - (Y-offY) * v2;
          break;
        case 1: // v1, v2 angles, v3 distance to origin
          v3 = sin1[i1] * (sin2[i2] * (X-offX) + cos2[i2] * (Y-offY)) +
               cos1[i1] * (Z-offZ);
          break;
        default:
          printf("Error: Invalid equation type %d in HoughSpace::ModifyPoint\n",
                 eq);
          exit(0);
      }
      i3 = (int) ((v3 - min3) / int3);
      if (i3 >= 0 && i3 < dim3) (*(sptr + i3)) += factor;
    }
  }
}
/*
--------------------------------------------------------------------------------
                         Add or remove a point in a 2D space
--------------------------------------------------------------------------------
*/

void HoughSpace::ModifyPoint(int factor, double X, double Y)
{
  int    i1, i2, *sptr;
  double v1, v2, v2min, v2max;

  for (i1=0, v1=min1 + int1 / 2.0, sptr=space; i1<dim1;
       i1++, v1+=int1, sptr+=dim2) {
    switch (eq) {
      case 0: // v1 slope in X, v2 distance to origin
        v2 = (Y-offY) - (X-offX) * v1;
        break;
      case 1: // v1 angle, v2 distance to origin X cos a + Y sin a = d
      case 2: // same, but with preferred directions instead of direction range
        v2 = cos1[i1] * (X-offX) + sin1[i1] * (Y-offY);
        if (i1 == 0) {
          v2min = v2max = v2;
        }
        else {
          if (v2 < v2min) v2min = v2;
          if (v2 > v2max) v2max = v2;
        }
        break;
      default:
        printf("Error: Invalid equation type %d in HoughSpace::ModifyPoint\n",
               eq);
        exit(0);
    }
    i2 = (int) ((v2 - min2) / int2);
    if (i2 >= 0 && i2 < dim2) (*(sptr + i2)) += factor;
  }
}

/*
--------------------------------------------------------------------------------
                Determine the n'th maximum in the Hough space
--------------------------------------------------------------------------------
*/

int HoughSpace::Maximum(int num_ignore, int local_max_window,
                        int &i1max, int &i2max, int &i3max)
{
  int i1, i2, i3, *count, ignored, numpts, local_max_window1;
  
  if (eq == 2) local_max_window1 = 1;
  else local_max_window1 = local_max_window;
  
  for (ignored=0; ignored<=num_ignore; ignored++) {

// Find the next maximum

    numpts = -1;
    for (i1=0, count=space; i1<dim1; i1++) {
      for (i2=0; i2<dim2; i2++) {
        for (i3=0; i3<dim3; i3++, count++) {
          if (*count > numpts) {
            numpts = *count;
            i1max = i1;  i2max = i2;  i3max = i3;
          }
        }
      }
    }

// Ignore the maximum and its local surrounding if this is not the last loop

    if (ignored != num_ignore) {
      for (i1=MAX(0, i1max-local_max_window1/2);
           i1<MIN(dim1, i1max + local_max_window1/2);
           i1++) {
        for (i2=MAX(0, i2max-local_max_window/2);
             i2<MIN(dim2, i2max + local_max_window/2);
             i2++) {
          for (i3=MAX(0, i3max-local_max_window/2);
               i3<MIN(dim3, i3max + local_max_window/2);
               i3++) {
            count = space + i1 * dim2 * dim3 + i2 * dim3 + i3;
            if (*count > 0) *count = -(*count);
          }
        }
      }
    }

// Reset all frequencies to positive numbers 

    else if (num_ignore > 0) {
      for (i1=0, count=space; i1<dim1; i1++) {
        for (i2=0; i2<dim2; i2++) {
          for (i3=0; i3<dim3; i3++, count++) {
            if (*count < 0) *count = -(*count);
          }
        }
      }
    }
  }
  return(numpts);
}

/*
--------------------------------------------------------------------------------
                     Determine the n'th best line
--------------------------------------------------------------------------------
*/

Line2D HoughSpace::BestLine(int *numpts, int num_ignore, int local_max_window)
{
  int      i1, i2, i1max, i2max, i3max, *count, ignored;
  double   v1, v2, length;
  Line2D   line;

  *numpts = Maximum(num_ignore, local_max_window, i1max, i2max, i3max);
  if (*numpts > 0) {
    v2 = min2 + int2 / 2.0 + i2max * int2;
    switch (eq) {
      case 0: // v1 slope in X, v2 distance to origin
        v1 = min1 + int1 / 2.0 + i1max * int1;
        length = sqrt(1.0 + v1 * v1);
        printf("Warning: line not corrected for offset\n");
        line = Line2D(v1/length, -1.0/length, -v2/length);
        break;
      case 1: // v1 angle, v2 distance to origin
      case 2: // same, but with preferred directions instead of direction range
        line = Line2D(sin1[i1max], cos1[i1max],
                      v2 + offX * cos1[i1max] + offY * sin1[i1max]);
        break;
      default:
        printf("Error: Invalid equation type %d in HoughSpace::BestLine\n",
               eq);
        exit(0);
    }
  }
  
  return(line);
}

// Used for debugging

void HoughSpace::Print()
{
  int i1, i2, *count;
  for (i1=0, count=space; i1<dim1; i1++) {
    for (i2=0; i2<dim2; i2++, count++) printf("%d ", *count);
    printf("\n");
  }
}

/*
--------------------------------------------------------------------------------
                     Determine the n'th best plane
--------------------------------------------------------------------------------
*/

Plane HoughSpace::BestPlane(int *numpts, int num_ignore, int local_max_window)
{
  int      i1, i2, i3, i1max, i2max, i3max, *count, ignored;
  double   v1, v2, v3, length;
  Plane    plane;
  Vector3D normal;
  bool debug = false;
  
  *numpts = Maximum(num_ignore, local_max_window, i1max, i2max, i3max);

  if (*numpts > 0) {
    v3 = min3 + int3 / 2.0 + i3max * int3;
    switch (eq) {
      case 0: // v1, v2 slope in X and Y, v3 distance to origin
        v1 = min1 + int1 / 2.0 + i1max * int1;
        v2 = min2 + int2 / 2.0 + i2max * int2;
        normal = Vector3D(-v1, -v2, 1);
        length = normal.Length();
        plane.Normal() =  normal.Normalize();
        // Distance including correction for offset
        plane.Distance() = (v3 - v1 * offX - v2 * offY + offZ) / length;
        break;
      case 1: // v1, v2 angles, v3 distance to origin
        plane.Normal() = Vector3D(sin1[i1max] * sin2[i2max],
                                  sin1[i1max] * cos2[i2max],
                                  cos1[i1max]);
        plane.Distance() = v3 +
                           offX * sin1[i1max] * sin2[i2max] +
                           offY * sin1[i1max] * cos2[i2max] +
                           offZ * cos1[i1max];
        break;
      default:
        printf("Error: Invalid equation type %d in HoughSpace::BestPlane\n",
               eq);
        exit(0);
    }
  }
  
  if (debug) {
  	if (plane.Offset() == NULL) printf("Returning plane without offset\n");
  	else printf("Returning plane with offset\n");
  	plane.Print();
  }
  
  return(plane);
}

/*
--------------------------------------------------------------------------------
                     Determine the n'th best gable
--------------------------------------------------------------------------------
*/

void HoughSpace::BestGable(int &numpts, int num_ignore, int local_max_window)
{
  int      i1, i2, i3, i1max, i2max, i3max, *count, ignored;
  double   v1, v2, v3;
  Plane    plane;
  Vector3D normal;
  
  numpts = Maximum(num_ignore, local_max_window, i1max, i2max, i3max);

  if (numpts > 0) {
    v1 = min1 + int1 / 2.0 + i1max * int1;
    v2 = min2 + int2 / 2.0 + i2max * int2;
    v3 = min3 + int3 / 2.0 + i3max * int3;
    printf("Hough: #pts %4d slope %6.2f (%6.2f-%6.2f) u %6.2f (%6.2f-%6.2f) h %6.2f (%6.2f-%6.2f)\n",
           numpts, tan(v1), tan(min1 + int1 / 2.0),
           tan(min1 + int1 / 2.0 + (dim1 - 1) * int1),
           v2, min2 + int2 / 2.0, min2 + int2 / 2.0 + (dim2 - 1) * int2,
           v3, min3 + int3 / 2.0, min3 + int3 / 2.0 + (dim3 - 1) * int3);
  }
}

/*
--------------------------------------------------------------------------------
                         Return bin size of variable
--------------------------------------------------------------------------------
*/

double HoughSpace::BinSize(int dimension)
{
  switch (dimension) {
    case 1 : return(int1);
    case 2 : return(int2);
    case 3 : return(int3);
    default: fprintf(stderr,
                     "HoughSpace::BinSize : dimension should be 1, 2, or 3.\n");
             exit(0);
  }
  return(0); // To avoid compiler warnings about function not returning value
}

/*
--------------------------------------------------------------------------------
                      Add or remove a gable roof point
--------------------------------------------------------------------------------
*/

void HoughSpace::ModifyGablePoint(int factor, const Position3D &pos,
                                  const Line2D &ref_line)
{
  int    i1, i2, i3, *sptr;
  double v1, v2, v3, u;

  for (i1=0, v1=min1 + int1 / 2.0, sptr=space;
       i1<dim1; i1++) {
    v1 = tan(min1 + int1 / 2.0 + i1 * int1);      // v1 is roof slope angle
    for (i2=0, v2=min2 + int2 / 2.0;              // v2 is ridge u coordinate
         i2<dim2; i2++, v2+=int2, sptr+=dim3) {
      u = ref_line.DistanceToPointSigned(Position2D(pos.X(), pos.Y()));
      v3 = pos.Z() + v1 * fabs(u - v2);           // v3 is ridge height
      i3 = (int) ((v3 - min3) / int3);
      if (i3 >= 0 && i3 < dim3) (*(sptr + i3)) += factor;
    }
  }
}

/*
--------------------------------------------------------------------------------
                      Erase Hough space and tables
--------------------------------------------------------------------------------
*/
void HoughSpace::Erase()
{
  if (space) {free(space); space = NULL;}
  if (sin1) {free(sin1); sin1 = NULL;}
  if (cos1) {free(cos1); cos1 = NULL;}
  if (sin2) {free(sin2); sin2 = NULL;}
  if (cos2) {free(cos2); cos2 = NULL;}
}
