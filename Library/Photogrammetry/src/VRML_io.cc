
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




/* Collection of functions for output to a VRML file:
 *
 * FILE *VRML_Open(char *)
 * void VRML_Close(FILE *)
 * void VRML_Write_Begin_Of_PointList(FILE *)
 * void VRML_Write_End_Of_PointList(FILE *)
 * void VRML_Set_Colour(float, float, float)
 * void VRML_Scalar_To_Colour(float, float *, float *, float *)
 * void VRML_Write_Cylinder(FILE *, Position3D &, Position3D &, double);
 *
 * Many other functions for writing points, lines, etc. are member functions
 * of the point and line classes.
 *
 * Last Update: 08-06-1999
 * Author     : George Vosselman
 */

/*
--------------------------------------------------------------------------------
                             Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <math.h>
#include "Position3D.h"

/*
--------------------------------------------------------------------------------
                             Open and close
--------------------------------------------------------------------------------
*/

FILE *VRML_Open(const char *filename)
{
  FILE *fd;

/* Open the file */

  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening VRML file %s\n", filename);
  }
  else {

/* Write the header */

    fprintf(fd, "#VRML V1.0 ascii\n");
    fprintf(fd, "# Produced by library functions in CppInterface\n");
    fprintf(fd, "ShapeHints {\n");
    fprintf(fd, "  faceType UNKNOWN_FACE_TYPE\n");
    fprintf(fd, "  vertexOrdering COUNTERCLOCKWISE\n");
    fprintf(fd, "}\n");
    fprintf(fd, "Separator \n{\n");
  }
  return(fd);
}

FILE *VRML2_Open(const char *filename)
{
  FILE *fd;

/* Open the file */

  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening VRML 2.0 file %s\n", filename);
  }

/* Write the header */

  else {
    fprintf(fd, "#VRML V2.0 utf8\n");
    fprintf(fd, "# Produced by library functions in FRSstd\n");
  }
  return(fd);
}

void VRML_Close(FILE *fd)
{
  fprintf(fd, "}\n");
  fclose(fd);
}

void VRML2_Close(FILE *fd)
{
  fclose(fd);
}

/*
--------------------------------------------------------------------------------
                             Begin and end of lists
--------------------------------------------------------------------------------
*/

void VRML_Write_Begin_Of_PointList(FILE *fd)
{
  fprintf(fd, "Coordinate3 {\n point [\n");
}

void VRML_Write_End_Of_PointList(FILE *fd)
{
  fprintf(fd, " ] }\n");
}

/*
--------------------------------------------------------------------------------
                             Colour settings
--------------------------------------------------------------------------------
*/

void VRML_Set_Colour(FILE *fd, float red, float green, float blue)
{
  fprintf(fd, "Material { diffuseColor  %4.3f %4.3f %4.3f }\n",
	  red, green, blue);
}

void VRML_Scalar_To_Colour(float scalar, float *red, float *green, float *blue)
{
  if (scalar < 1.0/3.0) {
    *red   = 1.0 - 3 * scalar;
    *green = 3 * scalar;
    *blue  = 0.0;
  }
  else if (scalar < 2.0/3.0) {
    *red   = 0.0;
    *green = 1.0 - 3 * (scalar-1.0/3.0);
    *blue  = 3 * (scalar-1.0/3.0);
  }
  else {
    *red   = 3 * (scalar - 2.0/3.0);
    *green = 0.0;
    *blue  = 1.0 - 3 * (scalar - 2.0/3.0);
  }
}

/*
--------------------------------------------------------------------------------
                             Write a cylinder
--------------------------------------------------------------------------------
*/

void VRML_Write_Cylinder(FILE *fd, Position3D &point1, Position3D &point2,
                         double radius)
{
  Vector3D normal;
  double   length, norm, angle;

/* Calculate cylinder parameters */

  Vector3D dir = point2 - point1;
  length = dir.SqLength();
  if (length > 0.0) {
    length = sqrt(length);
    normal = Vector3D(-dir.Z(), 0.0, dir.X());
    norm = normal.SqLength();
    if (norm > 0.0) {
      norm = sqrt(norm);
      normal /= norm;
      angle = -1.0 * acos(dir.Y() / length);
    }
    else {
      normal.Y() = 1.0;
      angle = 0.0;
    }
  }
  else {
    normal = Vector3D(0.0, 1.0, 0.0);
    angle = 0.0;
  }

/* Write the cylinder */

  fprintf(fd, "Separator {Transform {translation %10.2f %10.2f %10.2f\n",
          (point1.X() + point2.X()) / 2.0, (point1.Y() + point2.Y()) / 2.0,
          (point1.Z() + point2.Z()) / 2.0);
  fprintf(fd, "                      rotation %8.5f %8.5f %8.5f %8.5f}\n",
          normal.X(), normal.Y(), normal.Z(), angle);
  fprintf(fd, "           Cylinder {radius %.2f height %.2f}}\n",
          radius, length);
}
