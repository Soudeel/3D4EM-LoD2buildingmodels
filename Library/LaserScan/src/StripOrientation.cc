
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
 Collection of functions for class StripOrientation

 StripOrientation::StripOrientation                 Construct from flight path
   (const Positions3D &)
 StripOrientation::StripOrientation                 Construct from moments
   (double, double, double, double, double, double)
 void StripOrientation::Initialise()                Initialisation
 ObjectPoint StripOrientation::TerrainToStrip       Transform terrain to strip
   (const ObjectPoint &) const
 ObjectPoint StripOrientation::StripToTerrain       Transform strip to terrain
   (const ObjectPoint &) const
 void StripOrientation::Read(FILE *)                Read from strip meta file
 void STripOrientation::Write(FILE *, int) const    Write to strip meta file

 Initial creation
 Author : George Vosselman
 Date   : 05-07-2001

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
#include "stdmath.h"
#include "StripOrientation.h"
#include "Line2D.h"
#include "RotationParameter.h"
#include "vMatrix.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                           Construct from flight path
--------------------------------------------------------------------------------
*/

StripOrientation::StripOrientation(const Positions3D &flight_path)
{
  Positions3D::const_iterator first_point, last_point;
  double                      kappa;
  EulerRotation               rot;

  first_point = flight_path.begin();
  last_point  = flight_path.end()-1;
  vect() = (*first_point + *last_point) / 2.0;
  Z() = 0.0; // No transform for height
  kappa = atan2(last_point->Y() - first_point->Y(),
                last_point->X() - first_point->X());
  rotation() = EulerRotation(0.0, 0.0, kappa).to_matrix();
  known_orientation = 1;
}

/*
--------------------------------------------------------------------------------
                         Construct from moments of strip points
--------------------------------------------------------------------------------
*/

StripOrientation::StripOrientation(double m00, double m10, double m01,
                                   double m20, double m11, double m02)
{
  double mn20, mn11, mn02;

// Set strip centre

  X() = m10 / m00;
  Y() = m01 / m00;
  Z() = 0.0; // No transform for height

// Derive normalised second order moments

  mn20 = m20 - m10 * m10 / m00;
  mn11 = m11 - m10 * m01 / m00;
  mn02 = m02 - m01 * m01 / m00;

// Set the rotation matrix

  rotation() = EulerRotation(0.0, 0.0,
                             0.5 * atan2(2 * mn11, mn20 - mn02)).to_matrix();
  known_orientation = 1;
}

/*
--------------------------------------------------------------------------------
                                 Initialisation
--------------------------------------------------------------------------------
*/

void StripOrientation::Initialise()
{
  vect() = Vector3D(0.0, 0.0, 0.0);
  rotation() = Rotation3D();         // Identity matix
  known_orientation = 0;
}

/*
--------------------------------------------------------------------------------
           Transform a point from the terrain system to the strip system
--------------------------------------------------------------------------------
*/

ObjectPoint StripOrientation::TerrainToStrip(const ObjectPoint &terrainpt) const
{
  ObjectPoint strippt;

  strippt.Number() = terrainpt.Number();
  strippt.vect() = rotation().Transpose() * (terrainpt.vect() - vect());
  strippt.CovarianceMatrix(vMatrix(rotation().Transpose()) *
                           terrainpt.CovarianceMatrix() * vMatrix(rotation()));
  return(strippt);
}

/*
--------------------------------------------------------------------------------
           Transform a point from the strip system to the terrain system
--------------------------------------------------------------------------------
*/

ObjectPoint StripOrientation::StripToTerrain(const ObjectPoint &strippt) const
{
  ObjectPoint terrainpt;

  terrainpt.Number() = strippt.Number();
  terrainpt.vect() = rotation() * strippt.vect() + vect();
  terrainpt.CovarianceMatrix(vMatrix(rotation()) * strippt.CovarianceMatrix() *
                             vMatrix(rotation().Transpose()));
  return(terrainpt);
}

/*
--------------------------------------------------------------------------------
               Read orientation data from a strip meta data file
--------------------------------------------------------------------------------
*/

void StripOrientation::Read(FILE *fd)
{
  char          *buffer, *line, *keyword;
  int           keyword_length;
  double        pi, kappa;
  EulerRotation angles;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "xcentre", MAX(keyword_length, 7)))
          X() = BNF_Double(line);

        else if (!strncmp(keyword, "ycentre", MAX(keyword_length, 7)))
          Y() = BNF_Double(line);

        else if (!strncmp(keyword, "kappa", MAX(keyword_length, 5))) {
          pi = 4.0 * atan(1.0);
          kappa = BNF_Double(line) * pi / 180.0;
          angles = EulerRotation(0.0, 0.0, kappa);
          rotation() = angles.to_matrix();
        }

        else if (!strncmp(keyword, "endorientation", MAX(keyword_length, 14))) {
          free(buffer);
          known_orientation = 1;
          return;
        }

        else
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endorientation.\n");
}

/*
--------------------------------------------------------------------------------
               Write orientation data to a strip meta data file
--------------------------------------------------------------------------------
*/

void StripOrientation::Write(FILE *fd, int indent) const
{
  EulerRotation angles;
  double        pi, kappa;

  if (!known_orientation) return;
  BNF_Write_String(fd, "orientation", indent, NULL);
  BNF_Write_Double(fd, "xcentre", indent+2, X(), "%15.3f");
  BNF_Write_Double(fd, "ycentre", indent+2, Y(), "%15.3f");
  angles.from_matrix(rotation());
  pi = 4.0 * atan(1.0);
  kappa = angles[2] * 180.0 / pi;
  BNF_Write_Double(fd, "kappa", indent+2, kappa, "%17.3f");
  BNF_Write_String(fd, "endorientation", indent, NULL);
}
