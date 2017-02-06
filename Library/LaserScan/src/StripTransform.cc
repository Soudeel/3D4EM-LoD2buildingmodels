
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
 Collection of functions for class StripTransform

 ObjectPoint StripTransform::TerrainToStrip       Transform terrain to strip
   (const ObjectPoint &, int) const
 ObjectPoint StripTransform::StripToTerrain       Transform strip to terrain
   (const ObjectPoint &, int) const
 double *StripTransform::PartialDerivatives       Partial derivatives of
   (const ObjectPoint &, double *) const          strip point to errors in
                                                  terrain coordinate system
 ObjectPoint StripTransform::CorrectTerrainPoint  Correct observed terrain
   (const ObjectPoint &, int) const               point for strip errors

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
#include "StripTransform.h"

/*
--------------------------------------------------------------------------------
           Transform a point from the terrain system to the strip system
--------------------------------------------------------------------------------
*/

ObjectPoint StripTransform::TerrainToStrip(const ObjectPoint &terrainpt,
                                           int error_model) const
{
  ObjectPoint strippt;

  strippt = orientation.TerrainToStrip(terrainpt);
  if (error_model) strippt = errors.DistortStripPoint(strippt, error_model);
  return(strippt);
}

/*
--------------------------------------------------------------------------------
           Transform a point from the strip system to the terrain system
--------------------------------------------------------------------------------
*/

ObjectPoint StripTransform::StripToTerrain(const ObjectPoint &strippt,
                                           int error_model) const
{
  ObjectPoint terrainpt, strippt2;

  if (error_model) {
    strippt2 = errors.CorrectStripPoint(strippt, error_model);
    terrainpt = orientation.StripToTerrain(strippt2);
  }
  else terrainpt = orientation.StripToTerrain(strippt2);
  return(terrainpt);
}

/*
--------------------------------------------------------------------------------
    Partial derivatives of strip point with respect to the error parameters
    in the terrain coordinate system
--------------------------------------------------------------------------------
*/

double *StripTransform::PartialDerivatives(const ObjectPoint &strippt,
                                           double *a_ptr, int error_model) const
{
  Vector3D      dvect;
  double        *a;
  int           par, np;

/* Derive partial derivatives in the strip coordinate system */

  a = errors.PartialDerivatives(strippt, a_ptr, error_model);

/* Rotate the partial derivatives into the terrain coordinate system */

  np = errors.NumErrorParms(error_model);
  for (par=0; par<np; par++) {

    // Take a column out of the A matrix
    // This is the partial derivative vector with respect to an error parameter
    dvect.X() = a[par];
    dvect.Y() = a[np+par];
    dvect.Z() = a[2*np+par];

    // Rotate this difference vector from the strip to the terrain system
    dvect = orientation.rotation() * dvect;

    // Put the partial derivatives back into the A matrix
    a[par]      = dvect.X();
    a[np+par]   = dvect.Y();
    a[2*np+par] = dvect.Z();
  }

  return(a);
}

/*
--------------------------------------------------------------------------------
         Correct observed terrain point for (approximate) strip errors
--------------------------------------------------------------------------------
*/

ObjectPoint StripTransform::CorrectTerrainPoint(const ObjectPoint &obspt,
                                                int error_model) const
{
// Transform the observed terrain point into the strip without an error
// correction. Then transform it back with an error correction.

  return(StripToTerrain(TerrainToStrip(obspt, 0), error_model));
}
