
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
 Collection of functions for class ProjectiveTransform2D

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "ProjTransform2D.h"

/*
--------------------------------------------------------------------------------
                                C Routines
--------------------------------------------------------------------------------
*/

extern "C" int Put_ProjTrans2D(const char *, const double *);
extern "C" int Get_ProjTrans2D(const char *, double *);

/*
--------------------------------------------------------------------------------
            Read and write projective transformation parameters
--------------------------------------------------------------------------------
*/

int ProjectiveTransform2D::Write(const char *filename) const
{
  return(Put_ProjTrans2D(filename, par));
}

int ProjectiveTransform2D::Read(const char *filename)
{	
  return(Get_ProjTrans2D(filename, par));
}

/*
--------------------------------------------------------------------------------
            Derive the reverse projective transformation
--------------------------------------------------------------------------------
*/

ProjectiveTransform2D ProjectiveTransform2D::Inverse(int &invertible) const
{
  double z;
  ProjectiveTransform2D inv;

  z = par[4] * par[0] - par[1] * par[3];
  invertible =  (z != 0.0);
  if (invertible) {
    inv.par[0] = (par[4] - par[5]*par[7]) / z;
    inv.par[1] = (par[2]*par[7] - par[1]) / z;
    inv.par[2] = (par[1]*par[5] - par[2]*par[4]) / z;
    inv.par[3] = (par[5]*par[6] - par[3]) / z;
    inv.par[4] = (par[0] - par[2]*par[6]) / z;
    inv.par[5] = (par[2]*par[3] - par[0]*par[5]) / z;
    inv.par[6] = (par[3]*par[7] - par[4]*par[6]) / z;
    inv.par[7] = (par[1]*par[6] - par[0]*par[7]) / z;
  }
  return(inv);
}
