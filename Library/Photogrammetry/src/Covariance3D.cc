
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
 Collection of functions for class Covariance3D

 vMatrix Covariance3D::CovarianceMatrix() const     Return covariances as matrix
 void Covariance3D::CovarianceMatrix                Extract covariances from
   (const vMatrix &)                                a matrix

 Initial creation
 Author : George Vosselman
 Date   : 05-07-2001

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
#include "Covariance3D.h"
#include "vMatrix.h"

/*
--------------------------------------------------------------------------------
                          Declarations of C routines
--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                     Return covariances as a matrix
--------------------------------------------------------------------------------
*/

vMatrix Covariance3D::CovarianceMatrix() const
{
  vMatrix cov = vMatrix(3, 3);

  cov.Val(0,0) = var[0];
  cov.Val(1,1) = var[1];
  cov.Val(2,2) = var[2];
  cov.Val(0,1) = cov.Val(1,0) = var[3];
  cov.Val(0,2) = cov.Val(2,0) = var[4];
  cov.Val(1,2) = cov.Val(2,1) = var[5];

  return(cov);
}

/*
--------------------------------------------------------------------------------
                     Extract covariances from a matrix
--------------------------------------------------------------------------------
*/

void Covariance3D::CovarianceMatrix(const vMatrix &cov)
{
  var[0] = cov.Val(0,0);
  var[1] = cov.Val(1,1);
  var[2] = cov.Val(2,2);
  var[3] = cov.Val(0,1);
  var[4] = cov.Val(0,2);
  var[5] = cov.Val(1,2);
}
