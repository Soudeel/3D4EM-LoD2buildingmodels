
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
 Collection of functions for the class Adjustment

 void Adjustment::Initialise(int, int)              Initialisation
 int Adjustment::AddObservations(                   Add one or more observations
   const Matrix &, const Matrix &, const Matrix &)    
 int Adjustment::SolveEquationSystem()              Solve normal equations

 Initial creation
 Author : George Vosselman
 Date   : 06-07-2001

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
#include <strings.h>
#include <math.h>
#include "Adjustment.h"
#include "stdmath.h"
#include "digphot_arch.h"

/*
--------------------------------------------------------------------------------
                 Allocation and initialisation of matrices
--------------------------------------------------------------------------------
*/

void Adjustment::Initialise(int npar)
{
  ata.~vMatrix();   ata = vMatrix(npar, npar);
  aty.~vMatrix();   aty = vMatrix(npar, 1);
  x.~vMatrix();     x   = vMatrix(npar, 1);
  Qxx.~vMatrix();   Qxx = vMatrix(npar, npar);
  Clear();
}

void Adjustment::Clear()
{
  ata.Zero();  aty.Zero();  x.Zero();  Qxx.Zero();
}

/*
--------------------------------------------------------------------------------
                 Add observations to the normal equation system
--------------------------------------------------------------------------------
*/

int Adjustment::AddObservations(vMatrix &a, vMatrix &y, vMatrix &Qyy)
{
  vMatrix weight, at, atw;
  double sum;
  int    ir, ic;

// Check if the rows of the A matrix has the correct number of elements

  if (a.NumColumns() != ata.NumRows()) {
    fprintf(stderr, "Error: Number of columns of A (%d) does not equal the size of the normal matrix (%d)\n", a.NumColumns(), ata.NumRows());
    return(0);
  }

/* Convert the covariance matrix into the weight matrix */

  weight = vMatrix(a.NumRows(), a.NumRows());
  weight.InvertByGauss(Qyy);

/* Multiply the transposed design matrix rows with the weight matrix */

  atw = at.Transpose(a) * weight;

/* Update all elements of the ata matrix */

  for (ir=0; ir<ata.NumRows(); ir++) {
    for (ic=0; ic<ata.NumColumns(); ic++) {
//  for (ic=0; ic<=ir; ic++) {

/* Product of a row from atw with a column of a */

      sum = MultRowColumn(atw, ir, a, ic);
      
/* Update of two terms */

      ata.Val(ir, ic) += sum;
//    if (ir != ic) ata.Val(ic, ir) += sum;
    }
  }

/* Update all elements of the aty vector */

  for (ir=0; ir<aty.NumRows(); ir++)
    aty.Val(ir, 0) += MultRowColumn(atw, ir, y, 0);

  return(1);
}

/*
--------------------------------------------------------------------------------
                     Solve normal equation system
--------------------------------------------------------------------------------
*/

int Adjustment::SolveEquationSystem()
{
  double cond;

/* Invert the normal equation matrix */

  Qxx.InvertByGauss( ata );
  printf("Warning: using Gauss elimination instead of SVD in\n"
         "Adjustment::SolveEquationSystem()\n");
//  Qxx.Inverse(ata, cond);
//printf("Condition of normal matrix: %5.2e\n", cond);

/* Calculate the solution */

  x = Qxx * aty;
  return(1);
}
