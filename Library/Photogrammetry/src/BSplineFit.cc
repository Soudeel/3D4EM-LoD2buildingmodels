/*
                     Copyright 2014 University of Twente
 
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

#include <stdio.h>
#include <string.h>
#include "BSplineFit.h"
#include "normal_equations.h"

// Spline initialisation

void BSplineFit::Initialise(int spline_order, double start, double end, int numk,
                            bool erase)
{
  if (numk < 2) {
  	printf("Error: minimum number of knots is 2\n");
  	return;
  }
  knot_interval_size = (end - start) / (numk - 1);
  Initialise(spline_order, start, end, knot_interval_size, erase);
}


// Spline initialisation

void BSplineFit::Initialise(int spline_order, double start, double end,
                            double interval, bool erase)
{
  order               = spline_order;
  curve_start         = start;
  curve_end           = end;
  knot_interval_size  = interval;
  num_knots           = (int) ((end - start - 0.0001) / interval) + 2;
  num_splines         = num_knots + order - 2;
  num_obs             = 0;
  if (erase) Erase();
  spline_coefficients = (double *) malloc(num_splines * sizeof(double));
  ata                 = (double *) calloc(num_splines * num_splines, sizeof(double));
  aty                 = (double *) calloc(num_splines, sizeof(double));
  a_row               = (double *) malloc(num_splines * sizeof(double));
  if (!spline_coefficients || !ata || !aty || !a_row) {
  	printf("Error allocating memory in BSplineFit::Initialise(...)\n");
  	exit(0);
  }
}


// Erase allocated arrays

void BSplineFit::Erase()
{
  if (spline_coefficients) free(spline_coefficients);
  if (ata) free(ata);
  if (aty) free(aty);
  if (a_row) free(a_row);
}


// Recursive definition of unit B-spline (private)

double BSplineFit::UnitBSplineValue(double location, int interval,
                                    int recursion_order) const
{
  if (recursion_order == 1) {
  	if (location >= (double) interval &&
	    location < (double) interval + 1.0) return 1.0;
	else return 0.0;
  }
  
  return UnitBSplineValue(location, interval, recursion_order - 1) *
         (location - (double) interval) / ((double ) recursion_order - 1.0)  +
         UnitBSplineValue(location, interval + 1, recursion_order - 1) *
		 ((double) interval + (double) recursion_order - location) /
		 ((double) recursion_order - 1.0);
}


// Derivative of unit B-spline (private)

double BSplineFit::UnitBSplineDerivativeValue(double location, int interval,
                                              int recursion_order, int derivative) const
{
  if (derivative == 1) {
  	return UnitBSplineValue(location, interval, recursion_order-1) -
  	       UnitBSplineValue(location, interval+1, recursion_order-1);
  }
  return UnitBSplineDerivativeValue(location, interval, recursion_order-1, derivative-1) -
         UnitBSplineDerivativeValue(location, interval+1, recursion_order-1, derivative-1);
}


// Index of first B-spline used at a position

int BSplineFit::FirstBSplineIndex(double location) const
{
  int spline_index;
  
  spline_index = (int) ((location - curve_start) / knot_interval_size);
  if (spline_index < 0) spline_index = 0;

  return spline_index;
}
    

// B-spline at position in curve for a given spline index

double BSplineFit::BSplineValue(double location, int index) const
{
  double unit_location;
  
  unit_location = (location - curve_start) / knot_interval_size + (double) (order - 1 - index);
  if (unit_location < 0.0 || unit_location >= (double) order) return 0.0;
  return UnitBSplineValue(unit_location, 0, order);
}


// B-spline derivative at position in curve for a given spline index

double BSplineFit::BSplineDerivative(double location, int index, int derivative) const
{
  double unit_location;
  
  unit_location = (location - curve_start) / knot_interval_size + (double) (order - 1 - index);
  if (unit_location < 0.0 || unit_location >= (double) order) return 0.0;
  return UnitBSplineDerivativeValue(unit_location, 0, order, derivative);
}

// B-spline at position in curve for a given spline start location

double BSplineFit::BSplineValue(double location, double bspline_start_location) const
{
  double unit_location;

  unit_location = (location - curve_start - bspline_start_location) / knot_interval_size;
  if (unit_location < 0.0 || unit_location >= (double) order) return 0.0;
  return UnitBSplineValue(unit_location, 0, order);
}


bool BSplineFit::AddObservation(double t, double v)
{
  int    j, spline_index;
  
  // Initialise all coefficients
  memset((void *) a_row, 0, num_splines * sizeof(double));
  
  // Set the coefficients of these splines
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
  	  a_row[spline_index] = BSplineValue(t, spline_index);
  }
  num_obs++;
  
  // Update the normal equation system
  Update_Normal_Eq(a_row, v, 1.0, ata, aty, num_splines);
  
  return true;
}

bool BSplineFit::AddConstraint(double t, double v, int derivative, double weight)
{
  int    j, spline_index;
  
  // Initialise all coefficients
  memset((void *) a_row, 0, num_splines * sizeof(double));
  
  // Set the coefficients of these splines
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
  	  a_row[spline_index] = BSplineDerivative(t, spline_index, derivative);
  }
  num_obs++;
  
  // Update the normal equation system
  Update_Normal_Eq(a_row, v, weight, ata, aty, num_splines);
}

// Estimate spline coefficients

int BSplineFit::FitSpline()
{
  double rcond;
  int i, j;
  
  if (num_obs < num_splines) return 1; // Insufficient observations
  
  // Solve equation system. Use Cholesky for band matrices.
  Solve_Normal_Eq_Cholesky_Band(ata, aty, num_splines, num_splines, order);
//  Solve_Normal_Eq(ata, aty, num_splines);

/* Code for checking correlations
  Invert_And_Solve_Normal_Eq(ata, aty, num_splines, &rcond);
  printf("Condition number %.2e\n", rcond);
  Convert_To_Correlations(ata, num_splines);
  printf("Correlations\n");
  for (i=0; i<num_splines; i++) {
    for (j=0; j<num_splines; j++)
      printf("%7.2f ", ata[i*num_splines+j]);
    printf("\n");
  }
*/  
  
  // Transfer coefficients
  for (int i=0; i<num_splines; i++) spline_coefficients[i] = aty[i];
  
  return 0;
}


// B-Spline value at specified location

double BSplineFit::Value(double t) const
{
  int    j, spline_index;
  double value=0.0;
  
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
      value += spline_coefficients[spline_index] * BSplineValue(t, spline_index);
  }
  
  return value;
}

// B-Spline derivative value at specified location

double BSplineFit::Derivative(double t, int derivative) const
{
  int    j, spline_index;
  double value=0.0;
  
  for (j=0, spline_index=FirstBSplineIndex(t); j<order; j++, spline_index++) {
  	if (spline_index < num_splines)
      value += spline_coefficients[spline_index] * 
	           BSplineDerivative(t, spline_index, derivative);
  }
  
  return value;
}

// Extrapolate spline to a new range
int BSplineFit::Extrapolate(double new_start, double new_end)
{
  double value0, value1;
  
  if (order > 2) return 1; // Only implemented for constant linear splines
  if (num_knots > 2) return 2; // Only implemented for at most two knots
  if (spline_coefficients == NULL) return 5; // No spline defined yet
  
  // Determine new spline coefficients
  if (order == 1) {
    curve_start = new_start;
    curve_end   = new_end;
  }
  else if (order == 2) {
  	if (num_knots == 2) {
  	  SetLinear(new_start, new_end,
  	            spline_coefficients[0] + (new_start - curve_start) *
  	            (spline_coefficients[1] - spline_coefficients[0]) /
			    (curve_end - curve_start),
	            spline_coefficients[0] + (new_end - curve_start) *
  	            (spline_coefficients[1] - spline_coefficients[0]) /
			    (curve_end - curve_start));
  	}
  	else {
  	  printf("Extrapolate not implemented for %d knots\n", num_knots);
  	  return 3;
  	}
  }
  else {
  	printf("Invalid spline order %d\n", order);
  	return 4;
  }
  
  return 0;
}

void BSplineFit::SetLinear(double start, double end,
                           double value_start, double value_end)
{
  Initialise(2, start, end, 2, (spline_coefficients!=NULL));
  spline_coefficients[0] = value_start;
  spline_coefficients[1] = value_end;
}

