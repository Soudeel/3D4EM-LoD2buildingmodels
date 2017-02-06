
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
 Collection of functions for strip adjustment in the
 classes LaserBlock and LaserUnit

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
#include "LaserBlock.h"
#include "Adjustment.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                         The main strip adjustment function
--------------------------------------------------------------------------------
*/

int LaserBlock::AdjustStrips(int error_model, int correct_points,
                             const char *adjustment_output_file)
{
  LaserBlock::iterator strip;
  Adjustment adjustment;
  int        num_unknowns, convergence, iteration;
  double     sigma0prior, sigma0;
  FILE       *fd;

/*----------------------------- The preparations -----------------------------*/

// Open the output file

  fd = fopen(adjustment_output_file, "w");
  if (!fd) {
    fprintf(stderr, "Error opening output file %s\n", adjustment_output_file);
    return(0);
  }
  fprintf(fd, "Adjustment of block %s\n\n", name);
  fprintf(fd, "Number of strips: %d\n", size());
  fprintf(fd, "Error model     : %d\n", error_model);

// Read the block control points

  if (!ReadControlPoints()) return(0);
  fprintf(fd, "Number of control points: %d\n\n", control_points.size());

  fprintf(fd, "Strip information\n");

  for (strip=begin(); strip!=end(); strip++) {

// Check the data organisation

   if (strip->DataOrganisation() != StripWise) {
     fprintf(stderr, "Error: Unit %d is not a strip.\n", strip->Name());
     return(0);
   }

// Read the flight path information and extract the strip orientation
// Note that the flight path information is erased after extracting the strip
// orientation. This information might be needed in future error models.

    if (!strip->ReadFlightPath()) {
      fprintf(stderr, "Error reading flight path information of strip %s\n",
              strip->Name());
      return(0);
    }
    strip->Transform().Orientation() = StripOrientation(strip->FlightPath());
    strip->EraseFlightPath();

// Read the measured tie and control points

    if (!strip->ReadMeasurements(1, 1)) return(0);

// Output strip information

    fprintf(fd, "Strip %d (%s)\n", strip->StripNumber(), strip->Name());
    strip->Transform().Orientation().Write(fd, 2);
    fprintf(fd, "  Number of tie points    : %d\n",
            strip->TiePointMeasurements().size());
    fprintf(fd, "  Number of control points: %d\n\n",
            strip->ControlPointMeasurements().size());
  }

/*------------------------- The parameter estimation -------------------------*/

// Initialise the normal equation system

  num_unknowns = begin()->Transform().Errors().NumErrorParms(error_model);
  adjustment.Initialise(size() * num_unknowns);

// Estimate standard deviation from residuals

  sigma0prior = SigmaNaught(error_model);
  printf("Before adjustment: sigma_0 = %6.3f\n", sigma0prior);

// Start the iterations

  convergence = iteration = 0;
  do {
    adjustment.Clear();
    iteration++;

// Add the tie point observations

    AddTiePointObs(adjustment, error_model);

// Add the control point observations

    AddControlPointObs(adjustment, error_model);

// Add regularisation

// Solve equation system

    if (!adjustment.SolveEquationSystem()) {
      fprintf(stderr, "Error solving equation system in iteration %d\n",
              iteration);
      return(0);
    }

// Update the parameters of the strip transformations

    UpdateStripErrors(adjustment.EstPar(), error_model);
    printf("Estimated increments\n");
    adjustment.EstPar().Print("%9.4f");
/*
    printf("%9.4f  %9.4f\n", adjustment.EstPar().Val(0,0),
           begin()->Transform().Errors().Offsets().Z());
    printf("%9.4f  %9.4f\n", adjustment.EstPar().Val(1,0),
           begin()->Transform().Errors().RotError()[0]);
    printf("%9.4f  %9.4f\n", adjustment.EstPar().Val(2,0),
           begin()->Transform().Errors().RotError()[1]);
*/

// Estimate standard deviation from residuals

    sigma0 = SigmaNaught(error_model);

// Set convergence measure

    convergence = (sigma0 / sigma0prior > 0.999);
    if (iteration == 1) convergence = 0;
    sigma0prior = sigma0;
    printf("After iteration %d: sigma_0 = %6.3f\n", iteration, sigma0);

  } while (!convergence && iteration < 20);

/*--------------------- The adjustment of the coordinates --------------------*/

  if (correct_points) {
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                         Read the measured tie and control points
--------------------------------------------------------------------------------
*/

int LaserUnit::ReadMeasurements(int tie, int control)
{
  int success;

  if (tie && tie_points_file) {
    success = tie_points.Read(tie_points_file);
    if (!success) {
      fprintf(stderr, "Error reading measured tie points of strip %s\n", name);
      return(success);
    }
  }
  if (control && control_points_file) {
    success = control_points.Read(control_points_file);
    if (!success) {
      fprintf(stderr, "Error reading measured control points of strip %s\n",
              name);
      return(success);
    }
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                            Read the block control points
--------------------------------------------------------------------------------
*/

int LaserBlock::ReadControlPoints()
{
  int success;

  if (control_points_file) {
    success = control_points.Read(control_points_file);
    if (!success) {
      fprintf(stderr, "Error reading block control points from file %s\n",
              control_points_file);
      return(0);
    }
  }
  else {
    fprintf(stderr, "Error: no control points available for block %s\n",
            Name());
    return(0);
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                            Calculate sigma naught
--------------------------------------------------------------------------------
*/

double LaserBlock::SigmaNaught(int error_model)
{
  LaserBlock::const_iterator strip1, strip2;  
  ObjectPoints::const_iterator tiept1, tiept2, ctrlpt_s;
  ObjectPoint        refpt1, refpt2;
  ControlPoints::const_iterator ctrlpt;
  double             sigma0=0, zdif;
  int                numpts=0;

  for (strip1=begin(); strip1!=end(); strip1++) {

// Compare all measured tie points to tie points of other strips

    for (tiept1=strip1->TiePointMeasurements().begin();
         tiept1!=strip1->TiePointMeasurements().end(); tiept1++) {
      refpt1 = strip1->Transform().CorrectTerrainPoint(tiept1->ObjectPointRef(),
                                           error_model);
      for (strip2=strip1+1; strip2!=end(); strip2++) {
        for (tiept2=strip2->TiePointMeasurements().begin();
             tiept2!=strip2->TiePointMeasurements().end(); tiept2++) {
          if (refpt1.Number() == tiept2->Number()) {
            refpt2 = strip2->Transform().
                     CorrectTerrainPoint(tiept2->ObjectPointRef(), error_model);
            numpts++;
            zdif = refpt1.Z() - refpt2.Z();
            sigma0 += zdif * zdif;
          }
        }
      }
    }

// Compare all measured control points to the block control data

    for (ctrlpt_s=strip1->ControlPointMeasurements().begin();
         ctrlpt_s!=strip1->ControlPointMeasurements().end(); ctrlpt_s++) {
      refpt1 = strip1->Transform().
               CorrectTerrainPoint(ctrlpt_s->ObjectPointRef(), error_model);
      for (ctrlpt=control_points.begin();
           ctrlpt!=control_points.end(); ctrlpt++) {
        if (refpt1.Number() == ctrlpt->Number()) {
          numpts++;
          zdif = refpt1.Z() - ctrlpt->Z();
          sigma0+= zdif * zdif;
        }
      }
    }
  }

  if (numpts) return(sqrt(sigma0/numpts));
  return(0.0);
}

/*
--------------------------------------------------------------------------------
         Add the tie point observations to the normal equation system
--------------------------------------------------------------------------------
*/

int LaserBlock::AddTiePointObs(Adjustment &adjustment, int error_model) const
{
  int    num_obs=0, num_error_parms, num_obs_per_point;
  vMatrix a; // rows of the A-matrix
  vMatrix obs; // observations
  vMatrix asub; // Partial derivatives of one point w.r.t. error parameters
  double *asub_values;
  vMatrix covar; // Covariance matrix of observation
  LaserBlock::const_iterator strip1, strip2;  
  ObjectPoints::const_iterator tiept1, tiept2;
  ObjectPoint        refpt1, refpt2, strippt1, strippt2;
  
// The number of observation equations varies per model

  if (error_model == 1) num_obs_per_point = 1;
  else num_obs_per_point = 3;

// Memory allocation

  a = vMatrix(num_obs_per_point, adjustment.NumberOfUnknowns());
  obs = vMatrix(num_obs_per_point, 1);
  num_error_parms = begin()->Transform().Errors().NumErrorParms(error_model);
  asub_values = (double *) malloc(num_error_parms * 3 * sizeof(double));

// Start loops over strips and tie points

  for (strip1=begin(); strip1!=end(); strip1++) {
    for (tiept1=strip1->TiePointMeasurements().begin();
         tiept1!=strip1->TiePointMeasurements().end(); tiept1++) {
      for (strip2=strip1+1; strip2!=end(); strip2++) {
        for (tiept2=strip2->TiePointMeasurements().begin();
             tiept2!=strip2->TiePointMeasurements().end(); tiept2++) {
          if (tiept1->Number() == tiept2->Number()) {
            num_obs++;
  
// Derive the observation:
// - correct the terrain points for already estimated errors
// - take the coordinate differences as observation

            refpt1 = strip1->Transform().
                     CorrectTerrainPoint(tiept1->ObjectPointRef(), error_model);
            refpt2 = strip2->Transform().
                     CorrectTerrainPoint(tiept2->ObjectPointRef(), error_model);
//          printf("%7.2f %7.2f %7.2f %7.2f\n", tiept1->Z(), refpt1.Z(),
//                 tiept2->Z(), refpt2.Z());
            if (num_obs_per_point == 1) {
              obs.Val(0,0) = refpt1.Z() - refpt2.Z();
            }
            else {
              obs.Val(0,0) = refpt1.X() - refpt2.X();
              obs.Val(1,0) = refpt1.Y() - refpt2.Y();
              obs.Val(2,0) = refpt1.Z() - refpt2.Z();
            }

// Compose the covariance matrix. Unlike the coordinates, the covariance
// matrices are not corrected for already estimated errors.

//          covar = tiept1->CovarianceMatrix() + tiept2->CovarianceMatrix();
            covar = vMatrix(num_obs_per_point, num_obs_per_point);
            covar.Unit();

// Compose the rows of the A-matrix

            a.Zero();  // Set all elements to zero

            // Transform the first point to the strip system
            strippt1 = strip1->Transform().
                       TerrainToStrip(tiept1->ObjectPointRef(), 0);

            // Get the partial derivatives
            strip1->Transform().PartialDerivatives(strippt1, asub_values,
                                                   error_model);

            // and put them in the correct place of the A-matrix rows
            asub = vMatrix(asub_values, 3, num_error_parms);
            asub * (-1.0); // Change of signs of A-matrix coefficients
            if (num_obs_per_point == 1)
              a.Insert(asub, 2, 0, 1, num_error_parms,
                       0, strip1->StripNumber() * num_error_parms);
            else
              a.Insert(asub, 0, strip1->StripNumber() * num_error_parms);
            
            // Do the same for the second point, but with opposite signs
            strippt2 = strip2->Transform().
                       TerrainToStrip(tiept2->ObjectPointRef(), 0);
            strip2->Transform().PartialDerivatives(strippt2, asub_values,
                                                   error_model);
            asub = vMatrix(asub_values, 3, num_error_parms);
            if (num_obs_per_point == 1)
              a.Insert(asub, 2, 0, 1, num_error_parms,
                       0, strip2->StripNumber() * num_error_parms);
            else
              a.Insert(asub, 0, strip2->StripNumber() * num_error_parms);

// Add this observation to the normal equation system

            adjustment.AddObservations(a, obs, covar);
          }
        }
      }
    }
  }

  return(num_obs);
}

/*
--------------------------------------------------------------------------------
         Add the control point observations to the normal equation system
--------------------------------------------------------------------------------
*/

int LaserBlock::AddControlPointObs(Adjustment &adjustment,
                                   int error_model) const
{
  int    num_obs=0, num_error_parms, num_obs_per_point;
  vMatrix a; // Rows of the A-matrix
  vMatrix obs; // Observations
  vMatrix asub; // Partial derivatives of one point w.r.t. error parameters
  double *asub_values;
  vMatrix covar; // Covariance matrix of observation
  LaserBlock::const_iterator strip;  
  ObjectPoints::const_iterator obspt;
  ObjectPoint        refpt, strippt;
  ControlPoints::const_iterator ctrlpt;
  
// The number of observation equations varies per model

  if (error_model == 1) num_obs_per_point = 1;
  else num_obs_per_point = 3;

// Memory allocation

  a = vMatrix(num_obs_per_point, adjustment.NumberOfUnknowns());
  obs = vMatrix(num_obs_per_point,1);
  num_error_parms = begin()->Transform().Errors().NumErrorParms(error_model);
  asub_values = (double *) malloc(num_error_parms * 3 * sizeof(double));

  for (strip=begin(); strip!=end(); strip++) {
    for (obspt=strip->ControlPointMeasurements().begin();
         obspt!=strip->ControlPointMeasurements().end(); obspt++) {
      for (ctrlpt=control_points.begin();
           ctrlpt!=control_points.end(); ctrlpt++) {
        if (obspt->Number() == ctrlpt->Number()) {
          num_obs++;
  
// Derive the observation:
// - correct the observed terrain point for already estimated errors
// - take the coordinate differences as observation

          refpt = strip->Transform().
                  CorrectTerrainPoint(obspt->ObjectPointRef(), error_model);
          if (num_obs_per_point == 1)
            obs.Val(0,0) = refpt.Z() - ctrlpt->Z();
          else {
            obs.Val(0,0) = refpt.X() - ctrlpt->X();
            obs.Val(1,0) = refpt.Y() - ctrlpt->Y();
            obs.Val(2,0) = refpt.Z() - ctrlpt->Z();
          }

// Compose the covariance matrix. Unlike the coordinates, the covariance
// matrices are not corrected for already estimated errors.
// Currently, the control points are not stochastic.

//        covar = obspt->CovarianceMatrix();
          covar = vMatrix(3,3);
          covar.Unit();

// Compose the rows of the A-matrix

          a.Zero();  // Set all elements to zero

          // Transform the observed point to the strip system
          strippt = strip->Transform().
                    TerrainToStrip(obspt->ObjectPointRef(), 0);

          // Get the partial derivatives
          strip->Transform().PartialDerivatives(strippt, asub_values,
                                                error_model);

          // and put them in the correct place of the A-matrix rows
          asub = vMatrix(asub_values, 3, num_error_parms);
          asub * (-1.0); // Change of signs of A-matrix coefficients
          if (num_obs_per_point == 1)
            a.Insert(asub, 2, 0, 1, num_error_parms,
                     0, strip->StripNumber() * num_error_parms);
          else
            a.Insert(asub, 0, strip->StripNumber() * num_error_parms);
            
// Add this observation to the normal equation system

          adjustment.AddObservations(a, obs, covar);
        }
      }
    }
  }
  return(num_obs);
}

/*
--------------------------------------------------------------------------------
         Add the control point observations to the normal equation system
--------------------------------------------------------------------------------
*/

void LaserBlock::UpdateStripErrors(const vMatrix &increments, int error_model)
{
  LaserBlock::iterator strip;
  int       num_unknowns;
  vMatrix    strip_increments;

// Create a matrix for the increments of a single strip

  num_unknowns = begin()->Transform().Errors().NumErrorParms(error_model);
  strip_increments = vMatrix(num_unknowns, 1);

// Copy the appropriate increments into this matrix and update the strip

  for (strip=begin(); strip!=end(); strip++) {
    strip_increments.Insert(increments,
                            strip->StripNumber() * num_unknowns, 0,
                            num_unknowns, 1, 0, 0);
    strip->UpdateErrors(strip_increments, error_model);
  }
}
