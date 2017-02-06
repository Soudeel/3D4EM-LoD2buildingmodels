
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
 Collection of functions for class StripErrors

 void StripErrors::Initialise()               Initialisation
 int StripErrors::NumErrorParms(int)          Number of error parameters
 ObjectPoint StripErrors::CorrectStripPoint   Correct distorted strip point
   (const ObjectPoint &, int) const
 ObjectPoint StripErrors::DistortStripPoint   Add distortion to correct point
   (const ObjectPoint &, int) const
 void StripErrors::PartialDerivatives         Partial derivatives of strip point
   (const ObjectPoint &, double *) const      with respect to the errors
 void StripErrors:Update(vMatrix &, int)       Update the error values

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
#include "StripErrors.h"
#include "vMatrix.h"

/*
--------------------------------------------------------------------------------
                            Initialisation
--------------------------------------------------------------------------------
*/

void StripErrors::Initialise()
{
  trans_error        = Vector3D(0.0, 0.0, 0.0);
  rot_error          = EulerRotation(0.0, 0.0, 0.0);
  rot_error_time_dep = EulerRotation(0.0, 0.0, 0.0);
}

/*
--------------------------------------------------------------------------------
                 Return the number of error parameters
--------------------------------------------------------------------------------
*/

int StripErrors::NumErrorParms(int error_model) const
{
  switch (error_model) {
    case  0: return(0); // No error parameters
    case  1: return(3); // RWS-MD model
    case  2: return(9); // linear offsets and drifts
    default: fprintf(stderr, "Error: invalid error model (%d).\n",
                     error_model);
  }
  return(0);
}

/*
--------------------------------------------------------------------------------
              Correct a strip point for the strip distortion
--------------------------------------------------------------------------------
*/

ObjectPoint StripErrors::CorrectStripPoint(const ObjectPoint &errorpt,
                                           int error_model) const
{
  ObjectPoint   goodpt;
  EulerRotation angles;
  Rotation3D    rot;

// This transformation is valid for error models 1 and 2

  if (error_model != 1 && error_model != 2) {
    fprintf(stderr, "Error: invalid error model (%d) in CorrectStripPoint.\n",
            error_model);
    exit(0);
  }

  goodpt.Number() = errorpt.Number();
  angles = EulerRotation(rot_error[0] + errorpt.X() * rot_error_time_dep[0],
                         rot_error[1] + errorpt.X() * rot_error_time_dep[1],
                         rot_error[2] + errorpt.X() * rot_error_time_dep[2]);
  rot = angles.to_matrix();
  goodpt.vect() = rot.rotation() * (errorpt.vect() - trans_error);
  goodpt.CovarianceMatrix(vMatrix(rot.rotation().Transpose()) *
                          errorpt.CovarianceMatrix() *
                          vMatrix(rot.rotation()));
  return(goodpt);
}

/*
--------------------------------------------------------------------------------
                      Distort a correct strip point
--------------------------------------------------------------------------------
*/

ObjectPoint StripErrors::DistortStripPoint(const ObjectPoint &goodpt,
                                           int error_model) const
{
  ObjectPoint   errorpt;
  EulerRotation angles;
  Rotation3D    rot;

// This transformation is valid for error models 1 and 2

  if (error_model != 1 && error_model != 2) {
    fprintf(stderr, "Error: invalid error model (%d) in DistortStripPoint.\n",
            error_model);
    exit(0);
  }

// Copy the point number

  errorpt.Number() = goodpt.Number();

// First take the X-coordinate of the good point as an approximation

  angles = EulerRotation(rot_error[0] + goodpt.X() * rot_error_time_dep[0],
                         rot_error[1] + goodpt.X() * rot_error_time_dep[1],
                         rot_error[2] + goodpt.X() * rot_error_time_dep[2]);
  rot = angles.to_matrix();
  errorpt.vect() = rot.rotation().Transpose() * goodpt.vect() + trans_error;

// Redo the computation with the approximate erroneous X-coordinate

  angles = EulerRotation(rot_error[0] + errorpt.X() * rot_error_time_dep[0],
                         rot_error[1] + errorpt.X() * rot_error_time_dep[1],
                         rot_error[2] + errorpt.X() * rot_error_time_dep[2]);
  rot = angles.to_matrix();
  errorpt.vect() = rot.rotation().Transpose() * goodpt.vect() + trans_error;
  errorpt.CovarianceMatrix(vMatrix(rot.rotation()) * goodpt.CovarianceMatrix() *
                           vMatrix(rot.rotation().Transpose()));
  return(errorpt);
}

/*
--------------------------------------------------------------------------------
    Partial derivatives of strip point with respect to the error parameters
    in the strip coordinate system
--------------------------------------------------------------------------------
*/

double *StripErrors::PartialDerivatives(const ObjectPoint &strippt,
                                        double *a_ptr, int error_model) const
{
  Vector3D      dvect;
  EulerRotation angles;
  Rotation3D    rot;
  double        *a;
  int           np, par;

// Check the error model number

  if (error_model < 1 || error_model > 2) {
    fprintf(stderr, "Error: Invalid error model (%d) in PartialDerivatives\n",
            error_model);
    exit(0);
  }

/* Compose the error angles and their rotation matrix from the
 * approximate values.
 */

  angles = EulerRotation(rot_error[0] + strippt.X() * rot_error_time_dep[0],
                         rot_error[1] + strippt.X() * rot_error_time_dep[1],
                         rot_error[2] + strippt.X() * rot_error_time_dep[2]);
  rot = angles.to_matrix();

/* Allocate memory for rows in A-matrix (if needed) */

  np = NumErrorParms(error_model);
  if (a_ptr) a = a_ptr;
  else {
    a = (double *) malloc(3 * np * sizeof(double));
    if (!a) return(NULL);
  }

/* Partial derivatives with respect to translation error in X */

  par = 0;
  if (error_model == 2) {
    a[par]      = -rot.R(0, 0);
    a[np+par]   = -rot.R(1, 0);
    a[2*np+par] = -rot.R(2, 0);
    par++;
  }

/* Partial derivatives with respect to translation error in Y */

  if (error_model == 2) {
    a[par]      = -rot.R(0, 1);
    a[np+par]   = -rot.R(1, 1);
    a[2*np+par] = -rot.R(2, 1);
    par++;
  }

/* Partial derivatives with respect to translation error in Z */

  if (error_model == 1 || error_model == 2) {
    a[par]      = -rot.R(0, 2);
    a[np+par]   = -rot.R(1, 2);
    a[2*np+par] = -rot.R(2, 2);
    par++;
  }

/* Partial derivatives with respect to rotation error in omega */

  if (error_model == 1 || error_model == 2) {
    dvect = angles.PartialDeriv(0) * (strippt.vect() - trans_error);
    a[par]      = dvect.X();
    a[np+par]   = dvect.Y();
    a[2*np+par] = dvect.Z();
    par++;
  }

/* Partial derivatives with respect to rotation error in phi */

  if (error_model == 1 || error_model == 2) {
    dvect = angles.PartialDeriv(1) * (strippt.vect() - trans_error);
    a[par]      = dvect.X();
    a[np+par]   = dvect.Y();
    a[2*np+par] = dvect.Z();
    par++;
  }

/* Partial derivatives with respect to rotation error in kappa */

  if (error_model == 2) {
    dvect = angles.PartialDeriv(2) * (strippt.vect() - trans_error);
    a[par]      = dvect.X();
    a[np+par]   = dvect.Y();
    a[2*np+par] = dvect.Z();
    par++;
  }

/* Partial derivatives with respect to time dependent rotation error in omega */

  if (error_model == 2) {
    a[par]      = a[3] * strippt.X();
    a[np+par]   = a[np+3] * strippt.X();
    a[2*np+par] = a[2*np+3] * strippt.X();
    par++;
  }

/* Partial derivatives with respect to time dependent rotation error in phi */

  if (error_model == 2) {
    a[par]      = a[4] * strippt.X();
    a[np+par]   = a[np+4] * strippt.X();
    a[2*np+par] = a[2*np+4] * strippt.X();
    par++;
  }

/* Partial derivatives with respect to time dependent rotation error in kappa */

  if (error_model == 2) {
    a[par]      = a[5] * strippt.X();
    a[np+par]   = a[np+5] * strippt.X();
    a[2*np+par] = a[2*np+5] * strippt.X();
    par++;
  }

// Debug output 
/*
  int i;
  double *aptr;
  for (i=0, aptr=a; i<3; i++) {
    for (int j=0; j<np; j++, aptr++) printf("%7.2f ", *aptr);
    printf("\n");
  }
*/

  return(a);
}

/*
--------------------------------------------------------------------------------
                       Update the error values
--------------------------------------------------------------------------------
*/

void StripErrors::Update(const vMatrix &increments, int error_model)
{
  switch (error_model) {
    case 1:
      trans_error.Z() += increments.Val(0,0);
      rot_error[0]    += increments.Val(1,0);
      rot_error[1]    += increments.Val(2,0);
      break;

    case 2:
      trans_error.X()       += increments.Val(0,0);
      trans_error.Y()       += increments.Val(1,0);
      trans_error.Z()       += increments.Val(2,0);
      rot_error[0]          += increments.Val(3,0);
      rot_error[1]          += increments.Val(4,0);
      rot_error[2]          += increments.Val(5,0);
      rot_error_time_dep[0] += increments.Val(6,0);
      rot_error_time_dep[1] += increments.Val(7,0);
      rot_error_time_dep[2] += increments.Val(8,0);
      break;

    default:
      fprintf(stderr, "Error: Invalid error model (%d)\n", error_model);
      exit(0);
  }     
}
