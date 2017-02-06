
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



/*------ Intor_Correct: Corrections Interior orientation ------*/

/*  Version: 1.0
       Date: 14-nov-94
     Author: Frank A. van den Heuvel
      Input: Interior par - parameters interior orientation (Database.h)
             double x,y   - approximate (measured photo co-ordinates (mm)
     Output: double dx,dy - corrections for x,y (mm)
*/

#include <stdio.h>
#include <math.h>
#include "Database.h"

void Intor_Correct(Interior *par, double x, double y, double *dx, double *dy)
{   
  double r2, myr2, mxr2;

/*------ Initialize corrections -----*/
  *dy = 0;
  *dx = 0;

/*------ Compute correction: affinity (shear) ------*/
  if (par->shear != 0)
  {
    *dx += par->shear * y;
  }

/*------ Compute correction: radial lens distortion ------*/
  r2 = y * y + x * x;
  mxr2 = x * r2;
  myr2 = y * r2;
  if (par->k1 != 0)
  {
    *dy += par->k1 * myr2;
    *dx += par->k1 * mxr2;
  }
  if (par->k2 != 0)
  {
    *dy += par->k2 * myr2 * r2;
    *dx += par->k2 * mxr2 * r2;
  }
  if (par->k3 != 0)
  {
    *dy += par->k3 * myr2 * r2*r2;
    *dx += par->k3 * mxr2 * r2*r2;
  }

/*------ Compute correction: decentering ------*/
  if (par->p1 != 0)
  {
    *dy += par->p1 * 2 * x * y;
    *dx += par->p1 * (r2 + 2 * x * x);
  }
  if (par->p2 != 0)
  {
    *dy += par->p2 * (r2 + 2 * y * y);
    *dx += par->p2 * 2 * x * y;
  }
}
/*------ Intor_Correct_Bingo: Corrections Interior orientation for Bingo parameters ------*/

/*  Version: 1.0
       Date: 24-nov-99
     Author: Frank A. van den Heuvel
      Input: Interior par - parameters interior orientation (Database.h)
             double x,y   - approximate (measured photo co-ordinates (mm)
     Output: double dx,dy - corrections for x,y (mm)
*/

void Intor_Correct_Bingo(Interior *par, double x1, double y1,
                         double *dx, double *dy)
{   
  double f, fr, rzfac, sqfac, r, rr;
  double x, y, xx, yy, xy;

/*------ Initialize corrections -----*/
  *dy = 0;
  *dx = 0;

/*------ Check for Bingo parameters ------*/
  if (par->bingo[0] == 0) return;

/*------ Scale to aerial image! ------*/
  x = x1 * par->bingo[0];
  y = y1 * par->bingo[0];
  xx = x * x;
  yy = y * y;
  xy = x * y;
  rr = yy + xx;
  r = sqrt(rr);
  if (r<1.0) return;
  fr = 1e-4 -1.21/rr;
  rzfac = r/115.0;
  sqfac = sqrt (rzfac);

/*------ Compute corrections ------*/
  if (par->bingo[1] != 0)
  {
    f = par->bingo[1] * (xx/rr-0.5) * sqfac * 0.1739131E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[2] != 0)
  {
    f = par->bingo[2] * (xy/rr) * sqfac * 0.1034094E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[3] != 0)
  {
    f = par->bingo[3] * (x/r) * sqfac * 0.7312144E-02;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[4] != 0)
  {
    f = par->bingo[4] * (y/r) * sqfac * 0.7312144E-02;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[5] != 0)
  {
    *dx -= par->bingo[5] * xy * 0.5346744e-4;
    *dy += par->bingo[5] * xx * 0.5346744e-4;
  }
  if (par->bingo[6] != 0)
  {
    *dx -= par->bingo[6] * yy * 0.5346744e-4;
    *dy += par->bingo[6] * xy * 0.5346744e-4;
  }
  if (par->bingo[7] != 0)
  {
    f = par->bingo[7] * (rr-15625) * 0.5680144E-06;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[8] != 0)
  {
    f = par->bingo[8] * (r-50) * (r-125) * 0.1450537E-05;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[9] != 0)
  {
    f = par->bingo[9] * (r-25) * (r-50) * (r-80) * (r-105) * 0.8328048E-10;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[10] != 0)
  {
    f = par->bingo[10] * (r-20) * (r-40) * (r-60) * (r-80) * (r-100) * (r-120) * 0.1552095E-13;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[11] != 0)
  {
    f = par->bingo[11] * (xx/rr * (xx/rr-1) + 0.125) *sqfac * 0.4136373E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[12] != 0)
  {
    f = par->bingo[12] * xy/rr * (xx/rr-0.5) * sqfac * 0.6102325E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[13] != 0)
  {
    f = par->bingo[13] * fr * (xx-yy) * rzfac * 0.2241912E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[14] != 0)
  {
    f = par->bingo[14] * fr * xy * 0.8569694E-02;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[15] != 0)
  {
    f = par->bingo[15] * fr * ((xx/rr -1.0) * xx * 8.0 + rr) * 0.4284847E-02;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[16] != 0)
  {
    f = par->bingo[16] * fr * (xx/rr -0.5) * xy * rzfac * 0.9544139E-01;
    *dx -= f * x;
    *dy -= f * y;
  }
  if (par->bingo[17] != 0)
  {
    *dx -= par->bingo[17] * x * 0.6148755e-2;
    *dy += par->bingo[17] * y * 0.6148755e-2;
  }
  if (par->bingo[18] != 0)
  {
    *dx -= par->bingo[18] * y * 0.6148755E-02;
    *dy -= par->bingo[18] * x * 0.6148755E-02;
  }
  if (par->bingo[19] != 0)
  {
    *dx -= par->bingo[19] * xy * 0.7561437E-04;
  }
  if (par->bingo[20] != 0)
  {
    *dy -= par->bingo[20] * xy * 0.7561437E-04;
  }
  if (par->bingo[21] != 0)
  {
    *dx -= par->bingo[21] * x * (yy - (rr - 12100) * 0.1258) * 0.7018528E-06;
    *dy += par->bingo[21] * y * (3000.0 + (rr -12100) * 0.1258) * 0.7018528E-06;
  }
  if (par->bingo[22] != 0)
  {
    *dx += par->bingo[22] * x * (3000.0 + (rr - 12100) * 0.1258) * 0.7018528E-06;
    *dy -= par->bingo[22] * y * (xx - (rr - 12100) * 0.1258) * 0.7018528E-06;
  }
  if (par->bingo[23] != 0)
  {
    *dx -= par->bingo[23] * xy * (yy-8100) * 0.1475403E-07;
  }
  if (par->bingo[24] != 0)
  {
    *dy -= par->bingo[24] * xy * (xx-8100) * 0.1475403E-07;
  }
  if (par->bingo[25] != 0)
  {
    *dx -= par->bingo[25] * x * (rr-10000) * 0.3737845e-6;
    *dy -= par->bingo[25] * y * (rr-10000) * 0.3737845e-6;
  }
  if (par->bingo[26] != 0)
  {
    *dx -= par->bingo[26] * x * (rr-10000) * (r-40) * 0.3047954e-8;
    *dy -= par->bingo[26] * y * (rr-10000) * (r-40) * 0.3047954e-8;
  }
  if (par->bingo[27] != 0)
  {
    *dy += par->bingo[27] * xx * 0.7561437e-4;
  }
  if (par->bingo[28] != 0)
  {
    *dy += par->bingo[28] * x * ( xx -12100) * 0.1952022e-5;
  }
  if (par->bingo[29] != 0)
  {
    *dx += par->bingo[29] * yy * 0.7561437E-04;
  }
  if (par->bingo[30] != 0)
  {
    *dy += par->bingo[30] * x * 0.8695653E-02;
  }
}
