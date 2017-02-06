
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



/*------ Pix_To_Metric: Interior orientation of image measurements ------*/


/*  Version: 1.0
       Date: 14-nov-94   
     Author: Frank A. van den Heuvel
      Input: Interior *par - parameters interior orientation (Database.h)
             double r,c  - image co-ordinates (pixel)
     Output: double *x,*y  - collinear co-ordinates (mm)

     Update: Rotation parameter added (to be used with scanned images)
    Version: 1.1
       Date: 18-may-95
     Author: George Vosselman

     Update: Sign of correction for distortions is changed.
             Therefore an iteration is introduced (correction is function of
             corected values). In this way the definition corresponds to the
             one used in SCAN-3.
    Version: 1.2
       Date: 20-feb-98
     Author: Frank van den Heuvel

     Update: Intor_Correct_Bingo added
     Remark: Bingo distortions are computed relative to the approximate principle point (rh,ch in pix.) !
    Version: 1.3
       Date: 29-nov-99
     Author: Frank van den Heuvel
*/

#include <stdio.h>
#include <math.h>
#include "Database.h"
#define MAXITER 99 /* maximum number of iterations */
#define EPS 1e-10  /* stop criterion for iteration */

void Pix_To_Metric(Interior *par, double r, double c, double *x, double *y)
{   
  double dx,dy,dx0,dy0;
  int iter;
  
  void Intor_Correct(Interior *,double, double, double *, double *);
  void Intor_Correct_Bingo(Interior *,double, double, double *, double *);
  
/*------ Co-ordinate system transformation ------*/
  *x = cos(par->rot) * par->spacing_c * (c - par->ch) +
       sin(par->rot) * par->spacing_r * (par->rh - r);
  *y = cos(par->rot) * par->spacing_r * (par->rh - r) -
       sin(par->rot) * par->spacing_c * (c - par->ch);

/*------ Corrections interior orientation ------*/
  iter = 0;
  dx0 = dy0 = 0;
  Intor_Correct(par,*x,*y,&dx,&dy);

  while ((fabs(dx-dx0)>EPS || fabs(dy-dy0)>EPS) && iter<MAXITER)
  {
    dx0=dx; dy0=dy;
    Intor_Correct(par,*x-dx0,*y-dy0,&dx,&dy);
    iter++;
  }
  if (iter==MAXITER) fprintf(stderr,"\n *** ERROR in Pix_To_Metric: maximum number of iterations reached!\n");

   *x -= dx;
   *y -= dy;

  /*------ Bingo parameters ------*/
  if (par->bingo[0]>0)
    {
      Intor_Correct_Bingo(par,*x,*y,&dx,&dy);
      *x -= dx + par->bingo_xh;
      *y -= dy + par->bingo_yh;
    }

}
