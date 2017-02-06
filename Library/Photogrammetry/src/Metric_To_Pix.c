
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



/*------ Metric_To_Pix: Inverse interior orientation of image measurements ------*/

/*  Version: 1.0
       Date: 14-nov-94
     Author: Frank A. van den Heuvel
      Input: Interior *par - parameters interior orientation (Database.h)
             double x,y  - collinear co-ordinates (mm)
     Output: double *r,*c  - image co-ordinates (pixel)

     Update: Rotation parameter added (to be used with scanned images)
    Version: 1.1
       Date: 18-may-95
     Author: George Vosselman

     Update: Sign of correction for distortion is changed.
             Therefore the iteration is removed (moved to Pix_To_Metric).
             In this way the definition of the correction corresponds to the
             one used in SCAN-3.
    Version: 1.2
       Date: 20-feb-98
     Author: Frank van den Heuvel 

     Update: Bingo parameters added
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

void Metric_To_Pix(Interior *par, double x, double y, double *r, double *c)
{
   int iter;  
   double dx,dy,dx0,dy0,xrot,yrot;
   
   void Intor_Correct_Bingo(Interior *, double, double, double *, double *);
   void Intor_Correct(Interior *, double, double, double *, double *);
   
/*------ Corrections (dx,dy) interior orientation ------*/
  if (par->bingo[0]>0)
  {
    /*------ First correct for Bingo principle point -------*/
    x += par->bingo_xh;
    y += par->bingo_yh;
    iter = 0;
    dx0 = dy0 = 0;
    Intor_Correct_Bingo(par,x,y,&dx,&dy);

    while ((fabs(dx-dx0)>EPS || fabs(dy-dy0)>EPS) && iter<MAXITER)
      {
	dx0=dx; dy0=dy;
	Intor_Correct_Bingo(par,x+dx0,y+dy0,&dx,&dy);
	iter++;
      }
    if (iter==MAXITER) fprintf(stderr,"\n *** ERROR in Pix_To_Metric: maximum number of iterations reached!\n");

    x += dx; y += dy;
  }

  Intor_Correct(par,x,y,&dx,&dy);
 
/*------ Co-ordinate system transformation ------*/
  xrot = (x + dx) * cos(par->rot) - (y + dy) * sin(par->rot);
  yrot = (x + dx) * sin(par->rot) + (y + dy) * cos(par->rot);
  *r = par->rh - yrot / par->spacing_r;
  *c = par->ch + xrot / par->spacing_c;

}
