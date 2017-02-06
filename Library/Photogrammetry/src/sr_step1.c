
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



/* SR_STEP1 => erster Shritt der direkten Loesung.
   First step of the direct solution for the spatial resection: calculation
   of the distances between the projection center and the model points.
   The solution programmed here was published by:
   M.A. Fischler and R.C. Bolles in the Communication of the ACM, Vol. 24, No. 6,
   June 1981.
   Titel: Random Sample Consensus: A Paradigm for Model Fitting with Applications
   to Image Analysis and Automated Cartography.
   A fourth order equation is derived which can result in 0 to 4 solutions
   for the distance ratio (OA/OB) which is the unknown of the equation.
   In a special case there are two solutions for OC for one value of OA/OB,
   but the total number of solutions can not exceed four.

   Autor der Fortrunversion : Georg Vosselmann
   Uebersetzt in C von      : Christian Eckers
   Version vom 26.05.1992 */

#include <stdio.h>
#include <math.h>
#include "r_rueck.h"

void SR_STEP1 (double xim[4], double yim[4], double cc,
               double xmod[4], double ymod[4], double zmod[4], double distan,
               double s[4][5], int *nsol, int ind[4])
/*
int ind[4];                         point order index array
int *nsol;                          the number of solutions
double xim[4], yim[4];              the image koordinates of the three points
double cc;                          the principal distance
double xmod[4], ymod[4], zmod[4];   the modell koordinates of the three points
double distan;                      the maximum distance ratio between the first
                                    two legs of the tetraedon
double s[4][5];                     the solutions of the three distances
*/
{

int i,i1, i2, i3, isol, order;
double cos_alb, cos_blc, cos_cla, sin_blc, sin_cla;
double k1, k2;
double ab, bc, ac, la, lb, lc;
double d1, d2, p1, p2, q1, q2, m1, m2, m;
double disrec, help, bmax;
double ab2, ac2, bc2;
double a[5];
COMPLEX sol[5];

void SOLVE_EQ (double *, int, COMPLEX *, int *);
/*
 * Distances between the image points
 */

ab2 = sqrt ((xim[1]-xim[2])*(xim[1]-xim[2]) + (yim[1]-yim[2])*(yim[1]-yim[2]));
bc2 = sqrt ((xim[3]-xim[2])*(xim[3]-xim[2]) + (yim[3]-yim[2])*(yim[3]-yim[2]));
ac2 = sqrt ((xim[1]-xim[3])*(xim[1]-xim[3]) + (yim[1]-yim[3])*(yim[1]-yim[3]));

/*
 * Select the longest basis
 */

bmax = ab2;
i1 = 1;
i2 = 2;
i3 = 3;
if ( ac2 > bmax )
   {
   bmax = ac2;
   i1=1;
   i2=3;
   i3=2;
   }
if ( bc2 > bmax )
   {
   i1=3;
   i2=2;
   i3=1;
   }

/*
 * Distance between the model points and the angles in the  projection center
 */

ab = sqrt ((xmod[i1]-xmod[i2])*(xmod[i1]-xmod[i2]) +   
           (ymod[i1]-ymod[i2])*(ymod[i1]-ymod[i2]) +
	   (zmod[i1]-zmod[i2])*(zmod[i1]-zmod[i2]));
ac = sqrt ((xmod[i1]-xmod[i3])*(xmod[i1]-xmod[i3]) +
           (ymod[i1]-ymod[i3])*(ymod[i1]-ymod[i3]) +
	   (zmod[i1]-zmod[i3])*(zmod[i1]-zmod[i3]));
bc = sqrt ((xmod[i3]-xmod[i2])*(xmod[i3]-xmod[i2]) + 
           (ymod[i3]-ymod[i2])*(ymod[i3]-ymod[i2]) +
	   (zmod[i3]-zmod[i2])*(zmod[i3]-zmod[i2]));

la = sqrt ( xim[i1]*xim[i1] + yim[i1]*yim[i1] + cc*cc);
lb = sqrt ( xim[i2]*xim[i2] + yim[i2]*yim[i2] + cc*cc);
lc = sqrt ( xim[i3]*xim[i3] + yim[i3]*yim[i3] + cc*cc);

cos_alb = ( xim[i1]*xim[i2] + yim[i1]*yim[i2] + cc*cc)/(la*lb);
cos_blc = ( xim[i3]*xim[i2] + yim[i3]*yim[i2] + cc*cc)/(lc*lb);
cos_cla = ( xim[i1]*xim[i3] + yim[i1]*yim[i3] + cc*cc)/(la*lc);

/*
 * Squared distance ratios
 */

k1= (bc/ac)*(bc/ac);
k2= (bc/ab)*(bc/ab);

/*
 * The coefficience of the fourtch order equation
 */

a[4] = ((k1*k2 -k1 -k2)*(k1*k2 -k1 -k2)) - 4.0*k1*k2*(cos_blc*cos_blc);
a[3] = 4.0 * (k1*k2 - k1 - k2) * k2 * (1.0 - k1)* cos_alb+
       4.0 * k1 * cos_blc * ((k1*k2 + k2 - k1)* cos_cla +
       2.0 * k2 * cos_alb * cos_blc);
a[2] = (2.0 *k2 * (1.0 - k1)* cos_alb)*(2.0 *k2 * (1.0 - k1)* cos_alb) +
       2.0 * (k1*k2 + k1 - k2) * (k1*k2 - k1 - k2)+
       4.0 * k1 * ((k1-k2) * (cos_blc*cos_blc) +
       (1.0 - k2) * k1 * (cos_cla*cos_cla) -
       2.0 * k2 * (1.0 + k1) * cos_alb * cos_cla * cos_blc);
a[1] = 4.0 * (k1*k2 + k1 - k2) * k2 * (1.0 - k1)* cos_alb+
       4.0 * k1 * ((k1*k2 - k1 + k2)* cos_cla * cos_blc +
       2.0 * k1 * k2 * cos_alb * (cos_cla*cos_cla));
a[0] = (k1*k2 + k1 - k2)*(k1*k2 + k1 - k2) -
       4.0*k2*((k1 * cos_cla)*(k1 * cos_cla));

/*
 * Solution of the equation
 */

order = 4;
SOLVE_EQ (a, order, sol, &isol);

/*
 * The distance ratio should be real and smaller than "distan".
 */

sin_blc = sqrt (1.0 - (cos_blc*cos_blc));
sin_cla = sqrt (1.0 - (cos_cla*cos_cla));
d1 = bc/sin_blc;
d2 = ac/sin_cla;
disrec = 1.0/distan;
m1 = 1.0-k1;
m2 = 1.0;
*nsol = 0;
for ( i=1; i<=isol; ++i)
    {
    if (fabs(sol[i].y) < 1.0E-8)
       {
       m = sol[i].x;
/* Check on distance ratio removed in a close-range project. */
/*        if (m > disrec & m < distan) */
	  {

/*
 * Calculate 0a and 0b
 */
	  help = (1.0 + m*m - 2.0*m*cos_alb);
	  if (help < 0.0)
	     {
	     goto endfor;
	     }
	  la=ab/sqrt(1.0 + m*m -2.0*m*cos_alb);
	  lb=la*m;

/*
 * Calculate q1 and q2
 */
	  q1=m*m - k1;
	  q2=m*m*(1.0 - k2) + 2.0*m*k2*cos_alb - k2;

/*
 * two solution for 0C
 */
	  if (m1*q2 == m2*q1)
	     {
	     help = d2*d2-la*la;
	     if (help < 0.0)
		{
		goto endfor;
		}
	     lc = la*cos_cla - sqrt(help)*sin_cla;
	     if (lc > 0.0)
		{
		*nsol = *nsol+1;
		s[i1][*nsol] = la;
	        s[i2][*nsol] = lb;
		s[i3][*nsol] = lc;
		}
	     lc = la*cos_cla + sqrt(help)*sin_cla;
	     }

/*
 * One solution
 */
	  else
	     {
	     p1 =  2.0 * (k1*cos_cla - m*cos_blc);
	     p2 = -2.0 * m * cos_blc;
	     lc = la * (p2*q1 - p1*q2) / (m1*q2 - m2*q1);
	     }
	  *nsol = *nsol+1;
	  s[i1][*nsol] = la;
	  s[i2][*nsol] = lb;
	  s[i3][*nsol] = lc;
       }
    }
  endfor:;
  }

/*
 * transfer the point to SR_STEP2
 */

ind[1] = i1;
ind[2] = i2;
ind[3] = i3;
}
