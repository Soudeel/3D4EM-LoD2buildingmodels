
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



/* SR_STEP2 => zweiter Schritt der direkten Lösung.
   Berechnung der Koordinaten des Projektionszentrums und der rotation Matr.
   E.W. Grafarend,
   P. Lohse und B. Schaffarin : Dreidimensionaler Rueckwaertsschnitt. 
                                Teil III : Dreistufige Loesung der algebraischen
                                           gleichungen -
                                           Orientierungsparameter, Koordinaten -           
   Given are the image and modell koordinates and the distance between the modell 
   points and the projection center. */

#include <math.h>
#include <stdio.h>
#include "r_rueck.h"

void SR_STEP2 (double xim[4], double yim[4], double cc, double s[4],
               double xmod[4], double ymod[4], double zmod[4], 
               double *xpc, double *ypc, double *zpc,
		       double *qa, double *qb, double *qc, int ind[4])
/*
int ind[4];                         point order index array
double cc;                          principal distance
double s[4];                        distances between model points and 
                                    projectioncenter
double xim[4], yim[4];              image koordinates of the three points
double xmod[4], ymod[4], zmod[4];   the modell coordinates of the three points
double *xpc, *ypc, *zpc;            coordinates of the projection center
double *qa, *qb, *qc;               quaternions describing the rotation matrix
*/

{
int i, j;
int ih[4];
int i1, i2, i3;
double a11[4][4], a21[4][4], adif[4][4], adifhelp[10];
double b1[4], b2[4], bdif[4];
double x1[4], x2[4], x3[4];
double xsav, ysav, zsav;
double x[4], h1[4];
double det=0.0, lam[4];

void SR_GAUSS(int, double *, double *, double *, double *, double, int *);

/*
 * reduce the model coordinates and calculate the scale of the image vector
 */

xsav = xmod[1];
ysav = ymod[1];
zsav = zmod[1];

for (i=1; i<=3; ++i)
    {
    xmod[i] = xmod[i] - xsav;
    ymod[i] = ymod[i] - ysav;
    zmod[i] = zmod[i] - zsav;
    lam[i]  = xim[i]*xim[i] + yim[i]*yim[i] + cc*cc;
    lam[i]  = s[i]/sqrt(lam[i]);
    }

/*
 * get the point order from the index array
 */

i1 = ind[1];
i2 = ind[2];
i3 = ind[3];

/*
 * sub matrix A11
 */

a11[1][2] = -zmod[i1] + lam[i1]*cc;
a11[1][3] =  ymod[i1] + lam[i1]*yim[i1];
a11[2][1] =  zmod[i1] - lam[i1]*cc;
a11[2][3] = -xmod[i1] - lam[i1]*xim[i1];
a11[3][1] = -ymod[i1] - lam[i1]*yim[i1];
a11[3][2] =  xmod[i1] + lam[i1]*xim[i1];

/*
 * sub matrix A21
 */

a21[1][2] = -zmod[i2] + lam[i2]*cc;
a21[1][3] =  ymod[i2] + lam[i2]*yim[i2];
a21[2][1] =  zmod[i2] - lam[i2]*cc;
a21[2][3] = -xmod[i2] - lam[i2]*xim[i2];
a21[3][1] = -ymod[i3] - lam[i3]*yim[i3];
a21[3][2] =  xmod[i3] + lam[i3]*xim[i3];

/*
 * sub vektor b1
 */

b1[1] = lam[i1]*xim[i1] - xmod[i1];
b1[2] = lam[i1]*yim[i1] - ymod[i1];
b1[3] =-lam[i1]*cc - zmod[i1];

/*
 * sub vektor b2
 */

b2[1] = lam[i2]*xim[i2] - xmod[i2];
b2[2] = lam[i2]*yim[i2] - ymod[i2];
b2[3] =-lam[i3]*cc - zmod[i3];

/*
 * difference A11-A21 and  b1-b2
 */

for (i=1; i<=3; ++i)
    {
    for (j=1; j<=3; ++j)
	{
	if (i == j)
	   {
	   adif[i][j] = 0.0;
           a11[i][j] = 0.0;
           a21[i][j] = 0.0;
	   }
	else
	   {
	   adif[i][j] = a11[i][j] - a21[i][j];
	   }
	}
	bdif[i] = b2[i] - b1[i];
    }

/*
 * Umwandlung von adif ( 3 x 3 Matrix) in adifhelp ( 9 x 1 Vektor)
 * zur Anpassung an benoetigtes Datenvormat in SR_GAUSS !
 */

adifhelp[1] = adif[1][1];
adifhelp[2] = adif[2][1];
adifhelp[3] = adif[3][1];
adifhelp[4] = adif[1][2];
adifhelp[5] = adif[2][2];
adifhelp[6] = adif[3][2];
adifhelp[7] = adif[1][3];
adifhelp[8] = adif[2][3];
adifhelp[9] = adif[3][3];


/*
 * solve the system (A11-A21)*x (b2-b1) = 0
 */

SR_GAUSS(3, adifhelp, bdif, h1, x1, det, ih);

/*
 * the solution vector contains the quaternion elements which describe the
 * rotation matrix
 */

(*qa) = x1[1];
(*qb) = x1[2];
(*qc) = x1[3];

/*
 * calculate the help variables u, v, w
 */

for (i=1; i<=3; ++i)
    {
    x2[i] = b1[i] - a11[i][1]*x1[1] - a11[i][2]*x1[2] - a11[i][3]*x1[3];
    }
  
/*
 * compose the asymmetry matrix
 */

adif[1][1] =  1.0;
adif[1][2] =  *qc;
adif[1][3] = -*qb;
adif[2][1] = -*qc;
adif[2][2] =  1.0;
adif[2][3] =  *qa;
adif[3][1] =  *qb;
adif[3][2] = -*qa;
adif[3][3] =  1.0;

/*
 * Umwandlung von adif ( 3 x 3 Matrix) in adifhelp ( 9 x 1 Vektor)
 * zur Anpassung an benoetigtes Datenvormat in SR_GAUSS !
 */

adifhelp[1] = adif[1][1];
adifhelp[2] = adif[2][1];
adifhelp[3] = adif[3][1];
adifhelp[4] = adif[1][2];
adifhelp[5] = adif[2][2];
adifhelp[6] = adif[3][2];
adifhelp[7] = adif[1][3];
adifhelp[8] = adif[2][3];
adifhelp[9] = adif[3][3];

/* solve the equation
 * ( 1  c -b) (xpc)   (u)   (0)
 * (-c  1  a) (ypc) + (v) = (0)
 * ( b -a  1) (zpc)   (w)   (0)
 */

SR_GAUSS(3, adifhelp, x2, h1, x3, det, ih);

/*
 * correct for the reduction
 */

for (i=1; i<=3; ++i)
    {
    xmod[i] = xmod[i] + xsav;
    ymod[i] = ymod[i] + ysav;
    zmod[i] = zmod[i] + zsav;
    }

*xpc = x3[1] + xsav;
*ypc = x3[2] + ysav;
*zpc = x3[3] + zsav;

}
