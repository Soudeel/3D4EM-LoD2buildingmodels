
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



/* SR_GAUSS =>
   Gausscher algorithmus mit pivotstrategie 
   Schwarz, H.R.: Numerische Mathematik
                  Teubner, Stuttgart 1986
   Autor der Fortranversion : Peter Lose
   Uebersetzt nach C von    : Christian Eckers
   Version vom 15.04.92 */

#include <math.h>
#include <stdio.h>

void SR_GAUSS(int n, double a[10], double b[4], double c[4], double x[4],
              double det, int p[4])
/*
int n;                      Number of rows of the linear equation A*X+B=0
int p[4];                   n-1 Vector
double a[10];               nXn Matrix with full rank ( saved as nXn Vector )
double b[4];                nX1 Vector
double c[4];                nX1 Vector
double x[4];                nX1 Vector
double det;                 determinant
*/

{
int i, j, k, l, m, o;       /* Schleifen und index Variablen */
double s, q, max, h;

det = 1.0;

for (k=1; k<=n-1; ++k)
    {
    max = 0.0;
    p[k] = 0;
    for (i=k; i<=n; ++i)
	{
	s = 0.0;
	for (j=k; j<=n; ++j)
	    {
	    l = i + n*(j-1);
	    s = s + fabs(a[l]);
	    }
	l = i + n*(k-1);
	q = fabs(a[l])/s;
	if (q > max)
	   {
	   max = q;
	   p[k] = i;
	   }
	}   
    if (max == 0.0)
       {
       break;
       }
    if (p[k] != k)
       {
       det = (-1)*det;
       for (j=1; j<=n; ++j)
	   {
	   l = k + n*(j-1);
	   m = p[k] + n*(j-1);
	   h    = a[l];
	   a[l] = a[m];
	   a[m] = h;
	   }
       }
       l = k + n*(k-1);
       det = det*a[l];
       for (i=k+1; i<=n; ++i)
	   {
	   l = i + n*(k-1);
	   m = k + n*(k-1);
	   a[l] = a[l]/a[m];
	   for (j=k+1; j<=n; ++j)
	       {
	       l = i + n*(j-1);
	       m = i + n*(k-1);
	       o = k + n*(j-1);
	       a[l] = a[l] - a[m]*a[o];
	       }
	   }

     }
l = n + n*(n-1);
det = det*a[l];

/*
 * permutations in the nx1-vector b
 */

for (k=1; k<=n-1; ++k)
    {
    if (p[k] != k)
       {
       h = b[k];
       b[k] = b[p[k]];
       b[p[k]] = h;
       }
    }

/*
 * forward substituation
 */

for (i=1; i<=n; ++i)
    {
    c[i] = b[i];
    if (i == 1)
       {
       goto endfor;
       }
    for (j=1; j<=i-1; ++j)
	{
	l = i + n*(j-1);
	c[i] = c[i] - a[l]*c[j];
	}
endfor:;
    }

/*
 * backward substituation
 */

for (i=n; i>=1; --i)
    {
    s = c[i];
    for (k=i+1; k<=n; ++k)
	{
	l = i + n*(k-1);
	s = s + a[l]*x[k];
	}
    l = i + n*(i-1);
    x[i] = -s/a[l];
    }
}


