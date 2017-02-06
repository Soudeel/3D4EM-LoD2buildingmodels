/*SOLVE_EQ 
  direct solution of ponominal equations up to order four 
  Autor der Originalversion in VAX Fortran: Georg Vosselmann
  Uebersetzt in C                         : Christian Eckers
  Version vom 22.04.92 */

#include <stdio.h>
#include <math.h>
#include "r_rueck.h"

/*
 * quadratic equation, two solution
 */

void SOLVE_EQ2 (double a[3], COMPLEX sol[3])
{
int i;
COMPLEX dis, help;

help.x = a[1]*a[1] - 4.0*a[2]*a[0];
help.y = 0.0;
dis = c_sqrt (help);

sol[1].x = (-a[1] - dis.x)/(2.0 * a[2]);
sol[1].y = -dis.y/2.0;
sol[2].x = (-a[1] + dis.x)/(2.0 * a[2]);
sol[2].y =  dis.y/2.0;
}


/*
 * cubic equation, up to three solution
 */

void SOLVE_EQ3 (double a2[3], COMPLEX sol[4], int *nisol)
{

int j;
double a[3], q, p, dis, u, v, exp, r, phi, pi, sinphi, cosphi;
double help;
COMPLEX i;

for (j=0; j<=2; ++j)
    {
    a[j] = a2[j]/a2[3];
    }

if (fabs(a[2]) > 1.0E10)
   {
   *nisol = 2;
   SOLVE_EQ2 (a2, sol);
   printf (" coefficients too large for cubic solution\n");
   return ;
   }

*nisol = 3;
p = a[1]/3.0 - (a[2]*a[2])/9.0;
q = (3.0E0*a[0] - a[1]*a[2])/6.0E0 + (a[2]*a[2]*a[2])/27.0E0;
dis = q*q + p*p*p;
exp = 1.0E0/3.0E0;

if (dis > 0.0)
   {
   dis = sqrt(dis);
   u   = -q + dis;
   if (u < 0.0)
      {
      u = -1.0 * pow(-u,exp);
      }
   else
      {
      u = pow(u,exp);
      }

   v = -q - dis;
   if (v < 0.0)
      {
      v = -1.0 * pow(-v,exp);
      }
   else
      {
      v = pow(v,exp);
      }
   i.x = 0.0;
   i.y = 1.0;
   help = sqrt(3.0)/2;
   sol[1].x = u + v - a[2]/3.0;
   sol[1].y = 0.0;
   sol[2].x = -(u+v)/2.0 - a[2]/3.0;
   sol[2].y = -(u-v) * i.y * help;
   sol[3].x = -(u+v)/2.0 - a[2]/3.0;
   sol[3].y =  (u-v) * i.y * help;
   }
else
   {
   r      = sqrt (-p * -p * -p);
   cosphi = -1.0 * q/r;
   sinphi = sqrt((-1) * dis)/r;
   phi    = atan2(sinphi,cosphi);
   pi     = 4.0 * atan(1.0);
   r      = 2.0 * pow(r,exp);
   phi    = phi/3.0;
   pi     = 2.0 * pi/3.0;
   sol[1].x = r * cos(phi) - a[2]/3.0;
   sol[1].y = 0.0;
   sol[2].x = r * cos(phi+pi) - a[2]/3.0;
   sol[2].y = 0.0;
   sol[3].x = r * cos(phi+2.0*pi) - a[2]/3.0;
   sol[3].y = 0.0;

   }
}

/*
 * quadratic equation, up to four solution
 */

void SOLVE_EQ4 (double a2[5], COMPLEX sol[5], int *nsol)
{
int nsolb, i;
double a[5], b[5], u, t1, t2, h;
COMPLEX solb[5], s, s2, testval, h1, h2, h3, h4;

for (i=0; i<=3; ++i)
    {
    a[i] = a2[i]/a2[4];
    }

b[3] = 1.0;
b[2] = -a[2];
b[1] = a[1]*a[3] - 4.0*a[0];
b[0] = a[0] * (4.0*a[2] - a[3]*a[3]) - a[1]*a[1];

SOLVE_EQ3 (b, solb, &nsolb);

for (i=1; i<=nsolb; ++i)
    {
    if (fabs(solb[i].y) < 1.0E-10)
       {
       u = solb[i].x;
       t1 = a[3]*a[3]/4.0 + u - a[2];
       t2 = u*u/4.0 - a[0];
       if (t1>0.0 & t2>0.0)
	  {
	  goto loesung;
	  }
       }
    }

printf (" no solution for cubic equation or complex coefficient for quadr.eq.\n");
*nsol = 0;
return ;

loesung:;
*nsol = 4;
t1 = sqrt(t1);
t2 = sqrt(t2);
b[2] = 1.0;
b[1] = a[3]/2.0 - t1;
b[0] = u/2.0 - t2;

SOLVE_EQ2 (b, solb);

s.x = solb[1].x;
s.y = solb[1].y;
s2 = c_mul (s, s);
h1 = c_mul (s2, s2);
h2 = c_rmul (a[3], c_mul(s2,s));
h3 = c_rmul (a[2], s2);
h4 = c_rmul (a[1], s);
testval.x = h1.x + h2.x + h3.x + h4.x + a[0];
testval.y = h1.y + h2.y + h3.y + h4.y;

if (fabs (testval.x) < 1.0E-5)
   {
   sol[1].x = solb[1].x;
   sol[1].y = solb[1].y;
   sol[2].x = solb[2].x;
   sol[2].y = solb[2].y;
   b[1] = a[3]/2.0 + t1;
   b[0] = u/2.0 + t2;
   SOLVE_EQ2 (b,solb);
   sol[3].x = solb[1].x;
   sol[3].y = solb[1].y;
   sol[4].x = solb[2].x;
   sol[4].y = solb[2].y;
   }
else
   {
   b[0] = u/2.0 + t2;
   SOLVE_EQ2 (b,solb);
   sol[1].x = solb[1].x;
   sol[1].y = solb[1].y;
   sol[2].x = solb[2].x;
   sol[2].y = solb[2].y;
   b[1] = a[3]/2.0 +t1;
   b[0] = u/2.0 -t2;
   SOLVE_EQ2 (b, solb);
   sol[3].x = solb[1].x;
   sol[3].y = solb[1].y;
   sol[4].x = solb[2].x;
   sol[4].y = solb[2].y;
   }
}

/* General solution for polynomials up to degree 4 */

void SOLVE_EQ (double a[5], int order, COMPLEX sol[5], int *nsol)
{
int i, eq;
int nsolh;

/*for (i=0; i<=4; ++i)
    {
    printf ("%15.10f\n", a[i]);
    }*/

for (i=(order+1); i<=4; ++i)
    {
    a[i] = 0.0;
    }

for (i=4; i>=1; --i)
    {
    if ( a[i] != 0.0)
       {
       eq = i;
       break;
       }
    }

switch (eq)
   {

/*
 * zero order equation, no solution
 */

   case 0: *nsol = 0;
	   break;

/*
 * linear equation, one solution
 */

   case 1: *nsol = 1;
	   sol[1].x = (-1)*a[0]/a[1];
           sol[1].y = 0.0;
	   break;

   case 2: *nsol = 2;
	   SOLVE_EQ2 (a, sol);
	   break;

   case 3: SOLVE_EQ3 (a, sol, nsol);
	   break;

   case 4: SOLVE_EQ4 (a, sol, nsol);        
	   break;
   }
}










