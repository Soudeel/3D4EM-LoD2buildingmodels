
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



#include <stdio.h>
#include <math.h>

/*----------------------------------------------------------------------
   Nu worden bij meer dan 3 gevonden paspunten in de foto drie
   punten geselecteerd,die zover mogelijk uit elkaar liggen. Het vierde
   punt, dient om na te gaan welke van de meerdere oplossingen juist is.
   
   Procedure:
   1 Vind punt(A), zover mogelijk van hoofdpunt.
   2 Vind punt(B), zover mogelijk van punt A
   3 Vind punt(C), zover mogelijk van lijn AB.
   4 Vind punt(D), zover mogelijk van vlak door objectpunten A, B en C.
   De te gebruiken punten staan in lijstpnr, lijstxpi en lijstypi,
   de objectpunten in lijstxpas, lijstypas en lijstzpas.
  ---------------------------------------------------------------------*/

int Search_Start_Points(int ngevonden, int *lijstpnr, double *lijstxpi,
                        double *lijstypi, double *lijstxpas, double *lijstypas,
                        double *lijstzpas, int *a, int *b, int *c, int *d)
{
   double afstand; /* Voor berekende afstanden */
   double afstandmax;
   double wab;       /* hulpvariabele: ( y(B)-y(a))/(x(B)-x(A)) */
   double dif1[3], dif2[3], normal[3], distorg, distmin;
   int i;
   
   /* *a -> Positie van punt A in lijstpnr */
   /* *b -> Positie van punt B in lijstpnr */
   /* *c -> Positie van punt C in lijstpnr */
   /* *d -> positie van punt D in lijstpnr */
   
   *a = 0 ;
   *b = 0 ;
   *c = 0 ;
   *d = 0 ;
   
   /*------------------ Bepaling positie van A ---------------------*/

   afstandmax = 0;
   for (i = 0; i < ngevonden; i++)
   {
      afstand = lijstxpi[i]*lijstxpi[i] + lijstypi[i]*lijstypi[i];
      if (afstand >= afstandmax)
      {
	 afstandmax = afstand;
	 *a = i;
      }
   }
   
   /*------------------ Bepaling positie van B ---------------------*/

   afstandmax = 0;
   for (i = 0; i < ngevonden; i++)
   {
      if ( i != *a )
      {
	 afstand = (lijstxpi[i]-lijstxpi[*a]) * (lijstxpi[i]-lijstxpi[*a]) + 
                   (lijstypi[i]-lijstypi[*a]) * (lijstypi[i]-lijstypi[*a]);
	 if ( afstand >= afstandmax )
	 {
	    afstandmax = afstand;
	    *b = i;
	 }
      }
   }

   /*------------------ Bepaling positie van C ---------------------*/

   normal[0] = lijstypi[*b] - lijstypi[*a];
   normal[1] = lijstxpi[*a] - lijstxpi[*b];
   distorg = lijstxpi[*a] * normal[0] + lijstypi[*a] * normal[1];
   afstandmax = -1.0;
   for (i = 0; i < ngevonden; i++)
   {
      if ( i != *a && i != *b )
      {
	 afstand = lijstxpi[i] * normal[0] + lijstypi[i] * normal[1] - distorg;
         if (afstand < 0) afstand = -afstand;
	 if ( afstand > afstandmax )
	 {
	    afstandmax = afstand;
	    *c = i;
	 }
      }
   }
   
   /*------------------ Bepaling positie van D ---------------------*/
   
   dif1[0] = lijstxpas[*b] - lijstxpas[*a];   /* Vector AB */
   dif1[1] = lijstypas[*b] - lijstypas[*a];
   dif1[2] = lijstzpas[*b] - lijstzpas[*a];
   
   dif2[0] = lijstxpas[*c] - lijstxpas[*a];   /* Vector AC */
   dif2[1] = lijstypas[*c] - lijstypas[*a];
   dif2[2] = lijstzpas[*c] - lijstzpas[*a];

   normal[0] = dif1[1] * dif2[2] - dif1[2] * dif2[1];  /* Normaal op vlak ABC */
   normal[1] = dif1[2] * dif2[0] - dif1[0] * dif2[2];  /* Lengte != 1 !       */
   normal[2] = dif1[0] * dif2[1] - dif1[1] * dif2[0];

   distorg = lijstxpas[*a] * normal[0] +  /* Afstand vlak ABC tot oorsprong */
             lijstypas[*a] * normal[1] +
             lijstzpas[*a] * normal[2];
   
   afstandmax = -1.0;
   for (i = 0;i < ngevonden; i++)
     if ( i != *a && i != *b && i != *c) {
       afstand = lijstxpas[i] * normal[0] +
                 lijstypas[i] * normal[1] +
                 lijstzpas[i] * normal[2] - distorg;
       if (afstand < 0) afstand = -afstand;
       if (afstand > afstandmax) {
         afstandmax = afstand;
         *d = i;
       }
     }
   
   printf("\nFor direct exterior orientation the following pointnumbers are used:\n");
   printf("%d, %d, %d\n",lijstpnr[*a],lijstpnr[*b],lijstpnr[*c]);
   printf("As check point, point number %d will be used\n", lijstpnr[*d]);

   return(1);
}
