
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



/* Quatern.c :
 * berekent de benaderde waarden van de quaternionen q1 q2 en q3 
 *
 * input: 
 * de essentiele matrix e[3][3]
 * de modelbasis bx, by, en bz
 *  
 * output:
 * de quaternionen q1, q2 en q3
 *
 * werkwijze:
 * Gegeven is de vergelijking (Fot I, blz 107, formule 6.41):
 *       { Sb( I + Sq ) - E ( I - Sq )} * { Sb( I + Sq ) - E ( I - Sq )}
 * Na uitwerken en partieel differentieren naar q1, q2 en q3 ontstaan
 * er 3 vergelijkingen met als onbekenden de quaternionen q1 q2 en q3.
 * Eerst worden de coefficienten van deze vergelijkingen berekend
 * en in quat[3][3] geplaatst.
 * De uitkomsten van de vergelijkingen worden in vector y[3] geplaatst.
 * Vervolgens wordt met de routines dgeco() en dgesl() 
 * het stelsel quat.x = y opgelost, 
 * waarbij x staat voor de vector van de te bepalen quaternionen.
 */

//#include "digphot_arch.h"

void Approx_Quaternion(double e[3][3], double bx, double by, double bz,
                       double *q1, double *q2, double *q3)
{
   double quat[3][3], y[3];
 
   /* declaraties tbv dgeco() en dgesl(): */
   int quatdim=3, ipvt[3], job=1;
   double rcond, z[3];
   
   void dgeco_(double *, int *, int *, int *, double *, double *);
   void dgesl_(double *, int *, int *, int *, double *, int *);

   /* berekening van de coeff. van q1, q2 en q3 en plaatsing in quat[3][3] */

   quat[0][0] = 
   -4*bx*e[1][2]      +4*bx*e[2][1]      +4*bx*bx           +4*by*e[0][2]
   +2*by*by           -4*bz*e[0][1]      +2*bz*bz           +2*e[0][1]*e[0][1]
   +2*e[0][2]*e[0][2] +2*e[1][1]*e[1][1] +2*e[1][2]*e[1][2] +2*e[2][1]*e[2][1]
   +2*e[2][2]*e[2][2];

   quat[0][1] = 
   -2*bx*e[2][0]      +2*bx*by           +2*by*e[2][1]      +2*bz*e[0][0]
   -2*bz*e[1][1]      -2*e[0][0]*e[0][1] -2*e[1][0]*e[1][1] -2*e[2][0]*e[2][1];

   quat[0][2] =  
    2*bx*e[1][0]      +2*bx*bz           -2*by*e[0][0]      +2*by*e[2][2]
   -2*bz*e[1][2]      -2*e[0][0]*e[0][2] -2*e[1][0]*e[1][2] -2*e[2][0]*e[2][2];

   quat[1][0] = 
   -2*bx*e[2][0]      +2*bx*by           +2*by*e[2][1]      +2*bz*e[0][0]
   -2*bz*e[1][1]      -2*e[0][0]*e[0][1] -2*e[1][0]*e[1][1] -2*e[2][0]*e[2][1];

   quat[1][1] = 
   -4*bx*e[1][2]      +2*bx*bx           +4*by*e[0][2]      -4*by*e[2][0]
   +4*by*by           +4*bz*e[1][0]      +2*bz*bz           +2*e[0][0]*e[0][0]
   +2*e[0][2]*e[0][2] +2*e[1][0]*e[1][0] +2*e[1][2]*e[1][2] +2*e[2][0]*e[2][0]
   +2*e[2][2]*e[2][2];

   quat[1][2] = 
    2*bx*e[1][1]      -2*bx*e[2][2]      -2*by*e[0][1]      +2*by*bz
   +2*bz*e[0][2]      -2*e[0][1]*e[0][2] -2*e[1][1]*e[1][2] -2*e[2][1]*e[2][2];

   quat[2][0] = 
    2*bx*e[1][0]      +2*bx*bz           -2*by*e[0][0]      +2*by*e[2][2]
   -2*bz*e[1][2]      -2*e[0][0]*e[0][2] -2*e[1][0]*e[1][2] -2*e[2][0]*e[2][2];
   
   quat[2][1] = 
    2*bx*e[1][1]      -2*bx*e[2][2]      -2*by*e[0][1]      +2*by*bz
   +2*bz*e[0][2]      -2*e[0][1]*e[0][2] -2*e[1][1]*e[1][2] -2*e[2][1]*e[2][2];
   
   quat[2][2] =
    4*bx*e[2][1]      +2*bx*bx           -4*by*e[2][0]      +2*by*by
   -4*bz*e[0][1]      +4*bz*e[1][0]      +4*bz*bz           +2*e[0][0]*e[0][0]
   +2*e[0][1]*e[0][1] +2*e[1][0]*e[1][0] +2*e[1][1]*e[1][1] +2*e[2][0]*e[2][0]
   +2*e[2][1]*e[2][1];

   /* berekening van de termen van vector y[3] */

   y[0] = -4*bx*e[1][1] -4*bx*e[2][2] +4*by*e[0][1] +4*bz*e[0][2];
   y[1] =  4*bx*e[1][0] -4*by*e[0][0] -4*by*e[2][2] +4*bz*e[1][2];
   y[2] =  4*bx*e[2][0] +4*by*e[2][1] -4*bz*e[0][0] -4*bz*e[1][1];

   /* oplossing van het stelsel vergelijkingen quat.x = y  */ 

   /* dgeco() is een voorbewerkingsroutine die de matrix quat bewerkt 
   *  ten behoeve van  dgesl()
   *  dgesl() dient om de vector x uit het stelsel quat.x = y op te lossen
   *         de vector y wordt overschreven door de uitkomst x, 
   *         zijnde q1, q2 en q3.  
   */

   /* Daar dgeco() en dgesl() fortranroutines zijn moeten waarden 
      van variabelen zoals quatdim = 3 meegegeven worden via hun adres  */

#ifdef hpux
   dgeco(quat, &quatdim, &quatdim, ipvt, &rcond, z);
#else
   dgeco_((double *) quat, &quatdim, &quatdim, ipvt, &rcond, z);
#endif

   /* naarmate rcond meer naar nul nadert is de oplossing slechter bepaald */

#ifdef hpux
   dgesl(quat, &quatdim, &quatdim, ipvt, y, &job);
#else
   dgesl_((double *) quat, &quatdim, &quatdim, ipvt, y, &job);
#endif
   /* Als job=1 dan wordt de getransponeerde vorm van quat gebruikt,
      dit is hier nodig daar dgesl() een fortran routine is.         */

   *q1 = y[0];   *q2 = y[1];   *q3 = y[2];
}
