
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



/* Mod_Basis.c : 
*  - Berekent mbv de fortran-subroutines dgeco() en dgesl()
*    de vector x uit het stelsel A.x = y
*       waarbij:
*       A staat voor de normaalmatrix ata[8][8], (ata = a-transpose * a)
*       y staat voor de vector[8] aty, (aty = a-transpose * y)
*       vector x de 8 te ber. elementen van de essentiele matrix e[3][3] bevat
*  - Berekent de matrix eet[3][3] waarbij eet = e.e',
*    het product van de matrix e met zijn getransponeerde.
*  - Berekent mbv de fortran-subroutine rg() de eigenwaarden en vectoren van eet
*       het resultaat is de matrix rg_z[3][3]
*       die een schatting levert voor de modelbasis namelijk:
*       bx = rg_z[index][0], by = rg_z[index][1], bz = rg_z[index][2]
*/

#include <math.h>
//#include "digphot_arch.h"

int Mod_Basis(double *ata, double *aty, double e[3][3], double *wr,
              double rg_z[3][3])
{
   double  ry_a[8], eet[3][3], eigenwmin, bx;
   int i, j, k, index;
   
   /* argumenten van de subroutines dgeco() en dgesl() */
   int atadim=8, ipvt[8], job=1;
   double z[8], rcond;
   
   /* argumenten van de subroutine rg(), tbv eigenwaarden en -vectoren */
   int ord_eet=3, matz=1, iv1[3], ierr ;
   double  wi[3], fv1[3];

#ifdef hpux
   void dgeco(double *, int *, int *, int *, double *, double *);
   void dgesl(double *, int *, int *, int *, double *aty, int *);
   void rg(int *, int *, double *, double *, double *, int *, double *,
           int *, double *, int *); 
#else
   void dgeco_(double *, int *, int *, int *, double *, double *);
   void dgesl_(double *, int *, int *, int *, double *aty, int *);
   void rg_(int *, int *, double *, double *, double *, int *, double *,
            int *, double *, int *); 
#endif
   
   /* dgeco() is een voorbewerkingsroutine 
      die de matrix ata bewerkt tbv dgesl()
      dgesl() dient om de vector x uit het stelsel 
      ata.x = aty op te lossen         
      De vector aty wordt overschreven door de uitkomst x 
      */
   
   /* Daar dgeco() en dgesl() fortranroutines zijn moeten waarden van
      variabelen zoals atadim = 8 meegegeven worden via hun adres      */
   
#ifdef hpux
   dgeco(ata,&atadim,&atadim,ipvt,&rcond,z);
#else
   dgeco_(ata,&atadim,&atadim,ipvt,&rcond,z);
#endif

   /* naarmate rcond meer naar nul nadert is de oplossing slechter bepaald */

#ifdef hpux
   dgesl(ata,&atadim,&atadim,ipvt,aty,&job);
#else
   dgesl_(ata,&atadim,&atadim,ipvt,aty,&job);
#endif

   /* Als job=1 dan wordt de getransponeerde vorm van diff gebruikt,
      dit is hier nodig daar dgesl() een fortran routine is.         */
   
   e[0][0] = aty[0];
   e[1][0] = aty[1];
   e[2][0] = aty[2];
   e[0][1] = aty[3];
   e[1][1] = aty[4];
   e[2][1] = 1.0; 
   e[0][2] = aty[5];
   e[1][2] = aty[6];
   e[2][2] = aty[7];
   
   /* initialisatie op 0 en berekening van de matrix eet[3][3] waarbij
      eet = e.e', het product van de matrix e met zijn getransponeerde */
   for( i=0 ; i<3 ; i++ )
   {
      for( j=0 ; j<3 ; j++ )
      {
         eet[i][j] = 0.0;
         for( k=0 ; k<3 ; k++ )
            eet[i][j] += e[i][k] * e[j][k];
      }
   }
   
   /* De schatting voor de basis b is de eigenvector met de   
      * kleinste eigenwaarde, te ber. met fortran routine rg() .
      */
#ifdef hpux
   rg(&ord_eet, &ord_eet, (double *) eet, wr, wi, &matz,(double *) rg_z, iv1,
      fv1, &ierr); 
#else
   rg_(&ord_eet, &ord_eet, (double *) eet, wr, wi, &matz,(double *) rg_z, iv1,
       fv1, &ierr); 
#endif
   
   /* zoek de kleinste absolute waarde wr[]:  */
   eigenwmin = wr[0]; index = 0;
   for (i=1 ; i<3 ; i++)
      if ( fabs(wr[i]) < fabs(eigenwmin) )
      { 
	 eigenwmin = wr[i];
	 index = i;
      }

   /* rg_z[index][0] is bx en moet positief zijn */
   /* als bx < 0 dan de betreffende eigenvector met -1 vermenigvuldigen */
   /* in het volgende statement is rg_z het beginadres van de matrix   */
   
   bx = rg_z[index][0];
   if( bx < 0.0 )
   {
      rg_z[index][0] = - rg_z[index][0];
      rg_z[index][1] = - rg_z[index][1];
      rg_z[index][2] = - rg_z[index][2];
   }
   
   return( index );
}
