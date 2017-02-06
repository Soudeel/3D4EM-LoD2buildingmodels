
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



/* Coplan_direct.c : 
*  - Leest de fotocoordinaten (Campts in mm) uit 2 files
*  - Zoekt de gemene punten op
*  - Op basis van de coplanariteitsvoorwaarde geldt het stelsel a.x = y
*    Gebruik makend van de K.K. vereffening ontstaat het stelsel ata.x = aty
*    Berekent de matrix ata[8][8] (ata = A'A is de normaalmatrix)
*    en de vector aty[8] (aty = A'y)
*/

#include "Database.h" /*met o.a. definities van de structures CamPts en Pixel */
#define BUFSIZE 80

void Coplan_direct(CamPts *L, CamPts *R, double f, double ata[8][8], 
                   double aty[8])
{
   int i, j, k, l, m, count;   /* tellers */
   double xl, yl, xr, yr;    /* fotocoord Links en Rechts met cameraconstante */
   double ry_a[8], e[3][3], eet[3][3];
   
   /* initialisatie van de vector aty[] = b op nul    */
   /* en initialisatie van de normaalmatrix ata[][] op nul */
   for( k=0 ; k<8 ; k++ )
   {
      aty[k] = 0;
      for( l=0 ; l<8 ; l++ )
         ata[k][l] = 0 ;
   }
   
   /* Opzoeken van overeenkomstige gemeten punten in beide foto's L en R */
   count=0;
   for( i=0 ; i< L->num_pts ; i++ )  /* gemeten punten in Linker foto  */
   {
      for( j=0 ; j< R->num_pts ; j++ )  /* gemeten punten in Rechter foto  */
      {
         if( L->pts[i].num == R->pts[j].num )
	 {
            /* gemeenschappelijk punt gevonden in linker en rechter foto  */
	    /* hier tzt een conversieroutine aanroepen om row,col -> x,y  */
            /* thans bevatten de ingelezen files fotocoord vandaar:       */
            count++;
	    xl = L->pts[i].x;   yl = L->pts[i].y;
            xr = R->pts[j].x;   yr = R->pts[j].y;
	    
            /* berekening van de 8 elementen van de rij van matrix a */  
            /* die op te stellen is met het gevonden puntenpaar      */ 
	    
            ry_a[0] =  xl*xr ;
	    ry_a[1] =  yl*xr ;
	    ry_a[2] = -xr*f  ;
            ry_a[3] =  xl*yr ;
            ry_a[4] =  yl*yr ;
            ry_a[5] = -xl*f  ;
            ry_a[6] = -yl*f  ;
            ry_a[7] =   f*f  ;
            
            /* Berekening van de normaalmatrix ata en de vector aty.        */
            /* De berekening vindt plaats zonder de a matrix op te stellen. */
            /* De elementen van ata en aty worden bepaald door steeds de    */ 
            /* bijdrage van een nieuw gevonden puntenpaar bij de bestaande  */
            /* elementen van ata en aty op te tellen.                       */
	    
            for( k=0 ; k<8 ; k++ )
	    {
               for( l=0 ; l<8 ; l++ )
                  ata[k][l] += ry_a[k] * ry_a[l];
               aty[k] += ry_a[k] * yr*f;
	    }
	 } /* afsluiting if-loop */
      } /* afsluiting for-loop j */
   } /* afsluitng for-loop i */
}
