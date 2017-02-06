
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



/* SR_DIREC.C 
   Direct solution of the spatial resection problem:                      
   Given three points in the image and the model coordinate system,        
   determine the position and the orientation of the image system in the    
   model system. 
   
   Beschreibung: Hauptprogramm zur Steuerung der Unterprogramme
   SR_STEP1 und SR_STEP2, benoetigt Eingabedaten von RRS_D oder einem
   vergleichbarem Einleseprogramm.
   
   Ergebnis: Projektionszentrum und Drehelemente.

   Querverweis: Damit diese Programm laufen kannn, werden folgende Routinen
   benoetigt: rrs_d.c, sr_step1.c, sr_step2.c, solve_eq.c, sr_gauss.c und
              complex.c, sowie der Hedder "r_rueck.h".
   
   Beispiel: Als Beispiel dienen die Testdatensaetze iamg.in, rich.in, orip.in.
   
   Autor der Originalversion in Vax-Fortran: George Vosselman
   Bearbeitet von                          : Anja Wilkin
   Uebersetzt in C durch                   : Christian Eckers
   Version vom 26.05.1992                                                */




#include <stdio.h>
#include "r_rueck.h"

void SR_DIREC (double xim[4], double yim[4], double cc, 
               double xmod[4], double ymod[4], double zmod[4], double distan,
               double xpc[5], double ypc[5], double zpc[5],
    	       double qa[5], double qb[5], double qc[5], int *nsol)
/*
int *nsol;
double xpc[5], ypc[5], zpc[5];             Projektionszentrum 
double qa[5], qb[5], qc[5];                Quaternionen Elemente
double xim[4], yim[4];                     imaginaere Bildkoordinaten
double cc;                                 Kammerkonstante
double xmod[4], ymod[4], zmod[4];          Modellkoordinaten
double distan;                             Maximale Distanz zwischen den ersten
					                       beiden Armen des Tetraeders
*/
{
int ind[4],i,j;
int nsolh;
double s[4][5];                            /* Strecke vom Projektionszentrum zu
				              den Objektpunkten */
                                           /*  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  */
double shelp[4];                           /* Zeilenvektor aus s[4][5] */

void SR_STEP1 (double *, double *, double, double *, double *, double *, double,
               double [4][5], int *, int *);
void SR_STEP2 (double *, double *, double, double *, double *, double *,
               double *, double *, double *, double *, double *, double *,
               double *, int *);
distan = 3.0;

/*
 * Berechnung der Distanz zwischen den Modellpunkten und dem Projektionszentrum
 * Diese Routine bestimmt max. drei Loesungen
 */

SR_STEP1 (xim, yim, cc, xmod, ymod, zmod, distan, s, &nsolh, ind);
*nsol = nsolh;

/*
 * Berechnung des Projektionszenrums und der Qauternionen bezueglich der
 * Orientierungsmatrix
 */

for (i=1; i<=*nsol; ++i)
    {
    for (j=1; j<= 3; ++j)
        {
        shelp[j] = s[j][i];
        }    
    SR_STEP2 ( xim, yim, cc, shelp, xmod, ymod, zmod, &xpc[i], &ypc[i], &zpc[i],
			         		      &qa[i], &qb[i], &qc[i], ind);
    
   }
if (*nsol == 0)
   {
   printf (" Keine Loesung mit diesem Dreieck\n");
   }
}

