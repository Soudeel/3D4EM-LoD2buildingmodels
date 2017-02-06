
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



#include "math.h"
#include "Database.h"


int Exterior_Direct(double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix, 
		            int a, int b, int c, int d, double disratio, double cc, 
                    Exterior *sol)
{
   int isol, nsol, i;
   double xim[4],  yim[4];
   double ximt[4], yimt[4], zim, scale;
   double xpc2[5], ypc2[5], zpc2[5], qa2[5], qb2[5], qc2[5];
   double epsx[4], epsy[4], epsxy[4], epsmin;
   double xmod[4], ymod[4], zmod[4];
   Exterior temp_ext;
   
   void SR_DIREC(double *, double *, double, double *, double *, double *,
                 double, double *, double *, double *,
      	         double *, double *, double *, int *);
   /*
    * Nu worden de coordinaten van de geselecteerde punten in de arrays
    * xmod ,ymod,zmod,xim,yim geplaatst t.b.v. SR_DIREC. Het controlepunt
    * in positie 0 en de punten voor de uitwendige orientering in de posities
    * 1,2en 3.
    */
   
   /*------  ------*/
   xmod[0] = lijstxpas[d];
   ymod[0] = lijstypas[d];
   zmod[0] = lijstzpas[d];
   xim[0]  = lijstxpix[d];
   yim[0]  = lijstypix[d];

   /*------  ------*/
   xmod[1] = lijstxpas[a];
   ymod[1] = lijstypas[a];
   zmod[1] = lijstzpas[a];
   xim[1]  = lijstxpix[a];
   yim[1]  = lijstypix[a];
   
   /*------  ------*/
   xmod[2] = lijstxpas[b];
   ymod[2] = lijstypas[b];
   zmod[2] = lijstzpas[b];
   xim[2]  = lijstxpix[b];
   yim[2]  = lijstypix[b];
   
   /*------  ------*/
   xmod[3] = lijstxpas[c];
   ymod[3] = lijstypas[c];
   zmod[3] = lijstzpas[c];
   xim[3]  = lijstxpix[c];
   yim[3]  = lijstypix[c];
   
   /*
   for  (i = 0; i < 4; i++)   
      printf("terreinc. %+15.3lf %+15.3lf %+15.3lf\n",  xmod[i] ,ymod[i] , zmod[i]);
   
   for  (i = 0; i < 4; i++)
      printf("fotoc. %+15.3lf %+15.3lf\n", xim[i], yim[i]);
   */
   
   /*
    * Nu wordt SR_DIREC aangeroepen voor de berekening van de rotatie-
    *  matrix en projectiecentrumcoordinaten.
    */
   
   SR_DIREC (xim, yim, cc, xmod, ymod, zmod, disratio, 
	     xpc2, ypc2, zpc2, qa2, qb2, qc2, &nsol);
   
   printf ("\nNumber of direct solutions: %d\n", nsol); 
   
   epsmin = 999999.0;
   for (isol = 1; isol <= nsol; ++isol)          /* begin loop */
   {
      temp_ext.x = xpc2[isol];
      temp_ext.y = ypc2[isol];
      temp_ext.z = zpc2[isol];
      
    
      
      temp_ext.a = qa2[isol];
      temp_ext.b = qb2[isol];
      temp_ext.c = qc2[isol];
      
      Rot_From_Quat(&temp_ext);
      
      /* 
       * Nu wordt de natransformatie uitgevoerd van paspuntcoor. naar
       * fotocoor. en worden verschillen met fotocoor. berekend.
       * Uit de restverschillen in het eerste punt wordt afgeleid welke oplossing
       * juist is.   
       */
      
      for (i = 0; i < 4; ++i)
      {
	 ximt[i] = temp_ext.rot[0][0] * (xmod[i] - temp_ext.x) +
	           temp_ext.rot[1][0] * (ymod[i] - temp_ext.y) +
	           temp_ext.rot[2][0] * (zmod[i] - temp_ext.z);
	 
	 yimt[i] = temp_ext.rot[0][1] * (xmod[i] - temp_ext.x) +
	           temp_ext.rot[1][1] * (ymod[i] - temp_ext.y) +
	           temp_ext.rot[2][1] * (zmod[i] - temp_ext.z);
	 
	 zim     = temp_ext.rot[0][2] * (xmod[i] - temp_ext.x) +
	           temp_ext.rot[1][2] * (ymod[i] - temp_ext.y) +
	           temp_ext.rot[2][2] * (zmod[i] - temp_ext.z);
	 
	 scale   = -cc / zim;
	 ximt[i] *= scale;
	 yimt[i] *= scale;
      }
      
      printf ("\nCoordinate differences in camera coordinates:\n");
      
      for (i = 0; i < 4; ++i)
      {
 
	 epsx[i]  = ximt[i] - xim[i];
	 epsy[i]  = yimt[i] - yim[i];
	 epsxy[i] = epsx[i]*epsx[i] + epsy[i]*epsy[i];
	
	 if (epsxy[0] <= epsmin)
	 {
	    epsmin = epsxy[0];
	    
	    sol->a = temp_ext.a;
	    sol->b = temp_ext.b;
	    sol->c = temp_ext.c;
	    
	    sol->x = temp_ext.x;
	    sol->y = temp_ext.y;
	    sol->z = temp_ext.z;
	 }   
	 
	 printf ("%15.5f %15.5f\n", epsx[i], epsy[i]);
      }                                        /* einde loop */
   }
   
   sol->status = 0; /* Initialisation of validity status */
   Rot_From_Quat(sol);
   Angles_From_Rot(sol);
   
   printf("Approximate values for exterior orientation elm. from direct method:\n");
   Print_Exterior(sol);
}
