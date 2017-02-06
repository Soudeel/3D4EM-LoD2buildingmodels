
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



#include <stdlib.h>
#include <string.h>
#include "Database.h"
#include "math.h"
//#include "digphot_arch.h"

#define EPSILON 0.00000001

int Construct_Qy(double *qy, double *qyd, int ngevonden, double *xvar, 
                 double *yvar, double *covar, double *varfact)
{
   int k, l, n, i, j, num_obs;
   
   k = n = 0;
   for (l = 0; l < ngevonden; l++)
   {
      i = j = 2*k;
      qy[(i+1)*i/2 + j] = xvar[l];
      
      i = j = 2*k; i += 1; 
      qy[(i+1)*i/2 + j] = covar[l];
      
      j++;
      qy[(i+1)*i/2 + j] = yvar[l];
      
      qyd[n++] = xvar[l];
      qyd[n++] = yvar[l];
      
      k++;
   }
   
   num_obs = 2 * ngevonden;
   
   /*------ Determination of variance factor ------*/

   *varfact = 0;
   for (i = 0; i < num_obs; i++)
      *varfact += log(qyd[i]);
   
   *varfact /= i;
   *varfact = exp(*varfact);
   
   for (i = 0; i < num_obs * (num_obs + 1) / 2; i++)
      qy[i] /= *varfact; 
   
   for (i = 0; i < num_obs; i++)
      qyd[i] /= *varfact;
   return(1);
}

int Reduce_Coordinates(int ngevonden, double *xpas, double *ypas, double *zpas,
                       double *campos, double avg[3])
{
   int i;
   
   avg[0] = avg[1] = avg[2] = 0.;
   for (i = 0; i < ngevonden; i++)
   {
        avg[0] += xpas[i];
        avg[1] += ypas[i];
        avg[2] += zpas[i];
   }
      
   for (i = 0; i < 3; i++)
   {
      avg[i] /= (double) ngevonden;
      campos[i] -= avg[i];
   }
   
   for (i = 0; i < ngevonden; i++)
   {
      xpas[i] -= avg[0];
      ypas[i] -= avg[1];
      zpas[i] -= avg[2];
   }
   return(1);
}

int Print_Matrices(double *a, double *y, double *qy, double *qyd, int num_obs)
{
   int i, j;
   
   if (a != NULL)
   {
      printf ("Elements A matrix:\n");
      for (i = 0; i < num_obs; i++)
      {
	 for (j = 0; j < 6; j++)
	    printf("%15.5f ", a[i*6+j]);
	 printf("\n");
      }
   }
   
   if (y != NULL)
   {
      printf("Elements y vector:\n");
      for (i = 0; i < num_obs; i++)
	 printf("%15.5f\n", y[i]);
   }
   
   if (qy != NULL)
   {
      printf ("Elements qy matrix:\n");
      for (i = 1; i <= num_obs; i++)
      {
	 for (j = 1; j <= i; j++)
	    printf("%15.6lf", qy[i * (i - 1)/2 + j - 1]);
	 printf("\n");
      }
   }
   
   if (qyd != NULL)
   {
      printf ("Elements qyd:\n");
      for (i = 0; i < num_obs; i++)
	 printf("%15.5f\n", qyd[i]);
   }
   return(1);
}   

int Print_Results(double *campos, double *quat, double *x, double *avg)
{
   printf("Position of projection centre:\n");
   printf("\tx  = %15.5lf %+15.10lf = %15.5lf\n", campos[0] + avg[0], x[3], campos[0] + avg[0] + x[3]);
   printf("\ty  = %15.5lf %+15.10lf = %15.5lf\n", campos[1] + avg[1], x[4], campos[1] + avg[1] + x[4]);
   printf("\tz  = %15.5lf %+15.10lf = %15.5lf\n", campos[2] + avg[2], x[5], campos[2] + avg[2] + x[5]); 

   printf("Quaternion elements:\n");
   printf("\tq1 = %15.5lf %+15.10lf = %15.5lf\n", quat[0], x[0] / 2, quat[0] + x[0] / 2);
   printf("\tq2 = %15.5lf %+15.10lf = %15.5lf\n", quat[1], x[1] / 2, quat[1] + x[1] / 2);
   printf("\tq3 = %15.5lf %+15.10lf = %15.5lf\n", quat[2], x[2] / 2, quat[2] + x[2] / 2); 
   return(1);
}   

int Exterior_Indirect(int ngevonden, int *lijstpnr,
                      double *lijstxpix, double *lijstypix, 
		              double *lijstxpixvar, double *lijstypixvar,
                      double *lijstxycovar, 
                      double *lijstxpas, 
                      double *lijstypas, double *lijstzpas, 
		              int show_output, double cc, Exterior *sol)
{
   int num_obs, i, j, k, l, m ,n;
   int retval, num_iter;
   double *a, *y, *qy, *qyd, *x, *qx, *err, *w, *lamx, *lamy, *naby;
   double varfact, sigma;
   double campos[3], quat[3];
   double avg[3];
   double max_corr;
   AdjustInfo info, *infoptr;
   
   int Adjust(int, int, double *, double *, double *, double *,
              int, int, double, double *, double *, double *, double *,
              double *, double *, double *, double *, AdjustInfo *);   
#ifdef windows

	#ifdef __GNUC__
	#  if __GNUC__ > 3 // gcc 4.0 or newer
	   void srvv_ls_(double *, double *, int *, double *,
	    	         double *, double *, double *, 
		             double *, double *, double *, double *);
	#  else // Older gcc versions
	   void srvv_ls__(double *, double *, int *, double *,
	    	          double *, double *, double *, 
		              double *, double *, double *, double *);
	#  endif
	#else
	   void srvv_ls(double *, double *, int *, double *,
	     	        double *, double *, double *, 
		            double *, double *, double *, double *);
	#endif
#else
	printf ("ATTENTION: least square adj has been removed from linux version, since new lapack versions come without srvv_ls");
#endif	      
   /*------ Allocating memory for least squares variables ------*/
   num_obs = 2 * ngevonden;
   y   = (double *) calloc(num_obs, sizeof(double));
   a   = (double *)calloc(num_obs * 6, sizeof(double));
   qy  = (double *)calloc(num_obs * (num_obs + 1) / 2, sizeof(double));
   qyd = (double *)calloc(num_obs, sizeof(double));
   
   x    = (double *)calloc(6, sizeof(double) );
   qx   = (double *)calloc(21, sizeof(double) );
   err  = (double *)calloc(num_obs, sizeof(double) );
   w    = (double *)calloc(num_obs, sizeof(double) );
   lamx = (double *)calloc(num_obs, sizeof(double) );
   lamy = (double *)calloc(num_obs, sizeof(double) );
   naby = (double *)calloc(num_obs, sizeof(double) );
   
   if (a    == NULL || y   == NULL || x == NULL || qyd  == NULL || qy   == NULL ||
       qx   == NULL || err == NULL || w == NULL || lamx == NULL || lamy == NULL || 
       naby == NULL)
   {
      printf("vexterior : Not enough memory available for lst.sqr. variables\n");
      return( 0 );
   }
   
   /*------ Initial values from direct method ------*/
   campos[0] = sol->x; campos[1] = sol->y; campos[2] = sol->z;
   quat[0]   = sol->a; quat[1]   = sol->b; quat[2]   = sol->c;
   
   /*------ reduction of the model and projection center coordinates ------*/
   Reduce_Coordinates(ngevonden, lijstxpas, lijstypas, lijstzpas, campos, avg);
   
   /*------ Construction of Qy and Qyd ------*/
   Construct_Qy(qy, qyd, ngevonden, lijstxpixvar, lijstypixvar, lijstxycovar, 
		&varfact);

   /*------ Setup info structure for printing results in Adjust ------*/
   if (show_output) {
     info.n_o_p_p = 2;             /* number of observations per point                */
     info.numbers = lijstpnr;      /* array with numbers of the used points           */
     info.output =  stdout;
     info.units = (char *)calloc(80, sizeof(char));
     strcpy(info.units, "millimeters");
     info.order_string = (char *)calloc(80, sizeof(char));
     strcpy(info.order_string, "x and y coordinates");
     info.order_char = (char *)calloc(2, sizeof(char));
     info.order_char[0] = 'x';
     info.order_char[1] = 'y';
     info.var_num_obs = 0;
     infoptr = &info;
   }
   else {
     infoptr = NULL;
   }
   
   /*------------------------------------------------------------------------------
      Start of the iteration
      ---------------------------------------------------------------------------*/
   num_iter = 1;
   do
   {
      /*------ Construct the design matrix and observation vector ------*/
#ifdef windows
	#ifdef __GNUC__
	#  if __GNUC__ > 3 // gcc 4.0 or newer
	      srvv_ls_(lijstxpix, lijstypix, &ngevonden, &cc, 
	#  else // Older gcc versions
	      srvv_ls__(lijstxpix, lijstypix, &ngevonden, &cc, 
	#  endif
	#else
	      srvv_ls(lijstxpix, lijstypix, &ngevonden, &cc, 
	#endif
		          lijstxpas, lijstypas, lijstzpas, 
		          campos, quat, a, y);
#endif      
      printf("\nIteration number %d\n--------------------------\n", num_iter);;
      Print_Matrices(NULL /*a*/, NULL /*y*/, NULL /*qy*/, NULL /*qyd*/, num_obs);
      
      retval = Adjust(num_obs,   /* IN: Number of observations                  */
	      6,         /* IN: Number of unknowns                              */
	      a,         /* IN: Pointer to the full design matrix A[m][n]       */
	      y,         /* IN: Pointer to the observations y[m]                */
	      qy,        /* IN: variances of the observations (full)            */
	      qyd,       /* IN: variances of the observations (diagonal)        */
	      0,         /* IN: 0 = no correlation; 1 = unit; 2 = correlation   */
	      doAll,     /* IN: depth, doAdjust or doAll (adjustment & testing) */
	      varfact,   /* IN: a priori variance factor                        */
	      &sigma,    /* OUT: a posteriori variance factor (sigma^)          */
	      x,         /* OUT: adjusted unknowns                              */
	      err,       /* OUT: least squares residuals                        */
	      w,         /* OUT: w-tests datasnooping                           */
	      qx,        /* OUT: variances of unknowns                          */
	      naby,      /* OUT: internal reliability, nablas                   */
	      lamy,      /* OUT: internal reliability, lambda-y                 */
	      lamx,      /* OUT: external reliability, lamda-x                  */
	      infoptr);  /* IN: Information about printing */
      
      if (retval)
      {
	 printf("vexterior: Error in adjustment. Exiting.\n");
	 return 0;
      }
      
      /*------ Determine maximum correction ------*/
      max_corr = 0.0;
      for (i = 0; i < 3; i++)
	 if ((fabs(x[i]) / 2.) > max_corr)
	    max_corr = (fabs(x[i]) / 2.);
      for (i = 3; i < 6; i++)
	 if (fabs(x[i]) > max_corr)
	    max_corr = fabs(x[i]);
      printf("Maximum correction: %15.10lf\n", max_corr);
      
      /*------ Output the intermediate results ------*/
      Print_Results(campos, quat, x, avg);
      
      /*------ Update the unknowns ------*/
      campos[0] += x[3];      campos[1] += x[4];      campos[2] += x[5];
      quat[0]   += (x[0]/2.); quat[1]   += (x[1]/2.); quat[2]   += (x[2]/2.);
      
      num_iter++;
   } 
   while (max_corr > EPSILON && num_iter < 10);
   
   sol->x = campos[0] + avg[0];
   sol->y = campos[1] + avg[1];
   sol->z = campos[2] + avg[2];
   sol->a = quat[0];
   sol->b = quat[1];
   sol->c = quat[2];
   
   Rot_From_Quat(sol);
   Angles_From_Rot(sol);
   printf("\nExact solution from indirect method\n");
   Print_Exterior(sol);
   return(1);
}

