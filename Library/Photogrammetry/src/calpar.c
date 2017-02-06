
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



/*
                             _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Cal_Trans_Pars
  Author        : R.Th. Ursem
  Creation date : 19 - december - 1994
  Modification by Frank van den Heuvel
  Date          : 06 - Novenber - 1996
  Normally a projective transformation is computed.
  If only 3 points are available a affine transformation is adopted.
  Then par(6) and par(7) are zero and only 6 parameters have to be solved.
  Modification is only valid if routine Adjust is used.
  
  This function calculates the transformation parameters from one 
  system to another using least squares adjustment. Testing of the
  points that were input to this routine will be performed.
  
  Input  : Arrays with coordinate pairs in both systems, variances,
           point numbers the number of points and a file pointer for 
	   the output data.
  Output : A pointer to the transformation parameters that will be 
           determined
---------------------------------------------------------------------
*/

#include "Database.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define _HPADJUST  /*------ Use fortran adjustment and testing routine   ------*/
/*#define _CHECK     ------ Display a test of the transformation pars    ------*/

int Cal_Trans_Pars(double *x_org, double *y_org, double *x_res, double *y_res,
                   double *var, int num_pts, AdjustInfo *info,
                   double par_out[8])
/*
   x_org, y_org       - coordinate pairs in "to" system
   x_res, *y_res      - coordinate pairs in "from" system
   var                - vector with variances of observations
   num_pts            - number of points in all arrays
   info               - information for adjustment function
   par_out            - calculated transformation parameters
*/
   
   /*---------------------------------------------------------------------------    
   * Calculation of the parameters for the uniform projective transformation 
   *
   *            x_res(i) * par(0) + y_res(i) * par(1) + par(2)
   * x_org(i) = ----------------------------------------------
   *            x_res(i) * par(6) + y_res(i) * par(7) + 1
   *
   *            x_res(i) * par(3) + y_res(i) * par(4) + par(5)
   * y_org(i) = ----------------------------------------------
   *            x_res(i) * par(6) + y_res(i) * par(7) + 1
   * 
   *---------------------------------------------------------------------------   
   * Input parameters:
   *
   * y_org, x_org  - coordinate pairs in original image
   * y_res, x_res  - coordinate pairs in resampled image
   * 
   * Output parameters:
   *
   * par         - eight parameters of the projective 2D-2D transformation
   *--------------------------------------------------------------------------- 
   */ 
{
   double *par, *a, *at, ata[8][8], aty[8], *y;
   double *qy, *qx, *e, *w, *naby, *lamx, *lamy, varest;
   int i, j, num, depth;
   int n;
   double checkx, checky, denominator;
   int ipvt[8];      /* pivot vector                  */
   int job, error;   /* input and output numbers      */
   double det[2],    /* determinant of ata (not used) */
      rcond,         /* condition number of ata       */
      z[8];          /* work vector (not used)        */
   
   /* Modification 6-11-96 */
   int nonb=8;         /* number of unknowns */
   
   int Adjust(int, int, double *, double *, double *, double *,
              int, int, double, double *, double *, double *, double *,
              double *, double *, double *, double *, AdjustInfo *);   
   void matmult(double *, double *, double *, int, int, int);
      
   if (num_pts==3) nonb=6;
   /* Modification end */
   
   par  = (double *)calloc(               nonb, sizeof(double) );
   a    = (double *)calloc( 2 * num_pts * nonb, sizeof(double) );
   y    = (double *)calloc( 2 * num_pts,        sizeof(double) );
   if (a == NULL || y == NULL)
   {
      printf("Cal_Trans_Pars: Not enough memory available for variables\n");
      return( 1 );
   }
   
#ifdef _HPADJUST
   qy   = (double *)calloc( 2*num_pts * (2*num_pts+1)/2, sizeof(double));
   qx   = (double *)calloc( 2*num_pts * (2*num_pts+1)/2, sizeof(double));
   e    = (double *)calloc( 2*num_pts,   sizeof(double) );
   w    = (double *)calloc( 2*num_pts,   sizeof(double) );
   lamx = (double *)calloc( 2*num_pts,   sizeof(double) );
   lamy = (double *)calloc( 2*num_pts,   sizeof(double) );
   naby = (double *)calloc( 2*num_pts,   sizeof(double) );
   if (e == NULL || w == NULL || lamx == NULL || lamy == NULL || naby == NULL ||
       qx == NULL || qy == NULL)
   {
      printf("Cal_Trans_Pars: Not enough memory available for variables\n");
      return( 1 );
   }
#else
   at = (double *)calloc( nonb * 2 * num_pts, sizeof(double) );
   if (at == NULL)
   {
      printf("Cal_Trans_Pars: Not enough memory available for variables\n");
      return( 1 );
   }
#endif
   
   /*------ Construction of the FULL design matrix ------*/
   for (num = 0; num < num_pts; num++)
   {
      *(a + nonb * (num * 2    ) + 0) =  x_res[num];
      *(a + nonb * (num * 2    ) + 1) =  y_res[num];
      *(a + nonb * (num * 2    ) + 2) =  1.0;
      *(a + nonb * (num * 2    ) + 3) =  0.0;
      *(a + nonb * (num * 2    ) + 4) =  0.0;
      *(a + nonb * (num * 2    ) + 5) =  0.0;
      if (nonb>6) {
      *(a + nonb * (num * 2    ) + 6) = -1.0 * x_org[num] * x_res[num];
      *(a + nonb * (num * 2    ) + 7) = -1.0 * x_org[num] * y_res[num];
      }
      
      *(a + nonb * (num * 2 + 1) + 0) =  0.0;
      *(a + nonb * (num * 2 + 1) + 1) =  0.0;
      *(a + nonb * (num * 2 + 1) + 2) =  0.0;
      *(a + nonb * (num * 2 + 1) + 3) =  x_res[num];
      *(a + nonb * (num * 2 + 1) + 4) =  y_res[num];
      *(a + nonb * (num * 2 + 1) + 5) =  1.0;
      if (nonb>6) {
      *(a + nonb * (num * 2 + 1) + 6) = -1.0 * y_org[num] * x_res[num];
      *(a + nonb * (num * 2 + 1) + 7) = -1.0 * y_org[num] * y_res[num];
      }
   }
   
   /*------ Construction of the observations vector ------*/
   for (i = 0; i < num_pts; i++)
      for (j = 0; j < 2; j++)
	 if (!j)
	    y[2*i+j] = x_org[i];
	 else
	    y[2*i+j] = y_org[i];
   
#ifdef _HPADJUST
   
   /*------ Call the adjust routine which will do everything for us ------*/
   depth = (2 * num_pts == nonb) ? doAdjust : doAll;
   error = 0;
   error = Adjust(2 * num_pts,  /* IN: Number of observations                 */
	  nonb,         /* IN: Number of unknowns                             */
	  a,            /* IN: Pointer to the full design matrix A[m][n]      */
	  y,            /* IN: Pointer to the observations y[m]               */
	  qy,           /* IN: variances of the observations (full)           */
	  var,          /* IN: variances of the observations (diagonal)       */
	  no_corr,      /* IN: 0 = no correlation; 1 = unit; 2 = correlation  */
	  depth,        /* IN: depth, doAdjust or doAll (adjustment & testing)*/
	  1.0,          /* IN: a priori variance factor                       */
	  &varest,      /* OUT: a posteriori variance factor (sigma^)         */
	  par,          /* OUT: adjusted unknowns                             */
	  e,            /* OUT: least squares residuals                       */
	  w,            /* OUT: w-tests datasnooping                          */
	  qx,           /* OUT: variances of unknowns                         */
	  naby,         /* OUT: internal reliability, nablas                  */
	  lamy,         /* OUT: internal reliability, lambda-y                */
	  lamx,         /* OUT: external reliability, lamda-x                 */
	  info);        /* IN: info for printing the results                  */
   if (error) {
      free(a); free(y); free(e); free(w); free(qy); free(qx);
      free(naby); free(lamx); free(lamy);
      par_out[0] = par_out[4] = 1.0;
      par_out[1] = par_out[2] = par_out[3] =
      par_out[5] = par_out[6] = par_out[7] = 0.0;
      return (error);
   }
#else
   /* This part is not modified for 3 points (6 parameter) solution ! */
  /*------ Construction of the transposed design matrix ------*/
   for (i = 0; i < nonb; i++)
      for (j = 0; j < 2 * num_pts; j++)
	 *(at + i * 2 * num_pts + j) = *(a + j * nonb + i);
   
   /*------ Construction of the normal equations ------*/   
   matmult(at, y, aty, nonb, 2 * num_pts, 1);
   matmult(at, a, (double *) ata, nonb, 2 * num_pts, nonb);

   /*------ calculate the LU decomposition of ata -----------------*/
   n = nonb;
   dgeco(ata, &n, &n, ipvt, &rcond, z);
   if (1.0 + rcond == 1.0) 
   {
      printf("Cal_Trans_Pars: condition number too small\n");
      return( 1 );
   }
   
   /*------ calculate the inverse of ata -------------------------*/ 
   job = 1;
   dgedi(ata, &n, &n, ipvt, det, z, &job);
   
   /*------ solve the normal equation system ---------------------*/
   matmult(ata, aty, par, n, n, 1);
#endif
   
   /* set par_out */
   par_out[6] = par_out[7] = 0.0;
   for (i = 0; i < nonb; i++) par_out[i] = par[i];
   
   /*------ Print the results to the given output file pointer ------*/
   if (info) {
     if (info->output) {
       fprintf(info->output, "\nDetermined transformation equations:\n");
       fprintf(info->output, "    x * %6.3lf + y * %6.3lf + %6.3lf\n",
               par_out[0], par_out[1], par_out[2]);
       fprintf(info->output, "X = --------------------------------\n");
       fprintf(info->output, "    x * %6.3lf + y * %6.3lf + 1\n\n",
               par_out[6], par_out[7]);
       fprintf(info->output, "    x * %6.3lf + y * %6.3lf + %6.3lf\n",
               par_out[3], par_out[4], par_out[5]);
       fprintf(info->output, "Y = --------------------------------\n");
       fprintf(info->output, "    x * %6.3lf + y * %6.3lf + 1\n\n",
               par_out[6], par_out[7]);

#ifdef _CHECK     /*------ Check the determined solution ------*/
   
       fprintf(info->output, "\t         res\t\t\t\tcheck\t\t\t     org\n");
       for (i = 0; i < num_pts; i++) {
          denominator = par[6] * x_res[i] + par[7] * y_res[i] + 1.0;
          checkx = (par[0] * x_res[i] + par[1] * y_res[i] + par[2])/denominator;
          checky = (par[3] * x_res[i] + par[4] * y_res[i] + par[5])/denominator;
          fprintf(info->output, "%15.3lf%15.3lf", x_res[i], y_res[i]);
          fprintf(info->output, "%15.3lf%15.3lf%15.3lf%15.3lf\n",
                  checkx, checky, x_org[i], y_org[i]);
       }

#endif

     }
   }
   
   free(a);
   free(y);

#ifdef _HPADJUST
   free(e);
   free(w);
   free(naby);
   free(lamx);
   free(lamy);
   free(qy);
   free(qx);
#else
   free(at);
#endif
   
   return (0);
}

/*
                             _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Cal_Trans_Pars_Iterative
  Author        : George Vosselman
  Creation date : 18 May 2002
  
  Calculation of parameters of the 2D projective transformation using linearised
  observation equations that minimise the square sum of the misclosures in
  pixels.
  Approximate values are obtained with the above function Cal_Trans_Pars.

---------------------------------------------------------------------
*/
int Cal_Trans_Pars_Iterative(double *x_org, double *y_org, 
                             double *x_res, double *y_res, double *var,
                             int num_pts, AdjustInfo *info, double par[8],
                             int max_iter, double *e, double *w)
/*
  double *x_org, *y_org, coordinate pairs in "to" system 
         *x_res, *y_res, coordinate pairs in "from" system 
         *var;           vector with variances of observations
  int num_pts;           number of points in all arrays
  AdjustInfo *info;      information for adjustment function
  double par[8];         transformation parameters    *out*
  double *e, *w;         residuals and w-test statistics *out*
*/
{
  double *a, *y, *qy, *qx, *lamx, *lamy, *naby, varest, varest_previous;
  double e1, e2, d, difpar[8];
  int    i, num, converged, error, iter;
  int    nunk=8;         /* number of unknowns */
  
  int Adjust(int, int, double *, double *, double *, double *,
             int, int, double, double *, double *, double *, double *,
             double *, double *, double *, double *, AdjustInfo *);   
             
  if (num_pts==3) nunk=6; /* Allow affine transform in case of 3 points */

/* Get approximate solution from linear equations */

  Cal_Trans_Pars(x_org, y_org, x_res, y_res, var, num_pts, NULL, par);
  if (num_pts <= 4) {
    if (info) {
      fprintf(info->output,"Warning, no redundancy in parameter estimation.\n");
      fprintf(info->output,"No adjustment results shown\n");
    }
    return(0);
  }

/* Allocate arrays */

  a    = (double *)calloc( 2 * num_pts * nunk, sizeof(double) );
  y    = (double *)calloc( 2 * num_pts,        sizeof(double) );
  qy   = (double *)calloc( 2*num_pts * (2*num_pts+1)/2, sizeof(double));
  qx   = (double *)calloc( 2*num_pts * (2*num_pts+1)/2, sizeof(double));
  lamx = (double *)calloc( 2*num_pts,   sizeof(double) );
  lamy = (double *)calloc( 2*num_pts,   sizeof(double) );
  naby = (double *)calloc( 2*num_pts,   sizeof(double) );
  if (a == NULL || y == NULL ||
      lamx == NULL || lamy == NULL || naby == NULL ||
      qx == NULL || qy == NULL) {
    printf("Cal_Trans_Pars_Iterative: Not enough memory available for variables\n");
    return(1);
  }

/* Iterate adjustment with linearized observation equations */

  varest_previous = 1e10;
  iter = 0;
  do {
    iter++;

/* Construct A-matrix and y-vector */

    for (num = 0; num < num_pts; num++) {
      e1 = par[0] * x_res[num] + par[1] * y_res[num] + par[2];
      e2 = par[3] * x_res[num] + par[4] * y_res[num] + par[5];
      d  = par[6] * x_res[num] + par[7] * y_res[num] + 1;

      *(a + nunk * (num * 2    ) + 0) =  x_res[num] / d;
      *(a + nunk * (num * 2    ) + 1) =  y_res[num] / d;
      *(a + nunk * (num * 2    ) + 2) =  1.0 / d;
      *(a + nunk * (num * 2    ) + 3) =  0.0;
      *(a + nunk * (num * 2    ) + 4) =  0.0;
      *(a + nunk * (num * 2    ) + 5) =  0.0;
      if (nunk == 8) {
        *(a + nunk * (num * 2    ) + 6) = -e1 * x_res[num] / (d*d);
        *(a + nunk * (num * 2    ) + 7) = -e1 * y_res[num] / (d*d);
      }
      
      *(a + nunk * (num * 2 + 1) + 0) =  0.0;
      *(a + nunk * (num * 2 + 1) + 1) =  0.0;
      *(a + nunk * (num * 2 + 1) + 2) =  0.0;
      *(a + nunk * (num * 2 + 1) + 3) =  x_res[num] / d;
      *(a + nunk * (num * 2 + 1) + 4) =  y_res[num] / d;
      *(a + nunk * (num * 2 + 1) + 5) =  1.0 / d;
      if (nunk == 8) {
        *(a + nunk * (num * 2 + 1) + 6) = -e2 * x_res[num] / (d*d);
        *(a + nunk * (num * 2 + 1) + 7) = -e2 * y_res[num] / (d*d);
      }

      *(y + 2 * num    ) = x_org[num] - e1 / d;
      *(y + 2 * num + 1) = y_org[num] - e2 / d;
    }

/* Adjust */

    error = Adjust(2 * num_pts,  /* IN: Number of observations                */
	   nunk,         /* IN: Number of unknowns                            */
	   a,            /* IN: Pointer to the full design matrix A[m][n]     */
	   y,            /* IN: Pointer to the observations y[m]              */
	   qy,           /* IN: variances of the observations (full)          */
	   var,          /* IN: variances of the observations (diagonal)      */
	   no_corr,      /* IN: 0 = no correlation; 1 = unit; 2 = correlation */
	   doAll,        /* IN: doAdjust or doAll (adjustment & testing)      */
	   1.0,          /* IN: a priori variance factor                      */
	   &varest,      /* OUT: a posteriori variance factor (sigma^)        */
	   difpar,       /* OUT: adjusted unknowns                            */
	   e,            /* OUT: least squares residuals                      */
	   w,            /* OUT: w-tests datasnooping                         */
	   qx,           /* OUT: variances of unknowns                        */
	   naby,         /* OUT: internal reliability, nablas                 */
	   lamy,         /* OUT: internal reliability, lambda-y               */
	   lamx,         /* OUT: external reliability, lamda-x                */
	   NULL);        /* IN: info for printing the results                 */
    if (error) {
      free(a); free(y); free(e); free(w); free(qy); free(qx);
      free(naby); free(lamx); free(lamy);
      par[0] = par[4] = 1.0;
      par[1] = par[2] = par[3] =
      par[5] = par[6] = par[7] = 0.0;
      return (error);
    }
    converged = (varest / varest_previous > 0.999);
    varest_previous = varest;

/* Update the unknowns */

    for (i=0; i<nunk; i++) par[i] += difpar[i];

  } while (!converged && iter < max_iter);

/* Redo last adjustment with output */

  if (info) error = Adjust(2 * num_pts,  /* IN: Number of observations        */
	   nunk,         /* IN: Number of unknowns                            */
	   a,            /* IN: Pointer to the full design matrix A[m][n]     */
	   y,            /* IN: Pointer to the observations y[m]              */
	   qy,           /* IN: variances of the observations (full)          */
	   var,          /* IN: variances of the observations (diagonal)      */
	   no_corr,      /* IN: 0 = no correlation; 1 = unit; 2 = correlation */
	   doAll,        /* IN: doAdjust or doAll (adjustment & testing)      */
	   1.0,          /* IN: a priori variance factor                      */
	   &varest,      /* OUT: a posteriori variance factor (sigma^)        */
	   difpar,       /* OUT: adjusted unknowns                            */
	   e,            /* OUT: least squares residuals                      */
	   w,            /* OUT: w-tests datasnooping                         */
	   qx,           /* OUT: variances of unknowns                        */
	   naby,         /* OUT: internal reliability, nablas                 */
	   lamy,         /* OUT: internal reliability, lambda-y               */
	   lamx,         /* OUT: external reliability, lamda-x                */
	   info);        /* IN: info for printing the results                 */
  return(error);
}

/*
                             _\\|//_ 
                             ( .-. )
--------------------------o00--(_)--00o------------------------------

  Function      : Robust_Cal_Trans_Pars
  Author        : George Vosselman
  Creation date : 18 May 2002
  
  Calculation of parameters of the 2D projective transformation including
  data snooping.
  This function iteratively invokes the above function Cal_Trans_Pars_Iterative
  and removes observations until the rms bound is below some threshold
  or a maximum number of removed observations is reached.
---------------------------------------------------------------------
*/

int Robust_Cal_Trans_Pars(double *x_org, double *y_org,
                          double *x_res, double *y_res, double *var,
                          int initial_num_pts, AdjustInfo *info, double par[8],
                          int max_remove, double rms_bound, int max_iter)
/*
   double *x_org, *y_org, coordinate pairs in "to" system 
          *x_res, *y_res, coordinate pairs in "from" system 
	  *var;               vector with variances of observations
   int initial_num_pts;   number of points in all arrays
   AdjustInfo *info;      information for adjustment function
   double par[8];         determined transformation parameters
   int max_remove;        number of points that may be removed
   double rms_bound;      maximum allowed RMS value of residuals
*/
{
  int    i, error, worst_point, num_remove=0, num_pts, *numbers, iter,
         worst_point_number;
  double resid_x, resid_y, denominator, rms, largest_resid, largest_w,
         *x_org2, *y_org2, *x_res2, *y_res2, x_proj, y_proj, *e, *w;

/* Make a local copy of the points */

  num_pts = initial_num_pts;
  x_org2 = (double *) malloc(num_pts * sizeof(double));
  y_org2 = (double *) malloc(num_pts * sizeof(double));
  x_res2 = (double *) malloc(num_pts * sizeof(double));
  y_res2 = (double *) malloc(num_pts * sizeof(double));
  for (i=0; i<num_pts; i++) {
    x_org2[i] = x_org[i];  y_org2[i] = y_org[i];
    x_res2[i] = x_res[i];  y_res2[i] = y_res[i];
  }
  if (info) {
    if (info->numbers) {
      numbers = info->numbers;
      info->numbers = (int *) malloc(num_pts * sizeof(int));
      for (i=0; i<num_pts; i++) info->numbers[i] = numbers[i];
    }
  }

/* Iterate the parameter estimation */

  iter = 0;
  e = (double *) malloc(2 * num_pts * sizeof(double));
  w = (double *) malloc(2 * num_pts * sizeof(double));
  do {
    iter++;

/* Make an estimate of the transformation parameters */

    error = Cal_Trans_Pars_Iterative(x_org2, y_org2, x_res2, y_res2, var,
                                     num_pts, NULL, par, max_iter, e, w);

/* Determine the root mean square of the residuals */

    if (num_pts > 4) {
      if (info) {
        if (iter == 1) fprintf(info->output,"Residuals after first estimation\n");
        else fprintf(info->output, "Residuals after removing point %d\n",
                     worst_point_number);
      }
      rms = 0.0;
      for (i=0; i<num_pts; i++) {
        if (i == 0) {largest_w = w[0];  worst_point = i;}
        if (w[2*i] > largest_w) {largest_w = w[2*i];  worst_point = i;}
        if (w[2*i+1] > largest_w) {largest_w = w[2*i+1];  worst_point = i;}
        rms += e[2*i] * e[2*i] + e[2*i+1] * e[2*i+1];
        if (info) {
          fprintf(info->output, "%4d%10.3lf%10.3lf%10.3lf%10.3lf%15.3f%15.3f\n",
                  info->numbers[i], e[2*i], e[2*i+1], w[2*i], w[2*i+1],
                  x_org2[i], y_org2[i]);
        }
/*
      denominator = par[6] * x_res2[i] + par[7] * y_res2[i] + 1.0;
      x_proj = (par[0] * x_res2[i] + par[1] * y_res2[i] + par[2]) / denominator;
      resid_x = fabs(x_proj - x_org2[i]);
      if (i == 0) {
        largest_resid = resid_x;  worst_point = i;
      }
      if (largest_resid < resid_x) {largest_resid = resid_x;  worst_point = i;}
      y_proj = (par[3] * x_res2[i] + par[4] * y_res2[i] + par[5]) / denominator;
      resid_y = fabs(y_proj - y_org2[i]);
      if (largest_resid < resid_y) {largest_resid = resid_y;  worst_point = i;}
      rms += resid_x * resid_x + resid_y * resid_y;
      if (info) {
        fprintf(info->output, "%4d%15.3lf%15.3lf%15.3lf%15.3lf\n",
                info->numbers[i], resid_x, resid_y, x_org2[i], y_org2[i]);
      }
*/
    }
      if (rms > 0.0) rms = sqrt(rms/num_pts);
      if (info) {
        fprintf(info->output, "RMS residual value: %7.2f\n", rms);
/*      fprintf(info->output, "Largest residual  : %7.2f\n", largest_resid); */
        fprintf(info->output, "Largest w-test    : %7.2f\n", largest_w);
      }

/* Remove the observation with the largest residual */

      if (!error && num_remove < max_remove && rms && rms > rms_bound) {
        if (info) {
          if (info->numbers) {
            worst_point_number = info->numbers[worst_point];
            fprintf(info->output, "Removing point %d\n", worst_point_number);
          }
          else worst_point_number = 0;
        }
        for (i=worst_point; i<num_pts-1; i++) {
          x_org2[i] = x_org2[i+1];  y_org2[i] = y_org2[i+1];
          x_res2[i] = x_res2[i+1];  y_res2[i] = y_res2[i+1];
          if (info)
            if (info->numbers) 
              info->numbers[i] = info->numbers[i+1];
        }
        num_pts--;
      }
      else if (!error && num_remove == max_remove && rms > rms_bound) {
        if (info)
          fprintf(info->output, "Error: Can not remove more points, but RMS value is still above the specified bound.\n");
      }
      num_remove++;
    }
  } while (!error && num_remove <= max_remove && rms > rms_bound);

  if (rms > rms_bound) error = 1;

/* Redo the last adjustment with output */

  if (info && !error)
    error = Cal_Trans_Pars_Iterative(x_org2, y_org2, x_res2, y_res2, var,
                                     num_pts, info, par, max_iter, e, w);
    
/* Clear local copies */

  free(x_org2);  free(y_org2);  free(x_res2);  free(y_res2);  
  free(e);  free(w);
  if (info)
    if (info->numbers) {
      free(info->numbers);
      info->numbers = numbers;
    }
  return(error);
}
