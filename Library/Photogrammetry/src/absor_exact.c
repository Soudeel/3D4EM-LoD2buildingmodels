
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



#include "Database.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

/*------------- to create a maple commands file ------*/
/* #define MAPLE */

int Exact_Absor_Values(CtrlPts *ctrlpts, ModelPts *modelpts, double *lambda, 
                       int num_pts, int show,
                       double *q1, double *q2, double *q3, 
                       double *dX, double *dY, double *dZ)
{
   double h1;
   double *a, *y, *x, *qyd, *qy, *qx, *err, *w, *lamx, *lamy, *naby;
   double varest, varfact, delta;
   CtrlPt *ctrl;
   ModelPt *model;
   int i, iter, num, retval;
   AdjustInfo info;
   char tmpstr[80];
   
   int Construct_Qy_Absor(double *, double *, double *, CtrlPts *, ModelPts *);
   int Construct_A_and_y_Absor(double *, double *, int *, CtrlPts *,
                               ModelPts *, double, double, double,
                               double, double, double, double);
   num = 3 * num_pts;
   
   /*------ Allocatie van geheugenruimte ------*/
   a    = (double *) calloc( 7 * num, sizeof(double) );
   y    = (double *) calloc(     num, sizeof(double) );
   x    = (double *) calloc(       7, sizeof(double) );
   qyd  = (double *) calloc(     num, sizeof(double) );
   qx   = (double *) calloc(      28, sizeof(double) );
   err  = (double *) calloc(     num, sizeof(double) );
   w    = (double *) calloc(     num, sizeof(double) );
   lamx = (double *) calloc(     num, sizeof(double) );
   lamy = (double *) calloc(     num, sizeof(double) );
   naby = (double *) calloc(     num, sizeof(double) );
   qy   = (double *) calloc( num * (num + 1) / 2, sizeof(double) );
   
   if (a    == NULL || y   == NULL || x == NULL || qyd  == NULL || qy   == NULL ||
       qx   == NULL || err == NULL || w == NULL || lamx == NULL || lamy == NULL || 
       naby == NULL)
   {
      printf("lvabsolute (exact.c): Not enough memory available for lst.sqr. variables\n");
      return( 0 );
   }

   /*------ Filling of the info structure for adjust ------*/
   info.numbers =(int *) calloc( num_pts, sizeof(int));
   if (show) info.output = stdout;
   else info.output = NULL;
   info.n_o_p_p = 3;
   info.var_num_obs = 0;
   
   strcpy(tmpstr, "meters");
   info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.units, tmpstr);
   
   strcpy(tmpstr, "x, y and z coordinates");
   info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.order_string, tmpstr);
   
   info.order_char = (char *)malloc( 3 * sizeof(char) );
   info.order_char[0] = 'x';
   info.order_char[1] = 'y';
   info.order_char[2] = 'z';
   
   Construct_Qy_Absor(qy, qyd, &varfact, ctrlpts, modelpts);
   
   iter = 0;
   do {
      /*------ Construct the design matrix and observations vector ------*/
      Construct_A_and_y_Absor(a, y, info.numbers, ctrlpts, modelpts, 
			*q1, *q2, *q3, *dX, *dY, *dZ, *lambda);

      printf("\nIteration number %d\n-------------------\n", ++iter);
      
      retval = Adjust(num,      /* IN: Number of observations                          */
		      7,        /* IN: Number of unknowns                              */
		      a,        /* IN: Pointer to the full design matrix A[m][n]       */
		      y,        /* IN: Pointer to the observations y[m]                */
		      qy,       /* IN: variances of the observations (full)            */
		      qyd,      /* IN: variances of the observations (diagonal)        */
		      diag_corr,/* IN: 0 = no correlation; 1 = unit; 2 = correlation   */
		      doAll,    /* IN: depth, doAdjust or doAll (adjustment & testing) */
		      varfact,  /* IN: a priori variance factor                        */
		      &varest,  /* OUT: a posteriori variance factor (sigma^)          */
		      x,        /* OUT: adjusted unknowns                              */
		      err,      /* OUT: least squares residuals                        */
		      w,        /* OUT: w-tests datasnooping                           */
		      qx,       /* OUT: variances of unknowns                          */
		      naby,     /* OUT: internal reliability, nablas                   */
		      lamy,     /* OUT: internal reliability, lambda-y                 */
		      lamx,     /* OUT: external reliability, lamda-x                  */
		      &info);   /* IN: info about printing results                     */
		      
      if (retval) 
      {
	 free(a);
	 free(y);
	 free(x);
	 free(qyd);
	 free(qy);
	 free(qx);
	 free(err);
	 free(w);
	 free(lamx);
	 free(lamy);
	 free(naby);
	 
	 return 0;
      }
      
      printf("Quaternions :\n");
      printf("      q1 = %16.10lf +%16.10lf = %16.10lf\n", *q1, x[0], *q1 += x[0]);
      printf("      q2 = %16.10lf +%16.10lf = %16.10lf\n", *q2, x[1], *q2 += x[1]);
      printf("      q2 = %16.10lf +%16.10lf = %16.10lf\n", *q3, x[2], *q3 += x[2]);

      printf("Translations:\n");
      printf("      dX = %16.10lf +%16.10lf = %16.10lf\n", *dX, x[3], *dX += x[3]);
      printf("      dY = %16.10lf +%16.10lf = %16.10lf\n", *dY, x[4], *dY += x[4]);
      printf("      dZ = %16.10lf +%16.10lf = %16.10lf\n", *dZ, x[5], *dZ += x[5]);

      printf("Scale factor:\n");
      printf("  lambda = %16.10lf +%16.10lf = %16.10lf\n", *lambda, x[6], *lambda += x[6]);
      
      delta = 0;
      for (i = 0; i < 7; i++)
	 if (fabs(x[i]) > delta)
	    delta = fabs(x[i]);
      
      printf("\nLargest correction: %16.10lf\n", delta);

   } while (delta > 0.00000001 && iter < 5);
   
   printf("\nDetermined exact values:\n");
   printf("Quaternions : (%lf,%lf,%lf)\n", *q1, *q2, *q3);
   printf("Translations: (%lf,%lf,%lf)\n", *dX, *dY, *dZ);
   printf("Scale factor: %lf\n", *lambda);
   
   free(a);
   free(y);
   free(x);
   free(qyd);
   free(qy);
   free(qx);
   free(err);
   free(w);
   free(lamx);
   free(lamy);
   free(naby);

   return 1;
}

int Construct_A_and_y_Absor(double *A, double *Y, int *num, CtrlPts *ctrlpts,
                            ModelPts *modelpts, double q1, double q2, double q3,
                            double dX, double dY, double dZ, double lambda)
{
   int i,j,k,m;
   double *ptr_a, *ptr_y;
   CtrlPt *ctrl;
   ModelPt *model;
   double x, y, z;
   double h1;
   FILE *fp;
   
   k = m = 0;
   ptr_a = A;
   ptr_y = Y;
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < modelpts->num_pts; j++)
	 if (ctrlpts->pts[i].num == modelpts->pts[j].num &&
	     ctrlpts->pts[i].status == FULL_CTRL )
	 {
	    ctrl = &(ctrlpts->pts[i]);
	    model = &(modelpts->pts[j]);
	    x = ctrl->x; y = ctrl->y; z = ctrl->z;
	    
	    h1 = 1 + q1*q1 + q2*q2 + q3*q3; 
	    num[k++] = ctrl->num;
	    
	    /*------ First equation ------*/
            *ptr_a++ = 2 * ( 
			    (2*q1*q2*q2 + 2*q1*q3*q3)                       * (x-dX) +
			    (q2*q2*q2 + q2 - q2*q1*q1 - 2*q3*q1 + q3*q3*q2) * (y-dY) +
			    (q3*q3*q3 - q3*q1*q1 + q3*q2*q2 + 2*q2*q1 + q3) * (z-dZ)
			   ) / (lambda * h1*h1);
            *ptr_a++ = 2 * (
			    (-2*q2 - 2*q2*q1*q1)                            * (x-dX) +
			    (q1*q1*q1 - 2*q3*q2 - q1*q2*q2 + q1*q3*q3 + q1) * (y-dY) +
			    (q2*q2 - 1 - q1*q1 - q3*q3 - 2*q1*q2*q3)        * (z-dZ)
			   ) / (lambda * h1*h1);
            *ptr_a++ = 2 * (
			    (-2*q3 - 2*q3*q1*q1)                            * (x-dX) +
			    (1 - 2*q1*q2*q3 + q1*q1 + q2*q2 - q3*q3)        * (y-dY) +
			    (q1*q1*q1 + q1*q2*q2 - q1*q3*q3 + 2*q2*q3 + q1) * (z-dZ)
			   ) / (lambda * h1*h1);
	    
            *ptr_a++ = (-1 - q1*q1 + q2*q2 + q3*q3) / (lambda * h1);
            *ptr_a++ = -2 * (q3 + q2*q1) / (lambda * h1);
            *ptr_a++ = -2 * (q3*q1 - q2) / (lambda * h1);
	    
	    *ptr_a++ = (
			(q3*q3 + q2*q2 - q1*q1 - 1) * (x-dX) +
			(-2*q2*q1 - 2*q3)           * (y-dY) +
			(2*q2 - 2*q3*q1)            * (z-dZ)
		       ) / (lambda * lambda * h1);
	    
	    /*------ Second equation ------*/
	    *ptr_a++ = 2 * (
			    (q2*q2*q2 - q2*q1*q1 + q2*q3*q3 + q2 + 2*q3*q1) * (x-dX) +
			    (-2*q1*q2*q2 - 2*q1)                            * (y-dY) +
			    (q2*q2 - 2*q1*q2*q3 - q1*q1 + q3*q3 + 1)        * (z-dZ)
			   ) / (lambda * h1*h1);
	    *ptr_a++ = 2 * (
			    (q1 + q1*q1*q1 + q1*q3*q3 - q1*q2*q2 + 2*q3*q2) * (x-dX) +
			    (2*q2*q1*q1 + 2*q2*q3*q3)                       * (y-dY) +
			    (q3 + q3*q1*q1 + q3*q3*q3 - q3*q2*q2 - 2*q2*q1) * (z-dZ)
			   ) / (lambda * h1*h1);
	    *ptr_a++ = 2 * (
			    (-1 - q1*q1 - q2*q2 + q3*q3 - 2*q1*q2*q3)       * (x-dX) +
			    (-2*q3 - 2*q2*q2*q3)                            * (y-dY) +
			    (q2 + q2*q1*q1 - q2*q3*q3 + q2*q2*q2 - 2*q3*q1) * (z-dZ)
			   ) / (lambda * h1*h1);
	    
	    *ptr_a++ = 2 * (q3 - q1*q2) / (lambda * h1);
	    *ptr_a++ = (-1 + q1*q1 - q2*q2 + q3*q3) / (lambda * h1);
	    *ptr_a++ = -2 * (q1 + q3*q2) / (lambda * h1);
	    
	    *ptr_a++ = (
			(2*q3 - 2*q2*q1)            * (x-dX) +
			(q3*q3 - q2*q2 + q1*q1 - 1) * (y-dY) +
			(-2*q3*q2 - 2*q1)           * (z-dZ)
		       ) / (lambda * lambda * h1);
	    
	    /*------ Third equation ------*/
	    *ptr_a++ = 2 * (
			    (q3 - q3*q1*q1 + q3*q3*q3 + q3*q2*q2 - 2*q2*q1) * (x-dX) +
			    (q1*q1 - q2*q2 - q3*q3 - 1 - 2*q1*q2*q3)        * (y-dY) +
			    (-2*q1 - 2*q1*q3*q3)                            * (z-dZ)
			   ) / (lambda * h1*h1);
	    *ptr_a++ = 2 * (
			    (1 + q1*q1 - q2*q2 + q3*q3 - 2*q1*q2*q3)        * (x-dX) +
			    (q3 + q3*q1*q1 + q3*q3*q3 - q3*q2*q2 + 2*q2*q1) * (y-dY) +
			    (-2*q2 - 2*q2*q3*q3)                            * (z-dZ)
			   ) / (lambda * h1*h1);
	    *ptr_a++ = 2 * (
			    (q1 + q1*q1*q1 - q1*q3*q3 + q1*q2*q2 - 2*q3*q2) * (x-dX) +
			    (q2 + q2*q1*q1 - q2*q3*q3 + q2*q2*q2 + 2*q3*q1) * (y-dY) +
			    (2*q3*q1*q1 + 2*q3*q2*q2)                       * (z-dZ)
			   ) / (lambda * h1*h1);
	    
	    *ptr_a++ = -2 * (q2 + q3*q1) / (lambda * h1);
	    *ptr_a++ = -2 * (q2*q3 - q1) / (lambda * h1);
	    *ptr_a++ = (-1 + q1*q1 + q2*q2 - q3*q3) / (lambda * h1);
	    
	    *ptr_a++ = (
			(-2*q1*q3 - 2*q2)            * (x-dX) +
			(2*q1 - 2*q2*q3)             * (y-dY) +
			(-q3*q3 + q2*q2 + q1*q1 - 1) * (z-dZ)
		       ) / (lambda * lambda * h1);	
	    
	    /*------ Observations ------*/
	    *ptr_y++ = model->x - (
				   (1 + q1*q1 - q2*q2 - q3*q3) * (x-dX) + 
				   (2*q1*q2 + 2*q3)            * (y-dY) +
				   (2*q3*q1 - 2*q2)            * (z-dZ)
				  ) / (lambda * h1);
	    *ptr_y++ = model->y - (
				   (2*q2*q1 - 2*q3)            * (x-dX) +
				   (1 - q1*q1 + q2*q2 - q3*q3) * (y-dY) +
				   (2*q1 + 2*q2*q3)            * (z-dZ)
				  ) / (lambda * h1);
	    *ptr_y++ = model->z - (
				   (2*q2 + 2*q1*q3)            * (x-dX) +
				   (2*q2*q3 - 2*q1)            * (y-dY) +
				   (1 - q1*q1 - q2*q2 + q3*q3) * (z-dZ)
				  ) / (lambda * h1);
	 }

   /*------------------------------- maple stuff ------------------*/
#ifdef MAPLE   
   fp = fopen("exact.maple", "a");
   
   fprintf(fp, "design := matrix([\n");
   for (i = 0; i < 3*k; i++)
   {
      fprintf(fp, "[");
      
      for (j = 1; j < 6; j++)
	 fprintf(fp, "%24.20lf,", A[i*7+j]);
      fprintf(fp, "%24.20lf", A[i*7+6]);
      
      fprintf(fp, "]");
      if (i < 3*k-1) fprintf(fp, ",");
      fprintf(fp, "\n");
   }
   fprintf(fp, "]);\nqx := inverse(transpose(design) &* qyinv &* design);\n");
   
   fprintf(fp, "obs := vector([\n");
   for (i = 0; i < 3*k; i++)
   {
      fprintf(fp, "%24.20lf", Y[i]);
      if (i < 3*k-1) fprintf(fp, ",\n");
   }
   fprintf(fp, "]);\nx := evalm(qx &* transpose(design) &* qyinv &* obs);\n");
   fprintf(fp, "q1 := %24.20lf + expand(obs[1]);\n", q1);
   fprintf(fp, "q2 := %24.20lf + expand(obs[2]);\n", q2);
   fprintf(fp, "q3 := %24.20lf + expand(obs[3]);\n", q3);
   fprintf(fp, "dX := %24.20lf + expand(obs[4]);\n", dX);
   fprintf(fp, "dY := %24.20lf + expand(obs[5]);\n", dY);
   fprintf(fp, "dZ := %24.20lf + expand(obs[6]);\n", dZ);
   fprintf(fp, "lambda := %lf + expand(obs[7]);\n", lambda);
   fclose(fp);
#endif   
   return(1);
}

int Construct_Qy_Absor(double *qy, double *qyd, double *varfact, 
                       CtrlPts *ctrlpts, ModelPts *modelpts)
{
   int i,j,k,l,m,n;
   FILE *fp;
   
   k = n = 0;
   for (l = 0; l < ctrlpts->num_pts; l++)
      for (m = 0; m < modelpts->num_pts; m++)
	 if (ctrlpts->pts[l].num == modelpts->pts[m].num &&
	     ctrlpts->pts[l].status == FULL_CTRL)
	 {
	    i = j = 3*k;
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].v_x;
	    
	    i = j = 3*k; i += 1; 
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].cv_xy;
	    
	    j++;
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].v_y;
	    
	    i = j = 3*k; i += 2; 
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].cv_xz;
	    
	    j++;
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].cv_yz;
	    
	    j++;
	    qy[(i+1)*i/2 + j] = modelpts->pts[m].v_z;

	    qyd[n++] = modelpts->pts[m].v_x;
	    qyd[n++] = modelpts->pts[m].v_y;
	    qyd[n++] = modelpts->pts[m].v_z;
	    
	    k++;
	 }
   
   *varfact = 0;
   for (i = 0; i < 3*k; i++)
      *varfact += log(qyd[i]);
   
   *varfact /= i;
   *varfact = exp(*varfact);
   printf("\nVariance factor for qy matrix = %le\n", *varfact);
   
   for (i = 0; i < 3*k * (3*k + 1) / 2; i++)
      qy[i] /= *varfact; 
   
   for (i = 0; i < 3*k; i++)
      qyd[i] /= *varfact;

   /*------------------------------- maple stuff ------------------*/
#ifdef MAPLE   
   fp = fopen("exact.maple", "w");
   fprintf(fp, "with (linalg);\nvarfact := %26.16lf;\nqy := matrix([\n", *varfact);
   
   /*--------------------- diagonal variance matrix ---------------
   for (i = 0; i < 3*k; i++)
   {
      fprintf(fp, "[");
      
      for (j = 0; j < 3*k; j++)
      {
	 if (i == j)
	    fprintf(fp, "%24.20lf", qyd[i]);
	 else
	    fprintf(fp, "%24.20lf", 0.0);
	 if (j < 3*k-1)
	    fprintf(fp, ",");
      }
      
      fprintf(fp, "]");
      if (i < 3*k-1) fprintf(fp, ",");
      fprintf(fp, "\n");
   }
   ----------------------------------------------------------------*/

   /*------------------------ full variance matrix ----------------*/
   for (i = 1; i <= 3*k; i++)
   {
      fprintf(fp, "[");
      
      for (j = 1; j <= i; j++)
	 if (j < 3*k)
	    fprintf(fp, "%24.20lf,", qy[i*(i-1)/2+j-1]);
	 else
	    fprintf(fp, "%24.20lf", qy[i*(i-1)/2+j-1]);

      for (j = i+1; j < 3*k; j++)
	 if (j < 3*k)
	    fprintf(fp, "%24.20lf,", qy[j*(j-1)/2+i-1]);

      if (i < 3*k)
	 fprintf(fp, "%24.20lf", qy[j*(j-1)/2+i-1]);
      
      fprintf(fp, "]");
      if (i < 3*k) fprintf(fp, ",");
      fprintf(fp, "\n");
   }
   /*--------------------------------------------------------------*/
   
   fprintf(fp, "]);\nqyinv := inverse(varfact * qy);\ndet(qyinv);\ntrace(qyinv);\ndet(qy);\ntrace(qy);\n");
   fclose(fp);
#endif   
   return(1);
}
