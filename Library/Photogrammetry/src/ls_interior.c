
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
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Database.h"

int LS_Interior(ImgPts *imgpts, CamPts *photopts, int num_pts, double *r0,
                double *c0, double *rs, double *cs, double *rot)
{
   double *a, *y, *x, *qyd, *qy, *qx, *err, *w, *lamx, *lamy, *naby;
   double varest, varfact, delta;
   int i, iter, num, retval, diag_corr;
   AdjustInfo info;
   char tmpstr[80];
   
   int Construct_A_and_y_Intor(double *, double *, int *, ImgPts *, CamPts *,
                               double, double, double, double, double);
   int Construct_Qy_Intor(double *, double *, double *, ImgPts *, CamPts *); 
   int Adjust(int, int, double *, double *, double *, double *,
              int, int, double, double *, double *, double *, double *,
              double *, double *, double *, double *, AdjustInfo *);
                                        
   num = 2 * num_pts;
   
   /*------ Allocatie van geheugenruimte ------*/
   a    = (double *) calloc( 5 * num, sizeof(double) );
   y    = (double *) calloc(     num, sizeof(double) );
   x    = (double *) calloc(       5, sizeof(double) );
   qyd  = (double *) calloc(     num, sizeof(double) );
   qx   = (double *) calloc(      15, sizeof(double) );
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
      printf("lvinterior (ls_interior.c): Not enough memory available for least squares variables\n");
      return( 0 );
   }

   /*------ Filling of the info structure for adjust ------*/
   info.numbers =(int *) calloc( num_pts, sizeof(int));
   info.output = stdout;
   info.n_o_p_p = 2;
   info.var_num_obs = 0;
   
   strcpy(tmpstr, "pixels");
   info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.units, tmpstr);
   
   strcpy(tmpstr, "row and column");
   info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.order_string, tmpstr);
   
   info.order_char = (char *)malloc( 2 * sizeof(char) );
   info.order_char[0] = 'r';
   info.order_char[1] = 'c';
   
   Construct_Qy_Intor(qy, qyd, &varfact, imgpts, photopts);
   diag_corr = 0;
   iter = 0;
   do {
      /*------ Construct the design matrix and observations vector ------*/
      Construct_A_and_y_Intor(a, y, info.numbers, imgpts, photopts, 
			*r0, *c0, *rs, *cs, *rot);

      printf("\nIteration number %d\n-------------------\n", ++iter);
      
      retval = Adjust(num,      /* IN: Number of observations                          */
		      5,        /* IN: Number of unknowns                              */
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
      
      printf("Origin of photo coordinate system :\n");
      printf("      r0 = %16.10lf +%16.10lf = %16.10lf\n", *r0, x[0], *r0 += x[0]);
      printf("      c0 = %16.10lf +%16.10lf = %16.10lf\n", *c0, x[1], *c0 += x[1]);

      printf("Pixel spacing:\n");
      printf("      rs = %16.10lf +%16.10lf = %16.10lf\n", *rs, x[2], *rs += x[2]);
      printf("      cs = %16.10lf +%16.10lf = %16.10lf\n", *cs, x[3], *cs += x[3]);

      printf("Rotation angle:\n");
      printf("   alpha = %16.10lf +%16.10lf =",
	     *rot * 180 / 3.141592653589,
	     x[4] * 180 / 3.141592653589);
      *rot += x[4];
      printf(" %16.10lf\n", *rot * 180 / 3.141592653589);
      
      delta = 0;
      for (i = 0; i < 5; i++)
	 if (fabs(x[i]) > delta)
	    delta = fabs(x[i]);
      
      printf("\nLargest correction: %16.10lf\n", delta);

   } while (delta > 0.00000001 && iter < 5);
   
   printf("\nFinal estimated values:\n");
   printf("Origin of photo coordinate system: (%lf,%lf)\n", *r0, *c0);
   printf("Pixel spacing                    : (%lf,%lf)\n", *rs, *cs);
   printf("Rotation angle                   : %lf\n", *rot * 180 / 3.141592653589 );
   
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

int Construct_A_and_y_Intor(double *A, double *Y, int *num, ImgPts *imgpts,
                            CamPts *photopts, double r0, double c0, 
                            double rs, double cs, double rot)
{
   int i,j,k;
   double *ptr_a, *ptr_y;
   ImgPt *imgpt;
   CamPt *photopt;
   double r, c;
   
   k = 0;
   ptr_a = A;
   ptr_y = Y;
   for (i = 0; i < imgpts->num_pts; i++)
      for (j = 0; j < photopts->num_pts; j++)
	 if (imgpts->pts[i].num == photopts->pts[j].num )
	 {
	    imgpt = &(imgpts->pts[i]);
	    photopt = &(photopts->pts[j]);
	    
	    num[k++] = imgpt->num;
	    
	    /*------ First equation ------*/
            *ptr_a++ = 1.0;
	    *ptr_a++ = 0.0;
	    *ptr_a++ = photopt->x * sin(rot) / (rs * rs) +
		       photopt->y * cos(rot) / (rs * rs);
            *ptr_a++ = 0.0;
	    *ptr_a++ = -1.0 * photopt->x * cos(rot) / rs +
		       photopt->y * sin(rot) / rs;
	    
	    /*------ Second equation ------*/
	    *ptr_a++ = 0.0;
            *ptr_a++ = 1.0;
            *ptr_a++ = 0.0;
	    *ptr_a++ = -1.0 * photopt->x * cos(rot) / (cs * cs) +
		       photopt->y * sin(rot) / (cs * cs);
	    *ptr_a++ = -1.0 * photopt->x * sin(rot) / cs -
		       photopt->y * cos(rot) / cs;
	    
	    /*------ Observations ------*/
	    *ptr_y++ = imgpt->r - r0 + photopt->x * sin(rot) / rs +
		       photopt->y * cos(rot) / rs;
	    *ptr_y++ = imgpt->c - c0 - photopt->x * cos(rot) / cs +
		       photopt->y * sin(rot) / cs;
	 }

  return(1);
}

int Construct_Qy_Intor(double *qy, double *qyd, double *varfact,
                       ImgPts *imgpts, CamPts *photopts)
{
   int i,l,m,n;
   
   n = 0;
   for (l = 0; l < imgpts->num_pts; l++)
      for (m = 0; m < photopts->num_pts; m++)
	 if (imgpts->pts[l].num == photopts->pts[m].num )
	 {
	    qyd[n++] = imgpts->pts[l].v_r;
	    qyd[n++] = imgpts->pts[l].v_c;
	 }
   
   *varfact = 0;
   for (i = 0; i < n; i++)
      *varfact += log(qyd[i]);
   
   *varfact /= i;
   *varfact = exp(*varfact);
   printf("\nVariance factor for qy matrix = %le\n", *varfact);
   
   for (i = 0; i < n; i++)
      qyd[i] /= *varfact;
   return(1);
}
