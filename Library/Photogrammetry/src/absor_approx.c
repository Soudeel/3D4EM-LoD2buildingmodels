
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
   
int Approx_Absor_Values(CtrlPts *ctrlpts, ModelPts *modelpts, double lambda,
                        int num_pts, int show, double *A, double *B, double *C,
                        double *dX, double *dY, double *dZ)
{
   double q1, q2, q3, w1, w2, w3;
   double *a, *y, *x, *qyd, *qy, *qx, *err, *w, *lamx, *lamy, *naby;
   double varest, varfact, *ptr_a, *ptr_y, *ptr_qyd, det;
   CtrlPt *ctrl;
   ModelPt *model;
   AdjustInfo info;
   char tmpstr[80];
   int i, j, k, num, retval;
      
   num = 3 * num_pts;
   /*------ Allocation of memoryspace ------*/
   a    = (double *) calloc( 6 * num, sizeof(double) );
   y    = (double *) calloc(     num, sizeof(double) );
   x    = (double *) calloc(       6, sizeof(double) );
   qyd  = (double *) calloc(     num, sizeof(double) );
   qy   = (double *) calloc( num * (num + 1) / 2, sizeof(double) );
   qx   = (double *) calloc(      21, sizeof(double) );
   err  = (double *) calloc(     num, sizeof(double) );
   w    = (double *) calloc(     num, sizeof(double) );
   lamx = (double *) calloc(     num, sizeof(double) );
   lamy = (double *) calloc(     num, sizeof(double) );
   naby = (double *) calloc(     num, sizeof(double) );
   
   /*------ Filling of the info structure for adjust ------*/
   info.numbers =(int *) calloc( num_pts, sizeof(int));
   if (show) info.output = stdout;
   else info.output = NULL;
   info.n_o_p_p = 3;
   info.var_num_obs = 0;
   
   strcpy(tmpstr, "meters");
   info.units = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.units, tmpstr);
   
   strcpy(tmpstr, "x, y and z coordinate");
   info.order_string = (char *)malloc( sizeof(char) * (strlen(tmpstr) + 1) );
   strcpy(info.order_string, tmpstr);
   
   info.order_char = (char *)malloc( 3 * sizeof(char) );
   info.order_char[0] = 'x';
   info.order_char[1] = 'y';
   info.order_char[2] = 'z';
   
   /*------ Allocation of memory for adjustment ------*/   
   if (a    == NULL || y   == NULL || x == NULL || qyd == NULL || qy   == NULL ||
       qx   == NULL || err == NULL || w == NULL || lamx == NULL || lamy == NULL || 
       naby == NULL || info.numbers == NULL)
   {
      printf("lvabsolute (approx.c): Not enough memory available for lst.sqr. variables\n");
      return( 0 );
   }

   /*------ Construct the design matrix and observations vector ------*/
   k = 0;
   ptr_a = a;
   ptr_y = y;
   ptr_qyd = qyd;
   for (i = 0; i < ctrlpts->num_pts; i++)
      for (j = 0; j < modelpts->num_pts; j++)
	 if (ctrlpts->pts[i].num == modelpts->pts[j].num &&
	     ctrlpts->pts[i].status == FULL_CTRL )
	 {
	    ctrl = &(ctrlpts->pts[i]);
	    model = &(modelpts->pts[j]);
	    
	    info.numbers[k++] = ctrl->num;
	    
	    /*------ First equation ------*/
	    *ptr_a++ = 0.0;                                   /* q1 */
	    *ptr_a++ = lambda * model->z + ctrl->z;           /* q2 */
	    *ptr_a++ = -1.0 * (lambda * model->y + ctrl->y);  /* q3 */
	    *ptr_a++ = 1.0;                                   /* w1 */
	    *ptr_a++ = 0.0;                                   /* w2 */
	    *ptr_a++ = 0.0;                                   /* w3 */
	    
	    /*------ Second equation ------*/
	    *ptr_a++ = -1.0 * (lambda * model->z + ctrl->z);  /* q1 */
	    *ptr_a++ = 0.0;                                   /* q2 */
	    *ptr_a++ = lambda * model->x + ctrl->x;           /* q3 */
	    *ptr_a++ = 0.0;                                   /* w1 */
	    *ptr_a++ = 1.0;                                   /* w2 */
	    *ptr_a++ = 0.0;                                   /* w3 */
	    
	    /*------ Third equation ------*/
	    *ptr_a++ = lambda * model->y + ctrl->y;           /* q1 */
	    *ptr_a++ = -1.0 * (lambda * model->x + ctrl->x);  /* q2 */
	    *ptr_a++ = 0.0;                                   /* q3 */
	    *ptr_a++ = 0.0;                                   /* w1 */
	    *ptr_a++ = 0.0;                                   /* w2 */
	    *ptr_a++ = 1.0;                                   /* w3 */
	    
	    /*------ Observations ------*/
	    *ptr_y++ = ctrl->x - lambda * model->x;
	    *ptr_y++ = ctrl->y - lambda * model->y;
	    *ptr_y++ = ctrl->z - lambda * model->z;
	    
	    /*------ Variances ------*/
	    *ptr_qyd++ = lambda * lambda * model->v_x;
	    *ptr_qyd++ = lambda * lambda * model->v_y;
	    *ptr_qyd++ = lambda * lambda * model->v_z;
	 }
   
   varfact = 0;
   for (i = 0; i < num; i++)
      varfact += log(qyd[i]);
   varfact /= num;
   varfact = exp(varfact);
   for (i = 0; i < num; i++)
      qyd[i] /= varfact;
   
   retval = Adjust(num,      /* IN: Number of observations                          */
		   6,        /* IN: Number of unknowns                              */
		   a,        /* IN: Pointer to the full design matrix A[m][n]       */
		   y,        /* IN: Pointer to the observations y[m]                */
		   qy,       /* IN: variances of the observations (full)            */
		   qyd,      /* IN: variances of the observations (diagonal)        */
		   no_corr,  /* IN: 0 = no correlation; 1 = unit; 2 = correlation   */
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
   
   if (retval) return (0);
   
   /*------ calculate dX, dY and dZ from q1,q2,q3,w1,w2,w3 ------*/
   q1 = x[0]; q2 = x[1]; q3 = x[2]; 
   w1 = x[3]; w2 = x[4]; w3 = x[5];
   
   det = 1 + q1*q1 + q2*q2 + q3*q3;
   
   *dX = ( (1 +q1*q1) / det)*w1 + ( (q1*q2-q3) / det)*w2 + ( (q2+q1*q3) / det)*w3;
   *dY = ( (q3+q2*q1) / det)*w1 + ( (1 +q2*q2) / det)*w2 + ( (q3*q2-q1) / det)*w3;
   *dZ = ( (q3*q1-q2) / det)*w1 + ( (q1+q2*q3) / det)*w2 + ( (1 +q3*q3) / det)*w3;
   
   *A = q1;
   *B = q2;
   *C = q3;
   
   printf("\nDetermined approximate values:\n");
   printf("Quaternions: (%lf,%lf,%lf)\n", q1, q2, q3);
   printf("Translations: (%lf,%lf,%lf)\n", *dX, *dY, *dZ);
   
   return 1;
}
