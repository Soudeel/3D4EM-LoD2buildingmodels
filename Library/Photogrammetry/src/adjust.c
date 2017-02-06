
/*
              Copyright 2010 Delft University of Technology
 
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

  Function      : Adjust
  Author        : R.Th. Ursem
  Creation date : 19 - December - 1994
  
  This function is a shell around the fortran routine HPADJUST. It 
  handles the output of the adjustment en testing phase and creates
  and disposes some arrays that are normally not interesting to the
  project. This makes calling the adjustment routine a little easier.
  
  An option is provided to print the number of the point that generated
  the observation(s) next to the observation number. This facilitates
  the interpretation of the observation number by the operator. This
  option can turned off by specifying 0 for the number of obserations
  per point (n_o_p_p parameter).
  
  Input  : Described in detail below.
  Output : Described in detail below.
---------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
//#include "digphot_arch.h"
#include "Database.h"

int Adjust(int m, int n, double *a, double *y, double *qy, double *qyd, 
           int iqycor, int depth, double var, double *varest, double *x,
           double *e, double *w, double *qx, double *naby, double *lamy,
           double *lamx, AdjustInfo *info)
                    /*----------- I N P U T  P A R A M E T E R S --------*/
/*   int m;           /* number of observations                            */
/*   int n;           /* number of unknowns                                */
/*   double var;      /* variance-factor (a priori)                        */
/*   double *a;       /* full designmatrix A(m,n)                          */
/*   double *y;       /* observations y(m)                                 */
/*   double *qy;      /* variance matrix observations (undertriagle)       */
/*   double *qyd;     /* variance matrix observations (diagonal)           */
/*   int iqycor;      /* qy: 0 = no correlation; 1 = unit; 2 = correlation */
/*   int depth;       /* depth of the computation (perform testing or not) */
/*   AdjustInfo *info;/* Information about correctly printing results      */
   
                    /*---------- O U T P U T  P A R A M E T E R S -------*/
/*   double *varest;  /* var, estimated = testg (sigma^ after adjustment)  */
/*   double *x;       /* adjusted unknowns                                 */
/*   double *e;       /* least squares residuals                           */
/*   double *w;       /* vector w-tests datasnooping                       */
/*   double *qx;      /* variance matrix adj. unknowns (undertriangle)     */
/*   double *naby;    /* vector nablas int. reliab.                        */
/*   double *lamy;    /* vector (sqrt) lambda-y                            */
/*   double *lamx;    /* vector (sqrt) lambda-x                            */
{
                    /*------------ L O C A L  V A R I A B L E S ---------*/
   int i, j;
   int ret_val;     /* TRUE if an error has occured, otherwise FALSE     */
   int n1;          /* number first part unknowns (for ext. reliab.)     */
   FILE *fp;        /* Smaller name for info->output                     */
   double detqy;    /* determinant qy                                    */
   double detn;     /* determinant normal matrix                         */
   double *qx2;     /* variance matrix last n-n1 unknowns                */
   double *qe;      /* variance matrix residuals (undertr.or diag:qed)   */
   double *qed;     /* variance matrix residuals (undertr.or diag:qed)   */
   double *lamx1;   /* vector (sqrt) lambda-x first n1 unknowns          */
   double *tpar;    /* Test parameters:
		       [0] gamma0, 
		       [1] lambda0,
		       [2] alpha 1 dim test,
		       [3] critical value 1 dim test,
		       [4] alpha multidim test,
		       [5] critical value multidim test,
		       [6] global test value
		       Global test is accepted if [6] < [5].             */
#ifdef hpux
   int hpadjust(int *, int *, int *, double *, double *, double *,
#else
   int hpadjust_(int *, int *, int *, double *, double *, double *,
#endif   
                 double *, int *, int *, double *, double *, double *,
   		         double *, double *, double *, double *, double *,
                 double *, double *, double *, double *, double *,
                 double *, double *, double *);
                       
   /*------ Check for valid adresses ------*/
   if (a == NULL || y == NULL || qy == NULL || qyd == NULL || qx == NULL ||
       x == NULL || e == NULL || w == NULL || naby == NULL || lamy == NULL ||
       lamx == NULL)
   {
      printf("Adjust: ERROR! One of the arguments contains an invalid adress\n");
      return ( 1 );
   }
   
   /*------ Allocate memory for the variance matrices ------*/
   qx2   = (double *)calloc( n * (n + 1) / 2, sizeof(double) );
   qe    = (double *)calloc( m * (m + 1) / 2, sizeof(double) );
   qed   = (double *)calloc( m, sizeof(double));
   lamx1 = (double *)calloc( m, sizeof(double) );
   tpar  = (double *)calloc( 7, sizeof(double) );
   
   if (qx  == NULL || qx2   == NULL || qe   == NULL || 
       qed == NULL || lamx1 == NULL || tpar == NULL)
   {
      fprintf(stderr, "Not enough memory for adjustment variables\n");

      free(qx2);
      free(qe);
      free(qed);
      free(lamx1);
      free(tpar);
      
      return (1);
   }
   
   if (info != NULL) fp = info->output;
   if (fp == NULL) fp = stdout;
   
   if (info != NULL)
   {
      fprintf(fp, "********************* Input Data *********************\n");
      fprintf(fp, "Number of observations (m)     : %10d\n", m);
      fprintf(fp, "Number of unknown (n)          : %10d\n", n);
      fprintf(fp, "variance factor (sigma^2)      : %10.4lg\n\n\n", var);
   }
   
   /*------ Call the fortran routine ------*/
#ifdef hpux
   ret_val = hpadjust(&m, &n, &n, a, y, qy, qyd, &iqycor, &depth, &var, varest, 
#else
   ret_val = hpadjust_(&m, &n, &n, a, y, qy, qyd, &iqycor, &depth, &var, varest, 
#endif
		      &detqy, &detn, x, qx, qx2, e, qe, qed, w, naby, lamy, lamx, 
		      lamx1, tpar);

   if (ret_val)
   {
      free(qx2);
      free(qe);
      free(qed);
      free(lamx1);
      free(tpar);
      
      return (ret_val);
   }
   
   /*------ Print all the interpretation texts to the stream provided by the calling
      function (unless that is a NULL pointer of course) ------*/
   
   /*------ Include variance factor into qx ------*/
   for (i = 0; i < n * (n + 1) / 2; i++)
      qx[i] *= var;
   
   if (info != NULL)
   {
      fprintf(fp, "***************** Adjustment Results *****************\n");
      fprintf(fp, "Determinant variance matrix (qy)   : %15.6lg\n", detqy);
      fprintf(fp, "Determinant normal matrix (qx-INV) : %15.6lg\n\n\n", detn);
      
      fprintf(fp, "Adjusted unknowns and variances (incl. variance factor)\n\n");
      fprintf(fp, "   nr.        x[i]         var-x[i]\n");
      fprintf(fp, "------------------------------------\n");
      for (i = 0; i < n; i++)
	 fprintf(fp, "%5d%15.6lf%15.6lf\n", i+1, x[i], qx[(i+1)*(i+2)/2 - 1]);

      fprintf(fp, "\nVar-Cov.matrix Qx of the adjusted parameters [incl.VarFac]:\n\n");
      for (i = 1; i <= n; i++)
      {
	 for (j = 1; j <= i; j++)
	    fprintf(fp, "%15.6lf", qx[i * (i - 1)/2 + j - 1]);
	 fprintf(fp, "\n");
      }
      
      fprintf(fp, "\nAdjusted observations and variances (incl. variance factor)\n\n");
      if (info->n_o_p_p) fprintf(fp, " point ");
      else fprintf(fp, "   obs");
      
      fprintf(fp, "      adj-y[i]   var-adj-y[i]\n");
      if (info->n_o_p_p) fprintf(fp, "-------");
      fprintf(fp, "------------------------------------\n");
      for (i = 0; i < m; i++)
      {
	 if (info->n_o_p_p) 
	 {
	    if (i % info->n_o_p_p) fprintf(fp, "       ");
	    else fprintf(fp, "%6d ", info->numbers[i / info->n_o_p_p]);
	 }
	 else fprintf(fp, "%5d", i + 1);
	 
	 fprintf(fp, "%15.6lf%15.6lf\n", (y[i]-e[i]) * var, (qyd[i]-qed[i]) * var);
      }
      fprintf(fp, "\n");
      
      if (depth == doAdjust) 
      {
	 free(qx2);
	 free(qe);
	 free(qed);
	 free(lamx1);
	 free(tpar);
	 
	 return( 0 );
      }
      
      fprintf(fp, "****************** Quality Analysis ******************\n");
      fprintf(fp, "Test parameters:\n");
      fprintf(fp, "   Number of observations (m) : %10d\n", m);
      fprintf(fp, "   Number of unknown (n)      : %10d\n", n);
      fprintf(fp, "   Degrees of freedom (m-n)   : %10d\n", m - n);
      fprintf(fp, "   Variance factor (sigma^2)  : %10.4lg\n", var);
      fprintf(fp, "   Gamma-0                    : %10.6lf\n", tpar[0]);
      fprintf(fp, "   Lambda-0                   : %10.6lf\n", tpar[1]);
      fprintf(fp, "One-dimensional w-test\n");
      fprintf(fp, "   Level of significance      : %10.6lf\n", tpar[2]);
      fprintf(fp, "   Critical value             : %10.6lf\n", tpar[3]);
      fprintf(fp, "Global test\n");
      fprintf(fp, "   Level of significance      : %10.6lf\n", tpar[4]);
      fprintf(fp, "   Critical value             : %10.6lf\n\n", tpar[5]);
      fprintf(fp, "Global test : %1.6lg -> %sted\n", tpar[6], (tpar[6] < tpar[5]) ? "accep" : "rejec");
      
      fprintf(fp, "\nTesting and reliability based on conventional alternative hypothesis\n");
      fprintf(fp, "   Residuals, w-test (datasnooping)\n");
      fprintf(fp, "   Internal reliability, Minimal Detectable Bias       : nabla-y\n");
      fprintf(fp, "                         Norm (sqrt)                   : lam-y\n");
      fprintf(fp, "   External reliability, Norm all unknowns (sqrt)      : lam-x\n");
      fprintf(fp, "\n");

      if (info->units != NULL)
	 fprintf(fp, "Residuals are in %s.\n", info->units);
      if (info->order_string != NULL)
	 fprintf(fp, "Order per point is %s.\n", info->order_string);
      fprintf(fp, "\n");
      
      if (info->n_o_p_p) fprintf(fp, " point ");
      else fprintf(fp, "   obs");
      
      fprintf(fp, "    residual    w-test   reject?  nabla-y      lam-y      lam-x\n");
      if (info->n_o_p_p) fprintf(fp, "-------");
      fprintf(fp, "---------------------------------------------------------------------\n");
      for (i = 0; i < m; i++)
      {
	 if (info->n_o_p_p)
	 {
	    if (i % info->n_o_p_p) fprintf(fp, "       ");
	    else fprintf(fp, "%6d ", info->numbers[i / info->n_o_p_p]);
	 }
	 else
	    fprintf(fp, "%5d", i + 1);
	 
	 if (info->n_o_p_p)
	 {
	    if (info->order_char != NULL)
	       fprintf(fp, "%c", info->order_char[i % info->n_o_p_p]);
	    else
	       fprintf(fp, " ");
	 }
	 
	 fprintf(fp, "%11.6lf%11.6lf", e[i], w[i]);
	 if (naby[i] == 0.0) 
	    fprintf(fp, " (nearly) free observation\n");
	 else
	 {
	    if (w[i] < tpar[3])
	       fprintf(fp, "        %11.6lf%11.6lf%11.6lf\n",naby[i], lamy[i], lamx[i]);
	    else
	       fprintf(fp, "   yes  %11.6lf%11.6lf%11.6lf\n",naby[i], lamy[i], lamx[i]);
	 }
      }
      fprintf(fp, "\n***************** End of Adjustment ****************\n");
   }

   free(qx2);
   free(qe);
   free(qed);
   free(lamx1);
   free(tpar);
   
   return (0);
}
