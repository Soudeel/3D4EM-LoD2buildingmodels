
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
  * $Log$
  * Revision 1.3  2010/07/09 08:40:05  WINDOWSNT\rutzinger
  * update of folders Library and Tools
  * - removal of NR tags in Makefiles and dev-files
  * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
  *
  * Revision 1.2  2010/06/15 14:41:22  WINDOWSNT\vosselman
  * Prepared for adding copyright texts
  *
  * Revision 1.1  2006/04/06 14:10:16  WINDOWSNT\vosselman
  * *** empty log message ***
  *
  * Revision 1.1.1.1  2005/09/22 11:36:00  vosselm
  * Initial creation of TU Delft - ITC module
  *
  * Revision 1.1  2005/07/07 07:21:29  WINDOWSNT\heuel
  * first release
  *
  * Revision 1.1  2005/07/04 13:43:57  WINDOWSNT\heuel
  * first release, modified version for MinGW (SH)
  *
  * Revision 1.1.1.1  2003/04/09 11:07:37  rabbani
  * basic photogrammetric classes and functions (orientation, in/output, matrices, ...
  *
  * Revision 1.1.1.1  2003/03/07 13:19:28  pfeifer
  * Transformation between object space and image space and between images
  *
  * Revision 1.1.1.1  2003/03/03 14:12:52  rabbani
  * Routines for transformation between image-to-image and image-to-object space
  *
  */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>> 
   >>>> 	Library Routine for forward
   >>>> 
   >>>>  Private: 
   >>>> 
   >>>>   Static: 
   >>>>   Public: 
   >>>> 	lforward
   >>>> 
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */

#include "internals.h"

/* -library_includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdmath.h"
#include "Database.h"
   
typedef struct {
   double m_x, m_y; /* measured position of point                    */
   double c_x, c_y; /* calculated position of point (backprojected)  */
   double d_x, d_y; /* difference between measured and calculated    */
   double v_x, v_y; /* variance of measured point                    */
} SumEntry;

typedef struct {
   int num;
   SumEntry l,r;
} SumRecord;
/* -library_includes_end */


/****************************************************************
* 
*  Routine Name: lforward - *
* 
*       Purpose: Library Routine for forward
*         Input: 
*        Output: 
*       Returns: TRUE (1) on success, FALSE (0) on failure
*  Restrictions: 
*    Written By: Ir. R.Th. Ursem
*          Date: 
*      Verified: 
*  Side Effects: 
* Modifications: Corrected error propagation.
*                03-Nov-1999, George Vosselman
****************************************************************/
/* -library_def */
int lforward(CamPts *pts1, CamPts *pts2, Interior *int1, Interior *int2,
             Exterior *ext1, Exterior *ext2, ObjPts *pts3)
/* -library_def_end */

/* -library_code */
{
  double a[4][3], x_approx[3], x[3], y[4], *qy, *qx, *e, *w, *lamx, *lamy,
         *naby, *var, varest, varfact;
  int i, j, k, l, num_pts, max_pts;
  double enume, denom, enume_deriv, denom_deriv, denomsq, difsum_x, difsum_y;
  SumRecord *summary;
  double qypt[3][3], det;
  AdjustInfo info;
  
  int Adjust(int, int, double *, double *, double *, double *,
             int, int, double, double *, double *, double *, double *,
             double *, double *, double *, double *, AdjustInfo *);    
             
  qy   = (double *)calloc( 10, sizeof(double) );
  qx   = (double *)calloc(  6, sizeof(double) );
  e    = (double *)calloc(  4, sizeof(double) );
  w    = (double *)calloc(  4, sizeof(double) );
  lamx = (double *)calloc(  4, sizeof(double) );
  lamy = (double *)calloc(  4, sizeof(double) );
  naby = (double *)calloc(  4, sizeof(double) );
  var  = (double *)calloc(  4, sizeof(double) );

  if (e == NULL || w == NULL || lamx == NULL || lamy == NULL || naby == NULL) {
    printf("lforward: Not enough memory available for variables\n");
    return( 0 );
  }

  summary  = (SumRecord *)calloc( pts3->num_pts, sizeof( SumRecord ));
   
  info.numbers = (int *)calloc(2, sizeof(int));
  info.units = (char *)calloc(80, sizeof(char));
  info.order_char = (char *)calloc(2, sizeof(char));
  info.order_string = (char *)calloc(80, sizeof(char));
  info.order_char[0] = 'x'; info.order_char[1] = 'y';
  strcpy(info.units, "millimeters");
  strcpy(info.order_string, "x and y coordinates");
  info.output = NULL;
  info.n_o_p_p = 2;
   
  num_pts = 0;
  for (i = 0; i < pts1->num_pts; i++)
    for (j = 0; j < pts2->num_pts; j++)
      if (pts1->pts[i].num == pts2->pts[j].num) {

/* Determination of approximate values by a solution with a linear model
 * without a proper stochastic model.
 */

        /*------ Construct the design matrix ------*/
        a[0][0] = pts1->pts[i].x * ext1->rot[0][2] + int1->cc * ext1->rot[0][0];
        a[0][1] = pts1->pts[i].x * ext1->rot[1][2] + int1->cc * ext1->rot[1][0];
        a[0][2] = pts1->pts[i].x * ext1->rot[2][2] + int1->cc * ext1->rot[2][0];

        a[1][0] = pts1->pts[i].y * ext1->rot[0][2] + int1->cc * ext1->rot[0][1];
        a[1][1] = pts1->pts[i].y * ext1->rot[1][2] + int1->cc * ext1->rot[1][1];
        a[1][2] = pts1->pts[i].y * ext1->rot[2][2] + int1->cc * ext1->rot[2][1];

        a[2][0] = pts2->pts[j].x * ext2->rot[0][2] + int2->cc * ext2->rot[0][0];
        a[2][1] = pts2->pts[j].x * ext2->rot[1][2] + int2->cc * ext2->rot[1][0];
        a[2][2] = pts2->pts[j].x * ext2->rot[2][2] + int2->cc * ext2->rot[2][0];

        a[3][0] = pts2->pts[j].y * ext2->rot[0][2] + int2->cc * ext2->rot[0][1];
        a[3][1] = pts2->pts[j].y * ext2->rot[1][2] + int2->cc * ext2->rot[1][1];
        a[3][2] = pts2->pts[j].y * ext2->rot[2][2] + int2->cc * ext2->rot[2][1];
            
        /*------ Construct the observations matrix ------*/
        y[0] = ext1->x * a[0][0] + ext1->y * a[0][1] + ext1->z * a[0][2];
        y[1] = ext1->x * a[1][0] + ext1->y * a[1][1] + ext1->z * a[1][2];
        y[2] = ext2->x * a[2][0] + ext2->y * a[2][1] + ext2->z * a[2][2];
        y[3] = ext2->x * a[3][0] + ext2->y * a[3][1] + ext2->z * a[3][2];

        info.numbers[0] = pts1->pts[i].num;
        info.numbers[1] = pts2->pts[j].num;
            
        /*------ Call the adjust routine which will do everything for us -----*/
        Adjust(4,           /* IN:  Number of observations                    */
               3,           /* IN:  Number of unknowns                        */
               (double *) a,/* IN:  Full design matrix a[m][n]                */
               y,           /* IN:  Observations y[m]                         */
               qy,          /* IN:  variances of the observations (full)      */
               var,         /* IN:  variances of the observations (diagonal)  */
               unit_corr,   /* IN:  0 = no corr; 1 = unit; 2 = correlation    */
               doAdjust,    /* IN:  doAdjust or doAll (adjustment & testing)  */
               1.0,         /* IN:  a priori variance factor                  */
               &varest,     /* OUT: a posteriori variance factor (sigma^)     */
               x_approx,    /* OUT: adjusted unknowns                         */
               e,           /* OUT: least squares residuals                   */
               w,           /* OUT: w-tests datasnooping                      */
               qx,          /* OUT: variances of the unknowns                 */
               naby,        /* OUT: internal reliability, nablas              */
               lamy,        /* OUT: internal reliability, lambda-y            */
               lamx,        /* OUT: external reliability, lamda-x             */
               &info);      /* IN: info about printing results                */
                   
/* Estimation of position with stochastic model */

        /*------ Construct the design matrix ------*/

        /* left image */
        denom   = (x_approx[0] - ext1->x) * ext1->rot[0][2] + 
                  (x_approx[1] - ext1->y) * ext1->rot[1][2] +
                  (x_approx[2] - ext1->z) * ext1->rot[2][2];
        denomsq = denom * denom;

        /* x-coordinate, left image */
        enume   = (x_approx[0] - ext1->x) * ext1->rot[0][0] + 
                  (x_approx[1] - ext1->y) * ext1->rot[1][0] +
                  (x_approx[2] - ext1->z) * ext1->rot[2][0];
        enume  *= -int1->cc;
        enume_deriv = -int1->cc * ext1->rot[0][0];
        denom_deriv = ext1->rot[0][2];
        a[0][0] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int1->cc * ext1->rot[1][0];
        denom_deriv = ext1->rot[1][2];
        a[0][1] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int1->cc * ext1->rot[2][0];
        denom_deriv = ext1->rot[2][2];
        a[0][2] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        y[0] = pts1->pts[i].x - enume / denom;

        /* y-coordinate, left image */
        enume   = (x_approx[0] - ext1->x) * ext1->rot[0][1] + 
                  (x_approx[1] - ext1->y) * ext1->rot[1][1] +
                  (x_approx[2] - ext1->z) * ext1->rot[2][1];
        enume  *= -int1->cc;
        enume_deriv = -int1->cc * ext1->rot[0][1];
        denom_deriv = ext1->rot[0][2];
        a[1][0] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int1->cc * ext1->rot[1][1];
        denom_deriv = ext1->rot[1][2];
        a[1][1] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int1->cc * ext1->rot[2][1];
        denom_deriv = ext1->rot[2][2];
        a[1][2] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        y[1] = pts1->pts[i].y - enume / denom;

        /* right image */
        denom   = (x_approx[0] - ext2->x) * ext2->rot[0][2] + 
                  (x_approx[1] - ext2->y) * ext2->rot[1][2] +
                  (x_approx[2] - ext2->z) * ext2->rot[2][2];
        denomsq = denom * denom;

        /* x-coordinate, right image */
        enume   = (x_approx[0] - ext2->x) * ext2->rot[0][0] + 
                  (x_approx[1] - ext2->y) * ext2->rot[1][0] +
                  (x_approx[2] - ext2->z) * ext2->rot[2][0];
        enume  *= -int2->cc;
        enume_deriv = -int2->cc * ext2->rot[0][0];
        denom_deriv = ext2->rot[0][2];
        a[2][0] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int2->cc * ext2->rot[1][0];
        denom_deriv = ext2->rot[1][2];
        a[2][1] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int2->cc * ext2->rot[2][0];
        denom_deriv = ext2->rot[2][2];
        a[2][2] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        y[2] = pts2->pts[j].x - enume / denom;

        /* y-coordinate, right image */
        enume   = (x_approx[0] - ext2->x) * ext2->rot[0][1] + 
                  (x_approx[1] - ext2->y) * ext2->rot[1][1] +
                  (x_approx[2] - ext2->z) * ext2->rot[2][1];
        enume  *= -int2->cc;
        enume_deriv = -int2->cc * ext2->rot[0][1];
        denom_deriv = ext2->rot[0][2];
        a[3][0] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int2->cc * ext2->rot[1][1];
        denom_deriv = ext2->rot[1][2];
        a[3][1] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        enume_deriv = -int2->cc * ext2->rot[2][1];
        denom_deriv = ext2->rot[2][2];
        a[3][2] = (enume_deriv * denom - enume * denom_deriv) / denomsq;
        y[3] = pts2->pts[j].y - enume / denom;

        /* covariance matrix of observed camera coordinates */
        qy[0] = pts1->pts[i].v_x;
        qy[1] = pts1->pts[i].cv_xy;
        qy[2] = pts1->pts[i].v_y;
        qy[3] = qy[4] = qy[6] = qy[7] = 0.0;
        qy[5] = pts2->pts[j].v_x;
        qy[8] = pts2->pts[j].cv_xy;
        qy[9] = pts2->pts[j].v_y;

        varfact = log(qy[0]) + log(qy[2]) + log(qy[5]) + log(qy[9]);
        varfact = exp(varfact / 4); 
        if (varfact != 0) for (k = 0; k < 10; k++) qy[k] /= varfact;
                    
        info.output = NULL;
        /*------ Call the adjust routine which will do everything for us -----*/
        Adjust(4,           /* IN:  Number of observations                    */
               3,           /* IN:  Number of unknowns                        */
               (double *) a,/* IN:  Full design matrix a[m][n]                */
               y,           /* IN:  Observations y[m]                         */
               qy,          /* IN:  variances of the observations (full)      */
               var,         /* IN:  variances of the observations (diagonal)  */
               diag_corr,   /* IN:  0 = no corr; 1 = unit; 2 = correlation    */
               doAdjust,    /* IN:  doAdjust or doAll (adjustment & testing)  */
               varfact,     /* IN:  a priori variance factor                  */
               &varest,     /* OUT: a posteriori variance factor (sigma^)     */
               x,           /* OUT: adjusted unknowns                         */
               e,           /* OUT: least squares residuals                   */
               w,           /* OUT: w-tests datasnooping                      */
               qx,          /* OUT: variances of the unknowns                 */
               naby,        /* OUT: internal reliability, nablas              */
               lamy,        /* OUT: internal reliability, lambda-y            */
               lamx,        /* OUT: external reliability, lamda-x             */
               &info);      /* IN: info about printing results                */
                   
        printf("Observations: %7.4f %7.4f %7.4f %7.4f\n",
               y[0], y[1], y[2], y[3]);
        printf("Approximation: %7.4f %7.4f %7.4f\n",
               x_approx[0], x_approx[1], x_approx[2]);
        printf("Estimation: %7.4f %7.4f %7.4f\n",
               x[0], x[1], x[2]);
        /* Apply the corrections */
        for (k=0; k<3; k++) x[k] += x_approx[k];

        /* Store the intersected point */
        pts3->pts[num_pts].num = info.numbers[0];
        pts3->pts[num_pts].x = x[0];
        pts3->pts[num_pts].y = x[1];
        pts3->pts[num_pts].z = x[2];
            
        pts3->pts[num_pts].v_x   = qx[0];
        pts3->pts[num_pts].v_y   = qx[2];
        pts3->pts[num_pts].v_z   = qx[5];
        pts3->pts[num_pts].cv_xy = qx[1];
        pts3->pts[num_pts].cv_xz = qx[3];
        pts3->pts[num_pts].cv_yz = qx[4];
        
        summary[num_pts].num = info.numbers[0];
        summary[num_pts].l.m_x = pts1->pts[i].x;
        summary[num_pts].l.m_y = pts1->pts[i].y;
        summary[num_pts].r.m_x = pts2->pts[j].x;
        summary[num_pts].r.m_y = pts2->pts[j].y;
        
        summary[num_pts].l.v_x = sqrt(pts1->pts[i].v_x);
        summary[num_pts].l.v_y = sqrt(pts1->pts[i].v_y);
        summary[num_pts].r.v_x = sqrt(pts2->pts[j].v_x);
        summary[num_pts].r.v_y = sqrt(pts2->pts[j].v_y);
        
        denom = (x[0] - ext1->x) * ext1->rot[0][2] + 
                (x[1] - ext1->y) * ext1->rot[1][2] +
                (x[2] - ext1->z) * ext1->rot[2][2];
        enume = (x[0] - ext1->x) * ext1->rot[0][0] + 
                (x[1] - ext1->y) * ext1->rot[1][0] +
                (x[2] - ext1->z) * ext1->rot[2][0];
        summary[num_pts].l.c_x = -int1->cc * enume / denom;
        enume = (x[0] - ext1->x) * ext1->rot[0][1] + 
                (x[1] - ext1->y) * ext1->rot[1][1] +
                (x[2] - ext1->z) * ext1->rot[2][1];
        summary[num_pts].l.c_y = -int1->cc * enume / denom;

        denom = (x[0] - ext2->x) * ext2->rot[0][2] + 
                (x[1] - ext2->y) * ext2->rot[1][2] +
                (x[2] - ext2->z) * ext2->rot[2][2];
        enume = (x[0] - ext2->x) * ext2->rot[0][0] + 
                (x[1] - ext2->y) * ext2->rot[1][0] +
                (x[2] - ext2->z) * ext2->rot[2][0];
        summary[num_pts].r.c_x = -int2->cc * enume / denom;
        enume = (x[0] - ext2->x) * ext2->rot[0][1] + 
                (x[1] - ext2->y) * ext2->rot[1][1] +
                (x[2] - ext2->z) * ext2->rot[2][1];
        summary[num_pts].r.c_y = -int2->cc * enume / denom;
            
        summary[num_pts].l.d_x = summary[num_pts].l.m_x -summary[num_pts].l.c_x;
        summary[num_pts].l.d_y = summary[num_pts].l.m_y -summary[num_pts].l.c_y;
        summary[num_pts].r.d_x = summary[num_pts].r.m_x -summary[num_pts].r.c_x;
        summary[num_pts].r.d_y = summary[num_pts].r.m_y -summary[num_pts].r.c_y;
            
        num_pts++;
      }

  printf("\nResults of foreward intersection\n");
  printf("               coordinates                variances                   covariances\n");
  printf(" point   x         y         z       x        y        z         xy        xz        yz\n");
  printf("------------------------------------------------------------------------------------------\n");
  for (i = 0; i < num_pts; i++) {
    printf("%4d%9.5lf,%9.5lf,%9.5lf%9.2le%9.2le%9.2le%10.2le%10.2le%10.2le\n",
           summary[i].num, 
           pts3->pts[i].x,     pts3->pts[i].y,     pts3->pts[i].z,
           pts3->pts[i].v_x,   pts3->pts[i].v_y,   pts3->pts[i].v_z,
           pts3->pts[i].cv_xy, pts3->pts[i].cv_xz, pts3->pts[i].cv_yz);
  }   
  printf("------------------------------------------------------------------------------------------\n");
   
  printf("\nBackprojecting all points:");
  printf("\n                                   L E F T   I M A G E\n");
  printf(" point       measured           calculated          difference        std. dev.\n");
  printf("----------------------------------------------------------------------------------\n");

  difsum_x = 0;
  difsum_y = 0;
  for (i = 0; i < num_pts; i++) {
    if (ABS(summary[i].l.d_x) > 2 * summary[i].l.v_x || 
        ABS(summary[i].l.d_y) > 2 * summary[i].l.v_y)
      printf("*");
    else
      printf(" ");
      
    printf("%4d %9.5lf,%9.5lf  %9.5lf,%9.5lf  %8.5lf,%8.5lf  %7.5lf,%7.5lf\n",
           summary[i].num,
           summary[i].l.m_x, summary[i].l.m_y,
           summary[i].l.c_x, summary[i].l.c_y,
           summary[i].l.d_x, summary[i].l.d_y,
           summary[i].l.v_x, summary[i].l.v_y);
      
    difsum_x += summary[i].l.d_x;
    difsum_y += summary[i].l.d_y;
  }
  printf("----------------------------------------------------------------------------------\n");
  printf("                                                %8.5lf,%8.5lf\n", difsum_x, difsum_y);
   
  printf("\n                                   R I G H T   I M A G E\n");
  printf(" point       measured           calculated          difference        std. dev.\n");
  printf("----------------------------------------------------------------------------------\n");

  difsum_x = 0;
  difsum_y = 0;
  for (i = 0; i < num_pts; i++) {
    if (ABS(summary[i].r.d_x) > 2 * summary[i].r.v_x || 
        ABS(summary[i].r.d_y) > 2 * summary[i].r.v_y)
      printf("*");
    else
      printf(" ");
      
    printf("%4d %9.5lf,%9.5lf  %9.5lf,%9.5lf  %8.5lf,%8.5lf  %7.5lf,%7.5lf\n",
           summary[i].num,
           summary[i].r.m_x, summary[i].r.m_y,
           summary[i].r.c_x, summary[i].r.c_y,
           summary[i].r.d_x, summary[i].r.d_y,
           summary[i].r.v_x, summary[i].r.v_y);
    difsum_x += summary[i].r.d_x;
    difsum_y += summary[i].r.d_y;
  }
  printf("----------------------------------------------------------------------------------\n");
  printf("                                                %8.5lf,%8.5lf\n", difsum_x, difsum_y);

  difsum_x = 0;
  difsum_y = 0;
  for (i = 0; i < num_pts; i++) {
    difsum_x += summary[i].r.d_x;
    difsum_y += summary[i].r.d_y;
    difsum_x += summary[i].l.d_x;
    difsum_y += summary[i].l.d_y;
  }
  printf("\nTotal difference (both images): %8.5lf,%8.5lf\n\n", difsum_x, difsum_y);
  printf("NOTE: A \"*\" indicates that the difference between the measured and the\n");
  printf("      calculated position in the photo is larger than twice (2) the given\n");
  printf("      standard deviation for that point.\n\n");
  return(1);
}
/* -library_code_end */
