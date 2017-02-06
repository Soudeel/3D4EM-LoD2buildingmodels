
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



/*------------------------------------------------------------------------------
  This file contains the following routines:

  Multiple_Intersection  - Forward intersection with multiple images.
  Update_A_and_y_fwdmult - Calculation of coefficients of design matrix and
                           observation vector for the forward intersection.
  LS_Inter               - Forward intersection for one object point.

------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>
#include "Database.h"

struct ImgData { ImgPts   *imgpts;
                 char     *imgname;
                 char     *basename;
                 Exterior *extor;
                 Interior *intor; };

/*------------------------------------------------------------------------------
  Calculation the object coordinates by forward intersection of directions
  from multiple images.

  Author: M.G. Vosselman
  Date  : 26-10-95

  modified d.d. 09-02-01 by F. van den Heuvel
  Purpose: propagation of variances of observations in qy
------------------------------------------------------------------------------*/

void Multiple_Intersection(ObjPts **objptsptr, struct ImgData *imgdata, 
                           int num_imgs)
{

/* Variables */

  ObjPts         *objpts;
  ObjPt          *objpt, objpt2, query_objpt;
  ImgPts         *imgpts;
  ImgPt          *imgpt, *imgpt2, query_imgpt;
  int            i, j, ip, max_objpts, num_obs, index;
  double         *a, *y, *qy, *aptr, *yptr, *qyptr;

/* Functions */

  ObjPt    *Search_ObjPts(ObjPt *, ObjPts *, int *);
  int      LS_Inter(double *, double *, double *, int, ObjPt *);
  void     Update_A_and_y_fwdmult(ImgPt *, Interior *, Exterior *,
                                  double **, double **, double **);


/* Allocation of initial point structure, design matrix and observation
 * vector.
 */

  objpts = (ObjPts *) malloc(sizeof(ObjPts));
  *objptsptr = objpts;
  objpt = (ObjPt *) malloc(10 * sizeof(ObjPt));
  max_objpts = 10;
  objpts->pts = objpt;
  objpts->num_pts = 0;
  a = (double *) malloc(num_imgs * 3 * 2 * sizeof(double));
  y = (double *) malloc(num_imgs * 2 * sizeof(double));
  qy = (double *) malloc(num_imgs * 2 * sizeof(double)); /* modification 09-02-01 */

/* Process all points in all images */

  for (i=0; i<num_imgs; i++) {                         /* loop over images */
    for (ip=0, imgpt=imgdata[i].imgpts->pts;           /* loop over points */
         ip < imgdata[i].imgpts->num_pts;
         ip++, imgpt++) {

/* Check if the object point is already calculated */

      query_objpt.num = imgpt->num;
      if (Search_ObjPts(&query_objpt, objpts, &index) == NULL) {  /* New point */

/* Initialize A matrix and observation vector y, and qy */

        for (j=0, aptr=a; j<6*num_imgs; j++, aptr++) *aptr = 0;
        for (j=0, yptr=y; j<2*num_imgs; j++, yptr++) *yptr = 0;
        for (j=0, qyptr=qy; j<2*num_imgs; j++, qyptr++) *qyptr = 0;

/* Compose the first elements of the A matrix and the observation vector */

        aptr = a; yptr = y; qyptr = qy;
        Update_A_and_y_fwdmult(imgpt, imgdata[i].intor, imgdata[i].extor,
                               &aptr, &yptr, &qyptr);
        num_obs = 2;

/* Find the same point in the other images */

        query_imgpt.num = imgpt->num;
        for (j=i+1; j<num_imgs; j++) {
          imgpt2 = Search_ImgPts(&query_imgpt, imgdata[j].imgpts);
          if (imgpt2 != NULL) {
            Update_A_and_y_fwdmult(imgpt2, imgdata[j].intor, imgdata[j].extor,
                                   &aptr, &yptr, &qyptr);
            num_obs += 2;
          }
        }

/* Calculate the object coordinates if the point has been measured in two
 * or more images.
 */

        if (num_obs >= 4) {
          printf("Point %d has been measured in %d images.\n",
                 imgpt->num, num_obs/2);
          if (LS_Inter(a, y, qy, num_obs, &objpt2)) {

/* Add the object point to the list */

            if (objpts->num_pts == max_objpts) {
              max_objpts += 10;
              objpts->pts  = (ObjPt *) realloc(objpts->pts,
                                               max_objpts * sizeof(ObjPt));
            }
            objpts->pts[objpts->num_pts]     = objpt2;
            objpts->pts[objpts->num_pts].num = imgpt->num;
            objpts->num_pts++;
          }
          else {
            fprintf(stderr, "Calculation of point %d failed.\n",
                    imgpt->num);
            num_obs = 0;
          }
        }

/* Eliminate the image point records from the database if it is only
 * measured in the current image.
 */

        if (num_obs < 4) {
          if (num_obs > 0)
            fprintf(stderr,
                    "Warning: Point %d only has been measured in image %s\n",
                    imgpt->num, imgdata[i].imgname);
          fprintf(stderr, "It can not be used in the adjustment.\n");
        }
      }
    }
  }
}

/* Update_A_and_y_fwdmult
   modified d.d. 09-02-01 by F. van den Heuvel
   purpose: propagation of variances of observations to pseudo observations in y

   modified d.d. 07-03-01 by George Vosselman
   purpose: propagation of variances made optional. If qyptr equals NULL, no
            propagation is done.
*/
   
void Update_A_and_y_fwdmult(ImgPt *imgpt, Interior *intor, Exterior *extor,
                            double **aptr, double **yptr, double **qyptr)
{
  double xp, yp, vxp, vyp, *a, *y, *qy, tmp[6];

  void Pix_To_Metric(Interior *, double, double, double *, double *);

/* Transform to camera coordinates */

  Pix_To_Metric(intor, imgpt->r, imgpt->c, &xp, &yp);
  /* propagate variances, * Warning: not suitable for (large) rotation angles in intor */
  vxp = imgpt->v_c*intor->spacing_c*intor->spacing_c;
  vyp = imgpt->v_r*intor->spacing_r*intor->spacing_r;

/* Coefficients for the x-coordinate */

  a = *aptr;
  *a = xp * extor->rot[0][2] + intor->cc * extor->rot[0][0];  a++;
  tmp[0] = vxp*extor->rot[0][2]*extor->rot[0][2];
  *a = xp * extor->rot[1][2] + intor->cc * extor->rot[1][0];  a++;
  tmp[1] = vxp*extor->rot[1][2]*extor->rot[1][2];
  *a = xp * extor->rot[2][2] + intor->cc * extor->rot[2][0];  a++;
  tmp[2] = vxp*extor->rot[2][2]*extor->rot[2][2];

/* Coefficients for the y-coordinate */

  *a = yp * extor->rot[0][2] + intor->cc * extor->rot[0][1];  a++;
  tmp[3] = vyp*extor->rot[0][2]*extor->rot[0][2];
  *a = yp * extor->rot[1][2] + intor->cc * extor->rot[1][1];  a++;
  tmp[4] = vyp*extor->rot[1][2]*extor->rot[1][2];
  *a = yp * extor->rot[2][2] + intor->cc * extor->rot[2][1];
  tmp[5] = vyp*extor->rot[2][2]*extor->rot[2][2];
  a = *aptr;

/* Observations */

  y = *yptr;
  *y = extor->x * a[0] + extor->y * a[1] + extor->z * a[2];  y++;
  *y = extor->x * a[3] + extor->y * a[4] + extor->z * a[5];
  if (qyptr) {
    qy = *qyptr;
    *qy = tmp[0]*extor->x*extor->x + tmp[1]*extor->y*extor->y +
          tmp[2]*extor->z*extor->z; if(*qy<1e-9) *qy=1e-9; qy++;
    *qy = tmp[3]*extor->x*extor->x + tmp[4]*extor->y*extor->y +
          tmp[5]*extor->z*extor->z; if(*qy<1e-9) *qy=1e-9;
  }

/* Set the pointers for the next point */

  *aptr += 6;
  *yptr += 2;
  if (qyptr) *qyptr +=2;
}

/* LS_Inter
   modified d.d. 09-02-01 by F. van den Heuvel
   purpose: output of information on the observations (residuals, w-tests, etc.)
   therefore the variances of y are now input (qyd)

   modified d.d. 07-03-01 by George Vosselman
   purpose: variance propagation made optional. If qyd equals NULL, a unit
            covariance matrix is used.
*/

int LS_Inter(double *a, double *y, double *qyd, int num_obs, ObjPt *objpt)
{
  int    i;
  double x[3], *qy, *qx, *e, *w, *lamx, *lamy, *naby, varest, varfact, *qyd2;
 
  int Adjust(int, int, double *, double *, double *, double *,
             int, int, double, double *, double *, double *, double *,
             double *, double *, double *, double *, AdjustInfo *);   
   
  qy   = (double *) calloc( (num_obs + 1) * num_obs / 2, sizeof(double) );
  if (qyd == NULL) { /* Generate unit matrix diagonal */
    qyd2 = (double *) calloc(num_obs, sizeof(double));
    for (i=0; i<num_obs; i++) qyd2[i] = 1.0;
  }
  else qyd2 = qyd; /* Use supplied diagonal matrix */
  qx   = (double *) calloc(  6, sizeof(double) );
  e    = (double *) calloc(  num_obs, sizeof(double) );
  w    = (double *) calloc(  num_obs, sizeof(double) );
  lamx = (double *) calloc(  num_obs, sizeof(double) );
  lamy = (double *) calloc(  num_obs, sizeof(double) );
  naby = (double *) calloc(  num_obs, sizeof(double) );

  if (qy == NULL || qx == NULL || e == NULL || w == NULL ||
      lamx == NULL || lamy == NULL || naby == NULL || qyd2 == NULL) {
    printf("ls_inter: Not enough memory available for variables\n");
    return( 0 );
  }

/*------ Call the adjust routine which will do everything for us ------*/

  varfact = 1.0;

  Adjust(num_obs,      /* IN:  Number of observations                    */
         3,            /* IN:  Number of unknowns                        */
         a,            /* IN:  Full design matrix a[m][n]                */
         y,            /* IN:  Observations y[m]                         */
         qy,           /* IN:  variances of the observations (full)      */
         qyd2,         /* IN:  variances of the observations (diagonal)  */
	 no_corr,
	 /* unit_corr,     IN:  0 = no corr; 1 = unit; 2 = correlation    */
         doAll,        /* IN:  doAdjust or doAll (adjustment & testing)  */
         varfact,      /* IN:  a priori variance factor                  */
         &varest,      /* OUT: a posteriori variance factor (sigma^)     */
         x,            /* OUT: adjusted unknowns                         */
         e,            /* OUT: least squares residuals                   */
         w,            /* OUT: w-tests datasnooping                      */
         qx,           /* OUT: variances of the unknowns                 */
         naby,         /* OUT: internal reliability, nablas              */
         lamy,         /* OUT: internal reliability, lambda-y            */
         lamx,         /* OUT: external reliability, lamda-x             */
         NULL);        /* IN: info about printing results                */
                   
/* Store the calculated coordinates */

  objpt->x = x[0];
  objpt->y = x[1];
  objpt->z = x[2];

/* Variance propagation added d.d. 09-02-01 */

  objpt->v_x = qx[0]; objpt->v_y = qx[2]; objpt->v_z = qx[5];
  objpt->cv_xy = qx[1]; objpt->cv_xz = qx[3]; objpt->cv_yz = qx[4];

  /* print the observations, residuals, w-tests, naby, and lamy */

  if (qyd != NULL) {
    printf("\n A posteriori variance factor: %12.4f\n", varest);
    printf("\n Observation  st.deviation  residual   w-test  nabla_y  lambda_y  lambda_x\n");
    for (i=0; i<num_obs; i++) {
      printf(" %11.3f  %8.3f   %8.3f  %8.1f  %9.3f  %8.1f  %8.1f\n",
             y[i],sqrt(qyd2[i]),e[i],w[i],naby[i],lamy[i],lamx[i]);
    }
    printf("\n");
  }
  /* end of modification */

/* Release the memory space */

  free(qy);
  free(qx);
  free(e);
  free(w);
  free(lamx);
  free(lamy);
  free(naby);
  if (qyd == NULL) free(qyd2);

  return(1);
}           
