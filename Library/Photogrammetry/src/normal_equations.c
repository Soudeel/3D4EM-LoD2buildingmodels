
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



/* Two routines to do a simple least squares adjustment.
 *
 * Update_Normal_Eq
 * Solve_Normal_Eq
 *
 * Last update: 15-07-97
 *
 * George Vosselman
 */

#include <stdlib.h>
#include <math.h>
//#include "digphot_arch.h"

/*------------------------------------------------------------------------------
Updating of the normal equation system by one additional observation.

  a   - row of the design matrix
  y   - corresponding observation
  w   - weight of the observation
  ata - normal matrix
  aty - right hand side of normal equation system
  np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Update_Normal_Eq(double *a, double y, double w, double *ata, 
                      double *aty, int np)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2;
  int    ir, ic;

  w2 = w * w;
  for (ir=0, aptr1=a, ataptr=ata, atyptr=aty;
       ir<np;
       ir++, aptr1++, atyptr++) {
    for (ic=0, aptr2=a;
	 ic<np;
	 ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

/*------------------------------------------------------------------------------
 Partial updating of the normal equation system by one additional observation.
 All values outside the given range are 0.0 and do not require updating.

  a     - row of the design matrix
  y     - corresponding observation
  w     - weight of the observation
  ata   - normal matrix
  aty   - right hand side of normal equation system
  np    - number of unknown parameters
  start - first non-zero index of a
  end   - last non-zero index of a
------------------------------------------------------------------------------*/

void Partial_Update_Normal_Eq1(double *a, double y, double w, double *ata, 
                               double *aty, int np, int start, int end)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *atarowptr;
  int    ir, ic;

  w2 = w * w;
  for (ir=start, aptr1=a+start, atarowptr=ata+start*np, atyptr=aty+start;
       ir<=end;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start, aptr2=a+start, ataptr=atarowptr+start;
	     ic<=end;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

void Partial_Update_Normal_Eq2(double *a, double y, double w, double *ata, 
                               double *aty, int np, int start1, int end1,
							   int start2, int end2)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *atarowptr;
  int    ir, ic;

  w2 = w * w;
  for (ir=start1, aptr1=a+start1, atarowptr=ata+start1*np, atyptr=aty+start1;
       ir<=end1;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start2, aptr1=a+start2, atarowptr=ata+start2*np, atyptr=aty+start2;
       ir<=end2;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

/*------------------------------------------------------------------------------
Solving the normal equation system with LINPACK subroutines.

   ata - normal matrix
   aty - right hand side of normal equation system
	 this array is overwritten with the determined parameters
   np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Solve_Normal_Eq_Cond(double *ata, double *aty, int np, double *rcond)
{
  int    *ipvt, job=0;
  double *z;

#ifdef hpux
  void dgeco(double *, int *, int *, int *, double *, double *);
  void dgesl(double *, int *, int *, int *, double *, int *);
#else
  void dgeco_(double *, int *, int *, int *, double *, double *);
  void dgesl_(double *, int *, int *, int *, double *, int *);
#endif

  ipvt = (int *) malloc(np * sizeof(int));
  z    = (double *) malloc(np * sizeof(double));

/* LU decomposition */

#ifdef hpux
  dgeco(ata, &np, &np, ipvt, rcond, z);
#else
  dgeco_(ata, &np, &np, ipvt, rcond, z);
#endif

/* Solve the equation system */

#ifdef hpux
  dgesl(ata, &np, &np, ipvt, aty, &job);
#else
  dgesl_(ata, &np, &np, ipvt, aty, &job);
#endif

  free(ipvt);  free(z);
}

void Solve_Normal_Eq(double *ata, double *aty, int np)
{
  double rcond;
  Solve_Normal_Eq_Cond(ata, aty, np, &rcond);
}

/*------------------------------------------------------------------------------
Solving the normal equation system with LINPACK subroutines.

   ata - normal matrix
     this array is overwritten with the covariance matrix
   aty - right hand side of normal equation system
	 this array is overwritten with the determined parameters
   np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Invert_And_Solve_Normal_Eq(double *ata, double *aty, int np, double *rcond)
{
  int    i, j, *ipvt, job=1;
  double *z, det[2];

#ifdef hpux
  void dgeco(double *, int *, int *, int *, double *, double *);
  void dgesl(double *, int *, int *, int *, double *, int *);
#else
  void dgeco_(double *, int *, int *, int *, double *, double *);
  void dgesl_(double *, int *, int *, int *, double *, int *);
#endif

  ipvt = (int *) malloc(np * sizeof(int));
  z    = (double *) malloc(np * sizeof(double));

/* LU decomposition */

#ifdef hpux
  dgeco(ata, &np, &np, ipvt, rcond, z);
#else
  dgeco_(ata, &np, &np, ipvt, rcond, z);
#endif

/* Invert the normal matrix */

#ifdef hpux
  dgedi(ata, &np, &np, ipvt, det, z, &job);
#else
  dgedi_(ata, &np, &np, ipvt, det, z, &job);
#endif

/* Solve the unknowns */

  for (i=0; i<np; i++) { z[i] = aty[i];  aty[i] = 0.0; }
  for (i=0; i<np; i++)
  	for (j=0; j<np; j++)
      aty[i] += ata[i*np+j] * z[j];

  free(ipvt);  free(z);
}

void Convert_To_Correlations(double *cov, int np)
{
  int i, j;
  for (i=0; i<np; i++) {
  	for (j=0; j<np; j++) {
  	  if (i == j) continue;
  	  cov[i*np+j] /= sqrt(cov[i*np+i] * cov[j*np+j]);
  	}
  }
  for (i=0; i<np; i++) cov[i*np+i] = 1.0;
}

// Solve equation system using Cholesky decomposition

void Solve_Normal_Eq_Cholesky(double *ata, double *aty, int np)
{
  int i, j, k;
  double *element, *l_j_k, *l_i_k, *aty_k;
  
  // In-place Cholesky factorisation
  for (i=0; i<np; i++) {
  	for (j=0, element=ata+i*np; j<=i; j++, element++) {
  	  if (i == j) {
  	  	for (k=0, l_j_k=ata+j*np; k<j; k++, l_j_k++) {
  	  	  *element -= (*l_j_k) * (*l_j_k);
  	  	}
  	  	*element = sqrt(*element);
  	  }
  	  else {
  	  	for (k=0, l_i_k=ata+i*np, l_j_k=ata+j*np; k<j; k++, l_i_k++, l_j_k++) {
  	  	  *element -= (*l_i_k) * (*l_j_k);
  	  	}
  	  	*element /= ata[j*np+j];
  	  }
  	}
  }
  
  // Forward substitution
  for (i=0, element=aty; i<np; i++, element++) {
  	for (k=0, l_i_k=ata+i*np, aty_k=aty; k<i; k++, l_i_k++, aty_k++) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	*element /= ata[i*np+i];
  }
  
  // Backward substitution
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	for (k=np-1, l_i_k=ata+(np-1)*np+i, aty_k=aty+np-1; k>i; k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= ata[i*np+i];
  }
}


// Solve equation system using Cholesky decomposition, optimised for (partial)
// band matrices.
// ata, aty - Normal equation system
// np       - Number of parameters
// nb       - Number of parameters in the band
// hbw      - Half the band width

void Solve_Normal_Eq_Cholesky_Band(double *ata, double *aty, int np,
                                   int nb, int hbw)
{
  int i, j, k, kstop;
  double *element, *l_j_k, *l_i_k, *aty_k;
  
  // In-place Cholesky factorisation
  for (i=0; i<np; i++) {
  	if (i < nb) { // Inside band part
  	  j = i - hbw + 1;
  	  if (j < 0) j = 0;
  	  for (element=ata+i*np+j; j<=i; j++, element++) {
  	  	k = i - hbw + 1;
  	  	if (k < 0) k = 0;
  	    if (i == j) {
  	  	  for (l_j_k=ata+j*np+k; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	  	  for (l_i_k=ata+i*np+k, l_j_k=ata+j*np+k; k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= ata[j*np+j];
  	    }
  	  }
  	}
  	else { // Outside band part
  	  for (j=0, element=ata+i*np; j<=i; j++, element++) {
  	    if (i == j) {
  	  	  for (k=0, l_j_k=ata+j*np; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	  	  for (k=0, l_i_k=ata+i*np, l_j_k=ata+j*np; k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= ata[j*np+j];
  	    }
  	  }
  	}
  }
  
  // Forward substitution
  for (i=0, element=aty; i<np; i++, element++) {
  	if (i < nb) { // Inside band part
  	  k = i - hbw + 1;
  	  if (k < 0) k = 0;
  	  for (l_i_k=ata+i*np+k, aty_k=aty+k; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	else { // Outside band part
  	  for (k=0, l_i_k=ata+i*np, aty_k=aty; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	*element /= ata[i*np+i];
  }
  
  // Backward substitution
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	// First the part outside the band
  	kstop = nb - 1;           // Stop at the band part
  	if (kstop < i) kstop = i; // and do not cross diagonal
  	for (k=np-1, l_i_k=ata+(np-1)*np+i, aty_k=aty+np-1; k>kstop;
	     k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	// Then the band part
  	k = i + hbw - 1;           // Start at half band width from diagonal
  	if (k > nb - 1) k = nb- 1; // and avoid the non-band area
  	for (l_i_k=ata+k*np+i, aty_k=aty+k; k>i; k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= ata[i*np+i];
  }
}

