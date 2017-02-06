/* dblwrkset.f -- translated by f2c (version 20000817).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

#include "f2c.h"

/* Common Block Declarations */

struct {
    doublereal betkm2, d1km2, dkm2, fsqkm2, hsqkm2, b1km2, dxnkm2, alfkm2;
    integer rkakm2, rkckm2, tkm2, kodkm2;
    doublereal betkm1, d1km1, dkm1, fsqkm1, hsqkm1, b1km1, dxnkm1, alfkm1;
    integer rkakm1, rkckm1, tkm1, kodkm1;
    doublereal pgress, prelin;
} prec_;

#define prec_1 prec_

struct {
    integer nrrest, lattry;
    doublereal bestrk, bestpg;
} back_;

#define back_1 back_

struct {
    doublereal drelpr;
} machin_;

#define machin_1 machin_

/* WRKSET */
/* Subroutine */ int wrkset_(doublereal *a, integer *mda, integer *t, integer 
	*p, integer *n, doublereal *g, doublereal *b, doublereal *tau, 
	integer *mdf, integer *scale, integer *itno, doublereal *diag, 
	integer *active, integer *bnd, integer *inact, integer *q, doublereal 
	*h__, doublereal *gndnrm, integer *p4, doublereal *c__, integer *mdc, 
	integer *m, doublereal *f, integer *mdg, doublereal *x, U_fp hfunc, 
	S_fp ffunc, integer *funcev, integer *jacev, integer *p2, integer *p3,
	 doublereal *dx, doublereal *v1, doublereal *d2, doublereal *d3, 
	integer *rankc2, doublereal *d1nrm, doublereal *dnrm, doublereal *
	b1nrm, doublereal *d__, doublereal *gmat, integer *p1, doublereal *v, 
	doublereal *d1, doublereal *fmat, integer *ranka, doublereal *gres, 
	integer *nohous, integer *time, logical *del, doublereal *pivot, 
	doublereal *v2, doublereal *s, doublereal *u)
{
    /* System generated locals */
    integer a_dim1, a_offset, fmat_dim1, fmat_offset, c_dim1, c_offset, 
	    gmat_dim1, gmat_offset, i__1, i__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    extern /* Subroutine */ int grad_(doublereal *, integer *, integer *, 
	    integer *, doublereal *, doublereal *);
    static integer noeq, j, l;
    extern /* Subroutine */ int addit_(integer *, integer *, integer *, 
	    integer *, integer *), equal_(doublereal *, integer *, doublereal 
	    *, integer *, integer *, integer *, integer *, integer *, integer 
	    *), reord_(doublereal *, integer *, integer *, integer *, 
	    doublereal *, integer *, integer *, integer *, integer *, integer 
	    *, integer *, doublereal *, integer *, doublereal *), unscr_(
	    integer *, integer *, integer *, integer *);
    static integer i2, i3;
    extern /* Subroutine */ int jacdif_(doublereal *, integer *, doublereal *,
	     integer *, S_fp, doublereal *, integer *, doublereal *, integer *
	    ), signch_(integer *, integer *, doublereal *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *, doublereal *), evscal_(
	    integer *, doublereal *, integer *, integer *, integer *, 
	    doublereal *, doublereal *), leaest_(doublereal *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *), gnsrch_(integer *, doublereal *, 
	    integer *, integer *, integer *, doublereal *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, integer *, doublereal *,
	     integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    integer *, integer *, doublereal *, integer *, integer *, integer 
	    *, integer *, integer *, doublereal *, doublereal *, doublereal *,
	     doublereal *, integer *, doublereal *, doublereal *, doublereal *
	    , doublereal *, doublereal *, doublereal *, doublereal *), 
	    mulest_(integer *, doublereal *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *, doublereal *,
	     doublereal *, integer *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *, doublereal *,
	     doublereal *, doublereal *), newpnt_(doublereal *, integer *, 
	    doublereal *, integer *, doublereal *, integer *, U_fp, S_fp, 
	    integer *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *);
    static integer ier;
    static doublereal res, tol;


/*     DETERMINE THE CURRENT WORKING SET BY ESTIMATING THE LAGRANGE */
/*     MULTIPLIERS AND COMPUTE THE GN-SEARCH DIRECTION */
/*     IN ORDER TO DO THAT THE OVER DETERMINED SYSTEM */
/*                T */
/*               A *V = G     IS SOLVED FOR V */

/*     THEN V HOLDS A FIRST ORDER ESTIMATE OF THE MULTIPLIERS */
/*     THE MATRIX A IS DECOMPOSED AS              T */
/*                                      (L@D0) = P1 *A*Q1*FMAT */
/*     WHERE L IS T*T LOWER TRIANGULAR */
/*           P1 IS T*T PERMUTATION MATRIX */
/*           Q1 IS N*N ORTHOGONAL MATRIX */
/*           FMAT IS N*N ORTHOGONAL OR THE IDENTITY */
/*     THE LAGRANGE ESTIMATES CORRESPONDING TO INEQUALITIES ARE */
/*     INSPECTED BY THEIR SIGNS AND COMPARED RELATIVELY TO THE */
/*     RESIDUAL  A(TR)*V-G. */
/*     IF NO ESTIMATE V(I) IMPLIES DELETION ANOTHER ESTIMATE (THE GN- */
/*     ESTIMATE) IS COMPUTED. */
/*     THE GN-ESTIMATE IS THE SOLUTION (U) OF THE OVERDETERMINED SYSTEM */
/*          T        T         T */
/*         A *U = G+C *C*DX = C *(F+C*DX) */

/*     WHERE DX IS THE GN-SEARCH DIRECTION. */

/*     ON ENTRY@D */

/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINIG THE MATRIX A IN THE UPPER LEFT T*N RECTANGLE */
/*             (IF SCALING HAS BEEN USED ARRAY A CONTAINS DIAG*A ) */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     G()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE GRADIENT OF THE OBJECTIVE FUNCTION */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING -H (OR -DIAG*H IF SCALING HAS BEEN DONE) */
/*             WHERE H(I) I=1,2,....,T ARE THE VALUES OF THE CONSTRAINTS */
/*             IN CURRENT WORKING SET */
/*     TAU     REAL SCALAR CONTAINING A SMALL VALUE USED TO DETERMINE */
/*             PSEUDO RANK OF MATRIX A */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     ADD     LOGICAL SCALAR = TRUE IF SOME CONSTRAINTS WERE ADDED */
/*             IN THE LATEST STEP */
/*                               = FALSE IF NOT SO */
/*     SCALE   INTEGER SCALAR =0 IF NO SCALING HAS BEEN DONE */
/*                   > 0 IF ROW SCALING OF MATRIX A HAS BEEN DONE */
/*     ITNO    INTEGER SCALAR CONTAINING THE ITERATION NUMBER */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING THE DIAGONAL ELEMENTS IN THE SCALING MATRIX */
/*             DIAG (IF SCALING HAS NOT BEEN DONE DIAG(I)= THE LENGTH */
/*                   OF ROW NO I IN THE ORIGINAL MATRIX A) */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR THE CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     BND     INTEGER SCALAR CONTAINING MIN(L,N) */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR CONSTRAINTS NOT IN CURRENT */
/*             WORKING SET */
/*     Q       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS NOT IN */
/*             CURRENT WORKING SET */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT CURRENT POINT */
/*     GNDNRM  REAL SCALAR CONTAINING II D II AT POINT X(K-1) COMPUTED */
/*             BY THE GAUSS-NEWTON METHOD */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING INFO. TO COMPUTE A*DX FOR INACTIVE CONSTRAINTS */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING THE JACOBIAN OF THE RESIDUALS */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE CURRENT POINT */
/*     HFUNC   SUBROUTINE NAMES. SEE EXPLANATION IN SUBROUTINE NLSNIP */
/*     FFUNC */
/*     FUNCEV  INTEGER SCALAR CONTAINING @ OF TIMES THE RESIDUALS */
/*             ARE EVALUATED */
/*     JACEV   INTEGER SCALAR CONTAINING @ OF TIMES THE JACOBIANS */
/*             ARE EVALUATED */

/*     ON RETURN@D */
/*                                        T */
/*     A(,)    CONTAINS MATRIX L  FROM  P1 *A*Q1*FMAT = (L@D0) */
/*             AS THE T FIRST COLUMNS AND INFO. TO FORM MATRIX Q1 */
/*     T       CONTAINS NUMBER OF CONSTRAINTS IN UPDATED WORKING SET */
/*     B()     CONTAINS  T */
/*                     P1 *B */
/*     ACTIVE()CONTAINS INDECES FOR THE CONSTRAINTS IN THE */
/*             UPDATED WORKING SET */
/*     INACT() HOLDS INDECES FOR THE CONSTRAINTS NOT IN THE */
/*             UPDATED WORKING SET */
/*     Q       CONTAINS NUMBER OF CONSTRAINTS IN INACTIVE SET */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTS THE PERMUTATION MATRIX P1 IN */
/*             P1(TR)*A*Q1*FMAT */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P2 (IF IT IS USED) */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINS THE GAUSS-NEWTON SEARCH DIRECTION */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+T */
/*             CONTAINS THE COMPOUND VECTOR  (C*DX) */
/*                                           (A*DX) */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINS INFO. TO FORM Q2 (IF IT IS USED) */
/*     D3()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             CONTAINS INFO. TO FORM Q3 */
/*     RANKC2  INTEGER SCALAR-CONTAINS PSEUDO RANK OF MATRIX C2 */
/*                                      -1                     T */
/*             DENOTE  (D)=(D1)= -F-C1*L  *B1    WHERE  (B1)=Q2 *B */
/*                         (D2)                         (B2) */
/*             D1 IS RANKC2*1            B1 IS RANKA*1 */
/*             THEN */
/*     D1NRM   REAL SCALAR-CONTAINS EUCLIDEAN NORM OF D1 */
/*     DNRM    REAL SCALAR-CONTAINS EUCLIDEAB NORM OF D */
/*     B1NRM   REAL SCALAR-CONTAINS EUCLIDEAN NORM OF B1 */
/*     D()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING    T */
/*                         Q3 *(-F-C1*DY1) */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINS MATRIX R AND INFO. TO FORM Q2 (IF IT IS USED) */

/*     P4()    CONTAINS POSSIBLE CHANGED INFO. TO COMPUTE A*DX FOR */
/*             INACTIVE CONSTRAINTS */
/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS THE GN-MULTIPLIER ESTIMATES WHEN T=RANKA. */
/*             OTHERWISE V EQUALS THE USUAL 1ST-ORDER ESTIMATES */
/*     D1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS INFO. TO FORM Q1 */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             CONTAINS A N*N MATRIX (A PRODUCT OF GIVENS ROTATION */
/*             MATRICES) IF TIME>2 */
/*     RANKA   INTEGER SCALAR-CONTAINS THE PSEUDO RANK OF MATRIX A */
/*     GRES    REAL SCALAR-CONTAINS THE EUCLIDEAN NORM OF THE RESIDUAL */
/*               T           T      T */
/*              A *V-G  OR  A *V-G-C *C*DX     DEPENDING ON WHETHER */
/*             V IS A GN- OR ORDINARY 1ST ORDER ESTIMATE */
/*     NOHOUS  INTEGER SCALAR NUMBER OF HOUSEHOLDER TRANSFORMATIONS */
/*             DONE TO TRANSFORM ORIGINAL A TO LOWER TRIANGULAR FORM */
/*     TIME    INTEGER SCALAR-CONTAINS NUMBER OF DELETIONS+2 */
/*     DEL     LOGICAL SCALAR=TRUE IF ANY CONSTRAINT IS DELETED */
/*             =FALSE IF NO DELETION IS DONE */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS INFO. TO FORM Q1 */

/*     WORKING AREAS@D */

/*     V2()       REAL SINGLY SUBSCRIPTED ARRAYS ALL OF */
/*     S(),U()    DIMENSION N */

/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

/*      write(10,*) 'In WRKSET: t= ',t */
    /* Parameter adjustments */
    --u;
    --s;
    --v2;
    --d3;
    --dx;
    --x;
    --g;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    --diag;
    --active;
    --inact;
    --h__;
    --p4;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --d__;
    --v1;
    --f;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --p2;
    --p3;
    --d2;
    --p1;
    --v;
    --d1;
    --pivot;

    /* Function Body */
    j = 0;
    l = *q + *t;
    *time = 1;
    tol = sqrt((doublereal) (*t)) * *tau;

/*     COMPUTE FIRST ORDER ESTIMATES OF LAGRANGE MULTIPLIERS */

    *del = FALSE_;
    mulest_(time, &a[a_offset], mda, t, n, &g[1], &b[1], &j, &tol, &d1[1], &
	    fmat[fmat_offset], mdf, &pivot[1], &p1[1], scale, &diag[1], &v[1],
	     ranka, gres, &s[1], &u[1], &v2[1]);
    *nohous = *ranka;

/*     DETERMINE WHICH (IF ANY) CONSTRAINT THAT SHOULD BE DELETED */

    if (*m + *t <= *n) {
	goto L80;
    }
    signch_(time, &p1[1], &v[1], t, &active[1], bnd, &prec_1.d1km1, gndnrm, 
	    itno, scale, &diag[1], gres, &h__[1], p, &l, &j, &noeq, &u[1], &
	    v2[1]);
    if (noeq == 0) {
	goto L100;
    }
/*      write(10,*) 'Index for delete constraint (NOEQ=) ',noeq */

/*     UPDATE DECOMPOSED MATRIX A BY DELETING APPROPRIATE ROW */
/*     FROM MATRIX L */

    *del = TRUE_;
    reord_(&a[a_offset], mda, t, n, &b[1], &j, &noeq, &active[1], &inact[1], 
	    q, &p4[1], &u[1], scale, &diag[1]);

/*     UPDATE THE THE LOWER TRIANGULAR MATRIX L */

    mulest_(time, &a[a_offset], mda, t, n, &g[1], &b[1], &j, &tol, &d1[1], &
	    fmat[fmat_offset], mdf, &pivot[1], &p1[1], scale, &diag[1], &v[1],
	     ranka, gres, &s[1], &u[1], &v2[1]);
/*      write(10,*) 'RANKA after call mulest ',ranka */

/*     COMPUTE THE GN-SEARCH DIRECTION */

    gnsrch_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], ranka, nohous, &b[
	    1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &f[1], &
	    pivot[1], tau, mdg, scale, &diag[1], &inact[1], q, &p4[1], &p2[1],
	     &p3[1], &dx[1], &v1[1], &d2[1], &d3[1], rankc2, d1nrm, dnrm, 
	    b1nrm, &d__[1], &s[1], &u[1], &gmat[gmat_offset]);

/*     TEST FOR FEASIBLE DIRECTION */

    i2 = inact[*q];
    i3 = *m + l;
    if (v1[i3] >= -h__[i2] && v1[i3] > 0.) {
	goto L200;
    }
    *del = FALSE_;

/*     NOT FEASIBLE. */
/*     RECOMPUTE JACOBIANS, REARANGE WORKING- AND INACTIVE SETS */

/*      write(10,*) 'Not feasible direction: rearange' */
    newpnt_(&x[1], n, &h__[1], &l, &f[1], m, (U_fp)hfunc, (S_fp)ffunc, mda, 
	    mdc, funcev, &a[a_offset], &c__[c_offset], &b[1], &d__[1], &ier);
    ++(*jacev);
    addit_(&active[1], &inact[1], t, q, q);
    equal_(&b[1], &l, &a[a_offset], mda, n, &active[1], t, p, &p4[1]);
    grad_(&c__[c_offset], mdc, m, n, &f[1], &g[1]);
    evscal_(scale, &a[a_offset], mda, t, n, &b[1], &diag[1]);
    unscr_(&active[1], bnd, &l, p);
    j = 0;
    *time = 1;
    mulest_(time, &a[a_offset], mda, t, n, &g[1], &b[1], &j, &tol, &d1[1], &
	    fmat[fmat_offset], mdf, &pivot[1], &p1[1], scale, &diag[1], &v[1],
	     ranka, gres, &s[1], &u[1], &v2[1]);
    *nohous = *ranka;
L80:
    gnsrch_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], ranka, nohous, &b[
	    1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &f[1], &
	    pivot[1], tau, mdg, scale, &diag[1], &inact[1], q, &p4[1], &p2[1],
	     &p3[1], &dx[1], &v1[1], &d2[1], &d3[1], rankc2, d1nrm, dnrm, 
	    b1nrm, &d__[1], &s[1], &u[1], &gmat[gmat_offset]);
    goto L110;
L100:

/*     NO FIRST ORDER ESTIMATE IMPLIES DELETION OF A CONSTRAINT. */
/*     COMPUTE GN-ESTIMATE MULTIPLIERS */

    gnsrch_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], ranka, nohous, &b[
	    1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &f[1], &
	    pivot[1], tau, mdg, scale, &diag[1], &inact[1], q, &p4[1], &p2[1],
	     &p3[1], &dx[1], &v1[1], &d2[1], &d3[1], rankc2, d1nrm, dnrm, 
	    b1nrm, &d__[1], &s[1], &u[1], &gmat[gmat_offset]);

/*     COMPUTE GN-ESTIMATES V(I), I=1,2,....,T */

L110:
    if (prec_1.kodkm1 != 1) {
	goto L200;
    }
/* Computing MIN */
    i__1 = *m, i__2 = *n - *ranka;
    if (*t != *ranka || *rankc2 != min(i__1,i__2)) {
	goto L200;
    }
    leaest_(&a[a_offset], mda, t, &f[1], m, &v1[1], &c__[c_offset], mdc, &p1[
	    1], scale, &diag[1], &s[1], &v[1], &res);

/*     DETERMINE WHICH CONSTRAINT SHOULD BE DELETED */

    signch_(time, &p1[1], &v[1], t, &active[1], bnd, d1nrm, dnrm, itno, scale,
	     &diag[1], &res, &h__[1], p, &l, &j, &noeq, &s[1], &v2[1]);
    if (noeq == 0) {
	goto L200;
    }
    *del = TRUE_;

/*     DELETE APPROPRIATE ROW IN MATRIX A AND UPDATE MATRIX L */

    reord_(&a[a_offset], mda, t, n, &b[1], &j, &noeq, &active[1], &inact[1], 
	    q, &p4[1], &s[1], scale, &diag[1]);
    mulest_(time, &a[a_offset], mda, t, n, &g[1], &b[1], &j, &tol, &d1[1], &
	    fmat[fmat_offset], mdf, &pivot[1], &p1[1], scale, &diag[1], &v[1],
	     ranka, gres, &s[1], &d__[1], &v2[1]);
    ier = 2;
    (*ffunc)(&x[1], n, &f[1], m, &ier, &c__[c_offset], mdc);
    if (ier != 0) {
	goto L150;
    }
    jacdif_(&x[1], n, &f[1], m, (S_fp)ffunc, &c__[c_offset], mdc, &d__[1], &
	    ier);
    *funcev += *n;
L150:
    ++(*jacev);

/*     COMPUTE GN-SEARCH DIRECTION */

    gnsrch_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], ranka, nohous, &b[
	    1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &f[1], &
	    pivot[1], tau, mdg, scale, &diag[1], &inact[1], q, &p4[1], &p2[1],
	     &p3[1], &dx[1], &v1[1], &d2[1], &d3[1], rankc2, d1nrm, dnrm, 
	    b1nrm, &d__[1], &s[1], &u[1], &gmat[gmat_offset]);
    i2 = inact[*q];
    i3 = *m + l;
L200:
    return 0;
} /* wrkset_ */

