/* dblredunls.f -- translated by f2c (version 20000817).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

#include "f2c.h"

/* Common Block Declarations */

struct {
    doublereal drelpr;
} machin_;

#define machin_1 machin_

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
    integer ifree;
} negdir_;

#define negdir_1 negdir_

/* Table of constant values */

static integer c__1 = 1;
static integer c__2 = 2;
static doublereal c_b303 = 1.;
static doublereal c_b327 = -1.;
static integer c__0 = 0;


/*     HELP ROUTINES (FILE 1) FOR ENLSIP */
/*     DOUBLE PRECISION VERSION 841005 */
/* NEWPNT */
/* Subroutine */ int newpnt_(doublereal *x, integer *n, doublereal *h__, 
	integer *l, doublereal *f, integer *m, S_fp hfunc, S_fp ffunc, 
	integer *mda, integer *mdc, integer *funcev, doublereal *a, 
	doublereal *c__, doublereal *b, doublereal *d__, integer *ier)
{
    /* System generated locals */
    integer a_dim1, a_offset, c_dim1, c_offset, i__1;

    /* Local variables */
    static integer i__, ctrla, ctrlc;
    extern /* Subroutine */ int jacdif_(doublereal *, integer *, doublereal *,
	     integer *, S_fp, doublereal *, integer *, doublereal *, integer *
	    );


/*     COMPUTE THE JACOBIANS A AND C CORRESPONDING TO THE */
/*     CONSTRAINTS AND THE RESIDUALS RESPECTIVELY */

/*     ON ENTRY@D */

/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE CURRENT POINT */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETRES */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT X */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS AT X */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     HFUNC   SUBROUTINE NAME FOR CONSTRAINTS */
/*     FFUNC   SUBROUTINE NAME FOR RESIDUALS */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     FUNCEV  INTEGER SCALAR CONTAINING TOTAL NUMBER OF FUNCTION */
/*             EVALUATION DONE SO FAR */

/*     ON RETURN@D */

/*     FUNCEV  FUNCEV@D=FUNCEV+N */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             ROW NO. I CONTAINS GRADIENT OF CONSTRAINT NO. I */
/*             I=1,2,......,L */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             ROW NO. I CONTAINS GRADIENT OF RESIDUAL NO. I */
/*             I=1,2,......,M */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             B(I)@D=-H(I)     I=1,2,.......,L */
/*     IER   INTEGER SCALAR CONTAINING A VALUE < -10 TO INDICATE */
/*           A POSSIBLE USER STOP */

/*     WORKING AREA@D */

/*     D()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    --h__;
    --d__;
    --f;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --b;

    /* Function Body */
    ctrlc = 2;
    (*ffunc)(&x[1], n, &f[1], m, &ctrlc, &c__[c_offset], mdc);
    if (ctrlc < -10) {
	*ier = ctrlc;
    }
    if (*ier < -10) {
	goto L40;
    }
    if (ctrlc != 0) {
	goto L10;
    }

/*     COMPUTE THE JACOBIAN NUMERICALLY */

    jacdif_(&x[1], n, &f[1], m, (S_fp)ffunc, &c__[c_offset], mdc, &d__[1], 
	    ier);
    if (*ier < -10) {
	goto L40;
    }
    *funcev += *n;
L10:
    if (*l <= 0) {
	return 0;
    }
    ctrla = 2;
    (*hfunc)(&x[1], n, &h__[1], l, &ctrla, &a[a_offset], mda);
    if (ctrla < -10) {
	*ier = ctrla;
    }
    if (*ier < -10) {
	goto L40;
    }
    if (ctrla != 0) {
	goto L20;
    }

/*     COMPUTE THE JACOBIAN NUMERICALLY */

    jacdif_(&x[1], n, &h__[1], l, (S_fp)hfunc, &a[a_offset], mda, &b[1], ier);
    if (*ier < -10) {
	goto L40;
    }
L20:
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	b[i__] = -h__[i__];
/* L30: */
    }
L40:
/* L101: */
/*      write(10,*) 'Constraints' */
/*      write(10,101) (h(i),i=1,l) */
/*      write(10,*) 'Functions' */
/*      write(10,101) (f(i),i=1,m) */
    return 0;
} /* newpnt_ */

/* EQUAL */
/* Subroutine */ int equal_(doublereal *b, integer *l, doublereal *a, integer 
	*mda, integer *n, integer *active, integer *t, integer *p, integer *
	p4)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static doublereal temp;
    static integer i__, j, index, ik, ip;


/*     MOVE THE CONSTRAINTS IN CURRENT WORKING SET TO THE TOP */
/*     PART OF ARRAY B */
/*     THE CORRESPONDING GRADIENTS ARE MOVED TO THE TOP ROWS */
/*     OF THE ARRAY A */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --b;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --active;
    --p4;

    /* Function Body */
    if (*l <= 0) {
	goto L20;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p4[i__] = i__;
/* L10: */
    }
L20:
    if (*t <= 0 || *t == *p) {
	return 0;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	index = active[i__];
	ip = p4[index];
	if (ip == i__) {
	    goto L50;
	}
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    temp = a[i__ + j * a_dim1];
	    a[i__ + j * a_dim1] = a[ip + j * a_dim1];
	    a[ip + j * a_dim1] = temp;
/* L30: */
	}
	temp = b[i__];
	b[i__] = b[ip];
	b[ip] = temp;
	i__2 = *t;
	for (j = 1; j <= i__2; ++j) {
	    if (i__ != p4[j]) {
		goto L40;
	    }
	    ik = j;
L40:
	    ;
	}
	p4[ik] = ip;
	p4[index] = i__;
L50:
	;
    }
    return 0;
} /* equal_ */

/* EVSCAL */
/* Subroutine */ int evscal_(integer *scale, doublereal *a, integer *mda, 
	integer *t, integer *n, doublereal *b, doublereal *diag)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static doublereal rowi;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, j;
    static doublereal anorm;


/*     SCALE THE SYSTEM  A*DX = B    IF SO INDICATED BY FORMING */
/*     A@D=DIAG*A      B@D=DIAG*B */

/*     ON ENTRY@D */

/*     SCALE   INTEGER SCALAR =0 IF NO SCALING SHOULD BE DONE */
/*             >0 IF SCALING SHOULD BE DONE */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING THE JACOBIAN OF THE WORKING SET */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS */
/*             IN CURRENT WORKING SET */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING THE NEGATIVE VALUE OF THE CONSTRAINTS IN THE */
/*             CURRENT WORKING SET */

/*     ON RETURN@D */

/*     A(,)    CONTAINS THE SCALED JACOBIAN IF SCALE>0 */
/*     B()     CONTAINS THE SCALED RIGHT HAND SIDE IF SCALE>0 */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS THE DIAGONAL ELEMENTS OF THE SCALING MATRIX */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;
    --diag;

    /* Function Body */
    anorm = 0.;
    if (*t == 0) {
	return 0;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	rowi = enlsip_dnrm2_(n, &a[i__ + a_dim1], mda);
	if (rowi > anorm) {
	    anorm = rowi;
	}
	diag[i__] = rowi;
	if (*scale == 0) {
	    goto L20;
	}
	if (rowi == 0.) {
	    rowi = 1.;
	}
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    a[i__ + j * a_dim1] /= rowi;
/* L10: */
	}
	b[i__] /= rowi;
	diag[i__] = 1. / rowi;
L20:
	;
    }
    return 0;
} /* evscal_ */

/* GNSRCH */
/* Subroutine */ int gnsrch_(integer *time, doublereal *a, integer *mda, 
	integer *t, integer *n, doublereal *d1, integer *p1, integer *ranka, 
	integer *nohous, doublereal *b, doublereal *fmat, integer *mdf, 
	doublereal *c__, integer *mdc, integer *m, doublereal *f, doublereal *
	pivot, doublereal *tau, integer *mdg, integer *scale, doublereal *
	diag, integer *inact, integer *q, integer *p4, integer *p2, integer *
	p3, doublereal *dx, doublereal *v1, doublereal *d2, doublereal *d3, 
	integer *rankc2, doublereal *d1nrm, doublereal *dnrm, doublereal *
	b1nrm, doublereal *d__, doublereal *s, doublereal *u, doublereal *
	gmat)
{
    /* System generated locals */
    integer a_dim1, a_offset, fmat_dim1, fmat_offset, c_dim1, c_offset, 
	    gmat_dim1, gmat_offset;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer code;
    extern /* Subroutine */ int cqhp2_(integer *, doublereal *, integer *, 
	    integer *, integer *, integer *, integer *, doublereal *, integer 
	    *, doublereal *, integer *, doublereal *, doublereal *, integer *,
	     integer *, doublereal *), ltoup_(doublereal *, integer *, 
	    integer *, integer *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *), c2toup_(integer *, integer *, 
	    doublereal *, integer *, doublereal *, integer *, integer *, 
	    doublereal *), subdir_(integer *, doublereal *, integer *, 
	    integer *, integer *, doublereal *, integer *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, integer *, doublereal *,
	     integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *, integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     doublereal *, doublereal *, doublereal *, doublereal *);
    static integer kc2, nmp;
    static doublereal tol;


/*     SOLVE FOR DX ONE OF THE COMPOUND SYSTEMS */

/*                                               T */
/*          (L@D0)*DY = B  (1)       (R@D0)*DY = Q2 *B    (2) */
/*         C*DY APPR.=-F               C*DY APPR.=-F */
/*     WHERE */
/*               T                           T */
/*     (L@D0) = P1 *A*Q1*FMAT       (R@D0) = Q2 *(L@D0)*P2 */
/*                T */
/*          B = P1 *(-H) */
/*         DX = Q1*FMAT*DY              DX = Q1*FMAT*P2*DY */
/*     L IS LOWER TRIANGULAR RANKA*RANKA */
/*     R IS UPPER TRIANGULAR RANKA*RANKA */

/*     IF RANKA=T SYSTEM NO. (1) IS SOLVED OTHERWISE SYSTEM (2) */
/*     IS SOLVED */

/*     ON ENTRY@D */

/*     TIME    INTEGER SCALAR = 2 IF MATRIX FMAT IS THE IDENTITY MATRIX */
/*             >2 IF FMAT CONTAINS A PRODUCT OF GIVENS ROTATION MATRICES */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING MATRIX L AND INFO. TO FORM Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     D1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     NOHOUS  INTEGER SCALAR CONTAINING NUMBER OF HOUSEHOLDER */
/*             TRANSFORMATIONS USED TO FORM  (L@D0) */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING  T */
/*                       P1 *(-H) */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             IF TIME=2 FMAT IS UNDEFINED OTHERWISE IT CONTAINS */
/*             A PRODUCT OF GIVENS ROTATION MATRICES */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING THE JACOBIAN OF THE RESIDUALS */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     TAU     REAL SCALAR CONTAING A SMALL VALUE USED TO DETERMINE */
/*             PSEUDO RANK OF MATRIX C2 (SEE BELOW) */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     SCALE   INTEGER SCALAR =0 IF NO ROW SCALING OF MATRIX A HAS */
/*             BEEN DONE. >0 IF SCALING HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING THE DIAGONAL MATRIX IN   T */
/*                                         (L@D0) = P1 *DIAG*A*Q1*FMAT */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR THE INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAINING NO. OF INACTIVE CONSTRAINTS */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING INFO. TO COMPUTE A*DX FOR INACTIVE CONSTRAINTS */

/*     ON RETURN@D */
/*                                   T */
/*     B()     IF T>RANKA THEN  B@D=Q2 *B */
/*                        ELSE UNCHANGED */
/*     C(,)    CONTAINS  (C1 @D U)    WHERE  (C1@DC2)=C*Q1*FMAT*P2 */
/*                       (     0)              T */
/*                                           Q3 *C2*P3 = (U) */
/*                                                       (0) */
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

/*     WORKING AREAS@D */

/*     S(),U() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION N */

/*     INTERNAL VARIABLES */


/*     TRANSFORM L IN (L@D0) TO UPPER TRIANGULAR FORM IF T>RANKA */

/*      write(10,*) 'In GNSRCH: ranka,t= ', ranka,t */
    /* Parameter adjustments */
    --u;
    --s;
    --d3;
    --dx;
    --p3;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;
    --p1;
    --b;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --d__;
    --v1;
    --f;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --diag;
    --inact;
    --p4;
    --p2;
    --d2;

    /* Function Body */
    code = 1;
    if (*t == *ranka) {
	goto L10;
    }
    code = -1;
    ltoup_(&a[a_offset], mda, ranka, t, &b[1], mdg, &p2[1], &gmat[gmat_offset]
	    , &d2[1]);
L10:

/*     DO COORDINATE CHANGES IN MATRIX C BY FORMING */
/*     (C1@DC2)=C*Q1*FMAT*P2  AND STORE IN ARRAY C */

    cqhp2_(time, &c__[c_offset], mdc, m, n, t, ranka, &pivot[1], nohous, &a[
	    a_offset], mda, &d1[1], &fmat[fmat_offset], mdf, &p2[1], &s[1]);
    kc2 = *ranka + 1;
    nmp = *n - *ranka;
    tol = sqrt((doublereal) nmp) * *tau;

/*     TRANSFORM C2 TO UPPER TRIANGULAR FORM BY FORMING */
/*           T */
/*         Q3 *C2*P3 = (U) */
/*                     (0) */

    c2toup_(m, &nmp, &c__[kc2 * c_dim1 + 1], mdc, &tol, &p3[1], rankc2, &d3[1]
	    );

/*     COMPUTE GAUSS-NEWTON SEARCH DIRECTION */

    subdir_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], ranka, ranka, 
	    nohous, &b[1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &
	    f[1], &pivot[1], &gmat[gmat_offset], mdg, &d2[1], &p2[1], &p3[1], 
	    &d3[1], rankc2, &inact[1], q, &p4[1], rankc2, &code, scale, &diag[
	    1], &d__[1], &dx[1], &v1[1], d1nrm, dnrm, b1nrm, &u[1]);
    return 0;
} /* gnsrch_ */

/* MINMAX */
/* Subroutine */ int minmax_(integer *p, integer *t, doublereal *v, integer *
	scale, doublereal *diag, doublereal *sigmin, doublereal *absvmx)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal rowi;
    static integer i__;
    static doublereal absel, sqrel;


/*     ON ENTRY@D */

/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING ESTIMATES OF THE LAGRANGE MULTIPLIERS */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     SCALE   INTEGER SCALAR CONTAINING ZERO IF NO INTERNAL ROW */
/*             SCALING OF THE MATRIX A HAS BEEN DONE. OTHERWISE */
/*             SCALING HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING THE DIAGONAL ELEMENTS OF THE SCALING MATRIX */
/*             IF INTERNAL SCALING HAS BEEN DONE. OTHERWISE IT */
/*             CONTAINS THE LENGHT OF EACH ROW IN THE MATRIX A */

/*     ON RETURN@D */

/*     SIGMIN  REAL SCALAR-CONTAINS THE SMALLEST MULTIPLIER */
/*             CORRESPONDING TO INEQUALITY CONSTRAINTS */
/*     ABSVMX  REAL SCALAR-CONTAINS MAXIMUM(DABS(V(I))  I=1,2,.....,T */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --diag;
    --v;

    /* Function Body */
    sqrel = sqrt(machin_1.drelpr);
    *sigmin = 1e6;
    *absvmx = 0.;
    if (*p == *t) {
	return 0;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	absel = (d__1 = v[i__], abs(d__1));
	if (absel > *absvmx) {
	    *absvmx = absel;
	}
	if (i__ <= *p) {
	    goto L10;
	}
	rowi = diag[i__];
	if (*scale != 0) {
	    rowi = 1. / diag[i__];
	}
	if (-sqrel < v[i__] * rowi) {
	    goto L10;
	}
	if (v[i__] < *sigmin) {
	    *sigmin = v[i__];
	}
L10:
	;
    }
    return 0;
} /* minmax_ */

/* ANALYS */
/* Subroutine */ int analys_(integer *k, logical *restar, integer *code, 
	doublereal *fsum, doublereal *d1nrm, doublereal *dnrm, doublereal *
	c__, integer *mdc, integer *m, integer *n, integer *rankc2, 
	doublereal *d__, doublereal *f, integer *p3, doublereal *d3, integer *
	active, doublereal *v, integer *inact, integer *q, integer *p4, 
	integer *time, doublereal *a, integer *mda, integer *p, integer *t, 
	integer *ranka, doublereal *b1nrm, doublereal *hsum, integer *nohous, 
	doublereal *d1, integer *p1, doublereal *d2, integer *p2, doublereal *
	b, doublereal *h__, integer *l, doublereal *fmat, integer *mdf, 
	doublereal *pivot, doublereal *gmat, integer *mdg, U_fp ffunc, U_fp 
	hfunc, doublereal *x, logical *sec, logical *add, logical *del, 
	integer *scale, doublereal *diag, doublereal *dx, doublereal *dxnorm, 
	doublereal *v1, doublereal *epsrel, integer *error, integer *eval, 
	doublereal *betak, integer *dima, integer *dimc2, doublereal *v2, 
	doublereal *work1)
{
    /* System generated locals */
    integer c_dim1, c_offset, a_dim1, a_offset, fmat_dim1, fmat_offset, 
	    gmat_dim1, gmat_offset;

    /* Local variables */
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    extern /* Subroutine */ int gndchk_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *, logical *, doublereal *, 
	    logical *, integer *, integer *, logical *, integer *, integer *, 
	    integer *, doublereal *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, integer *, doublereal *, doublereal *, 
	    integer *), subdir_(integer *, doublereal *, integer *, integer *,
	     integer *, doublereal *, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, integer *, doublereal *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *, integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     doublereal *, doublereal *, doublereal *, doublereal *), subspc_(
	    logical *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *, integer *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, doublereal *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, doublereal *, doublereal *, integer *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *), newton_(U_fp, 
	    U_fp, doublereal *, integer *, doublereal *, integer *, integer *,
	     integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, integer *, 
	    doublereal *, integer *, integer *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *);
    static integer ind;


/*     CHECK IF THE LATEST STEP WAS SUFFICIENTLY GOOD AND EVENTUALLY */
/*     RECOMPUTE THE SEARCH DIRECTION  BY USING EITHER SUBSPACE */
/*     MINIMIZATION OR THE METHOD OF NEWTON */

/*     ON ENTRY@D */

/*     K       INTEGER SCALAR CONTAINING CURRENT ITERATION NUMBER */
/*     RESTAR  LOGICAL SCALAR = TRUE IF CURRENT STEP IS A RESTART STEP */
/*             =FALSE IF NO RESTART HAS BEEN DONE */
/*     CODE    INTEGER SCALAR CONTAINING A CODE INDICATING HOW */
/*             THE LATEST SEARCH DIRECTION WAS COMPUTED */
/*             = 1 IF GAUSS-NEWTON */
/*             =-1 IF SUBSPACE MINIMIZATION */
/*             = 2 IF NEWTON */
/*     FSUM    REAL SCALAR CONTAINING SQUARED SUM OF THE RESIDUALS */
/*     D1NRM   REAL SCALAR CONTAINING  II D1 II */
/*     DNRM    REAL SCALAR CONTAINING  II D II */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING   (C1 @D U)  WHERE  (C1@DC2)=C*Q1*FMAT*P2 */
/*               T          (     0)     FMAT AND P2 MAY BE THE IDENTITY */
/*             Q3 *C2*P3=(U) */
/*                       (0) */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*     D()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING    T */
/*                         Q3 *(-F-C1*DY1) */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUES OF THE RESIDUALS */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2 (IF IT IS HAS BEEN USED) */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES OF THE CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING ESTIMATES OF LAGRANGE MULTIPLIERS */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAINING NUMBER OF INACTIVE CONSTRAINTS */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING INFO. TO COMPUTE A*DX FOR INACTIVE CONSTRAINTS */
/*     TIME    INTEGER SCALAR CONTAINING =2 IF NO DELETION OF */
/*             CONSTRAINTS HAS BEEN DONE IN CURRENT STEP (I.E. FMAT=I) */
/*             >2 IF DELETIONS HAVE BEEN DONE */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING MATRIX L FROM          T */
/*                                       (L@D0)=P1 *A*Q1      AND INFO. */
/*             TO FORM  Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     RANKA   INTEGER SCALAR  CONTAINING PSEUDO RANK OF MATRIX L */
/*     B1NRM   REAL SCALAR CONTAINING  II B1 II */
/*     HSUM    INTEGER SCALAR CONTAINING SUM OF SQUARED CONSTRAINTS */
/*             IN CURRENT WORKING SET */
/*     NOHOUS  INTEGER SCALAR CONTAINING NUMBER OF HOUSEHOLDER */
/*             TRANSFORMATIONS DONE TO FORM  (L@D0) */
/*     D1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2 (IF IT HAS BEEN USED) */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P2  (IF IT HAS BEEN USED) */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING     T         OR       T   T */
/*                          P1 *(-H)           Q2 *P1 *(-H) */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             CONTAINING A PRODUCT OF GIVENS ROTATION  MATRICES */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINING MATRIX R FROM    T */
/*                                       Q2 *(L@D0) = (R@D0) */
/*             AND INFO. TO FORM Q2 */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     FFUNC   SUBROUTINE NAME USED TO EVALUATE THE RESIDUALS */
/*     HFUNC   SUBROUTINE NAME USED TO EVALUATE THE CONSTRAINTS */
/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE CURRENT POINT */
/*     SEC     LOGICAL SCALAR = TRUE IF THE USER HAS ALLOWED USE OF */
/*             2@DND DERIVATIVES . =FALSE IF NOT */
/*     ADD     LOGICAL SCALAR=TRUE IF AT LEAST ONE CONSTRAINT WAS ADDED */
/*             TO THE WORKING SET IN THE LATEST STEP */
/*             =FALSE IF NO CONSTRAINTS WERE ADDED IN THE LATEST STEP */
/*     DEL     LOGICAL SCALAR = TRUE IF SOME CONSTRAINT WAS DELETED */
/*             FROM THE WORKING SET IN THE CURRENT STEP */
/*     SCALE   INTEGER SCALAR CONTAINING =0 IF NO SCALING OF MATRIX A */
/*             HAS BEEN DONE. >0 IF ROW SCALING OF A HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING THE DIAGONAL SCALING MATRIX */
/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE GAUSS-NEWTON SEARCH DIRECTION FOR CURRENT */
/*             STEP */
/*     DXNORM  REAL SCALAR CONTAINING  II DX II */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+T */
/*             CONTAINING THE VECTOR   (C*DX) */
/*                                     (A*DX) */
/*     EPSREL  REAL SCALAR CONTAINING RELATIVE TERMINATION TOLERANCE */

/*     ON RETURN@D */

/*     VARIABLES FLAGGED WITH * ARE NOT CHANGED IF THE GAUSS-NEWTON */
/*     SEARCH DIRECTION IS ACCEPTED */

/*     CODE    CONTAINS 1,-1 OR 2 TO INDICATE WHICH METHOD WAS USED */
/*             TO COMPUTE THE FINAL SEARCH DIRECTION */
/*  *  D1NRM   CONTAINS  II D1 II   D =(D1) */
/*  *  DNRM    CONTAINS  II D II       (D2) */
/*  *  B1NRM   CONTAINS  II B1 II */
/*  *  DX()    CONTAINS FINAL SEARCH DIRECTION */
/*  *  DXNORM  CONTAINS  II DX II */
/*     ERROR   INTEGER SCALAR-CONTAINS = 0 IF NO TROUBLE AT ALL */
/*             = -4 IF WE ARE NOT ALLOWED TO USE SECOND DERIVATIVES */
/*             = -5 IF NOT POSITIVE DEF. HESSIAN FROM NEWTON */
/*             = -9 IF TOO MANY(>5) NEWTON STEPS HAVE BEEN USED */
/*             = -10 AS USER STOP INDICATOR */
/*     EVAL    INTEGER SCALAR-CONTAINS NUMBER OF FUNCTION EVALUATION */
/*             DONE INSIDE THIS ROUTINE */
/*     BETAK   REAL SCALAR-CONTAINS  II D1 II + II B1 II */
/*     DIMA    INTEGER SCALAR-CONTAINS */
/*             = DIMENSION OF SUBSPACE USED WHEN DY1 IS COMPUTED */
/*               IF SUBSPACE MINIMIZATION IS USED */
/*             =-T IF THE METHOD OF NEWTON IS USED */
/*     DIMC2   INTEGER SCALAR-CONTAINS */
/*             = DIMENSION OF SUBSPACE USED WHEN DY2 IS COMPUTED */
/*               IF SUBSPACE MINIMIZATION IS USED */
/*             =-(N-T) IF THE METHOD OF NEWTON IS USED */

/*     WORKING AREAS@D */

/*     V2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*     WORK1() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */


/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */



/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */


/*     COMPUTE  D1APM1 = II D1 II   WHERE D1 CONSISTS OF THE RKCKM1-1 */
/*     FIRST ELEMENTS OF THE VECTOR D  (SEE ABOVE) */
/*     RKCKM1 = DIMENSION OF SUBSPACE USED TO COMPUTE THE LAST BUT ONE */
/*     SEARCH DIRECTION. */
/*     HOWEVER, IF THE CURRENT WORKING SET HAS BEEN CHANGED SINCE THE */
/*     THE LATEST STEP RKCKM1 MUST BE MODIFIED APPROPRIATELY */

    /* Parameter adjustments */
    --f;
    --d__;
    --dx;
    --x;
    --d3;
    --p3;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --active;
    --v;
    --inact;
    --p4;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;
    --p1;
    --d2;
    --p2;
    --b;
    --h__;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --diag;
    --v1;
    --v2;
    --work1;

    /* Function Body */
    if (*restar) {
	goto L10;
    }
    ind = prec_1.rkckm1 + prec_1.tkm1 - *t;
    --ind;
    prec_1.d1km2 = enlsip_dnrm2_(&ind, &d__[1], &c__1);

/*     SET IND= 1 IF DX ON ENTRY IS ACCEPTED */
/*            =-1 IF SUBSPACE MINIMIZATION SHOULD BE USED */
/*            = 2 IF METHOD OF NEWTON IS USED */

L10:
    gndchk_(b1nrm, d1nrm, dnrm, hsum, k, restar, &prec_1.d1km2, add, m, n, 
	    del, &active[1], p, t, &v[1], &inact[1], q, &h__[1], epsrel, 
	    ranka, scale, &diag[1], betak, &ind);
    *eval = 0;
    *code = ind;
    *error = 0;
    if (ind != 1) {
	goto L30;
    }

/*     THE GAUSS-NEWTON SEARCH DIRECTION IS ACCEPTED */

    *dima = *ranka;
    *dimc2 = *rankc2;
    return 0;
L30:
    if (ind == 2) {
	goto L40;
    }

/*     USE SUBSPACE MINIMIZATION TO RECOMPUTE THE SEARCH DIRECTION */

    subspc_(restar, fsum, &c__[c_offset], mdc, m, n, rankc2, &f[1], &p3[1], &
	    d3[1], &a[a_offset], mda, t, ranka, hsum, &p1[1], &d2[1], &p2[1], 
	    &b[1], &fmat[fmat_offset], mdf, &pivot[1], &gmat[gmat_offset], 
	    mdg, &d__[1], &dx[1], &work1[1], dima, dimc2);
    ++back_1.nrrest;

/*     COMPUTE THE SEARCH DIRECTION BY USING DIMA COLUMNS OF MATRIX R */
/*     AND DIMC2 COLUMNS OF MATRIX U */

    subdir_(time, &a[a_offset], mda, t, n, &d1[1], &p1[1], dima, ranka, 
	    nohous, &b[1], &fmat[fmat_offset], mdf, &c__[c_offset], mdc, m, &
	    f[1], &pivot[1], &gmat[gmat_offset], mdg, &d2[1], &p2[1], &p3[1], 
	    &d3[1], dimc2, &inact[1], q, &p4[1], rankc2, code, scale, &diag[1]
	    , &d__[1], &dx[1], &v1[1], d1nrm, dnrm, b1nrm, &work1[1]);
    if (*dima == *ranka && *dimc2 == *rankc2) {
	*code = 1;
    }
    goto L70;
L40:

/*     USE 2@DND DERIVATIVES IF WE ARE ALLOWED TO */

    if (*sec) {
	goto L50;
    }
    *error = -4;
    goto L70;
L50:
    if (prec_1.kodkm1 != 2) {
	back_1.nrrest = 0;
    }
    ++back_1.nrrest;

/*     COMPUTE THE SEARCH DIRECTION BY USING THE METHOD OF NEWTON */

    newton_((U_fp)ffunc, (U_fp)hfunc, &x[1], n, &c__[c_offset], mdc, m, 
	    rankc2, &f[1], &p3[1], &d3[1], &v[1], &a[a_offset], mda, &active[
	    1], t, ranka, &d1[1], &p1[1], &p2[1], &d2[1], &b[1], &h__[1], l, 
	    mdf, &pivot[1], &gmat[gmat_offset], mdg, &dx[1], eval, error, &
	    fmat[fmat_offset], &d__[1], &v1[1], &v2[1]);
    *dima = -(*t);
    *dimc2 = -(*n) + *t;
    if (back_1.nrrest > 5) {
	*error = -9;
    }
L70:
    *dxnorm = enlsip_dnrm2_(n, &dx[1], &c__1);
    return 0;
} /* analys_ */

/* SUBDIR */
/* Subroutine */ int subdir_(integer *time, doublereal *a, integer *mda, 
	integer *t, integer *n, doublereal *d1, integer *p1, integer *dima, 
	integer *ranka, integer *nohous, doublereal *b, doublereal *fmat, 
	integer *mdf, doublereal *c__, integer *mdc, integer *m, doublereal *
	f, doublereal *pivot, doublereal *gmat, integer *mdg, doublereal *d2, 
	integer *p2, integer *p3, doublereal *d3, integer *dimc2, integer *
	inact, integer *q, integer *p4, integer *rankc2, integer *code, 
	integer *scale, doublereal *diag, doublereal *d__, doublereal *dx, 
	doublereal *v1, doublereal *d1nrm, doublereal *dnrm, doublereal *
	b1nrm, doublereal *work)
{
    /* System generated locals */
    integer a_dim1, a_offset, fmat_dim1, fmat_offset, c_dim1, c_offset, 
	    gmat_dim1, gmat_offset, i__1, i__2;

    /* Local variables */
    static integer irow;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, j, k;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static doublereal aitdx;
    extern /* Subroutine */ int ycomp_(integer *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *);
    static integer i2, i3, no;
    extern /* Subroutine */ int pv_(integer *, integer *, doublereal *, 
	    integer *, integer *);
    static integer noelem;
    extern /* Subroutine */ int hxcomp_(doublereal *, integer *, integer *, 
	    doublereal *, doublereal *), lsolve_(integer *, integer *, 
	    doublereal *, doublereal *), usolve_(integer *, integer *, 
	    doublereal *, doublereal *), adx_(integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, integer *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *), cdx_(doublereal *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *);
    static integer nmt;


/*     COMPUTE A SEARCH DIRECTION (DX) BY SOLVING, FOR DY1, EITHER */
/*     THE SYSTEM L*DY1 = B   (1)   OR THE SYSTEM  R*DY1 = B1  (2) */
/*     USING DIMA ROWS OR COLUMNS OF THE MATRICES L AND R */
/*     RESPECTIVELY */
/*                  L IS RANKA*RANKA LOWER TRIANGULAR */
/*                  R IS RANKA*RANKA UPPER TRIANGULAR */
/*             T                  T */
/*     (L@D0)=P1 *A*Q1*FMAT    B=P1 *(-H) */
/*             T                   T */
/*     (R@D0)=Q2 *(L@D0)*P2     B1=Q2 *B */

/*     AND THE MATRIX A IS THE ORIGINAL JACOBIAN CORRESPONDING */
/*     TO THE CONSTRAINTS IN CURRENT WORKING SET */
/*     THEN THE SYSTEM           T */
/*                      U*DY2 =Q3 *(-F-C1*DY1) */
/*     IS SOLVED FOR DY2 (USING DIMC2 COLUMNS OF MATRIX U) */
/*     WHERE   (C1@DC2) = C*Q1*FMAT*P2 */

/*             (U)     T */
/*             (0) = Q3 *C2*P3 */

/*     AND THE MATRIX C IS THE ORIGINAL JACOBIAN OF THE RESIDUALS. */
/*     FINALY THE SEARCH DIRECTION IS COMPUTED BY FORMING */
/*     DX @D= Q1*FMAT*P2*(DY1) */
/*                      (P3*DY2) */

/*     ON ENTRY@D */

/*     TIME    INTEGER SCALAR CONTAINING =2 IF DELETION FROM CURRENT */
/*             WORKING SET HAS NOT BEEN DONE (I.E.FMAT=I) */
/*             >2 IF AT LEAST ONE CONSTRAINT HAS BEEN DELETED */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING MATRIX L AND INFO. TO FORM Q1 (SEE ABOVE) */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     D1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     DIMA    INTEGER SCALAR CONTAINING NUMBER OF COLUMNS OF MATRIX */
/*             R THAT SHOULD BE USED WHEN R*DY1=B1 IS SOLVED */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     NOHOUS  INTEGER SCALAR CONTAINING NUMBER OF HOUSEHOLDER */
/*             TRANSFORMATIONS USED TO FORM L IN  (L@D0) */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING   T        OR    T   T */
/*                        P1 *(-H)       Q2 *P1 *(-H) */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             CONTAINING A PRODUCT OF GIVENS ROTATION MATRICES */
/*             IF TIME>2. FMAT IS UNDEFINED IF TIME=2 */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING   (C1@DU)    (SEE ABOVE)  AND INFO. TO FORM Q3 */
/*                          (   0) */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINING MATRIX R AND INFO. TO FORM Q2 (SEE ABOVE) */
/*             IF CODE<>1. GMAT IS UNDEFINED IF CODE=1 */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2 (IF IT IS USED) */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P2 (IF IT IS USED) */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     D3()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKC2 */
/*             CONTAINING INFO. TO FORM Q3 */
/*     DIMC2   INTEGER SCALAR CONTAINING NUMBER OF COLUMNS OF MATRIX */
/*             U THAT SHOULD BE USED TO SOLVE  U*DY2 = D */
/*             WHERE D=(D1)    T */
/*                     (D2)= Q3 *(-F-C1*DY1) */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR THE INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAINING NO. INACTIVE CONSTRAINTS */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING INFO. TO COMPUTE A*DX FOR INACTIVE CONSTRAINTS */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*     CODE    INTEGER SCALAR CONTAINING */
/*             = 1 IF  L*DY1 = B   SHOULD BE SOLVED FOR DY1 */
/*             =-1 IF  R*DY1 = B1  SHOULD BE SOLVED FOR DY1 */
/*     SCALE   INTEGER SCALAR =0 IF NO ROW SCALING OF MATRIX A HAS */
/*             BEEN DONE. >1 IF SCALING OF A HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING THE SCALING MATRIX IF IT HAS BEEN USED */

/*     ON RETURN@D */

/*     D()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINS      T */
/*                         Q3 *(-F-C1*DY1) */
/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINS THE COMPUTED SEARCH DIRECTION */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+T */
/*             CONTAINS THE COMPOUND VECTOR   (C*DX) */
/*                                            (A*DX) */
/*     D1NRM   REAL SCALAR-CONTAINS  II D1 II   (SEE ABOVE) */
/*     DNRM    REAL SCALAR-CONTAINS  II D II */
/*     B1NRM   REAL SCALAR-CONTAINS  II B1 II WHERE B1 CONTAINS */
/*             THE FIRST DIMA ELEMENTS OF B OR B1  (SEE ABOVE) */

/*     WORKING AREAS@D */

/*     WORK()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */

/*     INTERNAL VARIABLES */

/*      write(10,*) 'In SUBDIR: code= ',code */
    /* Parameter adjustments */
    --work;
    --dx;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;
    --p1;
    --b;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --d__;
    --f;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --d2;
    --p2;
    --p3;
    --d3;
    --inact;
    --p4;
    --diag;
    --v1;

    /* Function Body */
    *b1nrm = 0.;
    if (*t <= 0) {
	goto L20;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dx[i__] = b[i__];
/* L10: */
    }
    *b1nrm = enlsip_dnrm2_(dima, &dx[1], &c__1);
/*      write(10,*) 'B1NRM= ',b1nrm */

/*     COMPUTE THE VECTOR AHAT*DX */

    adx_(code, &gmat[gmat_offset], mdg, ranka, dima, &p1[1], &d2[1], &dx[1], 
	    t, nohous, scale, &diag[1], &v1[*m + 1], &work[1]);

/*     SOLVE FOR DY1 ONE OF THE STYSTEMS */
/*     L*DY1 = B       R*DY1 = B1 */

    if (*code == 1) {
	lsolve_(mda, t, &a[a_offset], &dx[1]);
    }
    if (*code != 1) {
	usolve_(mdg, dima, &gmat[gmat_offset], &dx[1]);
    }
L20:
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	d__[i__] = -f[i__];
/* L30: */
    }

/*     COMPUTE D@D=D-C1*DY1 */

    ycomp_(dima, m, &d__[1], &c__[c_offset], mdc, &dx[1]);
    *dnrm = enlsip_dnrm2_(m, &d__[1], &c__1);
/*      write(10,*) 'DNRM (D-C1*DT1)= ', dnrm */
/*                    T */
/*     COMPUTE   D@D=Q3 *D */

/*      write(10,*) 'RANKC2= ',rankc2 */
    if (*rankc2 <= 0) {
	goto L50;
    }
    i__1 = *rankc2;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *ranka + i__;
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, m, &c__[k * c_dim1 + 1], &c__1, &d3[i__], 
		&d__[1], &c__1, m, &c__1, &c__[i__ + k * c_dim1]);
/* L40: */
    }
L50:
    *d1nrm = enlsip_dnrm2_(dimc2, &d__[1], &c__1);

/*     COMPUTE THE VECTOR C*DX */

    k = *ranka + 1;
    nmt = *n - *ranka;
    cdx_(&c__[c_offset], mdc, m, dima, &dx[1], &c__[k * c_dim1 + 1], rankc2, 
	    dimc2, &d3[1], &d__[1], &v1[1]);
    if (nmt <= 0) {
	goto L80;
    }
    i__1 = nmt;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *ranka + i__;
	dx[j] = d__[i__];
/* L60: */
    }

/*     SOLVE FOR DY2 THE SYSTEM  U*DY2 = D1 */

    usolve_(mdc, dimc2, &c__[k * c_dim1 + 1], &dx[k]);

/*     DO BACK TRANSFORMATIONS TO FORM */
/*     DX@D= Q1*FMAT*P2*(DY1) */
/*                     (P3*DY2) */

    if (*dimc2 == nmt) {
	goto L80;
    }
    k = *dimc2 + 1;
    i__1 = nmt;
    for (i__ = k; i__ <= i__1; ++i__) {
	j = *ranka + i__;
	dx[j] = 0.;
/* L70: */
    }
L80:

/*     COMPUTE  DY2 @D= P3*DY2 */

    pv_(&p3[1], &nmt, &dx[*ranka + 1], n, &c__1);
    if (*code == 1) {
	goto L100;
    }

/*     COMPUTE DX @D= P2*DX */

    pv_(&p2[1], ranka, &dx[1], n, &c__1);
L100:
    if (*time <= 2) {
	goto L110;
    }
    if (*time == 3 && *t == 0) {
	goto L110;
    }

/*     COMPUTE DX @D= FMAT*DX */

    hxcomp_(&fmat[fmat_offset], mdf, n, &dx[1], &work[1]);
L110:

/*     COMPUTE A*DX FOR THE CONSTRAINT WHICH HAS BEEN DELETED AT */
/*     THE CURRENT POINT */

    no = *nohous - *t;
    if (no <= 0) {
	goto L140;
    }
    irow = *t + 1;
    i2 = inact[*q];
    noelem = p4[i2];
    aitdx = 0.;
    i__1 = noelem;
    for (j = 1; j <= i__1; ++j) {
	aitdx += a[irow + j * a_dim1] * dx[j];
/* L120: */
    }
    i3 = *m + *q + *t;
    v1[i3] = aitdx;
    if (*scale != 0) {
	v1[i3] /= diag[irow];
    }
L140:
    if (*nohous == 0) {
	return 0;
    }

/*     COMPUTE DX @D= Q1*DX */

    i__1 = *nohous;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *nohous - i__ + 1;
	i__2 = k + 1;
	h12per_(&c__2, &k, &i__2, n, &a[k + a_dim1], mda, &d1[k], &dx[1], &
		c__1, mda, &c__1, &pivot[k]);
/* L160: */
    }
    return 0;
} /* subdir_ */

/* UPBND */
/* Subroutine */ int upbnd_(doublereal *a, integer *mda, integer *q, integer *
	nh, integer *t, integer *n, doublereal *h__, integer *inact, integer *
	p4, doublereal *v1, integer *m, doublereal *dx, doublereal *alfupp, 
	integer *ind)
{
    /* Initialized data */

    static doublereal zero = 0.;
    static doublereal big = 1e6;

    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j, k, l;
    static doublereal aitdx;
    static integer ip, ir;
    static doublereal alf;
    static integer mpt;


/*     DETERMINE THE UPPER BOUND OF THE STEPLENGTH */
/*     I.E.  THE SMALLEST OF   -H (X)/A (X)*DX */
/*                               I     I */
/*     S.T. A (X)*DX < 0 */
/*           I */

/*     WHERE I BELONGS TO THE INACTIVE SET */
/*     IF A (X)*DX < 0 FOR NO INDEX I THEN ALFUPP IS SET TO A LARGE VALUE */
/*         I */

/*     ON ENTRY@D */

/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING THE Q GRADIENTS CORRESPONDING TO THE INACTIVE */
/*             CONSTRAINTS */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     Q       INTEGER SCALAR CONTAINING NO. INACTIVE CONSTRAINTS */
/*     NH      INTEGER SCALAR CONTAINING NUMBER OF HOUSEHOLDER */
/*             TRANSFORMATIONS DONE TO FORM      T */
/*                                      (L@D0) =P1 *A*Q1 */
/*     T       INTEGER SCALAR CONTAINING NO. CONSTRAINTS IN WORKING SET */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUES OF THE CONSTRAINTS */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR THE INACTIVE CONSTRAINTS */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             REPRESENTING THE REORDERING OF CONSTRAINT GRADIENTS */
/*             DUE TO THE WORKING SET SUCH THAT P4(I) POINTS TO THE */
/*             ROW IN ARRAY A WHERE GRADENT NUMBER I IS MOVED */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+L */
/*             CONTAINING THE COMPOUND MATRIX  (C*DX   ) */
/*             AS M+T FIRST ELEMENTS           (AHAT*DX) */
/*     M       INTEGER SCALAR CONTAINING NO. OF RESIDUALS */
/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE SEARCH DIRECTION */

/*     ON RETURN@D */

/*     V1()    NOW CONTAINS THE AUGMENTED COMPOUND MATRIX (C*DX   ) */
/*                                                        (A*DX   ) */
/*     ALFUPP  REAL SCALAR-CONTAINS THE UPPER BOUND (SEE ABOVE) */
/*     IND     INTEGER SCALAR  = 0 IF NO CONSTRAINT DEFINES ALFUPP */
/*             = INDEX FOR THE CONSTRAINT THAT DEFINES THE UPPER BOUND */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --dx;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --h__;
    --inact;
    --p4;
    --v1;

    /* Function Body */
    *alfupp = big;
    *ind = 0;
    mpt = *m + *t;
    l = *q - *nh + *t;
    if (*q <= 0) {
	goto L50;
    }
    i__1 = *q;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = inact[i__];
	ip = mpt + i__;
	ir = p4[k];
	if (i__ > l) {
	    goto L20;
	}
	aitdx = 0.;
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    aitdx += a[ir + j * a_dim1] * dx[j];
/* L10: */
	}
	v1[ip] = aitdx;
	goto L30;
L20:
	aitdx = v1[ip];
L30:
	if (aitdx >= zero) {
	    goto L40;
	}
	if (h__[k] <= zero) {
	    goto L40;
	}
	alf = -h__[k] / aitdx;
	if (alf > *alfupp) {
	    goto L40;
	}
	*alfupp = alf;
	*ind = k;
L40:
	;
    }
L50:
    *alfupp = min(*alfupp,3.);
    return 0;
} /* upbnd_ */

/* EVADD */
/* Subroutine */ int evadd_(doublereal *h__, integer *active, integer *t, 
	integer *bnd, integer *p, integer *inact, integer *q, integer *ind, 
	integer *itno, logical *add)
{
    /* Initialized data */

    static doublereal delta = .1;

    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer kind;
    static doublereal hmax;
    static integer i__, j, k;
    extern /* Subroutine */ int addit_(integer *, integer *, integer *, 
	    integer *, integer *);
    static integer jj, kk;
    extern /* Subroutine */ int delete_(integer *, integer *, integer *, 
	    integer *, integer *);
    static doublereal eps;


/*     MOVE VIOLATED CONSTRAINTS TO THE WORKING SET */

/*     ON ENTRY@D */

/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR THE CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     BND     INTEGER SCALAR CONTAINING MIN(L,N) */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR THE INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAING NUMBER OF INACTIVE CONSTRAINTS */
/*     IND     INTEGER SCALAR CONTAINING = 0 IF THE LATEST STEPLENGTH */
/*             IS LESS THAN THE UPPER BOUND */
/*             = INDEX FOR THE CONSTRAINT THAT DEFINES THE UPPER BOUND */
/*     ITNO    INTEGER SCALAR CONTAINING THE ITERATION NUMBER */

/*     ON RETURN@D */

/*     ADD     LOGICAL SCALAR = FALSE IF NO CONSTRAINT IS ADDED */
/*             = TRUE IF ANY CONSTRAINT IS ADDED TO THE WORKING SET */

/*     IF ADD=TRUE ON RETURN THE ARRAYS ACTIVE AND INACT ARE UPDATED */
/*     AS WELL AS THE SCALARS T AND Q */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --inact;
    --active;
    --h__;

    /* Function Body */
    eps = sqrt(machin_1.drelpr);
    i__ = 1;
L10:
    if (i__ > *q) {
	return 0;
    }
    k = inact[i__];
    if (h__[k] < eps) {
	goto L15;
    }
    if (! (k == *ind && h__[k] < delta)) {
	goto L20;
    }
L15:
    kind = 0;
    if (*t >= *bnd) {
	kk = 0;
	kind = 1;
	hmax = 0.;
	i__1 = *t;
	for (j = 1; j <= i__1; ++j) {
	    jj = active[j];
/*     write(10,*) 'h(jj)=' ,h(jj),' jj= ',jj */
	    if (h__[jj] > hmax) {
		hmax = h__[jj];
		kk = j;
	    }
/* L17: */
	}
	if (kk == 0) {
	    hmax = 100.;
	    i__1 = *t;
	    for (j = 1; j <= i__1; ++j) {
		jj = active[j];
		if ((d__1 = h__[jj], abs(d__1)) < hmax) {
		    hmax = (d__1 = h__[jj], abs(d__1));
		    kk = j;
		}
/* L18: */
	    }
	}
/*     write(10,*) 'Deleted constraint h(kk)=  ',kk */
	delete_(&active[1], &inact[1], q, t, &kk);
    }
/*     write(10,*) 'Added constraint: ',k */
    addit_(&active[1], &inact[1], t, q, &i__);
    j = *bnd + k - *p;
    if (-1 == active[j]) {
	active[j] = *itno;
    }
    if (active[j] == 0) {
	active[j] = 1;
    }
    *add = TRUE_;
    if (kind == 1) {
	i__ = *q + 1;
    }
    goto L10;
L20:
    ++i__;
    goto L10;
} /* evadd_ */

/* SCALV */
/* Subroutine */ int scalv_(doublereal *v, doublereal *factor, integer *n)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     FORM THE NEW CONTENTS OF THE VECTOR V AS */
/*            V(I)=1/FACTOR*V(I)  I=1,2,......,N */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --v;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v[i__] /= *factor;
/* L10: */
    }
    return 0;
} /* scalv_ */

/* EVREST */
/* Subroutine */ int evrest_(doublereal *x, doublereal *u, integer *n, 
	integer *m, integer *k, S_fp ffunc, integer *funcev, doublereal *f, 
	doublereal *d1nrm, doublereal *dnrm, doublereal *fsum, integer *dimc2,
	 integer *code, doublereal *dxnorm, doublereal *betak, doublereal *
	alpha, doublereal *alflow, integer *active, doublereal *h__, integer *
	l, integer *t, S_fp hfunc, doublereal *b1nrm, doublereal *hsum, 
	integer *dima, integer *error, logical *restar)
{
    /* System generated locals */
    doublereal d__1;

    /* Local variables */
    static integer ctrl;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *), sacth_(
	    doublereal *, integer *, integer *);
    static doublereal dummy;
    extern /* Subroutine */ int savexk_(doublereal *, integer *, doublereal *)
	    ;


/*     IF CURRENT STEP IS A FAILURE AND SUBSPACE MINIMIZATION WAS */
/*     USED WE RESTART AT THE PREVIOUS POINT. */
/*     We also restart at the previous point if the Newton method fails. */

/*     ON ENTRY@D */

/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE LATEST POINT */
/*     U()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE PREVIOUS POINT */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     K       INTEGER SCALAR CONTAINING ITERATION NUMBER */
/*     FFUNC   SUBROUTINE NAME-USED TO EVALUATE THE RESIDUALS */
/*     FUNCEV  INTEGER SCALAR CONTAINING TOTAL NUMBER OF FUNCTION */
/*             EVALUATIONS DONE SO FAR */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS AT X */
/*     D1NRM   REAL SCALAR CONTAINING  II D1 II */
/*     DNRM    REAL SCALAR CONTAINING  II D II */
/*     FSUM    REAL SCALAR CONTAINING SUM OF SQUARED RESIDUALS AT U */
/*     DIMC2   INTEGER SCALAR CONTAINING NUMBER OF COLUMNS OF MATRIX */
/*             U USED WHEN   U*DY2 = D   IS SOLVED */
/*     CODE    INTEGER SCALAR CONTAINING */
/*             = 1 IF GAUSS-NEWTON SEARCH DIRECTION */
/*             =-1 IF SUBSPACE MINIMIZATION SEARCH DIRECTION */
/*             = 2 IF NEWTON SEARCH DIRECTION */
/*     DXNORM  REAL SCALAR CONTAINING NORM OF SEARCH DIRECTION */
/*     BETAK   REAL SCALAR CONTAINING  II D1 II + II B1 II */
/*     ALPHA   REAL SCALAR CONTAINING THE LATEST STEPLENGTH */
/*     ALFLOW  REAL SCALAR CONTAINING LOWER BOUND OF STEPLENGTH */
/*             USED IN LATEST STEP */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR CONSTRAINTS IN CURRENT WORKING SET */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT X */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     HFUNC   SUBROUTINE NAME-USED TO EVALUATE THE CONSTRAINTS */
/*     B1NRM   REAL SCALAR CONTAINING  II B1 II */
/*     HSUM    REAL SCALAR CONTAINING SUM OF SQUARED CONSTRAINTS AT U */
/*     DIMA    INTEGER SCALAR CONTAINING NUMBER OF COLUMNS OF MATRIX R */
/*             USED WHEN   R*DY1 = B1  IS SOLVED */
/*     ERROR   INTEGER SCALAR CONTAINING */
/*             = 0 IF EVERY THING SEEMS OK */
/*             =-1 IF DX WAS NOT A DESCENT DIRECTION */
/*             =-2 IF ALPHA*II DX II IS SMALL */
/*             =-3 IF NOT POS. DEF. HESSIAN MATRIX IN NEWTON */
/*             =-4 IF NOT ALLOWED TO USE 2@DND DERIVATIVES */

/*     ON RETURN@D */

/*     RESTAR  = TRUE IF A RESTART IS SUGGESTED */
/*             = FALSE IF NO RESTART IS SUGGESTED */
/*     FSUM    CONTAINS SUM OF SQUARED RESIDUALS AT THE NEW POINT */
/*     HSUM    CONTAINS SUM OF SQUARED CONSTRAINTS IN CURRENT WORKING */
/*             SET AT THE NEW POINT */
/*     ERROR   < -10 TO INDICATE A POSSIBLE USER STOP */
/*             =-5 if Newton has failed */

/*     IF RESTAR=FALSE ON RETURN MANY COMMON VARIABLES ARE UPDATED */
/*     TO REPRESENT THE CORRECT HISTORY */
/*     IF RESTAR=TRUE ON RETURN THE FOLLOWING VARIABLES HAVE BEEN */
/*     CHANGED */

/*     X()     CONTAINS THE PREVIOUS POINT U */
/*     FUNCEV  INCREACED BY 1 */
/*     F()     CONTAINS THE VALUE OF THE RESIDUALS AT PREVIOUS POINT U */
/*     H()     CONTAINS THE VALUE OF THE CONSTRAINTS AT PREVIOUS POINT U */


/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     common for fixing Newton failures */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --u;
    --x;
    --f;
    --active;
    --h__;

    /* Function Body */
    *restar = FALSE_;

/*     Check if the Newton metod failed */

/*      succes=fsum+hsum-(dnrm2(m,f,1)**2+sacth(h,active,t)) */
/*      if((abs(code).eq.2).and.(succes.le.0.0d0)) then */
/*       write(10,*) 'In EVREST: succes= ',succes */
/*       ifree=5 */
/*       nrrest=0 */
/*       goto 55 */
/*      endif */

/*     CHECK IF SOMETHING HAS GONE WRONG IN THE LINESEARCH */
/*     AND POSSIBLY A RESTART IS SUGGESTED */

    if (back_1.lattry == 0 && back_1.bestpg > 0.) {
	goto L10;
    }
    if (-1 == *error || *error <= -3) {
	goto L10;
    }
/*      IF((-2.EQ.ERROR).AND.(ALPHA.LE.ALFLOW)) RESTAR=.TRUE. */
    if (*alpha <= *alflow) {
	*restar = TRUE_;
    }
    if (*restar) {
	goto L50;
    }
L10:

/*     NO RESTART UPDATE HISTORY VARIABLES */

    ++(*k);
    if (*code != 2) {
	back_1.nrrest = 0;
    }
    back_1.lattry = max(*dimc2,*dima);
    prec_1.betkm2 = prec_1.betkm1;
    prec_1.d1km2 = prec_1.d1km1;
    prec_1.dkm2 = prec_1.dkm1;
    prec_1.fsqkm2 = prec_1.fsqkm1;
    prec_1.b1km2 = prec_1.b1km1;
    prec_1.hsqkm2 = prec_1.hsqkm1;
    prec_1.dxnkm2 = prec_1.dxnkm1;
    prec_1.alfkm2 = prec_1.alfkm1;
    prec_1.rkakm2 = prec_1.rkakm1;
    prec_1.rkckm2 = prec_1.rkckm1;
    prec_1.tkm2 = prec_1.tkm1;
    prec_1.kodkm2 = prec_1.kodkm1;
    prec_1.betkm1 = *betak;
    prec_1.d1km1 = *d1nrm;
    prec_1.dkm1 = *dnrm;
    prec_1.fsqkm1 = *fsum;
    prec_1.b1km1 = *b1nrm;
    prec_1.hsqkm1 = *hsum;
    prec_1.dxnkm1 = *dxnorm;
    prec_1.alfkm1 = *alpha;
    prec_1.rkakm1 = *dima;
    prec_1.rkckm1 = *dimc2;
    prec_1.tkm1 = *t;
    prec_1.kodkm1 = *code;
    if (-1 == *error) {
	goto L55;
    }
    if (*error < 0) {
	return 0;
    }
    *hsum = sacth_(&h__[1], &active[1], t);
    prec_1.hsqkm2 = *hsum;
/* Computing 2nd power */
    d__1 = enlsip_dnrm2_(m, &f[1], &c__1);
    *fsum = d__1 * d__1;
    return 0;
L50:

/*     COUNT NUMBER OF RESTARTS AT CURRENT POINT AND SAVE LATEST */
/*     DIMENSIONS USED TO COMPUTE THE SEARCH DIRECTION */

    ++back_1.nrrest;
    prec_1.rkakm1 = *dima;
    prec_1.rkckm1 = *dimc2;
    if (*k != 0) {
	goto L55;
    }
    prec_1.d1km1 = *d1nrm;
    prec_1.b1km1 = *b1nrm;
L55:

/*     RESTART FROM PREVIOUS POINT */

    savexk_(&u[1], n, &x[1]);
    if (abs(*code) == 2) {
	*error = -5;
    }
    ctrl = -1;
    (*ffunc)(&x[1], n, &f[1], m, &ctrl, &dummy, &c__1);
    if (ctrl < -10) {
	*error = ctrl;
    }
    ctrl = -1;
    (*hfunc)(&x[1], n, &h__[1], l, &ctrl, &dummy, &c__1);
    if (ctrl < -10) {
	*error = ctrl;
    }
    *alpha = prec_1.alfkm1;
    ++(*funcev);
    return 0;
} /* evrest_ */

/* OUTPUT */
/* Subroutine */ int output_(integer *iprint, integer *k, integer *unit, 
	doublereal *gres, doublereal *w, integer *active, doublereal *speed)
{
#if 0    
	/* Format strings */
    static char fmt_1000[] = "(////10x,\002COLLECTED INFORMATION FOR ITERATI"
	    "ON STEPS\002//\002   K \002,\002  FSUM(K)  \002,\002  HSUM(K) "
	    " \002,\002  LAGRES  \002,\002  DXNORM  \002,\002KODA \002,\002KO"
	    "DC\002,\002   ALPHA  \002,\002CONV.SPEED\002,\002  MAX(W)  \002"
	    ",\002 PREDICTED\002,\002 REDUCTION\002,\002 WORKING SET\002)";
    static char fmt_1010[] = "(i4,2e11.3,2e10.3,i4,i5,5e10.3,1x,6i3/(105x,6i"
	    "3))";

    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    integer s_wsfe(cilist *), e_wsfe(void), do_fio(integer *, char *, ftnlen);

    /* Local variables */
    static integer itno;
    static doublereal wmax;
    static integer i__, j;

    /* Fortran I/O blocks */
    static cilist io___60 = { 0, 0, 0, fmt_1000, 0 };
    static cilist io___61 = { 0, 0, 0, fmt_1010, 0 };
    static cilist io___62 = { 0, 0, 0, fmt_1010, 0 };



/*     IF IPRINT>0 */
/*       THEN WRITE SOME INFO. EVERY IPRINT STEP ON UNIT */
/*       ELSE NO PRINTING IS DONE */

/*     ON ENTRY@D */

/*     IPRINT  INTEGER SCALAR CONTAINING PRINTING STEPSIZE */
/*     K       INTEGER SCALAR CONTAINING ITERATION NUMBER */
/*     UNIT    INTEGER SCALAR CONTAINING LOGICAL UNIT NUMBER FOR */
/*             OUTPUT FILE */
/*                                         T */
/*     GRES    REAL SCALAR CONTAINING  II A *V-G II */
/*     W()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING PENALTY WEIGHTS FOR CONSTRAINTS */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR CONSTRAINTS IN CURRENT WORKING SET */

/*     ON RETURN@D */

/*     SPEED   REAL SCALAR-CONTAINS AN ESTIMATE OF THE CONVERGENCE */
/*             FACTOR */


/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --active;
    --w;

    /* Function Body */
    if (*iprint <= 0) {
	return 0;
    }
    itno = *k - 1;
    if (itno / *iprint * *iprint != itno) {
	return 0;
    }
    *speed = 0.;
    if (itno != 0) {
	*speed = prec_1.betkm1 / prec_1.betkm2;
    }
    wmax = 0.;
    if (prec_1.tkm1 <= 0) {
	goto L20;
    }
    i__1 = prec_1.tkm1;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = active[i__];
	if (w[j] > wmax) {
	    wmax = w[j];
	}
/* L10: */
    }
L20:
    if (itno == 0) {
	io___60.ciunit = *unit;
	s_wsfe(&io___60);
	e_wsfe();
    }
    if (prec_1.tkm1 > 0) {
	io___61.ciunit = *unit;
	s_wsfe(&io___61);
	do_fio(&c__1, (char *)&itno, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.fsqkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.hsqkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&(*gres), (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.dxnkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.rkakm1, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.rkckm1, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.alfkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&(*speed), (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&wmax, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.prelin, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.pgress, (ftnlen)sizeof(doublereal));
	i__1 = prec_1.tkm1;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    do_fio(&c__1, (char *)&active[i__], (ftnlen)sizeof(integer));
	}
	e_wsfe();
    }
    if (prec_1.tkm1 <= 0) {
	io___62.ciunit = *unit;
	s_wsfe(&io___62);
	do_fio(&c__1, (char *)&itno, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.fsqkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.hsqkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&(*gres), (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.dxnkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.rkakm1, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.rkckm1, (ftnlen)sizeof(integer));
	do_fio(&c__1, (char *)&prec_1.alfkm1, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&(*speed), (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&wmax, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.prelin, (ftnlen)sizeof(doublereal));
	do_fio(&c__1, (char *)&prec_1.pgress, (ftnlen)sizeof(doublereal));
	e_wsfe();
    }
#endif	
    return 0;
} /* output_ */

/* ADX */
/* Subroutine */ int adx_(integer *code, doublereal *gmat, integer *mdg, 
	integer *ranka, integer *dima, integer *p1, doublereal *d2, 
	doublereal *b, integer *t, integer *nohous, integer *scale, 
	doublereal *diag, doublereal *hprim, doublereal *w)
{
    /* System generated locals */
    integer gmat_dim1, gmat_offset, i__1, i__2;

    /* Local variables */
    static integer i__, k;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *), pv_(integer *, 
	    integer *, doublereal *, integer *, integer *), pspecf_(integer *,
	     integer *, doublereal *, doublereal *);

/*                                 -1 */
/*     COMPUTE HPRIM@D= AHAT*DX=DIAG  *P1*Q2*(B1) */
/*                                          ( 0) */

/*     IF CODE=1 THEN      -1 */
/*             AHAT*DX=DIAG  *P1*B */
/*               ELSE      -1 */
/*             AHAT*DX=DIAG  *P1*Q2*(B1)    WHERE B1 IS THE FIRST DIMA */
/*                                  ( 0)    ELEMENTS OF B */

/*     ON ENTRY@D */

/*     CODE    INTEGER SCALAR CONTAINING */
/*             = 1 IF GAUSS-NEWTON SEARCH DIRECTION */
/*             =-1 IF SUBSPACE MINIMIZATION */
/*             = 2 IF NEWTON SEARCH DIRECTION */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINING INFO. TO FORM Q2 (IF IT IS USED) */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     DIMA    INTEGER SCALAR CONTAINING NUMBER OF ELEMENTS IN B THAT */
/*             SHOULD BE CONSIDDERED AS NON ZERO */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2  (IF IT IS USED) */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING    T   T */
/*                         Q2 *P1 *(-H) */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     NOHOUS  INTEGER SCALAR CONTAINING NO. OF HOUSEHOLDER */
/*             TRANSFORMATIONS DONE TO FORM (L@D0) */
/*     SCALE   INTEGER SCALAR CONTAINING =0 IF NO ROW SCALING OF */
/*             MATRIX A HAS BEEN DONE. >0 IF SCALING HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING THE DIAGONAL SCALING MATRIX (IF IT IS USED) */

/*     ON RETURN@D */

/*     HPRIM() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS AHAT*DX COMPUTED IN ONE OF THE TWO WAYS */
/*             INDICATED ABOVE */

/*     WORKING AREAS@D */

/*     W()     SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --p1;
    --d2;
    --b;
    --diag;
    --hprim;
    --w;

    /* Function Body */
    if (*dima == *t) {
	goto L20;
    }
    k = *dima + 1;
    i__1 = *t;
    for (i__ = k; i__ <= i__1; ++i__) {
	b[i__] = 0.;
/* L10: */
    }
L20:
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	hprim[i__] = b[i__];
/* L30: */
    }
    if (*code == 1) {
	goto L50;
    }

/*     COMPUTE  HPRIM@D=Q2*(B1) */
/*                        ( 0) */

    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *ranka + 1 - i__;
	i__2 = k + 1;
	h12per_(&c__2, &k, &i__2, t, &gmat[k * gmat_dim1 + 1], &c__1, &d2[k], 
		&hprim[1], &c__1, t, &c__1, &gmat[k + k * gmat_dim1]);
/* L40: */
    }
L50:

/*     COMPUTE  HPRIM@D=P1*HPRIM */

    if (*t < *nohous) {
	goto L60;
    }
    pv_(&p1[1], t, &hprim[1], t, &c__1);
    goto L80;
L60:
    pspecf_(t, &p1[1], &hprim[1], &w[1]);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	hprim[i__] = w[i__];
/* L70: */
    }
L80:
    if (*scale <= 0) {
	return 0;
    }
/*                         -1 */
/*     COMPUTE  HPRIM@D=DIAG  *HPRIM */

    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	hprim[i__] /= diag[i__];
/* L90: */
    }
    return 0;
} /* adx_ */

/* LTOUP */
/* Subroutine */ int ltoup_(doublereal *a, integer *mda, integer *ranka, 
	integer *t, doublereal *b, integer *mdg, integer *p2, doublereal *
	gmat, doublereal *d2)
{
    /* System generated locals */
    integer a_dim1, a_offset, gmat_dim1, gmat_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer cmax, i__, j;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static doublereal collng;
    extern /* Subroutine */ int colmax_(integer *, integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, doublereal *), 
	    permin_(integer *, integer *), prmcol_(doublereal *, integer *, 
	    integer *, integer *, integer *);


/*     TRANSFORM  (L@D0) TO UPPER TRIANGULAR FORM  (R@D0) */
/*     BY FORMING           T */
/*               (R@D0) =  Q2 *(L@D0)*P2 */
/*     LATER ON THE SYSTEM                 T */
/*     (L@D0)*X=B   WILL BE SOLVED SO  B@D=Q2 *B */

/*     ON ENTRY@D */

/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING MATRIX L AND INFO. TO FORM MATRIX Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     T       INTEGER SCALAR CONTAINING NO. OF PARAMETERS */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING   T */
/*                        P1 *(-H) */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */

/*     ON RETURN@D */
/*                        T */
/*     B()     CONTAINS Q2 *B */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINS A REPRESENTATION OF PERMUTATION MATRIX P2 */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINS THE MATRIX R AND INFO. TO FORM Q2 */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINS INFO. TO FORM MATRIX Q2 */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --p2;
    --d2;

    /* Function Body */
    permin_(&p2[1], ranka);

/*     MOVE MATRIX L TO ARRAY GMAT */

    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = *ranka;
	for (j = 1; j <= i__2; ++j) {
	    if (i__ < j) {
		gmat[i__ + j * gmat_dim1] = 0.;
	    }
	    if (i__ >= j) {
		gmat[i__ + j * gmat_dim1] = a[i__ + j * a_dim1];
	    }
/* L10: */
	}
/* L20: */
    }

/*     TRANSFORM L TO UPPER TRIANGULAR FORM */

    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {

/*     FIND LONGEST COLUMN AND PERMUTE IF NECESSARY */

	colmax_(t, ranka, &gmat[gmat_offset], mdg, &i__, &i__, &cmax, &collng)
		;
	p2[i__] = cmax;
	prmcol_(&gmat[gmat_offset], mdg, t, &i__, &cmax);

/*     CONSTRUCT A REFLECTION AND APPLY FROM THE LEFT ON MATRIX L */
/*     AND FROM THE LEFT ON VECTOR B */

	i__2 = i__ + 1;
	i__3 = *ranka - i__;
	h12per_(&c__1, &i__, &i__2, t, &gmat[i__ * gmat_dim1 + 1], &c__1, &d2[
		i__], &gmat[(i__ + 1) * gmat_dim1 + 1], &c__1, mdg, &i__3, &
		gmat[i__ + i__ * gmat_dim1]);
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, t, &gmat[i__ * gmat_dim1 + 1], &c__1, &d2[
		i__], &b[1], &c__1, t, &c__1, &gmat[i__ + i__ * gmat_dim1]);
/* L30: */
    }
    return 0;
} /* ltoup_ */

/* SACTH */
doublereal sacth_(doublereal *h__, integer *active, integer *t)
{
    /* System generated locals */
    integer i__1;
    doublereal ret_val, d__1;

    /* Local variables */
    static integer i__, j;
    static doublereal sum;


/*     COMPUTE SACTH=THE SUM OF SQUARED CONSTRAINTS FROM THE CURRENT */
/*     WORKING SET */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --active;
    --h__;

    /* Function Body */
    sum = 0.;
    if (*t <= 0) {
	goto L20;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = active[i__];
/* Computing 2nd power */
	d__1 = h__[j];
	sum += d__1 * d__1;
/* L10: */
    }
L20:
    ret_val = sum;
    return ret_val;
} /* sacth_ */

/* SAVEXK */
/* Subroutine */ int savexk_(doublereal *x, integer *n, doublereal *u)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     STORE X(I) IN U(I)  I=1,2,....,N */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --u;
    --x;

    /* Function Body */
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	u[i__] = x[i__];
/* L10: */
    }
    return 0;
} /* savexk_ */

/* ATOLOW */
/* Subroutine */ int atolow_(integer *t, integer *n, integer *p1, doublereal *
	a, integer *mda, doublereal *tol, integer *pranka, doublereal *d1, 
	doublereal *g)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer imax;
    static doublereal rmax;
    static integer i__, ldiag;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static integer krank;
    extern /* Subroutine */ int permin_(integer *, integer *), rowmax_(
	    integer *, integer *, doublereal *, integer *, integer *, integer 
	    *, integer *, doublereal *), prmrow_(doublereal *, integer *, 
	    integer *, integer *, integer *);


/*     THIS ROUTINE TRANSFORMS THE T*N (T<=N) MATRIX A TO LOWER */
/*     TRIANGULAR                                    ========== */
/*     FORM BY MULTIPLYING WITH AN ORTHOGONAL MATRIX Q1 AND A PERMUTA- */
/*     TION MATRIX P1 SO THAT */
/*                        P1(TR)*A*Q1 = (L@D0) */
/*     THE PURPOSE IS TO SOLVE FOR V, THE SYSTEM    A(TR)*V = G   (1) */
/*     THEREFOR THE TRANSFORMATION  G@D= Q1(TR)*G   IS ALSO DONE. */

/*     ON ENTRY */

/*     T    INTEGER SCALAR CONTAINING NO. OF ROWS IN MATRIX A */
/*          AND LENGTH OF ARRAYS P1,D1 */
/*     N    INTEGER SCALAR CONTAINING NO. OF COLUMNS IN MATRIX A AND */
/*          LENGTH OF ARRAY G */
/*     A(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N CONTAINING */
/*          THE MATRIX A THAT SHALL BE TRIANGULARIZED */
/*     MDA  INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*          MDA MUST BE >=T */
/*     TOL  REAL SCALAR CONTAINING A SMALL VALUE USED TO DETERMINE */
/*          PSEUDO RANK OF MATRIX A */
/*     G()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N CONTAINING */
/*          THE GRADIENT OF THE OBJECTIVE FUNCTION AT CURRENT POINT */
/*          WHICH IS THE RIGHT HAND SIDE OF SYSTEM (1) ABOVE */

/*     ON RETURN */

/*        L IS STORED IN THE LOWER PART OF A */
/*        THE NORMALS (EXCEPT ONE ELEMENT) THAT MAKES UP THE MATRIX Q1 */
/*        ARE STORED IN THE ROWS OF THE ARRAY A */
/*        P1 HOLDS THE PERMUTATION ORDER OF ROWS IN MATRIX A */
/*        D1(I) I=1,2,....,T HOLDS THE MISSING ELEMENT OF A NORMAL */
/*        PRANKA IS THE PSEUDORANK OF THE MATRIX A (NORMALLY =T) WHEN */
/*        TOL IS USED AS A SMALL POSITIVE NUMBER */
/*        G IS SET TO Q1(TR)*G */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --g;
    --p1;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;

    /* Function Body */
    *pranka = *t;
    if (*t == 0) {
	return 0;
    }
    ldiag = min(*t,*n);
    permin_(&p1[1], &ldiag);
    i__1 = ldiag;
    for (i__ = 1; i__ <= i__1; ++i__) {

/*     FIND LONGEST ROW AND DO PERMUTATIONS IF NECESSARY */

	krank = i__;
	rowmax_(t, n, &a[a_offset], mda, &i__, &i__, &imax, &rmax);
	if (rmax < *tol) {
	    goto L60;
	}
	p1[i__] = imax;
	prmrow_(&a[a_offset], mda, n, &i__, &imax);

/*     CONSTRUCT A REFLECTION AND APPLY IT FROM THE RIGHT ON MATRIX A */

	i__2 = i__ + 1;
	i__3 = *t - i__;
	h12per_(&c__1, &i__, &i__2, n, &a[i__ + a_dim1], mda, &d1[i__], &a[
		i__ + 1 + a_dim1], mda, &c__1, &i__3, &a[i__ + i__ * a_dim1]);

/*     APPLY IT FORM THE LEFT ON VECTOR G (R.H. SIDE IN  A(TR)*V=G ) */

	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, n, &a[i__ + a_dim1], mda, &d1[i__], &g[1],
		 &c__1, n, &c__1, &a[i__ + i__ * a_dim1]);
	krank = i__ + 1;
/* L50: */
    }
L60:
    *pranka = krank - 1;
    return 0;
} /* atolow_ */

/* C2TOUP */
/* Subroutine */ int c2toup_(integer *m, integer *nmp, doublereal *c2, 
	integer *mdc, doublereal *tol, integer *p3, integer *prankc, 
	doublereal *d3)
{
    /* System generated locals */
    integer c2_dim1, c2_offset, i__1, i__2, i__3;
    doublereal d__1;

    /* Local variables */
    static integer kmax;
    static doublereal rmax;
    static integer k, ldiag;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static integer krank;
    static doublereal u11;
    extern /* Subroutine */ int colmax_(integer *, integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, doublereal *), 
	    permin_(integer *, integer *), prmcol_(doublereal *, integer *, 
	    integer *, integer *, integer *);


/*     NMP=N-PSEUDORANK OF MATRIX A */

/*     THIS ROUTINE TRANSFORMS THE M*NMP (M>=NMP) MATRIX C2 TO UPPER */
/*                                                       =========== */
/*     TRIANGULAR FORM BY MULTIPLYING WITH AN ORTHOGONAL MATRIX Q3 AND */
/*     A PERMUTATION MATRIX P3 SO THAT */
/*                                       Q3(TR)*C2*P3 = (U) */
/*                                                      (0) */
/*     ON ENTRY */

/*     M    INTEGER SCALAR CONTAINING NO. OF ROWS IN MATRIX C2 */
/*     NMP  INTEGER SCALAR CONTAINING NO. OF COLUMNS IN MATRIX C2 */
/*          AND LENGTH OF ARRAYS P3,D3 */
/*     C2(,)REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*NMP */
/*          CONTAINING THE MATRIX C2 TO TRIANGULARIZE */
/*     MDC  INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C2 */
/*          MDC MUST BE>=M */
/*     TOL  REAL SCALAR CONTAINING A SMALL VALUE USED TO DETERMINE */
/*          PSEUDO RANK OF MATRIX C2 */

/*     ON RETURN */
/*        U IS STORED IN THE UPPER PART OF C2 */
/*        THE NORMALS (EXCEPT ONE ELEMENT) THAT MAKES UP THE MATRIX Q3 */
/*        ARE STORED IN THE COLUMNS OF THE ARRAY C2 */
/*        P3 HOLDS PERMUTATION ORDER OF COLUMNS IN MATRIX C2 */
/*        D3(I) I=1,2,....,NMP HOLDS THE MISSING ELEMENT OF A NORMAL */
/*        PRANKC IS THE PSEUDORANK OF THE MATRIX C2 (NORMALLY=NMP) WHEN */
/*        TOL IS USED AS A SMALL POSITIVE NUMBER */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    c2_dim1 = *mdc;
    c2_offset = 1 + c2_dim1 * 1;
    c2 -= c2_offset;
    --p3;
    --d3;

    /* Function Body */
    *prankc = min(*m,*nmp);
    if (*nmp == 0 || *m == 0) {
	return 0;
    }
    permin_(&p3[1], nmp);
    ldiag = *prankc;
    i__1 = ldiag;
    for (k = 1; k <= i__1; ++k) {

/*     FIND LONGEST COLUMN AND DO PERMUTATIONS IF NECESSARY */

	colmax_(m, nmp, &c2[c2_offset], mdc, &k, &k, &kmax, &rmax);
	p3[k] = kmax;
	prmcol_(&c2[c2_offset], mdc, m, &k, &kmax);

/*     CONSTRUCT A REFLECTION AND APPLY IT FROM THE LEFT ON MATRIX C2 */

	i__2 = k + 1;
	i__3 = *nmp - k;
	h12per_(&c__1, &k, &i__2, m, &c2[k * c2_dim1 + 1], &c__1, &d3[k], &c2[
		(k + 1) * c2_dim1 + 1], &c__1, mdc, &i__3, &c2[k + k * 
		c2_dim1]);
/* L50: */
    }

/*     DETERMINE PSEUDO RANK OF MATRIX C2 */

    krank = 0;
    u11 = (d__1 = c2[c2_dim1 + 1], abs(d__1));
    i__1 = ldiag;
    for (k = 1; k <= i__1; ++k) {
	if ((d__1 = c2[k + k * c2_dim1], abs(d__1)) <= *tol * u11) {
	    goto L70;
	}
	krank = k;
/* L60: */
    }
L70:
    *prankc = krank;
    return 0;
} /* c2toup_ */

/* CDX */
/* Subroutine */ int cdx_(doublereal *c1, integer *mdc, integer *m, integer *
	dima, doublereal *b1, doublereal *c2, integer *rankc2, integer *dimc2,
	 doublereal *d2, doublereal *dv, doublereal *fprim)
{
    /* System generated locals */
    integer c1_dim1, c1_offset, c2_dim1, c2_offset, i__1, i__2;

    /* Local variables */
    static integer i__, k;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static doublereal sum;


/*     COMPUTE THE ORTHOGONAL PROJECTION OF THE VECTOR OF RESIDUALS (F) */
/*     ONTO THE COLUMN SPACE OF THE JACOBIAN */
/*     I.E. */
/*          FPRIM@D= C1*B1+Q3*(DV1)  (1)  WHERE  DV= (DV1) RANKC2*1 */
/*                           ( 0 )                  (DV2) (M-RANKC2)*1 */

/*     ON ENTRY */

/*     C1(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*DIMA */
/*           CONTAINING THE MATRIX C1 OF ORDER M*DIMA */
/*     MDC   INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAYS C1,C2 */
/*           MDC MUST BE > M */
/*     M     INTEGER SCALAR CONTAINING NO. OF ROWS IN MATRICES C1 AND C2 */
/*           LENGTH OF ARRAYS DV,FPRIM */
/*     DIMA INTEGER SCALAR CONTAINING NO. OF COLUMNS IN MATRIX C1 */
/*           AND LENGTH OF ARRAY B1 */
/*     B1()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION DIMA */
/*           CONTAINING THE VECTOR B1 OF (1) ABOVE */
/*     C2(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*RANKC2 */
/*           CONTAINING INFO. TO FORM MATRIX Q3 OF (1) ABOVE */
/*     RANKC2 INTEGER SCALAR CONTAINING PSEUDO RANK OF ORIGINAL MATRIX */
/*           C2 EMANATING FROM THE JACOBIAN OF THE OBJECTIVE FUNCTION */
/*     DIMC2  INTEGER SCALAR CONTAINING NO. OF ELEMENTS IN DV THAT */
/*            SHOULD BE CONSIDDERED AS NONZERO */
/*     D2()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKC2 */
/*           CONTAINING MORE INFO. NEEDED TO FORM MATRIX Q3 OF (1) */
/*     DV()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M CONTAINING */
/*           THE CONCATENATED VECTORS DV1 AND DV2 OF (1) ABOVE */

/*     ON RETURN */

/*     FPRIM() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M CONTAINS */
/*             THE COMPUTED VALUE OF C1*B1+Q3*(DV1) */
/*                                            ( 0 ) */
/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    c2_dim1 = *mdc;
    c2_offset = 1 + c2_dim1 * 1;
    c2 -= c2_offset;
    c1_dim1 = *mdc;
    c1_offset = 1 + c1_dim1 * 1;
    c1 -= c1_offset;
    --fprim;
    --dv;
    --b1;
    --d2;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	fprim[i__] = dv[i__];
	if (i__ > *dimc2) {
	    fprim[i__] = 0.;
	}
/* L10: */
    }
    if (*rankc2 <= 0) {
	goto L30;
    }

/*     FORM FPRIM = Q3*(DV1) */
/*                     ( 0 ) */

    i__1 = *rankc2;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *rankc2 + 1 - i__;
	i__2 = k + 1;
	h12per_(&c__2, &k, &i__2, m, &c2[k * c2_dim1 + 1], &c__1, &d2[k], &
		fprim[1], &c__1, m, &c__1, &c2[k + k * c2_dim1]);
/* L20: */
    }
L30:

/*     FORM FPRIM@D= FPRIM+C1*B1 */

    if (*dima <= 0) {
	goto L60;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	sum = 0.;
	i__2 = *dima;
	for (k = 1; k <= i__2; ++k) {
	    sum += c1[i__ + k * c1_dim1] * b1[k];
/* L40: */
	}
	fprim[i__] += sum;
/* L50: */
    }
L60:
    return 0;
} /* cdx_ */

/* ROWMAX */
/* Subroutine */ int rowmax_(integer *m, integer *n, doublereal *a, integer *
	mda, integer *is, integer *js, integer *imax, doublereal *rmax)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;
    doublereal d__1;

    /* Local variables */
    static integer i__, j;
    static doublereal rowsum;


/*     FIND LONGEST ROW AMONG ROWS IS,IS+1,...,M  IN THE M*N MATRIX A */
/*     THE ELEMENTS CONSIDDERED IN EACH ROW START IN COLUMN JS AND END */
/*     IN COLUMN N */
/*                      N */
/*      RMAX@D=MAXIMUM(SIGMA A(I,J)**2) */
/*              I     J=JS */
/*     WHERE I=IS,IS+1,...,M */
/*     IMAX IS SET TO THE ROWNUMBER FOR WHICH THE MAXIMUM IS ATTAINED */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    *rmax = -1.;
    i__1 = *m;
    for (i__ = *is; i__ <= i__1; ++i__) {
	rowsum = 0.;
	i__2 = *n;
	for (j = *js; j <= i__2; ++j) {
/* Computing 2nd power */
	    d__1 = a[i__ + j * a_dim1];
	    rowsum += d__1 * d__1;
/* L10: */
	}
	if (rowsum <= *rmax) {
	    goto L20;
	}
	*rmax = rowsum;
	*imax = i__;
L20:
	;
    }
    return 0;
} /* rowmax_ */

/* COLMAX */
/* Subroutine */ int colmax_(integer *m, integer *n, doublereal *a, integer *
	mda, integer *is, integer *js, integer *jmax, doublereal *rmax)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;
    doublereal d__1;

    /* Local variables */
    static integer i__, j;
    static doublereal colsum;


/*     FIND LONGEST COLUMN AMONG JS,JS+1,.....,N IN THE M*N MATRIX A */
/*     THE ELEMENTS CONSIDDERED IN EACH COLUMN START IN ROW IS AND */
/*     END IN ROW M */
/*                     M */
/*     RMAX@D=MAXIMUM(SIGMA A(I,J)**2) */
/*             J     I=IS */
/*     WHERE J=JS,JS+1,.....,N */
/*     JMAX IS SET TO THE COLUMN NUMBER FOR WHICH THE MAXIMUM */
/*     IS ATTAINED */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    *rmax = -1.;
    i__1 = *n;
    for (j = *js; j <= i__1; ++j) {
	colsum = 0.;
	i__2 = *m;
	for (i__ = *is; i__ <= i__2; ++i__) {
/* Computing 2nd power */
	    d__1 = a[i__ + j * a_dim1];
	    colsum += d__1 * d__1;
/* L10: */
	}
	if (colsum <= *rmax) {
	    goto L20;
	}
	*rmax = colsum;
	*jmax = j;
L20:
	;
    }
    return 0;
} /* colmax_ */

/* PRMROW */
/* Subroutine */ int prmrow_(doublereal *a, integer *mda, integer *n, integer 
	*i__, integer *k)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1;

    /* Local variables */
    static doublereal temp;
    static integer j;


/*     CHANGE ROWS I AND K OF THE MATRIX A */
/*     N IS THE LENGTH OF A ROW */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    if (*i__ == *k) {
	return 0;
    }
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	temp = a[*i__ + j * a_dim1];
	a[*i__ + j * a_dim1] = a[*k + j * a_dim1];
	a[*k + j * a_dim1] = temp;
/* L10: */
    }
    return 0;
} /* prmrow_ */

/* PRMCOL */
/* Subroutine */ int prmcol_(doublereal *a, integer *mda, integer *m, integer 
	*j, integer *k)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1;

    /* Local variables */
    static doublereal temp;
    static integer i__;


/*     CHANGE COLUMN J AND K OF THE MATRIX A */
/*     M IS THE LENGTH OF A COLUMN */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    if (*j == *k) {
	return 0;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	temp = a[i__ + *j * a_dim1];
	a[i__ + *j * a_dim1] = a[i__ + *k * a_dim1];
	a[i__ + *k * a_dim1] = temp;
/* L10: */
    }
    return 0;
} /* prmcol_ */

/* PTRV */
/* Subroutine */ int ptrv_(integer *p, integer *m, doublereal *v, integer *
	mdv, integer *n)
{
    /* System generated locals */
    integer v_dim1, v_offset, i__1;

    /* Local variables */
    static integer i__;
    extern /* Subroutine */ int prmrow_(doublereal *, integer *, integer *, 
	    integer *, integer *);


/*     P IS A PERMUTATION MATRIX   T(1,J1)*T(2,J2)*......*T(M,JM) */
/*     THE ARRAY P CONTAINS THE VALUES J1,J2,....,JM */
/*     PERMUTE ROWS IN MATRIX V BY FORMING */
/*               V@D=P(TR)*V */

    /* Parameter adjustments */
    --p;
    v_dim1 = *mdv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;

    /* Function Body */
    if (*m <= 0 || *n <= 0) {
	return 0;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	prmrow_(&v[v_offset], mdv, n, &i__, &p[i__]);
/* L10: */
    }
    return 0;
} /* ptrv_ */

/* VPTR */
/* Subroutine */ int vptr_(integer *p, integer *m, doublereal *v, integer *
	mdv, integer *n)
{
    /* System generated locals */
    integer v_dim1, v_offset, i__1;

    /* Local variables */
    static integer i__, k;
    extern /* Subroutine */ int prmcol_(doublereal *, integer *, integer *, 
	    integer *, integer *);


/*     P IS A PERMUTATION MATRIX T(1,J1)*T(2,J2)*........*T(M,JM) */
/*     THE ARRAY P CONTAINS THE VALUES J1,J2,......,JM */
/*     PERMUTE COLUMNS IN MATRIX V BY FORMING */
/*         V@D=V*P(TR) */

    /* Parameter adjustments */
    --p;
    v_dim1 = *mdv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;

    /* Function Body */
    if (*m <= 0 || *n <= 0) {
	return 0;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *m + 1 - i__;
	prmcol_(&v[v_offset], mdv, m, &k, &p[k]);
/* L10: */
    }
    return 0;
} /* vptr_ */

/* PV */
/* Subroutine */ int pv_(integer *p, integer *m, doublereal *v, integer *mdv, 
	integer *n)
{
    /* System generated locals */
    integer v_dim1, v_offset, i__1;

    /* Local variables */
    static integer i__, k;
    extern /* Subroutine */ int prmrow_(doublereal *, integer *, integer *, 
	    integer *, integer *);


/*     P IS A PERMUTATION MATRIX   T(1,J1)*T(2,J2)*.......*T(M,JM) */
/*     THE ARRAY P CONTAINS THE VALUES  J1,J2,.....,JM */
/*     PERMUTE ROWS IN MATRIX V BY FORMING */
/*               V@D=P*V */

    /* Parameter adjustments */
    --p;
    v_dim1 = *mdv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;

    /* Function Body */
    if (*m <= 0 || *n <= 0) {
	return 0;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *m + 1 - i__;
	prmrow_(&v[v_offset], mdv, n, &k, &p[k]);
/* L10: */
    }
    return 0;
} /* pv_ */

/* VP */
/* Subroutine */ int vp_(doublereal *v, integer *mdv, integer *m, integer *n, 
	integer *p)
{
    /* System generated locals */
    integer v_dim1, v_offset, i__1;

    /* Local variables */
    static integer j;
    extern /* Subroutine */ int prmcol_(doublereal *, integer *, integer *, 
	    integer *, integer *);


/*     P IS A PERMUTATION MATRIX   T(1,J1)*T(2,J2)*.......*T(N,JN) */
/*     THE ARRAY P CONTAINS THE VALUES J1,J2,......,JN */
/*     PERMUTE COLUMNS IN MATRIX V BY FORMING */
/*               V@D=V*P */

    /* Parameter adjustments */
    v_dim1 = *mdv;
    v_offset = 1 + v_dim1 * 1;
    v -= v_offset;
    --p;

    /* Function Body */
    if (*m <= 0 || *n <= 0) {
	return 0;
    }
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	prmcol_(&v[v_offset], mdv, m, &j, &p[j]);
/* L10: */
    }
    return 0;
} /* vp_ */

/* H12PER */
/*     SUBROUTINE H12PER(MODE,LPIVOT,L1,M,U,IUE,UP,C,ICE,ICV,NCV,PIVEL) */

/*      *********************************************************** */
/*      THIS IS A MODIFIED H12(......)  ROUTINE  (PER LINDSTROM) */

/*      THE LAST PARAMETER (PIVEL) IS THE PIVOT ELEMENT */

/*      *********************************************************** */
/*     CONSTRUCTION AND/OR APPLICATION OF A SINGLE */
/*     HOUSEHOLDER TRANSFORMATION..     Q = I + U*(U**T)/B */

/*     MODE    = 1 OR 2   TO SELECT ALGORITHM  H1  OR  H2 . */
/*     LPIVOT IS THE INDEX OF THE PIVOT ELEMENT. */
/*     L1,M   IF L1 .LE. M   THE TRANSFORMATION WILL BE CONSTRUCTED TO */
/*            ZERO ELEMENTS INDEXED FROM L1 THROUGH M.   IF L1 GT. M */
/*            THE SUBROUTINE DOES AN IDENTITY TRANSFORMATION. */
/*     U(),IUE,UP    ON ENTRY TO H1 U() CONTAINS THE PIVOT VECTOR. */
/*                   IUE IS THE STORAGE INCREMENT BETWEEN ELEMENTS. */
/*                                       ON EXIT FROM H1 U() AND UP */
/*                   CONTAIN QUANTITIES DEFINING THE VECTOR U OF THE */
/*                   HOUSEHOLDER TRANSFORMATION.   ON ENTRY TO H2 U() */
/*                   AND UP SHOULD CONTAIN QUANTITIES PREVIOUSLY COMPUTED */
/*                   BY H1.  THESE WILL NOT BE MODIFIED BY H2. */
/*     C()    ON ENTRY TO H1 OR H2 C() CONTAINS A MATRIX WHICH WILL BE */
/*            REGARDED AS A SET OF VECTORS TO WHICH THE HOUSEHOLDER */
/*            TRANSFORMATION IS TO BE APPLIED.  ON EXIT C() CONTAINS THE */
/*            SET OF TRANSFORMED VECTORS. */
/*     ICE    STORAGE INCREMENT BETWEEN ELEMENTS OF VECTORS IN C(). */
/*     ICV    STORAGE INCREMENT BETWEEN VECTORS IN C(). */
/*     NCV    NUMBER OF VECTORS IN C() TO BE TRANSFORMED. IF NCV .LE. 0 */
/*            NO OPERATIONS WILL BE DONE ON C(). */
/*     PIVEL      IS THE PIVOT ELEMENT */

/* Subroutine */ int h12per_(integer *mode, integer *lpivot, integer *l1, 
	integer *m, doublereal *u, integer *iue, doublereal *up, doublereal *
	c__, integer *ice, integer *icv, integer *ncv, doublereal *pivel)
{
    /* System generated locals */
    integer u_dim1, u_offset, i__1, i__2;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer incr;
    static doublereal b;
    static integer i__, j;
    static doublereal clinv;
    static integer i2, i3, i4;
    static doublereal cl, sm, sm1, one;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    u_dim1 = *iue;
    u_offset = 1 + u_dim1 * 1;
    u -= u_offset;
    --c__;

    /* Function Body */
    one = 1.;

    if (0 >= *lpivot || *lpivot >= *l1 || *l1 > *m) {
	return 0;
    }
    cl = abs(*pivel);
    if (*mode == 2) {
	goto L60;
    }
/*                            ****** CONSTRUCT THE TRANSFORMATION. ****** */
    i__1 = *m;
    for (j = *l1; j <= i__1; ++j) {
/* L10: */
/* Computing MAX */
	d__2 = (d__1 = u[j * u_dim1 + 1], abs(d__1));
	cl = max(d__2,cl);
    }
    if (cl <= 0.) {
	goto L130;
    } else {
	goto L20;
    }
L20:
    clinv = one / cl;
/* Computing 2nd power */
    d__1 = *pivel * clinv;
    sm = d__1 * d__1;
    i__1 = *m;
    for (j = *l1; j <= i__1; ++j) {
/* L30: */
/* Computing 2nd power */
	d__1 = u[j * u_dim1 + 1] * clinv;
	sm += d__1 * d__1;
    }
/*                              CONVERT DBLE. PREC. SM TO SNGL. PREC. SM1 */
    sm1 = sm;
    cl *= sqrt(sm1);
    if (*pivel <= 0.) {
	goto L50;
    } else {
	goto L40;
    }
L40:
    cl = -cl;
L50:
    *up = *pivel - cl;
    *pivel = cl;
    goto L70;
/*            ****** APPLY THE TRANSFORMATION  I+U*(U**T)/B  TO C. ****** */

L60:
    if (cl <= 0.) {
	goto L130;
    } else {
	goto L70;
    }
L70:
    if (*ncv <= 0) {
	return 0;
    }
    b = *up * *pivel;
/*                       B  MUST BE NONPOSITIVE HERE.  IF B = 0., RETURN. */

    if (b >= 0.) {
	goto L130;
    } else {
	goto L80;
    }
L80:
    b = one / b;
    i2 = 1 - *icv + *ice * (*lpivot - 1);
    incr = *ice * (*l1 - *lpivot);
    i__1 = *ncv;
    for (j = 1; j <= i__1; ++j) {
	i2 += *icv;
	i3 = i2 + incr;
	i4 = i3;
	sm = c__[i2] * *up;
	i__2 = *m;
	for (i__ = *l1; i__ <= i__2; ++i__) {
	    sm += c__[i3] * u[i__ * u_dim1 + 1];
/* L90: */
	    i3 += *ice;
	}
	if (sm != 0.) {
	    goto L100;
	} else {
	    goto L120;
	}
L100:
	sm *= b;
	c__[i2] += sm * *up;
	i__2 = *m;
	for (i__ = *l1; i__ <= i__2; ++i__) {
	    c__[i4] += sm * u[i__ * u_dim1 + 1];
/* L110: */
	    i4 += *ice;
	}
L120:
	;
    }
L130:
    return 0;
} /* h12per_ */

/* CH */
/* Subroutine */ int ch_(integer *m, integer *n, doublereal *c__, integer *
	mdc, doublereal *h__, integer *mdh, doublereal *v)
{
    /* System generated locals */
    integer c_dim1, c_offset, h_dim1, h_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer i__, j, k;
    static doublereal s1;


/*     FORM PRODUCT   C@D=C*H */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    h_dim1 = *mdh;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = *n;
	for (k = 1; k <= i__2; ++k) {
	    v[k] = c__[i__ + k * c_dim1];
/* L10: */
	}
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    s1 = 0.;
	    i__3 = *n;
	    for (k = 1; k <= i__3; ++k) {
		s1 += v[k] * h__[k + j * h_dim1];
/* L20: */
	    }
	    c__[i__ + j * c_dim1] = s1;
/* L30: */
	}
/* L50: */
    }
    return 0;
} /* ch_ */

/* HXCOMP */
/* Subroutine */ int hxcomp_(doublereal *h__, integer *mdh, integer *n, 
	doublereal *x, doublereal *s)
{
    /* System generated locals */
    integer h_dim1, h_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j;
    static doublereal s1;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --s;
    --x;
    h_dim1 = *mdh;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;

    /* Function Body */
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	s[i__] = x[i__];
/* L10: */
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	s1 = 0.;
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    s1 += h__[i__ + j * h_dim1] * s[j];
/* L40: */
	}
	x[i__] = s1;
/* L50: */
    }
    return 0;
} /* hxcomp_ */

/* GIVEN2 */
/* Subroutine */ int given2_(doublereal *c__, doublereal *s, doublereal *x, 
	doublereal *y)
{
    static doublereal xt;


/*     APPLY THE ROTATION COMPUTED BY GIVEN1 TO  (X,Y) */

/*     X = (C S)(X)     Y = (S -C)(X) */
/*              (Y)               (Y) */

/*     INTERNAL VARIABLE */

    xt = *c__ * *x + *s * *y;
    *y = *s * *x - *c__ * *y;
    *x = xt;
    return 0;
} /* given2_ */

/* PSPECF */
/* Subroutine */ int pspecf_(integer *n, integer *p, doublereal *w, 
	doublereal *f)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, k;


/*     P CONTAINS A PERMUTATION MATRIX THAT HOLDS THE LOCATION */
/*     OF THE ORIGINAL ELEMENTS */
/*     I.E. */
/*          P(K) SAYS WHERE ELEMENT K IS MOVED */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --f;
    --w;
    --p;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
	i__ = p[k];
	f[k] = w[i__];
/* L10: */
    }
    return 0;
} /* pspecf_ */

/* PROD1 */
/* Subroutine */ int prod1_(doublereal *h__, integer *mdh, doublereal *s, 
	doublereal *p, doublereal *beta, integer *j, integer *tp1)
{
    /* System generated locals */
    integer h_dim1, h_offset, i__1, i__2;

    /* Local variables */
    static integer i__, k, ip1;


/*     FORM PRODUCT OF GIVENS MATRICES */
/*     AND TRANSPOSE IT */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    h_dim1 = *mdh;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;
    --beta;
    --p;
    --s;

    /* Function Body */
    if (*j > *tp1) {
	return 0;
    }
    i__1 = *tp1;
    for (i__ = *j; i__ <= i__1; ++i__) {
	ip1 = i__ + 1;
	if (ip1 <= *tp1) {
	    h__[ip1 + i__ * h_dim1] = s[i__];
	}
	i__2 = *tp1;
	for (k = i__; k <= i__2; ++k) {
	    h__[i__ + k * h_dim1] = p[k] * beta[i__];
/* L10: */
	}
/* L20: */
    }
    return 0;
} /* prod1_ */

/* CQHP2 */
/* Subroutine */ int cqhp2_(integer *time, doublereal *c__, integer *mdc, 
	integer *m, integer *n, integer *t, integer *pranka, doublereal *
	pivot, integer *na, doublereal *a, integer *mda, doublereal *d1, 
	doublereal *h__, integer *mdh, integer *p2, doublereal *v)
{
    /* System generated locals */
    integer c_dim1, c_offset, a_dim1, a_offset, h_dim1, h_offset, i__1, i__2;

    /* Local variables */
    static integer i__;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *), ch_(integer *, 
	    integer *, doublereal *, integer *, doublereal *, integer *, 
	    doublereal *), vp_(doublereal *, integer *, integer *, integer *, 
	    integer *);


/*     COMPUTE  C@D= C*Q1*H*P2     WHERE */
/*             Q1 IS ORTHOGONAL STORED IN ARRAY A AND D1 IN A SPECIAL WAY */
/*              H IS PRODUCT OF GIVENS ROTATIONS */
/*              P2 IS A PERMUTATION MATRIX */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --v;
    --p2;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --pivot;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;
    h_dim1 = *mdh;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;

    /* Function Body */
    if (*na == 0) {
	return 0;
    }

/*     COMPUTE    C@D=C*Q1 */
    i__1 = *na;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, n, &a[i__ + a_dim1], mda, &d1[i__], &c__[
		c_offset], mdc, &c__1, m, &pivot[i__]);
/* L10: */
    }
    if (*time <= 2) {
	goto L20;
    }
    if (*time == 3 && *t == 0) {
	goto L20;
    }

/*     COMPUTE   C@D=C*H */

    ch_(m, n, &c__[c_offset], mdc, &h__[h_offset], mdh, &v[1]);
L20:
    if (*t == *pranka) {
	return 0;
    }

/*     COMPUTE   C@D=C*P2 */

    vp_(&c__[c_offset], mdc, m, pranka, &p2[1]);
    return 0;
} /* cqhp2_ */

/* SIGNCH */
/* Subroutine */ int signch_(integer *time, integer *p1, doublereal *v, 
	integer *t, integer *active, integer *bnd, doublereal *betkm1, 
	doublereal *gndnrm, integer *itno, integer *scale, doublereal *diag, 
	doublereal *gres, doublereal *h__, integer *kp, integer *l, integer *
	j, integer *s, doublereal *p2, doublereal *work)
{
    /* Initialized data */

    static integer ival = 4;
    static doublereal delta = 10.;
    static doublereal tau = .5;

    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal rowi, e;
    static integer i__, k;
    static doublereal sqrel;
    extern /* Subroutine */ int unscr_(integer *, integer *, integer *, 
	    integer *), pv_(integer *, integer *, doublereal *, integer *, 
	    integer *), pspecf_(integer *, integer *, doublereal *, 
	    doublereal *);
    static integer kp1, tm1;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --work;
    --p2;
    --h__;
    --diag;
    --active;
    --v;
    --p1;

    /* Function Body */

/*     DETERMINE S.    THE CONSTRAINT THAT SHALL BE DELETED FROM */
/*                     THE CURRENT WORKING SET */
/*     S IS SET TO 0   IF NO CONSTRAINT SHALL BE DELETED */
/*     S IS SET TO K   (KP<K<=T) CONSTRAINT TO BE DELETED */

/*     J IS SET TO THE CORRESPONDING ROW NUMBER IN P1(TR)*DIAG*A */
/*                     IF A ROW SHALL BE DELETED */

    *s = 0;
    if (*kp == *t) {
	return 0;
    }

/*     LOOK FOR GREATEST NEGATIVE MULTIPLIER(DIFFERENT FROM ZERO) */
/*     AMONG MULTIPLIERS CORRESPONDING TO CONSTRAINTS DEFINED AS */
/*     INEQUALITIES */

    e = 0.;
    sqrel = sqrt(machin_1.drelpr);
    kp1 = *kp + 1;
    i__1 = *t;
    for (i__ = kp1; i__ <= i__1; ++i__) {
	k = active[i__];
	if (*scale == 0) {
	    rowi = diag[i__];
	}
	if (*scale != 0) {
	    rowi = 1. / diag[i__];
	}
	if (-sqrel < v[i__] * rowi) {
	    goto L5;
	}
	if (v[i__] * rowi >= e) {
	    goto L5;
	}
	e = v[i__] * rowi;
	*s = i__;
L5:
	;
    }
    if (*gres > -e * delta) {
	*s = 0;
    }
    if (*s == 0) {
	return 0;
    }
    k = active[*s];
    i__ = *bnd + k - *kp;
    if (active[i__] == 1) {
	goto L10;
    }
    if (*itno - active[i__] < ival && *betkm1 > tau * *gndnrm) {
	*s = 0;
    }
    if (*s == 0) {
	return 0;
    }
L10:
    active[i__] = -1;
    if (*betkm1 <= tau * *gndnrm) {
	unscr_(&active[1], bnd, l, kp);
    }
    if (*time >= 3) {
	goto L40;
    }

/*     TRANSFORM THE TRANPOSITION VECTOR P1 TO A ORDER VECTOR */

    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p2[i__] = (doublereal) i__ + .1;
/* L20: */
    }

/*     PERMUTE THE ORDER VECTOR */

    pv_(&p1[1], t, &p2[1], t, &c__1);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p1[i__] = (integer) p2[i__];
/* L30: */
    }
L40:
    *j = p1[*s];
    tm1 = *t - 1;
    if (tm1 < 1) {
	return 0;
    }

/*     FORM ORDER VECTOR OF DIMENSION T-1 */

    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (i__ < *j) {
	    p2[i__] = (doublereal) i__ + .1;
	}
	if (i__ == *j) {
	    p2[i__] = 0.;
	}
	if (i__ > *j) {
	    p2[i__] = (doublereal) (i__ - 1) + .1;
	}
/* L50: */
    }
    pspecf_(t, &p1[1], &p2[1], &work[1]);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p1[i__] = (integer) work[i__];
	if (p1[i__] == 0) {
	    k = i__;
	}
/* L60: */
    }
    if (k == *t) {
	return 0;
    }
    i__1 = tm1;
    for (i__ = k; i__ <= i__1; ++i__) {
	p1[i__] = p1[i__ + 1];
/* L70: */
    }
    return 0;
} /* signch_ */

/* REORD */
/* Subroutine */ int reord_(doublereal *a, integer *mda, integer *t, integer *
	n, doublereal *bv, integer *j, integer *s, integer *active, integer *
	inact, integer *kq, integer *p4, doublereal *u, integer *iscale, 
	doublereal *diag)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static doublereal temp;
    static integer i__, k;
    extern /* Subroutine */ int delete_(integer *, integer *, integer *, 
	    integer *, integer *);
    static integer ip1, tm1;


/*     DELETE COLUMN J AND COMPRESS THE UPPER TRIANGULAR MATRIX R */
/*     DELETE ELEMENT J AND COMPRESS THE VECTOR BV */
/*     DELETE ELEMENT S IN SCALING MATRIX DIAG AND COMPRESS IT */

/*     R(TR)  (T*T)   IS STORED AS THE FIRST T COLUMNS OF THE ARRAY A */

/*     CHANGES ARE DONE TO THE ACTIVE AND INACTIVE SETS */

/*     ON ENTRY@D */

/*     A(,)         SEE EXPLANATION IN SUBROUTINE MULEST */
/*     MDA          LEADING DIMENSION OF THE ARRAY A */
/*     T            INTEGER SCALAR CONTAINING NO. OF CONSTRAINTS IN WORKING SET */
/*     N            INTEGER SCALAR CONTAINING NO. OF VARIABLES */
/*     BV()         SEE EXPLANATION IN SUBROUTINE MULEST */
/*     J            INTEGER SCALAR CONTAINING THE ROW NUMBER OF THE ROW IN THE */
/*                  LOWER TRIANGULAR MATRIX L TO BE DELETED */
/*     S            INTEGER SCALAR CONTAINING THE CORRESPONDING ROW NUMBER IN THE */
/*                  ORIGINAL MATRIX A */
/*     ACTIVE()     SEE EXPLANATION IN SUBROUTINE WRKSET */
/*     INACT()      SEE EXPLANATION IN SUBROUTINE WRKSET */
/*     KQ           INTEGER SCALAR CONTAINING THE NO. OF CONSTRAINTS IN THE */
/*                  CURRENT INACTIVE SET */
/*     P4()         SEE EXPLANATION IN SUBROUTINE WRKSET */
/*     ISCALE       SEE EXPLANATION SUBROUTINE MULEST */
/*     DIAG()       SEE EXPLANATION IN SUBROUTINE MULEST */

/*     ON RETURN@D */

/*     A(,)         A(I,K)@D=A(I+1,K) ;I=J,J+1,.....,T-1 */
/*                    AND K=1,2,......,J */
/*     T            T@D=T-1 */
/*     BV()         BV(I)@D=BV(I+1) ; I=J,J+1,....,T-1 */
/*     ACTIVE()     ONE CONSTRAINT NO. IS SUBTRACTED */
/*     INACT()      ONE CONSTRAINT NO. IS ADDED */
/*     KQ           KQ@D=KQ+1 */
/*     P4()         P4(J)@D= THE NO. OF ELEMENTS IN ROW NO. J OF THE LOWER */
/*                  TRIANGULAR MATRIX L */

/*     WORKING AREA@D */

/*     U()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --bv;
    --active;
    --inact;
    --p4;
    --u;
    --diag;

    /* Function Body */
    tm1 = *t - 1;
    if (*j == *t) {
	goto L30;
    }
    i__1 = *j;
    for (i__ = 1; i__ <= i__1; ++i__) {
	u[i__] = a[*j + i__ * a_dim1];
/* L5: */
    }
    i__1 = tm1;
    for (i__ = *j; i__ <= i__1; ++i__) {
	ip1 = i__ + 1;
	i__2 = *j;
	for (k = 1; k <= i__2; ++k) {
	    a[i__ + k * a_dim1] = a[ip1 + k * a_dim1];
/* L10: */
	}
	bv[i__] = bv[ip1];
/* L20: */
    }
    i__1 = *j;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a[*t + i__ * a_dim1] = u[i__];
/* L25: */
    }
L30:
    if (*iscale == 0 || *s == *t) {
	goto L50;
    }
    temp = diag[*s];
    i__1 = tm1;
    for (i__ = *s; i__ <= i__1; ++i__) {
	diag[i__] = diag[i__ + 1];
/* L40: */
    }
    diag[*t] = temp;
L50:

/*     SAVE NO. OF ELEMENTS IN ROW CORRESPONDING TO CONSTRAINT NO. S */

    k = active[*s];
    p4[k] = *j;
    delete_(&active[1], &inact[1], kq, t, s);
    return 0;
} /* reord_ */

/* ATSOLV */
/* Subroutine */ int atsolv_(doublereal *a, integer *mda, integer *t, 
	doublereal *b, doublereal *x, integer *n, doublereal *residu)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, j, k, l;
    static doublereal s1;


/*     A CONTAINS ( L@D0 ) WHERE L IS T*T LOWER TRIANGULAR */

/*     SOLVE FOR X THE SYSTEM */
/*         L*X = B */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    --b;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    if (*t == 0) {
	goto L55;
    }
    i__1 = *t;
    for (k = 1; k <= i__1; ++k) {
	j = *t - k + 1;
	l = j + 1;
	s1 = 0.;
	if (l > *t) {
	    goto L40;
	}
	i__2 = *t;
	for (i__ = l; i__ <= i__2; ++i__) {
	    s1 += x[i__] * a[i__ + j * a_dim1];
/* L10: */
	}
L40:
	x[j] = (b[j] - s1) / a[j + j * a_dim1];
/* L50: */
    }
L55:

/*     COMPUTE THE NORM OF THE RESIDUAL L*X-B */
/*     IF T=0    THEN  II B II IS COMPUTED WHICH IS THE NORM OF THE */
/*     GRADIENT OF THE OBJECTIVE FUNCTION */

    i__1 = *n - *t;
    *residu = enlsip_dnrm2_(&i__1, &b[*t + 1], &c__1);
    return 0;
} /* atsolv_ */

/*     HELP ROUTINES (FILE 2) FOR ENLSIP */
/*     DOUBLE PRECISION VERSION 841005 */

/* GRAD */
/* Subroutine */ int grad_(doublereal *a, integer *mda, integer *m, integer *
	n, doublereal *f, doublereal *g)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j;
    static doublereal gs;


/*     COMPUTE THE GRADIENT G(X) TO  0.5* II F(X) II**2       T */
/*            WHERE  F(X) = (F1(X),F2(X),...............FM(X)) */
/*     THE MATRIX A  (M*N)  IS THE JACOBIAN OF  F(X) */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --f;
    --g;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	gs = 0.;
	i__2 = *m;
	for (j = 1; j <= i__2; ++j) {
	    gs += a[j + i__ * a_dim1] * f[j];
/* L10: */
	}
	g[i__] = gs;
/* L20: */
    }
    return 0;
} /* grad_ */

/* QUAMIN */
/* Subroutine */ int quamin_(doublereal *x, doublereal *fx, doublereal *w, 
	doublereal *fw, doublereal *v, doublereal *fv, doublereal *u)
{
    static doublereal q, s, d1, d2;


/*     COMPUTE THE MINIMUM POINT U OF A QUADRATIC POLYNOMIAL PASSING */
/*     THROUGH (V,F(V)), (W,F(W)) AND (X,F(X)) */

/*     INTERNAL VARIABLES */

    d1 = *fv - *fx;
    d2 = *fw - *fx;
    s = (*w - *x) * (*w - *x) * d1 - (*v - *x) * (*v - *x) * d2;
    q = ((*v - *x) * d2 - (*w - *x) * d1) * 2.;
    *u = *x - s / q;
    return 0;
} /* quamin_ */

/* DELETE */
/* Subroutine */ int delete_(integer *oa, integer *oi, integer *q, integer *t,
	 integer *k)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     REMOVE CONSTRAINT K FROM THE ACTIVE SET AND */
/*     PLACE IT IN THE INACTIVE SET */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --oi;
    --oa;

    /* Function Body */
    ++(*q);
    oi[*q] = oa[*k];
    i__1 = *t;
    for (i__ = *k; i__ <= i__1; ++i__) {
	oa[i__] = oa[i__ + 1];
/* L10: */
    }
    --(*t);
    return 0;
} /* delete_ */

/* ADDIT */
/* Subroutine */ int addit_(integer *oa, integer *oi, integer *t, integer *q, 
	integer *k)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     ADD A CONSTRAINT TO THE ACTIVE SET */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --oi;
    --oa;

    /* Function Body */
    ++(*t);
    oa[*t] = oi[*k];
    i__1 = *q;
    for (i__ = *k; i__ <= i__1; ++i__) {
	oi[i__] = oi[i__ + 1];
/* L10: */
    }
    --(*q);
    return 0;
} /* addit_ */

/* LINEC */
/* Subroutine */ int linec_(doublereal *xold, doublereal *p, doublereal *f, 
	doublereal *v1, integer *m, integer *n, doublereal *alpha, doublereal 
	*psizer, doublereal *dpsize, doublereal *alflow, U_fp ffunc, U_fp 
	hfunc, doublereal *h__, doublereal *hnew, integer *active, integer *t,
	 integer *inact, integer *q, integer *l, doublereal *w, doublereal *
	alfupp, doublereal *fnew, doublereal *v2, doublereal *g, doublereal *
	psialf, doublereal *xdiff, integer *eval, integer *exit)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static doublereal diff, beta, alfk;
    extern /* Subroutine */ int redc_(doublereal *, doublereal *, doublereal *
	    , doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, integer *,
	     integer *, U_fp, doublereal *, doublereal *, U_fp, integer *, 
	    integer *, integer *, doublereal *, integer *, doublereal *, 
	    logical *);
    static integer ctrl;
    static doublereal pmax, psik, xmin;
    extern /* Subroutine */ int linc1_(doublereal *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *), linc2_(integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *, doublereal *, integer *, 
	    integer *, integer *, doublereal *);
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, k;
    static doublereal gamma;
    extern /* Subroutine */ int chder_(doublereal *, doublereal *, doublereal 
	    *, doublereal *, integer *, integer *, U_fp, integer *, integer *,
	     doublereal *, integer *, U_fp, integer *, integer *, doublereal *
	    , doublereal *, doublereal *, doublereal *, doublereal *);
    static doublereal pbeta;
    extern /* Subroutine */ int minrm_(doublereal *, doublereal *, doublereal 
	    *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *), minrn_(
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *);
    static doublereal alfkm1, alfkm2, alfkp1, psikm1, psikm2, pk, alfmin;
    static logical reduce;
    static doublereal alfmax;
    extern /* Subroutine */ int concat_(doublereal *, integer *, doublereal *,
	     integer *, integer *, integer *, integer *, doublereal *), 
	    update_(doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *), gac_(doublereal *, 
	    doublereal *, doublereal *, integer *, integer *, U_fp, U_fp, 
	    doublereal *, integer *, integer *, integer *, doublereal *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *);
    static doublereal eta;
    static integer mpl;
    static doublereal tau, xel;
    extern doublereal psi_(doublereal *, doublereal *, integer *, doublereal *
	    , doublereal *, doublereal *, integer *, U_fp, doublereal *, 
	    integer *, integer *, integer *, U_fp, doublereal *, integer *);
    static integer mpt;


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */




/*     THIS IS A LINE SEARCH ROUTINE FOR CONSTRAINED LEAST */
/*     SQUARES PROBLEMS */

/*     COMPUTE THE STEPLENGTH ALPHA FOR THE ITERATION */
/*     XNEW@D=XOLD+ALPHA*P */
/*     WHERE   XOLD IS THE CURRENT POINT */
/*             P    IS THE SEARCH DIRECTION */
/*     ALPHA IS CLOSE TO THE SOLUTION OF THE PROBLEM */
/*             MINIMIZE  PSI(ALPHA) */
/*     WITH THE RESTRICTION  0<ALFLOW<=ALPHA<=ALFUPP */
/*     HOWEVER, IF WE ARE FORCED TO TAKE A PURE GOLDSTEIN-ARMIJO STEP */
/*     THE LOWER BOUND CAN BE SLIGHTLY DECREASED */

/*     PSI(ALPHA)=0.5*(IIF(XOLD+ALPHA*P)II**2+ */
/*                +SIGMA(W(I)*H (XOLD+ALPHA*P)**2) */
/*                  I          I */

/*     WHERE I BELONG TO THE CURRENT WORKING SET */

/*     F(X)= (F1(X),F2(X),.........,FM(X)) TRANSPOSE */
/*     H(X)= (H1(X),H2(X),............,HL(X)) TRANSPOSE */

/*     ON ENTRY@D */
/*     XOLD(N)      REAL ARRAY OF LENGTH N-THE CURRENT POINT.CHANGED */
/*                  ON RETURN */
/*     P(N)         REAL ARRAY OF LENGTH N-SEARCH DIRECTION */
/*     F(M+L)       REAL ARRAY OF LENGTH M+L -F(1)....F(M) CONTAIN THE */
/*                  VALUE OF F(X) AT THE CURRENT POINT.CHANGED ON RETURN */
/*     V1(M+L)      REAL ARRAY 0F LENGTH M+L CONTAINING THE COMPOUND */
/*                  VECTOR (C*P) WHERE C AND A ARE THE JACOBIANS */
/*                         (AHAT*P) */
/*     M            INTEGER-NO. OF FUNCTIONS IN F(X)=F1(X).......,FM(X) */
/*     N            INTEGER-NO. OF UNKNOWNS */
/*     ALPHA        REAL-THE DISIRED STEPLENGTH.CHANGED ON EXIT */
/*     PSIZER       REAL-PSI(ALPHA) AT ALPHA=0 */
/*     DPSIZE       REAL SCALAR CONTAINING THE DERIVATIVE OF PSI(ALPHA) */
/*                  AT ALPHA=0 */
/*     ALFLOW       REAL-FIX LOWER BOUND OF THE STEPLENGTH */
/*     FFUNC        SUBROUTINE NAME-USER WRITTEN SUBROUTINE WHICH */
/*                  EVALUATES THE FUNCTION F(X) */
/*     HFUNC        SUBROUTINE NAME-USER WRITTEN SUBROUTINE WHICH */
/*                  EVALUATES THE CONSTRAINTS H(X) */
/*                  BOTH FFUNC AND HFUNC MUST BE WRITTEN AS FOLLOWS */
/*                  (WITH THE OBVIOUS CHANGES FOR HFUNC) */

/*                  SUBROUTINE FFUNC(X,N,F,M,CTRL,C,MDC) */
/*                  INTEGER N,M,CTRL,MDC */
/*                  REAL X(N),F(M),C(MDC,N) */
/*                  ----------------------- */
/*                  CTRL CAN HAVE 3 DIFFERENT VALUES ON ENTRY */
/*         CTRL= 1  MEANS EVALUATE THE FUNCTIONS AT THE POINT X AND */
/*                  RETURN THIS VECTOR IN THE ARRAY F IF THE FUNCTIONS */
/*                  ARE COMPUTABLE. */
/*                  ON RETURN THE USER CAN INDICATE UNCOMPUTABILITY BY */
/*                  SETTING CTRL=-1 */
/*                  DO NOT ALTER ARRAY X. */
/*         CTRL=-1  MEANS EVALUATE THE FUNCTIONS AT THE POINT X AND */
/*                  RETURN THIS VECTOR IN THE ARRAY F IF THE FUNCTIONS */
/*                  ARE COMPUTABLE. DO NOT ALTER ARRAY X. */
/*                  POSSIBLE UNCOMPUTABILITY OF THE FUNCTIONS MUST BE */
/*                  INDICATED BY SETTING CTRL TO A VALUE <-10 ON RETURN */
/*         CTRL= 2  MEANS CALCULATE THE JACOBIAN OF F(X) AT THE POINT X */
/*                  AND RETURN THIS MATRIX IN THE ARRAY C IF THE JACOBIAN */
/*                  IS SUPPLIED ANALYTICALLY. */
/*                  POSSIBLE UNCOMPUTABILITY OF THE JACOBIAN MUST BE */
/*                  INDICATED BY SETTING CTRL TO A VALUE <-10 ON RETURN */
/*                  IF THE USER WANTS THE JACOBIAN BEING COMPUTED */
/*                  NUMERICALLY THAT SHOULD BE INDICATED BY SETTING */
/*                  CTRL=0 ON RETURN. */
/*                  DO NOT ALTER ARRAYS X AND F. */
/*                  ------------------------------ */
/*                  RETURN */
/*                  END */
/*     H(L)         REAL ARRAY OF LENGTH L-VALUE OF THE CONSTRAINTS AT */
/*                  THE CURRENT POINT.CHANGED ON EXIT */
/*     ACTIVE(T)    INTEGER ARRAY OF LENGTH T=INDECIES FOR THE T ACTIVE */
/*                  CONSTRAINTS */
/*     T            INTEGER-NO. OF ACTIVE CONSTRAINTS */
/*     INACT(Q)     INTEGER ARRAY OF LENGTH Q -INDECEC FOR THE INACTIVE */
/*                  CONSTRAINTS */
/*     Q            INTEGER - NO. OF INACTIVE CONSTRAINTS */
/*     L            INTEGER-TOTAL NO. OF CONSTRAINTS */
/*     ALFUPP       REAL-FIX UPPER BOUND OF THE STEPLENGTH DUE TO ANY */
/*                  INACTIVE CONSTRAINT */

/*     ON RETURN@D */
/*     XOLD(N)      THE NEW POINT */
/*     F(M)         THE VALUE OF F(X) AT THE NEW POINT */
/*     ALPHA        THE COMPUTED STEPLENGTH */
/*     H(L)         THE VALUE OF THE CONSTRAINTS AT THE NEW POINT */
/*     PSIALF       REAL-PSI(ALPHA) AT THE NEW POINT */
/*     XDIFF        REAL-THE 2-NORM  II XOLD-XNEW II */
/*     EXIT         INTEGER-  =-1 IF THE SEARCH DIRECTION IS NOT A */
/*                                DESCENT DIRECTION. */
/*                            =-2 IF PMAX*ALPHA<DSQRT(DRELPR) */
/*                                  OR ALPHA<ALFLOW */
/*                            =-3 IF FIRST GUESS OF STEPLENGTH MAKES THE */
/*                                RESIDUALS OR THE CONSTRAINTS */
/*                                UNCOMPUTABLE */
/*                            <-10 AS A USER STOP INDICATOR */
/*                            = 0  OTHERWISE */
/*     EVAL    INTEGER- THE NO. OF EVALUATIONS OF THE RESIDUALS INSIDE */
/*                   THIS ROUTINE */

/*     WORKING AREAS@D */
/*     FNEW(M+L)    REAL ARRAY OF LENGTH M+L */
/*     HNEW(L)      REAL ARRAY OF LENGTH L */
/*     V2(M+L)      REAL ARRAY OF LENGTH M+L */
/*     G(N)         REAL ARRAY OF LENGTH N */

/*     INTERNAL VARIABLES */

/*      write(10,*) 'In LINEC' */
/*      write(10,*) 'alpha,psizer,dpsize: ',alpha,psizer,dpsize */
    /* Parameter adjustments */
    --f;
    --v1;
    --g;
    --p;
    --xold;
    --h__;
    --hnew;
    --active;
    --inact;
    --w;
    --fnew;
    --v2;

    /* Function Body */
    k = 0;
    *xdiff = 0.;
    psikm1 = *psizer;
    mpt = *m + *t;
    mpl = *m + *l;

/*     SET VALUES OF CONSTANTS ETA,TAU,GAMMA */
/*     COMPUTE ALFMIN,ALFMAX,ALFK */

    linc1_(&p[1], n, alpha, alflow, alfupp, &eta, &tau, &gamma, &alfmin, &
	    alfmax, &alfk, &pmax);
    *exit = 0;
    alfkm1 = alfk;
/*      write(10,*) 'alfk= ',alfk */

/*     COMPUTE PSIK=PSI(ALF0) AND TEST UNCOMPUTABILITY */

    ctrl = 1;
    psik = psi_(&xold[1], &p[1], n, &alfk, &g[1], &fnew[1], m, (U_fp)ffunc, &
	    hnew[1], t, l, &active[1], (U_fp)hfunc, &w[1], &ctrl);
/*      write(10,*) 'After PSI-call. alfk= ',alfk */
    ++k;
    if (-1 == ctrl) {
	*exit = -3;
    }
    if (*exit < 0) {
	goto L1020;
    }
    diff = *psizer - psik;

/*     COMPUTE THE VECTOR V2 SO THAT A ONE DIMENSIONAL */
/*     MINIMIZATION IN R(M) CAN BE DONE */

    linc2_(m, n, &v1[1], &fnew[1], &f[1], &alfk, &h__[1], &hnew[1], t, &
	    active[1], &w[1], &inact[1], q, l, &v2[1]);

/*     SET XMIN = THE BEST OF THE POINTS 0 AND ALF0 */

    if (diff >= 0.) {
	xmin = alfk;
    }
    if (diff < 0.) {
	xmin = 0.;
    }

/*     MINIMIZE IN R(M). USE TWO POINTS @D 0 AND ALF0 */
/*     NEW SUGGESTION OF STEPLENGTH IS ALFKP1 */
/*     PK IS THE VALUE OF THE APPROXIMATING FUNCTION AT ALFKP1 */

    minrm_(&f[1], &v1[1], &v2[1], &mpl, &alfmin, &alfmax, &xmin, &alfkp1, &pk,
	     &beta, &pbeta);
/*      write(10,*) 'After MINRM-call. alfkp1= ',alfkp1 */
    if (alfkp1 == beta) {
	goto L20;
    }
    if (pk <= pbeta) {
	goto L20;
    }
    if (beta > alfk) {
	goto L20;
    }
    alfkp1 = beta;
    pk = pbeta;
L20:
    alfkm1 = 0.;
    psikm1 = *psizer;
    update_(&alfkm2, &psikm2, &alfkm1, &psikm1, &alfk, &psik, &alfkp1);

/*     TEST TERMINATION CONDITION AT ALPHA = ALF0 */

/*      write(10,*) 'diff,tau,dpsize,alfkm1,psikm1,gamma,psizer=' */
/*      write(10,*) diff,tau,dpsize,alfkm1,psikm1,gamma,psizer */
    if (! (-diff <= tau * *dpsize * alfkm1 || psikm1 < gamma * *psizer)) {
	goto L100;
    }
/*      write(10,*) 'Armijo satisfied at alf0' */

/*     TERMINATION CONDITION SATISFIED AT ALPHA = ALF0 */

L30:
    diff = *psizer - psik;

/*     CHECK IF ESSENTIAL REDUCTION IS LIKELY */

    redc_(&alfkm1, &psikm1, &alfk, &pk, &diff, &eta, &xold[1], &p[1], &f[1], &
	    g[1], &fnew[1], m, n, (U_fp)ffunc, &h__[1], &hnew[1], (U_fp)hfunc,
	     t, l, &active[1], &w[1], &k, &psik, &reduce);
    if (k < -10) {
	goto L1030;
    }
    if (! reduce) {
	goto L1000;
    }
/*      write(10,*) 'Essential reduction is likely' */

/*     THE VALUE OF THE OBJECTIVE FUNCTION CAN MOST LIKELY BE REDUCED */

/*     MINIMIZE IN R(N). USE THREE POINTS@D ALFKM2,ALFKM1,ALFK */
/*     NEW SUGGESTION OF THE STEPLENGTH IS ALFKP1 */
/*     PK IS THE VALUE OF THE APPROXIMATING FUNCTION AT ALFKP1 */

    minrn_(&alfk, &psik, &alfkm1, &psikm1, &alfkm2, &psikm2, &alfmin, &alfmax,
	     &pmax, &machin_1.drelpr, &alfkp1, &pk);
    update_(&alfkm2, &psikm2, &alfkm1, &psikm1, &alfk, &psik, &alfkp1);
    goto L30;
L100:

/*     TERMINATION CONDITION NOT SATISFIED AT ALPHA = ALF0 */

/*     COMPUTE PSIK=PSI(ALF1) */

    ctrl = -1;
    psik = psi_(&xold[1], &p[1], n, &alfk, &g[1], &fnew[1], m, (U_fp)ffunc, &
	    hnew[1], t, l, &active[1], (U_fp)hfunc, &w[1], &ctrl);
    if (ctrl < -10) {
	k = ctrl;
    }
    if (k < 0) {
	goto L1030;
    }

/*     TEST TERMINATION CONDITION AT ALPHA = ALF1 */

    diff = *psizer - psik;
    if (! (-diff <= tau * *dpsize * alfk || psik < gamma * *psizer)) {
	goto L200;
    }


/*     TERMINATION CONDITION SATISFIED AT ALPHA = ALF1 */

/*     CHECK IF ALF0 IS SOMEWHAT GOOD */

    if (*psizer > psikm1) {
	goto L120;
    }

/*     MINIMIZE IN R(M). USE TWO POINTS@D 0 AND ALF1 */
/*     NEW SUGGESTION OF STEPLENGTH IS ALFKP1 */
/*     PK IS THE VALUE OF THE APPROXIMATING FUNCTION AT ALFKP1 */

    xmin = alfk;
    concat_(&fnew[1], m, &hnew[1], &active[1], t, &inact[1], q, &w[1]);
    i__1 = mpl;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v2[i__] = ((fnew[i__] - f[i__]) / alfk - v1[i__]) / alfk;
/* L110: */
    }
    minrm_(&f[1], &v1[1], &v2[1], &mpl, &alfmin, &alfmax, &xmin, &alfkp1, &pk,
	     &beta, &pbeta);
    if (alfkp1 == beta) {
	goto L115;
    }
    if (pk <= pbeta) {
	goto L115;
    }
    if (beta > alfk) {
	goto L115;
    }
    alfkp1 = beta;
    pk = pbeta;
L115:
    alfkm1 = 0.;
    psikm1 = *psizer;
    goto L130;
L120:

/*     MINIMIZE IN R(N). USE THREE POINTS  0,ALF0 AND ALF1 */
/*     NEW SUGGESTION OF THE STEPLENGTH IS ALFKP1 */
/*     PK IS THE VALUE OF THE APPROXIMATING FUNCTION AT ALFKP1 */

    minrn_(&alfk, &psik, &alfkm1, &psikm1, &alfkm2, &psikm2, &alfmin, &alfmax,
	     &pmax, &machin_1.drelpr, &alfkp1, &pk);
L130:
    ++k;
L140:
    diff = *psizer - psik;
    update_(&alfkm2, &psikm2, &alfkm1, &psikm1, &alfk, &psik, &alfkp1);

/*     CHECK IF ESSENTIAL REDUCTION IS LIKELY */

    redc_(&alfkm1, &psikm1, &alfk, &pk, &diff, &eta, &xold[1], &p[1], &f[1], &
	    g[1], &fnew[1], m, n, (U_fp)ffunc, &h__[1], &hnew[1], (U_fp)hfunc,
	     t, l, &active[1], &w[1], &k, &psik, &reduce);
    if (k < -10) {
	goto L1030;
    }
    if (! reduce) {
	goto L1000;
    }

/*     MINIMIZE IN R(N). USE THREE POINTS@D ALFKM2,ALFKM1 AND ALFK */
/*     NEW SUGGESTION OF STEPLENGTH IS ALFKP1 */
/*     PK IS THE VALUE OF THE APPROXIMATING FUNCTION AT ALFKP1 */

    minrn_(&alfk, &psik, &alfkm1, &psikm1, &alfkm2, &psikm2, &alfmin, &alfmax,
	     &pmax, &machin_1.drelpr, &alfkp1, &pk);
    goto L140;
L200:

/*     TAKE A PURE GOLDSTEIN-ARMIJO STEP */

    ++k;
    gac_(&xold[1], &p[1], &f[1], m, n, (U_fp)ffunc, (U_fp)hfunc, &h__[1], &
	    active[1], t, l, &w[1], &k, &alfmin, exit, &g[1], &fnew[1], 
	    psizer, dpsize, &alfk, &psik, &tau, &pmax, &machin_1.drelpr);
    if (k < -10) {
	goto L1030;
    }
    if (-2 == *exit) {
	chder_(dpsize, psizer, &xold[1], &p[1], m, n, (U_fp)ffunc, &active[1],
		 t, &w[1], l, (U_fp)hfunc, &k, exit, &g[1], &fnew[1], &hnew[1]
		, &alfk, &psik);
    }
    if (k < -10) {
	goto L1030;
    }
    alfkm1 = alfk;
    psikm1 = psik;
L1000:
/*      write(10,*) 'Just after 1000 continue' */

/*     COMPUTE THE NEW POINT AND THE DIFFERENCE II XOLD-XNEW II */

    diff = 0.;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	xel = alfkm1 * p[i__];
	xold[i__] += xel;
	g[i__] = xel;
/* L1010: */
    }
    *xdiff = enlsip_dnrm2_(n, &g[1], &c__1);
L1020:
    *alpha = alfkm1;
    *psialf = psikm1;
    *eval = k;
    return 0;
L1030:

/*     A USER STOP INDICATOR IS DETECTED */

    *exit = k;
    return 0;
} /* linec_ */

/* CONCAT */
/* Subroutine */ int concat_(doublereal *f, integer *m, doublereal *h__, 
	integer *active, integer *t, integer *inact, integer *q, doublereal *
	w)
{
    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer i__, j, k;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --f;
    --h__;
    --active;
    --inact;
    --w;

    /* Function Body */
    if (*t == 0) {
	goto L20;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *m + i__;
	k = active[i__];
	f[j] = sqrt(w[k]) * h__[k];
/* L10: */
    }
L20:
    if (*q <= 0) {
	return 0;
    }
    i__1 = *q;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *m + *t + i__;
	k = inact[i__];
	f[j] = 0.;
	if (h__[k] <= 0.) {
	    f[j] = sqrt(w[k]) * h__[k];
	}
/* L30: */
    }
    return 0;
} /* concat_ */

/* LINC2 */
/* Subroutine */ int linc2_(integer *m, integer *n, doublereal *v1, 
	doublereal *fnew, doublereal *f, doublereal *alfk, doublereal *h__, 
	doublereal *hnew, integer *t, integer *active, doublereal *w, integer 
	*inact, integer *q, integer *l, doublereal *v2)
{
    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer i__, j, k;
    extern /* Subroutine */ int concat_(doublereal *, integer *, doublereal *,
	     integer *, integer *, integer *, integer *, doublereal *);
    static integer mpi, mpl;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v2;
    --inact;
    --w;
    --active;
    --hnew;
    --h__;
    --f;
    --fnew;
    --v1;

    /* Function Body */
    concat_(&f[1], m, &h__[1], &active[1], t, &inact[1], q, &w[1]);
    concat_(&fnew[1], m, &hnew[1], &active[1], t, &inact[1], q, &w[1]);
    if (*t <= 0) {
	goto L20;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	mpi = *m + i__;
	j = active[i__];
	v1[mpi] = sqrt(w[j]) * v1[mpi];
/* L10: */
    }
L20:
    if (*q <= 0) {
	goto L40;
    }
    i__1 = *q;
    for (i__ = 1; i__ <= i__1; ++i__) {
	mpi = *m + *t + i__;
	k = inact[i__];
	if (h__[k] <= 0.) {
	    v1[mpi] = sqrt(w[k]) * v1[mpi];
	}
	if (h__[k] > 0.) {
	    v1[mpi] = 0.;
	}
/* L30: */
    }
L40:
    mpl = *m + *l;
    i__1 = mpl;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v2[i__] = ((fnew[i__] - f[i__]) / *alfk - v1[i__]) / *alfk;
/* L60: */
    }
    return 0;
} /* linc2_ */

/* REDC */
/* Subroutine */ int redc_(doublereal *alf, doublereal *psialf, doublereal *
	alfk, doublereal *pk, doublereal *diff, doublereal *eta, doublereal *
	xold, doublereal *p, doublereal *f, doublereal *xnew, doublereal *
	fnew, integer *m, integer *n, U_fp ffunc, doublereal *h__, doublereal 
	*hnew, U_fp hfunc, integer *t, integer *l, integer *active, 
	doublereal *w, integer *k, doublereal *psik, logical *reduce)
{
    /* Initialized data */

    static doublereal delta = .2;

    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer ctrl, i__;
    extern doublereal psi_(doublereal *, doublereal *, integer *, doublereal *
	    , doublereal *, doublereal *, integer *, U_fp, doublereal *, 
	    integer *, integer *, integer *, U_fp, doublereal *, integer *);


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --fnew;
    --f;
    --xnew;
    --p;
    --xold;
    --h__;
    --hnew;
    --active;
    --w;

    /* Function Body */

/*     REDUCE IS SET TO TRUE IF ESSENTIAL REDUCTION IN THE */
/*     OBJECTIVE FUNCTION IS LIKELY */
/*     OTHERWISE REDUCE IS SET TO FALSE */

    if (*psialf - *pk < *eta * *diff) {
	goto L40;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	f[i__] = fnew[i__];
/* L10: */
    }
    if (*l == 0) {
	goto L30;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	h__[i__] = hnew[i__];
/* L20: */
    }
L30:
    ctrl = -1;
    *psik = psi_(&xold[1], &p[1], n, alfk, &xnew[1], &fnew[1], m, (U_fp)ffunc,
	     &hnew[1], t, l, &active[1], (U_fp)hfunc, &w[1], &ctrl);
    if (ctrl < -10) {
	*k = ctrl;
    }
    if (*k < 0) {
	return 0;
    }
    ++(*k);
    *reduce = TRUE_;
    if (! (*psialf - *psik < *eta * *diff && *psik > delta * *psialf)) {
	return 0;
    }

/*     TERMINATE BUT CHOOSE THE BEST POINT OUT OF ALF AND ALFK */

    if (*psialf <= *psik) {
	goto L70;
    }
    *alf = *alfk;
    *psialf = *psik;
L40:
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	f[i__] = fnew[i__];
/* L50: */
    }
    if (*l == 0) {
	goto L70;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	h__[i__] = hnew[i__];
/* L60: */
    }
L70:
    *reduce = FALSE_;
    return 0;
} /* redc_ */

/* GAC */
/* Subroutine */ int gac_(doublereal *xold, doublereal *p, doublereal *f, 
	integer *m, integer *n, U_fp ffunc, U_fp hfunc, doublereal *h__, 
	integer *active, integer *t, integer *l, doublereal *w, integer *k, 
	doublereal *alfmin, integer *exit, doublereal *xnew, doublereal *fnew,
	 doublereal *psi0, doublereal *dpsi0, doublereal *u, doublereal *psiu,
	 doublereal *tau, doublereal *pmax, doublereal *drelpr)
{
    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer ctrl;
    static doublereal psix, x, sqrrel;
    extern doublereal psi_(doublereal *, doublereal *, integer *, doublereal *
	    , doublereal *, doublereal *, integer *, U_fp, doublereal *, 
	    integer *, integer *, integer *, U_fp, doublereal *, integer *);


/*     THIS IS A ROUTINE FOR CONSTRAINED LEAST SQUARES PROBLEMS */
/*     THAT HALFES THE VALUE OF U UNTIL A GOLDSTEIN-ARMIJO */
/*     CONDITION IS SATISFIED OR UNTIL THE STEPLENGTH TIMES THE */
/*     NORM OF THE SEARCH DIRECTION IS BELOW DSQRT(REL.PREC.) */

/*     PSI(ALPHA)=0.5*(II F(XOLD+ALPHA*P) II**2+ */
/*                SIGMA(W(I)*H (XOLD+ALPHA*P)**2) */
/*                  I          I */

/*     WHERE I BELONGS TO THE CURRENT WORKING SET */
/*     CHOOSE ALPHA=X SO THAT */
/*                  PSI(X)<=PSI(0)+TAU*X*DPSI(0)   (1) */
/*     WE KNOW THAT PSI(U)>PSI(0)+TAU*U*DPSI(0) */
/*     THE SIMPLEST WE CAN DO IS TO SET   U=U*0.5 AND */
/*     TEST IF CONDITION  (1) IS SATISFIED FOR X=U */


/*     INTERNAL VARIABLES */
    /* Parameter adjustments */
    --fnew;
    --f;
    --xnew;
    --p;
    --xold;
    --h__;
    --active;
    --w;

    /* Function Body */
    ctrl = -1;
    sqrrel = sqrt(*drelpr);
    psix = *psi0;
    x = *u;
L10:
    if (*pmax * x < sqrrel || x <= *alfmin) {
	*exit = -2;
    }
    if (-2 == *exit) {
	goto L20;
    }
    x *= .5;
    psix = psi_(&xold[1], &p[1], n, &x, &xnew[1], &f[1], m, (U_fp)ffunc, &h__[
	    1], t, l, &active[1], (U_fp)hfunc, &w[1], &ctrl);
    if (ctrl < -10) {
	*k = ctrl;
    }
    if (*k < 0) {
	return 0;
    }
    ++(*k);
    if (psix > *psi0 + *tau * x * *dpsi0) {
	goto L10;
    }
L20:
    *u = x;
    *psiu = psix;
    return 0;
} /* gac_ */

/* LINC1 */
/* Subroutine */ int linc1_(doublereal *p, integer *n, doublereal *alpha, 
	doublereal *alflow, doublereal *alfupp, doublereal *eta, doublereal *
	tau, doublereal *gamma, doublereal *alfmin, doublereal *alfmax, 
	doublereal *alfk, doublereal *pmax)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    static doublereal absp;
    static integer i__;


/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --p;

    /* Function Body */
    *eta = .3;
    *tau = .25;
    *gamma = .4;
    *alfmax = *alfupp;
    *alfmin = *alflow;
    *alfk = min(*alpha,*alfmax);
    *pmax = 0.;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	absp = (d__1 = p[i__], abs(d__1));
	if (absp > *pmax) {
	    *pmax = absp;
	}
/* L10: */
    }
    return 0;
} /* linc1_ */

/* MINRN */
/* Subroutine */ int minrn_(doublereal *x, doublereal *fx, doublereal *w, 
	doublereal *fw, doublereal *v, doublereal *fv, doublereal *alfmin, 
	doublereal *alfmax, doublereal *pmax, doublereal *drelpr, doublereal *
	u, doublereal *pu)
{
    /* System generated locals */
    doublereal d__1, d__2, d__3;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal t1, t2, t3;
    extern /* Subroutine */ int quamin_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *);
    static doublereal eps;


/*     INTERNAL VARIABLES */

    eps = sqrt(*drelpr) / *pmax;
    *u = *x;
    *pu = *fx;
    if ((d__1 = *v - *x, abs(d__1)) < eps || (d__2 = *w - *x, abs(d__2)) < 
	    eps || (d__3 = *w - *v, abs(d__3)) < eps) {
	return 0;
    }
    quamin_(x, fx, w, fw, v, fv, u);
    *u = min(*u,*alfmax);
    *u = max(*u,*alfmin);
    t1 = (*u - *x) * (*u - *v) * *fw / (*w - *x) / (*w - *v);
    t2 = (*u - *w) * (*u - *v) * *fx / (*x - *w) / (*x - *v);
    t3 = (*u - *w) * (*u - *x) * *fv / (*v - *x) / (*v - *w);
    *pu = t1 + t2 + t3;
    return 0;
} /* minrn_ */

/* UPDATE */
/* Subroutine */ int update_(doublereal *x, doublereal *fx, doublereal *w, 
	doublereal *fw, doublereal *v, doublereal *fv, doublereal *u)
{
    *x = *w;
    *fx = *fw;
    *w = *v;
    *fw = *fv;
    *v = *u;
    return 0;
} /* update_ */

/* MINRM1 */
/* Subroutine */ int minrm1_(doublereal *v0, doublereal *v1, doublereal *v2, 
	integer *m, doublereal *sqnv0, doublereal *sqnv1, doublereal *sqnv2, 
	doublereal *scv0v1, doublereal *scv0v2, doublereal *scv1v2)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__;
    extern /* Subroutine */ int scalv_(doublereal *, doublereal *, integer *);
    static doublereal v0norm, v1norm, v2norm, sc1, sc2, sc3;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v2;
    --v1;
    --v0;

    /* Function Body */
    v0norm = enlsip_dnrm2_(m, &v0[1], &c__1);
    v1norm = enlsip_dnrm2_(m, &v1[1], &c__1);
    v2norm = enlsip_dnrm2_(m, &v2[1], &c__1);
/* Computing 2nd power */
    d__1 = v0norm;
    *sqnv0 = d__1 * d__1;
/* Computing 2nd power */
    d__1 = v1norm;
    *sqnv1 = d__1 * d__1;
/* Computing 2nd power */
    d__1 = v2norm;
    *sqnv2 = d__1 * d__1;
    if (v0norm != 0.) {
	scalv_(&v0[1], &v0norm, m);
    }
    if (v1norm != 0.) {
	scalv_(&v1[1], &v1norm, m);
    }
    if (v2norm != 0.) {
	scalv_(&v2[1], &v2norm, m);
    }
    sc1 = 0.;
    sc2 = 0.;
    sc3 = 0.;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	sc1 += v0[i__] * v1[i__];
	sc2 += v0[i__] * v2[i__];
	sc3 += v1[i__] * v2[i__];
/* L10: */
    }
    *scv0v1 = sc1 * v0norm * v1norm;
    *scv0v2 = sc2 * v0norm * v2norm;
    *scv1v2 = sc3 * v1norm * v2norm;
    if (v0norm != 0.) {
	d__1 = 1. / v0norm;
	scalv_(&v0[1], &d__1, m);
    }
    if (v1norm != 0.) {
	d__1 = 1. / v1norm;
	scalv_(&v1[1], &d__1, m);
    }
    if (v2norm != 0.) {
	d__1 = 1. / v2norm;
	scalv_(&v2[1], &d__1, m);
    }
    return 0;
} /* minrm1_ */

/* MINRM2 */
/* Subroutine */ int minrm2_(doublereal *sqnv1, doublereal *sqnv2, doublereal 
	*scv0v1, doublereal *scv0v2, doublereal *scv1v2, doublereal *p, 
	doublereal *q, doublereal *delta, doublereal *a1div3)
{
    /* System generated locals */
    doublereal d__1, d__2;

    /* Local variables */
    static doublereal a1, a2, a3;


/*     INTERNAL VARIABLES */

    a1 = *scv1v2 * 1.5 / *sqnv2;
    a2 = (*sqnv1 + *scv0v2 * 2.) * .5 / *sqnv2;
    a3 = *scv0v1 * .5 / *sqnv2;
    *p = a2 - a1 * a1 / 3.;
    *q = a3 - a1 * a2 / 3. + a1 * 2. * a1 * a1 / 27.;
/* Computing 2nd power */
    d__1 = *q / 2.;
/* Computing 3rd power */
    d__2 = *p / 3.;
    *delta = d__1 * d__1 + d__2 * (d__2 * d__2);
    *a1div3 = a1 / 3.;
    return 0;
} /* minrm2_ */

/* MINRM */
/* Subroutine */ int minrm_(doublereal *v0, doublereal *v1, doublereal *v2, 
	integer *m, doublereal *alfmin, doublereal *alfmax, doublereal *xmin, 
	doublereal *x, doublereal *px, doublereal *y, doublereal *py)
{
    /* Initialized data */

    static doublereal eps = 1e-4;

    /* System generated locals */
    doublereal d__1;

    /* Local variables */
    static doublereal beta;
    extern /* Subroutine */ int oner_(doublereal *, doublereal *, doublereal *
	    , doublereal *), twor_(doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *);
    static doublereal d__, sqnv0, sqnv1, sqnv2;
    static integer k;
    static doublereal p, q, a1div3, delta, pbiss, scv0v1, b2, b3, scv0v2, 
	    pprim, h0, error, scv1v2, x0, x1, x2, x3;
    extern /* Subroutine */ int minrm1_(doublereal *, doublereal *, 
	    doublereal *, integer *, doublereal *, doublereal *, doublereal *,
	     doublereal *, doublereal *, doublereal *), minrm2_(doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *);
    static doublereal dm, hm;
    extern /* Subroutine */ int choose_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, doublereal *, doublereal *, doublereal *)
	    ;
    extern doublereal pol3_(doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *), pol4_(doublereal *, doublereal *, 
	    doublereal *, integer *, doublereal *);


/*     A SUBROUTINE WHICH FINDS THE POINT X WHERE THE FUNCTION */
/*     P(X)= 0.5*II V0+V1*X+V2*X**2 II**2  IS MINIMIZED. */
/*     V0,V1 AND V2 BELONG TO R(M) AND X IS A SCALAR */
/*     THE VALUES OF V0,V1 AND V2 ARE BASED ON TW0 FUNCTION */
/*     VALUES @D  F(0) AND F(ALPHA) */
/*     THE FUNCTION P(X) IS ALWAYS >=0 AND IT IS A POLYNOMIAL */
/*     OF 4@DTH DEGREE */
/*     THE MINIMUM VALUE OF P(X) IS ATTAINED AT A POINT X WHERE */
/*     THE FIRST DERIVATIVE OF P(X)=DP(X) IS ZERO */
/*     DP(X) IS A POLYNOMIAL OF 3@DRD DEGREE */
/*     IN CASE OF THREE ROOTS (X1<X2<X3), X2 CORRESPONDS TO A LOCAL */
/*     MAXIMUM. CHOOSE THE ONE (OF X1 AND X3) THAT IS AT THE SAME */
/*     SIDE OF THE MAXIMUM AS XMIN IS PROVIDED NO EXTRAPOLATION */
/*     IS DONE */
/*     HOWEVER,THE MINIMUM POINT X MUST LIE IN THE INTERVALL */
/*     (ALFMIN,ALFMAX) */
/*     PX IS SET TO P(X) AT THE MINIMUM POINT */
/*     WHERE P(X) IS THE POLYNOMIAL ABOVE */
/*     Y IS SET TO THE OTHER MINIMIZER OF P(X)  (IF TWO ARE DETERMINED) */
/*     OTHERWISE Y IS SET TO THE SAME AS X IS SET */
/*     PY = P(Y) */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v2;
    --v1;
    --v0;

    /* Function Body */

/*     COMPUTE NORMS SQUARED AND SCALAR PRODUCTS */

    minrm1_(&v0[1], &v1[1], &v2[1], m, &sqnv0, &sqnv1, &sqnv2, &scv0v1, &
	    scv0v2, &scv1v2);
    beta = scv0v2 * 2. + sqnv1;
    b2 = scv1v2 * 3.;
    b3 = sqnv2 * 2.;
    pprim = pol3_(&scv0v1, &beta, &b2, &b3, xmin);
/* Computing 2nd power */
    d__1 = *xmin;
    pbiss = beta + scv1v2 * 6. * *xmin + sqnv2 * 6. * (d__1 * d__1);
    h0 = (d__1 = pprim / pbiss, abs(d__1));
    dm = (d__1 = scv1v2 * 6. + sqnv2 * 12. * *xmin, abs(d__1)) + h0 * 24. * 
	    sqnv2;

/*     DETERMINE IF DP(X)=0 SHALL BE SOLVED BY USING NEWTONS METHOD */

    hm = max(h0,1.);
    if (pbiss > hm * 20. * dm) {
	goto L40;
    }

/*     COMPUTE QUANTITIES P,Q,DELTA AND A1DIV3 SO THAT THE SOLUTION OF */
/*     X**3+A1*X**2+A2*X+A3=0    IS   X=T-A1/3 */
/*     WHERE T SOLVES     T**3+P*T+Q=0 */

    minrm2_(&sqnv1, &sqnv2, &scv0v1, &scv0v2, &scv1v2, &p, &q, &delta, &
	    a1div3);

/*     MATHEMATICALLY WE DESTINGWISH THREE DIFFERENT CASES */
/*     IF DELTA>0 THEN DF(X)=0 HAS ONE REAL ROOT */
/*     IF DELTA=0 THEN DF(X)=0 HAS ONE SINGLE AND ONE DOUBLE REAL ROOT */
/*     IF DELTA<0 THEN DF(X)=0 HAS THREE DIFFERENT REAL ROOTS */

/*     IF DELTA=0 THE ONLY ROOT OF INTEREST IS THE SINGLE ONE,SO */
/*     NUMERICALLY WE DISTINGWISH TWO CASES ,DELTA>=0 AND */
/*     DELTA<0 */

    if (delta < 0.) {
	goto L30;
    }

/*     DELTA>=0 ONE INTERESTING ROOT. X */

    oner_(&q, &delta, &a1div3, x);
    *y = *x;
    goto L100;
L30:

/*     DELTA<0  TWO INTERESTING ROOTS.  Y AND Z, Y<Z */

    twor_(&p, &q, &delta, &a1div3, &x1, &x2, &x3);

/*     CHOOSE X= X1 OR X2 OR X3 */

    choose_(&x1, &x2, &x3, xmin, &v0[1], &v1[1], &v2[1], m, x, y, py);
    goto L100;
L40:
    delta = 1.;

/*     ITERATE USING NEWTONS METHOD */

    k = 0;
    x0 = *xmin;
L50:
    pprim = pol3_(&scv0v1, &beta, &b2, &b3, &x0);
    pbiss = beta + scv1v2 * 6. * x0 + sqnv2 * 6. * x0 * x0;
    d__ = -pprim / pbiss;
    *x = x0 + d__;
    error = dm * 2. * d__ * d__ / abs(pbiss);
    x0 = *x;
    ++k;
    if (error > eps && k < 3) {
	goto L50;
    }
    *y = *x;
L100:

/*     MAKE THE MINIMUM POINT X LIE IN THE INTERVALL */
/*     (ALFMIN,ALFMAX) AND EVALUATE F(X) AT THE MINIMUM POINT */

    *x = min(*x,*alfmax);
    *x = max(*x,*alfmin);
    *px = pol4_(&v0[1], &v1[1], &v2[1], m, x);
    *y = min(*y,*alfmax);
    *y = max(*y,*alfmin);
    if (delta < 0.) {
	goto L110;
    }
    *y = *x;
    *py = *px;
L110:
    return 0;
} /* minrm_ */

/* ONER */
/* Subroutine */ int oner_(doublereal *q, doublereal *delta, doublereal *
	a1div3, doublereal *x)
{
    /* System generated locals */
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal), d_sign(doublereal *, doublereal *), pow_dd(
	    doublereal *, doublereal *);

    /* Local variables */
    static doublereal t, s1, s2, sqd, a3rd, arg1, arg2;


/*     INTERNAL VARIABLES */

    sqd = sqrt(*delta);
    arg1 = -(*q) / 2. + sqd;
    s1 = d_sign(&c_b303, &arg1);
    arg2 = -(*q) / 2. - sqd;
    s2 = d_sign(&c_b303, &arg2);
    a3rd = .33333333333333331;
    d__1 = abs(arg1);
    d__2 = abs(arg2);
    t = s1 * pow_dd(&d__1, &a3rd) + s2 * pow_dd(&d__2, &a3rd);
    *x = t - *a1div3;
    return 0;
} /* oner_ */

/* TWOR */
/* Subroutine */ int twor_(doublereal *p, doublereal *q, doublereal *delta, 
	doublereal *a1div3, doublereal *x1, doublereal *x2, doublereal *x3)
{
    /* System generated locals */
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal), atan(doublereal), cos(doublereal);

    /* Local variables */
    static doublereal t, tanfi, fi, pi, eps, sqd;


/*     INTERNAL VARIABLES */

    eps = 1e-8;
    sqd = sqrt(-(*delta));
    if (abs(*q) > eps * 2. * sqd) {
	goto L10;
    }
    fi = atan(1.) * 2.;
    goto L20;
L10:
    tanfi = (d__1 = sqd * 2. / *q, abs(d__1));
    fi = atan(tanfi);
L20:
    t = sqrt(-(*p) / 3.) * 2.;
    if (*q > 0.) {
	t = -t;
    }
    *x1 = t * cos(fi / 3.) - *a1div3;
    pi = atan(1.) * 4.;
    *x2 = t * cos((fi + pi * 2.) / 3.) - *a1div3;
    *x3 = t * cos((fi + pi * 4.) / 3.) - *a1div3;
    return 0;
} /* twor_ */

/* CHOOSE */
/* Subroutine */ int choose_(doublereal *x1, doublereal *x2, doublereal *x3, 
	doublereal *xmin, doublereal *v0, doublereal *v1, doublereal *v2, 
	integer *m, doublereal *root1, doublereal *root2, doublereal *proot2)
{
    /* System generated locals */
    doublereal d__1;

    /* Local variables */
    static doublereal x, y, z__;
    extern doublereal pol4_(doublereal *, doublereal *, doublereal *, integer 
	    *, doublereal *);


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v2;
    --v1;
    --v0;

    /* Function Body */
/* Computing MIN */
    d__1 = min(*x1,*x2);
    x = min(d__1,*x3);
/* Computing MAX */
    d__1 = max(*x1,*x2);
    z__ = max(d__1,*x3);
    if (*x1 <= *x2 && *x1 <= *x3) {
	y = min(*x2,*x3);
    }
    if (*x2 <= *x1 && *x2 <= *x3) {
	y = min(*x1,*x3);
    }
    if (*x3 <= *x1 && *x3 <= *x2) {
	y = min(*x1,*x2);
    }
    if (*xmin <= y) {
	*root1 = x;
    }
    if (*xmin <= y) {
	*root2 = z__;
    }
    if (*xmin > y) {
	*root1 = z__;
    }
    if (*xmin > y) {
	*root2 = x;
    }
    *proot2 = pol4_(&v0[1], &v1[1], &v2[1], m, root2);
    return 0;
} /* choose_ */

/* POL4 */
doublereal pol4_(doublereal *v0, doublereal *v1, doublereal *v2, integer *m, 
	doublereal *x)
{
    /* System generated locals */
    integer i__1;
    doublereal ret_val;

    /* Local variables */
    static integer i__;
    static doublereal p, s;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --v2;
    --v1;
    --v0;

    /* Function Body */
    s = 0.;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p = v0[i__] + *x * (v1[i__] + v2[i__] * *x);
	s += p * p;
/* L10: */
    }
    ret_val = s * .5;
    return ret_val;
} /* pol4_ */

/* POL3 */
doublereal pol3_(doublereal *a0, doublereal *a1, doublereal *a2, doublereal *
	a3, doublereal *x)
{
    /* System generated locals */
    doublereal ret_val;

    ret_val = *a0 + *x * (*a1 + *x * (*a2 + *a3 * *x));
    return ret_val;
} /* pol3_ */

/* PSI */
doublereal psi_(doublereal *xold, doublereal *p, integer *n, doublereal *alfk,
	 doublereal *xnew, doublereal *fnew, integer *m, S_fp ffunc, 
	doublereal *hnew, integer *t, integer *l, integer *active, S_fp hfunc,
	 doublereal *w, integer *ctrl)
{
    /* System generated locals */
    integer i__1;
    doublereal ret_val, d__1;

    /* Local variables */
    extern doublereal hsum_(integer *, integer *, doublereal *, doublereal *, 
	    integer *), enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, fctrl, hctrl;
    static doublereal dummy;


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --xnew;
    --p;
    --xold;
    --fnew;
    --hnew;
    --active;
    --w;

    /* Function Body */
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	xnew[i__] = xold[i__] + *alfk * p[i__];
/* L10: */
    }
    fctrl = *ctrl;
    (*ffunc)(&xnew[1], n, &fnew[1], m, &fctrl, &dummy, &c__1);
    hctrl = *ctrl;
    (*hfunc)(&xnew[1], n, &hnew[1], l, &hctrl, &dummy, &c__1);
    if (*ctrl == 1) {
	goto L20;
    }

/*     CTRL = -1 ON ENTRY */

    if (-1 == fctrl && -1 == hctrl) {
/* Computing 2nd power */
	d__1 = enlsip_dnrm2_(m, &fnew[1], &c__1);
	ret_val = (d__1 * d__1 + hsum_(&active[1], t, &hnew[1], &w[1], l)) * 
		.5;
    }
    if (fctrl < -10 || hctrl < -10) {
	*ctrl = min(fctrl,hctrl);
    }
    return ret_val;
L20:
    if (fctrl == 1 && hctrl == 1) {
/* Computing 2nd power */
	d__1 = enlsip_dnrm2_(m, &fnew[1], &c__1);
	ret_val = (d__1 * d__1 + hsum_(&active[1], t, &hnew[1], &w[1], l)) * 
		.5;
    }
    if (fctrl != 1 || hctrl != 1) {
	*ctrl = -1;
    }
    return ret_val;
} /* psi_ */

/* PERMIN */
/* Subroutine */ int permin_(integer *p, integer *n)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     INITIATE A PERMUTATION RECORD */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --p;

    /* Function Body */
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	p[i__] = i__;
/* L10: */
    }
    return 0;
} /* permin_ */

/* HESSF */
/* Subroutine */ int hessf_(S_fp ffunc, doublereal *b, integer *mdb, 
	doublereal *x, integer *n, doublereal *v, doublereal *f1, doublereal *
	f2, integer *m, integer *ier)
{
    /* System generated locals */
    integer b_dim1, b_offset, i__1, i__2, i__3;
    doublereal d__1;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static doublereal epsj, epsk;
    static integer ctrl;
    extern /* Subroutine */ int plus_(doublereal *, doublereal *, doublereal *
	    , integer *);
    static integer j, k, l;
    static doublereal dummy, xj, xk, athird, sum, eps1, eps2;


/*     COMPUTE THE N*N MATRIX             M */
/*                                 B @D= SIGMA(V *G ) */
/*                                       K=1   K  K */
/*     WHERE G   IS THE HESSIAN OF F (X) */
/*            K                     K */



/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    b_dim1 = *mdb;
    b_offset = 1 + b_dim1 * 1;
    b -= b_offset;
    --f2;
    --f1;
    --v;

    /* Function Body */
    *ier = 0;
    ctrl = -1;
    athird = .33333333333333331;
    eps2 = pow_dd(&machin_1.drelpr, &athird);
    eps1 = eps2;
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
	xk = x[k];
/* Computing MAX */
	d__1 = abs(xk);
	epsk = max(d__1,1.) * eps2;
	i__2 = k;
	for (j = 1; j <= i__2; ++j) {
	    xj = x[j];
/* Computing MAX */
	    d__1 = abs(xj);
	    epsj = max(d__1,1.) * eps1;
	    x[k] = xk + epsk;
	    x[j] += epsj;
	    (*ffunc)(&x[1], n, &f1[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk + epsk;
	    x[j] -= epsj;
	    (*ffunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    plus_(&f1[1], &c_b327, &f2[1], m);
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk - epsk;
	    x[j] += epsj;
	    (*ffunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    plus_(&f1[1], &c_b327, &f2[1], m);
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk - epsk;
	    x[j] -= epsj;
	    (*ffunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    plus_(&f1[1], &c_b303, &f2[1], m);
	    x[k] = xk;
	    x[j] = xj;
	    sum = 0.;
	    i__3 = *m;
	    for (l = 1; l <= i__3; ++l) {
		sum += f1[l] / (epsk * 4. * epsj) * v[l];
/* L10: */
	    }
	    b[k + j * b_dim1] = sum;
	    if (k == j) {
		goto L20;
	    }
	    b[j + k * b_dim1] = sum;
L20:
	    ;
	}
/* L30: */
    }
    return 0;

/*     A USER STOP INDICATOR IS DETECTED */

L40:
    *ier = ctrl;
    return 0;
} /* hessf_ */

/* HESSH */
/* Subroutine */ int hessh_(S_fp hfunc, doublereal *b, integer *mdb, 
	doublereal *x, integer *n, doublereal *v, integer *active, integer *t,
	 doublereal *f1, doublereal *f2, integer *m, integer *ier)
{
    /* System generated locals */
    integer b_dim1, b_offset, i__1, i__2, i__3;
    doublereal d__1;

    /* Builtin functions */
    double pow_dd(doublereal *, doublereal *);

    /* Local variables */
    static doublereal epsj, epsk;
    static integer ctrl, i__, j, k, l;
    static doublereal dummy;
    extern /* Subroutine */ int press_(doublereal *, integer *, integer *, 
	    doublereal *, doublereal *);
    static doublereal xj, xk, athird, sum, eps1, eps2;


/*     COMPUTE THE N*N MATRIX             T */
/*                            B =  B -  SIGMA(V *G ) */
/*                                       K=1   K  K */
/*     WHERE G   IS THE HESSIAN OF H (X) AND K IN CURRENT WORKING SET */
/*            K                     K */



/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    b_dim1 = *mdb;
    b_offset = 1 + b_dim1 * 1;
    b -= b_offset;
    --active;
    --v;
    --f2;
    --f1;

    /* Function Body */
    *ier = 0;
    ctrl = -1;
    athird = .33333333333333331;
    eps2 = pow_dd(&machin_1.drelpr, &athird);
    eps1 = eps2;
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
	xk = x[k];
/* Computing MAX */
	d__1 = abs(xk);
	epsk = max(d__1,1.) * eps2;
	i__2 = k;
	for (j = 1; j <= i__2; ++j) {
	    i__3 = *t;
	    for (i__ = 1; i__ <= i__3; ++i__) {
		f1[i__] = 0.;
/* L5: */
	    }
	    xj = x[j];
/* Computing MAX */
	    d__1 = abs(xj);
	    epsj = max(d__1,1.) * eps1;
	    x[k] = xk + epsk;
	    x[j] += epsj;
	    (*hfunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    press_(&f2[1], &active[1], t, &c_b303, &f1[1]);
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk + epsk;
	    x[j] -= epsj;
	    (*hfunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    press_(&f2[1], &active[1], t, &c_b327, &f1[1]);
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk - epsk;
	    x[j] += epsj;
	    (*hfunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    press_(&f2[1], &active[1], t, &c_b327, &f1[1]);
	    x[k] = xk;
	    x[j] = xj;
	    x[k] = xk - epsk;
	    x[j] -= epsj;
	    (*hfunc)(&x[1], n, &f2[1], m, &ctrl, &dummy, &c__1);
	    if (ctrl < -10) {
		goto L40;
	    }
	    press_(&f2[1], &active[1], t, &c_b303, &f1[1]);
	    x[k] = xk;
	    x[j] = xj;
	    sum = 0.;
	    i__3 = *t;
	    for (l = 1; l <= i__3; ++l) {
		sum += f1[l] / (epsk * 4. * epsj) * v[l];
/* L10: */
	    }
	    b[k + j * b_dim1] -= sum;
	    if (k == j) {
		goto L20;
	    }
	    b[j + k * b_dim1] = b[k + j * b_dim1];
L20:
	    ;
	}
/* L30: */
    }
    return 0;

/*     A USER STOP INDICATOR IS DETECTED */

L40:
    *ier = ctrl;
    return 0;
} /* hessh_ */

/* PRESS */
/* Subroutine */ int press_(doublereal *f2, integer *active, integer *t, 
	doublereal *factor, doublereal *f1)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, k;


/*     COMPUTE F1(I)@D= F1(I)+FACTOR*F2(K) */
/*     WHERE K BELONGS TO THE CURRENT WORKING SET */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --f2;
    --active;
    --f1;

    /* Function Body */
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	f1[i__] += *factor * f2[k];
/* L10: */
    }
    return 0;
} /* press_ */

/* PLUS */
/* Subroutine */ int plus_(doublereal *f1, doublereal *factor, doublereal *f2,
	 integer *m)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__;


/*     COMPUTE  F1@D= FACTOR*F2+F1 */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --f2;
    --f1;

    /* Function Body */
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	f1[i__] += *factor * f2[i__];
/* L10: */
    }
    return 0;
} /* plus_ */

/* CHDER */
/* Subroutine */ int chder_(doublereal *dpsize, doublereal *psizer, 
	doublereal *xold, doublereal *p, integer *m, integer *n, S_fp ffunc, 
	integer *active, integer *t, doublereal *w, integer *l, S_fp hfunc, 
	integer *k, integer *exit, doublereal *xnew, doublereal *fnew, 
	doublereal *hnew, doublereal *alfk, doublereal *psik)
{
    /* System generated locals */
    doublereal d__1, d__2, d__3;

    /* Local variables */
    static integer ctrl;
    static doublereal psimk, dpsiba, dpsice, maxdif, dpsifo;
    extern doublereal psi_(doublereal *, doublereal *, integer *, doublereal *
	    , doublereal *, doublereal *, integer *, S_fp, doublereal *, 
	    integer *, integer *, integer *, S_fp, doublereal *, integer *);


/*     MAKE A CONSISTENCY CHECK OF THE DERIVATIVE APPROXIMATION */
/*     BASED ON THE JACOBIAN MATRIX */
/*     THE FUNCTION UNDER CONSERN IS */
/*       PSI(ALPHA) = F(XOLD+ALPHA*P)+SIGMA(W(I)*H (XOLD+ALPHA*P)**2) */
/*                                      I         I */

/*     ON ENTRY */

/*     DPSIZE THE DERIVATIVE OF PSI(ALPHA) AT ALPHA=0 COMPUTED */
/*            BY P(TR)*G(XOLD), WHERE G(X) IS THE GRADIENT OF PSI(X) */
/*     PSIZER PSI(0) */
/*     XOLD() CONTAINS THE STARTING POINT OF THE LINESEARCH */
/*     P()    CONTAINS THE SEARCH DIRECTION */
/*     M      NO. OF RESIDUALS IN THE VECTOR VALUED FUNCTION F(X) */
/*     N      NO. OF UNKNOWNS */
/*     FFUNC  SUBROUTINE NAME FOR USER WRITTEN ROUTINE WHICH EVALUATES */
/*            THE RESIDUALS */
/*     ACTIVE() CONTAINS INDECES FOR CONSTRAINTS IN CURRENT WORKING SET */
/*     T      NO. OF CONSTRAINTS IN CURRENT WORKING SET */
/*     W()    CONTAINS PENALTY WEIGHTS CORRESPONDING TO CONSTRAINTS */
/*     L      TOTAL NO. OF CONSTRAINTS */
/*     HFUNC  SUBROUTINE NAME FOR USER WRITTEN ROUTINE WHICH EVALUATES */
/*            THE CONSTRAINTS */
/*     K      NO. OF FUNCTION EVALUATIONS DONE SO FAR IN THE LINESEARCH */
/*     EXIT   =-2 */
/*     ALFK   THE ALPHA VALUE FOR WHICH THE DIFFERENCES ARE COMPUTED */
/*     PSIK   PSI(ALFK) */

/*     ON RETURN */

/*     K      INCREASED BY 1 OR SET A VALUE <-10 TO */
/*            INDICATE A USER STOP */
/*     EXIT   SET = -1 IF INCONSISTENCY IS DETECTED */

/*     WORKING AREAS */

/*     XNEW() OF DIMENSION N */
/*     FNEW() OF DIMENSION M+L */
/*     HNEW() OF DIMENSION L */

/*     INTERNAL VARIABLES */


/*     COMPUTE PSI(-ALFK) */

    /* Parameter adjustments */
    --fnew;
    --xnew;
    --p;
    --xold;
    --active;
    --w;
    --hnew;

    /* Function Body */
    ctrl = -1;
    d__1 = -(*alfk);
    psimk = psi_(&xold[1], &p[1], n, &d__1, &xnew[1], &fnew[1], m, (S_fp)
	    ffunc, &hnew[1], t, l, &active[1], (S_fp)hfunc, &w[1], &ctrl);
    if (ctrl < -10) {
	*k = ctrl;
    }
    if (*k < 0) {
	return 0;
    }
    ++(*k);

/*     COMPUTE APPROXIMATIONS OF THE DERIVATIVE BY USING FORWARD, */
/*     BACKWARD AND CENTRAL DIFFERENCES */

    dpsifo = (*psik - *psizer) / *alfk;
    dpsiba = (*psizer - psimk) / *alfk;
    dpsice = (*psik - psimk) / 2. / *alfk;
    maxdif = (d__1 = dpsifo - dpsiba, abs(d__1));
/* Computing MAX */
    d__2 = maxdif, d__3 = (d__1 = dpsifo - dpsice, abs(d__1));
    maxdif = max(d__2,d__3);
/* Computing MAX */
    d__2 = maxdif, d__3 = (d__1 = dpsiba - dpsice, abs(d__1));
    maxdif = max(d__2,d__3);
    if ((d__1 = dpsifo - *dpsize, abs(d__1)) > maxdif && (d__2 = dpsice - *
	    dpsize, abs(d__2)) > maxdif) {
	*exit = -1;
    }
    return 0;
} /* chder_ */

/* PREGN */
/* Subroutine */ int pregn_(doublereal *s, doublereal *sn, doublereal *b, 
	doublereal *bn, integer *mindim, integer *prank, integer *dim)
{
    /* Initialized data */

    static doublereal smax = .2;
    static doublereal rmin = .5;

    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer i__, k, m1;


/*     GN-STEP IN PREVIOUS STEP */
/*     TAKE DIM AS THE LARGEST K (MINDIM<=K<=PRANK-1) FOR WHICH */
/*     S(K)<SMAX*SN AND B(K)>RMIN*BN */
/*     IF NO SUCH K EXISTS TAKE DIM=PRANK-1 PROVIDED (PRANK-1)>=MINDIM */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --b;
    --s;

    /* Function Body */
    m1 = *prank - 1;
    k = *mindim;
    if (*mindim > m1) {
	goto L20;
    }
    i__1 = m1;
    for (i__ = *mindim; i__ <= i__1; ++i__) {
	k = m1 - i__ + *mindim;
	if (s[k] < smax * *sn && b[k] > rmin * *bn) {
	    goto L20;
	}
/* L10: */
    }
/* Computing MAX */
    i__1 = *mindim, i__2 = *prank - 1;
    *dim = max(i__1,i__2);
    return 0;
L20:
    *dim = k;
    return 0;
} /* pregn_ */

/* PRESUB */
/* Subroutine */ int presub_(doublereal *s, doublereal *b, doublereal *bn, 
	doublereal *rabs, integer *prank, integer *km1rnk, doublereal *pgress,
	 doublereal *prelin, doublereal *asprev, doublereal *alfkm1, integer *
	dim)
{
    /* Initialized data */

    static doublereal stepb = .2;
    static doublereal pgb1 = .3;
    static doublereal pgb2 = .1;
    static doublereal predb = .7;
    static doublereal rlenb = 2.;
    static doublereal c2 = 100.;

    /* System generated locals */
    integer i__1, i__2;
    doublereal d__1, d__2;

    /* Local variables */
    static integer i__, i1;


/*     SUBSPACE MINIMIZATION IN LATEST STEP */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --b;
    --s;

    /* Function Body */

/*     IF THE LATEST STEP WAS FAIRLY GOOD THE DIMENSION MUST NOT */
/*     BE DECREASED */

/* Computing 2nd power */
    d__1 = *prelin;
/* Computing 2nd power */
    d__2 = *asprev;
    if (*alfkm1 >= stepb || *pgress > pgb1 * (d__1 * d__1) || *pgress > pgb2 *
	     (d__2 * d__2)) {
	goto L10;
    }

/*     A BAD STEP */

/* Computing MAX */
    i__1 = 1, i__2 = *km1rnk - 1;
    *dim = max(i__1,i__2);
    if (*km1rnk > 1 && b[*dim] > *rabs * *bn) {
	return 0;
    }
L10:
    *dim = *km1rnk;
    if (b[*dim] > predb * *bn && rlenb * s[*dim] < s[*dim + 1]) {
	return 0;
    }

/*     TEST POSSIBLE RANK DEFICIENCY */

    if (c2 * s[*dim] < s[*dim + 1]) {
	return 0;
    }
    i1 = *km1rnk + 1;
    i__1 = *prank;
    for (i__ = i1; i__ <= i__1; ++i__) {
	*dim = i__;
	if (b[i__] > predb * *bn) {
	    goto L30;
	}
/* L20: */
    }
L30:
    return 0;
} /* presub_ */

/* JACDIF */
/* global variable for setting step size */
double _enlsip_finite_diff_step_size = 1e-6;
/* Subroutine */ int jacdif_(doublereal *x, integer *n, doublereal *f, 
	integer *m, S_fp ffunc, doublereal *c__, integer *mdc, doublereal *w1,
	 integer *ier)
{
    /* System generated locals */
    integer c_dim1, c_offset, i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer ctrl, i__, j;
    static doublereal delta, xtemp, deltaj;


/*     COMPUTE THE M*N JACOBIAN OF F(X) AT THE CURRENT POINT BY */
/*     USING FORWARD DIFFERENCES */

/*     ON ENTRY */

/*     X()  CONTAINS THE CURRENT POINT */
/*     N    IS THE LENGTH OF THE ARRAY X */
/*     F()  CONTAINS THE VECTOR OF RESIDUALS AT CURRENT POINT */
/*     M    IS THE LENGTH OF THE ARRAYS F AND W1 */
/*     MDC  IS THE LEADING DIMENSION OF THE ARRAY C */

/*     ON RETURN */

/*     C(,) CONTAINS THE APPROXIMATED JACOBIAN IN THE M*N UPPER PART */
/*     IER  A VALUE < -10 TO INDICATE A POSSIBLE USER STOP */
/*          UNCHANGED OTHERWISE */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    --w1;
    --f;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;

    /* Function Body */
    /* delta = sqrt(machin_1.drelpr) */;
	delta = _enlsip_finite_diff_step_size;
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	xtemp = x[j];
/* Computing MAX */
	d__1 = abs(xtemp);
	deltaj = max(d__1,1.) * delta;
	/*printf("deltaj %8.6g for %8.6f\n",deltaj,d__1);*/
	x[j] = xtemp + deltaj;
	ctrl = -1;
	(*ffunc)(&x[1], n, &w1[1], m, &ctrl, &c__[c_offset], mdc);
	if (ctrl < -10) {
	    goto L30;
	}
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    c__[i__ + j * c_dim1] = (w1[i__] - f[i__]) / deltaj;
/* L10: */
	}
	x[j] = xtemp;
/* L20: */
    }
    return 0;

/*     A USER STOP INDICATOR IS DETECTED */

L30:
    *ier = ctrl;
    return 0;
} /* jacdif_ */

/* LSOLVE */
/* Subroutine */ int lsolve_(integer *ndim, integer *n, doublereal *a, 
	doublereal *b)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static integer j, k;
    static doublereal s;
    static integer jm;


/* LSOLVE SOLVES THE LOWER TRIANGULAR SYSTEM AX = B */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *ndim;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    b[1] /= a[a_dim1 + 1];
    if (*n == 1) {
	return 0;
    }
    i__1 = *n;
    for (j = 2; j <= i__1; ++j) {
	s = b[j];
	jm = j - 1;
	i__2 = jm;
	for (k = 1; k <= i__2; ++k) {
	    s -= a[j + k * a_dim1] * b[k];
/* L10: */
	}
	b[j] = s / a[j + j * a_dim1];
/* L20: */
    }
    return 0;
} /* lsolve_ */

/* USOLVE */
/* Subroutine */ int usolve_(integer *ndim, integer *n, doublereal *a, 
	doublereal *b)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static integer j, k;
    static doublereal s;
    static integer jc, jp, nm;


/* USOLVE SOLVES THE UPPER TRIANGULAR SYSTEM AX=B */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *ndim;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    b[*n] /= a[*n + *n * a_dim1];
    if (*n == 1) {
	return 0;
    }
    nm = *n - 1;
    i__1 = nm;
    for (jc = 1; jc <= i__1; ++jc) {
	j = *n - jc;
	s = b[j];
	jp = j + 1;
	i__2 = *n;
	for (k = jp; k <= i__2; ++k) {
	    s -= a[j + k * a_dim1] * b[k];
/* L10: */
	}
	b[j] = s / a[j + j * a_dim1];
/* L20: */
    }
    return 0;
} /* usolve_ */

/* YCOMP */
/* Subroutine */ int ycomp_(integer *kp1, integer *kq, doublereal *d__, 
	doublereal *c1, integer *mdc, doublereal *x1)
{
    /* System generated locals */
    integer c1_dim1, c1_offset, i__1, i__2;

    /* Local variables */
    static integer j, k;
    static doublereal sum;


/* THIS ROUTINE COMPUTES */
/*    D = D - C1*X1 */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --d__;
    c1_dim1 = *mdc;
    c1_offset = 1 + c1_dim1 * 1;
    c1 -= c1_offset;
    --x1;

    /* Function Body */
    if (*kq <= 0 || *kp1 <= 0) {
	return 0;
    }
    i__1 = *kq;
    for (j = 1; j <= i__1; ++j) {
	sum = 0.;
	i__2 = *kp1;
	for (k = 1; k <= i__2; ++k) {
	    sum += c1[j + k * c1_dim1] * x1[k];
/* L10: */
	}
	d__[j] -= sum;
/* L20: */
    }
    return 0;
} /* ycomp_ */

/* JTRJ */
/* Subroutine */ int jtrj_(doublereal *gmat, integer *mdg, integer *n, 
	doublereal *w)
{
    /* System generated locals */
    integer gmat_dim1, gmat_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer i__, j, k;
    static doublereal sum;


/*                                        T */
/*     FORM THE N*N SYMMETRIC MATRIX  GMAT *GMAT   AND STORE */
/*     IN GMAT */
/*                                      T */
/*     FIRST FORM THE LOWER PART OF GMAT *GMAT  AND STORE IN THE */
/*     LOWER PART OF GMAT */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --w;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;

    /* Function Body */
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *n;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    w[i__] = gmat[i__ + j * gmat_dim1];
/* L10: */
	}
	i__2 = *n;
	for (k = j; k <= i__2; ++k) {
	    sum = 0.;
	    i__3 = *n;
	    for (i__ = 1; i__ <= i__3; ++i__) {
		sum += gmat[i__ + k * gmat_dim1] * w[i__];
/* L20: */
	    }
	    gmat[k + j * gmat_dim1] = sum;
/* L30: */
	}
/* L40: */
    }

/*     MOVE THE LOWER PART OF GMAT TO THE UPPER PART OF GMAT */

    if (*n == 1) {
	return 0;
    }
    i__1 = *n;
    for (i__ = 2; i__ <= i__1; ++i__) {
	k = i__ - 1;
	i__2 = k;
	for (j = 1; j <= i__2; ++j) {
	    gmat[j + i__ * gmat_dim1] = gmat[i__ + j * gmat_dim1];
/* L50: */
	}
/* L60: */
    }
    return 0;
} /* jtrj_ */

/* NEWTON */
/* Subroutine */ int newton_(S_fp ffunc, S_fp hfunc, doublereal *x, integer *
	n, doublereal *c__, integer *mdc, integer *m, integer *rankc2, 
	doublereal *f, integer *p3, doublereal *d3, doublereal *v, doublereal 
	*a, integer *mda, integer *active, integer *t, integer *ranka, 
	doublereal *d1, integer *p1, integer *p2, doublereal *d2, doublereal *
	b, doublereal *h__, integer *l, integer *mdf, doublereal *pivot, 
	doublereal *gmat, integer *mdg, doublereal *dx, integer *eval, 
	integer *error, doublereal *fmat, doublereal *d__, doublereal *v1, 
	doublereal *v2)
{
    /* System generated locals */
    integer c_dim1, c_offset, a_dim1, a_offset, gmat_dim1, gmat_offset, 
	    fmat_dim1, fmat_offset, i__1, i__2;

    /* Local variables */
    extern /* Subroutine */ int c2tc2_(doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *);
    static integer info;
    extern /* Subroutine */ int dchdc_(doublereal *, integer *, integer *, 
	    doublereal *, integer *, integer *, integer *);
    static integer i__, j;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *), ecomp_(integer *, 
	    doublereal *, integer *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, doublereal *, integer *), hessf_(S_fp, 
	    doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *), hessh_(S_fp, 
	    doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    integer *, integer *, doublereal *, doublereal *, integer *, 
	    integer *), dposl_(doublereal *, integer *, integer *, doublereal 
	    *), wcomp_(doublereal *, integer *, doublereal *, integer *, 
	    integer *, integer *), ycomp_(integer *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *), p3utq3_(integer *, 
	    integer *, doublereal *, integer *, doublereal *, integer *, 
	    doublereal *, integer *, integer *, doublereal *), pv_(integer *, 
	    integer *, doublereal *, integer *, integer *), lsolve_(integer *,
	     integer *, doublereal *, doublereal *), usolve_(integer *, 
	    integer *, doublereal *, doublereal *);
    static integer tp1, nmr, nmt;


/*     COMPUTE THE SEARCH DIRECTION (DX) BY MINIMIZING */
/*           T   T                         T   T */
/*     0.5*DX *(C *C - FMAT + GMAT)*DX + (C *F) *DX */
/*     S.T. */
/*           (L@D0)*DY = B1   IF RANK(A)=T */
/*           (R@D0)*DY = B2   IF RANK(A)<T */
/*     WHERE */
/*                 T                T */
/*         (L@D0)=P1 *A*Q1     B1=-P1 *H    DX=Q1*DY */

/*                 T                  T */
/*         (R@D0)=Q2 *(L@D0)*P2    B2=Q2 *B1 DX=Q1*P2*DY */
/*     AND */
/*                 T                        M */
/*        FMAT = SIGMA(V(I)*K )    GMAT = SIGMA(F(I)*G ) */
/*                I=1        I             I=1        I */

/*     K  = HESSIAN OF CONSTRAINT H  WHERE I IS IN CURRENT WORKING SET */
/*      I                          I */

/*     G  = HESSIAN OF RESIDUAL NUMBER I */
/*      I */

/*     MOREOVER     (C1@DC2)=C*Q1*P2 */

/*                  (U)    T */
/*                  (0)= Q3 *C2*P3 */

/*     ON ENTRY@D */

/*     FFUNC   SUBROUTINE NAME-USED TO EVALUATE RESIDUALS */
/*     HFUNC   SUBROUTINE NAME-USED TO EVALUATE CONSTRAINTS */
/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE CURRENT POINT */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING  (C1@DU)  AND INFO. TO FORM Q3 */
/*                         (   0) */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS AT X */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     D3()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-RANKA */
/*             CONTAINING INFO. TO FORM Q3 */
/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING ESTIMATES OF LAGRANGE MULTIPLIERS */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING MATRIX L AND INFO. TO FORM Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR THE CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     D1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P2 (IF IT IS USED) */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2 (IF IT IS USED) */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING    T   T           T */
/*                         Q2 *P1 *(-H) OR P1 *(-H) */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT X */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINING MATRIX R AND INFO. TO FORM Q2 (IF IT IS USED) */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */

/*     ON RETURN@D */

/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINS THE COMPUTED SEARCH DIRECTION */
/*             (PROVIDED ERROR=0) */
/*     EVAL    INTEGER SCALAR-CONTAINS NUMBER OF FUNCTION EVALUATIONS */
/*             DONE INSIDE THIS ROUTINE */
/*     ERROR   INTEGER SCALAR = 0 IF NO TROUBLE */
/*             =-3 IF THE REDUCED HESSIAN MATRIX IS NOT POS. DEF. */
/*             < -10 AS A USER STOP INDICATOR */

/*     WORKING AREAS@D */

/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*     D(),V1(),V2() REAL SINGLY SUBSCRIPTED ARRAYS ALL OF DIMENSION M */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --dx;
    --x;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --v2;
    --v1;
    --d__;
    --f;
    --p3;
    --d3;
    --v;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --active;
    --d1;
    --p1;
    --p2;
    --d2;
    --b;
    --h__;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;

    /* Function Body */
    tp1 = *ranka + 1;
    nmt = *n - *t;
    nmr = *n - *ranka;

/*     MOVE -F TO D */

    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	d__[i__] = -f[i__];
/* L10: */
    }
/*                    T      T */
/*     FORM  D@D= P3*(U @D0)*Q3 *D */

    p3utq3_(&p3[1], &nmt, &c__[tp1 * c_dim1 + 1], mdc, &d3[1], &nmr, &d__[1], 
	    m, &c__1, &v2[1]);
/*                     T      T */
/*     FORM C1@D= P3 *(U @D0)*Q3 *C1 */

    p3utq3_(&p3[1], &nmt, &c__[tp1 * c_dim1 + 1], mdc, &d3[1], &nmr, &c__[
	    c_offset], m, ranka, &v2[1]);
    if (*ranka == 0) {
	goto L30;
    }
/*                 -1                  -1 */
/*     FORM DY1@D= R  *B2   OR   DY1@D= L  *B1 */
/*     STORE IN DX(I) I=1,2,...,RANKA */

    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dx[i__] = b[i__];
/* L20: */
    }
    if (*t == *ranka) {
	lsolve_(mda, t, &a[a_offset], &dx[1]);
    }
    if (*t > *ranka) {
	usolve_(mdg, ranka, &gmat[gmat_offset], &dx[1]);
    }
    if (*ranka == *n) {
	goto L80;
    }
/*                    T T     T */
/*     FORM C2@D= (U*P3 ) *U*P3 */

L30:
    c2tc2_(&c__[tp1 * c_dim1 + 1], mdc, &nmr, &p3[1], &nmt, &v2[1]);
/*                      M */
/*     COMPUTE GMAT@D= SIGMA(F(I)*G ) */
/*                     I=1        I */

    hessf_((S_fp)ffunc, &gmat[gmat_offset], mdg, &x[1], n, &f[1], &v1[1], &v2[
	    1], m, error);
    if (*error < -10) {
	return 0;
    }
    *eval = (*n << 1) * (*n + 1);
    if (*t == 0) {
	goto L40;
    }
/*                             T */
/*     COMPUTE GMAT@D= GMAT - SIGMA(V(I)*K ) */
/*                            I=1        I */

    hessh_((S_fp)hfunc, &gmat[gmat_offset], mdg, &x[1], n, &v[1], &active[1], 
	    t, &v1[1], &v2[1], l, error);
    if (*error < -10) {
	return 0;
    }

/*               (E11 E12)      T   T */
/*     COMPUTE E=(       ) @D= P2 *Q1 *GMAT*Q1*P2  (STORE IN GMAT) */
/*               (E21 E22) */

    ecomp_(&p2[1], &a[a_offset], mda, n, &d1[1], &pivot[1], ranka, t, &gmat[
	    gmat_offset], mdg);
L40:
/*                         T */
/*     COMPUTE W22@D= E22+C2 *C2   (STORE IN GMAT(I,J) I=RANKA+1,...,N */
/*                                                   J=RANKA+1,...,N */
/*                         T */
/*     COMPUTE W21@D= E21+C2 *C1  (STORE IN GMAT(I,J) I=RANKA+1,...,N */
/*                                                   J=1,2,...,RANKA */

    wcomp_(&gmat[tp1 + gmat_dim1], mdg, &c__[c_offset], mdc, &nmr, n);
    if (*ranka == 0) {
	goto L50;
    }
/*                         T */
/*     FORM D@D= -W21*DY1-C2 *F */

    ycomp_(ranka, &nmr, &d__[1], &gmat[tp1 + gmat_dim1], mdg, &dx[1]);
L50:
    i__1 = nmr;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *ranka + i__;
	dx[j] = d__[i__];
/* L60: */
    }

/*     SOLVE  W22*DY2 = D */

    dchdc_(&gmat[tp1 + tp1 * gmat_dim1], mdg, &nmr, &v2[1], &c__1, &c__0, &
	    info);
    *error = 0;
    if (nmr == info) {
	goto L70;
    }

/*     MATRIX W22 IS NOT POSITIVE DEFINTE */

    *error = -3;
    return 0;
L70:
    dposl_(&gmat[tp1 + tp1 * gmat_dim1], mdg, &nmr, &dx[tp1]);

/*     BACKTRANSFORM */

    if (*t == *ranka) {
	goto L80;
    }

/*     PERMUTE ELEMENTS IN DX */

    pv_(&p2[1], ranka, &dx[1], n, &c__1);
L80:
    if (*ranka == 0) {
	return 0;
    }

/*     COMPUTE DX@D= Q1*DX */

    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *ranka - i__ + 1;
	i__2 = j + 1;
	h12per_(&c__2, &j, &i__2, n, &a[j + a_dim1], mda, &d1[j], &dx[1], &
		c__1, mda, &c__1, &pivot[j]);
/* L90: */
    }
    return 0;
} /* newton_ */

/* P3UTQ3 */
/* Subroutine */ int p3utq3_(integer *p3, integer *nmt, doublereal *c2, 
	integer *mdc, doublereal *d3, integer *rankc2, doublereal *c__, 
	integer *m, integer *ranka, doublereal *v2)
{
    /* System generated locals */
    integer c2_dim1, c2_offset, c_dim1, c_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer i__, j, k;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *), pv_(integer *, 
	    integer *, doublereal *, integer *, integer *);
    static doublereal sum;

/*                       T      T */
/*     COMPUTE  C@D= P3*(U @D0)*Q3 *C */

/*     ON ENTRY@D */

/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION NMT */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     NMT     INTEGER SCALAR CONTAINING THE VALUE N-T */
/*     C2(,)   REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*RANKC2 */
/*             CONTAINING MATRIX U AND INFO. TO FORM Q3 */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF THE ARRAYS */
/*             C2 AND C */
/*     D3()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKC2 */
/*             CONTAINING INFO. TO FORM Q3 */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*RANKA */
/*             CONTAINING MATRIX C1 */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */

/*     ON RETURN */

/*     C(,)    CONTAINS THE MATRIX PRODUCT DEFINED ABOVE */

/*     WORKING AREA@D */

/*     V2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */

/*     INTERNAL VARIABLES */

/*                    T */
/*     COMPUTE  C@D= Q3 *C */

    /* Parameter adjustments */
    --p3;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    c2_dim1 = *mdc;
    c2_offset = 1 + c2_dim1 * 1;
    c2 -= c2_offset;
    --d3;
    --v2;

    /* Function Body */
    if (*rankc2 <= 0) {
	goto L60;
    }
    i__1 = *rankc2;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, m, &c2[i__ * c2_dim1 + 1], &c__1, &d3[i__]
		, &c__[c_offset], &c__1, mdc, ranka, &c2[i__ + i__ * c2_dim1])
		;
/* L10: */
    }
/*                    T */
/*     COMPUTE  C@D= (U @D0)*C */

    i__1 = *ranka;
    for (j = 1; j <= i__1; ++j) {
	i__2 = *rankc2;
	for (k = 1; k <= i__2; ++k) {
	    v2[k] = c__[k + j * c_dim1];
/* L20: */
	}
	i__2 = *rankc2;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    sum = 0.;
	    i__3 = i__;
	    for (k = 1; k <= i__3; ++k) {
		sum += c2[k + i__ * c2_dim1] * v2[k];
/* L30: */
	    }
	    c__[i__ + j * c_dim1] = sum;
/* L40: */
	}
/* L50: */
    }
L60:

/*     COMPUTE  C@D= P3*C */

    pv_(&p3[1], nmt, &c__[c_offset], mdc, ranka);
    return 0;
} /* p3utq3_ */

/* C2TC2 */
/* Subroutine */ int c2tc2_(doublereal *c2, integer *mdc, integer *rankc2, 
	integer *p3, integer *nmt, doublereal *v2)
{
    /* System generated locals */
    integer c2_dim1, c2_offset, i__1, i__2;

    /* Local variables */
    extern /* Subroutine */ int jtrj_(doublereal *, integer *, integer *, 
	    doublereal *), vptr_(integer *, integer *, doublereal *, integer *
	    , integer *);
    static integer i__, j, k;

/*                        T T     T */
/*     COMPUTE  C2@D= (U*P3 ) *U*P3 */

/*     ON ENTRY@D */

/*     C2(,)   REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*RANKC2 */
/*             CONTAINING THE UPPER TRIANGULAR MATRIX U */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C2 */
/*     RANKC2  INTEGER SCALAR CONTAINING THE ORDER OF MATRIX U */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION NMT */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     NMT     INTEGER SCALAR CONTAINING THE VALUE N-T */

/*     ON RETURN@D */

/*     C2(,)   CONTAINS THE MATRIX PRODUCT DEFINED ABOVE */

/*     WORKING AREA@D */

/*     V2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKC2 */

/*     INTERNAL VARIABLES */


/*     ZERO THE LOWER PART OF U */

    /* Parameter adjustments */
    c2_dim1 = *mdc;
    c2_offset = 1 + c2_dim1 * 1;
    c2 -= c2_offset;
    --p3;
    --v2;

    /* Function Body */
    if (*rankc2 <= 1) {
	goto L30;
    }
    i__1 = *rankc2;
    for (i__ = 2; i__ <= i__1; ++i__) {
	k = i__ - 1;
	i__2 = k;
	for (j = 1; j <= i__2; ++j) {
	    c2[i__ + j * c2_dim1] = 0.;
/* L10: */
	}
/* L20: */
    }
L30:

/*     PERMUTE COLUMNS IN U */

    vptr_(&p3[1], nmt, &c2[c2_offset], mdc, rankc2);
    if (*rankc2 <= 0) {
	return 0;
    }
/*                     T */
/*     COMPUTE  C2@D= C2 *C2 */

    jtrj_(&c2[c2_offset], mdc, rankc2, &v2[1]);
    return 0;
} /* c2tc2_ */

/* ECOMP */
/* Subroutine */ int ecomp_(integer *p2, doublereal *a, integer *mda, integer 
	*n, doublereal *d1, doublereal *pivot, integer *ranka, integer *t, 
	doublereal *gmat, integer *mdg)
{
    /* System generated locals */
    integer a_dim1, a_offset, gmat_dim1, gmat_offset, i__1, i__2;

    /* Local variables */
    extern /* Subroutine */ int ptrv_(integer *, integer *, doublereal *, 
	    integer *, integer *);
    static integer i__;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *), vp_(doublereal *, 
	    integer *, integer *, integer *, integer *);

/*                       T   T */
/*     COMPUTE  GMAT@D= P2 *Q1 *GMAT*Q1*P2 */
/*                           T */
/*     BY FORMING   GMAT@D= Q1 *GMAT*Q1 */
/*     AND THEN DO THE PERMUTATION IF P2 IS DIFFERENT FROM THE */
/*     IDENTITY MATRIX */

/*     INTERNAL VARIABLE */

    /* Parameter adjustments */
    --p2;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --d1;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;

    /* Function Body */
    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, n, &a[i__ + a_dim1], mda, &d1[i__], &gmat[
		gmat_offset], mdg, &c__1, n, &pivot[i__]);
/* L10: */
    }
    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, n, &a[i__ + a_dim1], mda, &d1[i__], &gmat[
		gmat_offset], &c__1, mdg, n, &pivot[i__]);
/* L20: */
    }
    if (*t == *ranka) {
	return 0;
    }

/*     DO THE PERMUTATION */

    ptrv_(&p2[1], ranka, &gmat[gmat_offset], mdg, ranka);
    vp_(&gmat[gmat_offset], mdg, ranka, ranka, &p2[1]);
    return 0;
} /* ecomp_ */

/* WCOMP */
/* Subroutine */ int wcomp_(doublereal *w, integer *mdw, doublereal *c__, 
	integer *mdc, integer *nmt, integer *n)
{
    /* System generated locals */
    integer w_dim1, w_offset, c_dim1, c_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j;


/*     COMPUTE  W@D= W+C     WHERE */
/*     BOTH W AND C ARE NMT*N */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    w_dim1 = *mdw;
    w_offset = 1 + w_dim1 * 1;
    w -= w_offset;

    /* Function Body */
    i__1 = *nmt;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = *n;
	for (j = 1; j <= i__2; ++j) {
	    w[i__ + j * w_dim1] += c__[i__ + j * c_dim1];
/* L10: */
	}
/* L20: */
    }
    return 0;
} /* wcomp_ */

/* HSUM */
doublereal hsum_(integer *active, integer *t, doublereal *h__, doublereal *w, 
	integer *l)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal ret_val, d__1, d__2;

    /* Local variables */
    static doublereal hval;
    static integer i__, j;


/*     COMPUTE THE CONSTRAINT PART OF THE MERIT FUNCTION */

/*     HSUM @D= SIGMA(W(I)*H(I)**2)+SIGMA(W(J)*MIN(0,H(J))**2) */
/*     WHERE */
/*            INDEX I BELONGS TO CURRENT WORKING SET */
/*            INDEX J BELONGS TO CURRENT INACTIVE SET */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --active;
    --w;
    --h__;

    /* Function Body */
    ret_val = 0.;
    if (*l <= 0) {
	return ret_val;
    }
    i__1 = *l;
    for (j = 1; j <= i__1; ++j) {
	if (*t <= 0) {
	    goto L40;
	}
	i__2 = *t;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    if (j == active[i__]) {
		goto L50;
	    }
/* L30: */
	}
L40:
/* Computing MIN */
	d__1 = 0., d__2 = h__[j];
	hval = min(d__1,d__2);
	goto L60;
L50:
	hval = h__[j];
L60:
/* Computing 2nd power */
	d__1 = hval;
	ret_val += w[j] * (d__1 * d__1);
/* L70: */
    }
    return ret_val;
} /* hsum_ */

/* UNSCR */
/* Subroutine */ int unscr_(integer *active, integer *bnd, integer *l, 
	integer *p)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, j, lmp;


/*     UNSCRAMBLE THE ARRAY WHICH HOLDS INFORMATION OF THE */
/*     APPEARANCE IN WORKING SET */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --active;

    /* Function Body */
    lmp = *l - *p;
    if (lmp <= 0) {
	goto L20;
    }
    i__1 = lmp;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = *bnd + i__;
	if (active[j] == -1) {
	    active[j] = 0;
	}
	if (active[j] >= 2) {
	    active[j] = 1;
	}
/* L10: */
    }
L20:
    return 0;
} /* unscr_ */

/* PREOBJ */
/* Subroutine */ int preobj_(doublereal *c__, integer *mdc, integer *m, 
	integer *ranka, doublereal *dx, doublereal *f, integer *t, doublereal 
	*fc1dy1, doublereal *c1dy1)
{
    /* System generated locals */
    integer c_dim1, c_offset, i__1, i__2;
    doublereal d__1;

    /* Local variables */
    static integer i__, j;
    static doublereal velem;


/*     COMPUTE            T */
/*              FC1DY1 = F *C1*DY1 */
/*               C1DY1 = II C1*DY1 II**2 */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --f;
    --dx;

    /* Function Body */
    *fc1dy1 = 0.;
    *c1dy1 = 0.;
    if (*t <= 0) {
	return 0;
    }
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	velem = 0.;
	i__2 = *ranka;
	for (j = 1; j <= i__2; ++j) {
	    velem += c__[i__ + j * c_dim1] * dx[j];
/* L20: */
	}
	*fc1dy1 += f[i__] * velem;
/* Computing 2nd power */
	d__1 = velem;
	*c1dy1 += d__1 * d__1;
/* L30: */
    }
    return 0;
} /* preobj_ */

/* NZESTM */
/* Subroutine */ int nzestm_(doublereal *a, integer *mda, integer *ranka, 
	integer *t, doublereal *b, doublereal *v, doublereal *w, doublereal *
	u)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1;

    /* Local variables */
    static integer i__;
    extern /* Subroutine */ int lsolve_(integer *, integer *, doublereal *, 
	    doublereal *), atsolv_(doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *);
    static doublereal res;


/*     COMPUTE THE NONZERO FIRST ORDER LAGRANGE MULTIPLIER */
/*     ESTIMATE BY FORMING               T -1 */
/*                         VNZ @D= VL-(A*A )  *H */
/*     WHERE VL = THE LEAST SQUARES FIRST ORDER ESTIMATE */
/*           A  = THE JACOBIAN OF THE CONSTRAINTS IN WORKING SET */
/*           H  = THE VALUE OF THE CONSTRAINTS IN WORKING SET */

/*     ON ENTRY@D */

/*     A()     REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*RANKA */
/*             CONTAININNG MATRIX L IN THE DECOMPOSITION   T */
/*                                                      P1 *A*Q1 = (L@D0) */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     RANKA   INTEGER SCALAR CONTAINING THE ORDER OF MATRIX L */
/*     T       INTEGER SCALAR CONTAINING NO. OF CONSTRAINTS IN WORKING SET */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING   T */
/*                        P1 *(-H) */
/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING   T */
/*                        P1 *VL */

/*     ON RETURN@D */
/*                        T        T   -T  -1 */
/*     V()     CONTAINS P1 *VL + P1 *(L  *L  *B) */

/*     WORKING AREAS@D */

/*     W(),U() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION T */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --u;
    --w;
    --v;
    --b;

    /* Function Body */
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	w[i__] = b[i__];
/* L10: */
    }
/*                   -1 */
/*     COMPUTE W @D= L  *W */

    lsolve_(mda, ranka, &a[a_offset], &w[1]);
/*                   -T */
/*     COMPUTE U @D= L  *W */

    atsolv_(&a[a_offset], mda, ranka, &w[1], &u[1], t, &res);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v[i__] += u[i__];
/* L20: */
    }
    return 0;
} /* nzestm_ */

/* LEAEST */
/* Subroutine */ int leaest_(doublereal *a, integer *mda, integer *t, 
	doublereal *f, integer *m, doublereal *v1, doublereal *c__, integer *
	mdc, integer *p1, integer *scale, doublereal *diag, doublereal *v2, 
	doublereal *v, doublereal *res)
{
    /* System generated locals */
    integer a_dim1, a_offset, c_dim1, c_offset, i__1, i__2;

    /* Local variables */
    static integer i__, j;
    extern /* Subroutine */ int pv_(integer *, integer *, doublereal *, 
	    integer *, integer *), atsolv_(doublereal *, integer *, integer *,
	     doublereal *, doublereal *, integer *, doublereal *);
    static doublereal sum;


/*     COMPUTE A SPECIAL LEAST SQUARES ESTIMATE OF LAGRANGE */
/*     MULTIPLIERS */

/*     SOLVE       T      T */
/*                A *V = C *(F+C*DX)     (1) */

/*     ON ENTRY@D */

/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING THE LOWER TRIANGULAR MATRIX L IN     T */
/*                                                     (L@D0)=P1 *A*D*Q1 */
/*             AND INFO. TO FORM THE MATRIX Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS */
/*             IN CURRENT WORKING SET */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUES OF THE RESIDUALS */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE PRODUCT C*DX */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING    (C1 @D U) WHERE  (C1@DC2)=C*Q1*FMAT*P2 */
/*               T           (     0)    FMAT AND P2 MAY BE THE IDENTITY */
/*             Q3 *C2*P2=(U) */
/*                       (0) */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING THE PERMUTATION MATRIX P1 */
/*     SCALE   INTEGER SCALAR =0 IF NO ROW SCALING OF MATRIX A HAS */
/*             BEEN DONE. >0 IF SCALING HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             REPRESENTING THE DIAGONAL SCALING MATRIX D IN */
/*             A*D */


/*     ON RETURN@D */

/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINS THE SOLUTION V OF SYSTEM (1) ABOVE */
/*     RES     REAL SCALAR CONTAINS THE RESIDUAL OF (1) ABOVE */

/*     WORKING AREA@D */

/*     V2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --v1;
    --f;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    --p1;
    --diag;
    --v2;
    --v;

    /* Function Body */
    if (*t <= 0) {
	return 0;
    }
/*                  T */
/*     FORM G1 @D= C1 *(F+C*DX) */

    i__1 = *t;
    for (j = 1; j <= i__1; ++j) {
	sum = 0.;
	i__2 = *m;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    sum += c__[i__ + j * c_dim1] * (f[i__] + v1[i__]);
/* L20: */
	}
	v2[j] = sum;
/* L30: */
    }

/*     SOLVE  L(TR)*V = G1 */

    atsolv_(&a[a_offset], mda, t, &v2[1], &v[1], t, res);

/*     PERMUTE */

    pv_(&p1[1], t, &v[1], t, &c__1);
    if (*scale == 0) {
	return 0;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v[i__] = diag[i__] * v[i__];
/* L50: */
    }
    return 0;
} /* leaest_ */

/* EUCMOD */
/* Subroutine */ int eucmod_(integer *ctrl, doublereal *w, integer *l, 
	integer *pset, integer *n, doublereal *y, doublereal *tau, doublereal 
	*wold)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    static doublereal prod;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, j, k;
    extern /* Subroutine */ int scalv_(doublereal *, doublereal *, integer *);
    static doublereal const__;
    static integer istop;
    static doublereal ynorm;
    static integer nrunch;
    static doublereal taunew, sum, yty;


/*     SOLVE THE PROBLEM */

/*           MINIMIZE II W II       (EUCLIDEAN NORM) */
/*      S.T. */
/*           Y(TR)*W >= TAU     (1) */
/*              W(I) >= WOLD(I) (2) */

/*     ON ENTRY@D */

/*     CTRL         INTEGER SCALAR CONTAINING AN INDICATOR */
/*                  = 1 WHEN THE >SIGN IN (1) IS NOT PRESENT */
/*                  = 2 WHEN THE >SIGN IN (1) IS PRESENT */
/*     W()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*                  CONTAINING WOLD (I.E. THE PENALTY CONSTANTS USED IN */
/*                  THE LATEST STEP) */
/*     L            INTEGER SCALAR CONTAINING THE TOTAL NUMBER OF */
/*                  CONSTRAINTS */
/*     PSET()       INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*                  CONTAINING INDECES FOR THE POSITIVE ELEMENTS IN THE */
/*                  VECTOR Y */
/*     N            INTEGER SCALAR CONTAINING THE NUMBER OF POSITIVE Y(I)@DS */
/*     Y()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*                  CONTAINING THE VECTOR Y IN PROBLEM (1) */
/*                  (DESTROYED ON RETURN) */
/*     TAU          REAL SCALAR CONTAING THE CONSTANT TAU IN (1) */

/*     ON RETURN@D */

/*     W()          CONTAINS THE NEW PENALTY CONSTANTS DERIVED BY SOLVING */
/*                  PROBLEM (1) ABOVE */

/*     WORKING AREA@D */

/*     WOLD()       REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --wold;
    --y;
    --pset;
    --w;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	wold[i__] = w[i__];
/* L5: */
    }
    ynorm = enlsip_dnrm2_(n, &y[1], &c__1);
/* Computing 2nd power */
    d__1 = ynorm;
    yty = d__1 * d__1;
    if (ynorm != 0.) {
	scalv_(&y[1], &ynorm, n);
    }
    taunew = *tau;
    sum = 0.;
    nrunch = *n;
L20:
    taunew -= sum;
    if (yty == 0.) {
	const__ = 1.;
    }
    if (yty != 0.) {
	const__ = taunew / yty;
    }
    yty = 0.;
    sum = 0.;
    istop = nrunch;
    k = 1;
L30:
    if (k > nrunch) {
	goto L70;
    }
    i__ = pset[k];
    prod = const__ * y[k] * ynorm;
    if (prod < wold[i__]) {
	goto L40;
    }
    w[i__] = prod;
/* Computing 2nd power */
    d__1 = y[k];
    yty += d__1 * d__1;
    ++k;
    goto L60;
L40:
    sum += w[i__] * y[k] * ynorm;
    i__1 = nrunch;
    for (j = k; j <= i__1; ++j) {
	pset[j] = pset[j + 1];
	y[j] = y[j + 1];
/* L50: */
    }
    --nrunch;
L60:
    goto L30;
L70:
    yty = yty * ynorm * ynorm;
    if (nrunch <= 0 || *ctrl == 2) {
	goto L80;
    }
    if (istop != nrunch) {
	goto L20;
    }
L80:
    return 0;
} /* eucmod_ */

/*     LINPACK ROUTINES FOR ENLSIP */
/*     DOUBLE PRECISION VERSION 841005 */

/* DPOSL */
/* Subroutine */ int dposl_(doublereal *a, integer *lda, integer *n, 
	doublereal *b)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    extern doublereal enlsip_ddot_(integer *, doublereal *, integer *, doublereal *, 
	    integer *);
    static integer k;
    static doublereal t;
    extern /* Subroutine */ int enlsip_daxpy_(integer *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *);
    static integer kb;


/*     DPOSL SOLVES THE DOUBLE PRECISION SYMMETRIC POSITIVE DEFINITE */
/*     SYSTEM A * X = B */
/*     USING THE FACTORS COMPUTED BY DPOCO OR DPOFA. */

/*     ON ENTRY */

/*        A       DOUBLE PRECISION(LDA, N) */
/*                THE OUTPUT FROM DPOCO OR DPOFA. */

/*        LDA     INTEGER */
/*                THE LEADING DIMENSION OF THE ARRAY  A . */

/*        N       INTEGER */
/*                THE ORDER OF THE MATRIX  A . */

/*        B       DOUBLE PRECISION(N) */
/*                THE RIGHT HAND SIDE VECTOR. */

/*     ON RETURN */

/*        B       THE SOLUTION VECTOR  X . */

/*     ERROR CONDITION */

/*        A DIVISION BY ZERO WILL OCCUR IF THE INPUT FACTOR CONTAINS */
/*        A ZERO ON THE DIAGONAL.  TECHNICALLY THIS INDICATES */
/*        SINGULARITY BUT IT IS USUALLY CAUSED BY IMPROPER SUBROUTINE */
/*        ARGUMENTS.  IT WILL NOT OCCUR IF THE SUBROUTINES ARE CALLED */
/*        CORRECTLY AND  INFO .EQ. 0 . */

/*     TO COMPUTE  INVERSE(A) * C  WHERE  C  IS A MATRIX */
/*     WITH  P  COLUMNS */
/*           CALL DPOCO(A,LDA,N,RCOND,Z,INFO) */
/*           IF (RCOND IS TOO SMALL .OR. INFO .NE. 0) GO TO ... */
/*           DO 10 J = 1, P */
/*              CALL DPOSL(A,LDA,N,C(1,J)) */
/*        10 CONTINUE */

/*     LINPACK.  THIS VERSION DATED 08/14/78 . */
/*     CLEVE MOLER, UNIVERSITY OF NEW MEXICO, ARGONNE NATIONAL LAB. */

/*     SUBROUTINES AND FUNCTIONS */

/*     BLAS DAXPY,DDOT */

/*     INTERNAL VARIABLES */


/*     SOLVE TRANS(R)*Y = B */

    /* Parameter adjustments */
    a_dim1 = *lda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;

    /* Function Body */
    i__1 = *n;
    for (k = 1; k <= i__1; ++k) {
	i__2 = k - 1;
	t = enlsip_ddot_(&i__2, &a[k * a_dim1 + 1], &c__1, &b[1], &c__1);
	b[k] = (b[k] - t) / a[k + k * a_dim1];
/* L10: */
    }

/*     SOLVE R*X = Y */

    i__1 = *n;
    for (kb = 1; kb <= i__1; ++kb) {
	k = *n + 1 - kb;
	b[k] /= a[k + k * a_dim1];
	t = -b[k];
	i__2 = k - 1;
	enlsip_daxpy_(&i__2, &t, &a[k * a_dim1 + 1], &c__1, &b[1], &c__1);
/* L20: */
    }
    return 0;
} /* dposl_ */

/* DCHDC */
/* Subroutine */ int dchdc_(doublereal *a, integer *lda, integer *p, 
	doublereal *work, integer *jpvt, integer *job, integer *info)
{
    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2, i__3;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static logical negk;
    static integer maxl;
    static doublereal temp;
    static integer j, k, l;
    extern /* Subroutine */ int enlsip_dswap_(integer *, doublereal *, integer *, 
	    doublereal *, integer *);
    static logical swapk;
    extern /* Subroutine */ int enlsip_daxpy_(integer *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *);
    static integer kb, jp, pl, jt;
    static doublereal maxdia;
    static integer pu, km1, kp1, plp1;


/*     DCHDC COMPUTES THE CHOLESKY DECOMPOSITION OF A POSITIVE DEFINITE */
/*     MATRIX.  A PIVOTING OPTION ALLOWS THE USER TO ESTIMATE THE */
/*     CONDITION OF A POSITIVE DEFINITE MATRIX OR DETERMINE THE RANK */
/*     OF A POSITIVE SEMIDEFINITE MATRIX. */

/*     ON ENTRY */

/*         A      DOUBLE PRECISION(LDA,P). */
/*                A CONTAINS THE MATRIX WHOSE DECOMPOSITION IS TO */
/*                BE COMPUTED.  ONLT THE UPPER HALF OF A NEED BE STORED. */
/*                THE LOWER PART OF THE ARRAY A IS NOT REFERENCED. */

/*         LDA    INTEGER. */
/*                LDA IS THE LEADING DIMENSION OF THE ARRAY A. */

/*         P      INTEGER. */
/*                P IS THE ORDER OF THE MATRIX. */

/*         WORK   DOUBLE PRECISION. */
/*                WORK IS A WORK ARRAY. */

/*         JPVT   INTEGER(P). */
/*                JPVT CONTAINS INTEGERS THAT CONTROL THE SELECTION */
/*                OF THE PIVOT ELEMENTS, IF PIVOTING HAS BEEN REQUESTED. */
/*                EACH DIAGONAL ELEMENT A(K,K) */
/*                IS PLACED IN ONE OF THREE CLASSES ACCORDING TO THE */
/*                VALUE OF JPVT(K). */

/*                   IF JPVT(K) .GT. 0, THEN X(K) IS AN INITIAL */
/*                                      ELEMENT. */

/*                   IF JPVT(K) .EQ. 0, THEN X(K) IS A FREE ELEMENT. */

/*                   IF JPVT(K) .LT. 0, THEN X(K) IS A FINAL ELEMENT. */

/*                BEFORE THE DECOMPOSITION IS COMPUTED, INITIAL ELEMENTS */
/*                ARE MOVED BY SYMMETRIC ROW AND COLUMN INTERCHANGES TO */
/*                THE BEGINNING OF THE ARRAY A AND FINAL */
/*                ELEMENTS TO THE END.  BOTH INITIAL AND FINAL ELEMENTS */
/*                ARE FROZEN IN PLACE DURING THE COMPUTATION AND ONLY */
/*                FREE ELEMENTS ARE MOVED.  AT THE K-TH STAGE OF THE */
/*                REDUCTION, IF A(K,K) IS OCCUPIED BY A FREE ELEMENT */
/*                IT IS INTERCHANGED WITH THE LARGEST FREE ELEMENT */
/*                A(L,L) WITH L .GE. K.  JPVT IS NOT REFERENCED IF */
/*                JOB .EQ. 0. */

/*        JOB     INTEGER. */
/*                JOB IS AN INTEGER THAT INITIATES COLUMN PIVOTING. */
/*                IF JOB .EQ. 0, NO PIVOTING IS DONE. */
/*                IF JOB .NE. 0, PIVOTING IS DONE. */

/*     ON RETURN */

/*         A      A CONTAINS IN ITS UPPER HALF THE CHOLESKY FACTOR */
/*                OF THE MATRIX A AS IT HAS BEEN PERMUTED BY PIVOTING. */

/*         JPVT   JPVT(J) CONTAINS THE INDEX OF THE DIAGONAL ELEMENT */
/*                OF A THAT WAS MOVED INTO THE J-TH POSITION, */
/*                PROVIDED PIVOTING WAS REQUESTED. */

/*         INFO   CONTAINS THE INDEX OF THE LAST POSITIVE DIAGONAL */
/*                ELEMENT OF THE CHOLESKY FACTOR. */

/*     FOR POSITIVE DEFINITE MATRICES INFO = P IS THE NORMAL RETURN. */
/*     FOR PIVOTING WITH POSITIVE SEMIDEFINITE MATRICES INFO WILL */
/*     IN GENERAL BE LESS THAN P.  HOWEVER, INFO MAY BE GREATER THAN */
/*     THE RANK OF A, SINCE ROUNDING ERROR CAN CAUSE AN OTHERWISE ZERO */
/*     ELEMENT TO BE POSITIVE. INDEFINITE SYSTEMS WILL ALWAYS CAUSE */
/*     INFO TO BE LESS THAN P. */

/*     LINPACK. THIS VERSION DATED 03/19/79 . */
/*     J.J. DONGARRA AND G.W. STEWART, ARGONNE NATIONAL LABORATORY AND */
/*     UNIVERSITY OF MARYLAND. */


/*     BLAS DAXPY,DSWAP */
/*     FORTRAN DSQRT */

/*     INTERNAL VARIABLES */


    /* Parameter adjustments */
    a_dim1 = *lda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --work;
    --jpvt;

    /* Function Body */
    pl = 1;
    pu = 0;
    *info = *p;
    if (*job == 0) {
	goto L160;
    }

/*        PIVOTING HAS BEEN REQUESTED. REARRANGE THE */
/*        THE ELEMENTS ACCORDING TO JPVT. */

    i__1 = *p;
    for (k = 1; k <= i__1; ++k) {
	swapk = jpvt[k] > 0;
	negk = jpvt[k] < 0;
	jpvt[k] = k;
	if (negk) {
	    jpvt[k] = -jpvt[k];
	}
	if (! swapk) {
	    goto L60;
	}
	if (k == pl) {
	    goto L50;
	}
	i__2 = pl - 1;
	enlsip_dswap_(&i__2, &a[k * a_dim1 + 1], &c__1, &a[pl * a_dim1 + 1], &c__1);
	temp = a[k + k * a_dim1];
	a[k + k * a_dim1] = a[pl + pl * a_dim1];
	a[pl + pl * a_dim1] = temp;
	plp1 = pl + 1;
	if (*p < plp1) {
	    goto L40;
	}
	i__2 = *p;
	for (j = plp1; j <= i__2; ++j) {
	    if (j >= k) {
		goto L10;
	    }
	    temp = a[pl + j * a_dim1];
	    a[pl + j * a_dim1] = a[j + k * a_dim1];
	    a[j + k * a_dim1] = temp;
	    goto L20;
L10:
	    if (j == k) {
		goto L20;
	    }
	    temp = a[k + j * a_dim1];
	    a[k + j * a_dim1] = a[pl + j * a_dim1];
	    a[pl + j * a_dim1] = temp;
L20:
/* L30: */
	    ;
	}
L40:
	jpvt[k] = jpvt[pl];
	jpvt[pl] = k;
L50:
	++pl;
L60:
/* L70: */
	;
    }
    pu = *p;
    if (*p < pl) {
	goto L150;
    }
    i__1 = *p;
    for (kb = pl; kb <= i__1; ++kb) {
	k = *p - kb + pl;
	if (jpvt[k] >= 0) {
	    goto L130;
	}
	jpvt[k] = -jpvt[k];
	if (pu == k) {
	    goto L120;
	}
	i__2 = k - 1;
	enlsip_dswap_(&i__2, &a[k * a_dim1 + 1], &c__1, &a[pu * a_dim1 + 1], &c__1);
	temp = a[k + k * a_dim1];
	a[k + k * a_dim1] = a[pu + pu * a_dim1];
	a[pu + pu * a_dim1] = temp;
	kp1 = k + 1;
	if (*p < kp1) {
	    goto L110;
	}
	i__2 = *p;
	for (j = kp1; j <= i__2; ++j) {
	    if (j >= pu) {
		goto L80;
	    }
	    temp = a[k + j * a_dim1];
	    a[k + j * a_dim1] = a[j + pu * a_dim1];
	    a[j + pu * a_dim1] = temp;
	    goto L90;
L80:
	    if (j == pu) {
		goto L90;
	    }
	    temp = a[k + j * a_dim1];
	    a[k + j * a_dim1] = a[pu + j * a_dim1];
	    a[pu + j * a_dim1] = temp;
L90:
/* L100: */
	    ;
	}
L110:
	jt = jpvt[k];
	jpvt[k] = jpvt[pu];
	jpvt[pu] = jt;
L120:
	--pu;
L130:
/* L140: */
	;
    }
L150:
L160:
    i__1 = *p;
    for (k = 1; k <= i__1; ++k) {

/*        REDUCTION LOOP. */

	maxdia = a[k + k * a_dim1];
	kp1 = k + 1;
	maxl = k;

/*        DETERMINE THE PIVOT ELEMENT. */

	if (k < pl || k >= pu) {
	    goto L190;
	}
	i__2 = pu;
	for (l = kp1; l <= i__2; ++l) {
	    if (a[l + l * a_dim1] <= maxdia) {
		goto L170;
	    }
	    maxdia = a[l + l * a_dim1];
	    maxl = l;
L170:
/* L180: */
	    ;
	}
L190:

/*        QUIT IF THE PIVOT ELEMENT IS NOT POSITIVE. */

	if (maxdia > 0.) {
	    goto L200;
	}
	*info = k - 1;
/*     ......EXIT */
	goto L280;
L200:
	if (k == maxl) {
	    goto L210;
	}

/*           START THE PIVOTING AND UPDATE JPVT. */

	km1 = k - 1;
	enlsip_dswap_(&km1, &a[k * a_dim1 + 1], &c__1, &a[maxl * a_dim1 + 1], &c__1);
	a[maxl + maxl * a_dim1] = a[k + k * a_dim1];
	a[k + k * a_dim1] = maxdia;
	jp = jpvt[maxl];
	jpvt[maxl] = jpvt[k];
	jpvt[k] = jp;
L210:

/*        REDUCTION STEP. PIVOTING IS CONTAINED ACROSS THE ROWS. */

	work[k] = sqrt(a[k + k * a_dim1]);
	a[k + k * a_dim1] = work[k];
	if (*p < kp1) {
	    goto L260;
	}
	i__2 = *p;
	for (j = kp1; j <= i__2; ++j) {
	    if (k == maxl) {
		goto L240;
	    }
	    if (j >= maxl) {
		goto L220;
	    }
	    temp = a[k + j * a_dim1];
	    a[k + j * a_dim1] = a[j + maxl * a_dim1];
	    a[j + maxl * a_dim1] = temp;
	    goto L230;
L220:
	    if (j == maxl) {
		goto L230;
	    }
	    temp = a[k + j * a_dim1];
	    a[k + j * a_dim1] = a[maxl + j * a_dim1];
	    a[maxl + j * a_dim1] = temp;
L230:
L240:
	    a[k + j * a_dim1] /= work[k];
	    work[j] = a[k + j * a_dim1];
	    temp = -a[k + j * a_dim1];
	    i__3 = j - k;
	    enlsip_daxpy_(&i__3, &temp, &work[kp1], &c__1, &a[kp1 + j * a_dim1], &
		    c__1);
/* L250: */
	}
L260:
/* L270: */
	;
    }
L280:
    return 0;
} /* dchdc_ */

/* DAXPY */
/* Subroutine */ int enlsip_daxpy_(integer *n, doublereal *da, doublereal *dx, 
	integer *incx, doublereal *dy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m, ix, iy, mp1;


/*     CONSTANT TIMES A VECTOR PLUS A VECTOR. */
/*     USES UNROLLED LOOPS FOR INCREMENTS EQUAL TO ONE. */
/*     JACK DONGARRA, LINPACK, 3/11/78. */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*da == 0.) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        CODE FOR UNEQUAL INCREMENTS OR EQUAL INCREMENTS */
/*          NOT EQUAL TO 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[iy] += *da * dx[ix];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        CODE FOR BOTH INCREMENTS EQUAL TO 1 */


/*        CLEAN-UP LOOP */

L20:
    m = *n % 4;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[i__] += *da * dx[i__];
/* L30: */
    }
    if (*n < 4) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 4) {
	dy[i__] += *da * dx[i__];
	dy[i__ + 1] += *da * dx[i__ + 1];
	dy[i__ + 2] += *da * dx[i__ + 2];
	dy[i__ + 3] += *da * dx[i__ + 3];
/* L50: */
    }
    return 0;
} /* enlsip_daxpy_ */

/* DCOPY */
/* Subroutine */ int enlsip_dcopy_(integer *n, doublereal *dx, integer *incx, 
	doublereal *dy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m, ix, iy, mp1;


/*     COPIES A VECTOR, X, TO A VECTOR, Y. */
/*     USES UNROLLED LOOPS FOR INCREMENTS EQUAL TO ONE. */
/*     JACK DONGARRA, LINPACK, 3/11/78. */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        CODE FOR UNEQUAL INCREMENTS OR EQUAL INCREMENTS */
/*          NOT EQUAL TO 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[iy] = dx[ix];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        CODE FOR BOTH INCREMENTS EQUAL TO 1 */


/*        CLEAN-UP LOOP */

L20:
    m = *n % 7;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dy[i__] = dx[i__];
/* L30: */
    }
    if (*n < 7) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 7) {
	dy[i__] = dx[i__];
	dy[i__ + 1] = dx[i__ + 1];
	dy[i__ + 2] = dx[i__ + 2];
	dy[i__ + 3] = dx[i__ + 3];
	dy[i__ + 4] = dx[i__ + 4];
	dy[i__ + 5] = dx[i__ + 5];
	dy[i__ + 6] = dx[i__ + 6];
/* L50: */
    }
    return 0;
} /* enlsip_dcopy_ */

/* DDOT */
doublereal enlsip_ddot_(integer *n, doublereal *dx, integer *incx, doublereal *dy, 
	integer *incy)
{
    /* System generated locals */
    integer i__1;
    doublereal ret_val;

    /* Local variables */
    static integer i__, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     FORMS THE DOT PRODUCT OF TWO VECTORS. */
/*     USES UNROLLED LOOPS FOR INCREMENTS EQUAL TO ONE. */
/*     JACK DONGARRA, LINPACK, 3/11/78. */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    ret_val = 0.;
    dtemp = 0.;
    if (*n <= 0) {
	return ret_val;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        CODE FOR UNEQUAL INCREMENTS OR EQUAL INCREMENTS */
/*          NOT EQUAL TO 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp += dx[ix] * dy[iy];
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    ret_val = dtemp;
    return ret_val;

/*        CODE FOR BOTH INCREMENTS EQUAL TO 1 */


/*        CLEAN-UP LOOP */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp += dx[i__] * dy[i__];
/* L30: */
    }
    if (*n < 5) {
	goto L60;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 5) {
	dtemp = dtemp + dx[i__] * dy[i__] + dx[i__ + 1] * dy[i__ + 1] + dx[
		i__ + 2] * dy[i__ + 2] + dx[i__ + 3] * dy[i__ + 3] + dx[i__ + 
		4] * dy[i__ + 4];
/* L50: */
    }
L60:
    ret_val = dtemp;
    return ret_val;
} /* enlsip_ddot_ */

/* DNRM2 */
doublereal enlsip_dnrm2_(integer *n, doublereal *dx, integer *incx)
{
    /* Initialized data */

    static doublereal zero = 0.;
    static doublereal one = 1.;
    static doublereal cutlo = 8.232e-11;
    static doublereal cuthi = 1.304e19;

    /* Format strings */
    static char fmt_30[] = "";
    static char fmt_50[] = "";
    static char fmt_70[] = "";
    static char fmt_110[] = "";

    /* System generated locals */
    integer i__1, i__2;
    doublereal ret_val, d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal xmax;
    static integer next, i__, j;
    static doublereal dtemp;
    static integer nn;
    static doublereal hitest, sum;

    /* Assigned format variables */
    static char *next_fmt;

    /* Parameter adjustments */
    --dx;

    /* Function Body */

/*     EUCLIDEAN NORM OF THE N-VECTOR STORED IN DX() WITH STORAGE */
/*     INCREMENT INCX . */
/*     IF    N .LE. 0 RETURN WITH RESULT = 0. */
/*     IF N .GE. 1 THEN INCX MUST BE .GE. 1 */

/*           C.L.LAWSON, 1978 JAN 08 */

/*     FOUR PHASE METHOD     USING TWO BUILT-IN CONSTANTS THAT ARE */
/*     HOPEFULLY APPLICABLE TO ALL MACHINES. */
/*         CUTLO = MAXIMUM OF  DSQRT(U/EPS)  OVER ALL KNOWN MACHINES. */
/*         CUTHI = MINIMUM OF  DSQRT(V)      OVER ALL KNOWN MACHINES. */
/*     WHERE */
/*         EPS = SMALLEST NO. SUCH THAT EPS + 1. .GT. 1. */
/*         U   = SMALLEST POSITIVE NO.   (UNDERFLOW LIMIT) */
/*         V   = LARGEST  NO.            (OVERFLOW  LIMIT) */

/*     BRIEF OUTLINE OF ALGORITHM.. */

/*     PHASE 1    SCANS ZERO COMPONENTS. */
/*     MOVE TO PHASE 2 WHEN A COMPONENT IS NONZERO AND .LE. CUTLO */
/*     MOVE TO PHASE 3 WHEN A COMPONENT IS .GT. CUTLO */
/*     MOVE TO PHASE 4 WHEN A COMPONENT IS .GE. CUTHI/M */
/*     WHERE M = N FOR X() REAL AND M = 2*N FOR COMPLEX. */

/*     VALUES FOR CUTLO AND CUTHI.. */
/*     FROM THE ENVIRONMENTAL PARAMETERS LISTED IN THE IMSL CONVERTER */
/*     DOCUMENT THE LIMITING VALUES ARE AS FOLLOWS.. */
/*     CUTLO, S.P.   U/EPS = 2**(-102) FOR  HONEYWELL.  CLOSE SECONDS ARE */
/*                   UNIVAC AND DEC AT 2**(-103) */
/*                   THUS CUTLO = 2**(-51) = 4.44089E-16 */
/*     CUTHI, S.P.   V = 2**127 FOR UNIVAC, HONEYWELL, AND DEC. */
/*                   THUS CUTHI = 2**(63.5) = 1.30438E19 */
/*     CUTLO, D.P.   U/EPS = 2**(-67) FOR HONEYWELL AND DEC. */
/*                   THUS CUTLO = 2**(-33.5) = 8.23181D-11 */
/*     CUTHI, D.P.   SAME AS S.P.  CUTHI = 1.30438D19 */
/*     DATA CUTLO, CUTHI / 8.232D-11,  1.304D19 / */
/*     DATA CUTLO, CUTHI / 4.441E-16,  1.304E19 / */

    if (*n > 0) {
	goto L10;
    }
    ret_val = zero;
    goto L300;

L10:
    next = 0;
    next_fmt = fmt_30;
    sum = zero;
    nn = *n * *incx;
/*                                                 BEGIN MAIN LOOP */
    i__ = 1;
L20:
    switch (next) {
	case 0: goto L30;
	case 1: goto L50;
	case 2: goto L70;
	case 3: goto L110;
    }
L30:
    if ((d__1 = dx[i__], abs(d__1)) > cutlo) {
	goto L85;
    }
    next = 1;
    next_fmt = fmt_50;
    xmax = zero;

/*                        PHASE 1.  SUM IS ZERO */

L50:
    if (dx[i__] == zero) {
	goto L200;
    }
    if ((d__1 = dx[i__], abs(d__1)) > cutlo) {
	goto L85;
    }

/*                                PREPARE FOR PHASE 2. */
    next = 2;
    next_fmt = fmt_70;
    goto L105;

/*                                PREPARE FOR PHASE 4. */

L100:
    i__ = j;
    next = 3;
    next_fmt = fmt_110;
    sum = sum / dx[i__] / dx[i__];
L105:
    xmax = (d__1 = dx[i__], abs(d__1));
    goto L115;

/*                   PHASE 2.  SUM IS SMALL. */
/*                             SCALE TO AVOID DESTRUCTIVE UNDERFLOW. */

L70:
    if ((d__1 = dx[i__], abs(d__1)) > cutlo) {
	goto L75;
    }

/*                     COMMON CODE FOR PHASES 2 AND 4. */
/*                     IN PHASE 4 SUM IS LARGE.  SCALE TO AVOID OVERFLOW. */

L110:
    if ((d__1 = dx[i__], abs(d__1)) <= xmax) {
	goto L115;
    }
/* Computing 2nd power */
    d__1 = xmax / dx[i__];
    sum = one + sum * (d__1 * d__1);
    xmax = (d__1 = dx[i__], abs(d__1));
    goto L200;

L115:
/* Computing 2nd power */
    d__1 = dx[i__] / xmax;
    sum += d__1 * d__1;
    goto L200;


/*                  PREPARE FOR PHASE 3. */

L75:
    sum = sum * xmax * xmax;


/*     FOR REAL OR D.P. SET HITEST = CUTHI/N */
/*     FOR COMPLEX      SET HITEST = CUTHI/(2*N) */

L85:
    hitest = cuthi / (doublereal) (*n);

/*                   PHASE 3.  SUM IS MID-RANGE.  NO SCALING. */

    i__1 = nn;
    i__2 = *incx;
    for (j = i__; i__2 < 0 ? j >= i__1 : j <= i__1; j += i__2) {
	if ((d__1 = dx[j], abs(d__1)) >= hitest) {
	    goto L100;
	}
	dtemp = dx[j] * dx[j];
/* L95: */
	sum = dtemp + sum;
    }
    ret_val = sqrt(sum);
    goto L300;

L200:
    i__ += *incx;
    if (i__ <= nn) {
	goto L20;
    }

/*              END OF MAIN LOOP. */

/*              COMPUTE SQUARE ROOT AND ADJUST FOR SCALING. */

    ret_val = xmax * sqrt(sum);
L300:
    return ret_val;
} /* enlsip_dnrm2_ */

/* DSCAL */
/* Subroutine */ int enlsip_dscal_(integer *n, doublereal *da, doublereal *dx, 
	integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;

    /* Local variables */
    static integer i__, m, nincx, mp1;


/*     SCALES A VECTOR BY A CONSTANT. */
/*     USES UNROLLED LOOPS FOR INCREMENT EQUAL TO ONE. */
/*     JACK DONGARRA, LINPACK, 3/11/78. */


    /* Parameter adjustments */
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1) {
	goto L20;
    }

/*        CODE FOR INCREMENT NOT EQUAL TO 1 */

    nincx = *n * *incx;
    i__1 = nincx;
    i__2 = *incx;
    for (i__ = 1; i__2 < 0 ? i__ >= i__1 : i__ <= i__1; i__ += i__2) {
	dx[i__] = *da * dx[i__];
/* L10: */
    }
    return 0;

/*        CODE FOR INCREMENT EQUAL TO 1 */


/*        CLEAN-UP LOOP */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__2 = m;
    for (i__ = 1; i__ <= i__2; ++i__) {
	dx[i__] = *da * dx[i__];
/* L30: */
    }
    if (*n < 5) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__2 = *n;
    for (i__ = mp1; i__ <= i__2; i__ += 5) {
	dx[i__] = *da * dx[i__];
	dx[i__ + 1] = *da * dx[i__ + 1];
	dx[i__ + 2] = *da * dx[i__ + 2];
	dx[i__ + 3] = *da * dx[i__ + 3];
	dx[i__ + 4] = *da * dx[i__ + 4];
/* L50: */
    }
    return 0;
} /* enlsip_dscal_ */

/* DSWAP */
/* Subroutine */ int enlsip_dswap_(integer *n, doublereal *dx, integer *incx, 
	doublereal *dy, integer *incy)
{
    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i__, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     INTERCHANGES TWO VECTORS. */
/*     USES UNROLLED LOOPS FOR INCREMENTS EQUAL ONE. */
/*     JACK DONGARRA, LINPACK, 3/11/78. */


    /* Parameter adjustments */
    --dy;
    --dx;

    /* Function Body */
    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       CODE FOR UNEQUAL INCREMENTS OR EQUAL INCREMENTS NOT EQUAL */
/*         TO 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = dx[ix];
	dx[ix] = dy[iy];
	dy[iy] = dtemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       CODE FOR BOTH INCREMENTS EQUAL TO 1 */


/*       CLEAN-UP LOOP */

L20:
    m = *n % 3;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dtemp = dx[i__];
	dx[i__] = dy[i__];
	dy[i__] = dtemp;
/* L30: */
    }
    if (*n < 3) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i__ = mp1; i__ <= i__1; i__ += 3) {
	dtemp = dx[i__];
	dx[i__] = dy[i__];
	dy[i__] = dtemp;
	dtemp = dx[i__ + 1];
	dx[i__ + 1] = dy[i__ + 1];
	dy[i__ + 1] = dtemp;
	dtemp = dx[i__ + 2];
	dx[i__ + 2] = dy[i__ + 2];
	dy[i__ + 2] = dtemp;
/* L50: */
    }
    return 0;
} /* enlsip_dswap_ */

