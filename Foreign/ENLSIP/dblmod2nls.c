/* dblmod2nls.f -- translated by f2c (version 20000817).
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
    integer ifree;
} negdir_;

#define negdir_1 negdir_

struct {
    doublereal drelpr;
} machin_;

#define machin_1 machin_

struct {
    doublereal u[400]	/* was [100][4] */;
} wsave_;

#define wsave_1 wsave_

/* Table of constant values */

static doublereal c_b3 = 1.;
static integer c__1 = 1;
static integer c__100 = 100;
static integer c__4 = 4;
static integer c__2 = 2;


/*     DOUBLE PRECISION VERSION 880505 */
/* GIVEN1 */
/* Subroutine */ int given1_(doublereal *z1, doublereal *z2, doublereal *c__, 
	doublereal *s, doublereal *sig)
{
    /* Builtin functions */
    double sqrt(doublereal), d_sign(doublereal *, doublereal *);

    /* Local variables */
    static doublereal gamma, gamma2;


/*     COMPUTE GIVENS ROTATIONS MATRIX SUCH THAT */

/*     (C  S)(Z1) = (+SIG)   WHERE SIG=DSQRT(Z1**2+Z2**2) */
/*     (S -C)(Z2) = ( 0  ) */

/*     INTERNAL VARIABLES */

    if (*z2 == 0.) {
	goto L10;
    }
    gamma2 = *z1 * *z1 + *z2 * *z2;
    gamma = sqrt(gamma2);
    *c__ = *z1 / gamma;
    *s = *z2 / gamma;
    *sig = gamma;
    return 0;
L10:
    *c__ = d_sign(&c_b3, z1);
    *s = 0.;
    *sig = abs(*z1);
    return 0;
} /* given1_ */


/*     DOUBLE PRECISION VERSION 910326 */

/* NLSNIP */
/* Subroutine */ int nlsnip_(doublereal *x, integer *n, integer *mdc, integer 
	*mda, integer *mdg, integer *mdf, integer *m, integer *p, integer *l, 
	doublereal *tol, doublereal *w, doublereal *epsrel, doublereal *
	epsabs, doublereal *epsx, doublereal *epsh, integer *iprint, integer *
	nout, integer *maxit, integer *norm, integer *scale, logical *sec, 
	S_fp ffunc, S_fp hfunc, integer *exit, doublereal *phi, integer *k, 
	integer *funcev, integer *jacev, integer *secev, integer *linev, 
	integer *ranka, integer *rank, doublereal *f, doublereal *h__, 
	integer *active, doublereal *speed, integer *p1, integer *p2, integer 
	*p3, integer *inact, integer *p4, doublereal *b, doublereal *d1, 
	doublereal *d2, doublereal *d3, doublereal *diag, doublereal *g, 
	doublereal *pivot, doublereal *dx, doublereal *v, doublereal *u, 
	doublereal *s, doublereal *wold, doublereal *a, doublereal *d__, 
	doublereal *v1, doublereal *v2, doublereal *c__, doublereal *fmat, 
	doublereal *gmat)
{
    /* System generated locals */
    integer a_dim1, a_offset, c_dim1, c_offset, fmat_dim1, fmat_offset, 
	    gmat_dim1, gmat_offset, i__1, i__2;
    doublereal d__1, d__2, d__3;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer code, dima;
    extern /* Subroutine */ int grad_(doublereal *, integer *, integer *, 
	    integer *, doublereal *, doublereal *);
    static integer eval, time;
    static doublereal gres, dnrm, fsum, hsum;
    static integer dimc2;
    static doublereal b1nrm, d1nrm;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, j, t;
    static doublereal alpha, betak, xdiff;
    extern doublereal sacth_(doublereal *, integer *, integer *);
    extern /* Subroutine */ int evadd_(doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    logical *), equal_(doublereal *, integer *, doublereal *, integer 
	    *, integer *, integer *, integer *, integer *, integer *);
    static integer ctrlf, ctrlh;
    static doublereal gnorm;
    static integer error;
    static doublereal xnorm;
    static integer rankc2;
    static doublereal whnrm2;
    static integer ii;
    extern /* Subroutine */ int inialc_(integer *, integer *, doublereal *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, integer *);
    static doublereal alfnoi;
    extern /* Subroutine */ int evscal_(integer *, doublereal *, integer *, 
	    integer *, integer *, doublereal *, doublereal *);
    static doublereal gndnrm, succes, alflow, alfupp, sigmin, athnrm, absvmx;
    static logical restar;
    extern /* Subroutine */ int releps_(doublereal *), minmax_(integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    doublereal *);
    static doublereal dxnorm;
    extern /* Subroutine */ int tercri_(integer *, integer *, integer *, 
	    integer *, integer *, integer *, logical *, logical *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     doublereal *, doublereal *, doublereal *, doublereal *, integer *
	    , integer *, doublereal *, doublereal *, doublereal *, doublereal 
	    *, doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *), analys_(
	    integer *, logical *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, integer *, doublereal *, 
	    integer *, doublereal *, integer *, integer *, integer *, integer 
	    *, doublereal *, integer *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *, integer *, 
	    doublereal *, integer *, doublereal *, doublereal *, integer *, 
	    doublereal *, integer *, doublereal *, doublereal *, integer *, 
	    S_fp, S_fp, doublereal *, logical *, logical *, logical *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     doublereal *, integer *, integer *, doublereal *, integer *, 
	    integer *, doublereal *, doublereal *), savexk_(doublereal *, 
	    integer *, doublereal *);
    static integer nohous;
    extern /* Subroutine */ int newpnt_(doublereal *, integer *, doublereal *,
	     integer *, doublereal *, integer *, S_fp, S_fp, integer *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *), stplng_(logical *, doublereal *, 
	    doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *, S_fp, integer *, integer *,
	     doublereal *, S_fp, integer *, integer *, integer *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *), wrkset_(doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *, integer *, integer 
	    *, integer *, integer *, doublereal *, doublereal *, integer *, 
	    doublereal *, integer *, integer *, doublereal *, integer *, 
	    doublereal *, S_fp, S_fp, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *,
	     integer *, doublereal *, doublereal *, doublereal *, doublereal *
	    , doublereal *, integer *, doublereal *, doublereal *, doublereal 
	    *, integer *, doublereal *, integer *, integer *, logical *, 
	    doublereal *, doublereal *, doublereal *, doublereal *), evrest_(
	    doublereal *, doublereal *, integer *, integer *, integer *, S_fp,
	     integer *, doublereal *, doublereal *, doublereal *, doublereal *
	    , integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, doublereal *, integer *, integer *, S_fp,
	     doublereal *, doublereal *, integer *, integer *, logical *), 
	    output_(integer *, integer *, integer *, doublereal *, doublereal 
	    *, integer *, doublereal *);
    static logical add;
    static integer bnd;
    static logical del;
    static integer ind;
    static doublereal tau, psi;
    static integer lmt;
    static doublereal sss;


/*   ***************************************************************** */
/*   * NLSNIP IS DEVELOPED BY PER LINDSTR@M AND PER-$KE WEDIN AT THE * */
/*   * INSTITUTE OF INFORMATION PROCESSING UNIVERSITY OF UME$,       * */
/*   * S-90187 UME$, SWEDEN                                          * */
/*   ***************************************************************** */

/*   PURPOSE... */

/*     SOLVE THE NONLINEAR LEAST SQUARES PROBLEM */

/*     MINIMIZE  0.5* II F(X) II**2 */
/*        X */
/*        SUBJECT TO THE NONLINEAR CONSTRAINTS */
/*     H (X) = 0     I=1,2,.....,P    (P<=N) */
/*      I */

/*     H (X)   >= 0  J=P+1,....,L  (L>=P) , (M+P)>= N */
/*      J */

/*     WHERE F(X) IS M-DIMENSIONAL AND X IS N-DIMENSIONAL */

/*     THE DIMENSION OF SOME ARRAYS IN THE CALLING PROGRAM DEPEND */
/*     ON BND=MIN(L,N) */

/*     ON ENTRY@D */

/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING A FIRST APPROXIMATION OF THE SOLUTION POINT */
/*     N       INTEGER SCALAR CONTAINING THE NUMBER OF PARAMETERS */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS IN F(X) */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINING THE TOTAL NUMBER OF CONSTRAINTS */
/*     TOL     REAL SCALAR CONTAING A SMALL VALUE >0 USED TO */
/*             DETERMINE PSEUDO RANKS OF MATRICES */
/*     W()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING VALUES >=0 USED AS FIRST PENALTY CONSTANTS */
/*             IN THE MERIT FUNCTION. IF AN ELEMENT IS <=0.0 THE FIRST */
/*             PENALTY CONSTANT IS CHOSEN AUTOMATICALLY */
/*     EPSREL  THEY ARE ALL REAL SCALARS CONTAINING SMALL POSITIVE */
/*     EPSABS  VALUES USED TO TEST CONVERGENCE */
/*     EPSX    RECOMMENDED VALUES ARE */
/*     EPSH    EPSREL=EPSX=EPSH=10**(-3),  EPSABS=10**(-10) */
/*     IPRINT  INTEGER SCALAR CONTAINING A CODE TO CONTROL */
/*             PRINTING INSIDE THIS ROUTINE */
/*             IF IPRINT<=0 THEN NO PRINTING IS DONE */
/*                         ELSE CERTAIN INFORMATION IS PRINTED EVERY */
/*                              IPRINT ITERATION ON UNIT NOUT */
/*     NOUT    INTEGER SCALAR CONTAINING LOGICAL UNIT NUMBER WHERE */
/*             PRINTING CAN BE DONE (RECORD LENGHT MUST NOT BE LESS */
/*             THAN 125) */
/*     MAXIT   INTEGER SCALAR CONTAINING MAXIMUM ALLOWED ITERATIONS */
/*     NORM    INTEGER SCALAR CONTAINING A CODE THAT CONTROLS WHICH */
/*             NORM THAT SHOULD BE USED WHEN PENALTY WEIGHTS ARE UPDATED */
/*             = 0 IF MAXIMUM NORM */
/*             = 2 IF EUCLIDEAN NORM */
/*     SCALE   INTEGER SCALAR =1 IF SCALING SHOULD BE DONE OF THE */
/*             ROWS IN THE CONSTRAINT MATRIX (MATRIX A) */
/*             =0 IF NO SCALING SHOULD BE DONE */
/*     SEC     LOGICAL SCALAR = TRUE IF SECOND DERIVATIVES ARE ALLOWED */
/*             =FALSE IF SECOND DERIVATIVES ARE NOT ALLOWED */
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

/*     ON RETURN@D    AND EXIT.NE.-1  AND  EXIT.GE.-10 */

/*     X()     CONTAINS THE TERMINATION POINT (HOPEFULLY THE SOLUTION) */
/*     P       CONTAINS NUMBER OF ACTIVE CONSTRAINTS AT X */
/*     W()     CONTAINS THE FINAL PENALTY CONSTANTS */
/*     EXIT    INTEGER SCALAR-CONTAINS A TERMINATION CODE REPRESENTING */
/*             THE TERMINATION CRITERIA DESCRIBED BELOW */
/*             =10000 IF CRITERION 4) IS SATISFIED */
/*             = 2000 IF CRITERION 5) IS SATISFIED */
/*             =  300 IF CRITERION 6) IS SATISFIED */
/*             =   40 IF CRITERION 7) IS SATISFIED */
/*             =    X IF CRITERION 8) IS SATISFIED */
/*                    WHERE X= 0,1,2,3 OR 4 */
/*             EXIT CAN ALSO BE A SUM OF THE ABOVE CODES TO INDICATE */
/*             THAT MORE THAN ONE CRITERION IS SATISFIED */

/*             IF ANY OF THE ABNORMAL CRITERIA IS SATISFIED EXIT IS */
/*             SET TO */

/*             =   -1 IF IMPROPER DATA ON ENTRY TO THIS ROUTINE */
/*             =   -2 IF CRITERION  9) IS SATISFIED */
/*             =   -3 IF convergence to a non-feasible point!!!! */
/*             =   -4 IF CRITERION 11) IS SATISFIED */
/*             =   -5 IF CRITERION 12) IS SATISFIED */
/*             =   -6 IF CRITERION 13) IS SATISFIED */
/*             =   -7 IF RESIDUALS OR CONSTRAINTS ARE UNCOMPUTABLE AT X */
/*             =   -8 IF MORE THAN N CONSTRAINTS IN FIRST WORKING */
/*                    SET (CHOOSE A BETTER STARTING POINT) */
/*             =   -9 IF TOO MANY(>5) NEWTON STEPS HAVE BEEN USED */
/*             =  -10 if not possible to satisfy the constraints */
/*             < -10  TERMINATION DUE TO USER STOP INDICATOR */
/*     THERE ARE CONVERGENCE CRITERIA AND ABNORMAL TERMINATION */
/*     CRITERIA */
/*     THE CONVERGENCE CRITERIA ARE ONLY TESTED IF THE LATEST STEP */
/*     WAS TAKEN ALONG A GAUSS-NEWTON DIRECTION WITH FULL PSEUDO RANK */
/*     OR IF THE METHOD OF NEWTON HAS BEEN USED IN THE LATEST STEP. */
/*     OF COURCE THE STEP MUST NOT BE A RESTART STEP WHEN CONVERGENCE */
/*     SHOULD BE TESTED */

/*     A SEARCH DIRECTION USING GAUSS-NETON'S METHOD IS COMPUTED */
/*     BY SOLVING FOR DX    A*DX = -H */
/*                          C*DX APPR.= -F */
/*     A FIRST ORDER ESTIMATE OF THE LAGRANGE MULTIPLIERS (V) HAS BEEN */
/*     COMPUTED BY SOLVING FOR V     T */
/*                                  A *V APPR.= G (THE GRADIENT ) */
/*     WHERE */
/*                     T                 -1   T */
/*           (L@D0) = P1 *A*Q1      DY1= L  *P1 *(-H) */
/*                                       T */
/*           (C1@DC2) = C*Q1      (U) = Q3 *C2*P3 */
/*                               (0) */
/*                    T */
/*           D=(D1)=Q3 *(-F-C1*DY1) */
/*             (D2) */

/*     A NECESSARY CONDITION FOR CONVERGENCE IS THAT CONDITIONS */
/*     1)-3) BELOW ARE SATISFIED */

/*     THE CONVERGENCE CRITERIA ARE */

/*      1) II H(X) II < EPSH   (WHERE ONLY CONSTRAINTS IN THE WORKING */
/*             T                SET ARE CONSIDERED) */
/*      2) II A *V -G II < EPSREL*(1.0+II G(X) II) */
/*      3) SIGMIN >= EPSREL*ABSVMX */
/*                >= EPSREL*(1+II F(X) II**2)  (IF 1 INEQUALITY) */
/*      4) II D1 II**2 <= EPSREL**2*II F(X) II**2 */
/*      5) II F(X) II**2 <= EPSABS**2 */
/*      6) II X(K-1)-X(K) II < EPSX*II X(K) II   ( TIME STEP K) */
/*      7) DSQRT(DRELPR)/II DX(GN) II >0.25 (GN=GAUSS NEWTON) */
/*      8) THE LAST DIGIT IN THE CONVERGENCE CODE (SEE ABOVE) INDICATES */
/*         HOW THE LAST STEPS WERE COMPUTED */
/*        = 0 NO TROUBLE (GAUSS-NEWTON THE LAST 3 STEPS) */
/*        = 1 RANKA<T OR RANKC2<(N-T) AT THE TERMINATION POINT */
/*            NO 2@DND DERIVATIVES HAS BEEN USED */
/*        = 2 THE METHOD OF NEWTON WAS USED (AT LEAST) IN THE LAST STEP */
/*        = 3 THE 2@DND BUT LAST STEP WAS SUBSPACE MINIMIZATION STEP */
/*            BUT THE LAST TWO WERE GAUSS-NEWTON STEPS */
/*        = 4 THE STEPLENGTH WAS NOT UNIT IN BOTH THE LAST 2 STEPS */

/*     THE ABNORMAL TERMINATION CRITERIA ARE */

/*      9) NUMBER OF ITERATIONS EXCEEDS THE MAXIMUM NUMBER */
/*     10) THE REDUCED HESSIAN EMANATING FROM THE METHOD OF NEWTON */
/*         IS NOT POSITIVE DEFINTE */
/*     11) THE USER HAS NOT ALLOWED USE OF 2@DND DERIVATIVES (NEWTON) */
/*     12) UNDAMPED NEWTON STEP FAILS */
/*     13) THE LATEST DX(GN) IS NOT A DESCENT DIRECTION TO THE MERIT */
/*         FUNCTION (PROBABLY CAUSED BY MISSCALCULATED JACOBIAN) */

/*     PHI     REAL SCALAR-CONTAINS THE VALUE OF THE OBJECTIVE */
/*             FUNCTION AT THE POINT X */
/*     K       INTEGER SCALAR-CONTAINS NUMBER OF ITERATIONS UNTIL */
/*             TERMINATION */
/*     FUNCEV  INTEGER SCALAR-CONTAINS TOTAL NUMBER OF FUNCTION */
/*             EVALUATIONS DONE */
/*     JACEV   INTEGER SCALAR-CONTAINS NUMBER OF EVALUATIONS OF */
/*             JACOBIANS */
/*     SECEV   INTEGER SCALAR-CONTAINS NUMBER OF FUNCTION EVALUATIONS */
/*             CAUSED BY 2@DND DERIVATIVE COMPUTATION USING DIFFERENCES */
/*     LINEV   INTEGER SCALAR-CONTAINS NUMBER OF FUNCTION EVALUATIONS */
/*             CAUSED BY THE LINESEARCH ROUTINE */
/*     RANKA   INTEGER SCALAR-CONTAINS ESTIMATED PSEUDO RANK OF THE */
/*             CONSTRAINT MATRIX (MATRIX A) AT THE TERMINATION POINT */
/*             RANKA=P IF FULL RANK */
/*     RANK    INTEGER SCALAR-CONTAINS ESTIMATED PSEUDO RANK OF THE */
/*             COMPOUND MATRIX (A) AT THE TERMINATION POINT */
/*                             (C) */
/*             RANK=N IF FULL RANK */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+L */
/*             CONTAINS THE VALUE OF THE RESIDUALS AT THE POINT X */
/*             AS THE FIRST M ELEMENTS */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINS THE VALUE OF THE CONSTRAINTS AT THE POINT X */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION BND+L */
/*             CONTAINS INDECES FOR ACTIVE CONSTRAINTS AT THE POINT X */
/*             SUCH THAT ACTIVE(I) I=1,2,....,P ARE THE INDECES FOR THE */
/*             ACTIVE CONSTRAINTS */
/*     SPEED   REAL SCALAR-CONTAINS AN ESTIMATE OF THE CONVERGENCE */
/*             FACTOR (SHOULD BE <1 , >0 ) */

/*     WORKING AREAS@D */

/*     P1(),P2() INTEGER SINGLY SUBSCRIPTED ARRAYS OF DIMENSION BND */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*     INACT(),P4() INTEGER SINGLY SUBSCRIPTED ARRAYS OF DIMENSION L */
/*     WOLD()      REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*     S()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION MAX(L,N) */
/*     B(),D1(),D2(),V() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION BND */
/*     G(),PIVOT(),D3() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION N */
/*     DIAG(),DX(),U() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION N */
/*     D(),V1(),V2() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION M+L */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             MDC>=M */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             MDA>=L */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             MDF>=N (IF ONLY EQUALITY CONSTRAINTS FMAT CAN BE A SCALAR) */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             MDG>=N */


/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 RESPECTIVELY. */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     THIS PROGRAM PACKAGE USES THE FOLLOWING LINPACK AND BLAS ROUTINES */

/*     DPOSL,DCHDC,DNRM2,DAXPY,DDOT,DSWAP */


/*     INTERNAL VARIABLES */


/*     VALIDATE INPUT VALUES */

    /* Parameter adjustments */
    --u;
    --dx;
    --pivot;
    --g;
    --d3;
    --p3;
    --x;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    --w;
    --f;
    --h__;
    --active;
    --p1;
    --p2;
    --inact;
    --p4;
    --b;
    --d1;
    --d2;
    --diag;
    --v;
    --s;
    --wold;
    --d__;
    --v1;
    --v2;

    /* Function Body */
    *exit = 0;
    if (*m + *l < *n || *n <= 0 || *m <= 0 || *mdc < *m) {
	*exit = -1;
    }
    if (*p > *n || *p < 0 || *l < *p || *mda < *l) {
	*exit = -1;
    }
    if (*mdg < *n || *maxit <= 0 || *norm < 0 || *norm > 2) {
	*exit = -1;
    }
    if (*l > *p && *mdf < *n) {
	*exit = -1;
    }
    if (*tol < 0. || *scale < 0 || *epsrel < 0.) {
	*exit = -1;
    }
    if (*epsabs < 0. || *epsx < 0. || *epsh < 0.) {
	*exit = -1;
    }
    if (*epsrel + *epsabs + *epsx + *epsh <= 0.) {
	*exit = -1;
    }

/*     COMPUTE DRELPR = DOUBLE RELATIVE PRECISION */

    releps_(&machin_1.drelpr);

/*     because of a local area in the subroutine eucnrm */

    if (*n > 100) {
	*norm = 0;
    }

/*     INITIATE VARIABLES */

    *k = 0;
    negdir_1.ifree = 0;
    bnd = min(*l,*n);
    error = 0;
    *funcev = 0;
    *jacev = 0;
    *secev = 0;
    *linev = 0;
    back_1.nrrest = 0;
    restar = FALSE_;
    prec_1.kodkm2 = 1;
    prec_1.alfkm2 = 1.;
    prec_1.kodkm1 = 1;
    tau = *tol;
    xdiff = enlsip_dnrm2_(n, &x[1], &c__1) * 2. * *epsx;

/*     EVALUATE AT USER SUPPLIED STARTING POINT */

    ctrlf = 1;
    (*ffunc)(&x[1], n, &f[1], m, &ctrlf, &c__[c_offset], mdc);
    ctrlh = 1;
    (*hfunc)(&x[1], n, &h__[1], l, &ctrlh, &a[a_offset], mda);
    ++(*funcev);
    if (-1 == ctrlf || -1 == ctrlh) {
	*exit = -7;
    }
    if (*exit < 0) {
	return 0;
    }

/*     CONSTRUCT FIRST WORKING SET AND INITIATE PENALTY */
/*     CONSTANTS (IF MORE THAN N CONSTRAINTS<=0 EXIT IS SET TO -8) */

    inialc_(p, l, &h__[1], &active[1], &t, &bnd, norm, &inact[1], &lmt, &w[1],
	     &wold[1], exit);
    if (*exit < 0) {
	return 0;
    }
    prec_1.rkckm1 = *n - t;
    prec_1.alfkm1 = 1.;
    back_1.lattry = *n;
    back_1.bestpg = 0.;
    prec_1.rkakm1 = t;
    prec_1.tkm1 = t;
    add = FALSE_;
/* Computing 2nd power */
    d__1 = enlsip_dnrm2_(m, &f[1], &c__1);
    fsum = d__1 * d__1;
    prec_1.fsqkm1 = fsum;
    prec_1.hsqkm1 = sacth_(&h__[1], &active[1], &t);

/*     START OF MAIN LOOP FOR THE ITERATION */

L10:
    if (-5 == error || -3 == error) {
	error = 0;
    }
    if (restar) {
	goto L20;
    }

/*     COMPUTE JACOBIANS AT THE CURRENT POINT AND SET B=-H */

    newpnt_(&x[1], n, &h__[1], l, &f[1], m, (S_fp)hfunc, (S_fp)ffunc, mda, 
	    mdc, funcev, &a[a_offset], &c__[c_offset], &b[1], &d__[1], &error)
	    ;
    if (error < -10) {
	goto L40;
    }
    ++(*jacev);

/*     compute the 2-norm of A(T)**h for the active set */

    athnrm = 0.;
    if (t == 0) {
	goto L17;
    }
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	sss = 0.;
	i__2 = t;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    ii = active[i__];
	    sss += a[ii + j * a_dim1] * h__[ii];
/* L13: */
	}
	athnrm += sss * sss;
/* L15: */
    }
    athnrm = sqrt(athnrm);
L17:

/*     MOVE CONSTRAINTS IN THE CURRENT WORKING SET AT THE TOP */
/*     OF THE ARRAY B AND THE CORRESPONDING GRADIENTS AT THE */
/*     TOP OF ARRAY A */

    equal_(&b[1], l, &a[a_offset], mda, n, &active[1], &t, p, &p4[1]);

/*     COMPUTE THE GRADIENT OF THE OBJECTIVE FUNCTION */

    grad_(&c__[c_offset], mdc, m, n, &f[1], &g[1]);
    gnorm = enlsip_dnrm2_(n, &g[1], &c__1);

/*     SCALE THE MATRIX A BY MAKING THE ROWS OF A HAVING */
/*     UNIT LENGTH IF INTERNAL SCALING IS INDICATED */

    evscal_(scale, &a[a_offset], mda, &t, n, &b[1], &diag[1]);
    xnorm = enlsip_dnrm2_(n, &x[1], &c__1);
    *phi = fsum * .5;

/*     DETERMINE THE CURRENT WORKING SET AND CORRESPONDING GN-SEARCH */
/*     DIRECTION (DX) */

    wrkset_(&a[a_offset], mda, &t, p, n, &g[1], &b[1], &tau, mdf, scale, k, &
	    diag[1], &active[1], &bnd, &inact[1], &lmt, &h__[1], &gndnrm, &p4[
	    1], &c__[c_offset], mdc, m, &f[1], mdg, &x[1], (S_fp)hfunc, (S_fp)
	    ffunc, funcev, jacev, &p2[1], &p3[1], &dx[1], &v1[1], &d2[1], &d3[
	    1], &rankc2, &d1nrm, &dnrm, &b1nrm, &d__[1], &gmat[gmat_offset], &
	    p1[1], &v[1], &d1[1], &fmat[fmat_offset], ranka, &gres, &nohous, &
	    time, &del, &pivot[1], &v2[1], &s[1], &u[1]);
/*      write(10,*) 'Search direction' */
/*      write(10,*) (dx(i),i=1,n) */

/*     COMPUTE THE SQUARE SUM OF THE CONSTRAINTS IN THE */
/*     WORKING SET */

    hsum = sacth_(&h__[1], &active[1], &t);
    gndnrm = dnrm;
    dxnorm = enlsip_dnrm2_(n, &dx[1], &c__1);
    alfnoi = sqrt(machin_1.drelpr) / (dxnorm + machin_1.drelpr);

/*     COMPUTE SIGMIN=MIN(V(I)) WHERE I CORRRESPOND TO */
/*     CONSTRAINTS DEFINED AS INEQUALITIES */
/*     ABSVMX=MAX(DABS(V(I))  , I=1,2,.....,T */

    minmax_(p, &t, &v[1], scale, &diag[1], &sigmin, &absvmx);

/*     IF THE METHOD OF NEWTON WAS USED IN THE LATEST STEP */
/*     IT MAY HAPPEN THAT IT FAILS */
/*     IF IT FAILS GN is used for at least 5 steps */

    if (prec_1.kodkm1 != 2) {
	goto L20;
    }
    succes = prec_1.d1km1 - d1nrm;
    if (prec_1.rkckm1 != rankc2) {
	goto L20;
    }
    if (succes < 0.) {
	negdir_1.ifree = 5;
	back_1.nrrest = 0;
	goto L10;
    }
L20:

/*     CHECK MAIN TERMINATION CRITERIA */

    tercri_(ranka, &rankc2, &error, p, &t, n, &restar, &del, maxit, k, &fsum, 
	    &d1nrm, &hsum, &gres, &xnorm, &gnorm, &alfnoi, &h__[1], &inact[1],
	     &lmt, &xdiff, epsabs, epsrel, epsx, epsh, &sigmin, &absvmx, &
	    wold[1], &x[1], &athnrm, &whnrm2, exit);

/*     IF EXIT<>0 THE ITERATION IS FINISHED */

    if (*exit != 0) {
	goto L30;
    }

/*     ANALYSE THE PAST AND RECOMPUTE THE SEARCH DIRECTION */
/*     IF NECESSARY */

    analys_(k, &restar, &code, &fsum, &d1nrm, &dnrm, &c__[c_offset], mdc, m, 
	    n, &rankc2, &d__[1], &f[1], &p3[1], &d3[1], &active[1], &v[1], &
	    inact[1], &lmt, &p4[1], &time, &a[a_offset], mda, p, &t, ranka, &
	    b1nrm, &hsum, &nohous, &d1[1], &p1[1], &d2[1], &p2[1], &b[1], &
	    h__[1], l, &fmat[fmat_offset], mdf, &pivot[1], &gmat[gmat_offset],
	     mdg, (S_fp)ffunc, (S_fp)hfunc, &x[1], sec, &add, &del, scale, &
	    diag[1], &dx[1], &dxnorm, &v1[1], epsrel, &error, &eval, &betak, &
	    dima, &dimc2, &v2[1], &u[1]);
    if (error < -10) {
	goto L40;
    }

/*     ACCUMULATE FUNCTION EVALUATIONS */

    *funcev += eval;
    *secev += eval;

/*     TEST SOME ABNORMAL TERMINATION CRITERIA */
/*     ERROR=-3 IF NOT POSSITIVE DEFINITE HESSIAN */
/*          =-4 IF NOT ALLOWED TO USE 2@DND DERIVATIVES */

    if (error == -3) {
	negdir_1.ifree = 5;
	back_1.nrrest = 0;
	goto L10;
    }
    if (error == -4) {
	goto L20;
    }

/*     SAVE THE CURRENT POINT */

    savexk_(&x[1], n, &u[1]);
    if (code == 2 && back_1.nrrest == 1) {
	savexk_(&x[1], n, &wold[1]);
    }

/*     COMPUTE A STEPLENGTH AND TAKE A STEP USING */
/*     A MERIT FUNCTION */

    stplng_(&restar, &x[1], &a[a_offset], mda, &dx[1], n, &f[1], &v1[1], m, &
	    fsum, (S_fp)ffunc, &rankc2, &code, &h__[1], (S_fp)hfunc, &active[
	    1], &t, &inact[1], &p4[1], &lmt, l, &w[1], &wold[1], &dima, norm, 
	    &nohous, &psi, &alpha, &alflow, &alfupp, &eval, &xdiff, &error, &
	    ind, &whnrm2, &d__[1], &v2[1], &s[1], &g[1]);
    if (error < -10) {
	goto L40;
    }

/*     ACCUMULATE FUNCTION EVALUATIONS */

    *funcev += eval;
    *linev += eval;

/*     POSSIBLY A RESTART IS DONE */
/*     IF NO RESTART IS DONE VARIABLES REPRESENTING */
/*     THE PAST ARE UPDATED */

    evrest_(&x[1], &u[1], n, m, k, (S_fp)ffunc, funcev, &f[1], &d1nrm, &dnrm, 
	    &fsum, &dimc2, &code, &dxnorm, &betak, &alpha, &alflow, &active[1]
	    , &h__[1], l, &t, (S_fp)hfunc, &b1nrm, &hsum, &dima, &error, &
	    restar);
    if (error < -10) {
	goto L40;
    }
    add = FALSE_;
    if (restar || -1 == error || -5 == error) {
	goto L10;
    }

/*     POSSIBLY SOME CONSTRAINTS ARE ADDED TO THE WORKING SET */

    evadd_(&h__[1], &active[1], &t, &bnd, p, &inact[1], &lmt, &ind, k, &add);

/*     PRINT SOME INFORMATION FOR LATEST STEP */

    output_(iprint, k, nout, &gres, &w[1], &active[1], speed);

/*     REPEAT FROM START OF ITERATION LOOP */

    goto L10;
L30:
/* Computing MAX */
/* Computing 2nd power */
    d__3 = machin_1.drelpr;
    d__1 = prec_1.betkm1, d__2 = d__3 * d__3;
    *speed = (d1nrm + b1nrm) / max(d__1,d__2);
    *p = t;
    *rank = rankc2 + *ranka;
    return 0;
L40:

/*     USER STOP INDICATOR DETECTED */

    *exit = error;
    return 0;
} /* nlsnip_ */


/*     DOUBLE PRECISION VERSION 910326 */

/* TERCRI */
/* Subroutine */ int tercri_(integer *ranka, integer *rankc2, integer *error, 
	integer *p, integer *t, integer *n, logical *restar, logical *del, 
	integer *maxit, integer *k, doublereal *fsum, doublereal *d1nrm, 
	doublereal *hsum, doublereal *gres, doublereal *xnorm, doublereal *
	gnorm, doublereal *alfnoi, doublereal *h__, integer *inact, integer *
	q, doublereal *xdiff, doublereal *epsabs, doublereal *epsrel, 
	doublereal *epsx, doublereal *epsh, doublereal *sigmin, doublereal *
	absvmx, doublereal *wold, doublereal *x, doublereal *athnrm, 
	doublereal *whnrm2, integer *exit)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer feas, i__, j;
    static doublereal factor;


/*     CHECK IF ANY OF THE TERMINATION CRITERIA ARE SATISFIED */
/*     THERE ARE CONVERGENCE CRITERIA AND ABNORMAL TERMINATION */
/*     CRITERIA */
/*     THE CONVERGENCE CRITERIA ARE ONLY TESTED IF THE LATEST STEP */
/*     WAS TAKEN ALONG A GAUSS-NEWTON DIRECTION WITH FULL PSEUDO RANK */
/*     OR IF THE METHOD OF NEWTON HAS BEEN USED IN THE LATEST STEP */
/*     OF COURCE THE STEP MUST NOT BE A RESTART STEP IF CONVERGENCE */
/*     SHOULD BE TESTED */

/*     A SEARCH DIRECTION USING GAUSS-NETON'S METHOD IS COMPUTED */
/*     BY SOLVING FOR DX    A*DX = -H */
/*                          C*DX APPR.= -F */
/*     A FIRST ORDER ESTIMATE OF THE LAGRANGE MULTIPLIERS (V) HAS BEEN */
/*     COMPUTED BY SOLVING FOR V     T */
/*                                  A *V APPR.= G (THE GRADIENT ) */
/*     WHERE */
/*                     T                 -1   T */
/*           (L@D0) = P1 *A*Q1      DY1= L  *P1 *(-H) */
/*                                       T */
/*           (C1@DC2) = C*Q1      (U) = Q3 *C2*P3 */
/*                               (0) */
/*                    T */
/*           D=(D1)=Q3 *(-F-C1*DY1) */
/*             (D2) */

/*     THE CONVERGENCE CRITERIA ARE */

/*      1) II H(X) II < EPSH   (WHERE ONLY CONSTRAINTS IN THE WORKING */
/*                              SET ARE CONSIDERED) */
/*      1.5) ALL INACTIVE CONSTRAINTS MUST BE GREATER THAN ZERO */
/*             T */
/*      2) II A *V -G II < DSQRT(EPSREL)*(1.0+II G(X) II) */
/*      3) SIGMIN >= EPSREL*ABSVMX */
/*                >= EPSREL*(1+II F(X) II**2)  (IF 1 INEQUALITY) */
/*      4) II D1 II**2 <= EPSREL**2*II F(X) II**2 */
/*      5) II F(X) II**2 <= EPSABS**2 */
/*      6) II X(K-1)-X(K) II < EPSX*II X(K) II   ( TIME STEP K) */
/*      7) DSQRT(DRELPR)/II DX(GN) II >0.25 (GN=GAUSS NEWTON) */
/*      8) THE LAST DIGIT IN THE CONVERGENCE CODE (SEE BELOW) INDICATES */
/*         HOW THE LASTS STEPS WERE COMPUTED */
/*        = 0 NO TROUBLE (GAUSS-NEWTON THE LAST 3 STEPS) */
/*        = 1 RANKA<>T OR RANKC2<>(N-T) AT THE TERMINATION POINT */
/*            NO 2@DND DERIVATIVES HAS BEEN USED */
/*        = 2 THE METHOD OF NEWTON WAS USED (AT LEAST) IN THE LAST STEP */
/*        = 3 THE 2@DND BUT LAST STEP WAS SUBSPACE MINIMIZATION STEP */
/*            BUT THE LAST TWO WERE GAUSS-NEWTON STEPS */
/*        = 4 THE STEPLENGTH WAS NOT UNIT IN BOTH THE LAST 2 STEPS */

/*     THE ABNORMAL TERMINATION CRITERIA ARE */

/*      9) NUMBER OF ITERATIONS EXCEEDS THE MAXIMUM NUMBER */
/*     10) Convergence to a non-feasible point. */
/*     11) THE USER HAS NOT ALLOWED USE OF 2@DND DERIVATIVES (NEWTON) */
/*     12) UNDAMPED NEWTON STEP FAILS */
/*     13) THE LATEST DX(GN) IS NOT A DESCENT DIRECTION TO THE */
/*         MERIT FUNCTION (PROBABLY CAUSED BY MISSCALCULATED JACOBIAN) */

/*     ON ENTRY@D */

/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF THE MATRIX */
/*             A IN CURRENT STEP */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF THE MATRIX */
/*             C2 IN CURRENT STEP */
/*     ERROR   INTEGER SCALAR CONTAINING */
/*             =-1 IF NO DESCENT DIRECTION */
/*             =-2 IF WE ARE COMPUTING AT NOISE LEVEL */
/*             =-3 IF NOT POSITIVE DEFINITE HESSIAN FROM NEWTON */
/*             =-4 IF NO USE OF 2@DND DERIVATIVES IS ALLOWED */
/*             =-5 IF UNDAMPED NEWTON FAILS */
/*             =-9 IF TOO MANY(>5) NEWTON STEPS */
/*             = 0 OTHERWISE */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     RESTAR  LOGICAL SCALAR =TRUE IF RESTART IS DONE AT CURRENT POINT */
/*             =FALSE IF NO RESTART HAS BEEN DONE */
/*     DEL     LOGICAL SCALAR = TRUE IF A CONSTRAINT IS DELETED */
/*             AT THE CURRENT POINT. = FALSE OTHERWISE */
/*     MAXIT   INTEGER SCALAR CONTAINING MAXIMUM ITERATIONS ALLOWED */
/*     K       INTEGER SCALAR CONTAINING CURRENT ITERATION NUMBER */
/*     FSUM    REAL SCALAR CONTAINING SQUARE SUM OF THE RESIDUALS */
/*     D1NRM   REAL SCALAR CONTAINING  II D1 II */
/*     HSUM    REAL SCALAR CONTAINING SQUARE SUM OF THE CURRENT */
/*             CONSTRAINTS                  T */
/*     GRES    REAL SCALAR CONTAINING   II A *V-G II */
/*     XNORM   REAL SCALAR CONTAINING  II X(K) II */
/*     GNORM   REAL SCALAR CONTAINING NORM OF THE GRADIENT OF */
/*             THE OBJECTIVE FUNCTION */
/*     ALFNOI  REAL SCALAR CONTAINING DSQRT(DRELPR)/II DX(GN) II */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR THE INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAINING NO. OF INACTIVE CONSTRAINTS */
/*     XDIFF   REAL SCALAR CONTAINING  II X(K-1)-X(K) II */
/*     EPSABS  REAL SCALARS CONTAINING SMALL NON NEGATIVE VALUES */
/*     EPSREL  USED IN CONVERGENCE TESTS */
/*     EPSX */
/*     EPSH */
/*     SIGMIN  REAL SCALAR CONTAINING SMALLEST (>0) LAGRANGE MULTIPLIER */
/*             ESTIMATE CORRESPONDING TO INEQUALITY CONSTRAINTS */
/*     ABSVMX  REAL SCALAR CONTAINING LARGEST ABSOLUTE LAGRANGE */
/*             MULTIPLIER ESTIMATES */
/*     WOLD()  REAL SINGLYSUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING NEWTON INITIAL X-POINT WHEN */
/*             ERROR = -9. OTHERWISE UNDEFINED */

/*     ON RETURN@D */

/*     A NECESSARY CONDITION FOR CONVERGENCE IS THAT CONDITIONS */
/*     1)-3) HOLD TRUE */

/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING WOLD WHEN ERROR = -9. OTHERWISE */
/*             UNDEFINED */
/*     EXIT    INTEGER SCALAR CONTAINING */
/*             =    0 IF NO TERMINATION CRITERION IS SATISFIED */
/*             =10000 IF CRITERION  4) IS SATISFIED */
/*             = 2000 IF     @      5)       @ */
/*             =  300 IF     @      6)       @ */
/*             =   40 IF     @      7)       @ */
/*             =    X IF     @      8)       @ */
/*                    WHERE X CAN BE   0,1,2,3 OR 4 */
/*             = -YYYYX if   #     10)       # */
/*                    where YYYYX is one of the convergence codes above. */
/*             =   -2 IF CRITERION  9) IS SATISFIED */
/*             =   -3 Not used */
/*             =   -4 IF     @     11)       @ */
/*             =   -5 IF     @     12)       @ */
/*             =   -6 IF     @     13)       @ */
/*             =   -9 IF TOO MANY (>5) NEWTON STEPS */
/*             =  -10 if not possible to satisfy the constraints */

/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --x;
    --wold;
    --h__;
    --inact;

    /* Function Body */
    *exit = 0;
    feas = 1;

/*     THE CONVERGENCE CRITERIA ARE NOT CHECKED IF A RESTART WAS */
/*     DONE AT THE CURRENT POINT OR IF CURRENT STEP IS NOT A FULL */
/*     PSEUDO RANK STEP */

    if (*restar || negdir_1.ifree > 3) {
	goto L60;
    }
    if (-1 == prec_1.kodkm1 && *alfnoi <= .25) {
	goto L60;
    }
    if (*error < 0 && -2 != *error) {
	goto L60;
    }

/*     CHECK NECESSARY CONDITIONS */

    if (*del) {
	goto L60;
    }
    if (sqrt(*hsum) > *epsh) {
	goto L60;
    }
    if (*gres > sqrt(*epsrel) * (*gnorm + 1.)) {
	goto L60;
    }
    if (*q <= 0) {
	goto L5;
    }
    i__1 = *q;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = inact[i__];
	if (h__[j] <= 0.) {
	    feas = -1;
	}
/* L1: */
    }
L5:
    if (*t == *p) {
	goto L10;
    }
    if (*t == 1) {
	factor = *fsum + 1.;
    }
    if (*t > 1) {
	factor = *absvmx;
    }
    if (*sigmin < *epsrel * factor) {
	goto L60;
    }
L10:

/*     CHECK THE SUFFICIENT CONDITIONS */

/*     CRITERION NO. 4 */

/* Computing 2nd power */
    d__1 = *d1nrm;
/* Computing 2nd power */
    d__2 = *epsrel;
    if (d__1 * d__1 <= d__2 * d__2 * *fsum) {
	*exit += 10000;
    }

/*     CRITERION NO. 5 */

/* Computing 2nd power */
    d__1 = *epsabs;
    if (*fsum <= d__1 * d__1) {
	*exit += 2000;
    }

/*     CRITERION NO. 6 */

    if (*xdiff < *epsx * *xnorm) {
	*exit += 300;
    }

/*     CRITERION NO. 7 */

    if (*alfnoi > .25 || -2 == *error) {
	*exit += 40;
    }
    if (*exit == 0) {
	goto L60;
    }

/*     CRITERION NO. 8 */

    if (prec_1.kodkm1 == 1) {
	goto L20;
    }
    *exit += 2;
    *exit *= feas;
    return 0;
L20:
    if (*ranka == *t && *rankc2 == *n - *t) {
	goto L30;
    }
    ++(*exit);
    *exit *= feas;
    return 0;
L30:
    if (prec_1.kodkm2 == 1 || *k < 2) {
	goto L40;
    }
    *exit += 3;
    *exit *= feas;
    return 0;
L40:
    if (! ((d__1 = prec_1.alfkm2 - 1., abs(d__1)) <= .01) && (d__2 = 
	    prec_1.alfkm1 - 1., abs(d__2)) <= .01) {
	*exit += 4;
    }
    *exit *= feas;
    return 0;
L60:

/*     CHECK ABNORMAL TERMINATION CRITERIA */

/*     CRITERION NO. 9 */

    if (*k + back_1.nrrest <= *maxit) {
	goto L70;
    }
    *exit = -2;
    return 0;
L70:
    if (negdir_1.ifree > 0) {
	*exit = 0;
	return 0;
    }

/*     CRITERION NO. 13 */

    if (*error != -1) {
	goto L80;
    }
    *exit = -6;
    return 0;
L80:

/*     test if impossible to satisfy the constraints */

    if (*xdiff > *epsx * 10. || *athnrm > *epsh * 10. || *whnrm2 < 1.) {
	goto L85;
    }
    *exit = -10;
    return 0;
L85:

/*     CRITERION NO. 10)-12) */

    if (*error >= -5 && *error <= -3) {
	*exit = *error;
    }
    if (-9 != *error) {
	return 0;
    }
    *exit = *error;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	x[i__] = wold[i__];
/* L90: */
    }
    return 0;
} /* tercri_ */

/*     DOUBLE PRECISION VERSION 880329 */

/* STPLNG */
/* Subroutine */ int stplng_(logical *restar, doublereal *x, doublereal *a, 
	integer *mda, doublereal *dx, integer *n, doublereal *f, doublereal *
	v1, integer *m, doublereal *fsum, S_fp ffunc, integer *rankc2, 
	integer *code, doublereal *h__, S_fp hfunc, integer *active, integer *
	t, integer *inact, integer *p4, integer *q, integer *l, doublereal *w,
	 doublereal *wold, integer *dima, integer *norm, integer *nohous, 
	doublereal *psi, doublereal *alpha, doublereal *alflow, doublereal *
	alfupp, integer *eval, doublereal *xdiff, integer *error, integer *
	ind, doublereal *whnrm2, doublereal *fnew, doublereal *v2, doublereal 
	*hnew, doublereal *g)
{
    /* Initialized data */

    static doublereal c1 = .001;

    /* System generated locals */
    integer a_dim1, a_offset, i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal atwa;
    static integer ctrl, exit;
    static doublereal dpsi0;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, k;
    extern /* Subroutine */ int linec_(doublereal *, doublereal *, doublereal 
	    *, doublereal *, integer *, integer *, doublereal *, doublereal *,
	     doublereal *, doublereal *, S_fp, S_fp, doublereal *, doublereal 
	    *, integer *, integer *, integer *, integer *, integer *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *);
    static doublereal magfy;
    extern /* Subroutine */ int upbnd_(doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, doublereal *, integer *, integer 
	    *, doublereal *, integer *, doublereal *, doublereal *, integer *)
	    ;
    static doublereal dummy, whsum, uppbnd, dxtctf;
    extern /* Subroutine */ int weight_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, doublereal *, integer *, integer *, integer *, logical 
	    *, doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *);
    static logical old;
    static doublereal cdx, xel, sum, psi0;


/*     FIND THE STEPLENGTH ALPHA IN    XNEW@D=XOLD+ALPHA*DX   BY */
/*     APPROXIMATELY MINIMIZING THE MERIT FUNCTION PSI(XOLD+ALPHA*DX,W) */
/*     WITH RESPECT TO ALPHA IF THE SEARCH DIRECTION (DX) IS COMPUTED */
/*     USING SUBSPACE MINIMIZATION */

/*     PSI(X,W)=0.5*(II F(X) II**2+SIGMA(W(I)*H (X)**2)+ */
/*                                   I         I */
/*       SIGMA(W(J))*H (X)**2))  WHERE H (X)=MIN(0,H (X)) */
/*         J          +                 +           J */
/*     WHERE I BELONGS TO THE CURRENT WORKING SET AND J TO THE INACTIVE */
/*     SET. HOWEVER, IF DX IS COMPUTED USING THE METHOD */
/*     OF NEWTON AN UNDAMPED STEP IS TAKEN (I.E. ALPHA@D=1 ) */

/*     ON ENTRY@D */

/*     VARIABLES FLAGGED WITH * ARE NOT DEFINED IF CODE=2 */

/*     RESTAR  LOGICAL SCALAR = TRUE IF CURRENT STEP IS A RESTART STEP */
/*             = FALSE IF NOT SO */
/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE POINT XOLD */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*N */
/*             CONTAINING THE GRADIENTS OF THE INACTIVE CONSTRAINTS AS */
/*             ROWS T+1,T+2,......,L */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING THE SEARCH DIRECTION */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+T */
/*             CONTAINING THE VALUE OF THE RESIDUALS IN THE FIRST M */
/*             ELEMENTS */
/*  *  V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+L */
/*             CONTAINING THE COMPOUND VECTOR  (C*DX   ) */
/*                                             (AHAT*DX) */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     FSUM    REAL SCALAR CONTAINING SUM OF SQUARED RESIDUALS */
/*     FFUNC   SUBROUTINE NAME-USED TO EVALUATE THE RESIDUALS */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*             IN  (C1@DC2)=C*Q1*FMAT*P2 */
/*     CODE     INTEGER SCALAR CONTAINING */
/*             = 1 IF GAUSS-NEWTON SEARCH DIRECTION */
/*             =-1 IF SUBSPACE MINIMIZATION SEARCH DIRECTION */
/*             = 2 IF NEWTON SEARCH DIRECTION */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT THE POINT XOLD */
/*     HFUNC   SUBROUTINE NAME-USED TO EVALUATE THE CONSTRAINTS */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR THE CONSTRAINTS IN CURRENT WORKING */
/*             SET */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR INACTIVE CONSTRAINTS */
/*     P4()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             REPRESENTING THE REORDERING OF CONSTRAINT GRADIENTS */
/*             DUE TO THE WORKING SET SUCH THAT P4(I) POINTS TO ROW */
/*             IN ARRAY A WERE GRADIENT NUMBER I IS MOVED */
/*     Q       INTEGER SCALAR CONTAINING NUMBER OF INACTIVE CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     W()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING LATEST VECTOR OF PENALTY CONSTANTS */
/*     WOLD()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE SAME AS W ABOVE UNLESS A RESTART IS DONE. */
/*     DIMA    INTEGER SCALAR CONTAINING THE DIMENSION OF MATRIX A USED */
/*             IN SUBSPACE MINIMAZATION */
/*     NORM    INTEGER SCALAR CONTAINING A CODE THAT CONTROLS WHICH */
/*             NORM THAT SHOULD BE USED WHEN PENALTY WEIGHTS ARE UPDATED */
/*             = 0 MAXIMUM NORM */
/*             = 1 ABSOLUTE NORM */
/*             = 2 EUCLIDEAN NORM */
/*     NOHOUS  INTEGER SCALAR CONTAINING NUMBER OF HOUSEHOLDER */
/*             TRANSFORMATIONS DONE TO FORM            T */
/*                                            (L@D0) =P1 *A*Q1 */

/*     ON RETURN@D */

/*     X()     CONTAINS THE NEW POINT XNEW */
/*     F()     CONTAINS THE VALUE OF THE RESIDUALS AT XNEW */
/*     H()     CONTAINS THE VALUE OF THE CONSTRAINTS AT XNEW */
/* *   W()     CONTAINS THE VECTOR OF PENALTY CONSTANTS THAT HAS BEEN */
/*             USED IN THIS LINESEARCH */
/*     WOLD()  CONTAINS THE VALUE OF W() ON ENTRY UNLESS A RESTART */
/*             IF RESTART THEN WOLD() IS UNCHANGED */
/*     PSI     REAL SCALAR-CONTAINS THE VALUE PSI(XNEW,W) */
/*     ALPHA   REAL SCALAR-CONTAINS THE STEPLENGTH */
/*     ALFLOW  REAL SCALAR-CONTAINS THE LOWER BOUND OF THE STEPLENGTH */
/*     ALFUPP  REAL SCALAR-CONTAINS THE UPPER BOUND OF THE STEPLENGTH */
/*     EVAL    INTEGER SCALAR-CONTAINS NUMBER OF FUNCTION EVALUATIONS */
/*             DONE INSIDE THIS ROUTINE */
/*     XDIFF   REAL SCALAR-CONTAINS  II XNEW-XOLD II */
/*     ERROR   INTEGER SCALAR-CONTAINS */
/*             = 0 IF EVERY THING IS OK */
/*             =-1 IF DX IS NOT A DESCENT DIRECTION TO PSI(XOLD,W) */
/*             =-2 IF ALPHA<=ALFLOW OR ALPHA*DX<DSQRT(DRELPR) */
/*             < -10 AS A USER STOP INDICATOR */
/*     IND     INTEGER SCALAR = 0 IF ALPHA IS LESS THAN ALFUPP */
/*             = INDEX FOR THE CONSTRAINT THAT DEFINES THE UPPER BOUND */
/*               WHICH EQUALS ALPHA */

/*     WORKING AREAS@D */

/*     FNEW(),V2() REAL SINGLY SUBSCRIPTED ARRAYS OF DIMENSION M+L */
/*     HNEW()      REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*      G()        REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */

/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --g;
    --dx;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --x;
    --f;
    --v1;
    --h__;
    --active;
    --inact;
    --p4;
    --w;
    --wold;
    --fnew;
    --v2;
    --hnew;

    /* Function Body */
    *error = 0;
    old = FALSE_;
    *alflow = c1;

/*     IF THE METHOD OF NEWTON HAS BEEN USED TO COMPUTE DX */
/*     AN UNDAMPED STEP IS TAKEN */

    if (abs(*code) != 1) {
	goto L60;
    }

/*     DETERMINE PENALTY CONSTANTS (W) IN THE MERIT FUNCTION PSI(X,W) */

    if (*restar) {
	goto L20;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	wold[i__] = w[i__];
/* L10: */
    }
L20:

/*     IN SOME SITUATIONS THE SEQUENCE OF PENALTY CONSTANTS ARE RESTARTED */

    if (*restar) {
	old = TRUE_;
    }
    weight_(&w[1], &wold[1], &v1[1], &f[1], fsum, m, &active[1], t, &h__[1], 
	    l, norm, dima, &old, whnrm2, &psi0, &dpsi0, &atwa, &cdx, &dxtctf, 
	    &fnew[1], &hnew[1], &v2[1]);

/*     CHECK IF DX IS A DESCENT DIRECTION */

    if (dpsi0 >= 0.) {
	*error = -1;
    }
    if (*error < 0) {
	return 0;
    }

/*     DETERMINE UPPER BOUND OF THE STEPLENGTH */

    upbnd_(&a[a_offset], mda, q, nohous, t, n, &h__[1], &inact[1], &p4[1], &
	    v1[1], m, &dx[1], alfupp, ind);
    *alflow = *alfupp / 3e3;

/*     DETERMINE A FIRST GUESS OF STEPLENGTH */

    magfy = 3.;
    if (*rankc2 < prec_1.rkckm1) {
	magfy *= 2.;
    }
/* Computing MIN */
    d__1 = 1., d__2 = magfy * prec_1.alfkm1, d__1 = min(d__1,d__2);
    *alpha = min(d__1,*alfupp) * 2.;
L40:
    *alpha *= .5;

/*     COMPUTE A STEPLENGTH BY APPROXIMATELY */
/*     MINIMIZING  PSI(XOLD+ALF*DX)  WITH RESPECT TO ALF */

    linec_(&x[1], &dx[1], &f[1], &v1[1], m, n, alpha, &psi0, &dpsi0, alflow, (
	    S_fp)ffunc, (S_fp)hfunc, &h__[1], &hnew[1], &active[1], t, &inact[
	    1], q, l, &w[1], alfupp, &fnew[1], &v2[1], &g[1], psi, xdiff, 
	    eval, &exit);
    if (exit < -10) {
	goto L90;
    }

/*     CHECK IF THE USER HAS SIGNALED UNCOMPUTABILTY FOR ALF=ALPHA */
/*     (THE FIRST GUESSS) */

    if (-3 == exit) {
	goto L40;
    }
    *error = exit;
/* L50: */
    uppbnd = min(1.,*alfupp);

/*     COMPUTE THE PREDICTED LINEAR PROGRESS (PRELIN) AND */
/*     THE ACTUAL PROGRESS (PGRESS) */

/* Computing 2nd power */
    d__1 = uppbnd;
    prec_1.prelin = uppbnd * (dxtctf * -2. - uppbnd * cdx + (2. - d__1 * d__1)
	     * atwa);
/* Computing 2nd power */
    d__1 = enlsip_dnrm2_(m, &f[1], &c__1);
    prec_1.pgress = psi0 * 2. - d__1 * d__1;
    if (*t <= 0) {
	goto L80;
    }
    whsum = 0.;
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
/* Computing 2nd power */
	d__1 = h__[k];
	whsum += w[k] * (d__1 * d__1);
/* L55: */
    }
    prec_1.pgress -= whsum;
    goto L80;
L60:

/*     TAKE AN UNDAMPED STEP */

    *alfupp = 3.;
    sum = 0.;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	xel = x[i__];
	x[i__] = xel + dx[i__];
/* Computing 2nd power */
	d__1 = xel - x[i__];
	sum += d__1 * d__1;
/* L70: */
    }
    *xdiff = sqrt(sum);

/*     EVALUATE RESIDUALS AND CONSTRAINTS AT THE NEW POINT */

    ctrl = -1;
    (*ffunc)(&x[1], n, &f[1], m, &ctrl, &dummy, &c__1);
    exit = ctrl;
    ctrl = -1;
    (*hfunc)(&x[1], n, &h__[1], l, &ctrl, &dummy, &c__1);
    exit = ctrl;
    if (exit < -10) {
	goto L90;
    }
    *alpha = 1.;
    *eval = 1;
L80:
    if (*ind == 0) {
	return 0;
    }
    if ((d__1 = *alpha - *alfupp, abs(d__1)) > .1) {
	*ind = 0;
    }
    return 0;

/*     A USER STOP INDICATOR IS DETECTED */

L90:
    *error = exit;
    return 0;
} /* stplng_ */

/*     DOUBLE PRECISION VERSION 911001 */

/* EUCNRM */
/* Subroutine */ int eucnrm_(doublereal *va, doublereal *h__, integer *active,
	 integer *t, doublereal *mu, doublereal *anorm, doublereal *bnorm, 
	integer *dima, doublereal *w, integer *l, integer *pset, doublereal *
	y, doublereal *wold)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    static integer ctrl, i__, j;
    static doublereal gamma;
    extern /* Subroutine */ int eucmod_(integer *, doublereal *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *), 
	    assort_(doublereal *, integer *, integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *);
    static doublereal yel, tau;
    static integer nrp;
    static doublereal ztw;


/*     UPDATE THE PENALTY CONSTANTS USING THE EUCLIDEAN NORM */

/*     ON ENTRY@D */

/*     VA()         REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*                  CONTAINING THE VECTOR AHAT*DX NORMALIZED TO HAVE */
/*                  LENGTH 1 */
/*     H()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*                  CONTAINING THE VALUE OF THE CONSTRAINTS DIVIDED */
/*                  BY BNORM */
/*     ACTIVE()     INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*                  CONTAINING INDECES FOR THE CONSTRAINTS IN THE */
/*                  CURRENT WORKING SET */
/*     T            INTEGER SCALAR CONTAINING THE NUMBER OF CONSTRAINTS */
/*                  IN THE CURRENT WORKING SET         T */
/*     MU           REAL SCALAR CONTAINING      ABS(CTD-CTC)/DELTA-CTC */
/*     ANORM        REAL SCALAR CONTAINING   II VA II */
/*     BNORM        REAL SCALAR CONTAINING   II H II */
/*     DIMA         INTEGER SCALAR CONTAINING DIMENSION <=T USED */
/*                  TO COMPUTE THE SEARCH DIRECTION DX */
/*     W()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*                  CONTAINING THE LATEST PENALTY CONSTANTS (WOLD) */
/*     L            INTEGER SCALAR CONTAINING THE TOTAL NUMBER OF */
/*                  CONSTRAINTS */

/*     ON RETURN@D */

/*     W()          CONTAINS THE NEW PENALTY CONSTANTS */

/*     WORKING AREAS@D */

/*     PSET()       INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*     Y()          REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*     WOLD()       REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --wold;
    --y;
    --pset;
    --w;
    --active;
    --h__;
    --va;

    /* Function Body */
    if (*t <= 0) {
	return 0;
    }
    ztw = 0.;

/*     COMPUTE ZTW= Z(TR)*WOLD where WOLD holds the 4'th lowest weights used */
/*     so far */

    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	w[i__] = wsave_1.u[i__ + 299];
/* L5: */
    }
/*      ynorm=0.0d0 */
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = active[i__];
/*           ynorm=ynorm+va(i)**4 */
/*            w(j)=u(j,4) */
/* Computing 2nd power */
	d__1 = va[i__];
	ztw += d__1 * d__1 * w[j];
/* L10: */
    }
/*      ynorm=ynorm*anorm**2 */
    ztw = *anorm * *anorm * ztw;
/*      goto 19 */
/*      do 15 i=1,l */
/*       wold(i)=w(i) */
/*   15 continue */
/*      do 18 i=1,t */
/*       j=active(i) */
/*       if(mu .le. 0.0d0)then */
/*         w(j)=0.0d0 */
/*       else */
/*         w(j)=mu*va(i)**2/ynorm */
/*       endif */
/*   18 continue */
/*      call assort(u,100,l,4,t,active,w,wold) */
/*      return */
/*   19 continue */
    if (ztw < *mu) {
	goto L40;
    }

/*     IF ZTW>=MU WE DON'T NEED TO CHANGE WOLD UNLESS T@DIMA */

    if (*t == *dima) {
	goto L106;
    }

/*     FORM THE VECTOR Y AND THE SCALAR GAMMA */
/*     PSET HOLDS INDECES FOR THOSE Y(I) BEING >0 */

    ctrl = 2;
    nrp = 0;
    gamma = 0.;
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = active[i__];
	yel = va[i__] * *anorm * (va[i__] * *anorm + *bnorm * h__[j]);
	if (yel <= 0.) {
	    goto L20;
	}
	++nrp;
	pset[nrp] = j;
	y[nrp] = yel;
	goto L30;
L20:
	gamma -= yel * w[j];
L30:
	;
    }
    eucmod_(&ctrl, &w[1], l, &pset[1], &nrp, &y[1], &gamma, &wold[1]);
    goto L106;
/*      RETURN */
L40:

/*     ZTW<MU WHEN EXECUTION CONTINUES FROM HERE */

    if (*t == *dima) {
	goto L70;
    }

/*     FORM THE VECTOR E AND THE SCALAR TAU */
/*     PSET HOLDS INDECES FOR THOSE E(I) BEING >0 */

    ctrl = 2;
    nrp = 0;
    tau = *mu;
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = active[i__];
	yel = -va[i__] * h__[j] * *anorm * *bnorm;
	if (yel <= 0.) {
	    goto L50;
	}
	++nrp;
	pset[nrp] = j;
	y[nrp] = yel;
	goto L60;
L50:
	tau -= yel * w[j];
L60:
	;
    }
    eucmod_(&ctrl, &w[1], l, &pset[1], &nrp, &y[1], &tau, &wold[1]);
    goto L106;
/*      RETURN */
L70:
    ctrl = 1;

/*     FORM THE VECTOR Z */
/*     PSET HOLDS THE INDECES ACTIVE(I) , I=1,2,....,T SINCE ALL */
/*     ELEMENTS IN Z IS >= 0 */

    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	pset[i__] = active[i__];
/* Computing 2nd power */
	d__1 = va[i__];
	y[i__] = d__1 * d__1 * *anorm * *anorm;
/* L80: */
    }
    eucmod_(&ctrl, &w[1], l, &pset[1], t, &y[1], mu, &wold[1]);
/*      RETURN */
L106:
    assort_(wsave_1.u, &c__100, l, &c__4, t, &active[1], &w[1], &wold[1]);
    return 0;
} /* eucnrm_ */


/*     double precision version 880329 */
/* MAXNRM */
/* Subroutine */ int maxnrm_(doublereal *ata, doublereal *rmy, doublereal *
	alfa, doublereal *delta, doublereal *w, integer *active, integer *t)
{
    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    static integer i__, j, k, l;
    static doublereal mu, nu, wkm1;


/*     UPDATE THE PENALTY WEIGHTS CORRESPONDING TO THE */
/*     CONSTRAINTS IN THE CURRENT WORKING SET */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --w;
    --active;

    /* Function Body */
    if ((d__1 = *alfa - 1., abs(d__1)) <= *delta) {
	mu = 0.;
    } else {
	mu = *rmy / *ata;
    }
    l = active[1];
    wkm1 = w[l];
    nu = max(mu,wsave_1.u[300]);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	w[k] = nu;
/* L10: */
    }
    if (mu <= wkm1) {
	goto L40;
    }
    for (i__ = 1; i__ <= 4; ++i__) {
	if (mu <= wsave_1.u[i__ * 100 - 100]) {
	    goto L30;
	}
	i__1 = i__ + 1;
	for (j = 4; j >= i__1; --j) {
	    wsave_1.u[j * 100 - 100] = wsave_1.u[(j - 1) * 100 - 100];
/* L20: */
	}
	wsave_1.u[i__ * 100 - 100] = mu;
	goto L40;
L30:
	;
    }
L40:
    return 0;
} /* maxnrm_ */


/*       double precision version 911001 */
/* assort */
/* Subroutine */ int assort_(doublereal *u, integer *mdu, integer *l, integer 
	*s, integer *t, integer *active, doublereal *w, doublereal *wold)
{
    /* System generated locals */
    integer u_dim1, u_offset, i__1, i__2, i__3;

    /* Local variables */
    static integer i__, j, k, ii;
    static doublereal wxk;


/* 	update row no. i in u when w(i)>max(u(i,s), wold(i)) */

    /* Parameter adjustments */
    u_dim1 = *mdu;
    u_offset = 1 + u_dim1 * 1;
    u -= u_offset;
    --active;
    --w;
    --wold;

    /* Function Body */
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	wxk = w[k];
/* 	 w(k)=max(u(k,s), wxk) */
/* 	 WXK=W(K) */
/* wmax=max(u(k,s), wold(k)) */
/*         WMAX=WOLD(K) */
/* 	 if(wxk.le.wmax)goto 30 */
	i__2 = *s;
	for (ii = 1; ii <= i__2; ++ii) {
	    if (wxk <= u[k + ii * u_dim1]) {
		goto L10;
	    }
	    i__3 = ii + 1;
	    for (j = *s; j >= i__3; --j) {
		u[k + j * u_dim1] = u[k + (j - 1) * u_dim1];
/* L20: */
	    }
	    u[k + ii * u_dim1] = wxk;
	    goto L30;
L10:
	    ;
	}
L30:
	;
    }
    return 0;
} /* assort_ */


/*     DOUBLE PRECISION VERSION 880406 */
/* INIALC */
/* Subroutine */ int inialc_(integer *p, integer *l, doublereal *h__, integer 
	*active, integer *t, integer *bnd, integer *norm, integer *inact, 
	integer *lmt, doublereal *w, doublereal *wold, integer *exit)
{
    /* Initialized data */

    static doublereal delta = .1;
    static doublereal eps = .01;

    /* System generated locals */
    integer i__1;
    doublereal d__1;

    /* Local variables */
    static doublereal absh;
    static integer lmin, i__, j;
    static doublereal wi;
    static integer pp1;
    static doublereal pos, sum;


/*     DETERMINE THE FIRST WORKING SET BY STORING THE INDECES */
/*     FOR CONSTRAINTS BEING REGARDED AS EQUALITIES AT THE */
/*     STARTING POINT IN THE ARRAY ACTIVE AND THE OTHER */
/*     INDECES IN THE ARRAY INACT */

/*     ON ENTRY@D */

/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINING TOTAL NUMBER OF CONSTRAINTS */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS AT X */
/*     W()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING VALUES >=0 USED AS FIRST PENALTY CONSTANTS */
/*             IF AN ELEMENT IS <=0.0 THE FIRST PENALTY CONSTANT IS */
/*             CHOOSEN AUTOMATICALLY */
/*     BND     INTEGER SCALAR CONTAINING MIN(L,N) */
/*     NORM    INTEGER SCALAR CONTAINING A CODE THAT CONTROLS WHICH */
/*             NORM THAT SHOULD BE USED WHEN PENALTY WEIGHTS ARE UPDATED */
/*             = 0 MAXIMUM NORM */
/*             = 1 ABSOLUTE NORM */
/*             = 2 EUCLIDEAN NORM */

/*     ON RETURN@D */

/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             ACTIVE(I)= INDEX FOR A CONSTRAINT IN CURRENT WORKING SET */
/*             I=1,2,......,T . ACTIVE(BND+I) =1 OR 0  I=1,2,...,(L-P) */
/*             1 IF CORRESPONDING CONSTRAINT IN WORKING SET.OTHERWISE 0 */
/*     T       INTEGER SCALAR-CONTAINS NUMBER OF CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             INACT(I)= INDEX FOR A CONSTRAINT NOT IN CURRENT WORKING */
/*             SET   I=1,2,.....,LMT */
/*     LMT     INTEGER SCALAR-CONTAINS NUMBER OF CONSTRAINTS NOT IN */
/*             CURRENT WORKING SET */
/*     WOLD()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINS PENALTY CONSTANTS CORRESPONDING TO EACH */
/*             CONSTRAINT */
/*     W()     W(I)=WOLD(I) I=1,2,.......,L */
/*     EXIT    INTEGER SCALAR SET TO -8 IF T>N */
/*             I.E. IF MORE THAN N CONSTRAINTS IN FIRST WORKING SET */

/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --wold;
    --w;
    --inact;
    --active;
    --h__;

    /* Function Body */
    *t = *p;
    *lmt = 0;
    if (*l == 0) {
	return 0;
    }
    lmin = min(*l,100);
    i__1 = lmin;
    for (i__ = 1; i__ <= i__1; ++i__) {
	for (j = 1; j <= 4; ++j) {
	    wsave_1.u[i__ + j * 100 - 101] = delta;
/* L5: */
	}
    }

/*     COMPUTE PENALTY CONSTANTS */

    sum = 0.;
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	absh = (d__1 = h__[i__], abs(d__1));
	wi = w[i__];
	if (wi > 0.) {
	    pos = wi;
	}
	if (wi <= 0.) {
/* Computing MIN */
	    d__1 = absh + eps;
	    pos = min(d__1,delta);
	}
	wold[i__] = pos;
	w[i__] = pos;
	sum += pos;
/* L10: */
    }
    if (*norm > 0) {
	goto L25;
    }
    sum /= (doublereal) (*l);
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	wold[i__] = sum;
	w[i__] = sum;
/* L20: */
    }
L25:
    if (*p == 0) {
	goto L40;
    }

/*     BUILD WORKING AND INACTIVE SET */

    i__1 = *p;
    for (i__ = 1; i__ <= i__1; ++i__) {
	active[i__] = i__;
/* L30: */
    }
L40:
    pp1 = *p + 1;
    if (pp1 > *l) {
	return 0;
    }
    i__1 = *l;
    for (i__ = pp1; i__ <= i__1; ++i__) {
	j = *bnd + i__ - *p;
	if (h__[i__] <= 0.) {
	    goto L50;
	}
	++(*lmt);
	inact[*lmt] = i__;
	active[j] = 0;
	goto L60;
L50:
	++(*t);
	active[*t] = i__;
	active[j] = 1;
L60:
	;
    }
    if (*t > *bnd) {
	*exit = -8;
    }
    return 0;
} /* inialc_ */


/*     double precision version 880329 */
/* RELEPS */
/* Subroutine */ int releps_(doublereal *drelpr)
{
    static doublereal frac, temp;


/*     COMPUTE DRELPR = DOUBLE RELATIVE PRECISION FOR A BINARY */
/*     MACHINE   I.E. */
/*     DETERMINE THE SMALLEST POSITIVE NUMBER 0.5**K FOR WHICH */
/*        (1.0+0.5**K) > 1.0  AND  (1.0+0.5**(K+1)) = 1.0 */
/*     WHERE K IS A POSITIVE INTEGER */

/*     INTERNAL VARIABLES */

    frac = 1.;
L10:
    frac *= .5;
    temp = frac + 1.;
    if (temp == 1.) {
	goto L20;
    }
    goto L10;
L20:
    *drelpr = frac * 2.;
    return 0;
} /* releps_ */

/* GNDCHK */
/* Subroutine */ int gndchk_(doublereal *b1nrm, doublereal *d1nrm, doublereal 
	*dnrm, doublereal *hsum, integer *k, logical *restar, doublereal *
	d1apm1, logical *add, integer *m, integer *n, logical *del, integer *
	active, integer *p, integer *t, doublereal *v, integer *inact, 
	integer *q, doublereal *h__, doublereal *epsrel, integer *ranka, 
	integer *scale, doublereal *diag, doublereal *betak, integer *ind)
{
    /* Initialized data */

    static doublereal delta = .1;
    static doublereal c1 = .5;
    static doublereal c2 = .1;
    static doublereal c3 = 4.;
    static doublereal c4 = 10.;
    static doublereal c5 = .05;

    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal rowi, nlkm1;
    static integer i__, j;
    static doublereal sqrel;
    static integer pp1;
    static logical neg;
    static doublereal nlk, eps;
    static integer nmt, qpt;


/*     DECIDE WHAT METHOD SHOULD BE USED TO COMPUTE THE SEARCH DIRECTION */

/*     IND IS SET = 1 IF GAUSS-NEWTON */
/*                =-1 IF SUBSPACE MINIMIZATION */
/*                = 2 IF NEWTON */

/*     ON ENTRY@D */

/*     B1NRM   REAL SCALAR CONTAINING  II B1 II */
/*     D1NRM   REAL SCALAR CONTAINING  II D1 II */
/*     DNRM    REAL SCALAR CONTAINING  II D II */
/*     HSUM    REAL SCALAR CONTAINING SUM OF SQUARED CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     K       INTEGER SCALAR CONTAINING ITERATION NUMBER */
/*     RESTAR  LOGICAL SCALAR = TRUE IF CURRENT STEP IS A RESTART STEP */
/*             = FALSE IF IT IS NOT */
/*     D1APM1  REAL SCALAR CONTAINING  II D1 II  WHERE D1 HAS AS MANY */
/*             ELEMENTS AS IN THE LATEST STEP  -1 */
/*     ADD     LOGICAL SCALAR = TRUE IF AT LEAST ONE CONSTRAINT WAS */
/*             ADDED TO THE WORKING SET IN THE LATEST STEP */
/*             =FALSE IF NO CONSTRAINT WAS ADDED */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     DEL     LOGICAL SCALAR = TRUE IF SOME CONSTRAINT WAS DELETED */
/*             FROM WORKING SET IN CURRENT STEP */
/*             = FALSE IF NO CONSTRAINT WAS DELETED */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES OF THE CONSTRAINTS IN CURRENT */
/*             WORKING SET */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     V()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING ESTIMATES OF LAGRANGE MULTIPLIERS */
/*     INACT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION Q */
/*             CONTAINING INDECES FOR INACTIVE CONSTRAINTS */
/*     Q       INTEGER SCALAR CONTAINING NUMBER OF INACTIVE CONSTRAINTS */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS */
/*     EPSREL  REAL SCALAR CONTAINING RELATIVE CONVERGENCE TOLERANCE */
/*     RANKA   INTEGER SCALAR CONTAINING THE PSEUDO-RANK OF */
/*             THE MATRIX AHAT */
/*     SCALE   INTEGER SCALAR CONTAINING ZERO IF NO INTERNAL ROW */
/*             SCALING OF THE MATRIX A HAS BEEN DONE. OTHERWISE */
/*             SCALING HAS BEEN DONE */
/*     DIAG()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING THE DIAGONAL ELEMENTS OF THE SCALING MATRIX */
/*             INTERNAL SCALING HAS BEEN DONE. OTHERWISE IT CONTAINS */
/*             THE LENGHT OF EACH ROW IN THE MATRIX A */

/*     ON RETURN@D */

/*     BETAK   REAL SCALAR-CONTAINS  DSQRT(D1NRM**2+B1NRM**2) */
/*     IND     INTEGER SCALAR-CONTAINS */
/*             = 1 IF GAUSS-NEWTON SEARCH DIRECTION IS ACCEPTED */
/*             =-1 IF SUBSPACE MINIMIZATION IS SUGGESTED */
/*             = 2 IF THE METHOD OF NEWTON IS SUGGESTED */


/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --diag;
    --h__;
    --inact;
    --v;
    --active;

    /* Function Body */
/* Computing 2nd power */
    d__1 = *d1nrm;
/* Computing 2nd power */
    d__2 = *b1nrm;
    *betak = sqrt(d__1 * d__1 + d__2 * d__2);
    pp1 = *p + 1;
    nmt = *n - *t;
    qpt = *q + *t;

/*     TO ACCEPT THE GAUSS-NEWTON SEARCH DIRECTION WE MUST NOT HAVE */
/*     USED THE METHOD OF NEWTON BEFORE AND CURRENT STEP MUST NOT */
/*     BE A RESTART STEP */

    if (*restar || prec_1.kodkm1 == 2) {
	goto L10;
    }

/*     IF ANY OF THE FOLLOWING CONDITIONS IS SATISFIED THE GAUSS- */
/*     NEWTON DIRECTION IS ACCEPTED */

/*     1) THE FIRST ITERATION STEP */
/*     2) ESTIMATED CONVERGENCE FACTOR < C1 */
/*     3) THE REAL PROGRESS > C2*PREDICTED LINEAR PROGRESS */
/*        (PROVIDED WE ARE NOT CLOSE TO THE SOLUTION) */

    *ind = 1;
    if (negdir_1.ifree > 0) {
	--negdir_1.ifree;
    }
    if (negdir_1.ifree == 4) {
	return 0;
    }
    if (*k == 0 && ! (*restar)) {
	return 0;
    }
    if (-1 == prec_1.kodkm1) {
	goto L10;
    }
    if (*add || *del) {
	return 0;
    }
    if (*betak < c1 * prec_1.betkm1) {
	return 0;
    }
    if (prec_1.pgress > c2 * prec_1.prelin && *dnrm <= c3 * *betak) {
	return 0;
    }
L10:

/*     SUBSPACE MINIMIZATION IS SUGGESTED IF ONE OF THE FOLLOWING */
/*     HOLDS TRUE */

/*     4) THERE IS SOMETHING LEFT TO REDUCE IN SUBSPACES */
/*     5) ADDITION AND/OR DELETION TO/FROM CURRENT WORKING SET */
/*        HAS BEEN DONE IN THE LATEST STEP */
/*     6) THE NONLINEARITY IS NOT TOO SEVER */

    *ind = -1;
    if (negdir_1.ifree > 0) {
	return 0;
    }
    if (prec_1.kodkm1 == 2 && ! (*del)) {
	goto L60;
    }
/* Computing 2nd power */
    d__1 = *d1nrm;
    nlk = sqrt(d__1 * d__1 + *hsum);
/* Computing 2nd power */
    d__1 = *d1apm1;
    nlkm1 = sqrt(d__1 * d__1 + *hsum);
    neg = FALSE_;
    pp1 = *p + 1;
    if (pp1 > *t) {
	goto L30;
    }
    sqrel = sqrt(machin_1.drelpr);
    i__1 = *t;
    for (i__ = pp1; i__ <= i__1; ++i__) {
	rowi = diag[i__];
	if (*scale != 0) {
	    rowi = 1. / diag[i__];
	}
	if (-sqrel < v[i__] * rowi) {
	    goto L20;
	}
	if (v[i__] < 0.) {
	    neg = TRUE_;
	}
L20:
	;
    }
L30:
    if (*q <= 0) {
	goto L50;
    }
    i__1 = *q;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = inact[i__];
	if (h__[j] < delta) {
	    neg = TRUE_;
	}
/* L40: */
    }
L50:

/*     IN SOME SITUATIONS WE CAN'T TAKE THE RISK OF SUGGESTING */
/*     THE METHOD OF NEWTON */

    if (*hsum > c2) {
	return 0;
    }
    if (*add || *del || neg || *t == *n && *t == *ranka) {
	return 0;
    }
    if (qpt == *p || *ranka < *t) {
	goto L55;
    }
/* Computing MAX */
    d__1 = .01, d__2 = *epsrel * 10.;
    eps = max(d__1,d__2);
    if (! (*betak < eps * *dnrm || *b1nrm < eps && *m == nmt)) {
	return 0;
    }
L55:
    if (*restar) {
	goto L70;
    }
    if (prec_1.alfkm1 < c5 && nlkm1 < c2 * nlk || *m == nmt) {
	goto L60;
    }
    if (*dnrm <= c4 * *betak) {
	return 0;
    }

/*     THE METHOD OF NEWTON IS THE ONLY ALTERNATIVE */

L60:
    *ind = 2;
    return 0;
L70:
    if (nlkm1 > c2 * nlk) {
	return 0;
    }
    *ind = 2;
    return 0;
} /* gndchk_ */

/* SUBSPC */
/* Subroutine */ int subspc_(logical *restar, doublereal *fsum, doublereal *
	c__, integer *mdc, integer *m, integer *n, integer *rankc2, 
	doublereal *f, integer *p3, doublereal *d3, doublereal *a, integer *
	mda, integer *t, integer *ranka, doublereal *hsum, integer *p1, 
	doublereal *d2, integer *p2, doublereal *b, doublereal *fmat, integer 
	*mdf, doublereal *pivot, doublereal *gmat, integer *mdg, doublereal *
	d__, doublereal *dx, doublereal *work1, integer *dima, integer *dimc2)
{
    /* Initialized data */

    static doublereal beta1 = .1;
    static doublereal beta2 = .1;
    static doublereal lowalf = .2;

    /* System generated locals */
    integer c_dim1, c_offset, a_dim1, a_offset, fmat_dim1, fmat_offset, 
	    gmat_dim1, gmat_offset, i__1, i__2;

    /* Local variables */
    static doublereal etaa, etac, r11td1;
    static integer drkm1;
    static doublereal r22td2;
    static integer dukm1;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, k;
    extern /* Subroutine */ int h12per_(integer *, integer *, integer *, 
	    integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    integer *, integer *, integer *, doublereal *);
    static doublereal fpgrs, hpgrs;
    extern /* Subroutine */ int ycomp_(integer *, integer *, doublereal *, 
	    doublereal *, integer *, doublereal *), ltoup_(doublereal *, 
	    integer *, integer *, integer *, doublereal *, integer *, integer 
	    *, doublereal *, doublereal *);
    static doublereal b1aspr, d1aspr;
    extern /* Subroutine */ int vp_(doublereal *, integer *, integer *, 
	    integer *, integer *), dimupp_(logical *, integer *, doublereal *,
	     integer *, doublereal *, integer *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, doublereal *)
	    , usolve_(integer *, integer *, doublereal *, doublereal *), rtd_(
	    doublereal *, integer *, integer *, doublereal *, doublereal *);


/*     THE ORIGINAL SYSTEM  A*DX = -H */
/*                          C*DX APPR.= -F */
/*     IS TRANSFORMED TO    R*DY1 = B1 */
/*                          U*DY2 = D1 */
/*     WHERE   R IS RANKA*RANKA UPPER TRIANGULAR */
/*             U IS RANKC2*RANKC2 UPPER TRIANGULAR */
/*     AND             T   T */
/*            (R@D0)= Q2 *P1 *A*Q1*FMAT*P2 */
/*          B=(B1)   T   T       (C1@DC2)=C*Q1*FMAT*P2 */
/*            (B2)=Q2 *P1 *(-H) */
/*     (U))    T             D=(D1)      T */
/*     (0) = Q3 *C2*P3         (D2) = -Q3 *(F+C1*DY1) */
/*         DX=Q1*FMAT*P2*(DY1) */
/*                       (P3*DY2) */

/*     THIS ROUTINE CHOOSES THE DIMENSIONS OF THE SUBSPACES */
/*     WHERE MINIMIZATION SHOULD BE DONE */
/*     I.E. HOW MANY COLUMNS OF R AND U SHOULD BE USED */

/*     ON ENTRY@D */

/*     RESTAR  LOGICAL SCALAR = TRUE IF CURRENT STEP IS A RESTART STEP */
/*             = FALSE IF NO RESTART WAS DONE */
/*     FSUM    REAL SCALAR CONTAINIG SUM OF SQUARED RESIDUALS */
/*     C(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDC*N */
/*             CONTAINING (C1@DU) AND INFO. TO FORM Q3 */
/*     MDC     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY C */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     N       INTEGER SCALAR CONTAINING NUMBER OF PARAMETERS */
/*     RANKC2  INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX C2 */
/*     D()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING    T */
/*                        -Q3 *(F+C1*DY1) */
/*     P3()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-T */
/*             REPRESENTING PERMUTATION MATRIX P3 */
/*     D3()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N-T */
/*             CONTAINING INFO. TO FORM Q3 */
/*     A(,)    REAL DOUBLY SUBSCRIPTED ARRAY OF DIMESION MDA*N */
/*             CONTAINING MATRIX L FROM (L@D0) AND INFO. TO FORM Q1 */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     RANKA   INTEGER SCALAR CONTAINING PSEUDO RANK OF MATRIX A */
/*     HSUM    REAL SCALAR CONTAINING SUM OF SQUARED CONSTRAINTS */
/*             IN CURRENT WORKING SET */
/*     P1()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P1 */
/*     D2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q2 */
/*     P2()    INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             REPRESENTING PERMUTATION MATRIX P2 */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING  (B1) */
/*                         (B2) */
/*     FMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDF*N */
/*             CONTAINING A PRODUCT OF GIVENS ROTATION MATRICES */
/*     MDF     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY FMAT */
/*     PIVOT() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION RANKA */
/*             CONTAINING INFO. TO FORM Q1 */
/*     GMAT(,) REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDG*N */
/*             CONTAINING MATRIX R AND INFO. TO FORM Q2 */
/*     MDG     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY GMAT */

/*     ON RETURN@D */

/*     DIMA    INTEGER SCALAR-CONTAINS DIMENSION SUGGESTED FOR R */
/*     DIMC2   INTEGER SCALAR-CONTAINS DIMENSION SUGGESTED FOR C2 */

/*     WORKING AREAS@D */

/*     DX()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*     WORK1() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */

/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --d__;
    --f;
    --dx;
    --d3;
    --p3;
    c_dim1 = *mdc;
    c_offset = 1 + c_dim1 * 1;
    c__ -= c_offset;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --p1;
    --d2;
    --p2;
    --b;
    fmat_dim1 = *mdf;
    fmat_offset = 1 + fmat_dim1 * 1;
    fmat -= fmat_offset;
    --pivot;
    gmat_dim1 = *mdg;
    gmat_offset = 1 + gmat_dim1 * 1;
    gmat -= gmat_offset;
    --work1;

    /* Function Body */
    *dima = 0;

/*     MOVE -F TO D */

    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	d__[i__] = -f[i__];
/* L10: */
    }

/*     TRANSFORM L IN (L@D0) TO UPPER TRIANGULAR FORM IF IT */
/*     IS NOT DONE BEFORE */

    etaa = 1.;
    if (*ranka <= 0) {
	goto L50;
    }
    if (*t > *ranka) {
	goto L20;
    }
    if (back_1.nrrest > 1) {
	goto L20;
    }
    ltoup_(&a[a_offset], mda, ranka, t, &b[1], mdg, &p2[1], &gmat[gmat_offset]
	    , &d2[1]);

/*     DO COORDINATE CHANGE IN MATRIX C1 BY FORMING */
/*     C1@D=C1*P2 */

    vp_(&c__[c_offset], mdc, m, ranka, &p2[1]);

/*     compute norms of R11(T)*d1 and R22(T)*d2 */

    rtd_(&a[a_offset], mda, ranka, &b[1], &r11td1);
    i__1 = *rankc2;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *ranka + i__;
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, m, &c__[k * c_dim1 + 1], &c__1, &d3[i__], 
		&d__[1], &c__1, m, &c__1, &c__[i__ + k * c_dim1]);
/* L15: */
    }
    rtd_(&c__[(*ranka + 1) * c_dim1 + 1], mdc, rankc2, &d__[1], &r22td2);
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	d__[i__] = -f[i__];
/* L17: */
    }
L20:
    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dx[i__] = b[i__];
/* L30: */
    }

/*     DRKM1 = PSEUDO RANK OF R USED IN THE LATEST STEP */
/*     B1ASPR = II B1 II  WHERE B1 BELONGS TO R(DRKM1) */

    drkm1 = abs(prec_1.rkakm1) + *t - prec_1.tkm1;
    b1aspr = enlsip_dnrm2_(&drkm1, &b[1], &c__1);

/*     DETERMINE DIMENSION OF MATRIX R TO BE USED */

    hpgrs = prec_1.hsqkm1 - *hsum;
    if (! (*restar) && r11td1 < beta1 * r22td2) {
	*dima = 0;
    } else {
	dimupp_(restar, &drkm1, &hpgrs, ranka, &gmat[gmat_offset], mdg, &dx[1]
		, &work1[1], &prec_1.b1km1, &b1aspr, &prec_1.alfkm1, dima, &
		etaa);
    }

/*     SOLVE FOR DY1 THE SYSTEM  R*DY1 = B1 */
/*     BY USING DIMA COLUMNS OF MATRIX R */

    i__1 = *ranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	dx[i__] = b[i__];
/* L40: */
    }
    usolve_(mdg, dima, &gmat[gmat_offset], &dx[1]);

/*     FORM RIGHT HAND SIDE   -F-C1*DY1 */
/*     AND STORE IN ARRAY D */

    ycomp_(dima, m, &d__[1], &c__[c_offset], mdc, &dx[1]);
L50:
    *dimc2 = *rankc2;
    if (*rankc2 <= 0) {
	goto L65;
    }
/*                   T */
/*     COMPUTE D@D= Q3 *(-F-C1*DY1) */

    i__1 = *rankc2;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = *ranka + i__;
	i__2 = i__ + 1;
	h12per_(&c__2, &i__, &i__2, m, &c__[k * c_dim1 + 1], &c__1, &d3[i__], 
		&d__[1], &c__1, m, &c__1, &c__[i__ + k * c_dim1]);
/* L60: */
    }

/*     DUKM1 = PSEUDO RANK OF MATRIX U USED IN THE LATEST STEP */
/*     D1ASPR =  II D II   WHERE D BELONGS TO R(DUKM1) */

L65:
    dukm1 = abs(prec_1.rkckm1) + prec_1.tkm1 - *t;
    d1aspr = enlsip_dnrm2_(&dukm1, &d__[1], &c__1);
    fpgrs = prec_1.fsqkm1 - *fsum;

/*     DETERMINE DIMENSION OF U TO BE USED */

    if (! (*restar) && r22td2 < beta2 * r11td1) {
	*dimc2 = 0;
	etac = 1.;
    } else {
	dimupp_(restar, &dukm1, &fpgrs, rankc2, &c__[(*ranka + 1) * c_dim1 + 
		1], mdc, &d__[1], &work1[1], &prec_1.d1km1, &d1aspr, &
		prec_1.alfkm1, dimc2, &etac);
    }
    if (! (*restar)) {
	goto L120;
    }
/* L70: */

/*     REDUCE THE DIMENSION CORRESPONDING TO EITHER THE CONSTRAINTS */
/*     OR THE OBJECTIVE FUNCTION */

    if (back_1.nrrest > 1) {
	goto L90;
    }
L80:
    back_1.bestpg = prec_1.pgress;
    prec_1.rkckm2 = prec_1.rkckm1;
    prec_1.rkakm2 = prec_1.rkakm1;
L90:
    if (prec_1.pgress > back_1.bestpg) {
	goto L80;
    }
    if (etaa >= etac) {
	goto L100;
    }
    *dimc2 = prec_1.rkckm1;
/* Computing MAX */
    i__1 = prec_1.rkakm1 - 1;
    *dima = max(i__1,0);
    goto L110;
L100:
    if (etaa == etac && *ranka > 1) {
	--prec_1.rkakm1;
    }
/* Computing MAX */
    i__1 = 0, i__2 = prec_1.rkckm1 - 1;
    *dimc2 = max(i__1,i__2);
    *dima = prec_1.rkakm1;
    if (*dimc2 == 0 && *ranka == 1) {
	*dima = 0;
    }
L110:
    if (*dimc2 > 0 || *dima > 0) {
	goto L120;
    }
    *dimc2 = prec_1.rkckm2;
    *dima = prec_1.rkakm2;
    back_1.lattry = 0;
L120:
    if (*restar) {
	return 0;
    }
    if (prec_1.alfkm1 >= lowalf) {
	*dima = max(*dima,drkm1);
	*dimc2 = max(*dimc2,dukm1);
    }
    return 0;
} /* subspc_ */


/*      double precsion 880410 */

/* Subroutine */ int rtd_(doublereal *r__, integer *mdr, integer *n, 
	doublereal *d__, doublereal *rtdnrm)
{
    /* System generated locals */
    integer r_dim1, r_offset, i__1, i__2;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer i__, j;
    static doublereal sum, sum2;


/*     compute rtdnrm = 2-norm of R(T)*d */

    /* Parameter adjustments */
    r_dim1 = *mdr;
    r_offset = 1 + r_dim1 * 1;
    r__ -= r_offset;
    --d__;

    /* Function Body */
    sum = 0.;
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
	sum2 = 0.;
	i__2 = j;
	for (i__ = 1; i__ <= i__2; ++i__) {
	    sum2 += r__[i__ + j * r_dim1] * d__[i__];
/* L10: */
	}
/* Computing 2nd power */
	d__1 = sum2;
	sum += d__1 * d__1;
/* L20: */
    }
    *rtdnrm = sqrt(sum);
    return 0;
} /* rtd_ */


/*     double precision 880410 */
/* DIMUPP */
/* Subroutine */ int dimupp_(logical *restar, integer *km1rnk, doublereal *
	pgress, integer *kp1, doublereal *a, integer *mda, doublereal *b, 
	doublereal *r__, doublereal *prelin, doublereal *asprev, doublereal *
	alfkm1, integer *newdim, doublereal *eta)
{
    /* Initialized data */

    static doublereal c1 = .1;

    /* System generated locals */
    integer a_dim1, a_offset, i__1;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal dsum;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, k;
    extern /* Subroutine */ int pregn_(doublereal *, doublereal *, doublereal 
	    *, doublereal *, integer *, integer *, integer *);
    static integer ik;
    static doublereal rn, sn;
    static integer mindim;
    static doublereal psimax;
    extern /* Subroutine */ int presub_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *);
    static doublereal psi;


/*     DETERMINE SUITABLE DIMENSION FOR SOLVING */
/*                A*X  = B */
/*     WHERE */
/*           A IS UPPER TRIANGULAR (KP1*KP1) */
/*           B IS KP1*1 */

/*         NEWDIM HOLDS THE SUGGESTED DIMENSION ON RETURN */

/*     ON ENTRY@D */

/*     RESTAR  LOGICAL SCALAR = TRUE IF THIS IS A RESTART STEP */
/*             = FALSE OTHERWISE */
/*     KM1RNK  INTEGER SCALAR CONTAINING DIMENSION USED IN THE */
/*             LATEST STEP */
/*     PGRESS  REAL SCALAR CONTAINING THE REDUCTION IN THE OBJECTIVE */
/*             OF CONSERN IN THE LATEST STEP */
/*     KP1     INTEGER SCALAR CONTAINING THE PSEUDO RANK OF MATRIX A */
/*     A()     REAL DOUBLY SUBSCRIPTED ARRAY OF DIMENSION MDA*KP1 */
/*             CONTAINING THE UPPER TRIANGULAR MATRIX A */
/*     MDA     INTEGER SCALAR CONTAINING LEADING DIMENSION OF ARRAY A */
/*     B()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION KP1 */
/*             CONTAINING THE RIGHT HAND SIDE */
/*     R()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION KP1 */
/*             WORKING AREA */
/*     PRELIN  REAL SCALAR CONTAINING THE SQUARE ROOT OF THE PREDICTED */
/*             REDUCTION IN THE OBJECTIVE OF CONSERN IN THE LATEST STEP */
/*     ASPREV  REAL SCALAR CONTAINING THE SQUARE ROOT OF THE PREDICTED */
/*             IN THE OBJECTIVE OF CONSERN IN THE CURRENT STEP IF THE SAME */
/*             DIMENSION AS IN THE PREVIOUS STEP IS USED */
/*     ALFKM1  REAL SCALAR CONTAINING THE STEPLENGTH USED IN THE */
/*             LATEST STEP */

/*     ON RETURN@D */

/*     NEWDIM  INTEGER SCALAR-CONTAINS THE SUGGESTED DIMENSION */
/*     ETA     REAL SCALAR-CONTAINS 1.0 WHEN RESTAR IS FALSE OR KP1<=0 */
/*             IF RESTAR IS TRUE AND KP1>0 ETA=L(KM1RNK-1)/L(KM1RNK) */
/*             WHERE L(I) IS THE LENGTH OF AN ESTIMATED SEARCH DIRECTION */
/*             COMPUTED BY USING DIMENSION I */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --b;
    --r__;

    /* Function Body */
    *newdim = *kp1;
    *eta = 1.;
    if (*kp1 <= 0) {
	return 0;
    }

/*     COMPUTE THE LENGTH OF THE ESTIMATED SEARCH DIRECTIONS */
/*     AND RIGHTHAND SIDES */

    r__[1] = abs(b[1]);
    b[1] = (d__1 = b[1] / a[a_dim1 + 1], abs(d__1));
    if (*kp1 == 1) {
	goto L15;
    }
    i__1 = *kp1;
    for (i__ = 2; i__ <= i__1; ++i__) {
	r__[i__] = b[i__];
	b[i__] /= a[i__ + i__ * a_dim1];
	b[i__] = enlsip_dnrm2_(&c__2, &b[i__ - 1], &c__1);
	r__[i__] = enlsip_dnrm2_(&c__2, &r__[i__ - 1], &c__1);
/* L10: */
    }
L15:
    rn = r__[*kp1];
    sn = b[*kp1];

/*     DETERMINE LOWEST POSSIBLE DIMENSION */

/*      DO 20 I=1,KP1 */
/*           MINDIM=I */
/*           IF(R(I).GT.C1*RN) GOTO 30 */
/*   20 CONTINUE */
/*   30 CONTINUE */
    dsum = 0.;
    psimax = 0.;
    i__1 = *kp1;
    for (i__ = 1; i__ <= i__1; ++i__) {
/* Computing 2nd power */
	d__1 = b[i__];
	dsum += d__1 * d__1;
	psi = sqrt(dsum) * (d__1 = a[i__ + i__ * a_dim1], abs(d__1));
	if (psi > psimax) {
	    psimax = psi;
	    mindim = i__;
	}
/* L20: */
    }

/*     IF THE LATEST STEP WAS FAIRLY GOOD THE DIMENSION MUST NOT */
/*     BE DECREASED */

/*      IF((ALFKM1.GE.STEPB).OR.(PGRESS.GT.PGB1*PRELIN**2).OR. */
/*     1  (PGRESS.GT.PGB2*ASPREV**2)) MINDIM=MAX0(MINDIM,KM1RNK) */
    k = mindim;
    if (*restar) {
	goto L40;
    }
    if (*km1rnk == *kp1 || *km1rnk <= 0) {
	pregn_(&b[1], &sn, &r__[1], &rn, &mindim, kp1, &ik);
    }
    if (*km1rnk != *kp1 && *km1rnk > 0) {
	presub_(&b[1], &r__[1], &rn, &c1, kp1, km1rnk, pgress, prelin, asprev,
		 alfkm1, &ik);
    }
    *newdim = max(mindim,ik);
    return 0;
L40:
    *newdim = min(*kp1,*km1rnk);
    *newdim = max(*newdim,0);
    if (*newdim == 0) {
	return 0;
    }
/* Computing MAX */
    i__1 = *km1rnk - 1;
    k = max(i__1,1);
    if (b[*newdim] != 0.) {
	*eta = b[k] / b[*newdim];
    }
    return 0;
} /* dimupp_ */


/*     DOUBLE PRECISION VERSION 880505 */
/* MULEST */
/* Subroutine */ int mulest_(integer *time, doublereal *a, integer *mda, 
	integer *t, integer *n, doublereal *g, doublereal *bv, integer *j, 
	doublereal *tol, doublereal *d1, doublereal *h__, integer *mdh, 
	doublereal *pivot, integer *p, integer *iscale, doublereal *diag, 
	doublereal *v, integer *pranka, doublereal *residu, doublereal *s, 
	doublereal *u, doublereal *w)
{
    /* System generated locals */
    integer a_dim1, a_offset, h_dim1, h_offset, i__1, i__2;
    doublereal d__1;

    /* Local variables */
    static integer ctrl;
    extern /* Subroutine */ int ptrv_(integer *, integer *, doublereal *, 
	    integer *, integer *);
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    extern /* Subroutine */ int prod1_(doublereal *, integer *, doublereal *, 
	    doublereal *, doublereal *, integer *, integer *);
    static integer i__, k, k1, k2;
    extern /* Subroutine */ int given1_(doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *), given2_(doublereal *, 
	    doublereal *, doublereal *, doublereal *);
    static doublereal co, pi, si;
    extern /* Subroutine */ int pv_(integer *, integer *, doublereal *, 
	    integer *, integer *), pspecf_(integer *, integer *, doublereal *,
	     doublereal *), atolow_(integer *, integer *, integer *, 
	    doublereal *, integer *, doublereal *, integer *, doublereal *, 
	    doublereal *), atsolv_(doublereal *, integer *, integer *, 
	    doublereal *, doublereal *, integer *, doublereal *);
    static integer ip1;
    extern /* Subroutine */ int nzestm_(doublereal *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, doublereal *, doublereal *)
	    ;
    static integer tp1;
    static doublereal eta;
    static integer ist;
    static doublereal com1;


/*     THE MAIN PURPOSE IS TO */
/*     COMPUTE ESTIMATES OF LAGRANGE MULTIPLIERS BY SOLVING FOR V */
/*     THE OVERDETERMINED SYSTEM */
/*          A(TR)*V = G   WHERE A IS  T*N  (T<=N) */
/*                                      -1 */
/*     THEN V IS REPLACED BY V-(A*A(TR))  *H */
/*     THE SOLUTION IS COMPUTED BY DETERMINING MATRICES Q1 AND P1 SO */
/*     THAT   P1(TR)*A*Q1 = (L@D0) */
/*     WHERE  Q1 IS SYMMETRIC ORTHOGONAL   N*N */
/*            P1 IS A PERMUTATION MATRIX   T*T */
/*            L IS LOWER TRIANGULAR       T*T */
/*     THIS FACTORIZATION CAN BE USED TO SOLVE       A(TR)*V = G */
/*     IF WE USE THE TRANSPOSE OF IT                T */
/*     NAMELY                   Q1(TR)*A(TR)*P1 = (L ) */
/*                                                (0 ) */
/*     THE FACTORIZATION  P1(TR)*A*Q1 */
/*     WILL LATER BE USED TO SOLVE THE SYSTEM  A*DX = BV */
/*     FORM  BV@D= P1(TR)*BV   TO BE ABLE TO COMPLETE THE SOLUTION LATER */

/*     THIS FACTORIZATION ARE UPDATED IF TIME>1 */

/*     ON ENTRY..... */

/*     TIME         INTEGER. THIS ROUTINE CAN BE CALLED IN 3 DIFFERENT */
/*                  SITUATIONS DEPENDING ON THE VALUE OF TIME */
/*                  IF TIME */
/*                  =1  COMPUTE FACTORIZATION  P1(TR)*A*Q1=(L@D0) */
/*                      L IS T*T LOWER TRIANGULAR */
/*                  =2  COLUMN J IN L(TR) IS DELETED I.E. ROW J IN ARRAY A */
/*                      COMPUTE GIVENS ROTATIONS SO THAT THE MODIFIED */
/*                      L-MATRIX IS TRANSFORMED TO LOWER TRIANGULAR */
/*                      FORM L2 WHERE  L2 IS (T-1)*(T-1) */
/*                  >2  COLUMN J IN L(TR) IS DELETED */
/*                      COMPUTE GIVENS ROTATIONS AS IF TIME=2 BUT */
/*                      DO MULTIPLICATIONS BY GIVENS MATRICES ONE BY ONE */
/*     A(T,N),MDA   REAL MATRIX OF ORDER T*N. MDA IS THE LEADING */
/*                  DIMENSION OF ARRAY A IN THE CALLING PROGRAM */
/*                  IF TIME=1 */
/*                     THEN A CONTAINS MATRIX TO BE FACTORIZED */
/*                     ELSE A CONTAINS L AS FIRST T COLUMNS AND */
/*                               NORMALS DEFINING Q1 AS FIRST T ROWS */
/*     T            INTEGER. NO. OF CONSTRAINTS IN CURRENT WORKING SET */
/*     G(N)         REAL VECTOR OF ORDER N. THE GRADIENT OF THE */
/*                  OBJECTIVE FUNCTION IF TIME=1.  IF TIME>1 */
/*                  THEN G=Q1(TR)*G */
/*     BV(T)        REAL VECTOR OF ORDER T */
/*                  IF TIME=1 */
/*                     THEN BV(I),I=1,...T IS NEG. VALUE OF CONSTRAINTS */
/*                            IN CURRENT WORKING SET */
/*                     ELSE BV IS MODIFIED BV SUCH THAT BV=P(TR)*BV */
/*     J            INTEGER */
/*                  IF TIME=1 */
/*                     THEN J IS UNDEFINED */
/*                     ELSE J IS THE COLUMN OF MATRIX L(TR) THAT IS */
/*                            DELETED */
/*     TOL          REAL. A SMALL POSITIVE VALUE USED TO DETERMINE */
/*                  THE PSEUDO RANK OF THE MATRIX A */
/*     D1(T)        REAL VECTOR OF ORDER T */
/*                  IF TIME=1 */
/*                     THEN D1 IS UNDEFINED */
/*                     ELSE D1 CONTAINS MISSING ELEMENTS OF NORMALS */
/*                          THAT DEFINE THE ORTHOGONAL MATRIX Q1 */
/*     H(N,N),MDH   REAL MATRIX OF ORDER N*N. MDH IS THE LEADING */
/*                  DIMENSION OF ARRAY H IN THE CALLING PROGRAM */
/*                  IF TIME=1 */
/*                     THEN H IS UNDEFINED */
/*                     ELSE H IS THE MATRIX FORMED WHEN GIVENS ROTATIONS */
/*                          IS MULTIPLIED TOGETHER */
/*     PIVOT(T)     REAL VECTOR OF ORDER PRANKA (FOR TIME=1 SEE PRANKA) */
/*                  IF TIME =1 */
/*                     THEN PIVOT IS UNDEFINED */
/*                     ELSE PIVOT HOLDS THE PIVOT ELEMENTS USED WHEN */
/*                          THE A-MATRIX WAS TRANSFORMED TO LOWER */
/*                          TRIANGULAR FORM */
/*     P(T)         INTEGER VECTOR OF ORDER T HOLDING THE PIVOT RECORD */
/*                  WHEN P1(TR)*A*Q1 IS COMPUTED */
/*                  IF TIME=1 */
/*                     THEN P IS UNDEFINED */
/*                     ELSE P REPRESENTS THE PIVOT MATRIX P1 */
/*     ISCALE     INTEGER. IF ISCALE */
/*                         =0 NO SCALING OF MATRIX A IS DONE */
/*                         >0 SCALING OF THE ROWS OF A IS DONE */
/*     DIAG(T)    REAL VECTOR OF ORDER T HOLDING THE DIAGONAL ELEMENTS */
/*                OF THE SCALING MATRIX IF ISCALE>0 */
/*                OTHERWISE DIAG(I) HOLDS THE EUCLIDEAN LENGTH OF ROW */
/*                NO. I OF THE MATRIX A */

/*     ON RETURN...... */
/*     TIME         TIME@D=TIME+1 */
/*     A(T,N)       SEE ON INPUT */
/*     H(N,N)       SEE ON INPUT */
/*     D1(T)        SEE ON INPUT */
/*     P(T)         SEE ON INPUT */
/*     V(T)         REAL VECTOR OF ORDER T. */
/*                  HOLDS THE ESTIMATED LAGRANGE MULTIPLIERS */
/*     PRANKA       INTEGER */
/*                  IF TIME=1 */
/*                    THEN PRANKA@D=PSEUDORANK OF A OBTAINED BY USING TOL */
/*                    ELSE PRANKA =T */
/*     RESIDU       REAL */
/*                  THE RESIDUAL OBTAINED WHEN A(TR)*L = G */
/*                  IS SOLVED */

/*     WORKING AREAS...... */
/*     S(N),U(N),W(N)   REAL VECTORS ALL OF ORDER N */

/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     INTERNAL VARIABLES....... */

    /* Parameter adjustments */
    --w;
    --u;
    --s;
    --g;
    a_dim1 = *mda;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --bv;
    --d1;
    h_dim1 = *mdh;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;
    --pivot;
    --p;
    --diag;
    --v;

    /* Function Body */
    ctrl = *time;
    ++(*time);
    *pranka = *t;
    *residu = 0.;
    if (ctrl == 1) {
	goto L100;
    }
    if (ctrl >= 2) {
	goto L200;
    }
L100:
    if (*t == 0) {
	*residu = enlsip_dnrm2_(n, &g[1], &c__1);
    }
    if (*t == 0) {
	return 0;
    }

/*     FACTORIZE MATRIX A SO THAT    P1(TR)*A*Q1=(L@D0) */
/*     AND FORM    G@D=Q1(TR)*G */

    atolow_(t, n, &p[1], &a[a_offset], mda, tol, pranka, &d1[1], &g[1]);

/*     SAVE THE COMPUTED PIVOT ELEMENTS IN THE PIVOT-VECTOR */

    i__1 = *pranka;
    for (i__ = 1; i__ <= i__1; ++i__) {
	pivot[i__] = a[i__ + i__ * a_dim1];
/* L110: */
    }

/*     TRANSFORM RIGHT HAND SIDE OF SYSTEM A*DX=BV */

    ptrv_(&p[1], t, &bv[1], t, &c__1);

/*     COMPUTE ESTIMATES OF LAGRANGE MULTIPLIERS */

    atsolv_(&a[a_offset], mda, pranka, &g[1], &v[1], n, residu);
    if (*pranka == *t) {
	goto L130;
    }
    ip1 = *pranka + 1;
    i__1 = *t;
    for (i__ = ip1; i__ <= i__1; ++i__) {
	v[i__] = 0.;
/* L120: */
    }
L130:
    nzestm_(&a[a_offset], mda, pranka, t, &bv[1], &v[1], &w[1], &u[1]);

/*     DO BACK PERMUTATIONS */

    pv_(&p[1], t, &v[1], t, &c__1);
    goto L290;
L200:
    if (*t == 0) {
	return 0;
    }

/*     COMPUTE GIVENS ROTATIONS SO THAT THE LOWER HESSENBERG */
/*     MATRIX L IS TRANSFORMED TO LOWER TRIANGULAR FORM */

    if (ctrl >= 3) {
	goto L215;
    }

/*     INITIATE THE PRODUCT OF GIVENS MATRICES */

    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	i__2 = *n;
	for (k = 1; k <= i__2; ++k) {
	    h__[i__ + k * h_dim1] = 0.;
/* L205: */
	}
	h__[i__ + i__ * h_dim1] = 1.;
/* L210: */
    }
L215:
/*      PRANKA=T */
    tp1 = *t + 1;
    if (*j == tp1) {
	goto L280;
    }
    pi = machin_1.drelpr;
    ist = *j;
    i__1 = *t;
    for (i__ = *j; i__ <= i__1; ++i__) {
	ip1 = i__ + 1;
	k1 = i__;
	k2 = i__;
	if (i__ > *j) {
	    ++k1;
	}
	if (ip1 > *j) {
	    ++k2;
	}

/*     COMPUTE GIVENS ROTATION */

	given1_(&a[k1 + i__ * a_dim1], &a[k2 + ip1 * a_dim1], &co, &si, &a[
		i__ + i__ * a_dim1]);

/*     AND APPLY IT TO COLUMN I AND I+1 OF ARRAY A WHERE THE */
/*     LOWER TRIANGULAR MATRIX L IS STORED */

	if (ip1 > *t) {
	    goto L230;
	}
	i__2 = *t;
	for (k = ip1; k <= i__2; ++k) {
	    k1 = k;
	    k2 = k;
	    if (i__ > *j) {
		++k1;
	    }
	    if (ip1 > *j) {
		++k2;
	    }
	    if (si != 0.) {
		given2_(&co, &si, &a[k1 + i__ * a_dim1], &a[k2 + ip1 * a_dim1]
			);
	    }
	    a[k + i__ * a_dim1] = a[k1 + i__ * a_dim1];
/* L220: */
	}
L230:

/*     IF SI IS EXACTLY ZERO THEN NO GIVENS TRANSFORMATION */
/*     WAS NECESSARY */

	if (si != 0.) {
	    goto L235;
	}
	u[i__] = eta;
	w[i__] = -com1 / eta;
	if (ctrl == 2) {
	    prod1_(&h__[h_offset], mdh, &s[1], &u[1], &w[1], &ist, &i__);
	}
/*     IF(CTRL.GE.3) CALL PROD2(H,MDH,N,S,U,W,IST,I,W1) */
	ist = i__ + 1;
	goto L250;
L235:

/*     APPLY IT TO THE RIGHT HAND SIDE OF Q1(TR)*A(TR)*P1= Q1(TR)*G */

	given2_(&co, &si, &g[i__], &g[ip1]);

/*     SAVE TO BE ABLE TO FORM THE PRODUCT OF GIVENS MATRICES */

	s[i__] = si;
	if (i__ > ist) {
	    goto L240;
	}
	u[i__] = co / pi;
	w[i__] = pi;
	eta = si / pi;
	com1 = co;
	goto L250;
L240:
	u[i__] = co * eta;
	w[i__] = -com1 / eta;
	eta = si * eta;
	com1 = co;
L250:
	;
    }
    u[tp1] = eta;
    w[tp1] = -com1 / eta;

/*     FORM THE PRODUCT OF GIVENS MATRICES */

    if (ctrl == 2) {
	prod1_(&h__[h_offset], mdh, &s[1], &u[1], &w[1], &ist, &tp1);
    }
/*     IF(CTRL.GE.3) CALL PROD2(H,MDH,N,S,U,W,IST,TP1,W1) */
L280:

/*     what pseudo rank should be used */

    k = 1;
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if ((d__1 = a[i__ + i__ * a_dim1], abs(d__1)) < *tol) {
	    goto L320;
	}
	++k;
/* L310: */
    }
L320:
    *pranka = k - 1;

/*     COMPUTE ESTIMATES AND RESIDUAL */

    atsolv_(&a[a_offset], mda, pranka, &g[1], &v[1], n, residu);
    nzestm_(&a[a_offset], mda, pranka, t, &bv[1], &v[1], &w[1], &u[1]);
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	w[i__] = v[i__];
/* L285: */
    }

/*     DO BACK PERMUTATIONS */

    pspecf_(t, &p[1], &w[1], &v[1]);
L290:

/*     BACK TRANSFORM DUE TO ROW SCALING OF MATRIX A */

    if (*iscale == 0) {
	return 0;
    }
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	v[i__] = diag[i__] * v[i__];
/* L300: */
    }
    return 0;
} /* mulest_ */

/*     DOUBLE PRECISION VERSION 880425 */

/* WEIGHT */
/* Subroutine */ int weight_(doublereal *w, doublereal *wold, doublereal *v1, 
	doublereal *f, doublereal *fsum, integer *m, integer *active, integer 
	*t, doublereal *h__, integer *l, integer *norm, integer *dima, 
	logical *old, doublereal *whnrm2, doublereal *psi0, doublereal *dpsi0,
	 doublereal *atwa, doublereal *ctc, doublereal *ctd, doublereal *fnew,
	 doublereal *hnew, doublereal *v2)
{
    /* Initialized data */

    static doublereal delta = .25;

    /* System generated locals */
    integer i__1;
    doublereal d__1, d__2;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal alfa, dalf, btwa;
    extern doublereal enlsip_dnrm2_(integer *, doublereal *, integer *);
    static integer i__, k;
    extern /* Subroutine */ int scalv_(doublereal *, doublereal *, integer *);
    static doublereal anorm, bnorm, cnorm, dnorm, whsum;
    extern /* Subroutine */ int eucnrm_(doublereal *, doublereal *, integer *,
	     integer *, doublereal *, doublereal *, doublereal *, integer *, 
	    doublereal *, integer *, integer *, doublereal *, doublereal *), 
	    maxnrm_(doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, integer *, integer *);
    static doublereal ata;
    static integer mpi;
    static doublereal rmy;


/*     DETERMINE THE PENALTY CONSTANTS THAT SHOULD BE USED IN */
/*     THE CURRENT LINESEARCH WHERE PSI(ALPHA)=PSI(XOLD+ALPHA*DX) */
/*     IS APPROXIMATELY MINIMIZED */

/*     ON ENTRY@D */

/*     W()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING PENALTY CONSTANTS USED IN THE LATEST TRY */
/*     WOLD()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING PENALTY CONSTANTS USED IN THE LATEST TRY */
/*             UNLESS THAT WAS A RESTART.IF RESTART WOLD=W */
/*     V1()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+T */
/*             CONTAINING THE COMPOUND VECTOR  (C*DX   ) */
/*                                             (AHAT*DX) */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M */
/*             CONTAINING THE VALUE OF THE RESIDUALS */
/*     FSUM    REAL SCALAR CONTAINING THE SUM OF SQUARED RESIDUALS */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*             CONTAINING INDECES FOR CONSTRAINTS IN CURRENT WORKING SET */
/*     T       INTEGER SCALAR CONTAINING NUMBER OF CONSTRAINTS IN */
/*             CURRENT WORKING SET */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINING THE VALUE OF THE CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINIG TOTAL NUMBER OF CONSTRAINTS */
/*     NORM    INTEGER SCALAR CONTAINING A CODE THAT CONTROLS WHICH */
/*             NORM THAT SHOULD BE USED WHEN PENALTY WEIGHTS ARE UPDATED */
/*             = 0 MAXIMUM NORM */
/*             = 2 EUCLIDEAN NORM */
/*     DIMA    INTEGER SCALAR CONTAINING DIMENSION USED FOR MATRIX AHAT */
/*     OLD     LOGICAL SCALAR = TRUE IF THE SEQUENCE OF PENALTY */
/*             CONSTANTS MAY RESTART BEGINNING WITH THE CONSTANTS */
/*             IN WOLD. = FALSE IF NO RESTART IS ALLOWED */

/*     ON RETURN@D */

/*     W()     CONTAINS THE NEW PENALTY CONSTANTS */
/*     PSI0    REAL SCALAR-CONTAINS PSI(0,W)  (SEE ABOVE) */
/*     DPSI0   REAL SCALAR-CONTAINS THE DERIVATIVE OF PSI(XOLD+ALF*DX,W) */
/*             WITH RESPECT TO ALF AT ALF=0 */
/*                                        T */
/*     ATWA    REAL SCALAR-CONTAINS (A*DX) *W*(A*DX) */
/*     CTC     REAL SCALAR-CONTAINS II C*DX II**2 */
/*     CTD     REAL SCALAR-CONTAINS THE DERIVATIVE OF FI(ALFHA) */
/*             AT ALPHA=0 */

/*     WORKING AREAS@D */

/*     FNEW()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*     HNEW()  REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION T */
/*     V2()    REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */

/*     INTERNAL VARIABLES */

    /* Parameter adjustments */
    --w;
    --wold;
    --v1;
    --f;
    --active;
    --h__;
    --fnew;
    --hnew;
    --v2;

    /* Function Body */
/*                            T */
/*     COMPUTE   ATWA = (A*DX) *W*(A*DX) */
/*                ATA = II A*DX II**2 */
/*                            T */
/*               BTWA = (A*DX) *W*H */


/*     RESTART THE PENALTY CONSTANT SEQUENCE IF SO INDICATED */

    if (! (*old)) {
	goto L20;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	w[i__] = wold[i__];
/* L10: */
    }
L20:
/*      write(10,*) 'In WEIGHT: dima= ',dima */
/*      write(10,*) 'The constraints' */
/*      write(10,*) (h(i),i=1,L) */
/*      write(10,*) 'A*p appr. = -c' */
/*      write(10,*) (v1(m+i),i=1,t) */
    anorm = enlsip_dnrm2_(dima, &v1[*m + 1], &c__1);
/*      anorm=0.0 */
/*      do 11 i=1,dima */
/*       anorm=anorm+v1(m+i)**2 */
/*   11 continue */
/*      anorm=sqrt(anorm) */
/* Computing 2nd power */
    d__1 = anorm;
    ata = d__1 * d__1;
    if (anorm != 0.) {
	scalv_(&v1[*m + 1], &anorm, dima);
    }
    btwa = 0.;
    *atwa = 0.;
    bnorm = 0.;
    if (*dima <= 0) {
	goto L50;
    }
    i__1 = *dima;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	mpi = *m + i__;
/* Computing 2nd power */
	d__1 = v1[mpi];
	*atwa += w[k] * (d__1 * d__1);
	if ((d__1 = h__[k], abs(d__1)) > bnorm) {
	    bnorm = (d__2 = h__[k], abs(d__2));
	}
/* L30: */
    }
    if (bnorm != 0.) {
	scalv_(&h__[1], &bnorm, l);
    }
    *atwa = anorm * anorm * *atwa;
    i__1 = *dima;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	mpi = *m + i__;
	btwa += w[k] * v1[mpi] * h__[k];
/* L40: */
    }
    btwa = bnorm * anorm * btwa;
L50:

/*     COMPUTE     CTC = II C*DX II**2 */
/*                         T  T */
/*                 CTD = DX *C *F */

    cnorm = enlsip_dnrm2_(m, &v1[1], &c__1);
/* Computing 2nd power */
    d__1 = cnorm;
    *ctc = d__1 * d__1;
    if (cnorm != 0.) {
	scalv_(&v1[1], &cnorm, m);
    }
    dnorm = sqrt(*fsum);
    if (dnorm != 0.) {
	scalv_(&f[1], &dnorm, m);
    }
    *ctd = 0.;
    i__1 = *m;
    for (i__ = 1; i__ <= i__1; ++i__) {
	*ctd += v1[i__] * f[i__];
/* L60: */
    }
    *ctd = cnorm * dnorm * *ctd;

/*     COMPUTE THE VALUE  ALFA(W) */

    if (*atwa + *ctc == 0.) {
	alfa = 1.;
    } else {
	alfa = (-btwa - *ctd) / (*atwa + *ctc);
    }
/*      ALFA=(-BTWA-CTD)/(ATWA+CTC) */
    *psi0 = *fsum * .5;
    *dpsi0 = *ctd;
/*      write(10,*) 'In WEIGHT: dpsi0= ',dpsi0 */
    *atwa = 0.;
    whsum = 0.;
    if (*t <= 0) {
	goto L100;
    }
    if (anorm != 0.) {
	d__1 = 1. / anorm;
	scalv_(&v1[*m + 1], &d__1, dima);
	scalv_(&v1[*m + 1], &anorm, t);
    }
    rmy = (d__1 = -(*ctd) - *ctc, abs(d__1)) / delta - *ctc;

/*     COMPUTE THE NEW PENALTY CONSTANTS */

    if (*norm == 0) {
	maxnrm_(&ata, &rmy, &alfa, &delta, &w[1], &active[1], t);
    }
    if (*norm != 0) {
	eucnrm_(&v1[*m + 1], &h__[1], &active[1], t, &rmy, &anorm, &bnorm, 
		dima, &w[1], l, (integer*)&fnew[1], &hnew[1], &v2[1]);
    }
/*      write(10,*) 'New penalty weights from EUCNRM' */
/*      write(10,*) (w(i),i=1,L) */
/*      write(10,*) 'Active(i),i=1,t' */
/*      write(10,*) (active(i),i=1,t) */

/*     COMPUTE    PSI0 = PSI(0) */
/*               DPSI0 = (SEE ABOVE) */
/*                BTWA = (SEE ABOVE) */

    btwa = 0.;
    i__1 = *t;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = active[i__];
	mpi = *m + i__;
/* Computing 2nd power */
	d__1 = h__[k];
	whsum += w[k] * (d__1 * d__1);
	btwa += w[k] * v1[mpi] * h__[k];
/* Computing 2nd power */
	d__1 = v1[mpi];
	*atwa += w[k] * (d__1 * d__1);
/* L80: */
    }
/*      write(10,*) 'anorm,bnorm,btwa= ',anorm,bnorm,btwa */
    whsum = bnorm * bnorm * whsum;
    btwa = anorm * bnorm * btwa;
    *atwa = anorm * anorm * *atwa;
    *psi0 = (whsum + *fsum) * .5;
    *dpsi0 = btwa + *ctd;
/*      write(10,*) 'After rescaling: atwa,btwa= ',atwa,btwa */
    dalf = (-btwa - *ctd) / (*atwa + *ctc);
/*      write(10,*) 'dalf in WEIGHT: ',dalf */
/*      write(10,*) 'dpsi0= ',dpsi0 */
L100:
    *whnrm2 = whsum;
    if (anorm != 0.) {
	d__1 = 1. / anorm;
	scalv_(&v1[*m + 1], &d__1, t);
    }
    if (bnorm != 0.) {
	d__1 = 1. / bnorm;
	scalv_(&h__[1], &d__1, l);
    }
    if (cnorm != 0.) {
	d__1 = 1. / cnorm;
	scalv_(&v1[1], &d__1, m);
    }
    if (dnorm != 0.) {
	d__1 = 1. / dnorm;
	scalv_(&f[1], &d__1, m);
    }
    return 0;
} /* weight_ */

