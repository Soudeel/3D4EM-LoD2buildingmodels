/* dblenls.f -- translated by f2c (version 20000817).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/
#define DEBUG 0
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

/*     ENLSIP DOUBLE PRECISION VERSION 841005 */
/* ENLSIP */
/* Subroutine */ int enlsip_(doublereal *x, integer *n, integer *m, integer *
	p, integer *l, integer *mdi, integer *mdr, U_fp ffunc, U_fp hfunc, 
	integer *wint, doublereal *wreal, integer *exit, doublereal *f, 
	doublereal *h__, integer *active)
{
    /* System generated locals */
    integer i__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static integer i__, scale, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11;
    extern /* Subroutine */ int releps_(doublereal *), nlsnip_(doublereal *, 
	    integer *, integer *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, logical *, U_fp, U_fp, integer *,
	     doublereal *, integer *, integer *, integer *, integer *, 
	    integer *, integer *, integer *, doublereal *, doublereal *, 
	    integer *, doublereal *, integer *, integer *, integer *, integer 
	    *, integer *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *, 
	    doublereal *, doublereal *, doublereal *, doublereal *);
    static integer mp1, mp2, mp3, mp4, mp5, mp6, mp7, mp8, mp9;
    static doublereal rootsp;
    static integer mda, mdc, bnd, mdf, mdg;
    static logical sec;
    static integer mp10;


/*   ***************************************************************** */
/*   * THIS IS AN EASY-TO-USE VERSION OF THE SUBROUTINE NLSNIP.      * */
/*   * NLSNIP IS DEVELOPED BY PER LINDSTR\M AND PER-]KE WEDIN AT THE * */
/*   * INSTITUTE OF INFORMATION PROCESSING UNIVERSITY OF UME],       * */
/*   * S-90187 UME], SWEDEN                                          * */
/*   ***************************************************************** */

/*     THE FOLLOWING INPUT PARAMETERS ARE GIVEN DEFAULT VALUES */
/*     PROVIDED THE USER HAS GIVEN A NEGATIVE VALUE TO THE */
/*     CORRESPONDING LOCATIONS IN THE AREAS WINT AND WREAL */
/*     RESPECTIVELY */

/*     IPRINT=1    (WRITE EVERY STEP) */
/*     NOUT=10     (WRITING IS DONE ON UNIT NO. 10) */
/*     MAXIT=20*N  (20 TIMES THE NO. OF PARAMETERS) */
/*     NORM=2      (EUCLIDEAN NORM FOR PENALTY UPDATING) */
/*     SCALE=0     (INTERNAL SCALING IS NOT USED WHEN THE SEARCH */
/*                  DIRECTION IS COMPUTED) */
/*     SEC=TRUE    (A SECOND DERIVATIVE (QP-BASED PROJECTED */
/*                  LAGRANGIAN) METHOD IS PERMITTED AT THE */
/*                  END OF THE ITERATION IN SOME SITUATIONS */
/*                  (LARGE RESIDULAS)) */
/*     TOL= SQRT(DOUBLE RELATIVE PRECISION) */
/*     EPSREL= SQRT(DOUBLE RELATIVE PRECISION) */
/*     EPSABS= DOUBLE RELATIVE PRECISION */
/*     EPSX= SQRT(DOUBLE RELATIVE PRECISION) */
/*     EPSH= SQRT(DOUBLE RELATIVE PRECISION) */
/*     W(I)= 0 ;I=1,2,......,L */
/*       (I.E. THE INITIAL PENALTY WEIGHTS ARE CHOSEN AUTOMATICALLY) */

/*     PURPOSE... */
/*     SOLVE THE NONLINEAR LEAST SQUARES PROBLEM */

/*     MINIMIZE  0.5* II F(X) II**2 */
/*        X */
/*          SUBJECT TO THE NONLINEAR CONSTRAINTS */
/*     H (X) = 0     I=1,2,.....,P   (P<=N) */
/*      I */

/*     H (X)   >= 0  J=P+1,....,L  (L>=P) , (M+P)>= N */
/*      J */

/*     WHERE F(X) IS M-DIMENSIONAL AND X IS N-DIMENSIONAL */

/*     THE DIMENSION OF SOME ARRAYS IN THE CALLING PROGRAM DEPENDS */
/*     ON BND=MIN(L,N) */

/*     ON ENTRY: */

/*     X()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION N */
/*             CONTAINING A FIRST APPROXIMATION OF THE SOLUTION POINT */
/*     N       INTEGER SCALAR CONTAINING THE NUMBER OF PARAMETERS */
/*     M       INTEGER SCALAR CONTAINING NUMBER OF RESIDUALS IN F(X) */
/*     P       INTEGER SCALAR CONTAINING NUMBER OF EQUALITY CONSTRAINTS */
/*     L       INTEGER SCALAR CONTAINING THE TOTAL NUMBER OF CONSTRAINTS */
/*     MDI     INTEGER SCALAR CONTAINING THE LENGTH OF THE WORKING */
/*             AREA WINT()   (SEE BELOW UNDER WORKING AREAS) */
/*     MDR     INTEGER SCALAR CONTAINING THE LENGTH OF THE WORKING */
/*             AREA WREAL()  (SEE BELOW UNDER WORKING AREAS) */
/*     THE FOLLOWING 11+L PARAMETERS ARE GIVEN DEFAULT VALUES PROVIDED */
/*     THE CORRESPONDING LOCATION IS < 0 ON ENTRY. THE DEFAULT VALUES */
/*     ARE GIVEN IN BRACKETS TOGETHER WITH THE ORIGINAL NAME OF */
/*     THE PARAMETER. WHERE APPLICABLE, THESE DEFAULT VALUES ARE RETURNED */
/*     IN THE CORRESPONDING LOCATION WHEN THE LOCATION IS <0 ON ENTRY. */

/*     WINT(1) (IPRINT=1) STEP BETWEEN WRITING */
/*     WINT(2) (NOUT=10) FORTRAN UNIT FOR WRITING */
/*     WINT(3) (MAXIT=20*N) MAXIMUM NO. OF PERMITTED ITERATIONS */
/*     WINT(4) (NORM=2) EUCLIDEAN NORM FOR UPDATING PENALTY WEIGHTS */
/*     WINT(5) (NO INTERNAL) INDICATOR FOR INTERNAL SCALING */
/*     WINT(6) (2:ND DERIVATIVES PERMITTED) INDICATOR FOR PERMITTING */
/*              A SECOND DERIVATIVE METHOD AT THE END */
/*     WREAL(1) (TOL=SQRT(DRELPR)) PSEUDO-RANK TOLERANCE CONSTANT */
/*     WREAL(2) (EPSREL=SQRT(DRELPR)) RELATIVE CONVERGENCE CONSTANT */
/*     WREAL(3) (EPSABS=DRELPR) ABSOLUTE CONVERGENCE CONSTANT */
/*     WREAL(4) (EPSX=SQRT(DRELPR)) PARAMETER CONVERGENCE CONSTANT */
/*     WREAL(5) (EPSH=SQRT(DRELPR)) CONSTRAINT CONVERGENCE CONSTANT */
/*     WREAL(I)  I=8,9,...,L+7 (W(I)) THE PENALTY WEIGHT CONSTANTS */

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

/*     ON RETURN:    AND EXIT .NE.-1  AND  EXIT.GE.-10 */

/*     X()     CONTAINS THE TERMINATION POINT (HOPEFULLY THE SOLUTION) */
/*     P       CONTAINS NUMBER OF ACTIVE CONSTRAINTS AT X */
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
/*             =   -3 IF CRITERION 10) IS SATISFIED */
/*             =   -4 IF CRITERION 11) IS SATISFIED */
/*             =   -5 IF CRITERION 12) IS SATISFIED */
/*             =   -6 IF CRITERION 13) IS SATISFIED */
/*             =   -7 IF RESIDUALS OR CONSTRAINTS ARE UNCOMPUTABLE AT X */
/*             =   -8 IF MORE THAN N CONSTRAINTS IN FIRST WORKING */
/*                    SET (CHOOSE A BETTER STARTING POINT) */
/*             =   -9  IF TOO MANY (>5) NEWTON STEPS HAVE BEEN USED */
/*             < -10 TERMINATION DUE TO A USER STOP INDICATOR */
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
/*           (L:0) = P1 *A*Q1      DY1= L  *P1 *(-H) */
/*                                       T */
/*           (C1:C2) = C*Q1      (U) = Q3 *C2*P3 */
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
/*      7) SQRT(DRELPR)/II DX(GN) II >0.25 (GN=GAUSS NEWTON) */
/*      8) THE LAST DIGIT IN THE CONVERGENCE CODE (SEE BELOW) INDICATES */
/*         HOW THE LASTS STEPS WERE COMPUTED */
/*        = 0 NO TROUBLE (GAUSS-NEWTON THE LAST 3 STEPS) */
/*        = 1 RANKA<>T OR RANKC2<>(N-T) AT THE TERMINATION POINT */
/*            NO 2:ND DERIVATIVES HAS BEEN USED */
/*        = 2 THE METHOD OF NEWTON WAS USED (AT LEAST) IN THE LAST STEP */
/*        = 3 THE 2:ND BUT LAST STEP WAS SUBSPACE MINIMIZATION STEP */
/*            BUT THE LAST TWO WERE GAUSS-NEWTON STEPS */
/*        = 4 THE STEPLENGTH WAS NOT UNIT IN BOTH THE LAST 2 STEPS */

/*     THE ABNORMAL TERMINATION CRITERIA ARE */

/*      9) NUMBER OF ITERATIONS EXCEEDS THE MAXIMUM NUMBER */
/*     10) THE REDUCED HESSIAN EMANATING FROM THE METHOD OF NEWTON */
/*         IS NOT POSITIVE DEFINTE */
/*     11) THE USER HAS NOT ALLOWED USE OF 2:ND DERIVATIVES (NEWTON) */
/*     12) UNDAMPED NEWTON STEP FAILS */
/*     13) THE LATEST DX(GN) IS NOT A DESCENT DIRECTION TO THE MERIT */
/*         FUNCTION (PROBABLY CAUSED BY MISSCALCULATED JACOBIAN) */

/*     WINT(7) (ITER)INTEGER SCALAR-CONTAINS NUMBER OF ITERATIONS */
/*             UNTIL TERMINATION */
/*     WINT(8) (FUNCEV)INTEGER SCALAR-CONTAINS TOTAL NUMBER OF FUNCTION */
/*             EVALUATIONS DONE BY THE ALGORITHM */
/*     WINT(9) (JACEV)INTEGER SCALAR-CONTAINS THE NUMBER OF */
/*             JACOBIAN EVALUATIONS */
/*     WINT(10) (SECEV)INTEGER SCALAR-CONTAINS THE NO. OF FUNCTION */
/*              EVALUATIONS CAUSED BY THE 2:ND DERIVATIVE METHOD */
/*     WINT(11) (LINEV)INTEGER SCALAR-CONTAINS THE NO. OF FUNCTION */
/*              EVALUATIONS DONE BY THE LINESEARCH ALGORITHM */
/*     WINT(12) (RANKA)INTEGER SCALAR-CONTAINS THE ESTIMATED PSEUDO */
/*             RANK OF THE CONSTRAINT MATRIX (MATRIX A) AT THE */
/*             TERMINATION POINT. (RANKA=P IF FULL RANK) */
/*     WINT(13) (RANK)INTEGER SCALAR-CONTAINS ESTIMATED PSEUDO RANK OF */
/*             COMPOUND MATRIX (A) AT THE TERMINATION POINT */
/*                             (C) */
/*             RANK=N IF FULL RANK */
/*     WREAL(6) (PHI)REAL SCALAR-CONTAINS THE VALUE OF THE OBJECTIVE */
/*              FUNCTION AT THE TERMINATION POINT */
/*     WREAL(7) (SPEED)REAL SCALAR-CONTAINS AN ESTIMATE OF THE */
/*              CONVERGENCE FACTOR (SHOULD BE <=1) */
/*     WREAL(I) I=8,9,....,L+7 (W(I))REAL SINGLY SUBSCRIPTED ARRAY OF */
/*              DIMENSION L CONTAINING THE FINAL PENALTY CONSTANTS */
/*     F()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION M+L */
/*             CONTAINS THE VALUE OF THE RESIDUALS AT THE POINT X */
/*             AS THE FIRST M ELEMENTS */
/*     H()     REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION L */
/*             CONTAINS THE VALUE OF THE CONSTRAINTS AT THE POINT X */
/*     ACTIVE()INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION BND+L */
/*             CONTAINS INDECES FOR ACTIVE CONSTRAINTS AT THE POINT X */
/*             SUCH THAT ACTIVE(I) I=1,2,....,P ARE THE INDECES FOR THE */
/*             ACTIVE CONSTRAINTS */

/*     WORKING AREAS: */

/*     WINT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION MDI */
/*            (MDI MUST BE >=(2*(L+BND)+N+13)) */
/*     WREAL() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION MDR */
/*          (MDR MUST BE >=(N*(M+2*N+L+6)+3*(M+2*L)+4*BND+7) */
/*                    WHEN INEQUALITIES ARE PRESENT IN THE PROBLEM) */
/*          (MDR MUST BE >=(N*(M+N+L+6)+3*(M+2*L)+4*BND+7) */
/*                    WHEN THERE ARE ONLY EQUALITIES IN THE PROBLEM) */

/*     COMMON VARIABLES CONTAINING INFORMATION CONCERNING PREVIOUS */
/*     TWO POINTS. THE SUFFICES KM2 AND KM1 IN THE NAMES OF THE */
/*     VARIABLES REPRESENT TIME STEP K-2 AND K-1 */
/*     THESE VARIABLES ARE UPDATED ONLY INSIDE THE ROUTINE EVREST */


/*     COMMON VARIABLES CONTAINING INFORMATION OF RESTART STEPS */


/*     COMMON VARIABLES CONTAINING MACHINE DEPENDENT CONSTANTS */
/*     DRELPR = DOUBLE RELATIVE PRECISION */


/*     THIS PROGRAM PACKAGE USES THE FOLLOWING LINPACK AND BLAS ROUTINES */

/*     DPOSL,DCHDC,DNRM2,DAXPY,DDOT,DSWAP */


/*     INTERNAL VARIABLES */


/*     COMPUTE DOUBLE RELATIVE PRECISION */

    /* Parameter adjustments */
    --x;
    --wint;
    --wreal;
    --f;
    --h__;
    --active;

    /* Function Body */
    releps_(&machin_1.drelpr);
    rootsp = sqrt(machin_1.drelpr);

/*     VALIDATE SOME INPUT VALUES */

    *exit = 0;
    bnd = min(*l,*n);
    if (*l > *p && *mdr < *n * (*m + (*n << 1) + *l + 6) + *m * 3 + *l * 5 + 
	    max(*l,*n) + (bnd << 2) + 7) {
	*exit = -1;
    }
    if (*l == *p && *mdr < *n * (*m + *n + *l + 6) + *m * 3 + *l * 5 + max(*l,
	    *n) + (bnd << 2) + 7) {
	*exit = -1;
    }
    if (*mdi < (*l + bnd << 1) + *n + 13) {
	*exit = -1;
    }
    if (*exit < 0) {
	return 0;
    }

/*     SET DEFAULT VALUES OF MISSING PARAMETERS */

    mdc = *m;
    mda = *l;
    mdg = *n;
    mdf = *n;
    if (wint[1] < 0) {
	wint[1] = 1;
    }
    if (wint[2] < 0) {
	wint[2] = 10;
    }
    if (wint[3] < 0) {
	wint[3] = *n * 20;
    }
    if (wint[4] < 0) {
	wint[4] = 2;
    }
    if (wint[4] != 2) {
	wint[4] = 0;
    }
    scale = 1;
    if (wint[5] < 0) {
	scale = 0;
    }
#if DEBUG	
	printf("Max iter %d\n",wint[3]);
	printf("Using scale of %d\n",scale);
#endif	
	
	
    sec = wint[6] < 0;
    if (wreal[1] < machin_1.drelpr) {
	wreal[1] = rootsp;
    }
    if (wreal[2] < 0.) {
	wreal[2] = rootsp;
    }
    if (wreal[3] < 0.) {
	wreal[3] = machin_1.drelpr;
    }
    if (wreal[4] < 0.) {
	wreal[4] = rootsp;
    }
    if (wreal[5] < 0.) {
	wreal[5] = rootsp;
    }
    if (*l <= 0) {
	goto L20;
    }
    i__1 = *l;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (wreal[i__ + 7] < 0.f) {
	    wreal[i__ + 7] = 0.;
	}
/* L10: */
    }
L20:

/*     SET UP INDEX BOUNDS FOR ARRAYS */

    mp1 = bnd << 1;
    mp2 = mp1 + *n;
    mp3 = mp2 + *l;
    mp4 = *l + bnd;
    mp5 = mp4 + bnd;
    mp6 = mp5 + bnd;
    mp7 = mp6 + *n;
    mp8 = mp7 + *n;
    mp9 = mp8 + *n;
    mp10 = mp9 + *n;
    m1 = mp10 + bnd;
    m2 = m1 + *n;
    m3 = m2 + max(*l,*n);
    m4 = m3 + *l;
    m5 = m4 + *l * *n;
    m6 = m5 + *m + *l;
    m7 = m6 + *m + *l;
    m8 = m7 + *m + *l;
    m9 = m8 + *m * *n;
    m10 = m9 + *n * *n;
    if (*p == *l) {
	m10 = m9;
    }
    m11 = m10 + *n * *n;

/*     THE WORKING AREA WREAL() IS STRUCTURED IN THE FOLLOWING WAY */

/*     TOL = WREAL(1) */
/*     EPSREL = WREAL(2) */
/*     EPSABS = WREAL(3) */
/*     EPSX = WREAL(4) */
/*     EPSH = WREAL(5) */
/*     PHI = WREAL(6) */
/*     SPEED = WREAL(7) */
/*      W = WREAL(8)......WREAL(L+7) */
/*      B = WREAL(L+8)......WREAL(L+BND+7) */
/*     D1 = WREAL(L+BND+8).......WREAL(L+2*BND+7) */
/*     D2 = WREAL(L+2*BND+8).........WREAL(L+3*BND+7) */
/*     D3 = WREAL(L+3*BND+8).........WREAL(L+3*BND+N+7) */
/*      G = WREAL(L+3*BND+N+8).......WREAL(L+3*BND+2*N+7) */
/*     PIVOT = WREAL(L+3*BND+2*N+8)........WREAL(L+3*BND+3*N+7) */
/*     DX = WREAL(L+3*BND+3*N+8).....WREAL(L+3*BND+4*N+7) */
/*      V = WREAL(L+3*BND+4*N+8).....WREAL(L+4*BND+4*N+7) */
/*      U = WREAL(M1+8).....WREAL(M1+7+N) */
/*      S = WREAL(M2+8).....WREAL(M2+7+MAX(L,N)) */
/*     WOLD = WREAL(M3+8).......WREAL(M3+7+L) */
/*      A = WREAL(M4+8).......WREAL(M4+7+L*N) */
/*      D = WREAL(M5+8)......WREAL(M5+7+M+L) */
/*     V1 = WREAL(M6+8)..........WREAL(M6+7+M+L) */
/*     V2 = WREAL(M7+8)...........WREAL(M7+7+M+L) */
/*      C = WREAL(M8+8).........WREAL(M8+7+M*N) */
/*     FMAT = WREAL(M9+8)......WREAL(M10+7) */
/*     GMAT = WREAL(M10+8)........WREAL(M10+N*N+7) */
/*     DIAG = WREAL(M10+N*N+8)......WREAL(M10+N*N+N+7) */

/*     THE WORKING AREA WINT() IS STRUCTURED IN THE FOLLOWING WAY */

/*     IPRINT = WINT(1) */
/*     NOUT = WINT(2) */
/*     MAXIT = WINT(3) */
/*     NORM = WINT(4) */
/*     SCALE = WINT(5) */
/*     SEC = WINT(6) */
/*     ITER = WINT(7) */
/*     FUNCEV = WINT(8) */
/*     JACEV = WINT(9) */
/*     SECEV = WINT(10) */
/*     LINEV = WINT(11) */
/*     RANKA = WINT(12) */
/*     RANK = WINT(13) */
/*     P1 = WINT(14)...................WINT(BND+13) */
/*     P2 = WINT(BND+14)...............WINT(2*BND+13) */
/*     P3 = WINT(2*BND+14).............WINT(2*BND+N+13) */
/*     INACT = WINT(2*BND+N+14)........WINT(2*BND+N+L+13) */
/*     P4 = WINT(2*BND+N+L+14).........WINT(2*BND+N+2*L+13) */

/*     CALL NLSNIP TO MINIMIZE */

    nlsnip_(&x[1], n, &mdc, &mda, &mdg, &mdf, m, p, l, &wreal[1], &wreal[8], &
	    wreal[2], &wreal[3], &wreal[4], &wreal[5], &wint[1], &wint[2], &
	    wint[3], &wint[4], &scale, &sec, (U_fp)ffunc, (U_fp)hfunc, exit, &
	    wreal[6], &wint[7], &wint[8], &wint[9], &wint[10], &wint[11], &
	    wint[12], &wint[13], &f[1], &h__[1], &active[1], &wreal[7], &wint[
	    14], &wint[bnd + 14], &wint[mp1 + 14], &wint[mp2 + 14], &wint[mp3 
	    + 14], &wreal[*l + 8], &wreal[mp4 + 8], &wreal[mp5 + 8], &wreal[
	    mp6 + 8], &wreal[m11 + 8], &wreal[mp7 + 8], &wreal[mp8 + 8], &
	    wreal[mp9 + 8], &wreal[mp10 + 8], &wreal[m1 + 8], &wreal[m2 + 8], 
	    &wreal[m3 + 8], &wreal[m4 + 8], &wreal[m5 + 8], &wreal[m6 + 8], &
	    wreal[m7 + 8], &wreal[m8 + 8], &wreal[m9 + 8], &wreal[m10 + 8]);
    return 0;
} /* enlsip_ */

