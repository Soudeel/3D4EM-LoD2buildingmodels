/* dblmain.f -- translated by f2c (version 20000817).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/


#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "f2c.h" 


#define CROSSVP(v,u,w)          /* CROSS Vector Product */              \
{                                                                       \
    (v)[0] = (u)[1]*(w)[2] - (u)[2]*(w)[1];                             \
    (v)[1] = (u)[2]*(w)[0] - (u)[0]*(w)[2];                             \
    (v)[2] = (u)[0]*(w)[1] - (u)[1]*(w)[0];                             \
}

#define DOTVP(u,v) ((u)[0]*(v)[0] + (u)[1]*(v)[1] + (u)[2]*(v)[2] )
#define MAX(a,b) (((a)>(b))?(a):(b))

#define ABSV(v) sqrt((v)[0]*(v)[0]+(v)[1]*(v)[1]+(v)[2]*(v)[2])
#define NORMALIZE(v) {double b=ABSV(v);(v)[0]/=b;(v)[1]/=b;(v)[2]/=b;}
/* Table of constant values */

static integer c__1 = 1;
static int i;

/* MAIN */

/*     THIS IS A MAIN PROGRAM FOR LEAST SQUARES PROBLEMS */
/*     WITH INEQUALITY CONSTRAINTS. */
/*     THE SOLVER IS ENLSIP WRITTEN BY PER LINDSTROM, UNIVERSITY */
/*     OF UMEA, SWEDEN */

/*     NAME OF USER WRITTEN SUBROUTINES ARE FEX65 AND HEX65 */


static double params[7] = {0,0,1,150,110,120,100};
static double* pData = NULL;
static int dataCount = 1000;

 int hexTest_(doublereal *x, integer *n, doublereal *h__, 
	integer *l, integer *ctrl, doublereal *a, integer *mda);
	
int fexTest_(doublereal *x, integer *n, doublereal *f, integer 
	*m, integer *ctrl, doublereal *c__, integer *mdc);	
 
 extern "C" int enlsip_(doublereal *, integer *, integer *, 
	    integer *, integer *, integer *, integer *, U_fp, U_fp, integer *,
	     doublereal *, integer *, doublereal *, doublereal *, integer *);


/* Main program */ int Main_Test(void)
{
	double c_;

    /* System generated locals */
    integer i__1;

#if 0	
    /* Builtin functions */
    integer s_wsfe(cilist *), e_wsfe(void), do_fio(integer *, char *, ftnlen);
    /* Subroutine */ int s_stop(char *, ftnlen);
#endif	

    /* Local variables */
   
    integer exit, *wint, nout;
    doublereal *f, *h__;
    integer i__, l, m, n, p;
    doublereal *x, *wreal;
    extern /* Subroutine */ int printTest_(integer *, integer *, integer *, 
	    integer *, doublereal *, doublereal *, integer *, integer *, 
	    integer *, integer *, doublereal *);
    integer *active;
    integer mdi, mdr, bnd;
	

    printf("Starting Test....\n");
	
	
/*	WINT() INTEGER SINGLY SUBSCRIPTED ARRAY OF DIMENSION MDI */
/*            (MDI MUST BE >=(2*(L+BND)+N+13)) */
/*     WREAL() REAL SINGLY SUBSCRIPTED ARRAY OF DIMENSION MDR */
/*          (MDR MUST BE >=(N*(M+2*N+L+6)+3*(M+2*L)+4*BND+7) */
/*                    WHEN INEQUALITIES ARE PRESENT IN THE PROBLEM) */
/*          (MDR MUST BE >=(N*(M+N+L+6)+3*(M+2*L)+4*BND+7) */
/*                    WHEN THERE ARE ONLY EQUALITIES IN THE PROBLEM) */
	
   // nout = 10;
    n = 3;
    m = dataCount;
    p = 0;
    l = 0;
	bnd = l+n;
	mdi = (2*(l+bnd)+n+13);
	mdr = MAX((n*(m+2*n+l+6)+3*(m+2*l)+4*bnd+7),mdr = (n*(m+n+l+6)+3*(m+2*l)+4*bnd+7));
		
	wint = (integer*)malloc(sizeof(integer)*mdi);
	wreal = (doublereal*)malloc(sizeof(doublereal)*mdr);
	x = (doublereal*)malloc(sizeof(doublereal)*n);
	f = (doublereal*)malloc(sizeof(doublereal)*(m+l));
	h__ = (doublereal*)malloc(sizeof(doublereal)*l);
	active = (integer*)malloc(sizeof(integer)*(bnd+l));

/*     SET THE FIRST ELEMENTS OF THE ARRAYS WINT AND WREAL TO */
/*     SOMETHING NEGATIVE TO INDICATE THAT DEFAULT VALUES SHOULD */
/*     BE USED */

    for (i__ = 1; i__ <= 6; ++i__) {
	wint[i__ - 1] = -1;
/* L10: */
    }
	//number of iterations.
	wint[2] = 10000;
	
	//For internal data scaling
	wint[4] = 1;
	
    for (i__ = 1; i__ <= 15; ++i__) {
	wreal[i__ - 1] = -1.f;
/* L20: */
    }
    
	for(i=0;i<3;i++)
		x[i] = 0.3;
	


/*     INVOKE ENLSIP TO MINIMIZE */
	for(i=0;i<n;i++)
		printf("Initial x[%d]: %f\n",i,x[i]);

    enlsip_(x, &n, &m, &p, &l, &mdi, &mdr, (U_fp)fexTest_, (U_fp)hexTest_, wint, 
	    wreal, &exit, f, h__, active);
		
	for(i=0;i<n;i++)
		printf("Final x[%d]: %f\n",i,x[i]);
	
	free(wint);
	free(wreal);
	free(x);
	free(f);
	free(h__);
	free(active);
	
	printf("Finished Cylinder....\n");
    return 0;
} /* MAIN__ */

/* PRINT */
/* Subroutine */ int printTest_(integer *nout, integer *exit, integer *active, 
	integer *p, doublereal *h__, doublereal *x, integer *n, integer *l, 
	integer *m, integer *wint, doublereal *wreal)
{
    return 0;
} /* printTest_ */

/* FEX65 */
/* Subroutine */ int fexTest_(doublereal *x, integer *n, doublereal *f, integer 
	*m, integer *ctrl, doublereal *c__, integer *mdc)
{
  	static int callCounter = 0;
	double sumSq = 0;
	int i = 0;
	int M = (*m);
	printf("fexTest_ %d\n",callCounter++);
    /* Parameter adjustments */
    	
    /* Function Body */
    if (abs(*ctrl) != 1) {
		*ctrl = 0;	return 0;
	}
	

/*     EVALUATE THE RESIDUALS */

	for(i=0;i<M;i++)
	{
		f[i] = x[0]*pow((double)i,2) + x[1]*i + x[2]*pow(i,2.5) - 
			(2*pow((double)i,2) + 30*i + 6*pow(i,2.5));
		sumSq += pow(f[i],2);
		//printf("%d: %f\n",i,f[i]);
	}
	printf("Sumsq: %f \n",sumSq);

    return 0;
} /* fexTest_ */

/* HEX65 */
/* Subroutine */ int hexTest_(doublereal *x, integer *n, doublereal *h__, 
	integer *l, integer *ctrl, doublereal *a, integer *mda)
{
    int i=0;
	/* Function Body */
    if (abs(*ctrl) != 1) {
		*ctrl = 0; return 0;
    }

/*     EVALUATE THE CONSTRAINTS */
	//h__[0] = x[2] - 6;

    for(i=0;i<*l;i++)
		printf("h[%d]: %f    ",i,h__[i]);
	printf("\n");
	
    return 0;
} /* hexTest_ */

