/* R_Rueck.H  Heddderfile fuer rrs_d 
   Version vom 15.04.92  Christian Eckers */

/* 
 * Typdefinitionen fuer Complex Libary
 * !!!! nach Test in eigenen Hedder !!!!
 */

typedef struct
          {
          double re, x;
          double im, y;
          }COMPLEX;

/*
 * Prototypen fuer Complex Libary
 * !!!! nach Test in eigenen Hedder (s.o.) !!!!
 */

/* 
 * Complexe Funktionen
 */

void complex ();

COMPLEX c_mul (COMPLEX, COMPLEX);
COMPLEX c_div (COMPLEX, COMPLEX);
COMPLEX c_add (COMPLEX, COMPLEX);
COMPLEX c_sub (COMPLEX, COMPLEX);
COMPLEX c_sqrt (COMPLEX);
COMPLEX c_sqr (COMPLEX);

double c_abs (COMPLEX);
double c_arg (COMPLEX);

/*
 * Funktionen mit einer realen und komplexen Zahl
 */

COMPLEX c_rmul (double, COMPLEX);
COMPLEX c_rdiv (double, COMPLEX);
COMPLEX c_radd (double, COMPLEX);
COMPLEX c_rsub (double, COMPLEX);


