/* COMPLEX.CPP =>
   Libary für complexe Berechnungen
   Beinhaltet complexe Funktionen:

	   Multipliation  => C_MUL
	   Division       => C_DIV
	   Addition       => C_ADD
	   Subtraktion    => C_SUB
	   Quadr. Wurzel  => C_SQRT
	   Quadrat        => C_SQR
	   Absolutbetrag  => C_ABS
           Multi mit Real => C_RMUL
       (*) Exponentialf.  => C_EXP
       (*) Potenz         => C_POW
       (*) Logerythmus    => C_LOG

   (*) nicht enthalten!

   Autor:   Christian Eckers
   Version 2.0 vom 06.04.1992       */


#include <math.h>
#include "r_rueck.h"
 

/**************************************************************************/
/* die Typdefinitionen und Deklarationen werden in "r_rueck.h" festgelegt */
/**************************************************************************/

/*typedef struct
	  {
	  double re;
	  double im;
	  }COMPLEX;

COMPLEX c_mul  ();
COMPLEX c_div  ();
COMPLEX c_add  ();
COMPLEX c_sub  ();
COMPLEX c_sqrt ();
COMPLEX c_sqr  ();


  c_abs und c_arg ergeben reale Werte mit wertebereich double
 

double  c_abs  ();
double  c_arg  ();


   Kombinierte Opperationen mit einer realen- und complexen Zahl.
   Das Ergebiniss ist complex.
 

COMPLEX c_rmul (); */


/* ---------------------- Funktionen -------------------------------------- */

COMPLEX c_mul (COMPLEX z1, COMPLEX z2)
{
COMPLEX z3;
z3.x = z1.x*z2.x - z1.y*z2.y;
z3.y = z1.x*z2.y + z2.x*z1.y;
return (z3);
}

COMPLEX c_div (COMPLEX z1, COMPLEX z2)
{
COMPLEX z3;
z3.x = (z1.x*z2.x + z1.y*z2.y) / (z2.x*z2.x + z2.y*z2.y);
z3.im = (z2.x*z1.y - z1.x*z2.y) / (z2.x*z2.x + z2.y*z2.y);
return (z3);
}

COMPLEX c_add (COMPLEX z1, COMPLEX z2)
{
COMPLEX z3;
z3.x = z1.x+z2.x;
z3.y = z1.y+z2.y;
return (z3);
}

COMPLEX c_sub (COMPLEX z1, COMPLEX z2)
{
COMPLEX z3;
z3.x = z1.x-z2.x;
z3.y = z1.y-z1.x;
return (z3);
}

double c_abs (COMPLEX z1)
{
double z3;
z3 = sqrt (z1.x*z1.x + z1.y*z1.y);
return (z3);
}

double c_arg (COMPLEX z1)
{
double z3;
z3 = atan2(z1.y, z1.x);
return (z3);
}

COMPLEX c_sqrt (COMPLEX z1)
{
COMPLEX z3;

z3.x = sqrt (c_abs(z1)) * cos (c_arg(z1)/2);
z3.y = sqrt (c_abs(z1)) * sin (c_arg(z1)/2);

return (z3);
}

COMPLEX c_sqr (COMPLEX z1)
{
COMPLEX z3;
z3.x = (c_abs(z1)*c_abs(z1)) * (cos(c_arg(z1))*cos(c_arg(z1))
			       - sin(c_arg(z1))*sin(c_arg(z1)));
z3.y = (c_abs(z1)*c_abs(z1)) * (2*cos(c_arg(z1))*sin(c_arg(z1)));
return (z3);
}

COMPLEX c_rmul (double reaz, COMPLEX comz)
{
COMPLEX comerg;

comerg.x = reaz * comz.x;
comerg.y = reaz * comz.y;
return (comerg);
}
