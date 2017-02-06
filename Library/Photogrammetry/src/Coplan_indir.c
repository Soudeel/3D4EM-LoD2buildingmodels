
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



/* Coplan-indir.c :
*  INPUT:
*     De benaderde waarden van de positie van de rechter opname t.o.v.
*     de linker.
*     (Van de linker opname zijn de rotaties nul
*     en bevindt het hoofdpunt zich in de oorsprong) 
*     De benaderde positie van het rechterhoofdpunt is by en bz (bx=1),
*     deze positie is berekend met de routine Mod_Basis.c
*     De benaderde rotaties van de rechter opname zijn gegeven door middel 
*     van de quaternionen q1, q2 en q3, berekend met de
*     routine Approx_Quaternion 
*  BEREKENING:
*     Leest de fotocoordinaten uit 2 files met imgpts (filename: *.imgpts).
*     Zoekt de gemeenschappelijke punten op.
*     Met behulp van Maple is de coplanariteitsvoorwaarde uitgewerkt
*     met de statements:
*         > with(linalg);
*         > v1:=vector([ xl , yl ,- f ]);
*         > v2:=vector([ xr , yr ,- f ]);
*         > sb := matrix([[ 0, -bz, bj],[ bz, 0, -1],[ -bj, 1, 0 ]]);
*         > c := 1/( 1 + q1^2  + q2^2  +  q3^2 );
*         > Q := matrix([[1+q1^2-q2^2-q3^2, 2*(q1*q2-q3), 2*(q1*q3+q2)],
*                        [2*(q2*q1+q3), 1-q1^2+q2^2-q3^2, 2*(q2*q3-q1)],
*                        [2*(q3*q1-q2), 2*(q3*q2+q1), 1-q1^2-q2^2+q3^2]]);
*         > r2 := c*Q;
*         > G := transpose(v1) &* sb &* r2 &* v2;.
*         > F := evalm(G);
*     Hierna is de functie F partieel gedifferentieerd. naar by, bz, q1, q2 en
*       q3 met:
*         > diff(F,bz);  enz
*     Berekent de coefficienten van de termen dby, dbz, dq1, dq2 en dq3 in 
*     deze lineaire vergelijking. 
*     Per gevonden puntenpaar worden de berekende coefficienten geplaatst in 
*     coefficientenmatrix a.
*     Er geldt de betrekking a.x=y, 
*     met in de vector x[5] de onbekenden dby, dbz, dq1, dq2 en dq3.
*     Voor de waarnemingsvector y geldt f( xl , yl , xr , yr ,by0,bz0,q10,q20,q30)=-y
*     waarin by0, bz0, q10, q20 en q310 benaderde waarden zijn.
*     Om de varianties van de waarnemingsvector y te bepalen wordt gebruik 
*     gemaakt van de routine Var_y( xl ,  yl ,  xr ,  yr , by, bz, q1, q2, q3,  f ).
*  OUTPUT:
*     a, de coefficientenmatrix
*     y, de waarnemingsvector 
*     vary, de vector met varianties van y 
*/

#include <stdio.h>
#include "Database.h"  /* met o.a. de f inities van de struct. ImgPts en Pixel */

void Coplan_indir(CamPts *L, CamPts *R, double by, double bz,
                  double q1, double q2, double q3, double f , double *a,
                  double *y, double *vary, int *numbers, int num_pts)
{
  double  xl ,  yl ,  xr ,  yr , noemer;
  double *ptr_a, *ptr_y, *ptr_vary ;
  int i, j, k, l;
  
  double Var_y(CamPt, CamPt, double, double, double, double, double, double);
  
  /* ptr_a enz. is een hulpvariabele die opgehoogd wordt, 
     het beginadres a blij f t op deze manier bewaard */
  ptr_a = a;  ptr_y = y;  ptr_vary = vary; 

  noemer = (1.0 +  q1 * q1  +  q2 * q2  +  q3 * q3 );
  /* Opzoeken van overeenkomstige gemeten punten in beide  f oto's L en R */
  for( i=0 ; i< L->num_pts ; i++ )  /* gemeten punten in Linker  f oto  */
     {
	for( j=0 ; j< R->num_pts ; j++ )  /* gemeten punten in Rechter  f oto  */
	{
	if ( L->pts[i].num == R->pts[j].num )
	   {
	   /* gemeenschappelijk punt gevonden in linker en rechter  f oto  */
	      xl  = L->pts[i].x;    yl  = L->pts[i].y;
	      xr  = R->pts[j].x;    yr  = R->pts[j].y;


           /* Berekening van de coefficient van dby  : */

           *ptr_a++ =
	   -1.0 * (-  xr * f  + 2.0 *  f * f * q2  +  f * xl * q3 * q3  -  f * xl * q1 * q1 
	   +  f * xl  -  xr * f * q1 * q1  +  xr * f * q2 * q2  +  xr * f * q3 * q3  
	   - 2.0 *  xr * xl * q1 * q3  
	   
	   + 2.0 *  xr * xl * q2  - 2.0 *  yr * f * q1 * q2  + 2.0 *  yr * f * q3  
	   - 2.0 *  yr * xl * q2 * q3  - 2.0 *  yr * xl * q1  + 2.0 *  f * f * q1 * q3  
	   -  f * xl * q2 * q2  )  / noemer; 

           /* Berekening van de coefficient van dbz  : */

           *ptr_a++ = 
	   -1.0 * (-  xr * yl  +  yr * xl  +  xr * yl * q2 * q2  
	   +  xr * yl * q3 * q3  + 2.0 *  f * xl * q1  -  xr * yl * q1 * q1  
	   + 2.0 *  xr * xl * q1 * q2  + 2.0 *  xr * xl * q3  - 2.0 *  yr * yl * q1 * q2  
	   
	   + 2.0 *  yr * yl * q3  -  yr * xl * q1 * q1  +  yr * xl * q2 * q2  
	   -  yr * xl * q3 * q3  + 2.0 *  f * yl * q1 * q3  + 2.0 *  f * yl * q2  
           - 2.0 *  f * xl * q2 * q3 )  / noemer;


           /* Berekening van de coefficient van dq1  : */

           *ptr_a++ = 
	   -1.0 * (- 2.0 *  yr * f * q1  + 2.0 *  yr * yl  + 2.0 *  f * yl * q1  
	   + 2.0 *  f * f  - 2.0 *  xr * yl * bz * q1  - 2.0 *  xr * f * by * q1  
	   + 2.0 *  xr * xl * bz * q2  + 2.0 *  xr * f * q2  - 2.0 *  xr * xl * by * q3  
	   
           + 2.0 *  xr * yl * q3  - 2.0 *  yr * yl * bz * q2  - 2.0 *  yr * f * by * q2  
           - 2.0 *  yr * xl * bz * q1  - 2.0 *  yr * xl * by  + 2.0 *  f * f * by * q3  
           + 2.0 *  f * yl * bz * q3  + 2.0 *  f * xl * bz  
	   
	   - 2.0 *  f * xl * by * q1 ) / noemer

           + 2.0 * ( yr * xl * bz  -  f * yl * q3 * q3  +  f * yl * q2 * q2  
	   -  yr * f * q1 * q1  + 2.0 *  yr * yl * q1  +  xr * yl * bz * q3 * q3  
	   +  f * yl * q1 * q1  -  xr * yl * bz  - 2.0 *  xr * yl * q2 

           + 2.0 *  xr * f * q3  +  yr * f  + 2.0 *  f * f * q1  -  f * yl  -  xr * f * by  
	   -  xr * yl * bz * q1 * q1  +  xr * yl * bz * q2 * q2  
	   -  xr * f * by * q1 * q1  +  xr * f * by * q2 * q2  
	   
	   +  xr * f * by * q3 * q3  + 2.0 *  xr * xl * bz * q1 * q2  
	   + 2.0 *  xr * xl * bz * q3  + 2.0 *  xr * f * q1 * q2  
	   - 2.0 *  xr * xl * by * q1 * q3  + 2.0 *  xr * xl * by * q2  
	   + 2.0 *  xr * yl * q1 * q3  
	   
	   - 2.0 *  yr * yl * bz * q1 * q2  + 2.0 *  yr * yl * bz * q3  
	   - 2.0 *  yr * f * by * q1 * q2  + 2.0 *  yr * f * by * q3  
	   -  yr * xl * bz * q1 * q1  +  yr * xl * bz * q2 * q2  
	   -  yr * xl * bz * q3 * q3  
	   
	   +  yr * f * q2 * q2  -  yr * f * q3 * q3  + 2.0 *  f * f * by * q2  
	   - 2.0 *  f * f * q2 * q3  +  f * xl * by  - 2.0 *  yr * xl * by * q2 * q3  
	   - 2.0 *  yr * xl * by * q1  + 2.0 *  yr * yl * q2 * q3  
	   
	   + 2.0 *  f * f * by * q1 * q3  + 2.0 *  f * yl * bz * q1 * q3  
	   + 2.0 *  f * yl * bz * q2  - 2.0 *  f * xl * bz * q2 * q3  
	   + 2.0 *  f * xl * bz * q1  -  f * xl * by * q1 * q1  
	   -  f * xl * by * q2 * q2   
	   
	   +  f * xl * by * q3 * q3  )* q1  / (noemer*noemer);


           /* Berekening van de coefficient van dq2  : */

           *ptr_a++ = 
	   -1.0 * (2.0 * f * yl * q2  - 2.0 *  xr * yl  + 2.0 *  xr * yl * bz * q2  
	   + 2.0 *  xr * f * by * q2  + 2.0 *  xr * xl * bz * q1  + 2.0 *  xr * f * q1  
	   + 2.0 *  xr * xl * by  - 2.0 *  yr * yl * bz * q1  
	   
	   - 2.0 *  yr * f * by * q1  + 2.0 *  yr * xl * bz * q2  + 2.0 *  yr * f * q2  
	   + 2.0 *  f * f * by  - 2.0 *  f * f * q3  - 2.0 *  yr * xl * by * q3  
	   + 2.0 *  yr * yl * q3  + 2.0 *  f * yl * bz  - 2.0 *  f * xl * bz * q3  
	   
	   - 2.0 *  f * xl * by * q2 ) / noemer

	   + 2.0 * (  yr * xl * bz  -  f * yl * q3 * q3  +  f * yl * q2 * q2  
	   -  yr * f * q1 * q1  + 2.0 *  yr * yl * q1  +  xr * yl * bz * q3 * q3  
	   +  f * yl * q1 * q1  -  xr * yl * bz  - 2.0 *  xr * yl * q2  

	   + 2.0 *  xr * f * q3  +  yr * f  + 2.0 *  f * f * q1  -  f * yl  -  xr * f * by  
	   -  xr * yl * bz * q1 * q1   +  xr * yl * bz * q2 * q2   
	   -  xr * f * by * q1 * q1   +  xr * f * by * q2 * q2   
	   
	   +  xr * f * by * q3 * q3   + 2.0 *  xr * xl * bz * q1 * q2  
	   + 2.0 *  xr * xl * bz * q3  + 2.0 *  xr * f * q1 * q2  
           - 2.0 *  xr * xl * by * q1 * q3  + 2.0 *  xr * xl * by * q2  
           + 2.0 *  xr * yl * q1 * q3  
	   
	   - 2.0 *  yr * yl * bz * q1 * q2  + 2.0 *  yr * yl * bz * q3  
	   - 2.0 *  yr * f * by * q1 * q2  + 2.0 *  yr * f * by * q3  
	   -  yr * xl * bz * q1 * q1  +  yr * xl * bz * q2 * q2  
	   -  yr * xl * bz * q3 * q3  
	   
	   +  yr * f * q2 * q2  -  yr * f * q3 * q3  + 2.0 *  f * f * by * q2  
	   - 2.0 *  f * f * q2 * q3  +  f * xl * by  - 2.0 *  yr * xl * by * q2 * q3  
	   - 2.0 *  yr * xl * by * q1  + 2.0 *  yr * yl * q2 * q3  
	   
	   + 2.0 *  f * f * by * q1 * q3  + 2.0 *  f * yl * bz * q1 * q3  
	   + 2.0 *  f * yl * bz * q2  - 2.0 *  f * xl * bz * q2 * q3  
	   + 2.0 *  f * xl * bz * q1  -  f * xl * by * q1 * q1  
	   -  f * xl * by * q2 * q2   
	   
	   +  f * xl * by * q3 * q3  )* q2  / (noemer*noemer);



           /* Berekening van de coefficient van dq3  : */

           *ptr_a++ = 
	   -1.0 * (- 2.0 *  f * yl * q3  + 2.0 *  xr * yl * bz * q3  + 2.0 *  xr * f  
	   + 2.0 *  xr * f * by * q3  + 2.0 *  xr * xl * bz  - 2.0 *  xr * xl * by * q1  
	   + 2.0 *  xr * yl * q1  + 2.0 *  yr * yl * bz  
	   
	   + 2.0 *  yr * f * by  - 2.0 *  yr * xl * bz * q3  - 2.0 *  yr * f * q3  
	   - 2.0 *  f * f * q2  - 2.0 *  yr * xl * by * q2  + 2.0 *  yr * yl * q2  
	   + 2.0 *  f * f * by * q1  + 2.0 *  f * yl * bz * q1  
	   
	   - 2.0 *  f * xl * bz * q2  + 2.0 *  f * xl * by * q3 ) / noemer

           + 2.0 * ( yr * xl * bz  -  f * yl * q3 * q3  +  f * yl * q2 * q2  
	   -  yr * f * q1 * q1  + 2.0 *  yr * yl * q1  +  xr * yl * bz * q3 * q3  
	   +  f * yl * q1 * q1  -  xr * yl * bz  - 2.0 *  xr * yl * q2  
	   
	   + 2.0 *  xr * f * q3  +  yr * f  + 2.0 *  f * f * q1  -  f * yl  -  xr * f * by  
	   -  xr * yl * bz * q1 * q1  +  xr * yl * bz * q2 * q2  
	   -  xr * f * by * q1 * q1  +  xr * f * by * q2 * q2  
	   
	   +  xr * f * by * q3 * q3  + 2.0 *  xr * xl * bz * q1 * q2  
	   + 2.0 *  xr * xl * bz * q3  + 2.0 *  xr * f * q1 * q2  
	   - 2.0 *  xr * xl * by * q1 * q3  + 2.0 *  xr * xl * by * q2  
	   + 2.0 *  xr * yl * q1 * q3  
	   
	   - 2.0 *  yr * yl * bz * q1 * q2  + 2.0 *  yr * yl * bz * q3  
	   - 2.0 *  yr * f * by * q1 * q2  + 2.0 *  yr * f * by * q3  
	   -  yr * xl * bz * q1 * q1  +  yr * xl * bz * q2 * q2  
	   -  yr * xl * bz * q3 * q3  
	   
	   +  yr * f * q2 * q2  -  yr * f * q3 * q3  + 2.0 *  f * f * by * q2  
	   - 2.0 *  f * f * q2 * q3  +  f * xl * by  - 2.0 *  yr * xl * by * q2 * q3  
	   - 2.0 *  yr * xl * by * q1  + 2.0 *  yr * yl * q2 * q3  
	   
	   + 2.0 *  f * f * by * q1 * q3  + 2.0 *  f * yl * bz * q1 * q3  
	   + 2.0 *  f * yl * bz * q2  - 2.0 *  f * xl * bz * q2 * q3  
	   + 2.0 *  f * xl * bz * q1  -  f * xl * by * q1 * q1  
	   -  f * xl * by * q2 * q2   
	   
	   +  f * xl * by * q3 * q3  )* q3  / (noemer*noemer);

           /* Zoals in de header van deze  f ile is beschreven,
	   *  is de coplanariteitsvoorwaarde met Maple uitgewerkt om de 
	   *  waarnemingsvector y te berekenen.
	   *  De waarneming is y = transpose(v1) &* sb &* r2 &* v2 
	   *  waarbij voor de termen by, bz, q1, q2 en q3 de berekende 
	   *  benaderde waarden gebruikt worden.
	   *  Per gevonden puntenpaar wordt de coplanariteitsvoorwaarde 
	   *  berekend en in de waarnemingsvector y geplaatst.
           */
	       *ptr_y++ = 
	       -1.0*(    xr * f * by  +  yr * f * q3 * q3   - 2.0 *  xr * f * q3  
	       +  yr * f * q1 * q1   +  xr * yl * bz  + 2.0 *  xr * yl * q2  
	       -  yr * xl * bz  -  yr * f * q2 * q2   - 2.0 *  yr * yl * q1  
	       - 2.0 *  f * f * by * q2  + 2.0 *  f * f * q2 * q3  -  f * xl * by 
	       -  f * yl * q1 * q1  -  f * yl * q2 * q2   +  f * yl * q3 * q3  
	       +  xr * yl * bz * q1 * q1   -  xr * yl * bz * q2 * q2  
	       -  xr * yl * bz * q3 * q3  +  xr * f * by * q1 * q1  
	       -  xr * f * by * q2 * q2   -  xr * f * by * q3 * q3  
	       - 2.0 *  xr * xl * bz * q1 * q2  - 2.0 *  xr * xl * bz * q3 
	       - 2.0 *  xr * f * q1 * q2  + 2.0 *  xr * xl * by * q1 * q3 
	       - 2.0 *  xr * xl * by * q2  - 2.0 *  xr * yl * q1 * q3  -  yr * f 
	       - 2.0 *  f * f * q1  +  f * yl  + 2.0 *  yr * yl * bz * q1 * q2 
	       - 2.0 *  yr * yl * bz * q3  + 2.0 *  yr * f * by * q1 * q2 
	       - 2.0 *  yr * f * by * q3  +  yr * xl * bz * q1 * q1  
	       -  yr * xl * bz * q2 * q2  +  yr * xl * bz * q3 * q3  
	       + 2.0 *  yr * xl * by * q2 * q3  + 2.0 *  yr * xl * by * q1 
	       - 2.0 *  yr * yl * q2 * q3  - 2.0 *  f * f * by * q1 * q3 
	       - 2.0 *  f * yl * bz * q1 * q3  - 2.0 *  f * yl * bz * q2 
	       + 2.0 *  f * xl * bz * q2 * q3  - 2.0 *  f * xl * bz * q1 
	       +  f * xl * by * q1 * q1   +  f * xl * by * q2 * q2  
	       -  f * xl * by * q3 * q3  )
	       / (1.0 +  q1 * q1   +  q2 * q2   +  q3 * q3  );

	     /* routine Var_y.c die de varianties van de vector y berekent:  */
	     *ptr_vary++ = Var_y(L->pts[i], R->pts[j], by, bz, q1, q2, q3,  f );
	     /* De routine Var_y retourneert een double,
	        zijnde de variantie van de betre f  f ende waarneming y  */

           } /* einde i f  loop:
	        berekening van matrix a en vector y 
		mits gelijk puntnummer in linker en rechter opname */
        } /* einde  f or loop:
	     systematisch doorlopen van alle puntnummers in rechter opname */
     } /* einde  f or loop:
          systematisch doorlopen van alle puntnummers in linker opname  */

}
