
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



/* Var_y.c :
 * INPUT:
 *    pntl, pntr : pointers naar het gevonden overeenkomstige punt in de
 *                 meetfile van de linker en rechter opname.
 *    by, bz     : benaderde y en z positie van het rechterhoofdpunt (bx=1).
 *    q1, q2, q3 : benaderde waarden voor de quaternionen q1, q2 en q3
 * BEREKENING:
 *    Met behulp van Maple is de coplanariteitsvoorwaarde uitgewerkt 
 *    tot functie F (zie de aanroepende functie Coplan_indir() ).
 *    Vervolgens is functie F partieel gedifferentieerd naar xl, yl, xr en yr,
 *    met de Maple commando's:
 *    > diff(F,xl);   > diff(F,yl);  > diff(F,xr);  > diff(F,yr);.
 *    Met Maple is ook de formule uitgewerkt voor de varianties van de
 *    waarnemingsvector y, deze varianties vormen een diagonaalmatrix.
 *    De gebruikte Maple statements om de variantie van y in een formule uit 
 *    te drukken zijn:
 *       > dF := vector([dFdxl, dFdyl, dFdxr, dFdyr]);
 *       > var := matrix([[vxl,vxlyl,0,0],[vxlyl,vyl,0,0],[0,0,vxr,vxryr],
 *                        [0,0,vxryr,vyr]]);
 *       > vary := transpose(dF) &* var &* dF;
 *       > evalm(vary);
 *    Dit levert de formule om per waarneming y de variantie te berekenen:
 *    vary =  dFdxl dFdxl vxl + 2 dFdxl dFdyl vxlyl + dFdyl dFdyl vyl 
 *          + dFdxr dFdxr vxr + 2 dFdxr dFdyr vxryr + dFdyr dFdyr vyr
 * OUTPUT:
 *    vary, de variantie vav de wng y die hoort bij het gevonden puntenpaar.
 lgens is functie F partieel gedifferentieerd naar xl, yl, xr en yr,
 */

#include <stdio.h>
#include "Database.h"  /* bevat o.a. def. van de structures ImgPts en Pixel  */

 double Var_y(CamPt pntl, CamPt pntr, double by, double bz, double q1,
              double q2, double q3, double f)
 /* de retourwaarde van Var_y() is de berekende variantie */

{
    double dFdxl, dFdyl, dFdxr, dFdyr; /* differenties */
    double xl, yl, xr, yr;
    double vxl, vxlyl, vyl, vxr, vxryr, vyr;  /* varianties en covarianties */
    double vary;                              /* retourwaarde van Var_y() */

    xl= pntl.x;  yl= pntl.y;  vxl= pntl.v_x;  vyl= pntl.v_y;  vxlyl= pntl.cv_xy;
    xr= pntr.x;  yr= pntr.y;  vxr= pntr.v_x;  vyr= pntr.v_y;  vxryr= pntr.cv_xy;

    /*   Berekening van de partiele afgeleiden dFdxl, dFdyl, dFdxr, dFdyr:  */

    dFdxl =
    -1.0 * (yr*bz + f*by + 2.0*xr*bz*q1*q2 + 2.0*xr*bz*q3 - 2.0*xr*by*q1*q3 
    + 2.0*xr*by*q2 - yr*bz*q1*q1  + yr*bz*q2*q2 - yr*bz*q3*q3 - 2.0*yr*by*q2*q3
    - 2.0*yr*by*q1 - 2.0*f*bz*q2*q3 + 2*f*bz*q1 - f*by*q1*q1 - f*by*q2*q2
    + f*by*q3*q3)
     /  (1.0 + q1*q1 + q2*q2 + q3*q3);

    dFdyl =
    -1.0* (-1.0*xr*bz - 2.0*xr*q2 + 2.0*yr*q1 + f*q1*q1 + f*q2*q2 - f*q3*q3
    - xr*bz*q1*q1 + xr*bz*q2*q2 + xr*bz*q3*q3 + 2.0*xr*q1*q3 - f
    - 2.0*yr*bz*q1*q2 + 2.0*yr*bz*q3 + 2.0*yr*q2*q3 + 2.0*f*bz*q1*q3
    + 2.0*f*bz*q2)
     /  (1.0 + q1*q1 + q2*q2 + q3*q3);

    dFdxr =
    -1.0* (2.0*f*q3 - f*by - yl*bz - 2.0*yl*q2 - yl*bz*q1*q1 + yl*bz*q2*q2
    + yl*bz*q3*q3 - f*by*q1*q1 + f*by*q2*q2 + f*by*q3*q3 + 2.0*xl*bz*q1*q2
    + 2.0*xl*bz*q3 + 2.0*f*q1*q2 - 2.0*xl*by*q1*q3 + 2.0*xl*by*q2
    + 2.0*yl*q1*q3)
     /  (1.0 + q1*q1 + q2*q2 + q3*q3);
 

    dFdyr =
    -1.0* ( -1.0*f*q3*q3 - f*q1*q1 + xl*bz + f*q2*q2 + 2.0*yl*q1 + f
    - 2.0*yl*bz*q1*q2 + 2.0*yl*bz*q3 - 2.0*f*by*q1*q2 + 2.0*f*by*q3
    - xl*bz*q1*q1 + xl*bz*q2*q2 - xl*bz*q3*q3 - 2.0*xl*by*q2*q3
    - 2.0*xl*by*q1 + 2.0*yl*q2*q3)
     /  (1.0 + q1*q1 + q2*q2 + q3*q3);

    if (!vxl) vxl = 0.00001;
    vary =  dFdxl*dFdxl*vxl + 2.0*dFdxl*dFdyl*vxlyl + dFdyl*dFdyl*vyl
           + dFdxr*dFdxr*vxr + 2.0*dFdxr*dFdyr*vxryr + dFdyr*dFdyr*vyr;
    
    return vary;
}
