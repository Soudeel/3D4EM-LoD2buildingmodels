
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class ExteriorOrientation          

 void ExteriorOrientation::Cpp2C(CamPts **)      Conversion of C++ to C object
 void ExteriorOrientation::C2Cpp(CamPts *)       Conversion of C to C++ object
 int  ExteriorOrientation::Read(char *)          Read Exterior orientation from a database
 int  ExteriorOrientation::Write(char *)         Write Exterior orientation to a database
 void ExteriorOrientation::Print()               Print Exterior orientation to stdout

 Initial creation
 Author : Ildiko Suveg
 Date   : 04-12-1998

 Update #1
 Author : 
 Date   : 
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "ExteriorOrientation.h"

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ExteriorOrientation::Cpp2C(Exterior **extorptr) const 
{
  Exterior *extor;

/* Allocate space if this has not been done yet */

  extor = *extorptr;
  if (extor == NULL) {
    extor = (Exterior *) malloc(sizeof(Exterior));
    *extorptr = extor;
  }

/* Copy the data */

   extor->x = x[0]; 
   extor->y = x[1]; 
   extor->z = x[2];
   for (int j = 0; j < W_DIM; j++)
	for (int i = 0; i < W_DIM; i++)	
	   extor->rot[j][i] = r[j][i];
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ExteriorOrientation::C2Cpp(Exterior *extor)
{
   x[0] = extor->x; 
   x[1] = extor->y; 
   x[2] = extor->z;
   for (int j = 0; j < W_DIM; j++)
    for (int i = 0; i < W_DIM; i++)	
    	r[j][i] = extor->rot[j][i];

}

/*
--------------------------------------------------------------------------------
                       Read the Exterior orientation from a database
--------------------------------------------------------------------------------
*/
int ExteriorOrientation::Read(const char *filename)
{
  Exterior *extor;
  int err; 	
  
  extor = Get_Exterior(filename, &err); /* Read the database into C structure */
  if (extor == NULL) return(0);
  C2Cpp(extor);                        /* Convert to C++ object */
  free(extor);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write the Exterior orientation to a database
--------------------------------------------------------------------------------
*/
int ExteriorOrientation::Write(const char *filename) const
{
  int    error;
  Exterior *extor;

  extor = NULL;
  Cpp2C(&extor);
  error = Put_Exterior(extor, filename);
  free(extor);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print Exterior orientation to stdout
--------------------------------------------------------------------------------
*/
void ExteriorOrientation::Print() const
{
  Exterior *extor;

  extor = NULL;
  Cpp2C(&extor);
  Print_Exterior(extor);
  free(extor);
}


/*
--------------------------------------------------------------------------------
                        direct spatial resection
--------------------------------------------------------------------------------
*/

int ExteriorOrientation::Direct(int num_pts, int *lijstpnr, double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix, double cc)
{
    
    Exterior *extor;
    
    int a, b,c, d;
    
    extor = (Exterior *) malloc(sizeof(Exterior));
    
    Search_Start_Points(num_pts, lijstpnr, lijstxpix, lijstypix, 
                         lijstxpas, lijstypas, lijstzpas,
                         &a, &b, &c, &d);
    Exterior_Direct(lijstxpas, lijstypas, lijstzpas, lijstxpix, lijstypix, 
                     a, b, c, d, 3.0, cc, extor);
   
    if (extor == NULL) return(0);
    
   C2Cpp(extor);                        /* Convert to C++ object */
   free(extor);
   return 1;
}

int ExteriorOrientation::InDirect(int num_pts, int *lijstpnr, double *lijstxpas, double *lijstypas, 
                    double *lijstzpas, double *lijstxpix, double *lijstypix, 
		            double *lijstxpixvar, double *lijstypixvar,double *lijstxycovar, double cc)
{
    
    Exterior *extor;
    
    extor = (Exterior *) malloc(sizeof(Exterior));
    
   Exterior_Indirect(num_pts, lijstpnr, 
                     lijstxpix, lijstypix, 
                     lijstxpixvar, lijstypixvar, lijstxycovar, 
                     lijstxpas, lijstypas, lijstzpas, 
                     1, cc, extor);
   
    if (extor == NULL) return(0);
    
   C2Cpp(extor);                        /* Convert to C++ object */
   free(extor);
   return 1;
}






