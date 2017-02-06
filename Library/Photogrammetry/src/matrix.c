
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



/* Collection of matrix and vector routines
 *
 * matmult - matrix multiplication
 * mattrans - matrix transposition
 * outer_product - outer product
 */

/*
-----------------------------------------------------------------------------
 Multiply matrix A with B = Matrix C. Matrix C has to be allocated 
 as double matrix (size : row_A * col_B)
-----------------------------------------------------------------------------
*/
void matmult (double *a, double *b, double *c, int row_a, int col_a, int col_b)
{
  int i, j, k;
  double *p;
  
  for (i = 0; i < row_a; i++)
    for (j = 0; j < col_b; j++)
      {
      *(c + i * col_b + j) = 0.0;
      for (k = 0; k < col_a; k++)
        *(c + i * col_b + j) += *(a + i * col_a + k) * *(b + k * col_b + j);
      }
  }

/*
-----------------------------------------------------------------------------
-----------------------------------------------------------------------------
*/
void mattrans (double *A, int nrow_A, int ncol_A, double *B)
{
  int i,j;
  double *p1, *p2;

/*------ Transpose matrix A to B, Matrix B has to be allocated ------*/
  for (p2 = B, i = 0; i < ncol_A; i++)
    for (p1 = A + i, j = 0; j < nrow_A; j++, p1 += ncol_A)
      *p2++ = *p1;
  }

/*
-----------------------------------------------------------------------------
-----------------------------------------------------------------------------
*/
void outer_product(double *a, double *b, double *c)
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}
