
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
 Collection of functions for class Matrix

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include "Matrix3.h"
#include "Vector3D.h"
#include "Vector2D.h"

/*
--------------------------------------------------------------------------------
                                Initialization constructor
--------------------------------------------------------------------------------
*/

Matrix3::Matrix3(double a[3][3])
{
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
       m[i][j] = a[i][j]; 
}


Matrix3::Matrix3(double *a)
{
  int k = 0;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
       m[i][j] = a[k++]; 
}

/*
--------------------------------------------------------------------------------
                                Copy Assignment
--------------------------------------------------------------------------------
*/

Matrix3& Matrix3::operator=(const Matrix3& a)
{
  // Check for self assignment
  if (this == &a) return *this;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
       m[i][j] = a.m[i][j]; 
    	
   return *this;
}

/*
--------------------------------------------------------------------------------
                                Transpose 
--------------------------------------------------------------------------------
*/

Matrix3 Matrix3::Transpose()
{
  Matrix3 a;
 
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
       a.m[i][j] = m[j][i];

  return a;        
}


/*
--------------------------------------------------------------------------------
                                Determinant
--------------------------------------------------------------------------------
*/

double Matrix3::Det()
{
  double d = m[0][0] * m[1][1] * m[2][2] + m[1][0] * m[2][1] * m[0][2];
  d += m[2][0] * m[0][1] * m[1][2] - m[2][0] * m[1][1] * m[0][2];
  d -= m[0][0] * m[2][1] * m[1][2] + m[1][0] * m[0][1] * m[2][2];
       
  return d;        
}

/*
--------------------------------------------------------------------------------
                                  Inverse
--------------------------------------------------------------------------------
*/

Matrix3 Matrix3::Inverse()
{
  if ((m[0][0] == 0) || (m[1][1] == 0) || (m[2][2] == 0))
  {
     printf("One of the diagonal elements is 0\n");
  }   	

  double det = Det();  
//  Matrix inv(det,0,0,0,det,0,0,0,det);
  Matrix3 inv(1,0,0,0,1,0,0,0,1);
  inv.m[0][0] = inv.m[0][0] / m[0][0];
  
  double b[3][2];
  b[0][0] = m[0][1] / m[0][0];
  b[0][1] = m[0][2] / m[0][0];
  b[1][0] = m[1][1] - m[1][0] * b[0][0];
  b[1][1] = m[1][2] - m[1][0] * b[0][1];
  b[2][0] = m[2][1] - m[2][0] * b[0][0];
  b[2][1] = m[2][2] - m[2][0] * b[0][1];
    
  inv.m[1][0] = - m[1][0] * inv.m[0][0];
  inv.m[2][0] = - m[2][0] * inv.m[0][0];  
    
  inv.m[1][0] = inv.m[1][0] / b[1][0];
  inv.m[1][1] = inv.m[1][1] / b[1][0];
    
  double c[3];
  c[0] = b[0][1] - b[0][0] * b[1][1] / b[1][0];
  c[1] = b[1][1] / b[1][0];
  c[2] = b[2][1] - b[2][0] * b[1][1] / b[1][0];  

  inv.m[0][0] -=  b[0][0] * inv.m[1][0];
  inv.m[0][1] = - b[0][0] * inv.m[1][1];  
  inv.m[2][0] -=  b[2][0] * inv.m[1][0];
  inv.m[2][1] = - b[2][0] * inv.m[1][1];  
  
  inv.m[2][2] = inv.m[2][2] / c[2];
  
  for(int k = 0; k < 2; k++)
    for(int i = 0; i < 3; i++)
      inv.m[k][i] -= c[k] * inv.m[2][i];

  return inv;
}

/* M*v=0 where v=(x,y,z);   */
/* Return x1=x/z; y1=y/z;   */
Vector2D Matrix3::Nullspace()
{
  int i, j; 
  
  i = 1, j = 2;	 
  double d = m[i][0] * m[j][1] - m[i][1] * m[j][0];
  double x = ( - m[i][2] * m[j][1] + m[i][1] * m[j][2]) / d;
  double y = ( - m[i][0] * m[j][2] + m[i][2] * m[j][0]) / d;
  
  return Vector2D(x, y);   
}


void Matrix3::Print()
{
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
      printf("%ld  ", (long)(10000 * m[i][j]));
    printf("\n");
  }       
}


/*
--------------------------------------------------------------------------------
                             Multiply a matrix by a constant
--------------------------------------------------------------------------------
*/

Matrix3 operator*(double p, const Matrix3 &a) 
{
   Matrix3 c;
   
   for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      c.m[i][j] = p * a.m[i][j];
    
   return c;  
}


/*
--------------------------------------------------------------------------------
                             Multiply two matrices
--------------------------------------------------------------------------------
*/

Matrix3 operator*(const Matrix3 &a, const Matrix3 &b) 
{
   Matrix3 c;
   
   for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
    {
      c.m[i][j] = 0;
      for(int k = 0; k < 3; k++)
        c.m[i][j] += a.m[i][k] * b.m[k][j];
    }    
    
   return c;  
}

/*
--------------------------------------------------------------------------------
                             Multiply a matrix with a vector
--------------------------------------------------------------------------------
*/

Vector3D operator*(const Matrix3 &a, const Vector3D &b) 
{
   Vector3D c;
   
   for(int i = 0; i < 3; i++)
    {
      c[i] = 0;
      for(int k = 0; k < 3; k++)
        c[i] += a.m[i][k] * b[k];
    }    
    
   return c;  
}


