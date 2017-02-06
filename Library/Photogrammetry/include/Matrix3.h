
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



/*!
 * \file
 * \brief Interface to Class Matrix3 - A 3x3 matrix class
 *
 */
/*!
 * \class Matrix3
 * \ingroup LinearAlgebra
 * \brief Interface to Class Matrix3 - A 3x3 matrix class
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Matrix_h_
#define _Matrix_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  Matrix3  - A 3x3 matrix class
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

class Vector2D;
class Vector3D;

/*
--------------------------------------------------------------------------------
                                    Matrix
--------------------------------------------------------------------------------
*/


class Matrix3
{
	double m[3][3];
	
public:
	Matrix3()			/* Default constructor                      */	
	  {
	    m[0][0] = 0; m[0][1] = 0; m[0][2] = 0; 
	    m[1][0] = 0; m[1][1] = 0; m[1][2] = 0; 
	    m[2][0] = 0; m[2][1] = 0; m[2][2] = 0;
	  } 
		
	Matrix3(double a11, double a12, double a13, 
	         double a21, double a22, double a23,
	         double a31, double a32, double a33) 	
	  { m[0][0] = a11; m[0][1] = a12; m[0][2] = a13; 
	    m[1][0] = a21; m[1][1] = a22; m[1][2] = a23; 
	    m[2][0] = a31; m[2][1] = a32; m[2][2] = a33;
	  }

        Matrix3(double[3][3]);

	Matrix3(double *);	  
	  				
	Matrix3 &operator=(const Matrix3&);    /* Copy assignment             */			
	  
	~Matrix3() {};		/* Default destructor                       */
	
	double Det();
		
	Matrix3 Transpose();
	
	Matrix3 Inverse();
	
	Vector2D Nullspace();
	
	void Print();
	
	friend Matrix3 operator*(double, const Matrix3 &);
	
	friend Matrix3 operator*(const Matrix3 &, const Matrix3 &);
	
	friend Vector3D operator*(const Matrix3 &, const Vector3D &); 

};

#endif /* _Matrix_h_ */   /* Do NOT add anything after this line */
