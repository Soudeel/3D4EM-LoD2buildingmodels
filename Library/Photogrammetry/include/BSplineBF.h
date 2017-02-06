
/*
                Copyright 2010 Delft University of Technology
 
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
 * \brief Interface to Class BSplineBF - A knot vector for a B-spline curve and the normalized B-spline functions
 *                                       where the knot vector does not have to be uniform. However, the basis
 *                                       functions are NOT rational, therefore no NURBS (only "NUBS").
 *
 */
/*!
 * \class BSplineBF
 * \ingroup Photogrammetry
 * \brief Interface to Class BSplineBF - A knot vector for a B-spline curve
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Norbert Pfeifer
 * \date        January 2004 (Created)
 *
 * \remark \li Define basis for B-spline curves, BSpineBF.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef B_SPLINE_BASIS_FUNCTIONS_H
#define B_SPLINE_BASIS_FUNCTIONS_H

#include <iostream>

/// The BSpline basis functions over a given knot vector

class BSplineBF
{
protected: 
  /// knot vector, has k+1 entries
  double* u;

  /// highest index in knot vector (stored this way for compliance with standard B-spline notation)
  int k;

public:
  /// default constructor, construct an empty knot vector
  BSplineBF(); 

  /// constructor, set value of k and provide values
  BSplineBF( int SetK, double* knots );

  /// constructor, set value of k, space is reserved, values not initialized
  BSplineBF( int SetK );

  /// copy constructor
  BSplineBF( const BSplineBF& other );

  /// destructor
  ~BSplineBF();

  /// assignment operator
  void operator=( const BSplineBF& right );

  /// reset with given size (k-value), do not initialize values
  void Reset( int k ); 

  /// check integrity, return 0 on failure, 1 otherwise
  /// This function checks, that the knot sequence is weakly increasing, 
  /// which means, that u_i < u_{i+m} for all indices
  int Check()const; 

  /// get function value for degree n in i-th segment at parameter t
  /// in a secure manner, throw assertions, if impossible n,i,t,u,k
  /// combinations are desired (u[0]..u[k] is the knot vector)
  /// uses function GetValue( n, t, i )
  double GetValueSecure( int n, double t, int i )const;

  /// get function value for degree n in i-th segment at parameter t
  /// no checking of parameter correctness
  /// function is computed recursively
  double GetValue( int n, double t, int i )const;

  /// return parameter value of knot i
  double GetKnot( int i )const; 

  /// return writeable reference to parameter value of knot i, 
  /// with this function an incorrect knot value can be 
  /// made. Use Check() to make sure, that this has not happened. 
  double& GetKnot( int i );

  /// get the domain for a degree n
  void GetDomain( int n, double& uMin, double& uMax )const; 

  /// get the highest knot vector index
  int GetMaxIndex()const; 

  /// write the knot vector to a stream, friend declaration also used as function declaration
  friend std::ostream& operator<<( std::ostream& s, const BSplineBF& knotV ); 

  /// read the knot vector from a stream (compatible to operator<<), 
  /// friend declaration also used as function declaration
  friend std::istream& operator>>( std::istream& s, BSplineBF& knotV );

protected:
  /// function for recursive computation of function value
  /// for given knot vector u, for degree n, to parameter t, in interval l
  double N( int n, double t, int l )const;
};

/// inline function definition of GetValue.
inline
double BSplineBF::GetValue( int n, double t, int i )const
{
  if( u[i] <= t && t <= u[i+n+1] )
    return N( n, t, i );
  else
    return 0.0;
}

/// inline function definition of recursive computation of normalized B-spline function
inline
double BSplineBF::N( int n, double t, int l )const
{
  if( n==0 )
  {
    if( u[l]<=t && t<u[l+1] )
	  return 1;
	return 0;
  }
  double denom1 = u[l+n]    -u[l];
  double denom2 = u[l+n+1]  -u[l+1];
  double add1=0;
  double add2=0;
  if( denom1>0 ) add1 = (t       -u[l])/denom1*N(n-1,t,l  );
  if( denom2>0 ) add2 = (u[l+n+1]-t   )/denom2*N(n-1,t,l+1);
  return add1+add2;
}

#endif

