
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



#include <assert.h>
#include "BSplineBF.h"

/// constructor without arguments, set number of elements to zero
BSplineBF::BSplineBF()
: u(0)
, k(0)
{}

/// constructor with number of elements and values of knot vector
BSplineBF::BSplineBF( int SetK, double* knots )
{
  assert( SetK > 0 );
  k = SetK; 
  u = new double[k+1]; 
  for( int i=0; i<=k; i++ )
    u[i]=knots[i]; 
}

/// constructor, set value of k, space is reserved, values not initialized
BSplineBF::BSplineBF( int SetK )
{
  assert( SetK > 0 ); 
  k = SetK; 
  u = new double[k+1]; 
}

/// copy constructor
BSplineBF::BSplineBF( const BSplineBF& other )
{
  k = other.GetMaxIndex(); 
  u = new double[k+1]; 
  for( int i=0; i<=k; i++ )
    u[i]=other.GetKnot(i); 
}

/// destructor
BSplineBF::~BSplineBF()
{
  if( k>0 )
    delete [] u;
}

/// assignment operator
void BSplineBF::operator=( const BSplineBF& right )
{
  if( this == &right )
    return; 
  if( k>0 )
    delete [] u;
  k = right.GetMaxIndex();
  for( int i=0; i<=k; i++ )
    u[i]=right.GetKnot(i); 
}

/// reset the knot vector with a given number of elements, values are not initialized
void BSplineBF::Reset( int SetK )
{
  if( k>0 )
    delete [] u;
  assert( SetK > 0 ); 
  k = SetK; 
  u = new double[k+1]; 
}

/// check integrity, return 0 on failure, 1 otherwise
int BSplineBF::Check()const
{
  for( int i=0; i<k; i++ )
    if( u[i] > u[i+1] )
      return 0; 
  return 1; 
}

/// get function value for degree n in i-th segment at parameter t
/// in a secure manner, throw assertions, if impossible n,i,t,u,k
/// combinations are desired
double BSplineBF::GetValueSecure( int n, double t, int i )const
{
  assert( n>=0 );        // proper degree
  assert( i>=0 );        // proper segment, lower bound
  assert( i<=k-n-1 );    // proper segment, upper bound
  // the following 2 assertions may be removed somewhen. 
  // WHAT IS CHECKED FOR IS NOT NECESSARILY AN ERROR CONDITION.
  assert( t>=u[i] );     // looking for a value, where basis function is zero? 
  assert( t<=u[i+n+1] ); // looking for a value, where basis function is zero?
  return GetValue( n, t, i );
}

/// return parameter value of knot i
double BSplineBF::GetKnot( int i )const 
{
  assert( i>=0 ); 
  assert( i<= k ); 
  return u[i]; 
}

/// return writeable reference to parameter value of knot i, 
/// with this function an incorrect knot value can be 
/// made. Use Check() to make sure, that this has not happened. 
double& BSplineBF::GetKnot( int i )
{
  return u[i]; 
}

/// get the domain for a degree n
void BSplineBF::GetDomain( int n, double& uMin, double& uMax )const 
{
  uMin = u[n]; 
  uMax = u[k-n]; 
}

/// get the highest knot vector index
int BSplineBF::GetMaxIndex()const 
{
  return k;
}

/// write knot vector to stream
std::ostream& operator<<( std::ostream& s, const BSplineBF& knotV )
{
  int k = knotV.GetMaxIndex();
  s << k << "    ";
  for( int i=0; i<=k; i++ )
    s << knotV.GetKnot( i ) << ' ';
  return s; 
}

/// read knot vector from stream
std::istream& operator>>( std::istream& s, BSplineBF& knotV )
{
  int kLoc;
  s >> kLoc; 
  knotV.Reset( kLoc );
  for( int i=0; i<=kLoc; i++ )
    s >> knotV.GetKnot( i );
}

