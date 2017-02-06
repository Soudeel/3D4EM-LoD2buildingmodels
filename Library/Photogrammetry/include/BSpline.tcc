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

#ifndef B_SPLINE_IMPL
#define B_SPLINE_IMPL

#include <assert.h>
#include <vector>
#include "BSpline.h"

/// constructor with given control polygon and set degree to 3
template< class POINT >
BSplineCurve< POINT >::BSplineCurve( const std::vector< POINT >& ctrlPoly )
{
  controlPoints.resize( ctrlPoly.size() ); 
  for( int i=0; i<controlPoints.size(); i++ )
    controlPoints[i] = ctrlPoly[i];
  n = 3; 
  l = controlPoints.size()-1;
  SetKnotsUniformEndInterpol( n ); 
}

/// get a point on the curve
template< class POINT >
POINT BSplineCurve< POINT >::GetPoint( double t )const
{
  POINT sum; /// must be initialized to origin !!!
  for( int j=0; j<=l; j++ )
    sum += controlPoints[j]*knotSequence.GetValue( n, t, j ); 
  return sum; 
}

/// get a point on the closed curve
template< class POINT >
POINT BSplineCurve< POINT >::GetPointClosed( double t )const
{
  POINT sum; /// must be initialized to origin !!!
  int maxIndex = knotSequence.GetMaxIndex()+1;
  for( int j=0; j<=l+n; j++ )
    sum += controlPoints[(j+n)%(l+1)]*knotSequence.GetValue( n, t, j ); 
/*    
  double scalarSum = 0.0;
  for( int jj=0; jj<=l+n; jj++ )
  {
    std::cerr << " deg " << n << " idx " << jj << " par " << t << " val " << knotSequence.GetValue( n, t, jj );
    scalarSum += knotSequence.GetValue( n, t, jj );
  }
  std::cerr << " SUM " << scalarSum << std::endl;
*/        
  return sum; 
}

/// return the parameter range of the curve. For values uMin <= t <= uMax
/// the call to GetPoint( t ) or GetPointClosed( t ) will give a meaningful result
template< class POINT >
void BSplineCurve< POINT >::GetDomain( double& uMin, double& uMax )const
{
  knotSequence.GetDomain( n, uMin, uMax );
//  uMin = knotSequence.GetKnot( n );
//  uMax = knotSequence.GetKnot( GetMaxIndex()-n );
}

template< class POINT >
int BSplineCurve< POINT >::ComputeK()const
{
  return controlPoints.size()+n;
}

/// set the knot vector to be uniform and interpolate the end points, its tangents, 
/// and the higher order elements as prescribed by the first (and last, resp.) n+1
/// control points. 
/// This function also sets the degree n of the curve. 
template< class POINT >
void BSplineCurve< POINT >::SetKnotsUniformEndInterpol( int degree )
{
  assert( degree>0 ); 
  n = degree; 
  int maxIndex = ComputeK(); 
  knotSequence.Reset( maxIndex ); 
  l = controlPoints.size()-1;
  double u=0.0; 
  int i=0;
  for( ; i<=n; i++ )
  {
    knotSequence.GetKnot( i ) = u; 
  }
  u += 1.0; 
  for( ; i<=l; i++ )
  {
    knotSequence.GetKnot( i ) = u;
    u += 1.0;
  }
  for( ; i<=maxIndex; i++ )
  {
    knotSequence.GetKnot( i ) = u; 
  }
}

/// set the knot vector to be uniform and cyclic. This can be used for a closed
/// B-spline curve. 
/// This function also sets the degree n of the curve.
template< class POINT >
void BSplineCurve< POINT >::SetKnotsCyclic( int degree )
{
  assert( degree>0 ); 
  n = degree; 
  int maxIndex = ComputeK(); 
  maxIndex += n;
  knotSequence.Reset( maxIndex ); 
  l = controlPoints.size()-1;
  double u=0; 
  for( int i=0; i<=maxIndex; i++, u+=1.0 )
    knotSequence.GetKnot( i ) = u; 
}

template< class POINT >
BSplineBF BSplineCurve< POINT >::GetKnotVector()const
{
  return knotSequence;
}

#endif
