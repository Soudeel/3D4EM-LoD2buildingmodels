
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
 * \brief Interface to Class BSplineCurve - A B-spline curve over a non uniform knot-vector. The curve may have any
 *                                          degree, larger zero. Open and closed, end point interpolation 
 *                                          B-Splines are supported. The control points, which are the same type as 
 *                                          the curve points, are templates. Therefore, 2D and 3D curves can be used. 
 *                                          Conversion into splines (e.g. in Bezier form) is currently not supported.
 *                                       
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
 * \remark \li Define B-spline curves, BSpineCurve.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef B_SPLINE_CURVE_H
#define B_SPLINE_CURVE_H

#include <vector>
#include "BSplineBF.h"

/// The B-spline curve over a given knot vector, for a given sequence of control points (template parameter). 

template< class POINT >
class BSplineCurve
{
protected:
  /// knot sequence u[0], u[1], ... u[k]
  BSplineBF knotSequence; 

  /// highest control point index, l=controlPoints.size()-1
  int l; 

  /// degree of curve (e.g. 3 ... cubic B-Spline)
  int n; 

  /// control points, indices go from 0 to k, where k is stored in knotSequence
  std::vector< POINT > controlPoints; 

public: 
  /// contstruct with the control polygon, assume degree 3 and end-point interpolation, 
  /// and uniform spacing of knot vector
  BSplineCurve( const std::vector< POINT >& ctrlPoly ); 

  /// get the point on the B-Spline curve to the parameter t, 
  /// do not check, if t is a valid parameter
  POINT GetPoint( double t )const; 

  /// get the point on the B-Spline curve to the parameter t, 
  /// under the assumption, that the polygon is closed. It is, according
  /// to the knot vector u periodically extended in itself
  /// i.e.: controlPoint[l+1] = controlPoint[0], controlPoint[l+2] = controlPoint[1], ... 
  /// it is NOT checked, if t is a valid parameter
  POINT GetPointClosed( double t )const; 

  /// get the domain of the B-Spline curve
  void GetDomain( double& uMin, double& uMax )const; 

  /// method to set knot vector
  /// end points and tangents (and higher order elements, up to degree n)
  /// are interpolated; knot vector is uniform
  void SetKnotsUniformEndInterpol( int degree ); 

  /// method to set knot vector
  /// control polygon is closed. Every knot has muliplicity one
  /// which leads to a B-Spline curve that is everywhere n-1 continuous
  /// knot vector is uniform
  void SetKnotsCyclic( int degree ); 

  /// return the knot vector
  BSplineBF GetKnotVector()const;

private:

  /// compute for the given degree n the highest knot sequence index
  /// from the given number of control points
  int ComputeK()const; 

}; 

#include "BSpline.tcc"

#endif
