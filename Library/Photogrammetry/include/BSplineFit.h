/*
                     Copyright 2014 University of Twente
 
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

/* Class for fitting B-spline curves to data. Knots are uniformly distributed.

   Example code:
   	
   #include "BSplineFit.h"
   
   {
   	  BSplineFit spline_curve;
   	  
   	  // Initialise spline of order "order" on range t_start-t_end with a
   	  // given interval size between the knots.
   	  spline_curve.Initialise(order, t_start, t_end, interval_size);
   	  
   	  // Add data
   	  spline_curve.AddObservation(t1, value1);
   	  spline_curve.AddObservation(t2, value2);
   	  ...
   	  
   	  // Fit spline
   	  spline_curve.FitSpline();
   	  
   	  // Retrieve spline value at a certain location
   	  value = spline_curve.Value(t);
   	  
   }
*/
   	  
#include <stdlib.h>

class BSplineFit
{

protected:
    /// Spline order
    int order;

    /// Spline curve start
    double curve_start;
    
    /// Spline curve end
    double curve_end;

    /// Number of internal knots
    int num_knots;
    
    /// Knot interval size
    double knot_interval_size;
    
    /// Number of spline functions
    int num_splines;
    
    /// Spline coefficients
    double *spline_coefficients;
    
    /// Row of design matrix
    double *a_row;
    
    /// Normal matrix
    double *ata;
    
    /// A{^T)y vector
    double *aty;
    
    /// Number of observations
    int num_obs;

private:
	
	/// Recursive definition of unit B-spline
	/** @param location Location within spline with unit knot distance
	    @param interval Spline interval
	    @param recursion_order Spline order in recursive formula
	    @return B-spline value
	*/
    double UnitBSplineValue(double location, int interval,
	                        int recursion_order) const;
	                        
	/// Recursive definition of unit B-spline derivative
	/** @param location Location within spline with unit knot distance
	    @param interval Spline interval
	    @param recursion_order Spline order in recursive formula
	    @param derivative Derivative number (first, second, ...)
	    @return B-spline value
	*/
    double UnitBSplineDerivativeValue(double location, int interval,
                                      int recursion_order, int derivative) const;

public:
    /// Index of first B-spline used at a position
    int FirstBSplineIndex(double location) const;
    
    /// B-spline at position in curve for a given spline index
    double BSplineValue(double location, int index) const;

    /// B-spline derivative at position in curve for a given spline index
    double BSplineDerivative(double location, int index, int derivative) const;

    /// B-spline at position in curve for a given spline start location
    double BSplineValue(double location, double bspline_start_location) const;
    
    /// Default constructor
	BSplineFit() { num_knots = 0; spline_coefficients = ata = aty = a_row = NULL; }
		
    /// Define spline range, order and knot interval
    /** Construct spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param numk  Number of internal knots
    */
	BSplineFit(int order, double start, double end, int numk)
	  { Initialise(order, start, end, numk, false); }	

    /// Initialise spline range, order and knot interval
    /** Initialise spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param numk  Number of internal knots
        @param erase Free previously allocated memory
    */
    void Initialise(int order, double start, double end, int numk,
	                bool erase=false);

    /// Initialise spline range, order and knot interval
    /** Initialise spline range, order and knot interval from
        @param order Spline order
        @param start Start of the spline curve
        @param end   End of the spline curve
        @param interval  Knot interval
        @param erase Free previously allocated memory
    */
    void Initialise(int order, double start, double end, double interval,
	                bool erase=false);

    /// Erase spline coefficients
    void Erase();
    
    /// Add observation for spline fitting
    /** @param t Location of spline value
        @param v Observed spline value at t
        @return false if t outside spline curve range
    */
    bool AddObservation(double t, double v);
    
    /// Add derivative constraint for spline fitting
    /** @param t Location of spline derivative
        @param v Desired derivative value at t
        @param derivative Derivative number (first, second, ...)
        @param weight Weight of the constraint
        @return false if t outside spline curve range
    */
    bool AddConstraint(double t, double v, int derivative, double weight);


    /// Fit a spline curve to the observations
    /** @return Error codes: 0 - success
	                         1 - insufficient observations
                             2 - singular equation system
    */
    int FitSpline();
    
    /// B-Spline value at specified location
    double Value(double t) const;
    
    /// B-Spline derivative at specified location
    double Derivative(double t, int derivative) const;
    
    /// Return order
    int Order() const {return order;}    
    
    /// Return the number of splines
    int NumberOfSplines() const {return num_splines;}
    
    /// Return the number of knots
    int NumberOfKnots() const {return num_knots;}
    
    /// Return the curve start
    double Start() const {return curve_start;}
    
    /// Return the curve end
    double End() const {return curve_end;}
	               
	/// Return the knot interval
	double KnotInterval() const {return knot_interval_size;}
    
    /// Return the spline coefficients
    double *SplineCoefficients() {return spline_coefficients;}

    /// Extrapolate spline to a new range
    int Extrapolate(double new_start, double new_end);

    /// Set up a linear spline with two knots
    void SetLinear(double start, double end,
	               double start_value, double end_value);
};
