
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
 * \brief Interface to Class Covariance2D - Covariances of a two-dimensional point
 *
 */
/*!
 * \class Covariance2D
 * \ingroup PStatistic
 * \brief Interface to Class Covariance2D - Covariances of a two-dimensional point
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
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


#ifndef _Covariance2D_h_
#define _Covariance2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Covariance2D         - Covariances of a two-dimensional point

--------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------
///                 Covariance matrix of a 2D point
//------------------------------------------------------------------------------

class Covariance2D {

  protected:
    /// Covariances (2 variances and 1 covariance)
    double var[3];

  public:

    /// Default constructor, all variances are set to zero
    Covariance2D()
      { var[0] = var[1] = var[2] = 0; }
	
    /// Construct from specified values
    /** @param v_x   Variance of x-coordinate
        @param v_y   Variance of y-coordinate
        @param cv_xy Covariance of x- and y-coordinate
    */
    Covariance2D(double v_x, double v_y, double cv_xy)
      { var[0] =v_x; var[1] = v_y; var[2] = cv_xy; }

    /// Copy constructor
    Covariance2D(const Covariance2D &cv)
      { var[0] = cv.var[0]; var[1] = cv.var[1]; var[2] = cv.var[2]; }

    /// Destructor
    ~Covariance2D() {};

    /// Copy assignament
    Covariance2D& operator=(const Covariance2D &cv)
      { var[0] = cv.var[0]; var[1] = cv.var[1]; var[2] = cv.var[2]; 
        return *this; }

    /// Return the readable reference
    const Covariance2D &covar() const
      { return *this;}

    /// Return the writable reference
    Covariance2D &covar()
      { return *this; }
      
    /// Return the (co-)variances in a vector
    /** Memory for the vector v needs to be allocated (space for 3 doubles)
        @param v Vector for the variance and covariance values
    */
    void GetCovariances(double *v) const
    { for(int i = 0; i < 3; i++) v[i] = var[i]; }  

    /// Return a specific (co-)variance
    /** Index 0 returns the variance of the x-coordinate.
        Index 1 returns the variance of the y-coordinate.
        Index 2 returns the covariance of the x- and y-coordinate.
    */
    double GetCovar(int i) const
      { return var[i]; }

    /// Return a specific readable (co-)variance
    /** Index 0 returns the variance of the x-coordinate.
        Index 1 returns the variance of the y-coordinate.
        Index 2 returns the covariance of the x- and y-coordinate.
    */
    double Covar(int i) const
      { return var[i]; }

    /// Return a specific writable (co-)variance
    /** Index 0 returns the variance of the x-coordinate.
        Index 1 returns the variance of the y-coordinate.
        Index 2 returns the covariance of the x- and y-coordinate.
    */
    double &Covar(int i)      
      { return var[i]; }

};

#endif /* _Covariance2D_h_ */   /* Do NOT add anything after this line */
