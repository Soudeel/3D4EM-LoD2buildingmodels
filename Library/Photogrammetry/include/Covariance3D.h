
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
 * \brief Interface to Class Covariance3D - Covariances of a three-dimensional point
 *
 */
/*!
 * \class Covariance3D
 * \ingroup PStatistic
 * \brief Interface to Class Covariance3D - Covariances of a three-dimensional point
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


#ifndef _Covariance3D_h_
#define _Covariance3D_h_

#include "vMatrix.h"

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Covariance3D         - Covariances of a three-dimensional point

--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                         Covariance matrix of a 3D point
--------------------------------------------------------------------------------
*/

class Covariance3D {

  protected:
    /// Covariances (3 variances and 3 covariances)
    double var[6];

  public:
    /// Default constructor
    Covariance3D()
      {for(int i = 0; i < 6; i++) var[i] = 0;}

    /// Construct from specified variances and covariances
    Covariance3D(double v_x, double v_y, double v_z,
    		 double cv_xy, double cv_xz, double cv_yz)
      { var[0] = v_x; var[1] = v_y; var[2] = v_z;
        var[3] = cv_xy; var[4] = cv_xz; var[5] = cv_yz; }		 	

    /// Copy constructor
    Covariance3D(const Covariance3D &cv)
      { for (int i = 0; i < 6; i++) var[i] = cv.var[i]; }	  	

    /// Destructor
    ~Covariance3D() {};

    /// Copy assignament
    Covariance3D& operator=(const Covariance3D &cv)
      { for (int i = 0; i < 6; i++) var[i] = cv.var[i]; 	  	
        return *this; }
      
    /// Return the covariances in a vector
    /** Memory for the double array v should have been allocated (6 doubles) */
    void GetCovariances(double *v) const
      {for (int i = 0; i < 6; i++) v[i] = var [i];}	    	

    /// Return a specific covariance
    /** Index 0, 1, 2 return the variance of the x-, y-, and z-coordinate
        respectively. Index 3, 4, and 5 return the covariances of the x- and
        y-coordinate, of the x- and z-coordinate and of the y- and z-coordinate
        respectively.
    */
    double GetCovar(int i)
      { return var[i]; }

    /// Return a specific readable covariance
    /** Index 0, 1, 2 return the variance of the x-, y-, and z-coordinate
        respectively. Index 3, 4, and 5 return the covariances of the x- and
        y-coordinate, of the x- and z-coordinate and of the y- and z-coordinate
        respectively.
    */
    double Covar(int i) const
      { return var[i]; }

    /// Return a specific writable covariance
    /** Index 0, 1, 2 return the variance of the x-, y-, and z-coordinate
        respectively. Index 3, 4, and 5 return the covariances of the x- and
        y-coordinate, of the x- and z-coordinate and of the y- and z-coordinate
        respectively.
    */
    double &Covar(int i)
      { return var[i]; } 

    /// Return the readable reference
    const Covariance3D &covar() const
      { return *this; }
  
    /// Return the writable reference
    Covariance3D &covar()
      { return *this; }	

    /// Return covariances as a 3x3 matrix
    vMatrix CovarianceMatrix() const;

    /// Extract values from a 3x3 covariance matrix
    void CovarianceMatrix(const vMatrix &);
};

#endif /* _Covariance3D_h_ */   /* Do NOT add anything after this line */
