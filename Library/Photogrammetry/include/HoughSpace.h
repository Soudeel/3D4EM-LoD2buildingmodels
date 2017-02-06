
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
 * \brief Class HoughSpace - Parameter space for 2D or 3D Hough transform
 *
 */
/*!
 * \class Covariance2D
 * \ingroup PStatistic
 * \brief Class HoughSpace - Parameter space for 2D or 3D Hough transform
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date                ---- (Created)
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

#ifndef _HoughSpace_h_
#define _HoughSpace_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  HoughSpace              - A Hough space of a clustering operation

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include "Line2D.h"
#include "Plane.h"

//------------------------------------------------------------------------------
/// Parameter space for a 2D or 3D Hough transform
//------------------------------------------------------------------------------

class HoughSpace {
  protected:

    /// The bins of the parameters space for counting the frequencies
    int *space;

    /// Dimensions of the Hough space
    int dim1, dim2, dim3;

    /// Minimum values for each dimension of the Hough space
    double min1, min2, min3;

    /// Maximum values for each dimension of the Hough space
    double max1, max2, max3;

    /// Bin sizes for each dimension of the hough space
    double int1, int2, int3;

    /// Offsets to be applied to coordinates before transformation to the Hough space
    double offX, offY, offZ;

    /// Equation type of the Hough space
    /** Possible equations for plane detection in 3D space
        0:  Z = slope_X * X + slope_Y * Y + d      (default model)
        1:  sin a sin b X + sin a cos b Y + cos a Z = d
        
        Possible equations for line detection in 2D space
        0:  Y = a X + b
        1:  X cos a + Y sin a = d (default model)
        2:  same as (1), but with preferred directions instead of direction range
    */
    int eq;

    /// Sine and cosine tables for equation type 1
    double *sin1, *cos1, *sin2, *cos2;

  public:

    /// Default constructor
    HoughSpace()
      {space = NULL;
       dim1 = dim2 = dim3 = 0;
       min1 = min2 = min3 = 0.0;
       max1 = max2 = max3 = 0.0;
       offX = offY = offZ = 0.0;
       eq = 0;
       sin1 = cos1 = sin2 = cos2 = NULL;}

    /// Constructor for a 2D Hough space
    HoughSpace(int dimension1, int dimension2,
               double minimum1, double minimum2,
               double maximum1, double maximum2, int equation=1);

    /// Constructor for a 3D Hough space
    HoughSpace(int dimension1, int dimension2, int dimension3,
               double minimum1, double minimum2, double minimum3,
               double maximum1, double maximum2, double maximum3,
               int equation=0);

    /// Default destructor
    ~HoughSpace()
      {if (dim1*dim2*dim3 && space) {
         dim1 = dim2 = dim3 = 0; free(space); space = NULL; }}

  private:
    /// Initialisation called by one of the next constructors
    void Initialise(double *directions=NULL);

  public:
    /// Initialisation of a 2D Hough space
    void Initialise(int dimension1, int dimension2,
                    double minimum1, double minimum2,
                    double maximum1, double maximum2, int equation=1);

    /// Initialisation of a 2D Hough space with preferred directions
    void Initialise(int dimension1, double *directions,
                    int dimension2, double minimum2, double maximum2);
                    
    /// Initialisation of a 3D Hough space
    void Initialise(int dimension1, int dimension2, int dimension3,
                    double minimum1, double minimum2, double minimum3,
                    double maximum1, double maximum2, double maximum3,
                    int equation=0);

  private:
    /// Set the dimensions of the Hough space
    /** This function is used by Initialise() */
    void SetDimensions(int dimension1, int dimension2, int dimension3);

    /// Set the range of the parameters of the Hough space
    /** This function is used by Initialise() */
    void SetRange(double, double, double, /* Set the range of the space       */
                  double, double, double);

  public:
    /// Return the first dimension size
    int Dimension1() const {return dim1;}
    
    /// Return the second dimension size
    int Dimension2() const {return dim2;}
    
    /// Return the third dimension size
    int Dimension3() const {return dim3;}
    
    /// Set the offsets for the coordinates
    /** This offsets are subtracted from the coordinate of points before these
        points are added or removed to/from the Hough space.
    */
    void SetOffsets(double offsetX, double offsetY, double offsetZ);

    /// Set the offsets for the coordinates
    /** This offsets are subtracted from the coordinate of points before these
        points are added or removed to/from the Hough space.
    */
    void SetOffsets(double offsetX, double offsetY)
      { SetOffsets(offsetX, offsetY, 0.0); }

    /// Set all counters of the Hough space to zero
    void Clear();
    
    /// Add or remove a point to/from the 2D Hough space
    /** A point is added to the 2D Hough space with a specified weight. A weight
        of -1 implies that the point is removed from the Hough space
    */
    void ModifyPoint(int weight, double X, double Y);

    /// Add or remove a point to/from the 2D Hough space
    /** A point is added to the 2D Hough space with a specified weight. A weight
        of -1 implies that the point is removed from the Hough space
    */
    void ModifyPoint(int weight, const Position2D &pos)
      { ModifyPoint(weight, pos.X(), pos.Y()); }

    /// Add a point to the 2D Hough space
    void AddPoint(double X, double Y)
      { ModifyPoint(1, X, Y); }

    /// Add a point to the 2D Hough space
    void AddPoint(const Position2D &pos)
      { ModifyPoint(1, pos.X(), pos.Y()); }

    /// Remove a point from the 2D Hough space
    void RemovePoint(double X, double Y)
      { ModifyPoint(-1, X, Y); }

    /// Remove a point from the 2D Hough space
    void RemovePoint(const Position2D &pos)
      { ModifyPoint(-1, pos.X(), pos.Y()); }

    /// Add or remove a point to/from the 3D Hough space
    /** A point is added to the 3D Hough space with a specified weight. A weight
        of -1 implies that the point is removed from the Hough space
    */
    void ModifyPoint(int weight, double X, double Y, double Z);

    /// Add or remove a point to/from the 3D Hough space
    /** A point is added to the 3D Hough space with a specified weight. A weight
        of -1 implies that the point is removed from the Hough space
    */
    void ModifyPoint(int weight, const Position3D &pos)
      { ModifyPoint(weight, pos.X(), pos.Y(), pos.Z()); }

    /// Add a point to the 3D Hough space
    void AddPoint(double X, double Y, double Z)
      { ModifyPoint(1, X, Y, Z); }

    /// Add a point to the 3D Hough space
    void AddPoint(const Position3D &pos)
      { ModifyPoint(1, pos.X(), pos.Y(), pos.Z()); }

    /// Remove a point from the 3D Hough space
    void RemovePoint(double X, double Y, double Z)
      { ModifyPoint(-1, X, Y, Z); }

    /// Remove a point from the 3D Hough space
    void RemovePoint(const Position3D &pos)
      { ModifyPoint(-1, pos.X(), pos.Y(), pos.Z()); }
      
    /// Print the values of a 2D Hough space
    void Print();
      
  private:
    /// Return the n-best maximum of the Hough space
    int Maximum(int num_ignore, int local_max_window,
                int &i1max, int &i2max, int &i3max);

  public:
    /// Return the n-best line from a 2D Hough space
    /** A local maximum filter is used to avoid multiple detections of the
        same line.
        @param numpts The number of points in the returned line is
                      stored in this variable.
        @param num_ignore Ignore the first "num_ignore" best lines. I.e.,
                          num_ignore=0 will return the best line.
        @param window_size Size of the local maximum filter window.
        @return The n-best line.
    */
    Line2D BestLine(int *numpts, int num_ignore, int window_size);

    /// Return the n-best plane from a 3D Hough space
    /** A local maximum filter is used to avoid multiple detections of the
        same plane.
        @param numpts The number of points in the returned plane is
                      stored in this variable.
        @param num_ignore Ignore the first "num_ignore" best planes. I.e.,
                          num_ignore=0 will return the best plane.
        @param window_size Size of the local maximum filter window.
        @return The n-best plane.
    */
    Plane BestPlane(int *numpts, int num_ignore, int window_size);

    /// Return the n-best gable roof from a 3D Hough space
    /** A local maximum filter is used to avoid multiple detections of the
        same roof.
        @param numpts The number of points in the returned gable roof is
                      stored in this variable.
        @param num_ignore Ignore the first "num_ignore" best roofs. I.e.,
                          num_ignore=0 will return the best roof.
        @param window_size Size of the local maximum filter window.
    */
    void BestGable(int &numpts, int num_ignore, int window_size);

    /// Return the bin size of the specified dimension
    double BinSize(int dimension);

    /// Add or remove a gable roof point to/from the Hough space
    /** A point of a gable roof is added to the Hough space with a specified
        weight. A weight of -1 implies that the point is removed from the
        Hough space.
    */
    void ModifyGablePoint(int weight, const Position3D &pos,
                          const Line2D &line);

    /// Add a gable roof point from the Hough space
    void AddGablePoint(const Position3D &pos, const Line2D &line)
      { ModifyGablePoint(1, pos, line); }

    /// Remove a gable roof point from the Hough space
    void RemoveGablePoint(const Position3D &pos, const Line2D &line)
      { ModifyGablePoint(-1, pos, line); }
      
    /// Delete all data and tables from Hough space
    void Erase();
};

#endif /* _HoughSpace_h_ */   /* Do NOT add anything after this line */
