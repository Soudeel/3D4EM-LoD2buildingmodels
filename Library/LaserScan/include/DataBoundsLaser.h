
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
 * \brief Interface to Class DataBoundsLaser - Data bounds of laser data
 *
 */
/*!
 * \class DataBoundsLaser
 * \ingroup LDataOrg
 * \brief Interface to Class DataBoundsLaser - Data bounds of laser data
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

#ifndef _DataBoundsLaser_h_
#define _DataBoundsLaser_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  DataBoundsLaser                     - Data bounds of laser data

--------------------------------------------------------------------------------
*/

#include "LaserPoint.h"
#include "DataGrid.h"
#include "DataBounds3D.h"
#include "DataBounds2D.h"

//------------------------------------------------------------------------------
/*                  Image and grid definition of imaged data                 */
//------------------------------------------------------------------------------

class DataBoundsLaser
{
  protected:

    /// Minimum coordinates and attributes
    LaserPoint minimum;

    /// Maximum coordinates and attributes
    LaserPoint maximum;

    /// 0/1 switch for (un)known minima
    LaserPoint min_is_set;

    /// 0/1 switch for (un)known maxima
    LaserPoint max_is_set;

  public:

    /// Default constructor
    DataBoundsLaser() { Initialise(); }

    /// Construct from 2D data bounds
    DataBoundsLaser(const DataBounds2D &bounds)
      { Initialise(bounds); }

    /// Construct from 3D data bounds
    DataBoundsLaser(const DataBounds3D &bounds)
      { Initialise(bounds); }

    /// Default destructor
    ~DataBoundsLaser() {};

    /// Copy assignment
    DataBoundsLaser operator = (const DataBoundsLaser &);

    /// Return the minimum
    LaserPoint & Minimum() {return(minimum);}

    /// Return the minimum
    const LaserPoint Minimum() const {return(minimum);}

    /// Return the maximum
    LaserPoint & Maximum() {return(maximum);}

    /// Return the maximum
    const LaserPoint Maximum() const {return(maximum);}

    /// Set all values to unknown
    void Initialise();

    /// Initialise laser bounds from 2D bounds
    /** Height, reflectance and pulse count data are set to unknown */
    void Initialise(const DataBounds2D &bounds);

    /// Initialise laser bounds from 3D bounds
    /** Reflectance and pulse count data are set to unknown */
    void Initialise(const DataBounds3D &bounds);

    /// Set a minimum integer attribute value
    void SetMinimumValue(const LaserPointTag tag, int value);
    /// Set a minimum float attribute value
    void SetMinimumValue(const LaserPointTag tag, float value);
    /// Set a minimum integer attribute value
    void SetMinimumValue(const LaserPointTag tag, double value);
    /// Set a maximum integer attribute value
    void SetMaximumValue(const LaserPointTag tag, int value);
    /// Set a maximum float attribute value
    void SetMaximumValue(const LaserPointTag tag, float value);
    /// Set a maximum integer attribute value
    void SetMaximumValue(const LaserPointTag tag, double value);

    /// Set the minimum X coordinate
    void SetMinimumX(double Xmin);
    /// Set the minimum Y coordinate
    void SetMinimumY(double Ymin);
    /// Set the minimum Z coordinate
    void SetMinimumZ(double Zmin);
    /// Set the minimum reflectance value
    void SetMinimumReflectance(int Rmin);
    /// Set the mininum RGB values
    void SetMinimumColour(int red, int green, int blue);
    /// Set the minimum pulse count value
    void SetMinimumPulseCount(int PCmin);
    /// Set the minimum pulse length value
    void SetMinimumPulseLength(int PLmin);
    /// Set the minimum label value
    void SetMinimumLabel(int Lmin);
    /// Set the minimum scalar value
    void SetMinimumScalar(float scalarmin);
    /// Set the maximum X coordinate
    void SetMaximumX(double Xmax);
    /// Set the maximum Y coordinate
    void SetMaximumY(double Ymax);
    /// Set the maximum Z coordinate
    void SetMaximumZ(double Zmax);
    /// Set the maximum reflectance value
    void SetMaximumReflectance(int Rmax);
    /// Set the maxinum RGB values
    void SetMaximumColour(int red, int green, int blue);
    /// Set the maximum pulse count value
    void SetMaximumPulseCount(int PCmax);
    /// Set the maximum pulse length value
    void SetMaximumPulseLength(int PLmax);
    /// Set the maximum label value
    void SetMaximumLabel(int Lmin);
    /// Set the maximum scalar value
    void SetMaximumScalar(float scalarmax);

    /// Check if a minimum tag value is set
    bool MinimumIsSet(const LaserPointTag tag) const;
    /// Check if a maximum tag value is set
    bool MaximumIsSet(const LaserPointTag tag) const;

    /// Check if the minimum X coordinate is set
    bool MinimumXIsSet() const;
    /// Check if the minimum Y coordinate is set
    bool MinimumYIsSet() const;
    /// Check if the minimum Z coordinate is set
    bool MinimumZIsSet() const;
    /// Check if the minimum reflectance value is set
    bool MinimumReflectanceIsSet() const;
    /// Check if the minimum RGB values are set
    bool MinimumColourIsSet() const;
    /// Check if the minimum pulse count value is set
    bool MinimumPulseCountIsSet() const;
    /// Check if the minimum pulse length value is set
    bool MinimumPulseLengthIsSet() const;
    /// Check if the minimum label value is set
    bool MinimumLabelIsSet() const;
    /// Check if the maximum X coordinate is set
    bool MaximumXIsSet() const;
    /// Check if the maximum Y coordinate is set
    bool MaximumYIsSet() const;
    /// Check if the maximum Z coordinate is set
    bool MaximumZIsSet() const;
    /// Check if the maximum reflectance value is set
    bool MaximumReflectanceIsSet() const;
    /// Check if the maximum RGB values are set
    bool MaximumColourIsSet() const;
    /// Check if the maximum pulse count value is set
    bool MaximumPulseCountIsSet() const;
    /// Check if the maximum pulse length value is set
    bool MaximumPulseLengthIsSet() const;
    /// Check if the maximum label value is set
    bool MaximumLabelIsSet() const;

    /// Check if all X and Y bounds are set
    bool XYBoundsSet() const;
    /// Check if all X, Y, and Z bounds are set
    bool XYZBoundsSet() const;
    /// Check if both reflectance bounds are set
    bool ReflectanceBoundsSet() const;
    /// Check if both RGB bounds are set
    bool ColourBoundsSet() const;
    /// Check if both pulse count bounds are set
    bool PulseCountBoundsSet() const;
    /// Check if both pulse length bounds are set
    bool PulseLengthBoundsSet() const;
    /// Check if both label bounds are set
    bool LabelBoundsSet() const;

    /// Read data bounds from a file in the BNF format
    void Read(FILE *file);

    /// Read data bounds from a file in the BNF format
    /** @param file File descriptor
        @param endkeyword BNF tag at end of bounds, usually "enddatabounds"
    */
    void Read(FILE *file, const char *endkeyword);

    /// Write the data bounds to a file in the BNF format
    /** @param file File descriptor
        @param indent Number of characters to indent for keyword "databounds"
        @param colour 1 if the reflectance attribute contains colour
                      information, 0 otherwise.
    */
    void Write(FILE *file, int indent, int colour=0) const;

    /// Write the data bounds to a file in the BNF format
    /** @param file File descriptor
        @param indent  Number of characters to indent for the first keyword
        @param keyword BNF tag to enclose the bounds
        @param colour  1 if the reflectance attribute contains colour
                       information, 0 otherwise.
    */
    void Write(FILE *file, int indent, const char *keyword, int colour=0) const;

    /// Print the data bounds to a file
    void Print(FILE *file) const;

    /// Print the data bounds to stdout
    void Print() const;

    /// Extract the data bounds from a meta data file
    int Extract(const char *filename);

    /// Return the X coordinate range
    double XRange() const;
    /// Return the Y coordinate range
    double YRange() const;
    /// Return the Z coordinate range
    double ZRange() const;
    
    ///Returns the max range.
    double MaxRange() const;
    
    /// Return the reflectance value range
    int ReflectanceRange() const;
    /// Return the pulse count value range
    int PulseCountRange() const;
    /// Return the pulse length value range
    int PulseLengthRange() const;
    /// Return the label  value range
    int LabelRange() const;

    /// Update the data bounds with another set of data bounds
    DataBoundsLaser Update(const DataBoundsLaser &bounds);
    /// Update the data bounds with values of a laser point
    DataBoundsLaser Update(const LaserPoint &point, bool useAtts=false);

    /// Use bounds to define a data grid of imaged height data
    DataGrid *HeightDataGrid(double pixelsize) const;
    /// Use bounds to define a data grid of imaged reflectance data
    DataGrid *ReflectanceDataGrid(double pixelsize) const;
    /// Use bounds to define a data grid of imaged colour data
    DataGrid *ColourDataGrid(double pixelsize) const;
    /// Use bounds to define a data grid of imaged pulse length data
    DataGrid *PulseLengthDataGrid(double pixelsize) const;
    /// Use bounds to define a data grid of point count data
    DataGrid *PointCountDataGrid(double pixelsize) const;

    /// Check if some data bound is set
    int HasSomeData() const;

    /// Check if a point is inside the X and Y coordinate bounds
    bool InsideXY(const LaserPoint *point) const;
    /// Check if the attributes of a point are inside all set bounds
    bool InsideAttributes(const LaserPoint *point) const;
    /// Check if a point is inside all set bounds
    bool Inside(const LaserPoint *point) const;

    /// Check if two data bounds show an overlap in X and Y coordinates
    friend bool OverlapXY(const DataBoundsLaser &b1, const DataBoundsLaser &b2);

    /// Return the averages of the minimum and maximum values
    LaserPoint MidPoint() const;

    /// Return the 3D bounds
    DataBounds3D &Bounds3D() const;
    
    /// Return the 2D bounds
    DataBounds2D &Bounds2D() const;
};
#endif /* _DataBoundsLaser_h_ */  /* Don't add after this point */
