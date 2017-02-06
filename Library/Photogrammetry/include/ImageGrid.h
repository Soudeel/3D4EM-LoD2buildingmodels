
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
 * \brief Interface to Class ImageGrid - Grid definition of an image
 *
 */
/*!
 * \class ImageGrid
 * \ingroup Photogrammetry
 * \brief Interface to Class ImageGrid - Grid definition of an image
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Grid definition of an image - The grid is defined by the X- and Y-coordinates in object space of the
 *   left upper corner of the left upper pixel and by the pixel size. All
 *   values are assumed to be specified in meters. This class allows easy
 *   conversion from pixel coordinates to object coordinates in case of
 *   rectified imagery, orthophoto's or imaged height data.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ImageGrid_h_
#define _ImageGrid_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImageGrid                          - Grid definition of an image

--------------------------------------------------------------------------------
*/

#include "Database4Cpp.h"

//------------------------------------------------------------------------------
/// Grid definition of an image
/** The grid is defined by the X- and Y-coordinates in object space of the
    left upper corner of the left upper pixel and by the pixel size. All
    values are assumed to be specified in meters. This class allows easy
    conversion from pixel coordinates to object coordinates in case of
    rectified imagery, orthophoto's or imaged height data.
*/
//------------------------------------------------------------------------------

class ImageGrid {

  protected:

    /// X-coordinate of the left upper corner of the left upper pixel
    double x;

    /// Y-coordinate of the left upper corner of the left upper pixel
    double y;

    /// Pixel spacing in (m)
    double pixelsize;

  public:

    /// Default constructor, coordinates set to 0, scale set to 1
    ImageGrid()
      {x = y = 0.0;  pixelsize = 1.0;};

    /// Construct with specified values
    ImageGrid(double x_coordinate, double y_coordinate, double pixel_size)
      {x = x_coordinate;  y = y_coordinate;  pixelsize = pixel_size;}

    /// Construct by reading a grid definition file
    ImageGrid(const char *filename, int *success)
      {*success = Read(filename);};

    /// Construct by converting a C object
    ImageGrid(Grid *grid)
      {C2Cpp(grid);};

    /// Return the readable reference
    const ImageGrid &ImageGridReference() const
      {return(*this);}

    /// Return the writable reference
    ImageGrid &ImageGridReference()
      {return(*this);}

    /// Conversion from C to C++ object
    void C2Cpp(Grid *);

    /// Conversion from C++ to C object
    void Cpp2C(Grid **) const;

    /// Read the image grid definition from a file
    int Read(const char *filename);

    /// Write the image grid definition to a file
    int Write(const char *filename) const;

    /// Print the image grid definition to stdout
    void Print() const;

    /// Return the grid definition data
    void GetData(double *x_coordinate, double *y_coordinate,
                 double *pixel_size) const
      {*x_coordinate = x;  *y_coordinate = y; *pixel_size = pixelsize;}

    /// Return the readable X offset
    double XOffset() const
      {return(x);}

    /// Return the writable X offset
    double &XOffset()
      {return(x);}

    /// Return the readable Y offset
    double YOffset() const
      {return(y);}

    /// Return the writable Y offset
    double &YOffset()
      {return(y);}

    /// Return the readable pixel spacing
    double Pixelsize() const
      {return(pixelsize);}

    /// Return the writable pixel spacing
    double &Pixelsize()
      {return(pixelsize);}
};
#endif /* _ImageGrid_h_ */   /* Do NOT add anything after this line */
