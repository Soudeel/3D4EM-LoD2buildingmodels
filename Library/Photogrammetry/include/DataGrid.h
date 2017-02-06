
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
 * \brief Interface to Class DataGrid - Grid definition of an image with data other than grey values
 *
 */
/*!
 * \class DataGrid
 * \ingroup Photogrammetry
 * \brief Interface to Class DataGrid - Grid definition of an image with data other than grey values
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li The relation between a row-column coordinate system and an XY coordinate
 *  system is inherited from the class ImageGrid. The DataGrid class adds an
 *  offset and scale for the conversion of image grey values to some other
 *  data type (e.g. height). The relation between these two is specified by:
 *  <br> data_value = data_offset + grey_value * data_scale.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _DataGrid_h_
#define _DataGrid_h_

/*
--------------------------------------------------------------------------------
  DataGrid - Grid definition of an image with data other than grey values
--------------------------------------------------------------------------------
*/

#include "ImageGrid.h"
#include "Database4Cpp.h"

//------------------------------------------------------------------------------
// Grid definition of imaged data.
/* The relation between a row-column coordinate system and an XY coordinate
    system is inherited from the class ImageGrid. The DataGrid class adds an
    offset and scale for the conversion of image grey values to some other
    data type (e.g. height). The relation between these two is specified by:
    data_value = data_offset + grey_value * data_scale.
*/
//------------------------------------------------------------------------------

class DataGrid : public ImageGrid {

  protected:

    /// Offset of the imaged data (data value corresponding to grey value 0)
    double data_offset;

    /// Scale of the imaged data (data value increment corresponding to one grey value level)
    double data_scale;

  public:

    /// Default constructor, offset set to 0, scale set to 1
    DataGrid() : ImageGrid()
      {data_offset = 0.0;  data_scale = 1.0;}

    /// Construct by reading a grid definition file
    DataGrid(const char *filename, int *success) : ImageGrid()
      {*success = Read(filename);}

    /// Construct from specified values
    DataGrid(double x_offset, double y_offset, double pixelsize,
             double d_offset, double d_scale) :
      ImageGrid(x_offset, y_offset, pixelsize)
      {data_offset = d_offset;  data_scale = d_scale;}

    /// Construct by converting a C object
    DataGrid(Grid3D *grid3D) : ImageGrid()
      {C2Cpp(grid3D);}

    /// Copy constructor
    DataGrid(const DataGrid &grid) : ImageGrid()
       { *this = grid; } 	

    /// Default destructor
    ~DataGrid() {};

    /// Copy assignament
    DataGrid& operator=(const DataGrid &grid);	

    /// Conversion from C to C++ object
    void C2Cpp(Grid3D *);

    /// Conversion from C++ to C object
    void Cpp2C(Grid3D **) const;

    /// Read data grid definition from a file
    int Read(const char *filename);

    /// Write data grid definition to a file
    int Write(const char *filename) const;

    /// Print the data grid definition to stdout
    void Print() const;

    /// Return the values of all DataGrid class parameters
    /** Pointers should point to already allocated memory */
    void GetData(double *dest_x, double *dest_y, double *dest_pixelsize,
		 double *dest_data_offset, double *dest_data_scale) const
      {*dest_x = x;  *dest_y = y; *dest_pixelsize = pixelsize;
       *dest_data_offset = data_offset; *dest_data_scale = data_scale;}

     /// Return the readable offset of the imaged data
     double DataOffset() const
       {return(data_offset);}

     /// Return the writable offset of the imaged data
     double &DataOffset()
       {return(data_offset);}

     /// Return the readable scale of the imaged data
     double DataScale() const
       {return(data_scale);}

     /// Return the writable scale of the imaged data
     double &DataScale()
       {return(data_scale);}

     /// Convert data to a grey value
     unsigned char GreyValue(double data) const;
};
#endif /* _DataGrid_h_ */   /* Do NOT add anything after this line */
