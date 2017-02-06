
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
 * \brief Interface to Class ImagedData - Image and grid of imaged data
 *
 */
/*!
 * \class ImagedData
 * \ingroup LIDAR
 * \brief Interface to Class ImagedData - Image and grid of imaged data
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

#ifndef _ImagedData_h_
#define _ImagedData_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImagedData                              - Image and grid of imaged data

--------------------------------------------------------------------------------
*/

#include "DataGrid.h"
#include "Image.h"

//------------------------------------------------------------------------------
/** Types of imaged data.
	Possible types are UnknownDataType, HeightData, ReflectanceData,
        ColourData, RedData, GreenData, BlueData, IntensityData,
        PulseLengthData, and PointCountData.  */
//------------------------------------------------------------------------------

enum ImagedDataType { UnknownDataType=-1,
                      HeightData,
                      ReflectanceData,
                      ColourData,
                      RedData, GreenData, BlueData,
                      IntensityData,
                      PulseLengthData,
                      PointCountData };
typedef enum ImagedDataType ImagedDataType;


//------------------------------------------------------------------------------
///                  Image and grid definition of imaged data
//------------------------------------------------------------------------------

class ImagedData : public Image
{
  protected:

    /// File with the data grid definition
    char *grid_file;

    /// Data grid definition
    DataGrid *grid;

    /// Type of imaged data
    ImagedDataType datatype;

  public:

    /// Default constructor
    ImagedData() : Image()
      {Initialise();}

    /// Default destructor
    ~ImagedData() {};

    /// Copy assignment
    ImagedData operator = (const ImagedData &);

    /// Initialisation of a new instant
    void Initialise();

    /// Reinitialisation of an old instant
    void ReInitialise();

    /// Set the grid definition file name
    void SetGridFile(const char *filename);

    /// Return the grid definition file name
    char *GridFile() const
      {return(grid_file);}

    /// Derive the grid definition file name from "image_file"
    char *DeriveGridFileName();

    /// Read the image from "image_file"
    int ReadImage();

    /// Read the image from a file
    int ReadImage(const char *filename);

    /// Write the image to "image_file"
    int WriteImage() const;

    /// Write the image to a file
    int WriteImage(const char *) const;

    /// Return the data grid definition
    DataGrid *Grid()
      {return(grid);}

    /// Read the data grid definition from "grid_file"
    DataGrid *ReadGrid();

    /// Read the data grid definition from a file
    DataGrid *ReadGrid(const char *filename);

    /// Set the data grid definition
    void SetGrid(DataGrid *grid_src)
      {grid = grid_src;}

    /// Write the data grid definition to "grid_file"
    int WriteGrid() const;

    /// Write the data grid definition to a file
    int WriteGrid(const char *filename) const;

    /// Read both the image and data grid definitions from "image_file" and "grid_file" respectively
    bool Read();

    /// Read both the image and data grid definitions from specified files
    bool Read(const char *image_filename, const char *grid_filename);

    /// Write both the image and data grid definitions to "image_file" and "grid_file" respectively
    bool Write() const;

    /// Write both the image and data grid definitions to the specified files
    bool Write(const char *image_filename, const char *grid_filename) const;

    /// Read the meta data file
    void ReadMetaData(FILE *file);

    /// Write the meta data
    /** @param file File descriptor
        @param indent Number of characters to indent meta data in BNF file
    */
    void WriteMetaData (FILE *file, int indent) const; 

    /// Return the type of imaged data
    ImagedDataType DataType() const
      {return(datatype);}

    /// Set the type of imaged data
    void SetDataType(ImagedDataType t)
      {datatype = t;}

    /// Check if some data is available
    int HasSomeData() const;
};
#endif /* _ImagedData_h_ */  /* Don't add after this point */
