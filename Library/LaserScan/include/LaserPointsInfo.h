
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
 * \brief Interface to Class LaserPointsInfo - Meta information on a set of laser points
 *
 */
/*!
 * \class LaserPointsInfo
 * \ingroup LMetaData
 * \brief Interface to Class LaserPointsInfo - Meta information on a set of laser points
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _LaserPointsInfo_h_
#define _LaserPointsInfo_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserPointsInfo                    - Meta information on a set of laser points

--------------------------------------------------------------------------------
*/

#include "Positions2D.h"
#include "LaserScanner.h"
#include "ImagedData.h"
#include "DataBoundsLaser.h"

//------------------------------------------------------------------------------
///              Meta information on a set of laser points
//------------------------------------------------------------------------------

class LaserPointsInfo
{
  protected:

    /// Characteristics of the laser scanner
    LaserScanner scanner;

    /// Bounds of the data set
    DataBoundsLaser bounds;

    /// Imaged height data
    ImagedData imaged_height;

    /// Imaged reflectance data
    ImagedData imaged_reflectance;

    /// Imaged colour data
    ImagedData imaged_colour;
    
    /// Imaged pulse length data
    ImagedData imaged_pulselength;

    /// Imaged point count data
    ImagedData imaged_pointcount;

    /// Number of points per square meter
    double point_density;

    /// Name of the file with the data boundary
    char *boundary_file;

    /// Boundary of the data set
    Positions2D *boundary;

  public:

    /// Default constructor
    LaserPointsInfo()
      {Initialise();}

    /// Default destructor
    virtual ~LaserPointsInfo() {}

    /// Copy assignment
    LaserPointsInfo operator = (const LaserPointsInfo &info);

    /// Return the const reference
    const LaserPointsInfo &LaserPointsInfoReference() const
      {return(*this);}

    /// Return the reference
    LaserPointsInfo &LaserPointsInfoReference()
      {return(*this);}

    /// Initialisation of new instant
    void Initialise();

    /// Reinitialisation of old instant
    void ReInitialise();

    /// Read the meta data
    void ReadMetaData(FILE *file);

    /// Write the meta data
    /** @param file File descriptor
        @param indent Number of characters to indent in BNF file
    */
    void WriteMetaData(FILE *file, int indent) const;

    /// Return the scanner characteristics
    LaserScanner &Scanner()
      {return(scanner);}

    /// Return the scanner characteristics
    const LaserScanner &Scanner() const
      {return(scanner);}

    /// Return the data bounds
    DataBoundsLaser &Bounds()
      {return(bounds);}

    /// Return the data bounds
    const DataBoundsLaser &Bounds() const
      {return(bounds);}

    /// Set the data bounds
    void SetBounds(DataBoundsLaser b)
      {bounds = b;}

    /// Print the data bounds
    void PrintBounds() const
      {bounds.Print();}

    /// Create an image
    /** Conversion of the laser data to an image.
        @param pixelsize Size of the pixels in meter
        @param viff_type Either VFF_TYP_1_BYTE or VFF_TYP_FLOAT
        @param type HeightData, ReflectanceData, ColourData, PulseLengthData,
                    or PointCountData
        @param int_method Interpolation method:
                    1 - Nearest neighbour within a pixel (also called binning),
                    2 - Barycentric coordinates,
                    3 - Planar interpolation in TIN mesh,
                    4 - Minimum value within a pixel
        @param max_mesh_size Maximum mesh size to interpolate
                             (relevant for method 2 and 3 only)
        @return The rasterised laser data
    */
    Image *CreateImage(double pixelsize, int viff_type,
                       ImagedDataType data_type, int int_method,
                       double max_mesh_size);

    /// Add laser data to an image
    /** Conversion of the laser data to an image.
        This is a virtual function. It should be called through one of the
        classes LaserBlock, LaserUnit, LaserSubUnit or LaserPoints.
        @param image The rasterised laser data
        @param dist_image Image to keep track of the nearest laser points
        @param grid The image grid specification
        @param type HeightData, ReflectanceData, ColourData, PulseLengthData,
                    or PointCountData
        @param int_method Interpolation method:
                    1 - Nearest neighbour within a pixel (also called binning),
                    2 - Barycentric coordinates,
                    3 - Planar interpolation in TIN mesh,
                    4 - Minimum value within a pixel
        @param max_mesh_size Maximum mesh size to interpolate
                             (relevant for method 2 and 3 only)
    */
    virtual void ImageData(Image &image, Image &dist_image,
                           const DataGrid &grid, ImagedDataType type,
                           int int_method, double max_mesh_size);

    /// Return the imaged height data
    ImagedData *ImagedHeight()
      {return(&imaged_height);}

    /// Create a height image
    /** Call to CreateImage with data type HeightData */
    Image *CreateHeightImage(double pixelsize, int viff_type,
                             int method, double max_mesh_size)
      {return(CreateImage(pixelsize, viff_type, HeightData, method,
                          max_mesh_size));}
 
    /// Read the imaged height data
    bool ReadImagedHeight()
      {return imaged_height.Read();}
    
    /// Write the imaged height data
    bool WriteImagedHeight() const
      {return imaged_height.Write();}

    /// Derive the file name of the height image
    char *DeriveHeightImageFileName(const char *rootname,
                                    const char *directory);

    /// Derive the file name of the height grid definition
    char *DeriveHeightGridFileName(const char *rootname, const char *directory);

    /// Derive the file name of the height grid definition
    char *DeriveHeightGridFileName();

    /// Return the imaged reflectance data
    ImagedData *ImagedReflectance()
      {return(&imaged_reflectance);}

    /// Create a reflectance image
    /** Call to CreateImage with data type ReflectanceData */
    Image *CreateReflectanceImage(double pixelsize, int viff_type, int method,
                                  double max_mesh_size)
      {return(CreateImage(pixelsize, viff_type, ReflectanceData, method,
                          max_mesh_size));}
 
    /// Read the imaged reflectance data
    void ReadImagedReflectance()
      {imaged_reflectance.Read();}

    /// Write the imaged reflectance data
    void WriteImagedReflectance() const
      {imaged_reflectance.Write();}

    /// Derive the file name of the reflectance image
    char *DeriveReflectanceImageFileName(const char *rootname,
                                         const char *directory);

    /// Derive the file name of the reflectance grid definition
    char *DeriveReflectanceGridFileName(const char *rootname,
                                        const char *directory);

    /// Derive the file name of the reflectance grid definition
    char *DeriveReflectanceGridFileName();

    /// Return the imaged colour data
    ImagedData *ImagedColour()
      {return(&imaged_colour);}

    /// Create a colour image
    /** Call to CreateImage with data type ColourData */
    Image *CreateColourImage(double pixelsize, int viff_type, int method,
                             double max_mesh_size)
      {return(CreateImage(pixelsize, viff_type, ColourData, method,
                          max_mesh_size));}
 
    /// Read the imaged colour data
    void ReadImagedColour()
      {imaged_colour.Read();}

    /// Write the imaged colour data
    void WriteImagedColour() const
      {imaged_colour.Write();}

    /// Derive the file name of the colour image
    char *DeriveColourImageFileName(const char *rootname,
                                    const char *directory);

    /// Derive the file name of the colour grid definition
    char *DeriveColourGridFileName(const char *rootname, const char *directory);

    /// Derive the file name of the colour grid definition
    char *DeriveColourGridFileName();

    /// Return the imaged pulse length data
    ImagedData *ImagedPulseLength()
      {return(&imaged_pulselength);}

    /// Create a pulse length image
    /** Call to CreateImage with data type PulseLengthData */
    Image *CreatePulseLengthImage(double pixelsize, int viff_type, int method,
                                  double max_mesh_size)
      {return(CreateImage(pixelsize, viff_type, PulseLengthData, method,
                          max_mesh_size));}
 
    /// Read the imaged pulse length data
    void ReadImagedPulseLength()
      {imaged_pulselength.Read();}

    /// Write the imaged pulse length data
    void WriteImagedPulseLength() const
      {imaged_pulselength.Write();}

    /// Derive the file name of the pulse length image
    char *DerivePulseLengthImageFileName(const char *rootname,
                                         const char *directory);

    /// Derive the file name of the pulse length grid definition
    char *DerivePulseLengthGridFileName(const char *rootname,
                                        const char *directory);

    /// Derive the file name of the pulse length grid definition
    char *DerivePulseLengthGridFileName();

    /// Return the imaged pulse length data
    ImagedData *ImagedPointCount()
      {return(&imaged_pointcount);}

    /// Create a point count image
    /** Call to CreateImage with data type PointCountData */
    Image *CreatePointCountImage(double pixelsize, int viff_type, int method,
                                 double max_mesh_size)
      {return(CreateImage(pixelsize, viff_type, PointCountData, method,
                          max_mesh_size));}
 
    /// Read the imaged point count data
    void ReadImagedPointCount()
      {imaged_pointcount.Read();}

    /// Write the imaged point count data
    void WriteImagedPointCount() const
      {imaged_pointcount.Write();}

    /// Derive the file name of the point count image
    char *DerivePointCountImageFileName(const char *rootname,
                                        const char *directory);

    /// Derive the file name of the point count grid definition
    char *DerivePointCountGridFileName(const char *rootname,
                                       const char *directory);

    /// Derive the file name of the pulse length grid definition
    char *DerivePointCountGridFileName();

    /// Return the point density
    double PointDensity() const
      {return(point_density);}

    /// Set the point density
    void SetPointDensity(double pd)
      {point_density = pd;}

    /// Return the boundary file name
    char *BoundaryFile() const
      {return(boundary_file);}

    /// Set the boundary file name
    void SetBoundaryFile(const char *filename);

    /// Return the data set boundary
    Positions2D *Boundary() const
      {return(boundary);}

    /// Read the data set boundary from a file
    Positions2D *ReadBoundary(const char *filename);

    /// Read the data set boundary from "boundary_file"
    Positions2D *ReadBoundary();

    /// Check if some data is known
    int HasSomeData() const;
};

#endif /* _LaserPointsInfo_h_ */  /* Don't add after this point */
