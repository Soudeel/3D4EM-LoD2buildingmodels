
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
 * \brief Interface to Class Image - Class for a Khoros xvimage
 *
 */
/*!
 * \class Image
 * \ingroup Photogrammetry
 * \brief Interface to Class Image - Class for a Khoros xvimage
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


#ifndef _Image_h_
#define _Image_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Image                          -  Class for a Khoros xvimage

--------------------------------------------------------------------------------
*/

#include "viff.h"

//------------------------------------------------------------------------------
/// An image
//------------------------------------------------------------------------------

class Image {

  protected:

    /// Khoros 1.0 VIFF image
    xvimage *image;

    /// Image file name
    char *image_file;

  public:

    /// Default constructor
    Image();

    /// Construct by reading an image file
    Image(const char *, int *);

    /// Construct a new unsigned char image
    Image(int num_rows, int num_columns);

    /// Construct a new image of arbitrary data storage type
    /** Consult the include file viff.h for valid data storage types */
    Image(int num_rows, int num_columns, int data_storage_type);

    /// Construct a new unsigned char image based on coordinate ranges and pixel size
    /** The number of rows and columns are derived from the coordinate ranges
        devided by the specified pixel size. Non-integer values are rounded up.
    */
    Image(double x_range, double y_range, double pixel_size);

    /// Default destructor
    ~Image();

    /// Create a new image of arbitrary data storage type
    /** Consult the include file viff.h for valid data storage types
        @return Pointer to the created xvimage structure */
    xvimage *NewImage(int num_rows, int num_columns, int data_storage_type,
                      int num_bands=1);

    /// Create a new unsigned char image based on coordinate ranges and pixel size
    /** The number of rows and columns are derived from the coordinate ranges
        devided by the specified pixel size. Non-integer values are rounded up.
    */
    xvimage *NewImage(double x_range, double y_range, double pixel_size);

    /// Initialisation
    void Initialise();

    /// Free all image memory
    void DeleteImage();

    /// Set all pixel values to zero
    void ClearImage();

    /// Set all pixel values to some unsigned char value
    /** Only use this function for images of type VFF_TYP_1_BYTE */
    void SetPixels(unsigned char value);

    /// Set all pixel values to some float value
    /** Only use this function for images of type VFF_TYP_FLOAT */
    void SetPixels(float);

    /// Return the writable reference
    Image &ImageReference()
      {return(*this);}

    /// Return the readable reference
    const Image &ImageReference() const
      {return(*this);}

    /// Return the pointer to the Khoros VIFF image structure
    xvimage *GetImage() const
      {return(image);}

    /// Return the image file name
    char *ImageFile() const
      {return(image_file);}

    /// Set the Khoros VIFF image
    void SetImage(xvimage *);

    /// Put new image data into the Khoros VIFF image structure
    /** @param image_data char pointer to the new image data */
    void SetImageData(char *image_data);

    /// Set the image sizes
    void SetImageSize(int num_rows, int num_columns);

    /// Set the image file name
    void SetImageFile(const char *file);

    /// Read the image from a file
    int Read(const char *filename);

    /// Read the image from the already set image file name
    int Read()
      {return(Read(image_file));}

    /// Write the image to a file
    int Write(const char *filename) const;

    /// Write the image to the already set image file name
    int Write() const
      {return(Write(image_file));}

    /// Return the number of rows
    int NumRows() const
      {return(image->col_size);}

    /// Return the number of columns
    int NumColumns() const
      {return(image->row_size);}

    /// Return the number of bands
    int NumBands() const
      {return(image->num_data_bands);}

    /// Return the image data type
    /** Consult the include file viff.h for valid data storage types */
    int DataType() const
      {return(image->data_storage_type);}

    /// Return the pointer to the unsigned char first pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    unsigned char *begin();

    /// Return the pointer to the unsigned char first pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const unsigned char *begin() const;

    /// Return the pointer to the last unsigned char pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    unsigned char *end();

    /// Return the pointer to the last unsigned char pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const unsigned char *end() const;

    /// Return the pointer to a specific pixel
    unsigned char *Pixel(int row, int column);

    /// Return the pointer to a specific pixel
    const unsigned char *Pixel(int row, int column) const;

    /// Return the pointer to the first int pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    int *int_begin();

    /// Return the pointer to the first int pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const int *int_begin() const;

    /// Return the pointer to the last int pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    int *int_end();

    /// Return the pointer to the last int pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const int *int_end() const;

    /// Return the pointer to a specific int pixel
    int *IntPixel(int row, int column);

    /// Return the pointer to a specific int pixel
    const int *IntPixel(int row, int column) const;

    /// Return the pointer to the first float pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    float *float_begin();

    /// Return the pointer to the first float pixel
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const float *float_begin() const;

    /// Return the pointer to the last float pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    float *float_end();

    /// Return the pointer to the last float pixel plus one
    /** This function allows looping over the image pixels as if they were
        stored in an STL vector
    */
    const float *float_end() const;

    /// Return the pointer to a specific float pixel
    float *FloatPixel(int row, int column);

    /// Return the pointer to a specific float pixel
    const float *FloatPixel(int row, int column) const;

    /// Set the location data for a one dimensional image
    void Set1DLocationData
      (double minimum_x, double maximum_x);

    /// Get the location data for a one dimensional image
    void Get1DLocationData
      (double *minimum_x, double *maximum_x) const;

    /// Set the location data for a two dimensional image
    /** Only two float values are available for storing the location data.
        For two-dimensional location data the following encoding is used: If x
        is positive, the coordinates are considered to range from 0 to x.
        Otherwise, the coordinates are considered to range from x to -x.
        The same holds for the y-coordinate.
    */
    void Set2DLocationData
      (double x, double y);

    /// Get the location data for a two dimensional image
    void Get2DLocationData
      (double *minimum_x, double *maximum_x,
       double *minimum_y, double *maximum_y) const;

    /// Determine the range of the grey values in an unsigned char image
    void ValueRange (unsigned char *minimum, unsigned char *maximum) const;

    /// Determine the range of the grey values in an int image
    void IntValueRange (int *minimum, int *maximum) const;

    /// Determine the range of the grey values in a float image
    void FloatValueRange (float *minimum, float *maximum) const;

    /// Create a shaded relief image
    Image * ShadedImage() const;

    /// Convert to an image in OpenGL format
    Image * ConvertToOpenGL() const;
    
    /// Check if a pixel location is inside an image
    bool Inside(int row, int column) const;
    
    /// Create kernel for filtering laser scanning data
    /**
      @param hdif0 - Allowed height difference at distance 0
      @param dist1 - 1st distance with specified allowed height difference
      @param hdif1 - Allowed height difference at distance dist1
      @param distmax - Maximum distance of filter kernel
      @param hdifmax - Allowed height difference at distance distmax
      @param distint - Distance discretisation interval
    */
    void CreateFilterKernel(double hdif0,
                            double dist1, double hdif1,
                            double dist2, double hdif2,
                            double dist3, double hdif3,
                            double dist4, double hdif4,
                            double dist5, double hdif5,
                            double distmax, double hdifmax,
                            double distint);
    
    /// Return the number of non-zero pixels
    int NumberNonZeroPixels() const;
    
    /// Return RGB values of a colour image pixel
    void Colour(int row, int column, int &red, int &green, int &blue) const;
    
    /// Reduce the image size by subsampling
    void Reduce(int factor);
    
    /// Interpolation in float image
    /** @param r Row coordinate of interpolation location 
        @param c Column coordinate of interpolation location
        @param method Interpolation method (1=nearest neighbour, 2=bilinear,
                      3=cubic)
        @return Interpolated value
	*/
    float InterpolateFloat(double r, double c, int method);
};
#endif /* _Image_h_ */   /* Do NOT add anything after this line */
