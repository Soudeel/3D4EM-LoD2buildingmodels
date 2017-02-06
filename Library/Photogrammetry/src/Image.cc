
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class Image

 Initial creation
 Author : George Vosselman
 Date   : 01-04-1999

 Update #1
 Author : George Vosselman
 Date   : 02-07-1999
 Changes: Added pointers for an integer image

 Update #2
 Author : George Vosselman
 Date   : 24-12-1999
 Changes: Added support for location data and range determination

 Update #3
 Author : Ildiko Suveg
 Date   : 20-03-2001
 Changes: Destructor

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "digphot_arch.h"
#ifdef windows
#  include <limits.h>
#endif
#include "Image.h"
#include "viff.h"
#include "stdmath.h"
#include "Viff4Cpp.h"

extern "C" int f_resample(float *image, double r, double c, int nrow, int ncol,
                          float *pixel, int method);
/*
--------------------------------------------------------------------------------
                       Image constructors and destructors
--------------------------------------------------------------------------------
*/

Image::Image()
{
  Initialise();
}

Image::Image(const char *filename, int *success)
{
  Initialise();
  *success = Read(filename);
}

Image::Image(int num_rows, int num_cols)
{
  Initialise();
  image = NewImage(num_rows, num_cols, VFF_TYP_1_BYTE);
}

Image::Image(int num_rows, int num_cols, int data_type)
{
  Initialise();
  image = NewImage(num_rows, num_cols, data_type);
}

Image::Image(double xrange, double yrange, double pixelsize)
{
  Initialise();
  image = NewImage(xrange, yrange, pixelsize); 
}

Image::~Image()
{
  DeleteImage();
  Initialise();
}

xvimage *Image::NewImage(int num_rows, int num_cols, int data_type,
                         int num_bands)
{
  if (image) DeleteImage();
  image = createimage(num_rows, num_cols, data_type, 1, num_bands, " ", 0, 0,
                      VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
  if (!image)
    fprintf(stderr, "Error allocating memory for image of size %d x %d pixels.\n",
            num_rows, num_cols);
  return(image);
}

xvimage *Image::NewImage(double xrange, double yrange, double pixelsize)
{
  int num_rows, num_cols;

  num_rows = (int) (yrange / pixelsize);
  if (num_rows < yrange / pixelsize) num_rows++;
  num_cols = (int) (xrange / pixelsize);
  if (num_cols < xrange / pixelsize) num_cols++;
  return(NewImage(num_rows, num_cols, VFF_TYP_1_BYTE));
}

void Image::Initialise()
{
  image = NULL;
  image_file = NULL;
}

void Image::DeleteImage()
{
  if (image) {(void) freeimage(image); image = NULL;}
}

/*
--------------------------------------------------------------------------------
                          Set all pixel value to zero
--------------------------------------------------------------------------------
*/

void Image::ClearImage()
{
  int bytes_per_pixel;

/* Determine the number of bytes per pixel */

  switch (image->data_storage_type) {
    case VFF_TYP_1_BYTE   : bytes_per_pixel = 1;  break;
    case VFF_TYP_2_BYTE   : bytes_per_pixel = 2;  break;
    case VFF_TYP_4_BYTE   : 
    case VFF_TYP_FLOAT    : bytes_per_pixel = 4;  break;
    case VFF_TYP_COMPLEX  : 
    case VFF_TYP_DOUBLE   : bytes_per_pixel = 8;  break;
    case VFF_TYP_DCOMPLEX : bytes_per_pixel = 16;  break;
    default: fprintf(stderr, "Error: Unknown data storage type.\n");
             fprintf(stderr, "       Unable to clear the image.\n");
             exit(0);
  }

/* Clear the image */

  memset((void *) image->imagedata, 0,
         image->col_size * image->row_size * bytes_per_pixel *
         image->num_data_bands * image->num_of_images);
}


/*
--------------------------------------------------------------------------------
                    Set all pixel value to some number
--------------------------------------------------------------------------------
*/

void Image::SetPixels(unsigned char greyvalue)
{
  unsigned char *pixel;

  if (image->data_storage_type != VFF_TYP_1_BYTE) {
    fprintf(stderr, "Image::SetPixels(unsigned char) is only valid for images of type VFF_TYP_1_BYTE.\n");
    exit(0);
  }

  for (pixel=begin(); pixel!=end(); pixel++) *pixel=greyvalue;
}

void Image::SetPixels(float greyvalue)
{
  float *pixel;

  if (image->data_storage_type != VFF_TYP_FLOAT) {
    fprintf(stderr, "Image::SetPixels(unsigned char) is only valid for images of type VFF_TYP_FLOAT.\n");
    exit(0);
  }

  for (pixel=float_begin(); pixel!=float_end(); pixel++) *pixel=greyvalue;
}

/*
--------------------------------------------------------------------------------
                          Set the image elements and name
--------------------------------------------------------------------------------
*/

void Image::SetImage(xvimage *newimage)
{
  if (image) DeleteImage();
  image = newimage;
}

void Image::SetImageData(char *data)
{
  if (image) image->imagedata = data;
  else fprintf(stderr, "Error: no image for storing the data\n");
}

void Image::SetImageSize(int num_rows, int num_columns)
{
  if (image) {
    image->col_size = num_rows;
    image->row_size = num_columns;
  }
  else fprintf(stderr, "Error: no image for storing the image sizes\n");
}

void Image::SetImageFile(const char *filename)
{
  if (filename) {
    image_file = (char *) realloc(image_file, strlen(filename) + 1);
    strcpy(image_file, filename);
  }
  else {
    if (image_file) free(image_file);
    image_file = NULL;
  }
}

/*
--------------------------------------------------------------------------------
                          Read an image from a file
--------------------------------------------------------------------------------
*/

int Image::Read(const char *filename)
{
  if (filename != image_file) SetImageFile(filename);
  if (image) DeleteImage();
  image = readimage(filename);
  if (!image) {
    fprintf(stderr, "Error reading image %s\n", filename);
    return(0);
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Write an image to a file
--------------------------------------------------------------------------------
*/

int Image::Write(const char *filename) const
{
  return(writeimage(filename, image));
}

/*
--------------------------------------------------------------------------------
                              Pixel pointers
--------------------------------------------------------------------------------
*/

unsigned char *Image::begin()
{
  return((unsigned char *) image->imagedata);
}

const unsigned char *Image::begin() const
{
  return((const unsigned char *) image->imagedata);
}

unsigned char *Image::end()
{
  return((unsigned char *) (image->imagedata +
	 image->row_size * image->col_size * image->num_data_bands));
}

const unsigned char *Image::end() const
{
  return((const unsigned char *) (image->imagedata +
	 image->row_size * image->col_size * image->num_data_bands));
}

unsigned char *Image::Pixel(int row, int column)
{
  return(begin() + row * image->row_size + column);
}

const unsigned char *Image::Pixel(int row, int column) const
{
  return(begin() + row * image->row_size + column);
}

int *Image::int_begin()
{
  return((int *) image->imagedata);
}

const int *Image::int_begin() const
{
  return((int *) image->imagedata);
}

int *Image::int_end()
{
  return((int *) (image->imagedata + sizeof(int) *
	 image->row_size * image->col_size * image->num_data_bands));
}

const int *Image::int_end() const
{
  return((int *) (image->imagedata + sizeof(int) *
	 image->row_size * image->col_size * image->num_data_bands));
}

int *Image::IntPixel(int row, int column)
{
  return(int_begin() + row * image->row_size + column);
}

const int *Image::IntPixel(int row, int column) const
{
  return(int_begin() + row * image->row_size + column);
}

float *Image::float_begin()
{
  return((float *) image->imagedata);
}

const float *Image::float_begin() const
{
  return((float *) image->imagedata);
}

float *Image::float_end()
{
  return((float *) (image->imagedata + sizeof(float) *
	 image->row_size * image->col_size * image->num_data_bands));
}

const float *Image::float_end() const
{
  return((float *) (image->imagedata + sizeof(float) *
	 image->row_size * image->col_size * image->num_data_bands));
}

float *Image::FloatPixel(int row, int column)
{
  return(float_begin() + row * image->row_size + column);
}

const float *Image::FloatPixel(int row, int column) const
{
  return(float_begin() + row * image->row_size + column);
}

/*
--------------------------------------------------------------------------------
                              Location data
--------------------------------------------------------------------------------
*/

void Image::Set1DLocationData(double xbegin, double xend)
{
  image->fspare1 = (float) xbegin;
  image->fspare2 = (float) xend;
  image->location_type = VFF_LOC_EXPLICIT;
  image->location_dim  = 3;
}

void Image::Get1DLocationData(double *xbegin, double *xend) const
{
  *xbegin = (double) image->fspare1;
  *xend   = (double) image->fspare2;
}

void Image::Set2DLocationData(double x, double y)
{
  image->fspare1 = (float) x;
  image->fspare2 = (float) y;
  image->location_type = VFF_LOC_EXPLICIT;
  image->location_dim  = 3;
}

void Image::Get2DLocationData(double *xbegin, double *xend,
                              double *ybegin, double *yend) const
{
  if (image->fspare1 > 0.0) {
    *xbegin = 0.0;
    *xend   = (double) image->fspare1;
  }
  else {
    *xbegin = (double) image->fspare1;
    *xend   = -(*xbegin);
  }
  if (image->fspare2 > 0.0) {
    *ybegin = 0.0;
    *yend   = (double) image->fspare2;
  }
  else {
    *ybegin = (double) image->fspare2;
    *yend   = -(*ybegin);
  }
}

/*
--------------------------------------------------------------------------------
                              Value ranges
--------------------------------------------------------------------------------
*/

void Image::ValueRange(unsigned char *minval, unsigned char *maxval) const
{
  unsigned char *pixel;
  int           numpix, i;

  *minval = 255;
  *maxval = 0;
  numpix = image->col_size * image->row_size;
  for (i=0, pixel=(unsigned char *) image->imagedata; i<numpix; i++, pixel++) {
    if (*pixel < *minval) *minval = *pixel;
    if (*pixel > *maxval) *maxval = *pixel;
  }
}

void Image::IntValueRange(int *minval, int *maxval) const
{
  int *pixel, numpix, i;

#ifdef windows
  *minval = INT_MAX;
  *maxval = INT_MIN;
#else
  *minval = MAXINT;
  *maxval = -MAXINT;
#endif
  numpix = image->col_size * image->row_size;
  for (i=0, pixel=(int *) image->imagedata; i<numpix; i++, pixel++) {
    if (*pixel < *minval) *minval = *pixel;
    if (*pixel > *maxval) *maxval = *pixel;
  }
}

void Image::FloatValueRange(float *minval, float *maxval) const
{
  float *pixel;
  int   numpix, i;

#ifdef windows
  *minval = 1e30;
  *maxval = -1e30;
#else
  *minval = MAXFLOAT;
  *maxval = -MAXFLOAT;
#endif
  numpix = image->col_size * image->row_size;
  for (i=0, pixel=(float *) image->imagedata; i<numpix; i++, pixel++) {
    if (*pixel < *minval) *minval = *pixel;
    if (*pixel > *maxval) *maxval = *pixel;
  }
}

/*
--------------------------------------------------------------------------------
                           Create a shaded image
--------------------------------------------------------------------------------
*/

Image * Image::ShadedImage() const
{
  unsigned char       *grey1;
  const unsigned char *greyrow, *grey1c, *grey2c;
  float               *grad, maxgrad;
  int                 ir, ic;
  Image               *shaded;

/* Create a FLOAT image */

  shaded = new Image(NumRows()-1, NumColumns()-1, VFF_TYP_FLOAT);
  if (!shaded) {
    fprintf(stderr, "Error allocating float image space\n");
    exit(0);
  }

/* Calculate the diagonal gradients */

  for (ir=0, greyrow=begin(), grad=shaded->float_begin();
       ir<shaded->NumRows(); ir++, greyrow+=NumColumns())
    for (ic=0, grey1c=greyrow, grey2c=grey1c+NumColumns()+1;
         ic<shaded->NumColumns(); ic++, grad++, grey1c++, grey2c++)
      *grad = (float) *grey2c - (float) *grey1c;
                     
/* Take the signed logarithm of the absolute gradient plus 1 */

  maxgrad = 0;
  for (ir=0, grad=shaded->float_begin(); ir<shaded->NumRows(); ir++) {
    for (ic=0; ic<shaded->NumColumns(); ic++, grad++) {
      if (*grad >= 0) {
        *grad = (float) log((double) *grad + 1);
        if (*grad > maxgrad) maxgrad = *grad;
      }
      else {
        *grad = -1 * (float) log(ABS((double) *grad) + 1);
        if (*grad < -maxgrad) maxgrad = -1 * (*grad);
      }
    }
  }

/* Normalise the data and store it in unsigned bytes */

  for (ir=0, grad=(float *) shaded->float_begin(), grey1=shaded->begin();
       ir<shaded->NumRows(); ir++) {
    for (ic=0; ic<shaded->NumColumns(); ic++, grad++, grey1++)
      *grey1 = (unsigned char) (*grad * 127.5 / maxgrad + 127.5);
    *grey1++ = *(grey1-1); /* Duplicate the last column */
  }
  for (ic=0; ic<=shaded->NumColumns(); ic++, grey1++)
    *grey1 = *(grey1 - shaded->NumColumns() - 1); /* Duplicate the last row */

/* Change the image size and data storage type */

  shaded->GetImage()->col_size++;
  shaded->GetImage()->row_size++;
  shaded->GetImage()->data_storage_type = VFF_TYP_1_BYTE;

  return shaded;
}

/*
--------------------------------------------------------------------------------
                           Convert to OpenGL format
--------------------------------------------------------------------------------
*/

Image * Image::ConvertToOpenGL() const
{
  Image               *opengl_image;
  unsigned char       *opengl_pix, *opengl_row;
  const unsigned char *pix, *colour;
  int                 r, c, i, numpix, num_rows, num_cols;

  // Determine rows and columns of the texture image
  num_rows = 2;
  while (num_rows < NumRows()) num_rows *= 2;
  num_cols = 2;
  while (num_cols < NumColumns()) num_cols *= 2;
  if (num_rows > 2048 || num_cols > 2048) {
    printf("Warning: Maximum OpenGL texture image size (2048x2048) exceeded (%dx%d)\n",
           NumRows(), NumColumns());
    printf("         OpenGL texture image will be truncated\n");
    if (num_rows > 2048) num_rows = 2048;
    if (num_cols > 2048) num_cols = 2048;
  }

  // Create a 3 band image
  opengl_image = new Image();
  opengl_image->NewImage(num_rows, num_cols, VFF_TYP_1_BYTE, 3);
  opengl_image->ClearImage();
  opengl_row = (unsigned char *) opengl_image->GetImage()->imagedata +
               (num_rows - MIN(2048, NumRows())) * num_cols * 3;

  // Put the grey level data into the OpenGL structure (revert rows)
  if (NumBands() == 1) {
    for (r=MIN(2048, NumRows())-1; r>=0; r--, opengl_row += num_cols * 3) {
      for (c=0, pix=(const unsigned char *) begin()+r*NumColumns(),
           opengl_pix = opengl_row;
           c<MIN(2048, NumColumns()); c++, pix++)  {
        *opengl_pix++ = *pix; *opengl_pix++ = *pix; *opengl_pix++ = *pix;
      }
    }
  }

  // Put colour data into the OpenGL structure (revert rows and group colours)
  else if (NumBands() == 3) {
    numpix = NumRows() * NumColumns();
    for (r=MIN(2048, NumRows())-1; r>=0; r--, opengl_row += num_cols * 3) {
      for (c=0, pix=(const unsigned char *) begin()+r*NumColumns(),
           opengl_pix = opengl_row;
           c<MIN(2048, NumColumns()); c++, pix++)  {
        for (i=0, colour=pix; i<3; i++, colour+=numpix) *opengl_pix++ = *colour;
      }
    }
  }
  else {
    printf("Error: Can not convert images with %d bands\n", NumBands());
    return NULL;
  }
  return opengl_image;
}

/*
--------------------------------------------------------------------------------
                    Check if a pixel is inside the image
--------------------------------------------------------------------------------
*/

bool Image::Inside(int row, int column) const
{
  if (row < 0 || row >= NumRows()) return false;
  if (column < 0 || column >= NumColumns()) return false;
  return true;
}

/*
--------------------------------------------------------------------------------
                    Create kernel for filtering laser points
--------------------------------------------------------------------------------
*/

void Image::CreateFilterKernel(double hdif0,
                               double dist1, double hdif1,
                               double dist2, double hdif2,
                               double dist3, double hdif3,
                               double dist4, double hdif4,
                               double dist5, double hdif5,
                               double distmax, double hdifmax,
                               double distint)
{
  double *dist, *hdif;
  int    numdist, i, id, id1, id2;
  float  *pixel;

  dist = (double *) malloc(7 * sizeof(double));
  hdif = (double *) malloc(7 * sizeof(double));
  
  // Create the kernel image
  NewImage(1, (int) (distmax/distint) + 1, VFF_TYP_FLOAT);
  ClearImage();

  // Set the break points in an array
  dist[0] = 0;
  hdif[0] = hdif0; numdist = 0;
  if (dist1 >= 0.0) {numdist++;  dist[numdist] = dist1; hdif[numdist] = hdif1;}
  if (dist2 >= 0.0) {numdist++;  dist[numdist] = dist2; hdif[numdist] = hdif2;}
  if (dist3 >= 0.0) {numdist++;  dist[numdist] = dist3; hdif[numdist] = hdif3;}
  if (dist4 >= 0.0) {numdist++;  dist[numdist] = dist4; hdif[numdist] = hdif4;}
  if (dist5 >= 0.0) {numdist++;  dist[numdist] = dist5; hdif[numdist] = hdif5;}
  numdist++; dist[numdist] = distmax; hdif[numdist] = hdifmax;

  // Verify that distances and height differences are monotonously increasing
  for (i=0; i<numdist; i++) {
    if (dist[i] > dist[i+1] || hdif[i] > hdif[i+1]) {
      fprintf(stderr, "Error: distances and differences should be monotonously increasing.\n");
      exit(0);
    }
  }
 
  // Set the kernel elements by linear interpolation
  for (i=0; i<numdist; i++) {
    id1 = (int) (dist[i] / distint);
    id2 = (int) (dist[i+1] / distint);
    if (id2 > id1) {
      for (id=id1, pixel=FloatPixel(0, id1); id<=id2; id++, pixel++)
        *pixel = (float) ((hdif[i] * (id2 - id) + hdif[i+1] * (id - id1)) /
                          (id2 - id1));
    }
    else {
      pixel = FloatPixel(0, id1);
      *pixel = (float) hdif[i];
    }
  }

  // Set the range of the filter data
  Set1DLocationData(dist[0], dist[numdist]);

  // Clear allocations
  free(dist);
  free(hdif);
}

/*
--------------------------------------------------------------------------------
                    Return the number of non-zero pixels
--------------------------------------------------------------------------------
*/

int Image::NumberNonZeroPixels() const
{
  int count=0;
  const unsigned char *pixel;
  
  for (pixel=begin(); pixel!=end(); pixel++)
    if (*pixel) count++;
  return count;
}

/*
--------------------------------------------------------------------------------
                  Return RGB values of a colour image pixel
--------------------------------------------------------------------------------
*/
void Image::Colour(int row, int column, int &red, int &green, int &blue) const
{
  const unsigned char *pixel = Pixel(row, column);
  red = *pixel;
  pixel += NumRows() * NumColumns();
  green = *pixel;
  pixel += NumRows() * NumColumns();
  blue = *pixel;
}

/*
--------------------------------------------------------------------------------
                  Reduce the image size by subsampling
--------------------------------------------------------------------------------
*/
void Image::Reduce(int factor)
{
  int new_num_rows=NumRows()/factor, new_num_columns=NumColumns()/factor,
      band, row, column, image_size;
  unsigned char *pixel, *new_pixel;
  
  new_pixel = Pixel(0, 0);
  image_size = NumRows() * NumColumns();
  for (band=0; band<image->num_data_bands; band++) {
    for (row=0; row<new_num_rows*factor; row+=factor) {
      pixel = Pixel(row, 0) + band * image_size;
      for (column=0; column<new_num_columns*factor;
           column+=factor, pixel+=factor, new_pixel++) {
        *new_pixel = *pixel;
      }
    }
  }
  SetImageSize(new_num_rows, new_num_columns);
}

/*
--------------------------------------------------------------------------------
                  Bilinear interpolation in float image
--------------------------------------------------------------------------------
*/ 
float Image::InterpolateFloat(double r, double c, int method)
{
  float value;
  if (f_resample(float_begin(), r, c, NumRows(), NumColumns(), &value, method))
    return value;
  return 0.0;
}
