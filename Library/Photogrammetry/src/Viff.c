
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



/* DONE (#1#): change "char" to "const char" whereever possible */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "viff.h"

/*
--------------------------------------------------------------------------------
 
 Collection of Khoros Viff I/O (KVIO) routines for reading and writing
 Khoros 1.0 and 2.0 formatted images without making use of Khoros libraries.
 All data is stored in the Khoros 1.0 image structure xvimage.

 Author: George Vosselman
 Date  : 27-Dec-2002

 Routines for internal usage only:
   KVIO_image_file_format - Determination of Khoros file format
   KVIO1_readimageheader  - Read image header of Khoros 1.0 image
   KVIO2_read_string      - Read a string of a Khoros 2.0 image header
   KVIO2_read_integers    - Read integers of a Khoros 2.0 image header
   KVIO2_readimageheader  - Read image header of Khoros 2.0 image
   KVIO_readimageheader   - Determine file format and read image header
   KVIO_pixel_size        - Determine the pixel size in bytes
   KVIO_image_size        - Determine the image size in bytes
   KVIO_map_size          - Determine the map size in bytes
   KVIO_location_size     - Determine the location size in bytes
   KVIO_readimagepart     - Read (parts of) the image data
   KVIO1_readimagedata    - Read map, location and image data of K 1.0 image
   KVIO2_transpose_maps   - Transpose maps
   KVIO2_uniform2location - Convert uniform to curvilinear location data
   KVIO2_readimagedata    - Read map, location and image data of K 2.0 image

 Routines for public usage with definitions according to Khoros 1.0:

   xvimage *createimage(unsigned long col_size, unsigned long row_size,
                        unsigned long data_storage_type,
                        unsigned long num_of_images,
                        unsigned long num_data_bands, char *comment,
                        unsigned long map_row_size, unsigned long map_col_size,
                        unsigned long map_scheme,
                        unsigned long map_storage_type,
                        unsigned long location_type, unsigned long location_dim)
     - Create an image structure and allocate data fields

   xvimage *readimageheader(char *filename)
     - Read the image header and return information in xvimage structure

   xvimage *readimage(char *filename)
     - Read the image and return the xvimage structure

   xvimage *readimagepart(char *filename, int row_offset, int col_offset,
                          int num_rows, int num_cols)
     - Read an image part and return the xvimage structure

   int writeimage(char *filename, xvimage *image)
     - Write image to a file (in Khoros 1.0 image format)

   int propertype(char *prog, xvimage *image, unsigned long type, int exit_flag)
     - Check image data storage type
   int proper_num_images(char *prog, xvimage *image, unsigned long type,
                         int exit_flag)
     - Check the number of images
   int proper_num_bands(char *prog, xvimage *image, unsigned long type,
                        int exit_flag)
     - Check the number of data bands
   int proper_map_enable(char *prog, xvimage *image, unsigned long type,
                         int exit_flag)
     - Check the enabling of the map
   int proper_map_type(char *prog, xvimage *image, unsigned long type,
                       int exit_flag)
     - Check the map data storage type
   int proper_map_scheme(char *prog, xvimage *image, unsigned long type,
                         int exit_flag)
     - Check the map scheme
   int proper_color_model(char *prog, xvimage *image, unsigned long type,
                          int exit_flag)
     - Check the colour model

   int freeimage(xvimage *image)
     - Free all allocated image memory

   void copyheader(xvimage *image_src, xvimage *image_dest)
     - Copy all header information, not the actual image, map or location data

   int imagesize(xvimage *image, int *dsize, int *dcount,
                 int *msize, int *mcount, int *lsize, int *lcount)
     - Return sizes of image, map and location data fields

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                          Constant definitions
--------------------------------------------------------------------------------
*/

#define XVIMAGE_FIXED_HEADER_SIZE 4096
#define KDBM_MAGIC_NUMBER 0x13579acf

#define KBYTE           (1L<<1)    /* Definitions of data types */
#define KUBYTE          (1L<<2)
#define KSHORT          (1L<<3)
#define KUSHORT         (1L<<4)
#define KINT            (1L<<5)
#define KUINT           (1L<<6)
#define KLONG           (1L<<7)
#define KULONG          (1L<<8)
#define KFLOAT          (1L<<9)
#define KDOUBLE         (1L<<10)
#define KCOMPLEX        (1L<<11)
#define KDCOMPLEX       (1L<<12)
#define KSTRING         (1L<<13)

#define KNONE            0
#define KGREY            1         /* Definitions of colour space models */
#define KRGB             2
#define KCMY             3
#define KYIQ             4
#define KHSV             5
#define KHLS             6
#define KIHS             7
#define KXYZ             8
#define KUVW             9
#define KUCSUVW         10
#define KUCSSOW         12
#define KUCSLab         13
#define KUCSLuv         14
#define KUSERDEFINED    15

/*
--------------------------------------------------------------------------------
   Determine whether the file contains a Khoros 1.0 or 2.0 formatted image
  
   Returns: 0 - File read error
            1 - Khoros 1.0 formatted image
            2 - Khoros 2.0 formatted image
            3 - Unrecognised format
--------------------------------------------------------------------------------
*/

int KVIO_image_file_format(FILE *fd)
{
  unsigned char header[5];
  int           nbytes, magic_number;

  nbytes = fread(header, sizeof(char), 5, fd);
  if (nbytes != 5) return(0);
  if (fseek(fd, 0, SEEK_SET)) return(0); /* Reposition at file begin */

/* Check for Khoros 1.0 image file format */

  if (header[0] == XV_FILE_MAGIC_NUM) {
    if (header[1] != XV_FILE_TYPE_XVIFF) {
      printf("Magic number corresponds to XV_FILE_MAGIC_NUM, but\n");
      printf("image type (%d) is unexpected, expected %d\n",
             header[1], XV_FILE_TYPE_XVIFF);
      return(3);
    }
    if (header[2] != XV_IMAGE_REL_NUM) {
      printf("Magic number corresponds to XV_FILE_MAGIC_NUM, but\n");
      printf("release number (%d) is unexpected, expected %d\n",
             header[2], XV_IMAGE_REL_NUM);
      return(3);
    }
    if (header[3] != XV_IMAGE_VER_NUM) {
      printf("Magic number corresponds to XV_FILE_MAGIC_NUM, but\n");
      printf("image version number (%d) is unexpected, expected %d\n",
             header[3], XV_IMAGE_VER_NUM);
      return(3);
    }
    return(1); /* Khoros 1.0 formatted image */
  }

/* Check for Khoros 2.0 image file format */

  if (header[0] == 1) {
    magic_number = * (int *) &header[1]; /* Next 4 bytes in an integer */
    if (magic_number == KDBM_MAGIC_NUMBER) return(2);
  }
  else if (header[0] == 8) {
    if (header[1] == 19 && header[2] == 87 && header[3] == 154 &&
        header[4] == 207) {
      fprintf(stderr, "Sorry, I can't read this VIFF image.\n");
      fprintf(stderr, "It was written on another machine that swaps bytes.\n");
      fprintf(stderr,
            "Use \"kformats -i <old_file> -o <new_file> -viff\" to convert.\n");
    }
  }

/* Unknown file type */

  return(3);
}

/*
--------------------------------------------------------------------------------
                 Read the header of a Khoros 1.0 image file
--------------------------------------------------------------------------------
*/
xvimage *KVIO1_readimageheader(FILE *fd)
{
  xvimage *image;
  int     nbytes;

  image = (xvimage *) calloc(1, sizeof(xvimage));
  nbytes = fread(image, sizeof(char), 1024, fd);
  if (nbytes != 1024) {
    free(image);
    return(NULL);
  }
  return(image);
}

/*
--------------------------------------------------------------------------------
                 Read the header of a Khoros 2.0 image file
--------------------------------------------------------------------------------
*/
int KVIO2_read_string(FILE *fd, char *string)
{
  char *ch, end_of_string;
  int  nbytes;

  ch = string;
  end_of_string = 0;
  while (!end_of_string) {
    nbytes = fread(ch, sizeof(char), 1, fd);
    if (nbytes != 1) return(0);
    end_of_string = (*ch == 0);
    ch++;
  }
  return(1);
}

int KVIO2_read_integers(FILE *fd, int *int_buffer, int num_int)
{
  int nread;

  nread = fread(int_buffer, sizeof(int), num_int, fd);
  if (nread != num_int) return(0);
  return(1);
}

/*
--------------------------------------------------------------------------------
                 Read the header of a Khoros 2.0 image file
--------------------------------------------------------------------------------
*/
xvimage *KVIO2_readimageheader(FILE *fd)
{
  xvimage *image;
  int     *ibuf, success, number_of_items, i;
  char    *token, *string;
  
/* Skip the first 4096 bytes */

  if (fseek(fd, XVIMAGE_FIXED_HEADER_SIZE, SEEK_SET)) return(NULL);

/* Allocate image structure and buffers */

  image  = (xvimage *) calloc(1, sizeof(xvimage));
  string = (char *) malloc(1024);
  token  = (char *) malloc(1024);
  ibuf   = (int *) malloc(10 * sizeof(int));

  success = 1;
  image->location_type = VFF_LOC_IMPLICIT;
  image->startx        = -1;
  image->starty        = -1;
  image->pixsizx       = 1.0;
  image->pixsizy       = 1.0;
  while (success && KVIO2_read_string(fd, token)) {
    if (strcmp(token, "value:") == 0) {
      break;
    }
    else if (strcmp(token, "map:") == 0) {
      printf("Maps are following, but image data was not found!\n");
      return(NULL);
    }
    else {
      success = KVIO2_read_integers(fd, &number_of_items, 1);
      if (success) {
        if (number_of_items != 1) {
          fprintf(stderr, "Error: multiple types of parameters to token %s\n",
                  token);
          fprintf(stderr,
                  "This feature of Khoros 2.0 format is not implemented.\n");
          success = 0;
        }
        else {
          success = KVIO2_read_integers(fd, ibuf, 2);
          if (success) {
            number_of_items = ibuf[0];
            switch (ibuf[1]) {
              case KINT: /* Read integers */
                success = KVIO2_read_integers(fd, ibuf, number_of_items);
/*
                printf("%s 1 %d 32", token, number_of_items);
                for (i=0; i<number_of_items; i++) printf(" %d", ibuf[i]);
                printf("\n");
*/
                if (strcmp(token, "value:size") == 0) {
                  image->row_size = ibuf[0];
                  image->col_size = ibuf[1];
                  image->num_data_bands = ibuf[2] * ibuf[3] * ibuf[4];
                  image->num_of_images = 1;
                }
                else if (strcmp(token, "value:data_type") == 0) {
                  switch (ibuf[0]) {
                    case KBYTE:
                    case KUBYTE:
                      image->data_storage_type = VFF_TYP_1_BYTE; break;
                    case KSHORT:
                    case KUSHORT:
                      image->data_storage_type = VFF_TYP_2_BYTE; break;
                    case KINT:
                    case KUINT:
                    case KLONG:
                    case KULONG:
                      image->data_storage_type = VFF_TYP_4_BYTE; break;
                    case KFLOAT:
                      image->data_storage_type = VFF_TYP_FLOAT; break;
                    case KDOUBLE:
                      image->data_storage_type = VFF_TYP_DOUBLE; break;
                    case KCOMPLEX:
                      image->data_storage_type = VFF_TYP_COMPLEX; break;
                    case KDCOMPLEX:
                      image->data_storage_type = VFF_TYP_DCOMPLEX; break;
                    default:
                      fprintf(stderr,"Unsupported data type %d\n", ibuf[0]);
                      success = 0; break;
                  }
                }
                else if (strcmp(token, "map:size") == 0) {
                  image->map_enable = VFF_MAP_OPTIONAL;
                  image->map_scheme = VFF_MS_ONEPERBAND;
                  image->map_row_size = ibuf[0];
                  image->map_col_size = ibuf[1];
                }
                else if (strcmp(token, "map:data_type") == 0) {
                  switch (ibuf[0]) {
                    case KBYTE:
                    case KUBYTE:
                      image->map_storage_type = VFF_MAPTYP_1_BYTE; break;
                    case KSHORT:
                    case KUSHORT:
                      image->map_storage_type = VFF_MAPTYP_2_BYTE; break;
                    case KINT:
                    case KUINT:
                    case KLONG:
                    case KULONG:
                      image->map_storage_type = VFF_MAPTYP_4_BYTE; break;
                    case KFLOAT:
                      image->map_storage_type = VFF_MAPTYP_FLOAT; break;
                    case KDOUBLE:
                      image->map_storage_type = VFF_MAPTYP_DOUBLE; break;
                    case KCOMPLEX:
                      image->map_storage_type = VFF_MAPTYP_COMPLEX; break;
                    default:
                      fprintf(stderr,"Unsupported map type %d\n", ibuf[0]);
                      success = 0; break;
                  }
                }
                else if (strcmp(token, ":colorspace") == 0) {
                  switch (ibuf[0]) {
                    case KNONE:
                    case KGREY:
                      image->color_space_model = VFF_CM_NONE; break;
                    case KRGB:
                      image->color_space_model = VFF_CM_ntscRGB; break;
                    case KCMY:
                      image->color_space_model = VFF_CM_ntscCMY; break;
                    case KUSERDEFINED:
                      image->color_space_model = VFF_CM_GENERIC; break;
                    default:
                      fprintf(stderr,"Unsupported colour space model %d\n",
                              ibuf[0]);
                      success = 0; break;
                  }
                }
                else if (strcmp(token, "location:data_type") == 0 ||
                         strcmp(token, "uniform:data_type") == 0) {
                  if (ibuf[0] != KFLOAT)
                    fprintf(stderr,"Error: only FLOAT location data allowed\n");
                }
                else if (strcmp(token, "location:dimension") == 0) {
                  if (ibuf[0] != 4)
                    fprintf(stderr,"Error: Expected location dimension 4, got %d\n", ibuf[0]);
                }
                else if (strcmp(token, "uniform:dimension") == 0) {
                  if (ibuf[0] != 2)
                    fprintf(stderr,"Error: Expected uniform dimension 2, got %d\n", ibuf[0]);
                }
                else if (strcmp(token, "location:size") == 0) {
                  image->location_type = VFF_LOC_EXPLICIT;
                  image->location_dim  = ibuf[3];
                }
                else if (strcmp(token, "uniform:size") == 0) {
                  if (ibuf[0] != 2 || ibuf[1] != 3) {
                    fprintf(stderr,"Error: Expected uniform sizes 2x3, got %dx%d\n", ibuf[0], ibuf[1]);
                  }
                  else {
                    image->location_type = VFF_LOC_EXPLICIT;
                    image->location_dim  = 3;
                  }
                }
                /* Use ispare1 to store dimension of uniform location */
                else if (strcmp(token, "uniform:uniformSize") == 0) {
                  if (ibuf[1] == 1) image->ispare1 = 1;
                  else image->ispare1 = 2;
                }
                break;
              case KSTRING: /* Read string */
                if (number_of_items != 1) {
                  fprintf(stderr, "Multiple (%d) strings for token %s\n",
                          number_of_items, token);
                  fprintf(stderr,
                     "This feature of Khoros 2.0 format is not implemented.\n");
                  success = 0;
                }
                else {
                  success = KVIO2_read_string(fd, string);
/*                printf("%s 1 1 8192 %s\n", token, string); */
                }
                break;
              case KLONG:
              case KULONG: /* Read (unsigned) longs */
                /* These ulongs can be :ispare1, :ispare2, or
                   :xvimageMapScheme. They are all ignored */
                success = KVIO2_read_integers(fd, ibuf, number_of_items);
                break;
              case KFLOAT: /* Read floats */
                /* These floats can be :fspare1 or :fspare2. They are ignored */
                success = KVIO2_read_integers(fd, ibuf, number_of_items);
                break;
              case KDOUBLE: /* Read doubles */
                /* They are ignored */
                success = KVIO2_read_integers(fd, ibuf, 2*number_of_items);
                break;
              default:
                fprintf(stderr, "Error: unexpected parameter type %d (KFLOAT %d)\n",
                        ibuf[1], KFLOAT);
                success = 0;
                break;
            }
          }
        }
      }
    }
  }

  free(token);  free(string);  free(ibuf);
  if (!success) { free(image); return(NULL); }
  return(image);
}

/*
--------------------------------------------------------------------------------
                Read the header of an arbitrary Khoros image
--------------------------------------------------------------------------------
*/
xvimage *KVIO_readimageheader(FILE *fd, int *file_format)
{

/* Determine the image file format */

  *file_format = KVIO_image_file_format(fd);
  switch (*file_format) {
    case 0: /* Open or read error */
      return(NULL);

    case 1: /* Khoros 1.0 file format */
      return(KVIO1_readimageheader(fd));
      break;

    case 2: /* Khoros 2.0 file format */
      return(KVIO2_readimageheader(fd));
      break;

    case 3: /* Unknown format */
      return(NULL);
  }
}

/*
--------------------------------------------------------------------------------
               Determine the pixel size in bytes
--------------------------------------------------------------------------------
*/
int KVIO_pixel_size(xvimage *image)
{
  int size;

  switch (image->data_storage_type) {
    case VFF_TYP_1_BYTE:   size = 1;                   break;
    case VFF_TYP_2_BYTE:   size = 2;                   break;
    case VFF_TYP_4_BYTE:   size = 4;                   break;
    case VFF_TYP_FLOAT:    size = sizeof(float);       break;
    case VFF_TYP_COMPLEX:  size = 2 * sizeof(float);   break;
    case VFF_TYP_DOUBLE:   size = sizeof(double);      break;
    case VFF_TYP_DCOMPLEX: size = 2 * sizeof(double);  break;
    default:
      fprintf(stderr, "Unsupported data type: %d\n", image->data_storage_type);
      size = 0;
  }
  return(size);
}
/*
--------------------------------------------------------------------------------
               Determine the sizes of the image, map and location data 
--------------------------------------------------------------------------------
*/

int KVIO_image_size(xvimage *image)
{
  return(image->row_size * image->col_size * image->num_data_bands *
         image->num_of_images * KVIO_pixel_size(image));
}

int KVIO_map_size(xvimage *image)
{
  int size;

  size = image->map_row_size * image->map_col_size;
  switch (image->map_storage_type) {
    case VFF_MAPTYP_2_BYTE:   size *= 2;                 break;
    case VFF_MAPTYP_4_BYTE:   size *= 4;                 break;
    case VFF_MAPTYP_FLOAT:    size *= sizeof(float);     break;
    case VFF_MAPTYP_COMPLEX:  size *= 2 * sizeof(float); break;
    case VFF_MAPTYP_DOUBLE:   size *= sizeof(double);    break;
    default:                                             break; 
  }
  switch (image->map_scheme) {
    case VFF_MS_NONE:         size = 0;                       break;
    case VFF_MS_CYCLE:
    case VFF_MS_ONEPERBAND:   size *= image->num_data_bands;  break;
    case VFF_MS_GROUP:
    case VFF_MS_SHARED:                                       break;
    default:
      fprintf(stderr, "Invalid map scheme %d\n", image->map_scheme);
      size = 0;                                               break;
  }
  return(size);
}

int KVIO_location_size(xvimage *image)
{
  return(image->row_size * image->col_size * image->location_dim *
         sizeof(float));
}

/*
--------------------------------------------------------------------------------
                     Read (parts of) the image data
--------------------------------------------------------------------------------
*/

int KVIO_readimagepart(FILE *fd, xvimage *image, int row_offset, int col_offset,
                       int num_rows, int num_cols)
{
  int  size, nbytes, pixelsize, ib, ir;
  char *pixel;

/* Faster reading of whole image */

  if (row_offset == 0 && col_offset == 0 &&
      num_rows == image->col_size && num_cols == image->row_size) {
    size = KVIO_image_size(image);
    image->imagedata = (char *) malloc(size);
    if (!image->imagedata) {
      fprintf(stderr, "Error allocating image data memory.\n");
      return(0);
    }
    nbytes = fread(image->imagedata, sizeof(char), size, fd);
    if (nbytes != size) {
      fprintf(stderr, "Error reading image data\n");
      return(0);
    }
    return(1);
  }

/* More elaborated reading for arbitrary image part */

  if (row_offset < 0 || col_offset < 0 ||
      row_offset + num_rows > image->col_size ||
      col_offset + num_cols > image->row_size) return(0);
  pixelsize = KVIO_pixel_size(image);
  image->imagedata = (char *) malloc(num_rows * num_cols * pixelsize *
                                     image->num_data_bands *
                                     image->num_of_images);
  pixel = image->imagedata;
  if (!image->imagedata) {
    fprintf(stderr, "Error allocating image part data memory.\n");
    return(0);
  }
  /* Loop over all bands of all images */
  for (ib=0; ib<image->num_data_bands * image->num_of_images; ib++) {
    /* Skip the first "row_offset" rows */
    if (row_offset) {
      if (fseek(fd, row_offset * image->row_size * pixelsize, SEEK_CUR)) {
        fprintf(stderr, "Error seeking first image part data\n");
        return(0);
      }
    }
    /* Read the next "num_rows" rows */
    for (ir=0; ir<num_rows; ir++) {
      if (col_offset) {
        if (fseek(fd, col_offset * pixelsize, SEEK_CUR)) {
          fprintf(stderr, "Error seeking image row data\n");
          return(0);
        }
      }
      nbytes = fread(pixel, sizeof(char), num_cols * pixelsize, fd);
      if (nbytes != num_cols * pixelsize) {
        fprintf(stderr, "Error reading image row part\n");
        return(0);
      }
      pixel += nbytes; /* Set pixel pointer for the next row part */
      if (col_offset + num_cols < image->row_size) {
        size = (image->row_size - col_offset - num_cols) * pixelsize;
        if (fseek(fd, size, SEEK_CUR)) {
          fprintf(stderr, "Error seeking image row data\n");
          return(0);
        }
      }
    }
    /* Skip the remainder of the rows */
    if (row_offset + num_rows < image->col_size) {
      size = (image->col_size - row_offset - num_rows) * image->row_size *
             pixelsize;
      if (fseek(fd, size, SEEK_CUR)) {
        fprintf(stderr, "Error seeking last image part data\n");
        return(0);
      }
    }
  }
  image->row_size = num_cols;
  image->col_size = num_rows;
  return(1);
}

/*
--------------------------------------------------------------------------------
            Read the map, location and image data of a Khoros 1.0 image
--------------------------------------------------------------------------------
*/
int KVIO1_readimagedata(FILE *fd, xvimage *image, int row_offset,
                        int col_offset, int num_rows, int num_cols)
{
  int size, nbytes;

/* Read the map data */

  if (image->map_scheme != VFF_MS_NONE) {
    size = KVIO_map_size(image);
    image->maps = (char *) malloc(size);
    nbytes = fread(image->maps, sizeof(char), size, fd);
    if (nbytes != size) {
      fprintf(stderr, "Error reading image map\n");
      return(0);
    }
  }

/* Read the image location data */

  if (image->location_type == VFF_LOC_EXPLICIT) {
    size = KVIO_location_size(image);
    image->location = (float *) malloc(size);
    nbytes = fread(image->location, sizeof(char), size, fd);
    if (nbytes != size) {
      fprintf(stderr, "Error reading image location data\n");
      return(0);
    }
  }

/* Read the image data */

  return(KVIO_readimagepart(fd, image, row_offset, col_offset,
                            num_rows, num_cols));
}

/*
--------------------------------------------------------------------------------
                             Transpose the map data
--------------------------------------------------------------------------------
*/
int KVIO2_transpose_maps(xvimage *image)
{
  char   *new_maps, *maps_char, *new_maps_char;
  short  *maps_short, *new_maps_short;
  int    im, ir, ic, size, num_maps, map_size, *maps_int, *new_maps_int;
  float  *maps_float, *new_maps_float;
  double *maps_double, *new_maps_double;

  size = KVIO_map_size(image);
  if (image->map_scheme == VFF_MS_ONEPERBAND ||
      image->map_scheme == VFF_MS_CYCLE) num_maps = image->num_data_bands;
  else num_maps = 1;
  map_size = size / num_maps;
  new_maps = (char *) malloc(size);
  if (!new_maps) return(0);

  switch (image->map_storage_type) {
    case VFF_MAPTYP_1_BYTE:
      maps_char       = (char *) image->maps;
      new_maps_char   = (char *) new_maps;  break;
    case VFF_MAPTYP_2_BYTE:
      maps_short      = (short *) image->maps;
      new_maps_short  = (short *) new_maps;  break;
    case VFF_MAPTYP_4_BYTE:
      maps_int        = (int *) image->maps;
      new_maps_int    = (int *) new_maps;  break;
    case VFF_MAPTYP_FLOAT:
      maps_float      = (float *) image->maps;
      new_maps_float  = (float *) new_maps;  break;
    case VFF_MAPTYP_COMPLEX:
    case VFF_MAPTYP_DOUBLE:
      maps_double     = (double *) image->maps;
      new_maps_double = (double *) new_maps;  break;
  }
  for (im=0; im<num_maps; im++) {
    for (ir=0; ir<image->map_col_size; ir++) {
      for (ic=0; ic<image->map_row_size; ic++) {
        switch (image->map_storage_type) {
          case VFF_MAPTYP_1_BYTE:
            *(new_maps_char + im * map_size + ir + ic * image->map_col_size) =
              *(maps_char + im * map_size + ir * image->map_row_size + ic);
            break;
          case VFF_MAPTYP_2_BYTE:
            *(new_maps_short + im * map_size + ir + ic * image->map_col_size) =
              *(maps_short + im * map_size + ir * image->map_row_size + ic);
            break;
          case VFF_MAPTYP_4_BYTE:
            *(new_maps_int + im * map_size + ir + ic * image->map_col_size) =
              *(maps_int + im * map_size + ir * image->map_row_size + ic);
            break;
          case VFF_MAPTYP_FLOAT:
            *(new_maps_float + im * map_size + ir + ic * image->map_col_size) =
              *(maps_float + im * map_size + ir * image->map_row_size + ic);
            break;
          case VFF_MAPTYP_COMPLEX:
          case VFF_MAPTYP_DOUBLE:
            *(new_maps_double + im * map_size + ir + ic * image->map_col_size) =
              *(maps_double + im * map_size + ir * image->map_row_size + ic);
            break;
        }
      }
    }
  }
  free(image->maps);
  image->maps = new_maps;
  return(1);
}

/*
--------------------------------------------------------------------------------
                    Transform uniform to curvilinear location data
--------------------------------------------------------------------------------
*/
void KVIO2_uniform2location(xvimage *image, float *uniform)
{
  int   size, ir, ic, num_step;
  float *loc, xbegin, xstep, x, ybegin, ystep, y, z;

/* Convert uniform location data to curvilinear location data */

  size = KVIO_location_size(image);
  image->location = (float *) malloc(size);
  loc      = image->location;
  num_step = image->row_size - 1; /* Derive X-locations */
  xbegin   = uniform[0];
  xstep    = (uniform[1] - uniform[0]) / num_step;
  for (ir=0; ir<image->col_size; ir++)
    for (ic=0, x=xbegin; ic<image->row_size; ic++, x+=xstep, loc++) *loc = x;
  num_step = image->col_size - 1; /* Derive Y-locations */
  ybegin   = uniform[2];
  ystep    = (uniform[3] - uniform[2]) / num_step;
  for (ir=0, y=ybegin; ir<image->col_size; ir++, y+=ystep)
    for (ic=0; ic<image->row_size; ic++, loc++) *loc = y;
  z = uniform[4]; /* Constant Z-location */
  for (ir=0; ir<image->col_size; ir++)
    for (ic=0; ic<image->row_size; ic++, loc++) *loc = z;

/* For compatibility to older versions, store two location bounds
 * in the fspare's
 */
  if (image->ispare1 == 1) {
    image->fspare1 = uniform[0];
    image->fspare2 = uniform[1];
    image->ispare1 = 0;
  }
  else if (image->ispare1 == 2) {
    image->fspare1 = uniform[0];
    image->fspare2 = uniform[2];
    image->ispare1 = 0;
  }

/* Signal that there is explicit location data */

  image->location_dim  = 3;
  image->location_type = VFF_LOC_EXPLICIT;
}

/*
--------------------------------------------------------------------------------
            Read the map, location and image data of a Khoros 2.0 image
--------------------------------------------------------------------------------
*/
int KVIO2_readimagedata(FILE *fd, xvimage *image, int row_offset,
                        int col_offset, int num_rows, int num_cols)
{
  int   size, nbytes, num_step, ir, ic, all_rows, all_cols, part;
  char  *token;
  float *uniform, xbegin, ybegin, xstep, ystep, x, y, z, *loc;

/* The file should be positioned at the first pixel value */

  part = (num_rows != image->col_size || num_cols != image->row_size);
  if (part) { /* Save dimensions of whole image for reading location data */
    all_rows = image->col_size;
    all_cols = image->row_size;
  }
  if (!KVIO_readimagepart(fd, image, row_offset, col_offset, num_rows,num_cols))
    return(0);

/* Read other data fields */

  token = (char *) malloc(1024);
  while (KVIO2_read_string(fd, token)) {
    if (strcmp(token, "map:") == 0) {
      size = KVIO_map_size(image);
      image->maps = (char *) malloc(size);
      nbytes = fread(image->maps, sizeof(char), size, fd);
      if (nbytes != size) {
        fprintf(stderr, "Error reading map data\n");
        return(0);
      }
      KVIO2_transpose_maps(image);
    }
    else if (strcmp(token, "location:") == 0) {
      if (part) { /* Temporarily restore complete image dimensions */
        image->col_size = all_rows;
        image->row_size = all_cols;
      }
      size = KVIO_location_size(image);
      image->location = (float *) malloc(size);
      nbytes = fread(image->location, sizeof(char), size, fd);
      if (nbytes != size) {
        fprintf(stderr, "Error reading location data\n");
        return(0);
      }
      if (part) { /* Set the image dimensions back to the part dimensions */
        image->col_size = num_rows;
        image->row_size = num_cols;
      }
    }
    else if (strcmp(token, "uniform:") == 0) {
      if (image->location_type == VFF_LOC_EXPLICIT) {
        size = 6 * sizeof(float);
        uniform = (float *) malloc(size);
        nbytes = fread(uniform, sizeof(char), size, fd);
        if (nbytes != size) {
          fprintf(stderr, "Error reading uniform location data\n");
          return(0);
        }
        KVIO2_uniform2location(image, uniform);
        free(uniform);
      }
    }
    else {
      printf("Ignored section %s, skipping remainder of file.\n", token);
      break;
    }
  }
  if (feof(fd)) return(1);
  else {
    fprintf(stderr, "Error parsing data fields.\n");
    return(0);
  }
}


/******************** Start of public routines ********************************/

/*
--------------------------------------------------------------------------------
          Create an image structure and allocate data fields
--------------------------------------------------------------------------------
*/
xvimage *createimage(unsigned long col_size, unsigned long row_size,
                     unsigned long data_storage_type,
                     unsigned long num_of_images,
                     unsigned long num_data_bands, const char *comment,
                     unsigned long map_row_size, unsigned long map_col_size,
                     unsigned long map_scheme,
                     unsigned long map_storage_type,
                     unsigned long location_type, unsigned long location_dim)
{
  xvimage *image;

  image = (xvimage *) calloc(1, sizeof(xvimage));
  if (!image) return(NULL);
  image->identifier        = XV_FILE_MAGIC_NUM;
  image->file_type         = XV_FILE_TYPE_XVIFF;
  image->release           = XV_IMAGE_REL_NUM;
  image->version           = XV_IMAGE_VER_NUM;
  image->machine_dep       = VFF_DEP_NSORDER;
  image->startx            = -1;
  image->starty            = -1;
  image->pixsizx           = 1.0;
  image->pixsizy           = 1.0;
  image->col_size          = col_size;
  image->row_size          = row_size;
  image->data_storage_type = data_storage_type;
  image->num_of_images     = num_of_images;
  image->num_data_bands    = num_data_bands;
  image->map_row_size      = map_row_size;
  image->map_col_size      = map_col_size;
  image->map_scheme        = map_scheme;
  image->map_storage_type  = map_storage_type;
  image->map_enable        = VFF_MAP_OPTIONAL;
  image->location_type     = location_type;
  image->location_dim      = location_dim;
  if (comment) {
    if (strlen(comment) < 512) strcpy(image->comment, comment);
    else strncpy(image->comment, comment, 511);
  }
  else image->comment[0] = 0;
  image->imagedata = (char *) calloc(KVIO_image_size(image), 1);
  if (!image->imagedata) {free(image); return(NULL);}
  if (image->location_type == VFF_LOC_EXPLICIT) {
    image->location = (float *) calloc(KVIO_location_size(image), 1);
    if (!image->location) {free(image->imagedata); free(image); return(NULL);}
  }
  if (image->map_scheme != VFF_MS_NONE) {
    image->maps = (char *) calloc(KVIO_map_size(image), 1);
    if (!image->maps) {
      free(image->imagedata);
      if (image->location_type == VFF_LOC_EXPLICIT) free(image->location);
      free(image); return(NULL);
    }
  }
  return(image);
}

/*
--------------------------------------------------------------------------------
                          Read the image header
--------------------------------------------------------------------------------
*/
xvimage *readimageheader(const char *filename)
{
  int     file_format;
  xvimage *image;
  FILE    *fd;

  fd = fopen(filename, "rb");
  if (!fd) return(NULL);
  image = KVIO_readimageheader(fd, &file_format);
  if (image) fclose(fd);
  return(image);
}

/*
--------------------------------------------------------------------------------
                              Read the image 
--------------------------------------------------------------------------------
*/
xvimage *readimage(const char *filename)
{
  xvimage *image;
  int     file_format, success;
  FILE    *fd;

/* Read header and determine file format */

  fd = fopen(filename, "rb");
  if (!fd) return(NULL);
  image = KVIO_readimageheader(fd, &file_format);
  if (!image) return(NULL);

/* fd already open and positioned after header */
/* Read Khoros 1.0 image data */

  if (file_format == 1)
    success = KVIO1_readimagedata(fd, image, 0, 0,
                                  image->col_size, image->row_size);

/* Read Khoros 2.0 image data */

  else /* (file_format == 2) */
    success = KVIO2_readimagedata(fd, image, 0, 0,
                                  image->col_size, image->row_size);

  if (success) { fclose(fd); return(image); }
  else return(NULL);
}

/*
--------------------------------------------------------------------------------
                            Read an image part
--------------------------------------------------------------------------------
*/
xvimage *readimagepart(const char *filename, int row_offset, int col_offset,
                       int num_rows, int num_cols)
{
  xvimage *image;
  int     file_format, success;
  FILE    *fd;

/* Read header and determine file format */

  fd = fopen(filename, "rb");
  if (!fd) return(NULL);
  image = KVIO_readimageheader(fd, &file_format);
  if (!image) return(NULL);

/* fd positioned after header */
/* Read Khoros 1.0 image data */

  if (file_format == 1)
    success = KVIO1_readimagedata(fd, image, row_offset, col_offset,
                                  num_rows, num_cols);

/* Read Khoros 2.0 image data */

  else /* (file_format == 2) */
    success = KVIO2_readimagedata(fd, image, row_offset, col_offset,
                                  num_rows, num_cols);

  if (success) { fclose(fd); return(image); }
  else return(NULL);
}

/*
--------------------------------------------------------------------------------
                  Write the image (in Khoros 1.0 image format)
--------------------------------------------------------------------------------
*/
int writeimage(const char *filename, xvimage *image)
{
  FILE  *fd;
  int   nbytes, size;
  float uniform[5];

  if (!(fd = fopen(filename, "wb"))) { /* Open image file */
    fprintf(stderr, "Error opening file %s for writing image.\n", filename);
    return(0);
  }

/* Write the header data */

  image->identifier  = XV_FILE_MAGIC_NUM;
  image->file_type   = XV_FILE_TYPE_XVIFF;
  image->release     = XV_IMAGE_REL_NUM;
  image->version     = XV_IMAGE_VER_NUM;
  image->machine_dep = VFF_DEP_NSORDER;
  nbytes = fwrite(image, sizeof(char), 1024, fd);
  if (nbytes != 1024) {fclose(fd);  return(0);}

/* Write map data */

  if (image->map_scheme != VFF_MS_NONE) {
    size = KVIO_map_size(image);
    nbytes = fwrite(image->maps, sizeof(char), size, fd);
    if (nbytes != size) {fclose(fd); return(0);}
  }

/* For compatibility, convert fspare values to location data */

  if (image->fspare1 != 0.0 || image->fspare2 != 0.0) {
    if (image->col_size == 1) {
      uniform[0] = image->fspare1;
      uniform[1] = image->fspare2;
      uniform[2] = uniform[4] = 0.0;
      uniform[3] = 1.0;
    }
    else {
      if (image->fspare1 < 0.0)
        {uniform[0] = image->fspare1; uniform[1] = -uniform[0];}
      else {uniform[0] = 0.0; uniform[1] = image->fspare1;}
      if (image->fspare2 < 0.0)
        {uniform[2] = image->fspare2; uniform[3] = -uniform[2];}
      else {uniform[2] = 0.0; uniform[3] = image->fspare2;}
    }
    KVIO2_uniform2location(image, uniform);
  }

/* Write the image location data */

  if (image->location_type == VFF_LOC_EXPLICIT) {
    size = KVIO_location_size(image);
    nbytes = fwrite(image->location, sizeof(char), size, fd);
    if (nbytes != size) {fclose(fd); return(0);}
  }

/* Write the image data */

  size = KVIO_image_size(image);
  nbytes = fwrite(image->imagedata, sizeof(char), size, fd);
  fclose(fd);
  if (nbytes != size) return(0);
  return(1);
}

/*
--------------------------------------------------------------------------------
                  Checking of several image properties
--------------------------------------------------------------------------------
*/
int propertype(const char *prog, xvimage *image, unsigned long type, int exit_flag)
{
  if (image->data_storage_type != type) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect data storage type.\n",
              prog);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_num_images(const char *prog, xvimage *image, unsigned long number,
                      int exit_flag)
{
  if (image->num_of_images != number) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect number of images.\n",
              prog);
      fprintf(stderr, "%s: File contains %d images, required are %d images.\n",
              prog, image->num_of_images, number);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_num_bands(const char *prog, xvimage *image, unsigned long number,
                     int exit_flag)
{
  if (image->num_data_bands != number) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect number of bands.\n",
              prog);
      fprintf(stderr, "%s: Image contains %d bands, required are %d bands.\n",
              prog, image->num_data_bands, number);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_map_enable(const char *prog, xvimage *image, unsigned long type,
                      int exit_flag)
{
  if (image->map_enable != type) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect map enable value.\n",
              prog);
      fprintf(stderr, "%s: Current value is %d , required is %d.\n",
              prog, image->map_enable, type);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_map_type(const char *prog, xvimage *image, unsigned long type,
                    int exit_flag)
{
  if (image->map_storage_type != type) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect map type.\n",
              prog);
      fprintf(stderr, "%s: Current value is %d , required is %d.\n",
              prog, image->map_storage_type, type);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_map_scheme(const char *prog, xvimage *image, unsigned long type,
                      int exit_flag)
{
  if (image->map_scheme != type) {
    if (exit_flag) {
      fprintf(stderr, "%s: Error reading image, incorrect mapping scheme.\n",
              prog);
      fprintf(stderr, "%s: Current value is %d , required is %d.\n",
              prog, image->map_scheme, type);
      exit(0);
    }
    return(0);
  }
  return(1);
}

int proper_color_model(const char *prog, xvimage *image, unsigned long type,
                       int exit_flag)
{   
  if (image->color_space_model != type) {
    if (exit_flag) {
      fprintf(stderr, 
              "%s: Error reading image, incorrect color space model.\n",
              prog);
      fprintf(stderr, "%s: Current value is %d , required is %d.\n",
              prog, image->color_space_model, type);
      exit(0);
    }
    return(0);
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Free allocated image memory
--------------------------------------------------------------------------------
*/
int freeimage(xvimage *image)
{
  if (image->imagedata) free(image->imagedata);
  if (image->location && image->location_type == VFF_LOC_EXPLICIT)
    free(image->location);
  if (image->maps && image->map_scheme != VFF_MS_NONE) free(image->maps);
  free(image);
  return(1);
}

/*
--------------------------------------------------------------------------------
                            Copy image header
--------------------------------------------------------------------------------
*/
void copyheader(xvimage *image_src, xvimage *image_dest)
{
  image_dest->identifier = image_src->identifier;
  image_dest->file_type = image_src->file_type;
  image_dest->release = image_src->release;
  image_dest->version = image_src->version;
  image_dest->machine_dep = image_src->machine_dep;
  strcpy(image_dest->comment, image_src->comment);
  image_dest->row_size = image_src->row_size;
  image_dest->col_size = image_src->col_size;
  image_dest->subrow_size = image_src->subrow_size;
  image_dest->startx = image_src->startx;
  image_dest->starty = image_src->starty;
  image_dest->pixsizx = image_src->pixsizx;
  image_dest->pixsizy = image_src->pixsizy;
  image_dest->location_type = image_src->location_type;
  image_dest->location_dim = image_src->location_dim;
  image_dest->num_of_images = image_src->num_of_images;
  image_dest->num_data_bands = image_src->num_data_bands;
  image_dest->data_storage_type = image_src->data_storage_type;
  image_dest->data_encode_scheme = image_src->data_encode_scheme;
  image_dest->map_scheme = image_src->map_scheme;
  image_dest->map_storage_type = image_src->map_storage_type;
  image_dest->map_row_size = image_src->map_row_size;
  image_dest->map_col_size = image_src->map_col_size;
  image_dest->map_subrow_size = image_src->map_subrow_size;
  image_dest->map_enable = image_src->map_enable;
  image_dest->maps_per_cycle = image_src->maps_per_cycle;
  image_dest->color_space_model = image_src->color_space_model;
  image_dest->ispare1 = image_src->ispare1;
  image_dest->ispare2 = image_src->ispare2;
  image_dest->fspare1 = image_src->fspare1;
  image_dest->fspare2 = image_src->fspare2;
}

/*
--------------------------------------------------------------------------------
                Sizes of image, map and location data fields
--------------------------------------------------------------------------------
*/
int imagesize(xvimage *image, int *dsize, int *dcount, int *msize, int *mcount,
              int *lsize, int *lcount)
{
  *dsize  = KVIO_image_size(image);
  *dcount = *dsize / KVIO_pixel_size(image);
  if (image->map_scheme == VFF_MS_NONE) {
    *msize  = KVIO_map_size(image);
    *mcount = image->map_row_size * image->map_col_size;
    if (image->map_scheme == VFF_MS_ONEPERBAND ||
        image->map_scheme == VFF_MS_CYCLE) *mcount *= image->num_data_bands;
  }
  else *msize = *mcount = 0;
  if (image->location_type == VFF_LOC_EXPLICIT) {
    *lsize  = KVIO_location_size(image);
    *lcount = *lsize / sizeof(float);
  }
  else *lsize = *lcount = 0;
  return(sizeof(xvimage) + *dsize + *msize + *lsize); /* Total size in bytes */
}
