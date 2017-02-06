
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
 Collection of functions for class ImagedData

 Initial creation
 Author : George Vosselman
 Date   : 29-03-1999

 Update #1
 Author : 
 Date   : 
 Changes: 

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
#include "ImagedData.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                            Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" int freeimage(xvimage *);

/*
--------------------------------------------------------------------------------
                            Copy assignment
--------------------------------------------------------------------------------
*/

ImagedData ImagedData::operator = (const ImagedData &i)
{
  // Check for self assignment
  if (this == &i) return *this;
  if (i.image || i.grid)
    fprintf(stderr,
    "Warning: ImagedData::operator = only copies pointers of image and grid\n");
  StringCopy(&image_file, i.image_file);
  image      = i.image; /* Only copying pointer! */
  StringCopy(&grid_file, i.grid_file);
  grid       = i.grid; /* Only copying pointer! */
  datatype   = i.datatype;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                          Set the image and grid file names
--------------------------------------------------------------------------------
*/

void ImagedData::Initialise()
{
  Image::Initialise();
  grid = NULL;
  grid_file = NULL;
  datatype = UnknownDataType;
}

void ImagedData::ReInitialise()
{
  if (image) (void) freeimage(image);
  if (image_file) free(image_file);
  if (grid) grid->~DataGrid();
  if (grid_file) free(grid_file);
  Initialise();
}

/*
--------------------------------------------------------------------------------
                          Set the grid file name
--------------------------------------------------------------------------------
*/

void ImagedData::SetGridFile(const char *filename)
{
  StringCopy(&grid_file, filename);
}

/*
--------------------------------------------------------------------------------
                          Derive grid file name
--------------------------------------------------------------------------------
*/

char *ImagedData::DeriveGridFileName()
{
  int  len;
  char *dot;

  if (!image_file) {
    fprintf(stderr, "Error: No image file name to derive grid file name.\n");
    return(NULL);
  } 
  dot = strrchr(image_file, '.');
  if (dot) len = (int) dot - (int) image_file;
  else len = strlen(image_file);
  grid_file = (char *) realloc(grid_file, len + 6);
  strncpy(grid_file, image_file, len);
  grid_file[len] = 0;
  strcat(grid_file, ".grid");
  return(grid_file);
}

/*
--------------------------------------------------------------------------------
                          Read the image file
--------------------------------------------------------------------------------
*/

int ImagedData::ReadImage(const char *filename)
{
  return(Image::Read(filename));
}

int ImagedData::ReadImage()
{
  return(Image::Read());
}

/*
--------------------------------------------------------------------------------
                          Write the image file
--------------------------------------------------------------------------------
*/

int ImagedData::WriteImage(const char *filename) const
{
  return(Image::Write(filename));
}

int ImagedData::WriteImage() const
{
  return(Image::Write());
}

/*
--------------------------------------------------------------------------------
                          Read the grid file
--------------------------------------------------------------------------------
*/

DataGrid *ImagedData::ReadGrid(const char *filename)
{
  if (grid_file) free(grid_file);
  grid_file = (char *) malloc(strlen(filename) + 1);
  strcpy(grid_file, filename);
  return(ReadGrid());
}

DataGrid *ImagedData::ReadGrid()
{
  int success;

  if (grid) delete grid;
  grid = new DataGrid(grid_file, &success);
  if (!success) {
    fprintf(stderr, "Error reading data grid file %s\n", grid_file);
    grid = NULL;
  }
  return(grid);
}

/*
--------------------------------------------------------------------------------
                          Write the grid file
--------------------------------------------------------------------------------
*/

int ImagedData::WriteGrid(const char *filename) const
{
  return(grid->Write(filename));
}

int ImagedData::WriteGrid() const
{
  return(grid->Write(grid_file));
}

/*
--------------------------------------------------------------------------------
                  Read both the image file and the grid file
--------------------------------------------------------------------------------
*/

bool ImagedData::Read()
{
  if (!ReadImage()) return false;
  return ReadGrid();
}

bool ImagedData::Read(const char *image_filename, const char *grid_filename)
{
  if (!ReadImage(image_filename)) return false;
  return ReadGrid(grid_filename);
}
/*
--------------------------------------------------------------------------------
                  Write both the image file and the grid file
--------------------------------------------------------------------------------
*/

bool ImagedData::Write() const
{
  if (!WriteImage()) return false;
  return WriteGrid();
}

bool ImagedData::Write(const char *image_filename,
                       const char *grid_filename) const
{
  if (!WriteImage(image_filename)) return false;
  return WriteGrid(grid_filename);
}
/*
--------------------------------------------------------------------------------
                      Read meta data from a file
--------------------------------------------------------------------------------
*/

void ImagedData::ReadMetaData(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "type", MAX(keyword_length, 4))) {
          keyword = BNF_String(line);
          if (!strncmp(keyword, "height", MAX(strlen(keyword), 6)))
            datatype = HeightData;
          else if (!strncmp(keyword, "reflectance", MAX(strlen(keyword), 11)))
            datatype = ReflectanceData;
          else if (!strncmp(keyword, "colour", MAX(strlen(keyword), 6)))
            datatype = ColourData;
          else if (!strncmp(keyword, "pulselength", MAX(strlen(keyword), 11)))
            datatype = PulseLengthData;
          else if (!strncmp(keyword, "pointcount", MAX(strlen(keyword), 10)))
            datatype = PointCountData;
          else
            fprintf(stderr, "Error: Unknown data type of imaged data: %s\n",
                    keyword);
        }
        else if (!strncmp(keyword, "image", MAX(keyword_length, 5)))
          SetImageFile(BNF_String(line));

        else if (!strncmp(keyword, "grid", MAX(keyword_length, 4)))
          SetGridFile(BNF_String(line));

        else if (!strncmp(keyword, "endimageddata", MAX(keyword_length, 13))) {
          free(buffer);
          return;
        }
        else
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endimageddata.\n");
}

/*
--------------------------------------------------------------------------------
                Write scanner characteristics to file in BNF format
--------------------------------------------------------------------------------
*/

int ImagedData::HasSomeData() const
{
  return(image_file || grid_file);
}

void ImagedData::WriteMetaData(FILE *fd, int indent) const
{
  if (!HasSomeData()) return;
  BNF_Write_String(fd, "imageddata", indent, NULL);
  if (datatype == HeightData) BNF_Write_String(fd, "type", indent+2, "height");
  else if (datatype == ReflectanceData)
    BNF_Write_String(fd, "type", indent+2, "reflectance");
  else if (datatype == ColourData)
    BNF_Write_String(fd, "type", indent+2, "colour");
  else if (datatype == PulseLengthData)
    BNF_Write_String(fd, "type", indent+2, "pulselength");
  else if (datatype == PointCountData)
    BNF_Write_String(fd, "type", indent+2, "pointcount");
  if (image_file) BNF_Write_String(fd, "image", indent+2, image_file);
  if (grid_file) BNF_Write_String(fd, "grid", indent+2, grid_file);
  BNF_Write_String(fd, "endimageddata", indent, NULL);
}
