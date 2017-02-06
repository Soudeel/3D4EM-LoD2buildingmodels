
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
 Collection of functions for class LaserPointsInfo

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
#include "LaserPointsInfo.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                              Copy assignment
--------------------------------------------------------------------------------
*/

LaserPointsInfo LaserPointsInfo::operator = (const LaserPointsInfo &i)
{
  if (i.boundary)
    fprintf(stderr,
      "Warning: LaserPointsInfo::operator = only copies pointer of boundary\n");
  scanner = i.Scanner();
  bounds = i.bounds;
  imaged_height = i.imaged_height;
  imaged_reflectance = i.imaged_reflectance;
  imaged_colour = i.imaged_colour;
  imaged_pulselength = i.imaged_pulselength;
  imaged_pointcount = i.imaged_pointcount;
  point_density = i.point_density;
  StringCopy(&boundary_file, i.boundary_file);
  boundary = i.boundary;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Initialise the laser points information
--------------------------------------------------------------------------------
*/

void LaserPointsInfo::Initialise()
{
  scanner.Initialise();
  bounds.Initialise();
  imaged_height.Initialise();
  imaged_height.SetDataType(HeightData);
  imaged_reflectance.Initialise();
  imaged_reflectance.SetDataType(ReflectanceData);
  imaged_colour.Initialise();
  imaged_colour.SetDataType(ColourData);
  imaged_pulselength.Initialise();
  imaged_pulselength.SetDataType(PulseLengthData);
  imaged_pointcount.Initialise();
  imaged_pointcount.SetDataType(PointCountData);
  point_density = -1.0;
  boundary_file = NULL;
  boundary = NULL;
}

void LaserPointsInfo::ReInitialise()
{
  scanner.ReInitialise();
  bounds.Initialise();
  imaged_height.ReInitialise();
  imaged_height.SetDataType(HeightData);
  imaged_reflectance.ReInitialise();
  imaged_reflectance.SetDataType(ReflectanceData);
  imaged_colour.ReInitialise();
  imaged_colour.SetDataType(ColourData);
  imaged_pulselength.ReInitialise();
  imaged_pulselength.SetDataType(PulseLengthData);
  imaged_pointcount.ReInitialise();
  imaged_pointcount.SetDataType(PointCountData);
  point_density = -1.0;
  if (boundary_file) free(boundary_file); boundary_file = NULL;
  if (boundary) boundary->~Positions2D(); boundary = NULL;
}

/*
--------------------------------------------------------------------------------
                               Read meta data
--------------------------------------------------------------------------------
*/

void LaserPointsInfo::ReadMetaData(FILE *fd)
{
  ImagedData imaged_data;
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "scanner", MAX(keyword_length, 7)))
          scanner.Read(fd);

        else if (!strncmp(keyword, "databounds", MAX(keyword_length, 10)))
          bounds.Read(fd);

        else if (!strncmp(keyword, "imageddata", MAX(keyword_length, 10))) {
          imaged_data.ReadMetaData(fd);
          if (imaged_data.DataType() == HeightData)
            imaged_height = imaged_data;
          else if (imaged_data.DataType() == ReflectanceData)
            imaged_reflectance = imaged_data;
          else if (imaged_data.DataType() == ColourData)
            imaged_colour = imaged_data;
          else if (imaged_data.DataType() == PulseLengthData)
            imaged_pulselength = imaged_data;
          else if (imaged_data.DataType() == PointCountData)
            imaged_pointcount = imaged_data;
          else
            fprintf(stderr,
                    "Error: Found imaged data without specified type.\n");
        }
        else if (!strncmp(keyword, "point_density", MAX(keyword_length, 13)))
          point_density = BNF_Double(line);

        else if (!strncmp(keyword, "boundary", MAX(keyword_length, 8)))
          boundary_file = BNF_String(line);

        else if (!strncmp(keyword, "endinfo", MAX(keyword_length, 7))) {
          free(buffer);
          return;
        }
        else
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
}

/*
--------------------------------------------------------------------------------
                            Write meta data
--------------------------------------------------------------------------------
*/

int LaserPointsInfo::HasSomeData() const
{
  return(scanner.HasSomeData() || bounds.HasSomeData() ||
         imaged_height.HasSomeData() || imaged_reflectance.HasSomeData() ||
         imaged_colour.HasSomeData() || imaged_pulselength.HasSomeData() ||
         point_density >= 0.0 || boundary_file);
}

void LaserPointsInfo::WriteMetaData(FILE *fd, int indent) const
{
  int colour;
  if (!HasSomeData()) return;
  BNF_Write_String(fd, "info", indent, NULL);
  scanner.Write(fd, indent+2);
  colour = (scanner.PointType() == ColourPoint ||
            scanner.PointType() == MultiColourPoint);
  bounds.Write(fd, indent+2, colour);
  if (imaged_height.DataType() != UnknownDataType ||
      imaged_height.ImageFile() || imaged_height.GridFile())
    imaged_height.WriteMetaData(fd, indent+2);
  if (imaged_reflectance.DataType() != UnknownDataType ||
      imaged_reflectance.ImageFile() || imaged_reflectance.GridFile())
    imaged_reflectance.WriteMetaData(fd, indent+2);
  if (imaged_colour.DataType() != UnknownDataType ||
      imaged_colour.ImageFile() || imaged_colour.GridFile())
    imaged_colour.WriteMetaData(fd, indent+2);
  if (imaged_pulselength.DataType() != UnknownDataType ||
      imaged_pulselength.ImageFile() || imaged_pulselength.GridFile())
    imaged_pulselength.WriteMetaData(fd, indent+2);
  if (imaged_pointcount.DataType() != UnknownDataType ||
      imaged_pointcount.ImageFile() || imaged_pointcount.GridFile())
    imaged_pointcount.WriteMetaData(fd, indent+2);

  if (point_density >= 0.0) BNF_Write_Double(fd, "point_density", indent+2,
                                             point_density, "%15.5f");
  if (boundary_file) BNF_Write_String(fd, "boundary", indent+2, boundary_file);
  BNF_Write_String(fd, "endinfo", indent, NULL);
}

/*
--------------------------------------------------------------------------------
                            Create an image
--------------------------------------------------------------------------------
*/

Image *LaserPointsInfo::CreateImage(double pixelsize, int viff_type,
                                    ImagedDataType datatype,
                                    int interpolation_method,
                                    double max_mesh_size)
{
  ImagedData    *imaged_data;
  Image         dist;
  double        xrange, yrange;
  int           num_rows, num_cols, num_bands;

/* Set the pointer to the correct type of imaged data and derive the 
 * data grid from the bounds and the pixel size.
 */

  switch (datatype) {
    case HeightData:
      imaged_data = &imaged_height;
      imaged_height.SetGrid(bounds.HeightDataGrid(pixelsize));
      num_bands = 1;
      break;

    case ReflectanceData:
      imaged_data = &imaged_reflectance;
      if (scanner.PointType() == ColourPoint ||
          scanner.PointType() == MultiColourPoint)
        imaged_reflectance.SetGrid(bounds.ColourDataGrid(pixelsize));
      else
        imaged_reflectance.SetGrid(bounds.ReflectanceDataGrid(pixelsize));
      num_bands = 1;
      break;

    case ColourData:
      imaged_data = &imaged_colour;
      imaged_colour.SetGrid(bounds.ColourDataGrid(pixelsize));
      num_bands = 3;
      break;

    case PulseLengthData:
      imaged_data = &imaged_pulselength;
      imaged_pulselength.SetGrid(bounds.PulseLengthDataGrid(pixelsize));
      num_bands = 1;
      break;

    case PointCountData:
      imaged_data = &imaged_pointcount;
      imaged_pointcount.SetGrid(bounds.PointCountDataGrid(pixelsize));
      num_bands = 1;
      break;

    default:
      fprintf(stderr, "Error: Unknown data type (%d) for imaging.\n",
              datatype);
      exit(0);
  }

/* Reset the data offset and data scale in case of a FLOAT image */

  if (viff_type == VFF_TYP_FLOAT) {
    imaged_data->Grid()->DataOffset() = 0.0;
    imaged_data->Grid()->DataScale()  = 1.0;
  }

/* Create new images for the grey values and distances. */

  xrange = bounds.XRange();  yrange = bounds.YRange();
  if (xrange == 0.0 || yrange == 0.0) {
    fprintf(stderr, "Error: Range of X- and/or Y-coordinates equals zero.\n");
    fprintf(stderr, "       Maybe you forgot to set the bounds.\n");
    fprintf(stderr, "       No image created!\n");
    exit(0);
  }
  num_rows = (int) (yrange / pixelsize);
  if (num_rows < yrange / pixelsize) num_rows++;
  num_cols = (int) (xrange / pixelsize);
  if (num_cols < xrange / pixelsize) num_cols++;
  imaged_data->NewImage(num_rows, num_cols, viff_type, num_bands);
  if (datatype == ColourData)
    imaged_data->GetImage()->color_space_model = VFF_CM_ntscRGB;
  dist.NewImage(num_rows, num_cols, VFF_TYP_FLOAT);

/* Initialise the grey values with 0 (for unsigned char images) or
 * the minimum Z-value (for float images).
 * Initialise the distances with 1.0e9.
 */

  if (viff_type == VFF_TYP_1_BYTE) imaged_data->ClearImage();
  else if (viff_type == VFF_TYP_FLOAT)
    imaged_data->SetPixels((float) bounds.Minimum().Z());
  dist.SetPixels((float) 1.0e9);

/* Fill the image with data of the laser points */

  ImageData(imaged_data->ImageReference(), dist, *(imaged_data->Grid()),
            datatype, interpolation_method, max_mesh_size);

/* Return the reference to the image */

  return(&(imaged_data->ImageReference()));
}

void LaserPointsInfo::ImageData(Image &image, Image &dist,
                                const DataGrid &grid, ImagedDataType datatype,
                                int interpolation_method, double max_mesh_size)
{
  fprintf(stderr,
       "Error: LaserPointsInfo::ImageData is a virtual function!\n");
  fprintf(stderr,
       "       There is no data in class LaserPointsInfo to fill the image.\n");
  fprintf(stderr,
       "       ImageData should be called as a member of class LaserBlock,\n");
  fprintf(stderr,
       "       LaserUnit, LaserSubUnit, or LaserPoints.\n");
  exit(0);
}

/*
--------------------------------------------------------------------------------
                            Set boundary file name
--------------------------------------------------------------------------------
*/

void LaserPointsInfo::SetBoundaryFile(const char *filename)
{
  StringCopy(&boundary_file, filename);
}

/*
--------------------------------------------------------------------------------
                          Read boundary from file
--------------------------------------------------------------------------------
*/

Positions2D *LaserPointsInfo::ReadBoundary(const char *filename)
{
  SetBoundaryFile(filename);
  return(ReadBoundary());
}


Positions2D *LaserPointsInfo::ReadBoundary()
{
  int success;

  if (boundary) delete boundary;
  boundary = new Positions2D(boundary_file, &success);
  if (!success) {
    fprintf(stderr, "Error reading boundary from file %s\n", boundary_file);
    boundary = NULL;
  }
  return(boundary);
}

/*
--------------------------------------------------------------------------------
                          Derive file names
--------------------------------------------------------------------------------
*/

char *LaserPointsInfo::DeriveHeightImageFileName(const char *name,
                                                 const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".height.xv");
  imaged_height.SetImageFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveHeightGridFileName(const char *name,
                                                const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".height.grid");
  imaged_height.SetGridFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveHeightGridFileName()
{
  return(imaged_height.DeriveGridFileName());
}

char *LaserPointsInfo::DeriveReflectanceImageFileName(const char *name,
                                                      const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".reflectance.xv");
  imaged_reflectance.SetImageFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveReflectanceGridFileName(const char *name,
                                                     const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".reflectance.grid");
  imaged_reflectance.SetGridFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveReflectanceGridFileName()
{
  return(imaged_reflectance.DeriveGridFileName());
}

char *LaserPointsInfo::DeriveColourImageFileName(const char *name,
                                                 const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".colour.xv");
  imaged_colour.SetImageFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveColourGridFileName(const char *name,
                                                const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".colour.grid");
  imaged_colour.SetGridFile(filename);
  return(filename);
}

char *LaserPointsInfo::DeriveColourGridFileName()
{
  return(imaged_colour.DeriveGridFileName());
}

char *LaserPointsInfo::DerivePulseLengthImageFileName(const char *name,
                                                      const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".pulselength.xv");
  imaged_pulselength.SetImageFile(filename);
  return(filename);
}

char *LaserPointsInfo::DerivePulseLengthGridFileName(const char *name,
                                                     const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".pulselength.grid");
  imaged_pulselength.SetGridFile(filename);
  return(filename);
}

char *LaserPointsInfo::DerivePulseLengthGridFileName()
{
  return(imaged_pulselength.DeriveGridFileName());
}

char *LaserPointsInfo::DerivePointCountImageFileName(const char *name,
                                                     const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".pointcount.xv");
  imaged_pointcount.SetImageFile(filename);
  return(filename);
}

char *LaserPointsInfo::DerivePointCountGridFileName(const char *name,
                                                    const char *directory)
{
  char *filename;

  filename = ComposeFileName(directory, name, ".pointcount.grid");
  imaged_pointcount.SetGridFile(filename);
  return(filename);
}

char *LaserPointsInfo::DerivePointCountGridFileName()
{
  return(imaged_pointcount.DeriveGridFileName());
}
