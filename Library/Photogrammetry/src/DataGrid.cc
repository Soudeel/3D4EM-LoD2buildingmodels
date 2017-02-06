
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
 Collection of functions for class DataGrid

 Initial creation
 Author : George Vosselman
 Date   : 12-03-1999

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
#include "DataGrid.h"

// Copy constructor

DataGrid& DataGrid::operator=(const DataGrid &grid)	
{
  // Check for self assignment
  if (this == &grid) return *this;
  ImageGridReference() = grid.ImageGridReference();
  data_offset = grid.data_offset;
  data_scale = grid.data_scale;
  return *this;
} 

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void DataGrid::Cpp2C(Grid3D **grid3Dptr) const
{
  Grid3D *grid3D;

/* Allocate space if this has not been done yet */

  grid3D = *grid3Dptr;
  if (grid3D == NULL) {
    grid3D = (Grid3D *) malloc(sizeof(Grid3D));
    *grid3Dptr = grid3D;
  }

/* Copy the data */

  grid3D->x           = x;
  grid3D->y           = y;
  grid3D->pixelsize   = pixelsize;
  grid3D->data_offset = data_offset;
  grid3D->data_scale  = data_scale;
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void DataGrid::C2Cpp(Grid3D *grid3D)
{
  x           = grid3D->x;
  y           = grid3D->y;
  pixelsize   = grid3D->pixelsize;
  data_offset = grid3D->data_offset;
  data_scale  = grid3D->data_scale;
}

/*
--------------------------------------------------------------------------------
                       Read data grid from a database
--------------------------------------------------------------------------------
*/
int DataGrid::Read(const char *filename)
{
  Grid3D *grid3D;

  grid3D = Get_Grid3D(filename);      /* Read the database into C structure */
  if (grid3D == NULL) return(0);
  C2Cpp(grid3D);                      /* Convert to C++ object */
  free(grid3D);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write data grid to a database
--------------------------------------------------------------------------------
*/
int DataGrid::Write(const char *filename) const
{
  int    error;
  Grid3D *grid3D;

  grid3D = NULL;
  Cpp2C(&grid3D);
  error = Put_Grid3D(grid3D, filename);
  free(grid3D);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print data grid to stdout
--------------------------------------------------------------------------------
*/
void DataGrid::Print() const
{
  Grid3D *grid3D;

  grid3D = NULL;
  Cpp2C(&grid3D);
  Print_Grid3D(grid3D);
  free(grid3D);
}


/*
--------------------------------------------------------------------------------
                          Convert data to a grey value
--------------------------------------------------------------------------------
*/
unsigned char DataGrid::GreyValue(double data) const
{
  double dgv;

  dgv = (data - data_offset) / data_scale;
  if (dgv < 0.0) return(0);
  else if (dgv > 255.0) return(255);
  else return((unsigned char) dgv);
}
