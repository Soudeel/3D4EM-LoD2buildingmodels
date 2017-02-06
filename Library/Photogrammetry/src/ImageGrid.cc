
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
 Collection of functions for class ImageGrid

 void ImageGrid::Cpp2C(Grid **)        Conversion of C++ to C object
 void ImageGrid::C2Cpp(Grid *)         Conversion of C to C++ object
 int  ImageGrid::Read(char *)          Read image grid from a database
 int  ImageGrid::Write(char *)         Write image grid to a database
 void ImageGrid::Print()               Print image grid to stdout

 Initial creation
 Author : George Vosselman
 Date   : 08-06-98

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
#include "ImageGrid.h"

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ImageGrid::Cpp2C(Grid **gridptr) const
{
  Grid *grid;

/* Allocate space if this has not been done yet */

  grid = *gridptr;
  if (grid == NULL) {
    grid = (Grid *) malloc(sizeof(Grid));
    *gridptr = grid;
  }

/* Copy the data */

  grid->x         = x;
  grid->y         = y;
  grid->pixelsize = pixelsize;
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ImageGrid::C2Cpp(Grid *grid)
{
  x         = grid->x;
  y         = grid->y;
  pixelsize = grid->pixelsize;
}

/*
--------------------------------------------------------------------------------
                       Read image grid from a database
--------------------------------------------------------------------------------
*/
int ImageGrid::Read(const char *filename)
{
  Grid *grid;

  grid = Get_Grid(filename);          /* Read the database into C structure */
  if (grid == NULL) return(0);
  C2Cpp(grid);                        /* Convert to C++ object */
  free(grid);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write image grid to a database
--------------------------------------------------------------------------------
*/
int ImageGrid::Write(const char *filename) const
{
  int  error;
  Grid *grid;

  grid = NULL;
  Cpp2C(&grid);
  error = Put_Grid(grid, filename);
  free(grid);
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print image grid to stdout
--------------------------------------------------------------------------------
*/
void ImageGrid::Print() const
{
  Grid *grid;

  grid = NULL;
  Cpp2C(&grid);
  Print_Grid(grid);
  free(grid);
}
