
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
                                 _\\|//_ 
                                 ( O-O )
------------------------------o00--(_)--00o---------------------------------

  Function      : Input and output routines for 2D grid specification data
  Author        : George Vosselman
  Creation date : 25 - Sep - 1997
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c 2D grid specification\n\
%c ----------------------------------------------------------\n"

/*
----------------------------[ Get_Grid ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Grid" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read parameters
----------------------------[ Get_Grid ]--------------------------
*/
Grid *Get_Grid(const char *filename)
{
  FILE *fp;                   /* File Pointer                     */
  Grid *grid;             /* The database that is constructed */
  char line[MAXCHARS];        /* Line read from intputfile        */
  int  file_id;               /* File identifier                  */
  int  i;

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for structure -----------------------------------*/
  grid = (Grid *) malloc(sizeof(Grid));

/*------ Read the file ---------------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != GRID && file_id != GRID_3D) {
    printf("Error in file identifier\n");
    printf("This file ( %s ) does not contain a 2D grid specification\n", filename);
    return( NULL );
  }

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp)) {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line)) {
      switch (i) {
	case 0:
	  sscanf( line, "%lf", &(grid->x) );
	  break;
	case 1:
	  sscanf( line, "%lf", &(grid->y) );
	  break;
	case 2:
	  sscanf( line, "%lf", &(grid->pixelsize) );
	  break;
        default:
          break;
      }
      i++;
    }  
  }
    
  fclose(fp);

/*------ Print out a message that we are done reading --------------------*/
  printf( "2D grid specifications read from %s.\n", filename);
  
  return (grid);
}
  
/*
------------------------------[ Put_Grid ]--------------------------------
  This function writes the Grid structure into a file.
  
  IN : Structure containing read Grid
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Grid ]--------------------------------
*/
int Put_Grid(const Grid *grid, const char *filename)
{
  FILE *fp;                        /* File Pointer */
  int i;

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL) {
    fprintf( stderr, "Could not open %s to write 2D grid specification database.\n", 
        filename);
    return (False);
  }
    
/*------ Print the headers and the information in the database file ------*/
  fprintf(fp, "%d # file identifier\n", GRID);
  fprintf(fp, HEADER, comment_char, comment_char);
  
  fprintf(fp, "%15.3lf    %c X-coordinate of left-upper pixel\n",
          grid->x, comment_char);
  fprintf(fp, "%15.3lf    %c Y-coordinate of left-upper pixel\n",
          grid->y, comment_char);
  fprintf(fp, "%15.3lf    %c Pixel size\n",
          grid->pixelsize, comment_char);
  
/*------ Print out a message that we are done writing --------------------*/
  fprintf( stdout, "2D grid specification written to %s.\n", filename);
  fclose(fp);

  return (True);
}

/*
------------------------------[ Print_Grid ]------------------------------
  This function prints the Grid structure to stdout.

  IN : Structure containing read Grid 
  RETURN: Nothing
------------------------------[ Print_Grid ]------------------------------
*/
void Print_Grid(const Grid *grid)
{
  int i,j;
  
  printf("2D Grid specification\n\n");
  printf("Coordinates of upper-left pixel:\n");
  printf("X: %15.3lf\n", grid->x);
  printf("Y: %15.3lf\n\n", grid->y);
  printf("Pixel size: %15.3lf\n\n", grid->pixelsize);
}
