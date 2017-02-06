
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

  Function      : Input and output routines for a grid specification data
		  including offset and scale of the imaged data
  Author        : George Vosselman
  Creation date : 12 - Mar - 1999
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Database.h"

/*--------------------- Header lines definition --------------------------*/
#define HEADER    "%c 3D grid specification\n\
%c ----------------------------------------------------------\n"

/*
----------------------------[ Get_Grid3D ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into a structure of type "Grid3D" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read parameters
----------------------------[ Get_Grid3D ]--------------------------
*/
Grid3D *Get_Grid3D(const char *filename)
{
  FILE   *fp;                   /* File Pointer                     */
  Grid3D *grid3D;               /* The database that is constructed */
  char   line[MAXCHARS];        /* Line read from intputfile        */
  int    file_id;               /* File identifier                  */
  int    i;

  if ((fp = Open_Compressed_File(filename, "r")) == NULL)
    {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return (NULL);
    }
  
/*------ Allocate memory for structure -----------------------------------*/
  grid3D = (Grid3D *) malloc(sizeof(Grid3D));

/*------ Read the file ---------------------------------------------------*/
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != GRID_3D && file_id != GRID) {
    printf("Error in file identifier\n");
    printf("This file ( %s ) does not contain a grid specification\n",filename);
    return( NULL );
  }

/*------ Read the file ---------------------------------------------------*/
  i = 0;
  while (!feof(fp)) {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line)) {
      switch (i) {
	case 0:
	  sscanf( line, "%lf", &(grid3D->x) );
	  break;
	case 1:
	  sscanf( line, "%lf", &(grid3D->y) );
	  break;
	case 2:
	  sscanf( line, "%lf", &(grid3D->pixelsize) );
	  break;
        case 3:
	  if (file_id == GRID_3D) 
	    sscanf( line, "%lf", &(grid3D->data_offset) );
	  break;
        case 4:
	  if (file_id == GRID_3D) 
	    sscanf( line, "%lf", &(grid3D->data_scale) );
	  break;
        default:
          break;
      }
      i++;
    }  
  }
  fclose(fp);

/*------ Use default data offset and scale for a 2D grid specification ------*/

  if (file_id == GRID) {
    grid3D->data_offset = 0;
    grid3D->data_scale  = 1;
  }

/*------ Print out a message that we are done reading --------------------*/

/*  printf( "3D grid specifications read from %s.\n", filename);*/
  return (grid3D);
}
  
/*
------------------------------[ Put_Grid3D ]--------------------------------
  This function writes the Grid3D structure into a file.
  
  IN : Structure containing read Grid3D
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
------------------------------[ Put_Grid3D ]--------------------------------
*/
int Put_Grid3D(const Grid3D *grid3D, const char *filename)
{
  FILE *fp;                        /* File Pointer */
  int i;

/*------ Open the file for writing ---------------------------------------*/
  if ((fp = fopen(filename, "w")) == NULL) {
    fprintf( stderr, "Could not open %s to write 3D grid specification database.\n", 
        filename);
    return (False);
  }
    
/*------ Print the headers and the information in the database file ------*/
  fprintf(fp, "%d # file identifier\n", GRID_3D);
  fprintf(fp, HEADER, comment_char, comment_char);
  
  fprintf(fp, "%15.3lf    %c X-coordinate of left-upper pixel\n",
          grid3D->x, comment_char);
  fprintf(fp, "%15.3lf    %c Y-coordinate of left-upper pixel\n",
          grid3D->y, comment_char);
  fprintf(fp, "%15.3lf    %c Pixel size\n",
          grid3D->pixelsize, comment_char);
  fprintf(fp, "%18.6lf %c Data offset\n",
          grid3D->data_offset, comment_char);
  fprintf(fp, "%18.6lf %c Data scale\n",
          grid3D->data_scale, comment_char);
  
/*------ Print out a message that we are done writing --------------------*/
/*  fprintf( stdout, "3D grid specification written to %s.\n", filename);*/ 
  fclose(fp);

  return (True);
}

/*
------------------------------[ Print_Grid3D ]------------------------------
  This function prints the Grid3D structure to stdout.

  IN : Structure containing read Grid3D 
  RETURN: Nothing
------------------------------[ Print_Grid3D ]------------------------------
*/
void Print_Grid3D(const Grid3D *grid3D)
{
  int i,j;
  
  printf("3D Grid specification\n\n");
  printf("Coordinates of upper-left pixel:\n");
  printf("X: %15.3lf\n", grid3D->x);
  printf("Y: %15.3lf\n", grid3D->y);
  printf("Pixel size : %15.3lf\n", grid3D->pixelsize);
  printf("Data offset: %18.6lf\n", grid3D->data_offset);
  printf("Data scale : %18.6lf\n\n", grid3D->data_scale);
}

Grid3D *Default_Grid3D()
{
  Grid3D *default_grid;

  default_grid = (Grid3D *) calloc(1, sizeof(Grid3D));
  default_grid->pixelsize = default_grid->data_scale = 1;
  
  return(default_grid);
}
