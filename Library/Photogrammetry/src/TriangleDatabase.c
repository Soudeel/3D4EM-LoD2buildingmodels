
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

  Function      : Input and output routines for TIN's
  Author        : George Vosselman
  Creation date : 17 - Oct - 1997
  
  Routines      : Put_Triangles
		  Get_Triangles

------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Database.h"

  
/*---------------------------[ Put_Triangles ]----------------------------------
  This function writes the Triangles structure into a file.
  
  IN : Structure containing the TIN data
  IN : Name of the file to write
  
  RETURN: True if succesfull, otherwise False.
-----------------------------[ Put_Triangles ]--------------------------------*/

int Put_Triangles(const Triangles *triangles, const char *filename)
{
  FILE           *fp;
  const Triangle *triangle;
  int            i;

/* Open the file for writing */

  if ((fp = fopen(filename, "w")) == NULL) {
    fprintf(stderr,"Could not open %s to write TIN.\n", filename);
    return(0);
  }
    
/* Print the header */

  fprintf(fp, "%d %c File identifier\n", TRIANGLES, comment_char);
  fprintf(fp, "%c Triangular Irregular Network data\n", comment_char);
  fprintf(fp, "%c Triangle Points                    Neighbouring Triangles\n",
	  comment_char);
  fprintf(fp, "%c Point 1    Point 2    Point 3      Triangle 1   Triangle 2    Triangle 3\n", comment_char);
  fprintf(fp, "%c ----------------------------------------------------------\n",
	  comment_char); 

/* Print the list of triangles */

  for (i=0, triangle=triangles->tri;
       i<triangles->num_tri;
       i++, triangle++)
    fprintf(fp, "%8d %8d %8d       %8d %8d %8d\n", 
	    triangle->node[0], triangle->node[1], triangle->node[2],
	    triangle->nb[0], triangle->nb[1], triangle->nb[2]);
  fclose(fp);
  
/* Print out a message that we are done writing  */

  printf("TIN data written to %s.\n", filename);
  return(1);
}

/*----------------------------[ Get_Triangles ]---------------------------------
  This function reads the file indicated by "filename", and puts the
  read triangles into a structure of type "Triangles" (see the header 
  file for a definition of this structure). If this function fails 
  then NULL is returned to the calling routine.
  
  IN : filename <- name of the file that is to be read
  RETURN: Structure containing the read triangles 
------------------------------[ Get_Triangles ]-------------------------------*/

Triangles *Get_Triangles(const char *filename)
{
  FILE *fp;                   /* File Pointer                      */
  int i, dimtri;              /* General counter                   */
  Triangles *triangles;       /* The database that is constructed  */
  char line[MAXCHARS];        /* Line read from intputfile         */
  int file_id;                /* Constant that identifies the file */

  if ((fp = Open_Compressed_File(filename, "r")) == NULL) {
    printf("Could not open database file \"%s\"\n", filename);
    return (NULL);
  }
  
/* Allocate and initialize Triangles structure */

  triangles = (Triangles *) malloc(sizeof(Triangles));
  triangles->num_tri = 0;
  triangles->tri     = NULL;
  dimtri = 0;
  
/*------ Check out the file's id -----------------------------------------*/

  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != TRIANGLES) {
    printf("Given file (%s) does not contain triangles!\n", filename);
    return( NULL );
  }
  
/*------ Read the file ---------------------------------------------------*/

  while (!feof(fp)) {
    fgets(line, MAXCHARS, fp);

    if (!Is_Comment(line)) {
      if (triangles->num_tri == dimtri) {
	dimtri += 1000;
	triangles->tri = (Triangle *) realloc(triangles->tri,
					      dimtri * sizeof(Triangle));
      }
      sscanf(line, "%d %d %d %d %d %d",
             &(triangles->tri[triangles->num_tri].node[0]),
             &(triangles->tri[triangles->num_tri].node[1]),
             &(triangles->tri[triangles->num_tri].node[2]), 
             &(triangles->tri[triangles->num_tri].nb[0]),
             &(triangles->tri[triangles->num_tri].nb[1]), 
             &(triangles->tri[triangles->num_tri].nb[2]));
      (triangles->num_tri)++; 
    }  
  }  
  (triangles->num_tri)--; 

/*------ Print out a message that we are done reading --------------------*/

  printf( "%d triangles read from %s.\n", triangles->num_tri, filename);
  
  fclose(fp);
  
  return (triangles);
}
  
/*
------------------------------[ Free_Triangles ]--------------------------------
  This function deallocates the memory used by Triangles.

  IN : Structure containing read Triangles 
  RETURN: Nothing
------------------------------[ Free_Triangles ]--------------------------------
*/

void Free_Triangles(Triangles *triangles)
{
   free(triangles->tri);
   free(triangles);
   triangles = NULL;
}	
