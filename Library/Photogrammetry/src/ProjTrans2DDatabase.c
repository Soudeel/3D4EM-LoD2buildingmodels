
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

  Function      : Input and output routines for 2D projective transformation
  Author        : George Vosselman
  Creation date : 18 - May - 2002
  Modified by   : 
  Dated         :
  Modification  :
  
------------------------------o00--(_)--00o---------------------------------
                                _( O-O )_
                                  //|\\
*/

#include <stdio.h>
#include <stdlib.h>
#include "Database.h"

/*
----------------------------[ Get_ProjTrans2D ]--------------------------
  This function reads the file indicated by "filename", and puts the
  read info into an double array. This array should have been allocated
  in advance (minimum size 8 doubles).
----------------------------[ Get_ProjTrans2D ]--------------------------
*/

int Get_ProjTrans2D(const char *filename, double *par)
{
  FILE   *fd;
  int    i, file_id;
  char   *line; 
  
/* Open the file */
  
  fd = Open_Compressed_File(filename, "r");
  if (!fd) {
    fprintf(stderr, "Error opening file %s\n", filename);
    return(0);
  }
  
/* Check the file ID */
  
  line = (char *) malloc(1024);
  fgets(line, 1024, fd);
  sscanf(line, "%d", &file_id);
  if (file_id != PROJECTIVE_TRANSFORMATION) {
    fprintf(stderr, "Error: Wrong file ID.\n");
    fprintf(stderr, "File %s does not contain projective transformation parameters.\n", filename);
    Close_Compressed_File(fd); free(line);
    return(0);
  }

/* Read the parameters */

  i = 0;
  while (!feof(fd)) {
    fgets(line, 1024, fd);
    if (!feof(fd) && !Is_Comment(line) && i<=2) {
      switch (i) {
        case 0: /* Read the first three parameters */
          sscanf(line, "%lf %lf %lf", par, par+1, par+2);
          break;
        case 1: /* Read the second three parameters */
          sscanf(line, "%lf %lf %lf", par+3, par+4, par+5);
          break;
        case 2: /* Read the last two parameters */
          sscanf(line, "%lf %lf", par+6, par+7);
          break;
      }
      i++;
    }
  }

/* Close the file and return */

  Close_Compressed_File(fd); free(line);
  if (i != 3) {
    fprintf(stderr, "Error: Missing lines in file %s\n", filename);
    return(0);
  }
  return(1);
}

/*
----------------------------[ Put_ProjTrans2D ]--------------------------
  This function writes the transformtion parameters to a file.
----------------------------[ Put_ProjTrans2D ]--------------------------
*/

int Put_ProjTrans2D(const char *filename, const double *par)
{ 
  FILE *fd;

/* Open the file */

  fd = fopen(filename, "w"); 
  if (!fd) {
    fprintf(stderr, "Error opening file %s\n", filename);
    return(0);
  } 

/* Write the header */

  fprintf(fd, "%d  # File identifier\n", PROJECTIVE_TRANSFORMATION);
  fprintf(fd, "#\n# Parameters of a 2D to 2D projective transformation\n");
  fprintf(fd, "#      p0 * x + p1 * y + p2           p3 * x + p4 * y + p5\n");
  fprintf(fd, "# x' = --------------------      y' = --------------------\n");
  fprintf(fd, "#      p6 * x + p7 * y + 1            p6 * x + p7 * y + 1\n");
  fprintf(fd, "#\n");

/* Write the parameters */

  fprintf(fd, "%16.7lf  %16.7lf  %16.7lf   # p0, p1, p2\n",
          par[0], par[1], par[2]);
  fprintf(fd, "%16.7lf  %16.7lf  %16.7lf   # p3, p4, p5\n",
          par[3], par[4], par[5]);
  fprintf(fd, "%16.7lf  %16.7lf                     # p6, p7\n",
          par[6], par[7]);

/* Close and return */

  fclose(fd);
  return(1);
}
