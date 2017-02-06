
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
 Collection of functions for class Positions3D

 int      Positions3D::Read(char *)          Read image points from a database
 int      Positions3D::Write(char *)         Write image points to a database
 
 Initial creation
 Author : George Vosselman
 Date   : 19-03-1999

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
#include "file_id.h"
#include "Positions3D.h"

#define MAXCHARS 256
/*
--------------------------------------------------------------------------------
                                Extern C functions
--------------------------------------------------------------------------------
*/

extern "C" FILE *Open_Compressed_File(const char *, const char *);
extern "C" int  Is_Comment(const char *);

/*
--------------------------------------------------------------------------------
                     Constructors / Destructors 
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                       Read positions from a database
--------------------------------------------------------------------------------
*/
int Positions3D::Read(char *filename)
{
  char       *line;
  int        file_id;
  Position3D pos3D;
  FILE       *fd;

/* Open the file */

  if ((fd = Open_Compressed_File(filename, "r")) == NULL) {
    fprintf(stderr, "Error opening 3D positions file %s\n", filename);
    return(0);
  }

/* Check the file id */

  line = (char *) malloc(MAXCHARS);
  fgets(line, MAXCHARS, fd);
  sscanf(line, "%d", &file_id);
  if (file_id != POSITIONS3D) {
    fprintf(stderr, "Given file (%s) does not contain 3D positions!\n",
	    filename);
    return(0);
  }

/* Read the positions */

  while (fgets(line, MAXCHARS, fd)) {
    if (!Is_Comment(line)) {
      pos3D = Position3D(line);
      push_back(pos3D);
    }
  }

  if (!feof(fd)) {
    fprintf(stderr, "Error while reading file %s\n", filename);
    free(line);
    return(0);
  }

  free(line);
  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write positions to a database
--------------------------------------------------------------------------------
*/
int Positions3D::Write(char *filename) const
{
  Positions3D::const_iterator i;
  Position3D pos3D;
  FILE       *fd;
  
/* Open the file */

  if ((fd = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Could not open %s to write 3D position database.\n", 
            filename);
    return(0);
  }

/* Write the header */

  fprintf(fd, "%d %c file ID\n", POSITIONS3D, comment_char);
  fprintf(fd, "%c Number of 3D positions: %d\n", comment_char, size());
  fprintf(fd, "%c        X                Y                Z\n", comment_char);
  fprintf(fd, "%c --------------- ---------------   ---------------\n",
	  comment_char);

/* Write the points */

  for (i=begin(); i!=end(); i++) {
    pos3D = *i;
    pos3D.Write(fd);
  }

/* Close the file */

  fclose(fd);
  return(1);
}
