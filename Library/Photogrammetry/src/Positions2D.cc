
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
 Collection of functions for class Positions2D

 int Positions2D::Read(char *)                Read image points from a database
 int Positions2D::Write(char *)               Write image points to a database
 void Positions2D::RemoveDoublePoints(double) Remove points with similar X and Y
 
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
#include "Positions2D.h"

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
int Positions2D::Read(char *filename)
{
  char       *line;
  int        file_id;
  Position2D pos2D;
  FILE       *fd;

/* Open the file */

  if ((fd = Open_Compressed_File(filename, "r")) == NULL) {
    fprintf(stderr, "Error opening 2D positions file %s\n", filename);
    return(0);
  }

/* Check the file id */

  line = (char *) malloc(MAXCHARS);
  fgets(line, MAXCHARS, fd);
  sscanf(line, "%d", &file_id);
  if (file_id != POSITIONS2D) {
    fprintf(stderr, "Given file (%s) does not contain 2D positions!\n",
	    filename);
    return(0);
  }

/* Read the positions */

  while (fgets(line, MAXCHARS, fd)) {
    if (!Is_Comment(line)) {
      pos2D = Position2D(line);
      push_back(pos2D);
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
int Positions2D::Write(char *filename) const
{
  Positions2D::const_iterator i;
  Position2D pos2D;
  FILE       *fd;
  
/* Open the file */

  if ((fd = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Could not open %s to write 2D position database.\n", 
            filename);
    return(0);
  }

/* Write the header */

  fprintf(fd, "%d %c file ID\n", POSITIONS2D, comment_char);
  fprintf(fd, "%c Number of 2D positions: %d\n", comment_char, size());
  fprintf(fd, "%c        X                Y\n", comment_char);
  fprintf(fd, "%c ---------------  ---------------\n", comment_char);

/* Write the points */

  for (i=begin(); i!=end(); i++) {
    pos2D = *i;
    pos2D.Write(fd);
  }

/* Close the file */

  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Remove double points
--------------------------------------------------------------------------------
*/
void Positions2D::RemoveDoublePoints(double distance)
{
  Positions2D::iterator pt1, pt2, lastgoodpt;

  lastgoodpt = end() - 1;
  for (pt1=begin(); pt1<=lastgoodpt; pt1++) {
    for (pt2=pt1+1; pt2<=lastgoodpt; pt2++) {
      if (DistancePosition2DToPosition2D(pt1->Position2DRef(),
                                         pt2->Position2DRef()) < distance) {
        *pt2 = *lastgoodpt;
        lastgoodpt--;
        pt2--;
      }
    }
  }
  if (lastgoodpt != end() - 1) erase(lastgoodpt+1, end());
}
