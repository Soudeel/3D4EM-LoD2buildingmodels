
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
 Collection of functions for class Planes          

 Planes &Planes::operator =(const Planes &)    Copy assignment
 
 Initial creation
 Author : George Vosselman
 Date   : 03-07-2000

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
#include "Planes.h"
#include "file_id.h"

/*
--------------------------------------------------------------------------------
                                 Copy assignment
--------------------------------------------------------------------------------
*/

Planes &Planes::operator = (const Planes &planes)
{
  // Check for self assignment
  if (this == &planes) return *this;
  if (!empty()) erase(begin(), end());
  if (!planes.empty()) insert(begin(), planes.begin(), planes.end());
  return(*this);
}

/*
--------------------------------------------------------------------------------
                                 Erase all planes
--------------------------------------------------------------------------------
*/

void Planes::Erase()
{
  for (Planes::iterator plane=begin(); plane!=end(); plane++) plane->Erase();
  erase(begin(), end());
}

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
int Planes::Read(char *filename)
{
  char       *line;
  int        file_id, number, num_attributes, attribute, value, i;
  float      fvalue;
  double     x, y, z, d;
  Plane      plane;
  FILE       *fd;

  // Open the file
  if ((fd = Open_Compressed_File(filename, "r")) == NULL) {
    fprintf(stderr, "Error opening planes file %s\n", filename);
    return(0);
  }

  // Check the file id 
  line = (char *) malloc(MAXCHARS);
  fgets(line, MAXCHARS, fd);
  sscanf(line, "%d", &file_id);
  if (file_id != MAPPINGLIB_PLANES) {
    fprintf(stderr, "Given file (%s) does not contain planes!\n", filename);
    return(0);
  }

  // Read the positions
  while (fgets(line, MAXCHARS, fd)) {
    if (!Is_Comment(line)) {
      plane.Initialise();
      sscanf(line, "%d%d", &number, &num_attributes);
      plane.Number() = number;
	  fgets(line, MAXCHARS, fd);
      sscanf(line, "%lf%lf%lf%lf", &x, &y, &z, &d);
      plane.SetNormal(Vector3D(x, y, z));
      plane.SetDistance(d);
      for (i=0; i<num_attributes; i++) {
      	fgets(line, MAXCHARS, fd);
      	sscanf(line, "%d", &attribute);
      	if (AttributeType((PlaneTag) attribute) == IntegerAttributeType) {
      	  sscanf(line, "%d%d", &attribute, &value);
      	  plane.SetAttribute((PlaneTag) attribute, value);
      	}
      	else { // Float value
      	  sscanf(line, "%d%f", &attribute, &fvalue);
      	  plane.SetAttribute((PlaneTag) attribute, fvalue);
      	}
      }
      push_back(plane);
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
                        Write planes to a database
--------------------------------------------------------------------------------
*/
int Planes::Write(char *filename) const
{
  Planes::const_iterator plane;
  FILE                   *fd;
  const unsigned char    *tag;
  const int              *value;
  int                    i;
  
  // Open the file
  if ((fd = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Could not open %s to write 3D position database.\n", 
            filename);
    return(0);
  }

  // Write the header
  fprintf(fd, "%d %c file ID\n", MAPPINGLIB_PLANES, comment_char);
  fprintf(fd, "%c Number of planes: %d\n", comment_char, size());
  fprintf(fd, "%c Number   # Attributes\n", comment_char);
  fprintf(fd, "%c Normal vector and distance to origin\n", comment_char);
  fprintf(fd, "%c Attribute tag  Attribute value\n", comment_char);
  fprintf(fd, "%c -------------------------------------------------------------------\n",
	      comment_char);

  // Write the planes
  for (plane=begin(); plane!=end(); plane++) {
  	fprintf(fd, " %7d %7d\n", plane->Number(), plane->NumAttributes());
    fprintf(fd, " %15.9f  %15.9f  %15.9f  %15.6f\n",
	        plane->Normal().X(), plane->Normal().Y(), plane->Normal().Z(),
	        plane->Distance());
	for (i=0, tag=plane->AttributeTags(), value=plane->AttributeValues();
	     i<plane->NumAttributes(); i++, tag++, value++) {
	  switch (AttributeType((PlaneTag) *tag)) {
	    default:
	  	case IntegerAttributeType:
	      fprintf(fd, " %7d %7d\n", *tag, *value); break;
	  	case FloatAttributeType:
	      fprintf(fd, " %7d %15.7f\n", *tag, *((const float *) value)); break;
	  }
	}
  }

  // Close the file
  fclose(fd);
  return(1);
}


