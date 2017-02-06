
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
 Collection of functions for class LaserScanLines

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
#include "LaserDataTypes.h"
#include "LaserScanLines.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                           Copy assignment
--------------------------------------------------------------------------------
*/

LaserScanLines LaserScanLines::operator = (const LaserScanLines &l)
{
  if (this == &l) return *this;  // Check for self assignment
  if (!empty()) erase(begin(),end());
  if (!l.empty()) insert(begin(), l.begin(), l.end());
  SetScanLinesFile(l.scanlines_file);
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Initialise the scan lines information
--------------------------------------------------------------------------------
*/

void LaserScanLines::Initialise()
{
  scanlines_file = NULL;
}

void LaserScanLines::ReInitialise()
{
  if (!empty()) erase(begin(), end());
  if (scanlines_file) {free(scanlines_file);  scanlines_file = NULL;}
}

/*
--------------------------------------------------------------------------------
                          Read scan lines from file
--------------------------------------------------------------------------------
*/

int LaserScanLines::Read(const char *filename)
{
  if (scanlines_file) free(scanlines_file);
  scanlines_file = (char *) malloc(strlen(filename) + 1);
  strcpy(scanlines_file, filename);
  return(Read());
}


int LaserScanLines::Read()
{
  int     file_id, num_bytes, num_lines, *endings, i, num_add, *ending,
	  num_alloc, first_number;
  FILE    *fd;
  char    *reserved_space;

/* Open the scan line file */
#ifdef windows
  if ((fd = Open_Compressed_File(scanlines_file, "rb")) == NULL) {
#else
  if ((fd = Open_Compressed_File(scanlines_file, "r")) == NULL) {
#endif
    fprintf(stderr, "Could not open laser scan line file %s\n", scanlines_file);
    return(0);
  }
  fread(&file_id, sizeof(int), 1, fd);  num_bytes = sizeof(int);
  if (file_id != LASER_SCANLINES) {
    fprintf(stderr, "Error: File %s does not contain laser scan lines\n",
	    scanlines_file);
    return(0);
  }

/* Clear old data */

  if (!empty()) erase(begin(), end());

/* Read the number of scan lines */

  fread(&num_lines, sizeof(int), 1, fd);  num_bytes += sizeof(int);
  
/* Read the rest of the 1024 byte header */

  num_bytes = 1024 - num_bytes;
  reserved_space = (char *) malloc(num_bytes);
  fread(reserved_space, sizeof(char), num_bytes, fd);
  free(reserved_space);

/* Memory allocation */

  num_alloc = 1000;
  if (num_lines < num_alloc) num_alloc = num_lines;
  endings = (int *) malloc(sizeof(int) * num_alloc);
  if (endings == NULL) {
    fprintf(stderr, "Error allocating memory for laser scan lines\n");
    return(0);
  }
  reserve(num_lines);

/* Read the data */

  first_number = 0;
  while (!feof(fd)) {
    num_add = fread(endings, sizeof(int), num_alloc, fd);
    for (i=0, ending=endings;
	 i<num_add;
	 i++, ending++) {
      if (i == 0) push_back(LaserScanLine(first_number, *ending));
      else push_back(LaserScanLine(*(ending-1)+1, *ending));
    }
    if (num_add) first_number = endings[num_add-1]+1;
  }
  free(endings);

/* Close the file and return success */

  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Write scan lines to file
--------------------------------------------------------------------------------
*/

int LaserScanLines::Write(const char *filename, int compress)
{
  if (scanlines_file) free(scanlines_file);
  scanlines_file = (char *) malloc(strlen(filename) + 1);
  strcpy(scanlines_file, filename);
  return(Write(compress));
}

int LaserScanLines::Write() const  /* Write compressed by default */
{
  return(Write(1));
}

int LaserScanLines::Write(int compress) const
{
  LaserScanLines::const_iterator scanline;
  int     file_id, num_bytes, num_lines, *endings, i, *ending, num_alloc;
  FILE    *fd;
  char    *reserved_space, *command;

/* Open the scan line file */

#ifdef windows
  if ((fd = fopen(scanlines_file, "wb")) == NULL) {
#else
  if ((fd = fopen(scanlines_file, "w")) == NULL) {
#endif
    fprintf(stderr, "Could not open laser scan line file %s for writing\n",
	    scanlines_file);
    return(0);
  }

/* Write the header information */

  file_id = LASER_SCANLINES;
  fwrite((const void *) &file_id, sizeof(int), 1, fd);  num_bytes = sizeof(int);
  num_lines = size();
  fwrite((const void *) &num_lines, sizeof(int), 1, fd);
  num_bytes += sizeof(int);

/* Reserve the rest of the 1024 byte header for future use */

  num_bytes = 1024 - num_bytes;
  reserved_space = (char *) malloc(num_bytes);
  fwrite((const void *) reserved_space, sizeof(char), num_bytes, fd);
  free(reserved_space);

/* Memory allocation */

  num_alloc = 1000;
  if (num_lines < num_alloc) num_alloc = num_lines;
  endings = (int *) malloc(sizeof(int) * num_alloc);
  if (endings == NULL) {
    fprintf(stderr, "Error allocating memory for laser scan lines\n");
    return(0);
  }

/* Write the data */

  for (scanline=begin(), i=0, ending=endings;
       scanline!=end();
       scanline++, i++, ending++) {
    if (i == num_alloc) {
      fwrite((const void *) endings, sizeof(int), num_alloc, fd);
      i = 0;
      ending = endings;
    }
    *ending = scanline->End().Number();
  }
  fwrite((const void *) endings, sizeof(int), i, fd);
  free(endings);

/* Close the file */

  fclose(fd);

/* Compress the data if required */

  if (compress) {
    command = (char *) malloc((9+strlen(scanlines_file)) * sizeof(char));
    strcpy(command, "gzip -f ");
    strcat(command, scanlines_file);
    system(command);
    free(command);
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Print scan lines to screen
--------------------------------------------------------------------------------
*/

void LaserScanLines::Print() const
{
  LaserScanLines::const_iterator scanline;

  for (scanline=begin(); scanline!=end(); scanline++) scanline->Print();
}

/*
--------------------------------------------------------------------------------
                          Set the scan lines file name
--------------------------------------------------------------------------------
*/

void LaserScanLines::SetScanLinesFile(const char *filename)
{
  StringCopy(&scanlines_file, filename);
}

/*
--------------------------------------------------------------------------------
        Simple scan line matching (used for indoor mapping) based on time
		difference, angle, distance and overlap
--------------------------------------------------------------------------------
*/
LaserScanLines::iterator LaserScanLines::MatchingScanLine(
         const LaserPoints &points, const LaserScanLine &new_scanline,
		 double max_angle, double max_dist, double min_overlap,
		 double max_time_diff, const Position3D &reference_pos)
{
  LaserScanLines::iterator old_scanline;
  Line3D                   new_line, old_line;
  double                   new_start_scalar, new_end_scalar, angle, distance,
                           old_start_scalar, old_end_scalar, scalar,
						   pi = 4.0 * atan(1.0), smallest_angle, smallest_distance,
						   largest_overlap;
  bool                     debug = false, print_report=false;
  char                     report[1024];
  Position3D               reference_projected;
  LaserPoints::const_iterator point1, point2;
  
  // Construct new 3D line
  new_line = Line3D(new_scanline.begin(points)->Position3DRef(),
                    new_scanline.end(points)->Position3DRef());
  new_start_scalar = new_line.Scalar(new_scanline.begin(points)->Position3DRef());                    
  new_end_scalar   = new_line.Scalar(new_scanline.end(points)->Position3DRef());    
  if (new_start_scalar > new_end_scalar) {
  	scalar           = new_start_scalar;
  	new_start_scalar = new_end_scalar;
  	new_end_scalar   = scalar;
  }
  reference_projected = new_line.Project(reference_pos);
  
  // Match with the old scan lines
  if (debug || print_report) {
    report[0] = 0;
    smallest_angle = 2 * pi;
    smallest_distance = 1.0e10;
    largest_overlap = 0.0;
  }
  for (old_scanline=begin(); old_scanline!=end(); old_scanline++) {
  	// Check time difference
  	if (new_scanline.begin(points)->DoubleAttribute(TimeTag) -
	    old_scanline->begin(points)->DoubleAttribute(TimeTag) >
		max_time_diff) {
	  if (debug || print_report) {
        strcat(report, "t");
	  }
	  continue;
	}
  	// Construct old 3D line
  	old_line = Line3D(old_scanline->begin(points)->Position3DRef(),
                      old_scanline->end(points)->Position3DRef());
    // Check angle
    angle = Angle2Lines(new_line, old_line);
    if (angle > pi / 2.0) angle = pi - angle;
    if (angle > max_angle) {
      if (debug || print_report) {
        strcat(report, "a");
        if (angle < smallest_angle) smallest_angle = angle;
      }
      continue;
    }
    
    // Check distance. 
    distance = old_line.DistanceToPoint(reference_projected);
    if (distance > max_dist) {
      if (debug || print_report) {
	    strcat(report, "d");
        if (distance < smallest_distance) smallest_distance = distance;
      }
      continue;
    }
    
    // Check overlap (on the new line)
    old_start_scalar = new_line.Scalar(old_scanline->begin(points)->Position3DRef());                    
    old_end_scalar   = new_line.Scalar(old_scanline->end(points)->Position3DRef());                    
    if (old_start_scalar > old_end_scalar) {
      scalar           = old_start_scalar;
  	  old_start_scalar = old_end_scalar;
  	  old_end_scalar   = scalar;
    }                
    if (new_start_scalar > old_start_scalar) old_start_scalar = new_start_scalar;
    if (new_end_scalar < old_end_scalar) old_end_scalar = new_end_scalar;
    if (old_end_scalar - old_start_scalar < min_overlap) {
      if (debug || print_report) {
        strcat(report, "o");
        if (old_end_scalar - old_start_scalar > largest_overlap)
          largest_overlap = old_end_scalar - old_start_scalar;
      }
      continue;
    }
    
    // Found a good scan line, return iterator
    if (debug) {
      point1 = new_scanline.begin(points);
  	  point2 = new_scanline.end(points);
  	  printf("GOOD MATCH\n");
  	  printf("new line %6.2f %6.2f angle %6.2f mid %6.2f %6.2f\n", 
  	         point2->X() - point1->X(), point2->Y() - point1->Y(),
  	         atan2(point2->X() - point1->X(), point2->Y() - point1->Y()) *
  	         180.0 / pi,
			 (point1->X() + point2->X()) / 2.0,
			 (point1->Y() + point2->Y()) / 2.0);
  	  point1 = old_scanline->begin(points);
  	  point2 = old_scanline->end(points);
  	  printf("old line %6.2f %6.2f angle %6.2f mid %6.2f %6.2f\n", 
  	         point2->X() - point1->X(), point2->Y() - point1->Y(),
  	         atan2(point2->X() - point1->X(), point2->Y() - point1->Y()) *
  	         180.0 / pi,
			 (point1->X() + point2->X()) / 2.0,
			 (point1->Y() + point2->Y()) / 2.0);
      printf("angle %6.2f", angle * 180.0 / pi);
      printf(" distance %6.2f", distance);
      printf(" overlap %6.2f\n\n\n", old_end_scalar - old_start_scalar);    	
    }
    return old_scanline;
  }
  
  // No matching scan line found.
  if (print_report) {
    printf("%s angle %.2f dist %.2f overlap %.2f time %.5f\n", report,
           smallest_angle * 180.0 / pi, smallest_distance, largest_overlap,
		   new_scanline.begin(points)->DoubleAttribute(TimeTag));
  }
  if (debug) {
    printf("NO MATCH\n");
    // output all results		
  	point1 = new_scanline.begin(points);
  	point2 = new_scanline.end(points);
  	printf("new line %6.2f %6.2f angle %6.2f mid %6.2f %6.2f\n", 
  	       point2->X() - point1->X(), point2->Y() - point1->Y(),
  	       atan2(point2->X() - point1->X(), point2->Y() - point1->Y()) *
  	       180.0 / pi,
		   (point1->X() + point2->X()) / 2.0,
		   (point1->Y() + point2->Y()) / 2.0);
    for (old_scanline=begin(); old_scanline!=end(); old_scanline++) {
      // Construct old 3D line
  	  old_line = Line3D(old_scanline->begin(points)->Position3DRef(),
                        old_scanline->end(points)->Position3DRef());
  	  point1 = old_scanline->begin(points);
  	  point2 = old_scanline->end(points);
  	  printf("old line %6.2f %6.2f angle %6.2f mid %6.2f %6.2f\n", 
  	         point2->X() - point1->X(), point2->Y() - point1->Y(),
  	         atan2(point2->X() - point1->X(), point2->Y() - point1->Y()) *
  	         180.0 / pi,
			 (point1->X() + point2->X()) / 2.0,
			 (point1->Y() + point2->Y()) / 2.0);
    
      // Check angle
      angle = Angle2Lines(new_line, old_line);
      if (angle > pi / 2.0) angle = pi - angle;
      printf("angle %6.2f", angle * 180.0 / pi);
    
      // Check distance. 
      printf(" distance %6.2f", distance);
 
      // Check overlap (on the new line)
      old_start_scalar = new_line.Scalar(old_scanline->begin(points)->Position3DRef());                    
      old_end_scalar   = new_line.Scalar(old_scanline->end(points)->Position3DRef());                    
      if (old_start_scalar > old_end_scalar) {
        scalar           = old_start_scalar;
  	    old_start_scalar = old_end_scalar;
  	    old_end_scalar   = scalar;
      }                
      if (new_start_scalar > old_start_scalar) old_start_scalar = new_start_scalar;
      if (new_end_scalar < old_end_scalar) old_end_scalar = new_end_scalar;
      printf(" overlap %6.2f\n", old_end_scalar - old_start_scalar);
    }
    printf("\n\n");
  }
  
  return end();
}

