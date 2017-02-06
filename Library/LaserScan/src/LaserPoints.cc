
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
 Collection of functions for class LaserPoints
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/
#include <cstdlib>
#include <iostream>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include "LaserPoints.h"
#include "Line2D.h"
#include "BNF_io.h"
#include "VRML_io.h"
#include "triangle.h"
#include "viff.h"
#include "compression.h"
#include "stdmath.h"
#include "digphot_arch.h"
#include "TextFileUtilities.h"
#include "KNNFinder.h"
#include "ProgressDisplay.h"
#include "Histo3D.h"
#include "StlUtilities.h"
#include "GeneralUtility.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <errno.h>


#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines
using namespace NEWMAT;              // access NEWMAT namespace
#define SH cerr<<__FILE__<<": "<<__LINE__<<endl<<flush;	
/*
--------------------------------------------------------------------------------
                  Definition of current and old storage types
--------------------------------------------------------------------------------
*/

typedef struct {         // File storage format version 2
  float x, y, z;
  int r, pc;
} StoredLaserPointV2;

typedef struct {         // File storage format version 1
  float x, y, z;
  unsigned char r, pc;
} StoredLaserPointV1;

typedef struct {         // File storage format version 0
  float x, y, z;
  unsigned char r;
} StoredLaserPointV0;

/*
--------------------------------------------------------------------------------
                            Destructor
--------------------------------------------------------------------------------
*/

LaserPoints::~LaserPoints()
{
  // Remove all attributes
  RemoveAttributes();
  
  // Delete all points
  if (!empty()) erase(begin(), end());

  // Invalidate the old bounds
  bounds.Initialise();
  
  // Delete TIN
  if (tin) tin->Erase();
}

/*
--------------------------------------------------------------------------------
                            Copy assignment
--------------------------------------------------------------------------------
*/

LaserPoints & LaserPoints::operator = (const LaserPoints &pts)
{
  if (this == &pts) return *this;  // Check for self assignment
  // Warning: copy assignment of LaserPoints in part only copies pointers!!!
  if (!empty()) erase(begin(), end());
  if (!pts.empty()) insert(begin(), pts.begin(), pts.end());
  LaserPointsInfoReference() = pts.LaserPointsInfoReference();
  LaserDataFilesReference() = pts.LaserDataFilesReference();
  tin = pts.tin;
  nbh_edges = pts.nbh_edges;
  seek_offset = pts.seek_offset;
  highest_surface_number = pts.highest_surface_number;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                       Initialisation of class LaserPoints
--------------------------------------------------------------------------------
*/

void LaserPoints::Initialise()
{
  LaserPointsInfo::Initialise();        /* Initialise info class members      */
  LaserDataFiles::Initialise();         /* Initialise file names              */
  tin = NULL;                           /* Initialise TIN                     */
  nbh_edges = NULL;                  // Initialisation of neighbourhood edges
  seek_offset = -1;                  // No multiple point sets in the point file
  highest_surface_number = -1;       // No highest surface number recorded
}

void LaserPoints::ReInitialise()
{
  if (!empty()) erase(begin(), end());  /* Delete old laser points            */
  LaserPointsInfo::ReInitialise();      /* Initialise info class members      */
  LaserDataFiles::ReInitialise();       /* Initialise file names              */
  if (tin) tin->~TIN(); tin = NULL;     /* Initialise TIN                     */
  nbh_edges = NULL;                  // No erase as edges may have been handled outside class
  seek_offset = -1;                  // No multiple point sets in the point file
  highest_surface_number = -1;       // No highest surface number recorded
}

/*
--------------------------------------------------------------------------------
                       Read the laser points meta data
--------------------------------------------------------------------------------
*/

int LaserPoints::ReadMetaData()
{
  return(ReadMetaData(meta_file));
}

int LaserPoints::ReadMetaData(const char *filename)
{
  FILE       *fd;
  char       *buffer, *line, *keyword;
  int        keyword_length;

/* Open the file */

  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening laser points meta data file%s\n", filename);
    exit(0);
  }

/* Verify that the first keyword is "laserpointset" */

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading laser points meta data from file %s\n",
            filename);
    Close_Compressed_File(fd);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "laserpointset", MAX(keyword_length, 13))) {
    fprintf(stderr,"Error: File %s does not contain laser points meta data .\n",
            filename);
    fprintf(stderr, "       First keyword is %s, not laserpointset.\n",
	    keyword);
    return(0);
  }

/* Initialise all meta data */

  Initialise();

/* Process all lines */

  SetMetaDataFile(filename);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {

	if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
	  name = BNF_String(line);

	else if (!strncmp(keyword, "points", MAX(keyword_length, 6)))
	  point_file = BNF_String(line);

	else if (!strncmp(keyword, "tin", MAX(keyword_length, 3)))
	  tin_file = BNF_String(line);

    else if (!strncmp(keyword, "seek_offset", MAX(keyword_length, 11)))
      seek_offset = BNF_LongInteger(line);

    else if (!strncmp(keyword, "highest_surface_number", MAX(keyword_length, 22)))
      highest_surface_number = BNF_Integer(line);

	else if (!strncmp(keyword, "info", MAX(keyword_length, 4)))
	  LaserPointsInfo::ReadMetaData(fd);

	else if (!strncmp(keyword, "endlaserpointset",MAX(keyword_length,16))) {
	  free(buffer);
          Close_Compressed_File(fd);
          return(1);
        }
	else {
          keyword[keyword_length] = 0;
	  fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserpointset.\n");
  Close_Compressed_File(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the laser points meta data to file
--------------------------------------------------------------------------------
*/

int LaserPoints::WriteMetaData() const
{
  return(WriteMetaData(meta_file));
}

int LaserPoints::WriteMetaData(const char *filename) const
{
  FILE       *fd;
  int        indent;

/* Open the file */

  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening laser points meta data file %s\n", filename);
    return(0);
  }

/* Write the meta data */

  indent = 0;
  BNF_Write_String(fd, "laserpointset", indent, NULL);  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (point_file) BNF_Write_String(fd, "points", indent, point_file);
  if (seek_offset != -1)
    BNF_Write_LongInteger(fd, "seek_offset", indent, seek_offset, "%I64d");
  if (highest_surface_number != -1)
    BNF_Write_Integer(fd, "highest_surface_number", indent,
	                  highest_surface_number, "%d");
  if (tin_file) BNF_Write_String(fd, "tin", indent, tin_file);
  LaserPointsInfo::WriteMetaData(fd, indent);
  indent -= 2;
  BNF_Write_String(fd, "endlaserpointset", 0, NULL);

/* Close the file and return */

  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                          Derive meta data file name
--------------------------------------------------------------------------------
*/

char *LaserPoints::DeriveMetaDataFileName(const char *directory)
{
  return(LaserDataFiles::DeriveMetaDataFileName(directory, ".pointset"));
}

/*
--------------------------------------------------------------------------------
                    Read laser altimetry points from file
--------------------------------------------------------------------------------
*/

int LaserPoints::Read()
{
  return(Read(point_file, false));
}

int LaserPoints::ReadHeader(FILE *fd, int *num_pts, double *x_offset,
                            double *y_offset, double *z_offset,
                            unsigned char *max_num_attributes)
{
  int  file_id, type, num_bytes, offset=0, i;
  char *header, *ch;

  // Position stream, if required
  if (seek_offset != -1) {
    if (fseeko64(fd, (off64_t) seek_offset, SEEK_SET) != 0) {
      printf("Error applying seek offset for reading in %s\n", point_file);
      printf("errno %d\n", errno);
      return -1;
    }
  }

  // Read all header bytes  
  header = (char *) malloc(1025 * sizeof(char));
  if (header == NULL) {
    printf("LaserPoints::ReadHeader : Error allocating header memory\n");
    return -1;
  }
  num_bytes = fread(header, sizeof(char), 1024, fd);
  if (num_bytes != 1024) {
    printf("Error reading header of laser points file %s\n", PointFile());
    printf("%d bytes read instead of expected 1024 bytes\n", num_bytes);    
    return(-1);
  }
  
  // Check file id
  memcpy((void *) &file_id, (const void *) header, sizeof(int));
  offset += sizeof(int);
  if (file_id != LASER_POINTS_V1 && file_id != LASER_POINTS_V2 &&
      file_id != LASER_POINTS_V3) {
    if (file_id == LASER_LAS) return LASER_LAS;
    printf("Error in file id of point file %s\n", PointFile());
 	return(-1);
  }

  // Extract other header information
  memcpy((void *) num_pts, (const void *) (header+offset), sizeof(int));
  offset += sizeof(int);
  memcpy((void *) &type, (const void *) (header+offset), sizeof(int));
  offset += sizeof(int);
  scanner.SetPointType((LaserPointType) type);
  memcpy((void *) x_offset, (const void *) (header+offset), sizeof(double));
  offset += sizeof(double);
  memcpy((void *) y_offset, (const void *) (header+offset), sizeof(double));
  offset += sizeof(double);
  memcpy((void *) z_offset, (const void *) (header+offset), sizeof(double));
  offset += sizeof(double);
  if (file_id == LASER_POINTS_V3) {
    memcpy((void *) max_num_attributes, (const void *) (header+offset),
           sizeof(unsigned char));
    offset += sizeof(unsigned char);
  }
  else
    *max_num_attributes = 2;

  free(header);
    
  return(file_id);
}

int LaserPoints::Read(const char *filename, bool verbose, int max_num_pts)
{
  FILE *fd;
  int  ret_val;
    
  // Set new point file if required
  if (filename != point_file && filename != NULL) {
    if (point_file) free(point_file);
    point_file = (char *) malloc(strlen(filename) + 1);
    strcpy(point_file, filename);
  }

  // Open the file
#ifdef windows
  if ((fd = Open_Compressed_File(point_file, "rb")) == NULL) {
#else
  if ((fd = Open_Compressed_File(point_file, "r")) == NULL) {
#endif
    fprintf(stderr, "Could not open laser points file %s\n", point_file);
    return(0);
  }

  // Clear the old data
  if (!empty()) erase(begin(), end());

  // Read the file and close it
  ret_val = Read(fd, verbose, max_num_pts);
  Close_Compressed_File(fd);
  
  // Read LAS file
  if (ret_val == LASER_LAS) 
    ret_val = ReadLAS(point_file, verbose, max_num_pts);
    
  return ret_val;
}

int LaserPoints::Read(FILE *fd, bool verbose, int max_num_pts)
{
  double           x_offset, y_offset, z_offset;
  int              file_id, num_pts, ret_val;
  unsigned char    max_num_attributes;

// Read the header information

  file_id = ReadHeader(fd, &num_pts, &x_offset, &y_offset, &z_offset,
                       &max_num_attributes);
  if (file_id == -1) return 0;

// Switch to the appropriate reading routine

  switch (file_id) {
    case 0: // No file id, might be the very first version
      Close_Compressed_File(fd);
      return(ReadV0(verbose));

    case LASER_POINTS_V1: // File format version 1
      return(ReadV1(fd, num_pts, x_offset, y_offset, z_offset, verbose));

    case LASER_POINTS_V2: // File format version 2
      return(ReadV2(fd, num_pts, x_offset, y_offset, z_offset, verbose));

    case LASER_POINTS_V3: // File format version 3
      ret_val = ReadV3(fd, num_pts, x_offset, y_offset, z_offset,
                       max_num_attributes, max_num_pts);
      if (verbose) printf("%d laser points read, file format 3.\n", size());
      return ret_val;
      
    case LASER_LAS:
      return LASER_LAS;

    default: // Should not happen -- Changed by tahir to switch to ascii mode if all else fails
	{
      //fprintf(stderr, "Error, unrecognized file format.\n");
	  Close_Compressed_File(fd);
	  fprintf(stderr, "Warning: File %s has no laser points file ID.\n",point_file);
	  fprintf(stderr, "Switching to ascii mode...\n");
	  this->LoadFromAscii(point_file);
	 }
  }
}

int LaserPoints::ReadLAS(const char *filename, bool verbose, int max_num_pts)
{
  int           max_num, num_read, rgb_scale;
  LASreadOpener lasreadopener;
  LASheader     lasheader;
  LASreader     *lasreader;
  LaserPoint    point;

  // Set maximum number of points to read
  if (max_num_pts == -1) max_num = INT_MAX;
  else max_num = max_num_pts;
  
  // Open the LAS file
  lasreadopener.set_file_name(filename);
  lasreader = lasreadopener.open();
  if (lasreader == 0) {
    fprintf(stderr, "ERROR: could not open LAS file %s\n", filename);
    return 0;
  }

  // Read the points
  num_read = 0;
  while (lasreader->read_point() && num_read < max_num) {
  	num_read++;
  	point.X() = lasreader->point.get_x();
  	point.Y() = lasreader->point.get_y();
  	point.Z() = lasreader->point.get_z();
  	if (lasreader->point.number_of_returns_of_given_pulse) {
  	  point.Attribute(PulseCountTag) = lasreader->point.return_number;
  	  point.SetLastPulseFlag(lasreader->point.number_of_returns_of_given_pulse ==
		                     lasreader->point.return_number);
  	}
  	if (lasreader->point.have_rgb) {
  	  if (num_read == 1) {
  	    if (lasreader->point.rgb[0] > 255 || lasreader->point.rgb[1] > 255 ||
		    lasreader->point.rgb[2] > 255) rgb_scale = 256;
		else rgb_scale = 1;
	  }
  	  point.SetColour(lasreader->point.rgb[0] / rgb_scale,
		              lasreader->point.rgb[1] / rgb_scale,
		              lasreader->point.rgb[2] / rgb_scale);
  	}
	if (lasreader->point.have_gps_time)
	  point.SetDoubleAttribute(TimeTag, lasreader->point.gps_time);
	push_back(point);
  }

  // Close the LAS file
  lasreader->close();
  delete lasreader;
  
  if (verbose) printf("%d points read from LAS file %s\n", size(), filename);
  return 1;
}

int LaserPoints::ReadV3(FILE *fd, int num_pts, double x_offset,
                        double y_offset, double z_offset,
                        unsigned char max_num_attributes,
                        int max_num_pts)
{
  int                num_alloc, max_point_size, buffer_size, num_bytes_left,
                     num_bytes, index, num_attributes, attribute_value, i;
  unsigned char      *buffer, attribute_tag;
  float              coordinates[3];
  LaserPoints::iterator point;

// Memory allocation

  if (max_num_pts > -1 && max_num_pts < num_pts) num_pts = max_num_pts;
  num_alloc = 500000;
  if (num_pts < num_alloc) num_alloc = num_pts + 1;
  max_point_size = 3 * sizeof(float) +
                   (max_num_attributes + 1) * sizeof(unsigned char) +
                   max_num_attributes * sizeof(int);
  buffer_size = num_alloc * max_point_size;
  buffer = (unsigned char *) malloc(buffer_size);
  if (!buffer) {
  	printf("\nError allocating buffer of size %d in LaserPoints::ReadV3\n",
	       buffer_size);
    exit(0);
  }
  reserve(num_pts); /* Allocate vector space */
  
// Read the data
  
  num_bytes_left = 0;
  while (!feof(fd) && size() < num_pts) {
    // Read data into the buffer
    num_bytes = fread(buffer + num_bytes_left, sizeof(unsigned char),
                      buffer_size - num_bytes_left, fd) + num_bytes_left;
    if (num_bytes == 0) {
      printf("\nNo bytes could be read, but still %d points to go\n", num_pts - size());
      printf("ferror returns %d\n", ferror(fd));
      perror("perror output:");
      exit(0);
    }
    index = 0;
    // Extract points from the buffer
//    while ((feof(fd) && size() != num_pts) ||
//           index < num_bytes - max_point_size) {
    while (size() < num_pts &&
           (feof(fd) || index < num_bytes - max_point_size)) {
      memcpy((void *) coordinates, (const void *) (buffer + index),
             3 * sizeof(float));
      index += 3 * sizeof(float);
      push_back(LaserPoint((double) coordinates[0] + x_offset,
                           (double) coordinates[1] + y_offset,
                           (double) coordinates[2] + z_offset));
      point = end() - 1;
      num_attributes = buffer[index]; index++;
      if (num_attributes > max_num_attributes) {
        printf("Incorrect max num attributes (%d) for point %d, index %d\n", 
               num_attributes, size(), index);
        printf("Abort in function LaserPoints::ReadV3\n");
        exit(0);
      }
      for (i=0; i<num_attributes; i++) {
        attribute_tag = buffer[index];
        index++;
        memcpy((void *) &attribute_value, (const void *) (buffer + index),
               sizeof(int));
        index += sizeof(int);
        point->Attribute((const LaserPointTag) attribute_tag) = attribute_value;
      }
    }
    // Transfer the remainder of the buffer to the begin of the buffer
    num_bytes_left = num_bytes - index;
    if (num_bytes_left > 0)
      memmove((void *) buffer, (const void *) (buffer + index), num_bytes_left);
  }
  free(buffer);
  return(1);
}

int LaserPoints::ReadV2(FILE *fd, int num_pts, double x_offset,
                        double y_offset, double z_offset, bool verbose)
{
  int                   i, num_add, num_alloc;
  StoredLaserPointV2    *stored_laserpts, *stored_laserpt;
  LaserPoints::iterator point;
  LaserPointType        pointtype = scanner.PointType();

/* Memory (de)allocation */

  num_alloc = 10000;
  if (num_pts < num_alloc) num_alloc = num_pts + 1;
  stored_laserpts = (StoredLaserPointV2 *) malloc(sizeof(StoredLaserPointV2)
						  * num_alloc);
  if (stored_laserpts == NULL) {
    printf("Error allocating memory for laser points.\n");
    Close_Compressed_File(fd);
    return(0);
  }
  reserve(num_pts); /* Allocate vector space */

/* Read the data */

  while (!feof(fd)) {
    num_add = fread(stored_laserpts, sizeof(StoredLaserPointV2), num_alloc, fd);
    for (i=0, stored_laserpt=stored_laserpts;
	 i<num_add;
	 i++, stored_laserpt++) {
      push_back(LaserPoint((double) stored_laserpt->x + x_offset,
			   (double) stored_laserpt->y + y_offset,
			   (double) stored_laserpt->z + z_offset));
      point = end() - 1; // Last point
      if (pointtype == ColourPoint || pointtype == MultiColourPoint)
        point->SetAttribute(ColourTag, stored_laserpt->r);
      else if (pointtype == ReflectancePoint ||
               pointtype == MultiReflectancePoint)
        point->SetAttribute(ReflectanceTag, stored_laserpt->r);
      if (pointtype == MultiReflectancePoint || pointtype == MultiColourPoint ||
          pointtype == MultiPoint)
        point->SetAttribute(PulseCountTag, stored_laserpt->pc);
    }
  }
  free(stored_laserpts);

/* return success */

  if (verbose) printf("%d laser points read, file format 2.\n", num_pts);
  return(1);
}

int LaserPoints::ReadV1(FILE *fd, int num_pts, double x_offset,
                        double y_offset, double z_offset, bool verbose)
{
  int                i, num_add, num_alloc;
  StoredLaserPointV1 *stored_laserpts, *stored_laserpt;

/* Memory (de)allocation */

  num_alloc = 10000;
  if (num_pts < num_alloc) num_alloc = num_pts + 1;
  stored_laserpts = (StoredLaserPointV1 *) malloc(sizeof(StoredLaserPointV1)
						  * num_alloc);
  if (stored_laserpts == NULL) {
    printf("Error allocating memory for laser points.\n");
    Close_Compressed_File(fd);
    return(0);
  }
  reserve(num_pts); /* Allocate vector space */

/* Read the data */

  while (!feof(fd)) {
    num_add = fread(stored_laserpts, sizeof(StoredLaserPointV1), num_alloc, fd);
    for (i=0, stored_laserpt=stored_laserpts;
	 i<num_add;
	 i++, stored_laserpt++) {
      push_back(LaserPoint((double) stored_laserpt->x + x_offset,
			   (double) stored_laserpt->y + y_offset,
			   (double) stored_laserpt->z + z_offset,
			   stored_laserpt->r,
			   stored_laserpt->pc));
    }
  }
  free(stored_laserpts);

/* return success */

  if (verbose) printf("%d laser points read, file format 1.\n", num_pts);
  return(1);
}

/*
--------------------------------------------------------------------------------
               Read laser altimetry points from a file in old format
--------------------------------------------------------------------------------
*/

int LaserPoints::ReadV0(bool verbose)
{
  FILE               *fd;
  StoredLaserPointV0 *stored_laserpts, *stored_laserpt;
  int                i, num_add;
  LaserPoint         laserpt;

/* Point type is point with reflectance */

  scanner.SetPointType(ReflectancePoint);

/* Open the file */

#ifdef windows
  if ((fd = Open_Compressed_File(point_file, "rb")) == NULL) {
#else
  if ((fd = Open_Compressed_File(point_file, "r")) == NULL) {
#endif
    printf("Could not open laser points file %s\n", point_file);
    return(0);
  }

/* Read the data */

  stored_laserpts = (StoredLaserPointV0 *) malloc(sizeof(StoredLaserPointV0)
					     	   * 10000);
  if (stored_laserpts == NULL) {
    printf("Error allocating memory for laser points.\n");
    return(0);
  }
  while (!feof(fd)) {
    num_add = fread(stored_laserpts, sizeof(StoredLaserPointV0), 10000, fd);
    for (i=0, stored_laserpt=stored_laserpts;
	 i<num_add;
	 i++, stored_laserpt++) {
      push_back(LaserPoint((double) stored_laserpt->x,
			   (double) stored_laserpt->y,
			   (double) stored_laserpt->z,
			   stored_laserpt->r,
			   0));
    }
  }
  free(stored_laserpts);

/* return success */

  if (verbose) printf("%d laser points read, file format 0.\n", size());
  return(1);
}

/*
--------------------------------------------------------------------------------
                    Write laser altimetry points to a file
--------------------------------------------------------------------------------
*/

int LaserPoints::Write(const char *filename, int compress, bool verbose)
{
  FILE *fd;
  char *unzipped_file;
  int  num_bytes;
  
  // Open the output file
  unzipped_file = Name_Uncompressed_File(filename);
  if ((fd = fopen(unzipped_file, "wb")) == NULL) {
    printf("Could not open output file %s\n", unzipped_file);
    return(0);
  }

  // Write to the opened file
  num_bytes = Write(fd);
  
  // Close the file
  fclose(fd);

  // Compress the data if required
  if (compress) Compress_File(unzipped_file, NULL, COMPRESSION_GNU);

  if (verbose) printf("%d points written to file %s\n", size(), filename);
  if (num_bytes == 0) return 0;
  return 1;
}

int LaserPoints::Write(FILE *fd)
{
  unsigned char      *buffer, max_num_attributes, num_attributes;
  double             x_offset, y_offset, z_offset;
  LaserPoints::const_iterator pt;
  char               *reserved_space;
  int                index, type, file_id, num_pts, num_bytes, num_alloc,
                     max_point_size, buffer_size, ipt, ia;
  float              coords[3];
  const unsigned char *tags;
  const int           *values;

  // Determine the maximum number of attributes
  max_num_attributes = 0;
  for (pt=begin(); pt!=end(); pt++)
    if (pt->NumAttributes() > max_num_attributes)
      max_num_attributes = pt->NumAttributes();

  // Write the header information
  file_id = LASER_POINTS_V3;
  fwrite((const void *) &file_id, sizeof(int), 1, fd); num_bytes = sizeof(int);
  num_pts = size();
  fwrite((const void *) &num_pts, sizeof(int), 1, fd); num_bytes += sizeof(int);
  type = (int) scanner.PointType();
  fwrite((const void *) &type, sizeof(int), 1, fd); num_bytes += sizeof(int);
  if (size()) {
    pt = begin();
    x_offset = pt->X();
    y_offset = pt->Y();
    z_offset = pt->Z();
  }
  else
    x_offset = y_offset = z_offset = 0;
  fwrite((const void *) &x_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);
  fwrite((const void *) &y_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);
  fwrite((const void *) &z_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);
  fwrite((const void *) &max_num_attributes, sizeof(unsigned char), 1, fd);
  num_bytes += sizeof(unsigned char);

  // Reserve the rest of the 1024 byte header for future use
  num_bytes = 1024 - num_bytes;
  reserved_space = (char *) calloc(num_bytes, sizeof(char));
  fwrite((const void *) reserved_space, sizeof(char), num_bytes, fd);
  num_bytes = 1024;
  free(reserved_space);
  
  // Memory allocation
  num_alloc = 100000;
  if (num_pts < num_alloc) num_alloc = num_pts;
  max_point_size = 3 * sizeof(float) +
                   (max_num_attributes + 1) * sizeof(unsigned char) +
                   max_num_attributes * sizeof(int);
  buffer_size = num_alloc * max_point_size;
  buffer = (unsigned char *) malloc(buffer_size);
  if (buffer == NULL) {
    printf("Error allocating memory in LaserPoints::Write().\n");
    return 0;
  }

  // Write the data
  for (pt=begin(), ipt=0, index=0; pt!=end(); pt++, ipt++) {
    if (ipt == num_alloc) {
      fwrite((const void *) buffer, sizeof(unsigned char), index, fd);
      num_bytes += index;
      ipt = index = 0;
    }
    coords[0] = (float) (pt->X() - x_offset);
    coords[1] = (float) (pt->Y() - y_offset);
    coords[2] = (float) (pt->Z() - z_offset);
    memcpy((void *) (buffer + index), (const void *) coords, 3 * sizeof(float));
    index += 3 * sizeof(float);
    num_attributes = pt->NumAttributes();
    memcpy((void *) (buffer + index), (const void *) &num_attributes,
           sizeof(unsigned char));
    index += sizeof(unsigned char);
    tags = pt->AttributeTags();
    values = pt->AttributeValues();
    for (ia=0; ia<num_attributes; ia++) {
      memcpy((void *) (buffer + index), (const void *) (tags + ia),
             sizeof(unsigned char));
      index += sizeof(unsigned char);
      memcpy((void *) (buffer + index), (const void *) (values + ia),
             sizeof(int));
      index += sizeof(int);
    }
  }
  fwrite((const void *) buffer, sizeof(unsigned char), index, fd);
  num_bytes += index;
  free(buffer);

  return(num_bytes);
}

int LaserPoints::WriteV2(const char *filename, int compress)
{
  FILE               *fd;
  StoredLaserPointV2 *stored_laserpts, *stored_laserpt;
  double             x_offset, y_offset, z_offset;
  LaserPoints::const_iterator ipt;
  char               *reserved_space, *unzipped_file;
  int                i, type, file_id, num_pts, num_bytes, num_alloc;

// Open the output file

  unzipped_file = Name_Uncompressed_File(filename);
  if ((fd = fopen(unzipped_file, "wb")) == NULL) {
    printf("Could not open output file %s\n", unzipped_file);
    return(0);
  }

// Write the header information

  file_id = LASER_POINTS_V2;
  fwrite((const void *) &file_id, sizeof(int), 1, fd); num_bytes = sizeof(int);
  num_pts = size();
  fwrite((const void *) &num_pts, sizeof(int), 1, fd); num_bytes += sizeof(int);
  type = (int) scanner.PointType();
  fwrite((const void *) &type, sizeof(int), 1, fd); num_bytes += sizeof(int);
  if (size()) {
    ipt = begin();
    x_offset = ipt->X();
    y_offset = ipt->Y();
    z_offset = ipt->Z();
  }
  else
    x_offset = y_offset = z_offset = 0;
  fwrite((const void *) &x_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);
  fwrite((const void *) &y_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);
  fwrite((const void *) &z_offset, sizeof(double), 1, fd);
  num_bytes += sizeof(double);

// Reserve the rest of the 1024 byte header for future use

  num_bytes = 1024 - num_bytes;
  reserved_space = (char *) calloc(num_bytes, sizeof(char));
  fwrite((const void *) reserved_space, sizeof(char), num_bytes, fd);
  free(reserved_space);
  
// Memory allocation

  num_alloc = 10000;
  if (num_pts < num_alloc) num_alloc = num_pts;
  stored_laserpts = (StoredLaserPointV2 *) malloc(sizeof(StoredLaserPointV2)
						  * num_alloc);
  if (stored_laserpts == NULL) {
    printf("Error allocating memory for laser points.\n");
    return(0);
  }

// Write the data

  for (ipt=begin(), i=0, stored_laserpt=stored_laserpts;
       ipt!=end();
       ipt++, i++, stored_laserpt++) {
    if (i==num_alloc) {
      fwrite((const void *) stored_laserpts, sizeof(StoredLaserPointV2),
	     num_alloc, fd);
      i = 0;
      stored_laserpt = stored_laserpts;
    }
    stored_laserpt->x  = (float) (ipt->X() - x_offset);
    stored_laserpt->y  = (float) (ipt->Y() - y_offset);
    stored_laserpt->z  = (float) (ipt->Z() - z_offset);
    stored_laserpt->r  = ipt->Reflectance();
    stored_laserpt->pc = ipt->PulseCount();
  }
  fwrite((const void *) stored_laserpts, sizeof(StoredLaserPointV2), i, fd);
  free(stored_laserpts);

// Close the file

  fclose(fd);

// Compress the data if required

  if (compress) Compress_File(unzipped_file, NULL, COMPRESSION_GNU);

  return(1);
}

LASwriter *LaserPoints::WriteLASHeader(const char *filename, long long int num_pts,
                                       bool colour, LASpoint &laspoint,
                                       LASheader &lasheader,
						               const DataBoundsLaser *lbounds) const
{
  LASwriter         *laswriter;
  LASwriteOpener    laswriteopener;

  // Create LAS file header 
  lasheader.clean();
  if (colour) {
    lasheader.point_data_format = 2; /// X Y Z R G B
    lasheader.point_data_record_length = 26;
  }
  else {
    lasheader.point_data_format = 0; // only X Y Z
    lasheader.point_data_record_length = 20;
  }
  lasheader.number_of_point_records = num_pts;
  strncpy(lasheader.generating_software, "Mapping library", 10);
  if (lbounds)
    if (lbounds->XYZBoundsSet())
      lasheader.set_bounding_box(lbounds->Minimum().X(), lbounds->Minimum().Y(),
		                         lbounds->Minimum().Z(), lbounds->Maximum().X(),
		                         lbounds->Maximum().Y(), lbounds->Maximum().Z());
		                         
  // Open the LAZ file
  laswriteopener.set_file_name(filename);
  laswriter = laswriteopener.open(&lasheader);
  if (laswriter == 0) {
    fprintf(stderr, "ERROR: could not open laswriter\n");
    return NULL;
  }

  // Initialise laspoint for later writing
  laspoint.init(&lasheader, lasheader.point_data_format,
                lasheader.point_data_record_length, &lasheader);

  return laswriter;
}

void LaserPoints::WriteLASPoints(LASwriter *laswriter, LASpoint &laspoint,
                                bool colour) const
{
  LaserPoints::const_iterator point;
  
  // Write all points
  for (point=begin(); point!=end(); point++) {
    laspoint.set_x(point->X()); 
    laspoint.set_y(point->Y()); 
    laspoint.set_z(point->Z()); 
    if (point->HasAttribute(PulseCountTag)) {
      laspoint.return_number = point->Attribute(PulseCountTag);
      if (point->IsPulseType(LastPulse))
        laspoint.number_of_returns_of_given_pulse = laspoint.return_number;
      else // Just assume there's one more pulse
        laspoint.number_of_returns_of_given_pulse = laspoint.return_number + 1;
    }
    if (colour) {
      laspoint.rgb[0] = point->Red();
      laspoint.rgb[1] = point->Green();
      laspoint.rgb[2] = point->Blue();
      laspoint.have_rgb = true;
    }
    laswriter->write_point(&laspoint);
  }
}


int LaserPoints::WriteLAS(const char *filename, bool verbose) const
{
  LASpoint   laspoint;
  LASwriter  *laswriter;
  LASheader  lasheader;

  // Write the header and initialise the laspoint structure
  laswriter = WriteLASHeader(filename, size(), begin()->HasAttribute(ColourTag),
                             laspoint, lasheader, &DataBounds());
  if (!laswriter) return 0;

  // Write all points
  WriteLASPoints(laswriter, laspoint, begin()->HasAttribute(ColourTag));
        
  // Close laz file
  laswriter->close();
  
  if (verbose) printf("%d points written to LAS file %s\n", size(), filename);
  return 1;
}


/* Old code
int LaserPoints::WriteLAS(const char *filename, bool verbose) const
{
  LASpoint                    laspoint;
  LASwriteOpener              laswriteopener;
  LASwriter                   *laswriter;
  LASheader                   lasheader;
  LaserPoints::const_iterator point;

  // Create las file header 
  lasheader.clean();
  if (begin()->HasAttribute(ColourTag)) {
    lasheader.point_data_format = 2; /// X Y Z R G B
    lasheader.point_data_record_length = 26;
  }
  else {
    lasheader.point_data_format = 0; // only X Y Z
    lasheader.point_data_record_length = 20;
  }
  lasheader.number_of_point_records = size();
  strncpy(lasheader.generating_software, "Mapping library", 10);
  if (DataBounds().XYZBoundsSet())
    lasheader.set_bounding_box(DataBounds().Minimum().X(),
		                       DataBounds().Minimum().Y(),
		                       DataBounds().Minimum().Z(),
		                       DataBounds().Maximum().X(),
		                       DataBounds().Maximum().Y(),
		                       DataBounds().Maximum().Z());

  // Open the laz file
  laswriteopener.set_file_name(filename);
  laswriter = laswriteopener.open(&lasheader);
  if (laswriter == 0) {
    fprintf(stderr, "ERROR: could not open laswriter\n");
    return 0;
  }

  // Write all points
  laspoint.init(&lasheader, lasheader.point_data_format,
                lasheader.point_data_record_length, &lasheader);
  for (point=begin(); point!=end(); point++) {
    laspoint.set_x(point->X()); 
    laspoint.set_y(point->Y()); 
    laspoint.set_z(point->Z()); 
    if (point->HasAttribute(PulseCountTag)) {
      laspoint.return_number = point->Attribute(PulseCountTag);
      if (point->IsPulseType(LastPulse))
        laspoint.number_of_returns_of_given_pulse = laspoint.return_number;
      else // Just assume there's one more pulse
        laspoint.number_of_returns_of_given_pulse = laspoint.return_number + 1;
    }
    if (point->HasAttribute(ColourTag)) {
      laspoint.rgb[0] = point->Red();
      laspoint.rgb[1] = point->Green();
      laspoint.rgb[2] = point->Blue();
      laspoint.have_rgb = true;
    }
    laswriter->write_point(&laspoint);
  }
        
  // Close laz file
  laswriter->close();
  
  if (verbose) printf("%d points written to LAS file %s\n", size(), filename);
  return 1;
}

//*/
/*
--------------------------------------------------------------------------------
                           Delete all points
--------------------------------------------------------------------------------
*/
void LaserPoints::ErasePoints(bool erase_bounds)
{
  LaserPoints newset = LaserPoints();

  // Remove all attributes
  RemoveAttributes();
  
  // Delete all points
  if (!empty()) erase(begin(), end());

  /* Although all points are deleted, the memory space is still allocated by
   * the vector. In order to force deallocation we swap the vector with an new
   * empty vector. Note that this swap only affects the vector<LaserPoint>
   * even though LaserPoints also inherits from LaserPointsInfo and
   * LaserDataFiles. All meta data of "this" therefore remains unchanged.
   */
  swap(newset);
  
  // Invalidate the old bounds
  if (erase_bounds) bounds.Initialise();
}

/*
--------------------------------------------------------------------------------
                           Set an attribute value
--------------------------------------------------------------------------------
*/
void LaserPoints::SetAttribute(const LaserPointTag tag, const int value)
{
  LaserPoints::iterator point;

  for (point=begin(); point!=end(); point++) point->SetAttribute(tag, value);
}

void LaserPoints::SetAttribute(const LaserPointTag tag, const float value)
{
  LaserPoints::iterator point;

  for (point=begin(); point!=end(); point++) point->SetAttribute(tag, value);
}

/*
--------------------------------------------------------------------------------
                  Check if some points have a specified attribute
--------------------------------------------------------------------------------
*/
bool LaserPoints::HasAttribute(const LaserPointTag tag) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++) 
    if (point->HasAttribute(tag)) return true;
  return false;
}

/*
--------------------------------------------------------------------------------
                           Remove an attribute
--------------------------------------------------------------------------------
*/
void LaserPoints::RemoveAttribute(const LaserPointTag tag)
{
  for (LaserPoints::iterator point=begin(); point!=end(); point++)
    point->RemoveAttribute(tag);
}

void LaserPoints::RemoveAttributes()
{
  for (LaserPoints::iterator point=begin(); point!=end(); point++)
    point->RemoveAttributes();
}

/*
--------------------------------------------------------------------------------
                           Rename an attribute
--------------------------------------------------------------------------------
*/
void LaserPoints::RenameAttribute(const LaserPointTag oldtag,
                                  const LaserPointTag newtag)
{
  LaserPoints::iterator point;

  for (point=begin(); point!=end(); point++)
    point->RenameAttribute(oldtag, newtag);
}

/*
--------------------------------------------------------------------------------
                   Collect all values of a specific attribute
--------------------------------------------------------------------------------
*/
void LaserPoints::AttributeValues(const LaserPointTag tag, 
                                  vector <int> & values) const
{
  vector <int>::iterator      stored_value;
  int                         value;
  LaserPoints::const_iterator point;
  bool                        found;
  
  if (values.size()) values.erase(values.begin(), values.end());
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      value = point->Attribute(tag);
      for (stored_value=values.begin(), found=false;
           stored_value!=values.end() && !found; stored_value++)
        if (value == *stored_value) found = true;
      if (!found) values.push_back(value);
    }
  }
}

vector<int> &LaserPoints::AttributeValues(const LaserPointTag tag) const
{
    vector <int> *values = new vector <int>();
	vector <int>::iterator stored_value;
	int value;
	LaserPoints::const_iterator point;
	bool found;

	for (point=begin(); point!=end(); point++) {
		if (point->HasAttribute(tag)) {
			value = point->Attribute(tag);
			for (stored_value=values->begin(), found=false;
				stored_value!=values->end() && !found; stored_value++)
				if (value == *stored_value) found = true;
			if (!found) values->push_back(value);
		}
	}
  return *values;

}

void LaserPoints::AttributeValueCounts(const LaserPointTag tag, 
          vector <int> & values, vector <int> &counts) const
{
  vector <int>::iterator      stored_value, stored_count;
  int                         value;
  LaserPoints::const_iterator point;
  bool                        found;
  
  if (values.size()) values.erase(values.begin(), values.end());
  if (counts.size()) counts.erase(counts.begin(), counts.end());
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      value = point->Attribute(tag);
      for (stored_value=values.begin(), stored_count=counts.begin(), found=false;
           stored_value!=values.end() && !found; stored_value++, stored_count++)
        if (value == *stored_value) {
          found = true;
          (*stored_count)++;
        }
      if (!found) {
        values.push_back(value);
        counts.push_back(1);
      }
    }
  }
}

void LaserPoints::LongSegmentNumbers(vector <long long int> & values) const
{
  vector <long long int>::iterator stored_value;
  long long int                    value;
  LaserPoints::const_iterator      point;
  bool                             found;
  
  if (values.size()) values.erase(values.begin(), values.end());
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      value = point->LongSegmentNumber();
      for (stored_value=values.begin(), found=false;
           stored_value!=values.end() && !found; stored_value++)
        if (value == *stored_value) found = true;
      if (!found) values.push_back(value);
    }
  }
}

vector <LaserPointTag> & LaserPoints::UsedAttributes() const
{
  vector <LaserPointTag>           *used_attributes = new vector <LaserPointTag>();
  vector <LaserPointTag>::iterator used_attribute;
  const unsigned char              *attribute_tag;
  int                              num_attributes, index;
  LaserPoints::const_iterator      point;
  bool                             found;
  

  for (point=begin(); point!=end(); point++) {
    num_attributes = point->NumAttributes();
    for (index=0, attribute_tag=point->AttributeTags();
         index<num_attributes; index++, attribute_tag++) {
      if ((int) *attribute_tag < 254) { // Ignore DoubleTag and UndefinedTag
        for (used_attribute=used_attributes->begin(), found=false;
             used_attribute!=used_attributes->end() && !found; used_attribute++)
          if (*attribute_tag == (int) *used_attribute) found = true;
        if (!found) used_attributes->push_back((LaserPointTag) *attribute_tag);
      }
    }
  }

  return *used_attributes;
}

bool LaserPoints::AttributeRange(const LaserPointTag tag, int &minimum,
                                 int &maximum) const
{
  bool                        found=false;
  int                         value;
  LaserPoints::const_iterator point;
  
  point = begin();
  while (!found && point != end()) {
    if (point->HasAttribute(tag)) {
      found = true;
      minimum = maximum = point->Attribute(tag);
    }
    else point++;
  }
  if (!found) return false;
  for (; point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      value = point->Attribute(tag);
      if (value < minimum) minimum = value;
      else if (value > maximum) maximum = value;
    }
  }
  return true;
}

bool LaserPoints::AttributeRange(const LaserPointTag tag, double &minimum,
                                 double &maximum) const
{
  bool                        found=false;
  int                         value;
  LaserPoints::const_iterator point;
  LaserPointAttributeType     type;
  
  type = AttributeType(tag);
  if (type == IntegerAttributeType) return false;
  
  // Initialise minimum and maximum
  point = begin();
  while (!found && point != end()) {
    if (point->HasAttribute(tag)) {
      found = true;
      if (type == FloatAttributeType)
        minimum = maximum = point->FloatAttribute(tag);
      else
        minimum = maximum = point->DoubleAttribute(tag);
    }
  }
  if (!found) return false;
  
  // Update minimum and maximum
  for (; point!=end(); point++) {
    if (point->HasAttribute(tag)) {
      if (type == FloatAttributeType)
        value = point->FloatAttribute(tag);
      else
        value = point->DoubleAttribute(tag);
      if (value < minimum) minimum = value;
      else if (value > maximum) maximum = value;
    }
  }
}

/*
--------------------------------------------------------------------------------
               Derive the bounds of the laser data
--------------------------------------------------------------------------------
*/

DataBoundsLaser LaserPoints::DeriveDataBounds(int use_known_bounds)
{
  LaserPoints::const_iterator pt;
  int                         delete_points;

  // Return old bounds if that is allowed
  if (use_known_bounds && bounds.XYZBoundsSet()) return(bounds);
  
// Read point data if not done yet

  delete_points = 0;
  if (empty()) {
    if (PointFile() != NULL) {
      if (Read(PointFile(), false)) delete_points = 1;
      else fprintf(stderr, "Error reading laser points from file %s\n",
                   PointFile());
    }
  }

// Determine the bounds

  bounds.Initialise();
  for (pt=begin(); pt!=end(); pt++) bounds.Update(pt->LaserPointRef());

// Delete point data if read in this function, but don't delete bounds!

  if (delete_points) ErasePoints(false);

  return(bounds);
}

/*
--------------------------------------------------------------------------------
               Read the Delaunay triangulation of the laser points
--------------------------------------------------------------------------------
*/

TIN *LaserPoints::ReadTIN(const char *filename)
{
  if (tin_file) free(tin_file);
  tin_file = (char *) malloc(strlen(filename) + 1);
  strcpy(tin_file, filename);
  return(ReadTIN());
}


TIN *LaserPoints::ReadTIN()
{
  int success;

  if (tin) tin->~TIN();
  tin = new TIN(tin_file, &success);
  if (success) return(tin);
  else {
    fprintf(stderr, "Error reading TIN of laser points %s\n", name);
    return(NULL);
  }
}

/*
--------------------------------------------------------------------------------
               Write the Delaunay triangulation of the laser points
--------------------------------------------------------------------------------
*/

int LaserPoints::WriteTIN(const char *filename)
{
  tin_file = (char *) realloc(tin_file, strlen(filename) + 1);
  strcpy(tin_file, filename);
  return(WriteTIN());
}


int LaserPoints::WriteTIN() const
{
  return(tin->Write(tin_file, 1));
}

/*
--------------------------------------------------------------------------------
               Derive the Delaunay triangulation of the laser points
--------------------------------------------------------------------------------
*/

double *LaserPoints::TriangleCoordinateList() const
{
  LaserPoints::const_iterator laserpt;
  double                      *coordlist, *coord;

/* Put all X and Y coordinates into a double vector required by the
 * Delaunay triangulation function.
 */

  coordlist = (double *) malloc(2 * size() * sizeof(double));
  if (coordlist == NULL) {
    fprintf(stderr, "Error allocating coordinate list\n");
  }
  for (laserpt=begin(), coord=coordlist; laserpt!=end(); laserpt++) {
    *coord++ = laserpt->X();
    *coord++ = laserpt->Y();
  }
  return(coordlist);
}

TIN *LaserPoints::DeriveTIN()
{
  double *coordlist;
  
/* Create an empty TIN if there are less than three points. */

  if (size() < 3) {
    tin = new TIN();
  }
  else {

/* Put all X and Y coordinates into an array of double */

    coordlist = TriangleCoordinateList();

/* Derive the Delaunay triangulation */

    if (tin) tin->~TIN(); /* Remove old tin */
    tin = new TIN(coordlist, size());
    free(coordlist);
  }

  return(tin);
}

TIN *LaserPoints::VerifyTIN()
{
  if (tin == NULL) return DeriveTIN();
  else if (tin->HighestNodeNumber().Number() != size() - 1) {
    tin->Erase();
    return DeriveTIN();
  }
  return tin;
}

/*
--------------------------------------------------------------------------------
                    Convert laser points to image points
--------------------------------------------------------------------------------
*/

ImagePoints *LaserPoints::Map_Into_Image(const ImageGrid &grid) const
{
  ImagePoints                 *imagepoints;
  LaserPoints::const_iterator laserpoint;
  int                         number;

  imagepoints = new ImagePoints();
  for (laserpoint=begin(), number=0; laserpoint!=end(); laserpoint++, number++)
    imagepoints->push_back(laserpoint->Map_Into_Image(number, grid));
  return(imagepoints);
}

/*
--------------------------------------------------------------------------------
                    Add data of laser points to an image
--------------------------------------------------------------------------------
*/

/* Code used for testing on HP */
/*
void memory_usage_ImageData(const char *string)
{
  struct mallinfo info;

  info = mallinfo();
  printf("%s: %d bytes\n", string, info.uordblks);
}
*/

void LaserPoints::ImageData(Image &image, Image &dist, const DataGrid &grid,
                            ImagedDataType datatype, int interpolation_method,
                            double max_mesh_size)
{
  int delete_points, delete_tin;

/* Read point data if not done yet */

  delete_points = 0;
  if (empty())
    if (Read(PointFile(), false)) delete_points = 1;
    else {
      fprintf(stderr,
              "No point data available for imaging in point set %s.\n", name);
      return;
    }
    
/* Derive the TIN data if required */

  delete_tin = 0;
  if (interpolation_method >= 2) {
    if (GetTIN() == NULL) {
      DeriveTIN();
      delete_tin = 1;
    }
  }

/* Select the imaging function with the appropriate interpolation method */

  switch (interpolation_method) {
    case  1: /* Nearest neighbour or minimum or maximum within a pixel */
    case  4:
    case  5: ImageDataBinning(image, dist, grid, datatype,
                              interpolation_method);
             break;
    case  2: /* Barycentric or planar interpolation in a TIN */
    case  3: ImageDataTriangle(image, dist, TINReference(), grid, datatype,
                          interpolation_method, max_mesh_size);
             break;
    default: fprintf(stderr,"Invalid interpolation method\n");
             exit(0);
  }

/* Delete point data if read in this function */

  if (delete_points) ErasePoints();

/* Delete TIN data if read in this function */

  if (delete_tin) {
    TINReference().Erase();
    SetTIN(NULL);
  }
}

void LaserPoints::ImageDataBinning(Image &image, Image &dist,
                                   const DataGrid &grid,
                                   ImagedDataType datatype,
                                   int interpolation_method)
{
  LaserPoints::const_iterator laserpoint;
  ImagePoint                  imagepoint;
  int                         ir, ic, numpix, compute_intensity;
  double                      dr, dc, d;
  float                       *dist_pixel, *fl_image_pixel;
  unsigned char               *image_pixel;

/* Update the current image with the data of the laser points */

  numpix = image.NumRows() * image.NumColumns();
  compute_intensity = (scanner.PointType() == ColourPoint ||
                       scanner.PointType() == MultiColourPoint);
  for (laserpoint=begin(); laserpoint!=end(); laserpoint++) {
    imagepoint = laserpoint->Map_Into_Image(1, grid.ImageGridReference());
    ir = (int) (imagepoint.Row() + 0.5);
    ic = (int) (imagepoint.Column() + 0.5);
    if (ir >= 0 && ir < image.NumRows() &&
        ic >= 0 && ic < image.NumColumns()) {
      if (datatype == PointCountData) { // Just count, no interpolation
        if (image.DataType() == VFF_TYP_1_BYTE) {
          image_pixel = image.Pixel(ir, ic);
          if (*image_pixel < 255) (*image_pixel)++;
        }
        else { // Float count
          fl_image_pixel = image.FloatPixel(ir, ic);
          *fl_image_pixel += 1.0;
        }
      }
      else {  
        if (interpolation_method == 1) { /* Nearest neighbour */
          dr = imagepoint.Row() - ir;
          dc = imagepoint.Column() - ic;
          d = dr*dr + dc*dc;   /* Squared distance to pixel centre */
        }
        else if (interpolation_method == 4) { /* Minimum */
          /* Use (abuse) the data value as distance */
          if (datatype == HeightData) d = laserpoint->Z();
          else d = laserpoint->Reflectance();
        }
        else if (interpolation_method == 5) { /* Maximum */
          /* Use (abuse) the data value as distance */
          if (datatype == HeightData) d = -laserpoint->Z();
          else d = -laserpoint->Reflectance();
        }
        dist_pixel = dist.FloatPixel(ir, ic);     
        if (d < *dist_pixel) {
          *dist_pixel = d;
          if (image.DataType() == VFF_TYP_1_BYTE) {
            image_pixel = image.Pixel(ir, ic);
            switch (datatype) {
              case HeightData:
                *image_pixel = grid.GreyValue(laserpoint->Z());
                break;

              case ReflectanceData:
                if (compute_intensity)
                  *image_pixel = grid.GreyValue((double) laserpoint->Intensity());
                else
                  *image_pixel=grid.GreyValue((double) laserpoint->Reflectance());
                break;

              case ColourData:
                *image_pixel = grid.GreyValue((double) laserpoint->Red());
                *(image_pixel + numpix) =
                   grid.GreyValue((double) laserpoint->Green());
                *(image_pixel + 2 * numpix) =
                   grid.GreyValue((double) laserpoint->Blue());
                break;

              case PulseLengthData:
                *image_pixel = grid.GreyValue((double) laserpoint->PulseLength());
                break;
              
              case PointCountData: // Just to satisfy compilers
                break;
              
              default:
                fprintf(stderr, "Unknown data type for imaging.\n");
                exit(0);
            }
          }
          else { /* Float image */
            fl_image_pixel = image.FloatPixel(ir, ic);
            switch (datatype) {
              case HeightData:
                *fl_image_pixel = (float) laserpoint->Z();
                break;

              case ReflectanceData:
                *fl_image_pixel = (float) laserpoint->Reflectance();
                break;
  
              case ColourData:
                *fl_image_pixel = (float) laserpoint->Red();
                *(fl_image_pixel + numpix) = (float) laserpoint->Green();
                *(fl_image_pixel + 2 * numpix) = (float) laserpoint->Blue();
                break;

              case PulseLengthData:
                *fl_image_pixel = (float) laserpoint->PulseLength();
                break;

              case PointCountData: // Just to satisfy compilers
                break;

              default:
                fprintf(stderr, "Unknown data type for imaging.\n");
                exit(0);
            }
          }
        }
      }
    }
  }  
}


/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		Barycentric interpolation of variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

double LaserPoints::Inter_bary(const TINMesh *mesh, double X, double Y,
                               ImagedDataType datatype) const
{
  double     det123, w[3], b_x, b_y, b_a;
  int        i;
  LaserPoint vertex[3];
		
  vertex[0] = *(begin() + mesh->Nodes()->Number());
  vertex[1] = *(begin() + (mesh->Nodes()+1)->Number());
  vertex[2] = *(begin() + (mesh->Nodes()+2)->Number());

  // Calculate the determinant
					
  det123 = (vertex[1].X()-vertex[0].X()) * (vertex[2].Y()-vertex[0].Y()) -
                 (vertex[2].X()-vertex[0].X()) * (vertex[1].Y()-vertex[0].Y());

  // calculate the barycentric coordinates of the point	
  w[0] = (((vertex[1].X()-X)*(vertex[2].Y()-Y)) -
         ((vertex[2].X()-X)*(vertex[1].Y()-Y))) / det123;
  
  w[1] = (((vertex[2].X()-X)*(vertex[0].Y()-Y)) -
         ((vertex[0].X()-X)*(vertex[2].Y()-Y))) / det123;
	
  w[2] = (((vertex[0].X()-X)*(vertex[1].Y()-Y)) -
         ((vertex[1].X()-X)*(vertex[0].Y()-Y))) / det123;

  if ( w[0] > 1 || w[0] < 0)	{
    printf("\nw0 %f > 1 or < 0; moving to next point\n", w[0]);
    printf("\nX is %f, Y is %f\n", X, Y);
    exit(1);
  }

  if ( w[1] > 1 || w[1] < 0)	{
    printf("\nw1 %f > 1 or < 0; moving to next point\n", w[1]);
    printf("\nX is %f, Y is %f\n", X, Y);
    exit(1);
  }

  if ( w[2] > 1 || w[2] < 0)	{
    printf("\nw2 %f > 1 or < 0; moving to next point\n", w[2]);
    printf("\nX is %f, Y is %f\n", X, Y);
    exit(1);
  }

  // check on the barycnetric coordinates 
        
  for (i=0, b_x=0; i<3; i++) b_x += w[i] * vertex[i].X();
  for (i=0, b_y=0; i<3; i++) b_y += w[i] * vertex[i].Y();
	
  if ((X-.1) > b_x || b_x > (X+.1) || (Y-.1) > b_y || b_y > (Y+.1)) {
    printf ("\nError in Barycentric coordinates");
    exit(1);
  }	
		
  // get the dependent variable value
  switch(datatype) {
    case HeightData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Z();
      break;

    case ReflectanceData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Reflectance();
      break;

    case RedData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Red();
      break;

    case GreenData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Green();
      break;

    case BlueData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Blue();
      break;

    case IntensityData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].Intensity();
      break;

    case PulseLengthData:
      for (i=0, b_a=0.0; i<3; i++) b_a += w[i] * vertex[i].PulseLength();
      break;

    case PointCountData:
      printf("Error: No interpolation for point count data. Use binning only.\n");
      exit(0);
      break;

    default:
      fprintf(stderr, "Unknown data type for imaging (%d).\n", datatype);
      exit(0);
  }
  return b_a;					
}

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		Extract bounding boxes for each triangle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

DataBoundsLaser LaserPoints::ExtractBounding(const TINMesh *triangle) const
{
 	LaserPoint 		new_lpt[3];
 	int 			b;
 	DataBoundsLaser		dbl;
	double 			max_x, min_x, max_y, min_y; 	
	
	new_lpt[0] = *(begin()+ triangle->Nodes()->Number());
	new_lpt[1] = *(begin()+ (triangle->Nodes()+1)->Number());	
	new_lpt[2] = *(begin()+ (triangle->Nodes()+2)->Number());	
	
	for (b = 0; b < 3; b++)	{

		if (b == 0)	{
			max_x = min_x = new_lpt[b].X();
			max_y = min_y = new_lpt[b].Y();
			}
		else	{
			if (new_lpt[b].X() < min_x)  
				min_x = new_lpt[b].X();
			if (new_lpt[b].X() > max_x)  
				max_x = new_lpt[b].X();
			if (new_lpt[b].Y() < min_y)  
				min_y = new_lpt[b].Y();						
			if (new_lpt[b].Y() > max_y)  
				max_y = new_lpt[b].Y();						
			}		
		}					
	dbl.SetMaximumX(max_x);
	dbl.SetMaximumY(max_y);
	dbl.SetMinimumX(min_x);
	dbl.SetMinimumY(min_y);		

/*      delete [] new_lpt; */
	return (dbl);
}

/*
--------------------------------------------------------------------------------
             Adds a point to an image using TIN structure
--------------------------------------------------------------------------------
*/

void LaserPoints::ImageDataTriangle(Image &image, Image &used_mesh_sizes,
                                    const TIN &tin, const DataGrid &grid,
                                    ImagedDataType datatype,
                                    int interp_type, double max_mesh_size)
{
  int				row, col;	/* Iterators for row, column  */
  int 				in_row, in_col;	/* Iterators for BB test      */
  double 			X, Y;		/* Pixel location             */
  double			in_X, in_Y;	/* Pixel location within BB   */
  double			max_x, max_y;	/* Extracted from BB          */
  double			min_x, min_y;
  unsigned char               	*image_pixel;	/* Pointer to image           */
  float                       	*fl_image_pixel;/* Pointer to float image     */
  float                         *mesh_size_ptr; /* Pointer to stored mesh size*/
  float                         mesh_size;      /* Size of current mesh       */
  TIN::const_iterator 	        triangle;	/* TIN iterator               */
  LaserPoint 			lpt;		/* Individual LaserPoint from vector */
  LaserPoint			max, min;	/* From data bounds of BB     */
  
  DataBoundsLaser 		dbl;		/* Databounds of bounding box */
  int                           numpix;         /* Number of image pixels     */
  int                           compute_intensity; /* Flag for colour imagery */
  ImagedDataType                ref_data;          /* Reflectance data type   */
  
  numpix = image.NumRows() * image.NumColumns();
  compute_intensity = (scanner.PointType() == ColourPoint ||
                       scanner.PointType() == MultiColourPoint);
  if (datatype == ReflectanceData && compute_intensity) ref_data =IntensityData;
  else ref_data = ReflectanceData;

/* Loop over all triangles */

  for (triangle=tin.begin(); triangle!=tin.end(); triangle++) {

/* Calculate bounding box */

    dbl = ExtractBounding(&*triangle);
    max = dbl.Maximum();	max_x = max.X();	max_y = max.Y();
    min = dbl.Minimum();	min_x = min.X();	min_y = min.Y();

/* Check if the mesh size is below the set maximum */

    mesh_size = max_x - min_x;
    if (max_y - min_y > mesh_size) mesh_size = max_y - min_y;
    if (mesh_size < max_mesh_size) {

/* Calculate positions */

      row = (int) ((grid.YOffset() - max_y)/grid.Pixelsize());	
      if (row < 0) row = 0;
      col = (int) ((min_x - grid.XOffset())/grid.Pixelsize());
      if (col < 0) col = 0;

      X = grid.XOffset() + (col*grid.Pixelsize()) + 0.5*grid.Pixelsize();
      Y = grid.YOffset() - (row*grid.Pixelsize()) - 0.5*grid.Pixelsize();
	
/* Loop over all pixels within bounding box of the mesh */

      for (in_Y = Y, in_row = row;
           in_Y >= min_y && in_row < image.NumRows();
           in_Y-=grid.Pixelsize(), in_row++) {

	for (in_X = X, in_col = col;
             in_X <= max_x && in_col < image.NumColumns();
             in_X+=grid.Pixelsize(), in_col++) {

/* Only update pixel value if the mesh size is smaller than previously used
 * sizes of other strips.
 */
          mesh_size_ptr = used_mesh_sizes.FloatPixel(in_row, in_col);
          if (mesh_size < *mesh_size_ptr) {

/* Check if the pixels is inside the triangle */

	    if (InsideTriangle(&*triangle, in_X, in_Y))	{
              *mesh_size_ptr = mesh_size; // store current mesh size
	      lpt.X() = in_X;             // assign values to laser point
              lpt.Y() = in_Y;
					// Map point to position in Image
              if (image.DataType() == VFF_TYP_1_BYTE)
	        image_pixel = image.Pixel(in_row, in_col);
              else /* Float image */
	        fl_image_pixel = image.FloatPixel(in_row, in_col);

	      switch (datatype) {
		case HeightData:
		  if (interp_type == 2)
		    lpt.Z() = Inter_bary(&*triangle, in_X, in_Y, datatype);
		  else if (interp_type == 3)
		    lpt.Z() = Inter_plane(&*triangle, in_X, in_Y, datatype);
		  else if (interp_type != 2 && interp_type != 3) {
		    fprintf(stderr,
                            "Wrong Interpolation type has been passed\n");
		    exit(0);
	  	  }								
                  if (image.DataType() == VFF_TYP_1_BYTE)
		    *image_pixel = lpt.Height_To_GreyValue(grid);
                  else
		    *fl_image_pixel = lpt.Z();
		  break;

		case ReflectanceData:
		  if (interp_type == 2)
		    lpt.SetReflectance((int) Inter_bary(&*triangle, in_X,
					                in_Y, ref_data));
		  else if (interp_type == 3)
		    lpt.SetReflectance((int) Inter_plane(&*triangle,in_X,
							 in_Y, ref_data));
		  else if (interp_type != 2 && interp_type != 3) {
		    fprintf(stderr,
                            "Wrong Interpolation type has been passed\n");
		    exit(0);
		  }								
                  if (image.DataType() == VFF_TYP_1_BYTE)
	  	    *image_pixel = lpt.Reflectance_To_GreyValue(grid);
                  else
		    *fl_image_pixel = lpt.Reflectance();
		  break;

		case ColourData:
		  if (interp_type == 2) {
                    lpt.SetColour((int) (unsigned char)
                                  Inter_bary(&*triangle, in_X, in_Y, RedData),
                                  lpt.Green(), lpt.Blue());
                    lpt.SetColour(lpt.Red(), (int) (unsigned char)
                                  Inter_bary(&*triangle, in_X, in_Y, GreenData),
                                  lpt.Blue());
                    lpt.SetColour(lpt.Red(), lpt.Green(), (int) (unsigned char)
                                  Inter_bary(&*triangle, in_X, in_Y, BlueData));
                  }
		  else if (interp_type == 3) {
                    lpt.SetColour((int) (unsigned char)
                                  Inter_plane(&*triangle, in_X, in_Y, RedData),
                                  lpt.Green(), lpt.Blue());
                    lpt.SetColour(lpt.Red(), (int) (unsigned char)
                                  Inter_plane(&*triangle, in_X,in_Y, GreenData),
                                  lpt.Blue());
                    lpt.SetColour(lpt.Red(), lpt.Green(), (int) (unsigned char)
                                  Inter_plane(&*triangle, in_X,in_Y, BlueData));
                  }
		  else {
		    fprintf(stderr,
                            "Wrong Interpolation type has been passed\n");
		    exit(0);
		  }								
                  if (image.DataType() == VFF_TYP_1_BYTE) {
                    *image_pixel = grid.GreyValue((double) lpt.Red());
                    *(image_pixel + numpix) =
                       grid.GreyValue((double) lpt.Green());
                    *(image_pixel + 2 * numpix) =
                       grid.GreyValue((double) lpt.Blue());
                  }
                  else {
                    *fl_image_pixel = (float) lpt.Red();
                    *(fl_image_pixel + numpix) = (float) lpt.Green();
                    *(fl_image_pixel + 2 * numpix) = (float) lpt.Blue();
                  }
		  break;

		case PulseLengthData:
		  if (interp_type == 2)
		    lpt.SetPulseLength((int) Inter_bary(&*triangle, in_X,
					                in_Y, PulseLengthData));
		  else if (interp_type == 3)
		    lpt.SetPulseLength((int) Inter_plane(&*triangle,in_X,
							 in_Y, PulseLengthData));
		  else if (interp_type != 2 && interp_type != 3) {
		    fprintf(stderr,
                            "Wrong Interpolation type has been passed\n");
		    exit(0);
		  }								
          if (image.DataType() == VFF_TYP_1_BYTE)
//          *image_pixel = grid.GreyValue(lpt.Reflectance_To_GreyValue(grid);
            *image_pixel = grid.GreyValue((double) lpt.PulseLength());
                  else
		    *fl_image_pixel = (float) lpt.PulseLength();
		  break;

		case PointCountData:
          printf("Error: No interpolation for count point data. Use binning only.\n");
          exit(0);

		default:
		  fprintf(stderr, "Unknown data type for imaging (%d).\n",
                          datatype);
		  exit(0);
	      } /* End switch data type */
	    } /* End inside triangle */
          } /* End smallest mesh size test */
	} /* End column loop */
      } /* End row loop */
    } /* End mesh size test */
  } /* End loop over triangles */
}

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			Inside Triangle Test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

int LaserPoints::InsideTriangle(const TINMesh *mesh, double X, double Y) const
{
	int area0, area1, area2;
	Vector2D v2d = Vector2D (X, Y);
	Vector2D vertex0 = *(begin() + mesh->Nodes()->Number());
	Vector2D vertex1 = *(begin() + (mesh->Nodes()+1)->Number());
	Vector2D vertex2 = *(begin() + (mesh->Nodes()+2)->Number());
	area0 = v2d.AreaSign(vertex0, vertex1);
	area1 = v2d.AreaSign(vertex1, vertex2);
	area2 = v2d.AreaSign(vertex2, vertex0);	
	
	if ( ( area0 == 0) && (area1 == 0) && (area2 == 0) )		{
		return 0;
	}

	if ( ( area0 == 0 ) && ( area1 == 0 ) ||
	     ( area0 == 0 ) && ( area2 == 0 ) ||
	     ( area1 == 0 ) && ( area2 == 0 ) )		{
	     	return 1;
	}

	if ( ( area0 == 0) && (area1 > 0) && (area2 > 0) || 
	     ( area1 == 0) && (area0 > 0) && (area2 > 0) ||
             ( area2 == 0) && (area0 > 0) && (area1 > 0))	{
		return 1;
	}
		
	if ( ( area0 == 0) && (area1 < 0) && (area2 < 0) || 
	     ( area1 == 0) && (area0 < 0) && (area2 < 0) ||
             ( area2 == 0) && (area0 < 0) && (area1 < 0))	{
		return 1;
	}

	if ( ( area0 > 0) && (area1 > 0) && (area2 > 0) || 
             ( area0 < 0) && (area1 < 0) && (area2 < 0))	{
		return 1;
	}
		
	else	{
		return 0;	   
	}
}

/*
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		Planar interpolation of dependant variable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

double LaserPoints::Inter_plane(const TINMesh *mesh, double X, double Y,
                                ImagedDataType datatype) const
{
  int 	   lpt1, lpt2, lpt3;
  Vector3D v0, v1, v2;
  Plane    plane;
  double   v0_a;

  lpt1 = mesh->Nodes()->Number();
  lpt2 = (mesh->Nodes()+1)->Number();
  lpt3 = (mesh->Nodes()+2)->Number();

  v1 = (*this)[lpt2] - (*this)[lpt1];
  v2 = (*this)[lpt3] - (*this)[lpt1];
  v0 = (*this)[lpt1];

  switch(datatype) {
    case HeightData:
      v0_a = (*this)[lpt1].Z();
      break;
  
    case ReflectanceData:
      v1.Z() = (double) ((*this)[lpt2].Reflectance() -
                         (*this)[lpt1].Reflectance());
      v2.Z() = (double) ((*this)[lpt3].Reflectance() -
                         (*this)[lpt1].Reflectance());
      v0_a = (*this)[lpt1].Reflectance();
      break;

    case RedData:
      v1.Z() = (double) ((*this)[lpt2].Red() - (*this)[lpt1].Red());
      v2.Z() = (double) ((*this)[lpt3].Red() - (*this)[lpt1].Red());
      v0_a = (*this)[lpt1].Red();
      break;

    case GreenData:
      v1.Z() = (double) ((*this)[lpt2].Green() - (*this)[lpt1].Green());
      v2.Z() = (double) ((*this)[lpt3].Green() - (*this)[lpt1].Green());
      v0_a = (*this)[lpt1].Green();
      break;

    case BlueData:
      v1.Z() = (double) ((*this)[lpt2].Blue() - (*this)[lpt1].Blue());
      v2.Z() = (double) ((*this)[lpt3].Blue() - (*this)[lpt1].Blue());
      v0_a = (*this)[lpt1].Blue();
      break;

    case IntensityData:
      v1.Z() = (double) ((*this)[lpt2].Intensity() - (*this)[lpt1].Intensity());
      v2.Z() = (double) ((*this)[lpt3].Intensity() - (*this)[lpt1].Intensity());
      v0_a = (*this)[lpt1].Intensity();
      break;

    case PulseLengthData:
      v1.Z() = (double) ((*this)[lpt2].PulseLength() -
                         (*this)[lpt1].PulseLength());
      v2.Z() = (double) ((*this)[lpt3].PulseLength() -
                         (*this)[lpt1].PulseLength());
      v0_a = (*this)[lpt1].PulseLength();
      break;

    case PointCountData:
      printf("Error: No interpolation for point count data. Use binning only.\n");
      exit(0);
      
    default:	
      fprintf(stderr, "Wrong Data type has been passed (%d)\n", datatype);
      exit(0);
  }
	
  // Compute the interpolated value
  plane.SetNormal(v1, v2);
  plane.SetDistance(v0_a, v0);
  return(plane.Interp(X, Y));
}

/*
--------------------------------------------------------------------------------
                    Select points within bounds
--------------------------------------------------------------------------------
*/

void LaserPoints::Select(LaserPoints &selection,
                         const DataBoundsLaser &sel_bounds) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    if (sel_bounds.Inside(&*point)) selection.push_back(*point);
 // printf("Dataset with %6d points, selection with %6d points.\n",
      //   size(), selection.size());
}

void LaserPoints::Select(LaserPoints &selection,
                         const Line2D &line, double max_dist,
                         double scalar_min, double scalar_max) const
{
  LaserPoints::const_iterator point;
  double                      dist, scalar;
  Position2D                  position;
  DataBounds2D                bounds;
  int                         num_inside=0;
  DataBoundsLaser             lbounds;
  
  // Determine the bounding box of the rotated rectangle
  bounds = line.BoundingBox(scalar_min, scalar_max, max_dist);

  // Collect the points
  for (point=begin(); point!=end(); point++) {
    lbounds.Update(*point);
    position.X() = point->X();
    position.Y() = point->Y();
    if (bounds.Inside(position)) { // First check on bounding box, much quicker
      num_inside++;
      if (line.DistanceToPoint(position) <= max_dist) {
        scalar = line.Scalar(position);
        if (scalar >= scalar_min && scalar <= scalar_max)
          selection.push_back(*point);
      }
    }
  }
}

void LaserPoints::ReduceData(const DataBoundsLaser &sel_bounds)
{
  LaserPoints::iterator point, goodpoint;

  for (point=begin(), goodpoint=begin();
       point!=end();
       point++) {
    if (sel_bounds.Inside(&*point)) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
}

/*
--------------------------------------------------------------------------------
                    Select points within polygons
--------------------------------------------------------------------------------
*/

void LaserPoints::Select(LaserPoints &selection, const ObjectPoints2D &corners,
                         const LineTopologies &polygons) const
{
  LineTopologies::const_iterator polygon;
  LaserPoints::const_iterator    point;

  for (point=begin(); point!=end(); point++) {
    for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
      if (point->InsidePolygon(corners, polygon->PointNumberListReference()))
        selection.push_back(*point);
    }
  }
}

/*
--------------------------------------------------------------------------------
                    Select points within labeled pixels
--------------------------------------------------------------------------------
*/

void LaserPoints::Select(LaserPoints &selection, const Image &image,
                         const ImageGrid &grid) const
{
  LaserPoints::const_iterator point;
  ImagePoint                  imagepoint;
  int                         ir, ic;

  for (point=begin(); point!=end(); point++) {
    imagepoint = point->Map_Into_Image(1, grid);
    ir = (int) (imagepoint.Row() + 0.5);
    ic = (int) (imagepoint.Column() + 0.5);
    if (ir >= 0 && ir < image.NumRows() &&
        ic >= 0 && ic < image.NumColumns())
      if (*image.Pixel(ir, ic)) selection.push_back(*point);
  }
}

/*
--------------------------------------------------------------------------------
                 Direction and angle between points in XY plane
--------------------------------------------------------------------------------
*/

double LaserPoints::Direction2D(const PointNumber &n1,
                                const PointNumber &n2) const
{
  LaserPoints::const_iterator p1, p2;
  Vector3D c2;

  p1 = begin() + n1.Number();
  p2 = begin() + n2.Number();

  c2 = *p2;
  return(p1->Direction2D(c2));
}

double LaserPoints::Angle2D(const PointNumber &n1, const PointNumber &n2,
                            const PointNumber &n3) const
{
  LaserPoints::const_iterator p1, p2, p3;
  Vector3D c1, c3;

  p1 = begin() + n1.Number();
  p2 = begin() + n2.Number();
  p3 = begin() + n3.Number();

  c1 = *p1;
  c3 = *p3;
  return(p2->Angle2D(c1, c3));
}

/*
--------------------------------------------------------------------------------
                Write points to a VRML file as points or crosses
--------------------------------------------------------------------------------
*/
void LaserPoints::VRML_Write(FILE *fd, int version, bool colour) const
{
  LaserPoints::const_iterator point;

  switch (version) {
    case 1:
    default:
      VRML_Write_Begin_Of_PointList(fd);
      for (point=begin(); point!=end(); point++)
        point->Position3D::VRML_Write(fd);
      VRML_Write_End_Of_PointList(fd);
      break;
      
    case 2:
      fprintf(fd, "#VRML V2.0 utf8\n");
      fprintf(fd, "Shape {\n");
      fprintf(fd, "  geometry PointSet {\n");
      fprintf(fd, "    coord Coordinate { point [\n");
      for (point=begin(); point!=end(); point++)
        fprintf(fd, "%.3f %.3f %.3f,\n", point->X(), point->Y(), point->Z());
      fprintf(fd, "    ]}\n");
      if (colour) {
        fprintf(fd, "    color Color { color [\n");
        for (point=begin(); point!=end(); point++)
          fprintf(fd, "%4.2f %4.2f %4.2f,\n", point->Red() / 255.0,
                  point->Green() / 255.0, point->Blue() / 255.0);
        fprintf(fd, "    ]}\n");
      }
      fprintf(fd, "  }\n");
      fprintf(fd, "}\n");
      break;
  }
}

void LaserPoints::VRML_Write_Crosses(FILE *fd, double size) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    point->VRML_Write_Cross(fd, size);
}

void LaserPoints::VRML_Write_Spheres(FILE *fd, double radius) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    point->VRML_Write_Sphere(fd, radius);
}

/*
--------------------------------------------------------------------------------
                 Write lines as cylinders to a VRML file
--------------------------------------------------------------------------------
*/

void LaserPoints::VRML_Write_Cylinders(FILE *fd, const LineTopologies & lines,
                                       double radius) const
{
  LineTopologies::const_iterator line;
  LineTopology::const_iterator   pn;
  LaserPoints::const_iterator    point1, point2;
  Position3D                     pos1, pos2;

  for (line=lines.begin(); line!=lines.end(); line++) {
    if (line->size() > 1) {
      point1 = begin() + line->begin()->Number();
      for (pn=line->begin()+1; pn!=line->end(); pn++) {
        point2 = begin() + pn->Number();
        pos1 = *point1;
        pos2 = *point2;
        VRML_Write_Cylinder(fd, pos1, pos2, radius);
        point1 = point2;
      }
    }
  }
}

/*
--------------------------------------------------------------------------------
                 Add coordinates or attributes to a histogram
--------------------------------------------------------------------------------
*/

void LaserPoints::AddToHistogram(Histogram &histo, int type) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    point->AddToHistogram(histo, type);
}

void LaserPoints::AddToHistogram(Histogram &histo, int type,
                                 const LaserPointTag &tag) const
{
  LaserPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    point->AddToHistogram(histo, type, tag);
}


/*
--------------------------------------------------------------------------------
                       Thinning of laser data
--------------------------------------------------------------------------------
*/

void LaserPoints::ReduceData(int factor, int random_selection, int start_offset)
{
  LaserPoints::iterator store_position, selected_point;
  int                   random_offset;
  long                  random_number;

#ifdef hpux
  if (random_selection) srandom(1);
#else
  if (random_selection) srand(1);
#endif

  store_position = begin();
  selected_point = begin() + start_offset;
  if (random_selection) {
#ifdef hpux
    random_number = random();
#else
    random_number = rand();
#endif
    random_offset = random_number - (random_number / factor) * factor;
    selected_point += random_offset;
  }
  while (selected_point < end()) {
    *store_position = *selected_point;
    store_position++;
    if (random_selection) {
      selected_point += factor - random_offset;
#ifdef hpux
      random_number = random();
#else
      random_number = rand();
#endif
      random_offset = random_number - (random_number / factor) * factor;
      selected_point += random_offset;
    }
    else selected_point += factor;
  }
  if (!empty()) erase(store_position, end());
}

void LaserPoints::ReduceData(double min_dist, int knn)
{
  PointNumberList           list, nbhood;
  PointNumberList::iterator node, nb_node;
  TINEdges                  edges, *edges_ptr;
  TINEdges::iterator        start_edgeset;
  LaserPoints::iterator     point, nb_point, goodpoint, start_point;
  int                       first_number, i, count0, knn_max, inode;
  SegmentationParameters    par;
  bool                      debug=false, found;

  if (empty()) return;
  
  // Initialise all points and derive the TIN */
  SetAttribute(IsProcessedTag, 0); // Set all points unprocessed
  SetAttribute(IsSelectedTag, 1); // Accept all points

  edges_ptr = &edges;
  // Special case: Two points (no TIN creation possible)
  if (size() == 2) {
    point = begin();
    nb_point = begin() + 1;
    if ((point->vect() - nb_point->vect()).Length2D() <= min_dist)
      nb_point->UnSelect();
  }
  
  // Regular case for three or more points
  else if (size() > 2) {
    first_number = 0;
    // Create neighbourhood edges
    if (knn) {
      if (debug) printf("Creating kd-tree of %d points\n");
      // Determine maximum number of neighbours
      knn_max = knn > size() ? size() : knn;
      par.NeighbourhoodStorageModel() = 2; // knn
      par.NumberOfNeighbours() = knn_max;
      // Derive edges from kd-tree
      edges_ptr = DeriveEdges(par);
    }
    else {
      // Create TIN and TIN edges
      if (debug) printf("Creating TIN\n");
      if (!tin) DeriveTIN();
      edges.Derive(TINReference());
      // Remove points without TIN edges. These are double XY points.
      for (point=begin(), count0=0, i=0; point!=end(); point++, i++) {
        if (!edges[i].size()) {
          count0++;
          point->UnSelect();
        }
      }
      if (count0 > (double) size() / 100.0)
        printf("Warning: more than 1\% of the edge sets (%d/%d) are empty\n",
               count0, size());
    }
    if (debug) printf("Minimum distance %.2f\n", min_dist);


    // Loop over all points, put the first unprocessed point with a
    // neighbourhood on the stack
    list.reserve(size());
    for (start_point=begin(), start_edgeset=edges_ptr->begin(), first_number=0; 
         start_point!=end(); start_point++, start_edgeset++, first_number++) {
      if (!start_point->Processed() && start_edgeset->size()) {
        // Initialise node list 
        list.Erase();
        list.push_back(PointNumber(first_number));
        if (knn == 0 && count0 > (double) size() / 100.0 && debug)
          printf("First point with (%d) edges is %d\n",
                 (edges_ptr->begin()+first_number)->size(), first_number);

        // Process all nodes
        for (node=list.begin(), i=0; node!=list.end(); node++, i++) {
          if ((i/100)*100 == i && debug) printf("Processed points %d\r", i);
          if (node->Number() < 0 || node->Number() >= size())
            printf("Invalid node number %d\n", node->Number());
          point = begin() + node->Number();

          // Accepted point, delete all points within specified distance
          if (point->IsSelected()) {
            nbhood = Neighbourhood(*node, min_dist, *edges_ptr, (knn==0));
            if (debug && node == list.begin())
              printf("Neighbourhood of first point has %d points\n", nbhood.size());
            for (nb_node=nbhood.begin(); nb_node!=nbhood.end(); nb_node++) {
              nb_point = begin() + nb_node->Number();
             nb_point->UnSelect();
            }
            point->Select(); // Undo unselection of central point
          }
 
          // Put neighbouring unprocessed points on list
          nbhood = (edges_ptr->begin() + node->Number())->PointNumberListReference();
          for (nb_node=nbhood.begin(); nb_node!=nbhood.end(); nb_node++) {
            nb_point = begin() + nb_node->Number();
            if (!nb_point->Processed()) {
              list.push_back(*nb_node);
              node = list.begin() + i; // Should not be necessary because of list.reserve(size())
              nb_point->SetProcessed();
            }
          }
          point->SetProcessed();

          // Search for other non-processed points if we're at the end of the list
          if (node == list.end() - 1) {
            found = false;
            for (point=begin(), inode=0; point!=end() && !found; point++, inode++) {
              if (!point->Processed()) {
                list.push_back(PointNumber(inode));
                found = true;
              }
            }
            if (found) node = list.end() - 2;
          }
        }
      }
    }
  }
  
  // Eliminate unselected points
  if (debug) printf("\nRemoving not selected points\n");
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    if (point->IsSelected()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
  
  // Remove selection and processed attributes
  if (debug) printf("Removing IsProcessedTag and IsSelectedTag attributes\n");
  RemoveAttribute(IsProcessedTag);
  RemoveAttribute(IsSelectedTag);
  
  // Erase edges
  edges_ptr->Erase();
  if (debug) printf("Done\n");
}

int Compare_double_int_pair(const void *ptr1, const void *ptr2)
{
  std::pair<double, int> *pair1, *pair2;
  
  pair1 = (std::pair<double, int> *) ptr1;
  pair2 = (std::pair<double, int> *) ptr2;
  if (pair1->first > pair2->first) return 1;
  if (pair1->first < pair2->first) return -1;
  return 0;
}

void LaserPoints::ReduceData(int knn_start, int knn_max, 
                             double optimal_reduction_factor)
{
  TINEdges::iterator        nbhood;
  PointNumberList::iterator nb_node;
  TINEdges                  *edges;
  LaserPoints::iterator     point, nb_point, goodpoint, middle_point,
                            swap_point;
  LaserPoint                swap_buffer;
  int                       i, num_selected, knn, knn_optimal,
                            num_additional_selection, num_additional_selected,
                            num_additional_candidates, num_selected_old,
                            total_additional_candidates, used_knn_max;
  SegmentationParameters    par;
  bool                      verbose=false, all_unselected;
  double                    reduction_factor;
  std::pair<double, int>    *process_index, *process_indices;

  // Check for small point sets (only keep first point)
  if (size() <= 1) return;
  if (size() <= optimal_reduction_factor) {
    erase(begin()+1, end());
    return;
  }
  // Initialise all points and derive the edges
  used_knn_max = knn_max;
  if (knn_max > size()) used_knn_max = size();
  par.NumberOfNeighbours() = used_knn_max;
  edges = DeriveEdges(par);
  
  // Generate random numbers to process the points in random order
  process_indices =
    (std::pair<double, int> *) malloc(sizeof(std::pair<double, int>) * size());
  Randomize();
  for (i=0, process_index=process_indices; i<size(); i++, process_index++) {
    process_index->first = GenerateRandom();
    process_index->second = i;
  }
  qsort((void *) process_indices, size(), sizeof(std::pair<double, int>),
        Compare_double_int_pair);
    
  // Find the knn for which the reduction is just above the optimal
  reduction_factor = optimal_reduction_factor - 1.0;
  num_selected = 0;
  for (knn=knn_start; knn<=used_knn_max && reduction_factor<optimal_reduction_factor;
       knn++) {
    SetAttribute(IsSelectedTag, 0); // Unselect all points
    num_selected_old = num_selected;
    num_selected = 0;
    knn_optimal = knn;
    // Select points
    for (i=0, process_index=process_indices; i<size(); i++, process_index++) {
      point = begin() + process_index->second;
      nbhood = edges->begin() + process_index->second;
      for (nb_node=nbhood->begin()+1, all_unselected=true;
           nb_node!=nbhood->begin()+knn && all_unselected; nb_node++) {
        nb_point = begin() + nb_node->Number();
        if (nb_point->IsSelected()) all_unselected=false;
      }
      if (all_unselected) {
        point->Select();
        num_selected++;
      }
    }
    reduction_factor = (double) size() / (double) num_selected;
    if (verbose) printf("knn %d, reduction %.2f\n", knn, reduction_factor);
  }
  
  // Fine tuning of the reduction, see how many more points need to be accepted
  num_additional_selection = (int) (size() / optimal_reduction_factor) -
                             num_selected;
  total_additional_candidates = num_selected_old - num_selected;
  knn = knn_optimal - 1;
  // Accept some more points, this gives an approximation of the optimal
  // reduction, usually within 0.1 or 0.2%.
  if (knn >= knn_start) {
    num_additional_selected = num_additional_candidates = 0;
    for (i=0, process_index=process_indices; i<size(); i++, process_index++) {
      point = begin() + process_index->second;
      nbhood = edges->begin() + process_index->second;
      if (!point->IsSelected()) {
        for (nb_node=nbhood->begin()+1, all_unselected=true;
             nb_node!=nbhood->begin()+knn && all_unselected; nb_node++) {
          nb_point = begin() + nb_node->Number();
          if (nb_point->IsSelected()) all_unselected=false;
        }
        if (all_unselected) num_additional_candidates++;
        if ((float) num_additional_selected <
            (float) num_additional_candidates * (float) num_additional_selection /
            (float) total_additional_candidates) {
          point->Select();
          num_additional_selected++;
        }
      }
    }
  }
  
  // Do the rest by subsampling
  num_selected += num_additional_selected;
  num_additional_selection = (int) (size() / optimal_reduction_factor) -
                             num_selected;
  num_additional_selected = 0;
  num_additional_candidates = 0;
  if (num_additional_selection > 0) {
    total_additional_candidates = size() - num_selected;
    for (point=begin(); point!=end(); point++) {
      if (!point->IsSelected()) {
        num_additional_candidates++;
        if ((float) num_additional_selected <
            (float) num_additional_candidates * (float) num_additional_selection /
            (float) total_additional_candidates) {
          point->Select();
          num_additional_selected++;
        }
      }
    }
  }
  else if (num_additional_selection < 0) {
    num_additional_selection = -num_additional_selection;
    total_additional_candidates = num_selected;
    for (point=begin(); point!=end(); point++) {
      if (point->IsSelected()) {
        num_additional_candidates++;
        if ((float) num_additional_selected <
            (float) num_additional_candidates * (float) num_additional_selection /
            (float) total_additional_candidates) {
          point->UnSelect();
          num_additional_selected++;
        }
      }
    }
  }
  
  // Eliminate unselected points
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    if (point->IsSelected()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  i = size();
  erase(goodpoint, end());
  if (verbose) 
    printf("Point set reduced from %d to %d points (%.1f)\n",
           i, size(), 100.0 * size() / (double) i);
  
  // Remove IsSelected attribute
  RemoveAttribute(IsSelectedTag);
  // Erase edges
  edges->Erase();
  delete edges;
  free(process_indices);
}

/*
--------------------------------------------------------------------------------
                       Conversion to object points
--------------------------------------------------------------------------------
*/

ObjectPoints LaserPoints::ConstructObjectPoints() const
{
  Covariance3D          covariance;
  int                   num;
  ObjectPoints          objectpoints;
  LaserPoints::const_iterator laserpoint;
  PointNumber           number;

  for (laserpoint=begin(), num=0; laserpoint!=end(); laserpoint++, num++) {
    number.Number() = num;
    objectpoints.push_back(laserpoint->ConstructObjectPoint(number,covariance));
  }
  return(objectpoints);
}

/*
--------------------------------------------------------------------------------
                  Median distance between consecutive points
--------------------------------------------------------------------------------
*/

int Compare_Double(const void *v1, const void *v2)
{
  double *d1, *d2;

  d1 = (double *) v1;  d2 = (double *) v2;
  if (*d1 < *d2) return(-1);
  else if (*d1 > *d2) return(1);
  else return(0);
}

double LaserPoints::MedianInterPointDistance(int num_pairs) const
{
  double                      *dist_list, *dist, median_dist;
  LaserPoints::const_iterator pt1, pt2;
  int                         num;

/* Check if there are sufficient points */

  if (size() < num_pairs + 1) {
    fprintf(stderr, "Only %d points in the data set.\n", size());
    fprintf(stderr, "%d points are needed to calculate %d distances.\n",
            num_pairs+1, num_pairs);
    exit(0);
  }

/* Put the distances in an array */

  dist_list = (double *) malloc(num_pairs * sizeof(double));
  for (num=0, pt1=begin(), pt2=pt1+1, dist=dist_list;
       num<num_pairs;
       num++, pt2=pt1, pt1++, dist++) {
    *dist = (*pt1 - *pt2).Length2D();
  }

/* Sort the array of distances and return the middle element */

  qsort((void *) dist_list, num_pairs, sizeof(double), Compare_Double);
  median_dist = *(dist_list + num_pairs / 2);
  free(dist_list);
  return(median_dist);
}

/*
--------------------------------------------------------------------------------
          Calculate zero, first and second order non central moments
--------------------------------------------------------------------------------
*/

void LaserPoints::Moments(double &m00, double &m10, double &m01,
                          double &m20, double &m11, double &m02) const
{
  LaserPoints::const_iterator point;

  m00 = size();
  m10 = m01 = m20 = m11 = m02 = 0.0;
  for (point=begin(); point!=end(); point++) {
    m10 += point->X();
    m01 += point->Y();
    m20 += point->X() * point->X();
    m11 += point->X() * point->Y();
    m02 += point->Y() * point->Y();
  }
}

/*
--------------------------------------------------------------------------------
                        Smooth a polygon of laser points
--------------------------------------------------------------------------------
*/

LineTopology LaserPoints::SmoothOutline(const LineTopology &top,
                                        double max_dist_outlier,
                                        double min_dist_bound_points) const
{

  PointNumberList           component;
  TINEdges                  edges;
  int                       i, node_added;
  LaserPoints               end_points;
  LineTopology              outline, final_outline;
  PointNumberList::const_iterator node, node1, node2;
  LaserPoints::const_iterator cpoint;
  LaserPoints::iterator point, point1, point2, best_point;
  Line2D                    line;
  double                    edgelength, dist, best_dist, edge_offset1,
                            edge_offset2, point_offset;

// Create a point set with the end points off all scan lines

  for (node=top.begin(), i=0; node!=top.end()-1; node++, i++) {
    cpoint = begin() + node->Number();
    end_points.push_back(*cpoint);
    component.push_back(PointNumber(i));
  }

// Construct TIN and TIN edges

  end_points.Label(0);                       // Initialise all labels.
  end_points.DeriveTIN();                    // Compute a TIN 
  edges.Derive(end_points.TINReference());   // Derive TIN edges.

// Derive convex hull as initial outline and label outline points

  outline = end_points.DeriveContour(0, component, edges);
  for (node=outline.begin(); node!=outline.end(); node++) {
    point = end_points.begin() + node->Number();
    point->Label(1);
  }

// Reconstruct outline with sorted point numbers

  outline.erase(outline.begin(), outline.end());
  for (point=end_points.begin(), i=0; point!=end_points.end(); point++, i++)
    if (point->Label()) outline.push_back(PointNumber(i));
  outline.push_back(*(outline.begin()));

// Determine a line along the main strip orientation
// Currently not used.

  node = outline.begin() + 5;
  point1 = end_points.begin() + node->Number();
  node = outline.begin() + outline.size()/2 - 5;
  point2 = end_points.begin() + node->Number();
  line = Line2D(Position2D(point1->X(), point1->Y()),
                Position2D(point2->X(), point2->Y()));

// Refine the outline

  do {
    node_added = 0;

// Label points that are within the specified distance from the outline

    node1 = outline.begin();
    point1 = end_points.begin() + node1->Number();
    for (node2=node1+1; node2!=outline.end(); node2++) {
      point2 = end_points.begin() + node2->Number();
//    edge_offset1 = line.DistanceToPoint(Position2D(point1->X(), point1->Y()));
//    edge_offset2 = line.DistanceToPoint(Position2D(point2->X(), point2->Y()));
//    edge_offset = line.DistanceToPoint(
//      Position2D((point1->X() + point2->X())/2.0,
//                 (point1->Y() + point2->Y())/2.0));
      line = Line2D(Position2D(point1->X(), point1->Y()),
                    Position2D(point2->X(), point2->Y()));
      edgelength = (point1->vect() - point2->vect()).Length2D();
      if (edgelength > min_dist_bound_points) {
        best_dist = max_dist_outlier;
        for (i=node1->Number()+1; i<node2->Number(); i++) {
          point = end_points.begin() + i;
          dist = line.DistanceToPoint(Position2D(point->X(),point->Y()));
//        dist = MIN(fabs(edge_offset1 - point_offset),
//                   fabs(edge_offset2 - point_offset));
//        dist = fabs(edge_offset - point_offset);
          if (dist < best_dist) {
            best_dist = dist;
            best_point = point;
          }
        }
        if (best_dist < max_dist_outlier) {
          best_point->Label(1);
          node_added = 1;
        }
      }
      node1 = node2;
      point1 = point2;
    }

// Make a new outline if points were added

    if (node_added) {
      outline.erase(outline.begin(), outline.end());
      for (point=end_points.begin(), i=0;
           point!=end_points.end();
           point++, i++) {
        if (point->Label()) outline.push_back(PointNumber(i));
      }
      outline.push_back(*(outline.begin()));
    }
  } while (node_added);

// Reduce the number of outline points and convert to the original node numbers

  node1 = outline.begin();
  point1 = end_points.begin() + node1->Number();
  node = top.begin() + node1->Number();
  final_outline.push_back(*node);
  for (node2=node1+1; node2!=outline.end()-1; node2++) {
    point2 = end_points.begin() + node2->Number();
    dist = (point1->vect() - point2->vect()).Length2D();
    if (dist > min_dist_bound_points) {
      node = top.begin() + node2->Number();
      final_outline.push_back(*node);
      point1 = point2;
    }
  }
  node = top.begin() + outline.begin()->Number();
  final_outline.push_back(*node);


  end_points.ErasePoints();
  return(final_outline);
}

/*
--------------------------------------------------------------------------------
                        Derive TIN edges for 3D connectivity
--------------------------------------------------------------------------------
*/

TINEdges * LaserPoints::NeighbourhoodEdges3D(const TINEdges &edges,
                                             double range) const
{
  int number;
  TINEdges *edges_3d;
  bool show_progress=false;
  
  edges_3d = new TINEdges();
  for (number=0; number<size(); number++) {
    edges_3d->push_back(TINEdgeSet(Neighbourhood3D(PointNumber(number),
                                                   range, edges)));
    if (show_progress) {
      if ((number / 1000) * 1000 == number)
        printf("%9d %9d\r", number, (edges_3d->end()-1)->size()); fflush(stdout);
    }
  }
  if (show_progress) printf("\n");
  return edges_3d;
}

void CountEdges(const TINEdges &edges, char *string)
{
  TINEdges::const_iterator edgeset;
  int                      num=0, num0=0;

  for (edgeset=edges.begin(); edgeset!=edges.end(); edgeset++) {
    if (edgeset->size() == 0) num0++;
    else num += edgeset->size();
  }
  printf("%s %d edges and %d empty edge sets\n", string, num, num0);
}

TINEdges * LaserPoints::TINEdges3D(double range)
{
  double   *coordlist;
  TINEdges *tin_edges, *edges_3d, *extra_edges_3d;
  TIN      *extra_tin;
  LineTopologies *top;

  // Derive a normal TIN and TIN edges in the XY plane
  printf("Triangulation in XZ plane\n");
  DeriveTIN();
  printf("TIN derived\n");
  tin_edges = new TINEdges();
  tin_edges->Derive(TINReference());
  CountEdges(tin_edges->TINEdgesRef(), "  tin_edges");
  edges_3d = NeighbourhoodEdges3D(tin_edges->TINEdgesRef(), range);
  CountEdges(edges_3d->TINEdgesRef(), "  edges_3d");
  tin_edges->Erase();
  
  // Triangulate in the XZ plane and derive and add the resulting edges
  printf("Triangulation in XZ plane\n");
  SwapYZ();
  coordlist = TriangleCoordinateList();
  extra_tin = new TIN(coordlist, size());
  printf("TIN derived\n");
  tin_edges->Derive(extra_tin->TINReference());
  CountEdges(tin_edges->TINEdgesRef(), "  tin_edges");
  extra_edges_3d = NeighbourhoodEdges3D(tin_edges->TINEdgesRef(), range);
  CountEdges(extra_edges_3d->TINEdgesRef(), "  extra_edges_3d");
  edges_3d->Add(extra_edges_3d->TINEdgesRef());
  CountEdges(edges_3d->TINEdgesRef(), "  merged edges_3d");
  free(coordlist);
  delete extra_tin;
  tin_edges->Erase();

  // Do the same in the YZ plane
  printf("Triangulation in YZ plane\n");
  SwapXZ();
  coordlist = TriangleCoordinateList();
  extra_tin = new TIN(coordlist, size());
  printf("TIN derived\n");
  tin_edges->Derive(extra_tin->TINReference());
  CountEdges(tin_edges->TINEdgesRef(), "  tin_edges");
  extra_edges_3d = NeighbourhoodEdges3D(tin_edges->TINEdgesRef(), range);
  CountEdges(extra_edges_3d->TINEdgesRef(), "  extra_edges_3d");
  edges_3d->Add(extra_edges_3d->TINEdgesRef());
  top = extra_edges_3d->EdgeTopologies();
  top->Write("edges.top", false);
  CountEdges(edges_3d->TINEdgesRef(), "  merged edges_3d");
  free(coordlist);
  delete extra_tin;
  tin_edges->Erase();

  printf("3D triangulation done.\n");
  return edges_3d;
}

/*
--------------------------------------------------------------------------------
                              Swap coordinates
--------------------------------------------------------------------------------
*/

void LaserPoints::SwapXY()
{
  for (LaserPoints::iterator point=begin(); point!=end(); point++)
    point->SwapXY();
}

void LaserPoints::SwapXZ()
{
  for (LaserPoints::iterator point=begin(); point!=end(); point++)
    point->SwapXZ();
}

void LaserPoints::SwapYZ()
{
  for (LaserPoints::iterator point=begin(); point!=end(); point++)
    point->SwapYZ();
}

/*
--------------------------------------------------------------------------------
                   Check if a point is part of this point set
--------------------------------------------------------------------------------
*/

bool LaserPoints::Contains(const LaserPoint &point) const
{
  LaserPoints::const_iterator pt;

  for (pt=begin(); pt!=end(); pt++)
    if (*pt == point) return true;
  return false;
}
/*
--------------------------------------------------------------------------------
                       Read/Save the laser points from ascii and ptx files
--------------------------------------------------------------------------------
*/
//loads from a text file.
int LaserPoints::LoadFromAscii(const char* fileName,const char* delimiters,vector<int> order)
{
	LaserPoints& laserPoints = *this;
	FILE* pFile = fopen(fileName,"rt");
	

	if(!pFile)
	{
		cerr<<"Failed to open "<<fileName<<endl;
		return -1;
	}
	
	//Get the file size.
	fseek (pFile,0,SEEK_END);
	long fileSize = ftell(pFile);
	fseek(pFile,0,SEEK_SET);
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(fileSize,"Reading points from ascii");


	
	//Make and order if we don't have one.
	if(order.size()<3)
	{
		order.resize(4);
		for(int i=0;i<order.size();i++)
			order[i] = i;
		
	}
	
	int max_order = *(max_element(order.begin(),order.end()));
	
	int line_count = 0;
	int points_read = 0;
	//Keep reading and parsing the file, line by line.
	while(1)
	{
		if(feof(pFile))
			break;
		line_count++;
		
		vector<string> fields = ReadTokensFromLine(pFile,delimiters);
		
		if(fields.size()<3)
		{
			cerr<<"\nskipping line number "<<line_count
				<<" as tokens are "<<fields.size()
				<<" which is less than "<< 3<<endl;
			continue;
		}
		else
		{
/*			
			for(int i=0;i<fields.size();i++)
				cerr<<fields[i]<<"|";
			cerr<<endl;
			
*/			
			if(laserPoints.size()%10000	== 0)
				cerr<<"Read "<<laserPoints.size()/1000<<"K points\r";

			LaserPoint pt;
			pt.X() = atof(fields[order[0]].c_str());
			pt.Y() = atof(fields[order[1]].c_str());
			pt.Z() = atof(fields[order[2]].c_str());
			if(order.size()>3 && fields.size()>3)
				pt.Reflectance() =  (int)atof(fields[order[3]].c_str());
			laserPoints.push_back(pt);
			progress.Step("Reading points from ascii",ftell(pFile));
		}					
	}
	fclose(pFile);
	progress.End();
	//cerr<<"Now laser points has "<<laserPoints.size()<<endl;
	return 0;
}

//loads from a ptx file.
//Contains entry about transformation in ten lines.
//First two are the number of points.
//Next 4 can be skipped.
//Last 4 given full transformation matrix, that has to be pre-multiplied by points in a row vector.
//
// [x y z 1] * T 
//
int LaserPoints::LoadFromPtx(const char* fileName)
{
	FILE* pFile = fopen(fileName,"rt");

	if(!pFile)
	{
		cerr<<"Failed to open "<<fileName<<endl;
		return -1;
	}
	
	const char* delimiters = " \t:,;";
	vector<int> order=vector<int>();
	//Make and order if we don't have one.
	if(order.size()<3)
	{
		order.resize(4);
		for(int i=0;i<order.size();i++)
			order[i] = i;
		
	}
	
	int max_order = *(max_element(order.begin(),order.end()));
	
	///Initialize this object.
	this->resize(0);
	
	int line_count = 0;
	int points_read = 0;
	//Keep reading and parsing the file, line by line.
	Vector3D r[3], t(0,0,0);
	while(1)
	{
		if(feof(pFile))
			break;
		line_count++;
		
		vector<string> fields = ReadTokensFromLine(pFile,delimiters);
				
		if(fields.size()<3)
		{
			if(fields.size()==1)
			{
				//Another number.
				fields = ReadTokensFromLine(pFile,delimiters);
				
				//skip next 4 lines;
				for(int j=0;j<4;j++)
				{
					fields = ReadTokensFromLine(pFile,delimiters);
				}
				
				//rotation.
				for(int j=0;j<3;j++)
				{
					fields = ReadTokensFromLine(pFile,delimiters);
					
					for(int i=0;i<3;i++)
						r[i][j] = atof(fields[order[i]].c_str());
				}
				
				//translation.
				fields = ReadTokensFromLine(pFile,delimiters);
				
				for(int i=0;i<3;i++)
					t[i] = atof(fields[order[i]].c_str());
				
				
				cerr<<"\nRead a new transform: \n"<<endl;		
				//continue.
				continue;
				
			}
			else
			{
				cerr<<"\nskipping line number "<<line_count
					<<" as tokens are "<<fields.size()
					<<" which is less than "<< 3<<endl;
				continue;
			}
		}
		else
		{
			LaserPoint pt;
			pt.X() = atof(fields[order[0]].c_str());
			pt.Y() = atof(fields[order[1]].c_str());
			pt.Z() = atof(fields[order[2]].c_str());
			
			LaserPoint qt;
			for(int i=0;i<3;i++)
				qt[i] = r[i].DotProduct(pt) + t[i];
				
			double ref = 0;
			if(order.size()>3 && fields.size()>3)
				ref =  atof(fields[order[3]].c_str());
				
			this->push_back(LaserPoint(qt[0],qt[1],qt[2],(int)ref));
			points_read++;
			
			if(points_read%1000 == 0)
				cerr<<"Read "<<points_read/1000<<"K points\r";
				
		}					
	}
	fclose(pFile);
	return 0;
}



int LaserPoints::SaveToAscii(const char* fileName,const char* format ,char* delimiters)const
{
	const LaserPoints& laserPoints = *this;
	const char *use_format, *use_delimiters;
	FILE* pFile = fopen(fileName,"wt");

	if(!pFile)
	{
		cerr<<"Failed to open "<<fileName<<endl;
		return -1;
	}
	
	//local variables.
	const int buffsize = 1024*64;
	char buff[buffsize];
		
	if (format) use_format = format;
	else use_format = "%8.5f";
	if (delimiters) use_delimiters = delimiters;
	else use_delimiters = " ";
	
	snprintf(buff,buffsize,"%s%s %s%s %s%s %s%s %%d %%d %%d %%d %%ld\n",
	         use_format, use_delimiters, use_format, use_delimiters,
			 use_format, use_delimiters, use_format, use_delimiters);
	
	
	for(int i=0;i<laserPoints.size();i++)
	{
		LaserPoint pt = laserPoints[i];
		fprintf(pFile,buff,pt.X(),pt.Y(),pt.Z(),(double)pt.Reflectance(),
				(int)pt.Red(),(int)pt.Green(),(int)pt.Blue(),(int)pt.PulseCount(),(long long int)(pt.GetId()));
	}
	
	fclose(pFile);
	return 0;
}

/*
--------------------------------------------------------------------------------
                           Various Functions - added by Tahir
--------------------------------------------------------------------------------
*/

/// + operator, combines two laser points
LaserPoints LaserPoints::operator +(const LaserPoints& b)const
{
	LaserPoints pts = *this;
	pts.reserve(size()+b.size());
	for(int i=0;i<b.size();i++)
		pts.push_back(b[i]);

	return pts;
}

///Set reflectance of all points to one value.
LaserPoints& LaserPoints::SetReflectance(double d)
{
	for(int i=0; i<size();i++)
		(*this)[i].Reflectance() = (int)d;

	return *this;
}


///Collect ids of all points and return them as a vector.
vector<long long int> LaserPoints::GetIds() const
{
	vector<long long int> ids(size());

	for(int i=0;i<size();i++)
		ids[i] = (*this)[i].GetId();

	return ids;
}


/// Retrieve readable attribute value
vector<int> LaserPoints::GetAttribute(const LaserPointTag tag) const
{
	vector<int> att(size());
	
	for(int i=0;i<size();i++)
		att[i] = (*this)[i].Attribute(tag);
		
	return att;
}

vector<int> LaserPoints::GetReflectance() const
{
	return GetAttribute(ReflectanceTag);	
}

//Trasform all points and return in the form of another LaserPoints.
LaserPoints LaserPoints::operator*(const Rotation3D& r)const
{
	LaserPoints pts = *this;
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = r*Vector3D((pts)[i]);
		pts[i][0]=v[0];pts[i][1]=v[1];pts[i][2]=v[2];
	}
	return pts;
}

///Trasform all points using orientation 3D.
LaserPoints LaserPoints::operator*(const Orientation3D& r)const
{
	LaserPoints pts = *this;
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = r*Vector3D((pts)[i]);
		pts[i][0]=v[0];pts[i][1]=v[1];pts[i][2]=v[2];
	}
	return pts;
}

//Traslate all points.
LaserPoints LaserPoints::operator+(const Vector3D& r)const
{
	LaserPoints pts = *this;
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = r + Vector3D((*this)[i]);
		pts[i][0]=v[0];pts[i][1]=v[1];pts[i][2]=v[2];
	}
	return pts;
}

//Traslate all points.
LaserPoints LaserPoints::operator-(const Vector3D& r)const
{
	LaserPoints pts = *this;
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = Vector3D((*this)[i]) - r;
		pts[i][0]=v[0];pts[i][1]=v[1];pts[i][2]=v[2];
	}
	return pts;
}


//Add noise to all points.
LaserPoints LaserPoints::AddNoise(double level)const
{
	LaserPoints pts = *this;
	srand(clock());

	for(int i=0;i<pts.size();i++)
	{
		double r = level*(2*rand()/(double)RAND_MAX - 1);
		(pts)[i] = r*Vector3D(1,1,1) + Vector3D((*this)[i]);
	}
	return pts;
}

//Add gaussian noise to all points.
LaserPoints LaserPoints::AddNoiseG(double stdDev, double mean)const
{
	LaserPoints pts = *this;
	srand(clock());

	for(int i=0;i<pts.size();i++)
	{
		//TODO: Put a Gaussian random generator from Numerical recipes here.
		double r = stdDev*(2*rand()/(double)RAND_MAX - 1);
		(pts)[i] = r*Vector3D(1,1,1) + Vector3D((*this)[i]);
	}
	return pts;
}

///Rotate and translate by the given amount at the same time.
LaserPoints LaserPoints::Transform(const Rotation3D& rot,const Vector3D&trans)const
{
	LaserPoints pts = *this;

	for(int i=0;i<pts.size();i++)
	{
		(pts)[i] = rot*Vector3D((pts)[i]);
		pts[i] = pts[i] + trans;
	}
	return pts;
}

///Calculate the mean of the laser points.
Vector3D LaserPoints::Mean()const
{
	Vector3D m;
	if(empty())
		return m;

	double factor = 1.00/(double)size();

	for(int i=0;i<size();i++)
		m += (Vector3D((*this)[i])*factor);
	return m;
}


///Calculate normal using all points. Also return rho.
Vector3D LaserPoints::Normal(double* pResidual,double *pRho,vector<Vector3D>* directions)const
{
	LaserPoints pts = *this;
	
	//subtract the mean
	Vector3D mean = pts.Mean();
	pts = pts + (mean*-1);
	
	//Allocate variance matrix.
	SymmetricMatrix M(3);
	M = 0;
	
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = pts[i];
		M(1,1) += v.X()*v.X();
		M(1,2) += v.X()*v.Y();
		M(1,3) += v.X()*v.Z();
		M(2,2) += v.Y()*v.Y();
		M(2,3) += v.Y()*v.Z();
		M(3,3) += v.Z()*v.Z();
	}
	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	
	Jacobi(M,D,V);
	
	//The residual equal eigen value with minimum value.
	if(pResidual)
		*pResidual = D(1);
	
	NEWMAT::Matrix q = V.Column(1);
	Vector3D normal= Vector3D(q(1,1),q(2,1),q(3,1)).Normalize();
	//The Rho of the plane.
	if(pRho)
		*pRho = (mean.DotProduct(normal));
		
	if(directions)
	{
		directions->resize(3);
		for(int i=0;i<3;i++)
		{
			NEWMAT::Matrix q = V.Column(i+1);
			Vector3D temp;
			temp = Vector3D(q(1,1),q(2,1),V(3,1));
			
			if(temp.Length()>1e-12)
				temp = temp.Normalize();
			
			(*directions)[i] = temp;
			
		}
	}
	
	//The eigenvector with minimum eigenvalues is the one we want.
	return normal;
}

///Calculate normal using all points. Also return rho.
Vector3D LaserPoints::NormalAndEigenValues(Vector3D &eigenvalues)const
{
	LaserPoints pts = *this;
	
	//subtract the mean
	Vector3D mean = pts.Mean();
	pts = pts + (mean*-1);
	
	//Allocate variance matrix.
	SymmetricMatrix M(3);
	M = 0;
	
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = pts[i];
		M(1,1) += v.X()*v.X();
		M(1,2) += v.X()*v.Y();
		M(1,3) += v.X()*v.Z();
		M(2,2) += v.Y()*v.Y();
		M(2,3) += v.Y()*v.Z();
		M(3,3) += v.Z()*v.Z();
	}
	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	
	Jacobi(M,D,V);
	
	//The residual equal eigen value with minimum value.
	eigenvalues.X() = D(1);
	eigenvalues.Y() = D(2);
	eigenvalues.Z() = D(3);
	
	NEWMAT::Matrix q = V.Column(1);
	Vector3D normal= Vector3D(q(1,1),q(2,1),q(3,1)).Normalize();
	
	//The eigenvector with minimum eigenvalues is the one we want.
	return normal;
}

//Calculate Curvature using new-mat.
double LaserPoints::Curvature(double* majorCurvature,double* minorCurvature,Vector3D* majorDirection,Vector3D* minorDirection)const
{
	const LaserPoints& laserPoints = *this;
	int pointCount = laserPoints.size();
	
	NEWMAT::Matrix A(pointCount,3);

	LaserPoint mean = laserPoints.Mean();
	//Do plane fitting and get orthogonal frame.
	for(int j=0;j<laserPoints.size();j++)
	{
		for(int i=0;i<3;i++)
			A(j+1,i+1) = laserPoints[j][i] - mean[i];
	}
	
	SymmetricMatrix C; 
	C << ((A.t()) * A);
	
	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(3);
	NEWMAT::Matrix V(3,3);
	Jacobi(C,D,V);
	
	int minLoc;
	D.MinimumAbsoluteValue1(minLoc);
	//cerr<<"MinLoc: "<<minLoc<<endl;
	int locations[3];
	locations[2] = minLoc;
	
	if(minLoc==1)
	{
		locations[0]=2;locations[1]=3;
	}
	else if(minLoc==2)
	{
		locations[0]=1;locations[1]=3;
	}
	else
	{
		locations[0]=1;locations[1]=2;
	}
	
	NEWMAT::Matrix VCopy = V;
	
	for(int i=0;i<3;i++)
	{
		//cerr<<"i: "<<i<<endl;
		for(int j=0;j<3;j++)
			V(j+1,i+1) = VCopy(j+1,locations[i]);
	}
	
	if(V.Determinant()<0)
	{
		for(int j=0;j<3;j++)
			V(j+1,1) = -V(j+1,1);
	}
		
	
	//Now we need to rotate to the new axis system.
	NEWMAT::Matrix Ar = (V*(A.t())).t();	
		
	//Now lets fit a biquadratic surface.
	NEWMAT::Matrix B(pointCount,6);
	NEWMAT::Matrix yy(pointCount,1);
	for(int i=0;i<pointCount;i++)
	{
		double x = Ar(i+1,1);
		double y = Ar(i+1,2);
		double z = Ar(i+1,3);
		
		B(i+1,1) = x*x;
		B(i+1,2) = y*y;
		B(i+1,3) = x*y;
		B(i+1,4) = x;
		B(i+1,5) = y;
		B(i+1,6) = 1;
		
		yy(i+1,1) = z;
	}
		
	//Fit through LSQ fitting.
	NEWMAT::Matrix P = ((B.t()) * B);
	double eps = 0;//1e-16;
	P(1,1) += eps;
	P(2,2) += eps;
	
	NEWMAT::Matrix Vb = P.i()*(B.t()*yy);
	
	//cerr<<"Params: "<<Vb<<endl;
	
	//Now we need the shape operator.
	SymmetricMatrix S(2);
	S.Row(1)<<2*Vb(1,1);
	S.Row(2)<<Vb(3,1)<<2*Vb(2,1);
	DiagonalMatrix Ds;
	NEWMAT::Matrix Vs;
	Jacobi(S,Ds,Vs);
		
	//cerr<<"Ds: "<<Ds<<endl;
	//cerr<<"Vs: "<<Vs<<endl;
	
	int minIndex = fabs(Ds(2))>fabs(Ds(1))?1:2;
	int maxIndex = (minIndex==1)?2:1;
	//Copy the values back.
	double curvature = Ds(maxIndex);
	if(majorCurvature)
		*majorCurvature =Ds(maxIndex);
		
	if(minorCurvature)
		*minorCurvature = Ds(minIndex);
		
	if(majorDirection)
	{
		NEWMAT::Matrix v(3,1);
		v(1,1) = Vs(1,maxIndex);
		v(2,1) = Vs(2,maxIndex);
		v(3,1) = 0;
		NEWMAT::Matrix dir = (V.t())*v;
		*majorDirection = Vector3D(dir(1,1),dir(2,1),dir(3,1));
		//cerr<<"major direction: "<<(*majorDirection)<<endl;
		//cerr<<"determinant of rotation matrix: "<<V.Determinant()<<endl;
		//cerr<<"determinant of shape eig vector: "<<Vs.Determinant()<<endl;
	}
									
	if(minorDirection)
	{
		NEWMAT::Matrix v(3,1);
		v(1,1) = Vs(1,minIndex);
		v(2,1) = Vs(2,minIndex);
		v(3,1) = 0;
		NEWMAT::Matrix dir = (V.t())*v;
		*minorDirection = Vector3D(dir(1,1),dir(2,1),dir(3,1));

	}								

	//return the result.
	return curvature;
	

}

std::vector<double> LaserPoints::Curvatures(
					int kNN,
					std::vector<double>* pMajorCurvatures,
					std::vector<double>* pMinorCurvatures,
					std::vector<Vector3D>* pMajorDirs,
					std::vector<Vector3D>* pMinorDirs)
{
	LaserPoints& laserPoints = (*this);
	vector<double> ans(laserPoints.size());
	
	if(pMajorCurvatures)
		pMajorCurvatures->resize(laserPoints.size());	
		
	if(pMajorDirs)
		pMajorDirs->resize(laserPoints.size());	

	if(pMinorCurvatures)
		pMinorCurvatures->resize(laserPoints.size());	

	if(pMinorDirs)
		pMinorDirs->resize(laserPoints.size());
		
	KNNFinder<LaserPoint> finder(laserPoints);		
	double val, majorC, minorC;
	Vector3D majorD, minorD;
	for(int i=0;i<laserPoints.size();i++)
	{
		val = laserPoints.Select(finder.FindIndices(laserPoints[i],kNN)).Curvature(&majorC,&minorC,&majorD,&minorD);	
		ans[i] = val;
		
		if(pMajorCurvatures)
			(*pMajorCurvatures)[i] = majorC;
		
		if(pMajorDirs)
			(*pMajorDirs)[i] = majorD;
	
		if(pMinorCurvatures)
			(*pMinorCurvatures)[i] = minorC;
	
		if(pMinorDirs)
			(*pMinorDirs)[i] = minorD;
	}			
	return ans;		
}					


//Fit a quadric. We have 10 parameters for it.
vector<double> LaserPoints::FitQuadric(Vector3D* pEigVectors , double* pEigenValues)const
{
	vector<double> result(10);
	
	if(this->size()<10)
	{
		cerr<<"Not enough points for LaserPoints::FitQuadric. "<<this->size()<<"\n";
		return result;
	}
	
	LaserPoints pts = *this;
	
	//subtract the mean
	pts = pts + (pts.Mean()*-1);
		
	//Matrix of values.
	NEWMAT::Matrix A(pts.size(),10);
	
	for(int i=0;i<pts.size();i++)
	{
		Vector3D v = pts[i];
		//square terms.
		A(i+1,1) = v.X()*v.X();
		A(i+1,2) = v.Y()*v.Y();
		A(i+1,3) = v.Z()*v.Z();
		
		//cross terms
		A(i+1,4) = 2*v.X()*v.Y();
		A(i+1,5) = 2*v.Y()*v.Z();
		A(i+1,6) = 2*v.Z()*v.X();
		
		//linear terms.
		A(i+1,7) = 2*v.X();
		A(i+1,8) = 2*v.Y();
		A(i+1,9) = 2*v.Z();
		
		//dc-term.
		A(i+1,10) = 1;
	}
	
	//Make the A'*A matrix, we know it is symmetrix.
	SymmetricMatrix C; C << (A.t()) * A;
	
	//Calculate eigen values and eigen vectors.
	DiagonalMatrix D(10);
	NEWMAT::Matrix V(10,10);
	
	Jacobi(C,D,V);
	
	//The residual equal eigen value with minimum value.
	double residual = D(1);
	
	//The eigenvector with minimum eigenvalues is the one we want.
	NEWMAT::Matrix t = V.Column(1);
	
	for(int i=0;i<10;i++)
		result[i] = t(i+1,1);
		
	//Try to find the type of quadric by analysis of e matrix.
	if(pEigVectors && pEigenValues)
	{
		SymmetricMatrix E(3);
		E.Row(1) << t(1,1);
    	E.Row(2) << t(4,1) << t(2,1);
		E.Row(3) << t(6,1) << t(5,1) << t(3,1);
	
		//Calculate eigen values and eigen vectors.
		DiagonalMatrix De(3);
		NEWMAT::Matrix Ve(3,3);
	
		Jacobi(E,De,Ve);
	
		//cerr<<"De: "<<De<<endl;
		//cerr<<"Ve: "<<Ve<<endl;
		
		for(int i=0;i<3;i++)
			pEigenValues[i] = De(i+1);
			
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				pEigVectors[i][j] = Ve(j+1,i+1);
	}
		
	//return the parameters of the quadric.
	return result;
}


//Subsample skipping every nth point.
LaserPoints LaserPoints::SubSampleSimple(int step)const
{
	step = max(2,step);
	
	LaserPoints pts;
	for(int i=0;i<size();i+=step)
	{
		pts.push_back((*this)[i]);
	}
	return pts;
}
	
//Subsample using space partition but with constant distance along each axis.
LaserPoints LaserPoints::SubSample(double stepSize)const
{
	Histo3D<int> partition(stepSize);
	partition.Add(*this);
	
	return partition.ToPoints();
}

//Saves to a passed vrml file.
void LaserPoints::SaveVrml(std::string fileName)const
{
  FILE *fd = fopen(fileName.c_str(), "w");
  if (!fd) return;
  // Implementation of Tahir's function through VRML_Write
  VRML_Write(fd);
  fclose(fd);
}

//Show as vrml.
void LaserPoints::ShowVrml(std::string msg)const
{
	static int counter = 0;
	char *buff;
	FILE *fd;
	
	buff = (char *) malloc(4096);
	sprintf(buff,"/tmp/__LaserPoints_%d_%s.wrl",counter++,msg.c_str());
	fd = fopen(buff, "w");
	if (!fd) return;
	VRML_Write(fd);
	fclose(fd);
	
	sprintf(buff,"vrmlview /tmp/__LaserPoints_%d_%s.wrl",counter++,msg.c_str());
	system(buff);
	free(buff);
}

///Histoequalize the reflectance.
LaserPoints LaserPoints::Histoequalize(const int binCount)const
{
	LaserPoints laserPoints = *this;
	
	vector<double> histogram(binCount);
	vector<double> lut(binCount);
	double scalingFactor;
	int i;
	double maxReflectance,minReflectance;
	
	//Find min and max of reflectance.
	maxReflectance = minReflectance = laserPoints[0].Reflectance();
	for(i=0;i<laserPoints.size();i++)
	{
		maxReflectance = MAX(maxReflectance,laserPoints[i].Reflectance());
		minReflectance = MIN(minReflectance,laserPoints[i].Reflectance());
	}
	
	scalingFactor = (double)(binCount-1)/(double)(maxReflectance-minReflectance);
	
	//Scale Reflectance to 0 to 255.
	for(i=0;i<laserPoints.size();i++)
	{
		laserPoints[i].Reflectance() = (int)((laserPoints[i].Reflectance()-minReflectance)*scalingFactor);
	}
	
	//Make histogram.
	for(i=0;i<laserPoints.size();i++)
	{
		histogram[laserPoints[i].Reflectance()]++;
	}
	
	//Calculative cumulative histogram.
	for(i=1;i<binCount;i++)
	{
		histogram[i] = histogram[i]+histogram[i-1];
	}
	
	//Calculate scaling factor, that will convert the cumulative histogram to lut.
	scalingFactor = (double)(binCount)/(double)(histogram[binCount-1]-histogram[0]+1e-3);
	
	//Update lut.
	for(i=0;i<binCount;i++)
	{
		lut[i] = scalingFactor*(histogram[i]-histogram[0]);
		//cerr<<"lut["<<i<<"]: "<<lut[i]<<endl;
	}
	
	//Apply lut to laserPoints Reflectances.
	for(i=0;i<laserPoints.size();i++)
	{
		laserPoints[i].Reflectance() = (int)lut[laserPoints[i].Reflectance()];
	}
	
	return laserPoints;
}


//Calculate normals for all points using kNN.
vector<Vector3D> LaserPoints::Normals(int kNN,bool showProgress,vector<double>* pResiduals)
{
	const int progressCount = 100;
	double residual;
	
	KNNFinder<LaserPoint> finder(*this);
	vector<Vector3D> normals(size());
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(this->size(),"Progress");
	
	
	if(pResiduals)
		pResiduals->resize(size());

	for(int i=0;i<size();i++)
	{
		if(showProgress)
			progress.Step("Calculating Normals: ",i);
			
		normals[i] = this->Select(finder.FindIndices((*this)[i],kNN)).Normal(&residual);
		
		if(pResiduals)
			(*pResiduals)[i] = residual;
	}
	progress.End();
	return normals;
}

//Calculate normals for all points using kNN 
//Also return rhos.
vector<Vector3D> LaserPoints::Normals(int kNN,vector<double>& rhos,bool showProgress,vector<double>* pResiduals)const
{
	const int progressCount = 100;
	KNNFinder<LaserPoint> finder(*this);
	vector<Vector3D> normals(size());
	rhos.resize(size());
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	progress.Initialize(this->size(),"Progress");
	
	
	if(pResiduals)
		pResiduals->resize(size());
		
	for(int i=0;i<size();i++)
	{
		if(showProgress)
			progress.Step("Calculating Normals: ",i);
			
		normals[i] = this->Select(finder.FindIndices((*this)[i],kNN)).Normal(pResiduals?&((*pResiduals)[i]):NULL,&(rhos[i]));
	}
	progress.End();
	return normals;
}
	
//Calculate normals for all points using kNN.
vector<Vector3D> LaserPoints::NormalsAndEigenValues(int kNN,
                                                    vector<Vector3D>* eigenvalues,
													bool showProgress)
{
	const int progressCount = 100;
	double residual;
    Vector3D eigen;
	
	KNNFinder<LaserPoint> finder(*this);
	vector<Vector3D> normals(size());
	
	ProgressDisplay<int>& progress = *(ProgressDisplayManager::GetProgressObject());
	if (showProgress) progress.Initialize(this->size(),"Progress");
	
	if(eigenvalues) eigenvalues->resize(size());

	for(int i=0;i<size();i++) {
	  if (showProgress) progress.Step("Calculating Normals: ",i);
	  normals[i] = this->Select(finder.FindIndices((*this)[i],kNN)).NormalAndEigenValues(eigen);
	  (*eigenvalues)[i] = eigen;
	}
	if (showProgress) progress.End();
	return normals;
}

void LaserPoints::CalculateScaledNormals(int knn)
{
  vector<Vector3D>           normals, all_eigenvalues;
  vector<Vector3D>::iterator normal, eigenvalues;
  float                      flatness;      
  LaserPoints::iterator      point;
//  LaserPoint                 normal_point;
//  LaserPoints                normal_points, scaled_normal_points;
                             
  // Compute normals
  normals = NormalsAndEigenValues(knn, &all_eigenvalues, false); 

  // Transfer values
  for (point=begin(), normal = normals.begin(),
	   eigenvalues=all_eigenvalues.begin();
	   point!=end(); point++, normal++, eigenvalues++) {
// Normally, one would use the smallest and largest eigenvalue to define flatness
// Here we use the smallest and middle eigenvalue. In this case linear features
// like branches will not be considered flat. This helps to separate trees
// from surfaces.
	flatness = ((*eigenvalues)[1] - (*eigenvalues)[0]) /
	           ((*eigenvalues)[1] + (*eigenvalues)[0]);
	point->FloatAttribute(FlatnessTag) = flatness;
	point->FloatAttribute(LinearityTag) =
	  ((*eigenvalues)[2] - (*eigenvalues)[1]) /
	  ((*eigenvalues)[2] + (*eigenvalues)[1]);
	flatness = flatness * flatness;
	if ((*normal)[2] < 0.0) flatness = -flatness;
	point->FloatAttribute(ScaledNormalXTag) = (*normal)[0] * flatness;
	point->FloatAttribute(ScaledNormalYTag) = (*normal)[1] * flatness;
	point->FloatAttribute(ScaledNormalZTag) = (*normal)[2] * flatness;
	
/*
	// Temporary code for visualisation
	if ((*normal)[2] < 0.0) normal_point = LaserPoint((*normal) * (-1.0));
	else normal_point = LaserPoint(*normal);
	normal_point.Attribute(LabelTag) = point->Attribute(LabelTag);
	normal_points.push_back(normal_point);
    normal_point = LaserPoint(point->ScaledNormal());
	normal_point.Attribute(LabelTag) = point->Attribute(LabelTag);
	scaled_normal_points.push_back(normal_point);
*/
  }
  
  all_eigenvalues.erase(all_eigenvalues.begin(), all_eigenvalues.end());
  normals.erase(normals.begin(), normals.end());
  
/*
  // Temporary code
  normal_points.Write("a_normals.laser", false, false);
  scaled_normal_points.Write("a_scaled_normals.laser", false, false);
  normal_points.ErasePoints();
  scaled_normal_points.ErasePoints();
*/
}

void LaserPoints::CalculateNormals(int knn)
{
  vector<Vector3D>           normals, all_eigenvalues;
  vector<Vector3D>::iterator normal, eigenvalues;
  float                      flatness;      
  LaserPoints::iterator      point;
  
  // Compute normals
  normals = NormalsAndEigenValues(knn, &all_eigenvalues, false); 

  // Transfer values
  for (point=begin(), normal = normals.begin(),
	   eigenvalues=all_eigenvalues.begin();
	   point!=end(); point++, normal++, eigenvalues++) {
	flatness = ((*eigenvalues)[2] - (*eigenvalues)[0]) /
	           ((*eigenvalues)[2] + (*eigenvalues)[0]);
	point->FloatAttribute(FlatnessTag) = flatness;
	point->FloatAttribute(LinearityTag) =
	  ((*eigenvalues)[2] - (*eigenvalues)[1]) /
	  ((*eigenvalues)[2] + (*eigenvalues)[1]);
	flatness = flatness * flatness;
	point->FloatAttribute(NormalXTag) = (*normal)[0];
	point->FloatAttribute(NormalYTag) = (*normal)[1];
	point->FloatAttribute(NormalZTag) = (*normal)[2];
  }
  
  all_eigenvalues.erase(all_eigenvalues.begin(), all_eigenvalues.end());
  normals.erase(normals.begin(), normals.end());
}

//Return the extents calculated by PCA.
vector<double> LaserPoints::Extents(vector<Vector3D>* pOrthogonalDirections)const
{

	vector<Vector3D> n;
	//Get principal directions.
	this->Normal(NULL,NULL,&n);

	//Project along those.
	vector<double> vX,vY,vZ;
	for(int i=0;i<size();i++)
	{
		Vector3D v = (*this)[i];
		vZ.push_back(v.DotProduct(n[0]));
		vX.push_back(v.DotProduct(n[1]));
		vY.push_back(v.DotProduct(n[2]));
	}
	//copy to extents.
	vector<double> extents;
	
	//Put negative extents so that the sorting works in descending order
	extents.push_back(-Max(vZ)+Min(vZ));
	extents.push_back(-Max(vX)+Min(vX));
	extents.push_back(-Max(vY)+Min(vZ));
	
	//Sort the extents.
	sort(extents.begin(),extents.end());
	
	//Reverse the signs to nullify the affects of previous change.
	for(int i=0;i<extents.size();i++)
	{
		extents[i] = -extents[i];
	}
	
	if(pOrthogonalDirections)
		*pOrthogonalDirections = n;
		
	return extents;
}


///Return the distance from the closest points from a given pointcloud. 
vector<double> LaserPoints::Distance(const LaserPoints& pts,double factor)const
{

	KNNFinder<LaserPoint> finder(*this);
	vector<double> distances(pts.size());
	
	vector<int> indices = finder.FindIndices((vector<LaserPoint>)pts,0);
	
	for(int i=0;i<pts.size();i++)
	{
		
		distances[i] = (pts[i]-(*this)[finder.FindIndex(pts[i])]).Length()*factor;
	}
		
	return distances;
}

///Distance from the closest point.
double LaserPoints::Distance(Vector3D v)const
{

	KNNFinder<LaserPoint> finder(*this);
		
	int index = finder.FindIndex(v,1);
	
	return ((*this)[index]-v).Length();
}
	
LaserPoints LaserPoints::ProjectToPlane(Vector3D zv)const
{
	Vector3D xv,yv;
	zv.PerpendicularVectors(xv, yv);

	LaserPoints pts;
	for(int k=0;k<size();k++)
	{
		double dx = xv.DotProduct((*this)[k]);	
		double dy = yv.DotProduct((*this)[k]);	
		pts.push_back(LaserPoint(dx,dy,0,1));
	}
	return pts;
}

///Return the reflectance range.
double LaserPoints::ReflectanceRange(double* pMin, double *pMax, double* pScaleFactor)const
{
	double minR = 1e20;
	double maxR = -1e20;
	
	for(int i=0; i<size();i++)
	{
		minR = std::min((double)((*this)[i].Reflectance()),minR);
		maxR = std::max((double)((*this)[i].Reflectance()),maxR);
	}
	if(pMin) *pMin = minR;
	if(pMax) *pMax = maxR;
	if(pScaleFactor) *pScaleFactor = 1.00/std::max(1e-12,(maxR-minR));
	
	return (maxR-minR);
}

int LaserPoints::SelectByTag(const LaserPointTag tag, LaserPoints &selection) const
{
  LaserPoints::const_iterator point;
  int                         initial_size = selection.size();
  
  for (point=begin(); point!=end(); point++)
    if (point->HasAttribute(tag)) selection.push_back(*point);
  return selection.size() - initial_size;
}

int LaserPoints::SelectByNotTag(const LaserPointTag tag, LaserPoints &selection) const
{
  LaserPoints::const_iterator point;
  int                         initial_size = selection.size();
  
  for (point=begin(); point!=end(); point++)
    if (!point->HasAttribute(tag)) selection.push_back(*point);
  return selection.size() - initial_size;
}

LaserPoints LaserPoints::SelectTagValue(const LaserPointTag tag,int i) const
{
            LaserPoints tmp;
            tmp.Initialise();
            for(int k=0;k<size();k++)
            {
                    if(((*this)[k]).Attribute(tag)==i)
                    tmp.push_back((*this)[k]);
                    }
            
            return tmp;
            
            }
            
PointNumberList LaserPoints::SelectTagValueList(const LaserPointTag tag, int value) const
{
           PointNumberList list;   
           for(int k=0;k<size();k++)
            {
                    if(((*this)[k]).Attribute(tag)==value)
                    list.push_back(k);
                    }
            
            return list;
            
}


LaserPoints LaserPoints::SelectLastPulse() const
{
            LaserPoints tmp;
            tmp.Initialise();
            for(int k=0;k<size();k++)
            {
                    if(((*this)[k]).IsLastPulse())
                    tmp.push_back((*this)[k]);
                    }
            
            return tmp;
            
            }
            
LaserPoints LaserPoints::ReColorPointsPerSegment()
{
            LaserPoints        seg_laser_points, recolored_points, renumbered_laser_points;
            vector <int>       segment_numbers, colour_values,
                               colour_red, colour_green, colour_blue;
            vector <int>::iterator       segment_number;
            PointNumberLists             segments;
            PointNumberLists::iterator   segment;
            LaserPoints::iterator        laser_point;
            int                median_colour, median_colour_red, 
                               median_colour_green, median_colour_blue, index;

          // Check if the laser points have the required attributes
          if (!begin()->HasAttribute(SegmentNumberTag)) {
              printf("ReColorPointsPerSegment requires laser points with SegmentNumberTags\n");
              printf("Laser points have not been recolored\n");
                  return *this;
                 }
  
         printf("Recoloring laser points\n");
         segment_numbers = AttributeValues(SegmentNumberTag);
            printf("Size of segment numbers %d\n", segment_numbers.size());
         index = 0;
         for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++, index++) {
             printf("%4.1f\r", (100.0*index)/segment_numbers.size());    
             segments.push_back(TaggedPointNumberList(SegmentNumberTag, *segment_number));
            }  
         printf("Segments pushed back in segments %d\n", segments.size());
         index = 0;
         for (segment=segments.begin(), segment_number=segment_numbers.begin();
              segment!=segments.end(); segment++, segment_number++, index++) {
                printf("%4.1f\r", (100.0*index)/segments.size());                       
                seg_laser_points.ErasePoints();
                seg_laser_points = SelectTagValue(SegmentNumberTag, *segment_number);
                colour_values = seg_laser_points.AttributeValues(ColourTag);
                colour_red.erase(colour_red.begin(), colour_red.end());
                colour_green.erase(colour_green.begin(), colour_green.end());
                colour_blue.erase(colour_blue.begin(), colour_blue.end());

                for (laser_point = seg_laser_points.begin(); laser_point!=seg_laser_points.end(); laser_point++){
                    median_colour = laser_point->Attribute(ColourTag);
                    median_colour_red = median_colour >> 16;
                    median_colour_green = ((median_colour >> 8) - ((median_colour>>16)<<8));
                    median_colour_blue = median_colour-((median_colour>>8)<<8);
                    
                    colour_red.push_back(median_colour_red);
                    colour_green.push_back(median_colour_green);
                    colour_blue.push_back(median_colour_blue);
                    }
                sort(colour_red.begin(), colour_red.end());
                sort(colour_green.begin(), colour_green.end());
                sort(colour_blue.begin(), colour_blue.end());
                median_colour_red = colour_red[int(colour_red.size()/2)];
                median_colour_green = colour_green[int(colour_green.size()/2)];
                median_colour_blue = colour_blue[int(colour_blue.size()/2)];
                
                seg_laser_points.SetAttribute(ColourTag, (median_colour_red << 16) + (median_colour_green << 8) + median_colour_blue);
                recolored_points.AddPoints(seg_laser_points);
         }
         printf("\ndone.\n");
         return recolored_points;
         }

double LaserPoints::ReturnHeightOfPercentilePoint(int perc)
{
       double height;
       int    index;
       LaserPoints::iterator point;
       vector<double> vZ;
  
       for (point=begin(); point!=end(); point++){
            height = point->Z();
            vZ.push_back(height);
            }
            
       index = int (perc*1.0*size()/100.0);
       sort(vZ.begin(), vZ.end());
       height = vZ[index];   
       return height;
         }
         
double LaserPoints::ReturnDifferenceToGutterHeight(int segment_number, double gutterheight)
{
       double min_height, difference;
       LaserPoints        seg_laser_points;
       
       seg_laser_points = SelectTagValue(SegmentNumberTag, segment_number);
       min_height = seg_laser_points.ReturnHeightOfPercentilePoint(0);
       difference = min_height - gutterheight;
       
       return difference;
         }
         

int LargerPointNumberqsort(const void *pt1, const void *pt2)
{
  const LaserPoint *laser_point1, *laser_point2;
  int              number1, number2;
  
  laser_point1 = (const LaserPoint *) pt1;
  laser_point2 = (const LaserPoint *) pt2;
  number1 = laser_point1->Attribute(PointNumberTag);
  number2 = laser_point2->Attribute(PointNumberTag);
  if (number1 > number2) return 1;
  if (number1 < number2) return -1;
  return 0;
}


int LaserPoints::RemoveDoublePoints(bool only_check_X_and_Y)
{
  int                   number, num_removed;
  LaserPoints::iterator point, prev_point;
  
  // Add sequence numbers
  for (point=begin(), number=0; point!=end(); point++, number++)
    point->SetAttribute(PointNumberTag, number);
    
  // Sort on coordinates
  SortOnCoordinates();
  
  // Select points to be removed
  RemoveAttribute(IsSelectedTag);
  for (prev_point=begin(), point=begin()+1; point!=end(); point++, prev_point++)
    if (point->X() == prev_point->X() &&
        point->Y() == prev_point->Y() &&
        (point->Z() == prev_point->Z() || only_check_X_and_Y))
      point->Select();
      
  // Remove the selected points
  num_removed = RemoveTaggedPoints(1, IsSelectedTag);
  
  // Restore original sequence by sorting on point numbers
  qsort((void *) &*begin(), (int) size(), sizeof(LaserPoint),
        LargerPointNumberqsort);
  RemoveAttribute(PointNumberTag);
        
  // Return the number of removed points
  return num_removed;
}

void LaserPoints::ConvexHull(ObjectPoints &objpts, LineTopologies &tops, int max) const
  
  {
   Position3D p3d,a,b;
   LaserPatches lps_tmp;
   LaserPoints testpoints;
   double max_x=-65535, min_x=65535;
   double max_i=0, min_i=0;
   double tmp_min=0, tmp_max=0;
    Plane myplane;
    LineTopology Piece_out;
  ObjectPoint vertex_out;
  int obj_num=0;
  LineTopologies::const_iterator line_in, line_out;
  LineTopologies::const_iterator temp;
  int t=0;
   int i,vertical,j=0; 
    
   
   lps_tmp=LaserPatches(max+1);
   
   cout<<"The max segmenttag value is "<<max<<endl;

 //start loop for all Pieces
   for(i=0;i<=max;i++){
   max_x=-32576999;
   min_x=33576999;
   max_i=0, min_i=0;
   tmp_min=0, tmp_max=0;
   
   testpoints=SelectTagValue(SegmentNumberTag,i);
   
   if(testpoints.size()<30)
   {
    testpoints.ErasePoints(true);
    continue;
   }
     
   printf("No. %d \r",i);
   myplane=testpoints.FitPlane(i,i,SegmentNumberTag);  
   
   if(myplane.SmallestEigenvalue () ==0 )
    continue; 
   
   
   if(myplane.IsVertical(0.1))
   {
   vertical=1;
   }
   else vertical=0;

   lps_tmp[i].setPlane(myplane);

   for(j=0;j<testpoints.size();j++)  //make a segment more flat by projecting all points to plane
      {
      testpoints[j]=myplane.Project(testpoints[j]);
      }
   
   //cout<<"size: "<<testpoints.size()<<endl;
   for(j=0;j<testpoints.size();j++)   //Determine most left and right points
      {
         if((tmp_min=testpoints[j].GetX())<=min_x)
         {
               min_x=tmp_min;
               min_i=j;
         }
         if((tmp_max=testpoints[j].GetX())>=max_x)
         {
               max_x=tmp_max;
               max_i=j;
         }
       } 
   
    a= Position3D(testpoints[min_i].GetX(),testpoints[min_i].GetY(),testpoints[min_i].GetZ());
    b= Position3D(testpoints[max_i].GetX(),testpoints[max_i].GetY(),testpoints[max_i].GetZ());
   
   lps_tmp[i].push_back(a);
   lps_tmp[i].push_back(b);
   LaserPatch s1,s2;  
  
  
   for(j=0;j<testpoints.size();j++)
   {
       if((j!=min_i)&&(j!=max_i))
       {
          p3d=Position3D(testpoints[j].GetX(),testpoints[j].GetY(),testpoints[j].GetZ());
          if(((Vector2D)p3d).AreaSign(a,b)==-1)
              s1.push_back(p3d);
          else
              s2.push_back(p3d); 
       }
   }  
   FindHull(lps_tmp,s1,a,b,i,1,vertical);
   FindHull(lps_tmp,s2,b,a,i,0,vertical);  
  
  }
  
  testpoints.ErasePoints(true);
  cout<<"Find Hull complete"<<endl;
  
   objpts.clear();
 tops.clear();
 
 
  for(t=0;t<=max;t++)
  {
     if(lps_tmp[t].size()==0)
     continue;
                                                 
     Piece_out.clear();
     Piece_out.Label()=t;
  
     for(int t1=0;t1<lps_tmp[t].size();t1++)
     {
            
        vertex_out=ObjectPoint(lps_tmp[t][t1].GetX(),lps_tmp[t][t1].GetY(),lps_tmp[t][t1].GetZ(),obj_num,0,0,0,0,0,0);   
        Piece_out.push_back(PointNumber(obj_num));    
        objpts.push_back(vertex_out);
        obj_num++;
     }
    
     Piece_out.push_back(Piece_out[0]);          
    
     tops.push_back(Piece_out);     

          }
    // building_patches.clear();
    
     for(i=0;i<tops.size();i++) 
     tops[i].push_back(tops[i][0]);
     
     lps_tmp.Release();
  

}
///private, used by interations in ConvexHull()
void LaserPoints::FindHull(LaserPatches &lps_tmp,LaserPatch &sk, Position3D P, Position3D Q, int num, int flag, int vertical) const
{
    if(sk.size()==0)
        return;
    vector<Position3D>::iterator iter_tmp,iter_tmp2;
    LaserPatch s1,s2;
    Position3D C;   
    Vector2D vec,p1,q1,c1;
   
    Line2D l;
    if(vertical)  //if vertical, use XZ plane
    l=Line2D(Position2D(P.GetX(),P.GetZ()),Position2D(Q.GetX(),Q.GetZ()));
    else
    l=Line2D(Position2D(P.GetX(),P.GetY()),Position2D(Q.GetX(),Q.GetY()));
    
    double maxdistance=-32576,distance=0;
    vector<Position3D>::iterator iter, iter1;
  
   
    for(iter=sk.begin();iter!=sk.end();iter++)
    {
      if(vertical)
      distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetZ()));
      else   
      distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetY()));
      if(distance>maxdistance) 
         {
          maxdistance=distance;
          iter_tmp=iter;
          }      
                                       
    }
    
    C=*iter_tmp;
    
    for(iter_tmp2=lps_tmp[num].begin();iter_tmp2!=lps_tmp[num].end();iter_tmp2++)
    {
    if(*iter_tmp2==P)
      {iter=iter_tmp2;iter++;break;}
    }
     
    
    lps_tmp[num].insert(iter,C);
  
    for(iter=sk.begin();iter!=sk.end();iter++)
    {
    if(iter!=iter_tmp)
    {
    if(vertical)
    { 
    vec=Vector2D((*iter).GetX(),(*iter).GetZ());
    p1=Vector2D(P.GetX(),P.GetZ());
    c1=Vector2D(C.GetX(),C.GetZ());
    q1=Vector2D(Q.GetX(),Q.GetZ());   
    }
    else
    {
    vec=(Vector2D)(*iter);
    p1=(Vector2D)P;
    c1=(Vector2D)C;
    q1=(Vector2D)Q;   
    }
    if(vec.AreaSign(p1,c1)==-1)
        s1.push_back(*iter);
        
    if(vec.AreaSign(c1,q1)==-1)
    s2.push_back(*iter);
       
 
    }    
    }
    
    FindHull(lps_tmp,s1,P,C,num,flag,vertical);
    FindHull(lps_tmp,s2,C,Q,num,flag,vertical);
}


void LaserPoints::DeriveContour3D(ObjectPoints &contour_obj, LineTopology &contour_top, double max_edge_dist) const
{

     ObjectPoint objpt;
     ObjectPoints objpts;
     PointNumberList        component;
     TINEdges edges;
     LineTopology           laser_contour;
     Vector3D point, direction;
     Vector3D vec1=Vector3D(0,0,1);
     Line3D horizontal_line;
     Position3D pt;
     Plane myplane;
     double angle;
     Rotation3D *rot1; 
     LaserPoints rotated, input_laser;
     
     input_laser=(*this);
     
     pt=input_laser[0];
     
     input_laser.Label(1);
     myplane=input_laser.FitPlane(1,1,LabelTag);
    
      for(int j=0;j<input_laser.size();j++)
     {
        point=myplane.Project(input_laser[j]); 
        input_laser[j].SetX(point.X());  
        input_laser[j].SetY(point.Y());  
        input_laser[j].SetZ(point.Z());     
     }

     objpts=input_laser.ConstructObjectPoints();
      
     Plane horizontal_plane(pt,vec1);
     
    
     
     
     Intersect2Planes(horizontal_plane, myplane, horizontal_line);
     angle=Angle(myplane.Normal(), vec1);
     
     rot1=new Rotation3D(horizontal_line.Direction(), angle);    
     
     rotated=input_laser;
     
     for(int j=0;j<rotated.size();j++)
     {      
        point=(rot1->Transpose ()).Rotate(rotated[j]);
        rotated[j].SetX(point.X());
        rotated[j].SetY(point.Y());
        rotated[j].SetZ(point.Z());
        }
   
     rotated.DeriveTIN();
     rotated.DeriveDataBounds(0);
     edges.Derive(rotated.TINReference()); 
     rotated.RemoveLongEdges(edges, max_edge_dist);
  
     component.clear();
     
     for (int j=0; j<rotated.size(); j++) 
         component.push_back(PointNumber(j));
     
     contour_top = rotated.DeriveContour(1, component, edges, false);
   
     contour_obj.clear();
     for(int j=0;j<contour_top.size();j++)
      {
             objpt=objpts[contour_top[j].Number()];
             contour_obj.push_back(objpt);
      }
  
      
}

Circle2D LaserPoints::MBC()
{
    
    DeriveDataBounds(false); 
    
    Position3D center=bounds.MidPoint();
    
    LaserPoint lpt;
    
    double max_distance=0;
    
    double distance=0;
    
    for(int i=0;i<size();i++)
    {
       distance=center.Distance((*this)[i]);
       max_distance=distance>max_distance?distance:max_distance;
    }
    
    return Circle2D(center.Position2DOnly(), max_distance);     
         
}
int LaserPoints::RemoveAlmostDoublePoints(bool only_check_X_and_Y, double dist)
{
  int                   number, num_removed;
  LaserPoints::iterator point, prev_point;
  
  // Add sequence numbers
  for (point=begin(), number=0; point!=end(); point++, number++)
    point->SetAttribute(PointNumberTag, number);
    
  // Sort on coordinates
  SortOnCoordinates();
  
  // Select points to be removed
  RemoveAttribute(IsSelectedTag);
  for (prev_point=begin(); prev_point!=end(); prev_point++){
    if (!prev_point->IsSelected()){
      for (point=begin(); point!=end(); point++){
       if (prev_point!=point){
          if (fabs(point->X()-prev_point->X())<dist &&
                  fabs(point->Y()-prev_point->Y())<dist &&
                  (fabs(point->Z()-prev_point->Z())<dist || only_check_X_and_Y))
                  point->Select();
      }
      }
      }
  }
  // Remove the selected points
  num_removed = RemoveTaggedPoints(1, IsSelectedTag);
  
  // Restore original sequence by sorting on point numbers
  qsort((void *) &*begin(), (int) size(), sizeof(LaserPoint),
        LargerPointNumberqsort);
  RemoveAttribute(PointNumberTag);
        
  // Return the number of removed points
  return num_removed;
}

void LaserPoints::DeriveTopologicalRelations(ObjectPoints &obj, LineTopologies &top, double max_dist) const
{

     ObjectPoint objpt, objpt2;
     ObjectPoints objpts;
     TINEdges *edges;
     LaserPoints input_laser, seg_laser_points, sel_laser_points, longedgepoints;
     vector <int>                 seg_nums;
     vector <int>::iterator       seg_id, seg_id2;
     PointNumberList              pnlseg1,pnlseg2;
     Position3D                   pos1, pos2;
     int                          totedges2, totedges3, line_number3=0, count,
                                  count2, i, j, knn=20;
     double                       dist;
     input_laser=(*this);
     LineTopology        pairline;
     TINEdges::iterator          neighbours;
     LaserPoints                 points_on_edge;
     LaserPoints::const_iterator point, nb_point;
     TINEdgeSet::iterator        nb_node;
     vector<int>            indices;
     TINEdgeSet             edgeset;
     printf("\nstart deriving topological relations between segments...\n");
     seg_nums = input_laser.AttributeValues(SegmentNumberTag);
          
     for (seg_id = seg_nums.begin(), count=0; seg_id!=seg_nums.end(); seg_id++, count++){
         printf("%7d  %5.1f \r", count, 100.0 * count / seg_nums.size());
          pnlseg1 = input_laser.SelectTagValueList(SegmentNumberTag, *seg_id);
          objpt = input_laser.Centroid(pnlseg1, *seg_id);
          pos1 = Position3D(objpt.X(), objpt.Y(), objpt.Z());
          obj.push_back(objpt);
          sel_laser_points.ErasePoints();
          sel_laser_points = input_laser.SelectTagValue(SegmentNumberTag, *seg_id);

         for (seg_id2 = seg_id+1, count2=0; seg_id2!=seg_nums.end(); seg_id2++, count2++){
             pnlseg2 = input_laser.SelectTagValueList(SegmentNumberTag, *seg_id2);
             objpt2 = input_laser.Centroid(pnlseg2, *seg_id2);
             pos2 = Position3D(objpt2.X(), objpt2.Y(), objpt2.Z());
             dist = pos1.Distance(pos2);
              seg_laser_points = sel_laser_points;
             if (dist<50){
              seg_laser_points.AddTaggedPoints(input_laser, *seg_id2, SegmentNumberTag);
                edges = new TINEdges();
                KNNFinder <LaserPoint> finder(seg_laser_points, 3);
                for (i=0; i<(int) seg_laser_points.size(); i++) {
                    indices = finder.FindIndices(seg_laser_points[i], knn+1);
                    edgeset.Erase();
                    for (int j=0; j<knn+1; j++)
                    if (indices[j] != i) edgeset.push_back(PointNumber(indices[j]));
                       edges->push_back(edgeset);
                 }
                 finder.FreeResources();
                
              seg_laser_points.RemoveLongEdges(edges->TINEdgesRef(), max_dist, true); //remove long edges in 2D
              totedges2 = seg_laser_points.CountMixedEdges(edges->TINEdgesRef(), SegmentNumberTag); // and see if there are any edges between 2 segments left
              if (totedges2>10){
                  line_number3++;
                  pairline = LineTopology(line_number3, 1, *seg_id, *seg_id2);
                  seg_laser_points.RemoveLongEdges(edges->TINEdgesRef(), max_dist, false); //now in 3D
                  totedges3 = seg_laser_points.CountMixedEdges(edges->TINEdgesRef(), LabelTag);

                  if (totedges3 < 1){
                            pairline.Label() = 1;
                  }
                  else {
                            pairline.Label() = 2;                       
                       }
                  top.push_back(pairline);
              }
              }
         }
     }                
     
}

/*
--------------------------------------------------------------------------------
                 Simplify mesh based on one distance threshold
                 If point and all direct neighbours are in within threshold to
                 a plane, set point filtered. Neighbours cannot be filtered in
                 this same iteration. Repeat untill unchanged.
--------------------------------------------------------------------------------
*/

LaserPoints LaserPoints::SimplifyMesh(double max_dist)
{

  TINEdges           *edges;
  TINEdges::iterator nbh;
    TINEdges::iterator        start_edgeset;
  LaserPoints::iterator     point, nb_point, goodpoint, start_point;
  LaserPoints               simplifiedpoints;
  int                       first_number, i, iter = 0, startsize;
  PointNumberList node_list;
  Plane           plane;
  bool changed = false;
  startsize = size();
  if (size()<10){
    printf("not enough points to derive TIN, no simplification\n");
    for (point=begin(); point!=end(); point++) {
        simplifiedpoints.push_back(*point);
    }
     erase(begin(), end());
    return(simplifiedpoints);
  }
 // printf("Start simplifying laser data, threshold %4.2f\n", max_dist);
  do {
  SetAttribute(IsProcessedTag, 0); // Set all points unprocessed
  SetUnFiltered(); // Accept all points
  
  edges = new TINEdges();
  DeriveTIN();
  edges->Derive(TINReference());
  changed = false;
  for (start_point=begin(), start_edgeset=edges->begin(), first_number=0; 
         start_point!=end(); start_point++, start_edgeset++, first_number++) {
      if (!start_point->Processed() && start_edgeset->size()) {
        node_list.erase(node_list.begin(), node_list.end());
        bool processed_neighbours = false;
        for (TINEdgeSet::iterator node=start_edgeset->begin(); node!=start_edgeset->end(); node++){
          node_list.push_back(*node);
          point = begin() + node->Number();
          if (point->Processed()) processed_neighbours = true;
        }
        node_list.push_back(PointNumber(first_number));
        if (!processed_neighbours && node_list.size()>3 && node_list.size()<size()){
 //         printf("iter = %2d, size of node list %2d, total size %6d\r", iter, node_list.size(), size());
          plane = FitPlane(node_list, 0);
          if (PointsInPlane(node_list, plane, max_dist, 100)){
            start_point->SetFiltered();
            start_point->SetProcessed();
            changed = true;
            for (TINEdgeSet::iterator node=start_edgeset->begin(); node!=start_edgeset->end(); node++){
              nb_point = begin() + node->Number();
              nb_point->SetProcessed();
            }
          }
        }
      }
  }
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    if (!point->Filtered()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
  iter++;
  edges->Erase();
  } while (changed);
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    simplifiedpoints.push_back(*point);
  }
 edges->Erase();
// printf("Finished simplifying laser data, percentage of original %4.2f\n", 100.0*size()/(1.0*startsize));
 erase(begin(), end());
 return(simplifiedpoints);
 
}  
/*
--------------------------------------------------------------------------------
                 Simplify mesh based on one distance threshold
                 If point and all direct neighbours are in within threshold to
                 a plane, set point filtered. Neighbours cannot be filtered in
                 this same iteration. Repeat untill unchanged.
--------------------------------------------------------------------------------
*/

LaserPoints LaserPoints::SimplifyMesh_KeepLabel(double max_dist, const LaserPointTag tag, int keepvalue)
{

  TINEdges           *edges;
  TINEdges::iterator nbh;
    TINEdges::iterator        start_edgeset;
  LaserPoints::iterator     point, nb_point, goodpoint, start_point;
  LaserPoints               simplifiedpoints;
  int                       first_number, i, iter = 0, startsize;
  PointNumberList node_list;
  Plane           plane;
  bool changed = false;
  startsize = size();
  if (size()<10){
    printf("not enough points to derive TIN, no simplification\n");
    for (point=begin(); point!=end(); point++) {
        simplifiedpoints.push_back(*point);
    }
    erase(begin(), end());
    return(simplifiedpoints);
  }
//  printf("Start simplifying laser data, threshold %4.2f\n", max_dist);
  do {
  SetAttribute(IsProcessedTag, 0); // Set all points unprocessed
  SetUnFiltered(); // Accept all points
  
  edges = new TINEdges();
  DeriveTIN();
  edges->Derive(TINReference());
  changed = false;
  for (start_point=begin(), start_edgeset=edges->begin(), first_number=0; 
         start_point!=end(); start_point++, start_edgeset++, first_number++) {
      if (!start_point->Processed() && start_edgeset->size() &&
         start_point->Attribute(tag)!= keepvalue) {
        node_list.erase(node_list.begin(), node_list.end());
        bool processed_neighbours = false;
        for (TINEdgeSet::iterator node=start_edgeset->begin(); node!=start_edgeset->end(); node++){
          node_list.push_back(*node);
          point = begin() + node->Number();
          if (point->Processed()) processed_neighbours = true;
        }
        node_list.push_back(PointNumber(first_number));
        if (!processed_neighbours && node_list.size()>3 && node_list.size()<size()){
   //       printf("iter = %2d, size of node list %2d, total size %6d\r", iter, node_list.size(), size());
          plane = FitPlane(node_list, 0);
          if (PointsInPlane(node_list, plane, max_dist, 100)){
            start_point->SetFiltered();
            start_point->SetProcessed();
            changed = true;
            for (TINEdgeSet::iterator node=start_edgeset->begin(); node!=start_edgeset->end(); node++){
              nb_point = begin() + node->Number();
              nb_point->SetProcessed();
            }
          }
        }
      }
  }
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    if (!point->Filtered()) {
      *goodpoint = *point;
      goodpoint++;
    }
  }
  erase(goodpoint, end());
  iter++;
  edges->Erase();
  } while (changed);
  for (point=begin(), goodpoint=begin();
       point!=end(); point++) {
    simplifiedpoints.push_back(*point);
  }
 edges->Erase();
// printf("Finished simplifying laser data, percentage of original %4.2f\n", 100.0*size()/(1.0*startsize));
 erase(begin(), end());
 return(simplifiedpoints);
 
}  


/*
LaserPoints LaserPoints::ExtractLongEdges(double max_dist,
                                  bool planimetric)
{
  TINEdges::iterator          neighbours;
  LaserPoints::const_iterator point, nb_point;

  TINEdgeSet::iterator        nb_node;
  double                      dist;
  int count=1;
  int i=0;
  int label[size()];
  
  LaserPoints out;
  LaserPoints copy=LaserPoints(*this);
  
  copy.Label(0);
  TINEdges edges=TINEdges(copy.TINReference());
  
  
  for (point=copy.begin(), neighbours=edges.begin(); point!=copy.end();
       point++, neighbours++) {
    for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) {
      nb_point = copy.begin() + nb_node->Number();
      if (planimetric) dist = (*point - *nb_point).Length2D();
      else dist = (*point - *nb_point).Length();
      if (dist > max_dist) {
        {
         if(point->Label()==0&&nb_point->Label()==0)
         {point->Label(count);   nb_point->Label(count);   count++;}
        else if(point->Label()==0)
        {point->Label(nb_point->Label());}
        else if(nb_point->Label()==0)
        {nb_point->Label(point->Label());}
        
        //if(label)
        
        out.push_back(*nb_point);
        }
        //nb_node--;
      }
    }
  }
  
  return out;
  
}
*/

