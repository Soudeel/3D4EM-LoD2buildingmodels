
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
 Collection of functions for class LaserSubUnit

 Initial creation
 Author : George Vosselman
 Date   : 24-03-1999

 Update #1
 Author : George Vosselman
 Date   : 18-11-1999
 Changes: Added support for tiled blocks and tiled strips

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
#include "LaserSubUnit.h"
#include "ObjectPoints2D.h"
#include "LineTopologies.h"
#include "Line2D.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "digphot_arch.h"

/*
--------------------------------------------------------------------------------
                         Construct empty tile
--------------------------------------------------------------------------------
*/

LaserSubUnit::LaserSubUnit(LaserDataOrganisation org, int row, int column,
                           DataBoundsLaser &bounds, double border)
{
  Initialise();
  data_org = org;
  tile_row = row;
  tile_column = column;
  tile_border = border;
  tile_bounds = bounds;
}

/*
--------------------------------------------------------------------------------
                        Copy constructor and assignment
--------------------------------------------------------------------------------
*/

LaserSubUnit::LaserSubUnit(const LaserSubUnit &subunit)
{
  Initialise();
  LaserPointsReference() = subunit.LaserPointsReference();
  data_org       = subunit.data_org;
  whole_strip    = subunit.whole_strip;
  first_scanline = subunit.first_scanline;
  last_scanline  = subunit.last_scanline;
  tile_row       = subunit.tile_row;
  tile_column    = subunit.tile_column;
  north = south = west = east = NULL; /* Copying makes no sense */
  tile_bounds    = subunit.tile_bounds;
  tile_border    = subunit.tile_border;
}

LaserSubUnit & LaserSubUnit::operator = (const LaserSubUnit &subunit)
{
  if (this == &subunit) return *this;  // Check for self assignment
  LaserPointsReference() = subunit.LaserPointsReference();
  data_org       = subunit.data_org;
  whole_strip    = subunit.whole_strip;
  first_scanline = subunit.first_scanline;
  last_scanline  = subunit.last_scanline;
  tile_row       = subunit.tile_row;
  tile_column    = subunit.tile_column;
  north = south = west = east = NULL; /* Copying makes no sense */
  tile_bounds    = subunit.tile_bounds;
  tile_border    = subunit.tile_border;
  return(*this);
}
/*
--------------------------------------------------------------------------------
                         Initialise the meta data
--------------------------------------------------------------------------------
*/

void LaserSubUnit::Initialise()
{
  LaserPoints::Initialise();
  data_org = UnknownOrganisation;
  meta_file = NULL;
  first_scanline = last_scanline = -1;
  whole_strip = 0;
  tile_row = tile_column = -1;
  north = east = west = south = NULL;
  tile_bounds.Initialise();
  tile_border = -1;
}

void LaserSubUnit::ReInitialise()
{
  LaserPoints::ReInitialise();
  data_org = UnknownOrganisation;
  if (meta_file) free(meta_file);  meta_file = NULL;
  first_scanline = last_scanline = -1;
  whole_strip = 0;
  tile_row = tile_column = -1;
  north = east = west = south = NULL;
  tile_bounds.Initialise();
  tile_border = -1;
}

/*
--------------------------------------------------------------------------------
                       Read the strip part meta data
--------------------------------------------------------------------------------
*/
int LaserSubUnit::ReadMetaData(const char *filename)
{
  FILE *fd;
  char *buffer, *keyword, *line;
  int  keyword_length, success;
  int debug = false;

/* Open the file */

//  fd = Open_Compressed_File(filename, "r");
  if (debug) printf("ReadMetaData open %s\n", filename);
  fd = fopen(filename, "r");
  if (debug) printf("Opened %s %d\n", filename, (int) fd);
  if (fd == NULL) {
    fprintf(stderr, "Error opening strip part or tile database %s\n", filename);
    return(0);
  }

/* Verify that the first keyword is "laserstrippart", "laserstriptile" or
 * "lasertile".
 */

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading strip part or tile database %s\n", filename);
    fclose(fd);
    free(buffer);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "laserstrippart", MAX(keyword_length, 14)) &&
      strncmp(keyword, "laserstriptile", MAX(keyword_length, 14)) &&
      strncmp(keyword, "lasertile", MAX(keyword_length, 9))) {
    fprintf(stderr, "Error: File %s is not a strip part or tile database.\n",
            filename);
    fprintf(stderr, "       First keyword is %s, not laserstrippart, laserstriptile or lasertile.\n",
	    keyword);
  }

  // Set the data organisation variable
  if (!strncmp(keyword, "laserstrippart", MAX(keyword_length, 14)))
    data_org = StripWise;
  else if (!strncmp(keyword, "laserstriptile", MAX(keyword_length, 14)))
    data_org = StripTileWise;
  else // lasertile
    data_org = TileWise;

  success = ReadMetaData(fd, data_org);
  SetMetaDataFile(filename);
  fclose(fd);
  free(buffer);
  return success;
}


int LaserSubUnit::ReadMetaData(FILE *fd, LaserDataOrganisation org)
{
  char *line, *keyword, *buffer;
  int  keyword_length;
  
  // Initialise all meta data
  ReInitialise();

  // Set data organisation
  data_org = org;
  
  // Process all lines
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "metadata", MAX(keyword_length, 8)))
          SetMetaDataFile(BNF_String(line));

        else if (!strncmp(keyword, "points", MAX(keyword_length, 6)))
          point_file = BNF_String(line);

        else if (!strncmp(keyword, "seek_offset", MAX(keyword_length, 11)))
          seek_offset = BNF_LongInteger(line);

        else if (!strncmp(keyword, "highest_surface_number", MAX(keyword_length, 22)))
          highest_surface_number = BNF_Integer(line);

        else if (!strncmp(keyword, "tin", MAX(keyword_length, 3)))
          tin_file = BNF_String(line);

        else if (!strncmp(keyword, "first_scanline", MAX(keyword_length, 14)) &&
                 data_org == StripWise)
          first_scanline = BNF_Integer(line);

        else if (!strncmp(keyword, "last_scanline", MAX(keyword_length, 13)) &&
                 data_org == StripWise)
          last_scanline = BNF_Integer(line);

        else if (!strncmp(keyword, "info", MAX(keyword_length,4)))
          LaserPointsInfo::ReadMetaData(fd);

        else if (!strncmp(keyword, "tilebounds", MAX(keyword_length, 10)) &&
                 data_org & TileWise)
          tile_bounds.DataBoundsLaser::Read(fd, "endtilebounds");

        else if (!strncmp(keyword, "tileborder", MAX(keyword_length, 10)) &&
                 data_org & TileWise)
          tile_border = BNF_Double(line);

        else if (!strncmp(keyword, "tilelocation", MAX(keyword_length, 9)) &&
                 data_org & TileWise)
          BNF_Two_Integers(line, &tile_row, &tile_column);

        else if (!strncmp(keyword,"endlaserstrippart",MAX(keyword_length,17)) ||
	         !strncmp(keyword,"endlaserstriptile",MAX(keyword_length,17)) ||
	         !strncmp(keyword,"endlasertile",MAX(keyword_length,12))) {
          free(buffer);
          return(1);
        }
        
        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  switch (data_org) {
    case StripWise:
      fprintf(stderr, "Error: Did not find keyword endlaserstrippart.\n");
      break;
    case TileWise:
      fprintf(stderr, "Error: Did not find keyword endlasertile.\n");
      break;
    case StripTileWise:
      fprintf(stderr, "Error: Did not find keyword endlaserstriptile.\n");
      break;
  }
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the strip part meta data to file
--------------------------------------------------------------------------------
*/

int LaserSubUnit::WriteMetaData() const
{
  return(WriteMetaData(meta_file));
}

int LaserSubUnit::WriteMetaData(const char *filename) const
{
  FILE       *fd;
  int        success;

  // Don't write meta data for a strip part equivalent to a complete strip
  if (data_org == StripWise && whole_strip) return(1);

  // Open the file
  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening strip part database %s\n", filename);
    return(0);
  }

  // Write the meta data
  success =  WriteMetaData(fd);
  
  // Close file and return success code
  fclose(fd);
  return success;
}

int LaserSubUnit::WriteMetaData(FILE *fd) const
{
  int        indent;

  indent = 0;
  switch (data_org) {
    case StripWise:
      BNF_Write_String(fd, "laserstrippart", indent, NULL);  indent += 2;
      break;
    case StripTileWise:
      BNF_Write_String(fd, "laserstriptile", indent, NULL);  indent += 2;
      break;
    case TileWise:
      BNF_Write_String(fd, "lasertile", indent, NULL);  indent += 2;
      break;
    default: /* UnknownOrganisation */
      BNF_Write_String(fd, "laserpointset", indent, NULL);  indent += 2;
  }
  if (name) BNF_Write_String(fd, "name", indent, name);
  if (meta_file) BNF_Write_String(fd, "metadata", indent, meta_file);
  if (point_file) BNF_Write_String(fd, "points", indent, point_file);
  if (seek_offset != -1)
    BNF_Write_LongInteger(fd, "seek_offset", indent, seek_offset, "%I64d");
  if (highest_surface_number != -1)
    BNF_Write_Integer(fd, "highest_surface_number", indent,
	                  highest_surface_number, "%d");
  if (tin_file) BNF_Write_String(fd, "tin", indent, tin_file);
  if (data_org == StripWise && first_scanline != -1)
    BNF_Write_Integer(fd, "first_scanline", indent, first_scanline, "%d");
  if (data_org == StripWise && last_scanline != -1)
    BNF_Write_Integer(fd, "last_scanline", indent, last_scanline, "%d");
  if (data_org & TileWise) {
    BNF_Write_Two_Integers(fd, "tilelocation", indent, tile_row, tile_column,
                           "%d %d");
    tile_bounds.Write(fd, indent, "tilebounds");
    BNF_Write_Double(fd, "tileborder", indent, tile_border, "%8.3f");
  }
  LaserPointsInfo::WriteMetaData(fd, indent);
  indent -= 2;
  switch (data_org) {
    case StripWise:
      BNF_Write_String(fd, "endlaserstrippart", indent, NULL);
      break;
    case StripTileWise:
      BNF_Write_String(fd, "endlaserstriptile", indent, NULL);
      break;
    case TileWise:
      BNF_Write_String(fd, "endlasertile", indent, NULL);
      break;
    default: /* UnknownOrganisation */
      BNF_Write_String(fd, "endlaserpointset", indent, NULL);
  }

  return(1);
}

/*
--------------------------------------------------------------------------------
                          Derive name of the sub unit
--------------------------------------------------------------------------------
*/

char *LaserSubUnit::DeriveName(const char *parentname, LaserSubUnit *first_part,
                               bool six_digits)
{
  int  part_number, i;
  char *ch;

  if (name) free(name);
  switch (data_org) {
    case StripWise:
      name = (char *) malloc(strlen(parentname) + 5);
      part_number = ((int) this - (int) first_part) / sizeof(LaserSubUnit);
      sprintf(name, "%s_%3d", parentname, part_number);
      break;

    case TileWise:
    case StripTileWise:
      if (six_digits) {
        name = (char *) malloc(strlen(parentname) + 15);
        sprintf(name, "%s_%6d_%6d", parentname, tile_row, tile_column);
      }
      else {
        name = (char *) malloc(strlen(parentname) + 9);
        sprintf(name, "%s_%3d_%3d", parentname, tile_row, tile_column);
      }
      break;

    default:
      return(NULL);
  }

  for (i=strlen(parentname), ch=name+i; i<strlen(name); i++, ch++)
   if (*ch == ' ') *ch = '0';
  return(name);
}

/*
--------------------------------------------------------------------------------
                          Derive meta data file name
--------------------------------------------------------------------------------
*/

char *LaserSubUnit::DeriveMetaDataFileName(const char *directory)
{

/* Extension of meta data file name depends on data organisation type */

  switch (data_org) {
    case StripWise:
      return(LaserDataFiles::DeriveMetaDataFileName(directory, ".part"));
    case StripTileWise:
      return(LaserDataFiles::DeriveMetaDataFileName(directory, ".striptile"));
    case TileWise:
      return(LaserDataFiles::DeriveMetaDataFileName(directory, ".tile"));
    default:
      return(LaserDataFiles::DeriveMetaDataFileName(directory, ".pointset"));
  }
}

/*
--------------------------------------------------------------------------------
                  Set name of sub unit, point file and meta data file
--------------------------------------------------------------------------------
*/

void LaserSubUnit::SetFileNames(const char *parentname, const char *directory,
                                LaserSubUnit *first_part, bool six_digits)
{
  DeriveName(parentname, first_part, six_digits);
  DeriveMetaDataFileName(directory);
  DerivePointFileName(directory);
}

/*
--------------------------------------------------------------------------------
                          Derive the scan lines
--------------------------------------------------------------------------------
*/

void LaserSubUnit::DeriveScanLines(LaserScanLines &lines, double median_dist,
                                   double max_terrain_height,
                                   double approximate_flight_height,
                                   int num_scanlines_flight_path,
                                   Positions3D &flight_path,
                                   Position3D &flight_path_point,
                                   int &num_flight_path_summed)
{
  int        first_num_in_part, begin_num, end_num, next_num, num_interval_turn,
             max_line_size;
  bool       found, erase_points=false;
  LaserPoints::const_iterator begin_point, next_point, end_point;
  double     line_length, dist, angle, median_angle, begin_dir, end_dir, u,
             next_dir, line_angle, previous_dir;
  LaserScanLines::const_iterator line, long_line;
  Position3D start_pos, end_pos;
  Line2D     flight_line;

/* For the analysis high points like reflections on birds should not be used,
 * since their XY coordinates may be very different from other points and
 * cause the "detection" of a scan line end. Therefore, only points below a
 * certain height (max_terrain_height) are used.
 */

/* An end point is detected by checking the distances of points to the first
 * point of a scan line. First, this distance will increase. After the end
 * of the scan line this distance will become smaller. If the distance between
 * the current point and the point most distance from the first point is larger
 * than a multiple of the median distance between consecutive points, the
 * latter point is considered to be the end of the scan line. 
 */

  num_interval_turn = 50;

  // Read strip part points if required
  if (empty()) {
    if (!Read(point_file, false)) {
      printf("Error reading points from file %s\n", point_file);
      return;
    }
    erase_points = true;
  }
  
  // Determine the number of first point in this strip part
  if (lines.empty()) first_num_in_part = 0;
  else first_num_in_part = (lines.end()-1)->End().Number()+1;

  // Initialise the first scan line
  begin_point = end_point = NextPointOfPulseType(LastPulse, begin());
  begin_num = end_num = 0;
  line_length = -1.0;
  next_point = begin_point+1;
  while (next_point->Z() >= max_terrain_height)
    next_point = NextPointOfPulseType(LastPulse, next_point+1);

  // Start analysing
  for (; next_point!=end();
       next_point = NextPointOfPulseType(LastPulse, next_point+1)) {

    // Only use this point if it could be a terrain point
    if (next_point->Z() > max_terrain_height) continue;

    /* If the current point is most distant from the first point, store it as the
     * potential end point of the scan line.
     */
    dist = (*next_point - *begin_point).Length2D();
    if (dist > line_length) {
      line_length = dist;
      end_point = next_point;
      end_num = ((int) &*end_point - (int) &*begin()) / sizeof(LaserPoint);
    }

    /* Otherwise, if the distance to the potential end point is larger than a
     * threshold, this point is stored as an end point.
     */
    else {
      if ((*next_point - *end_point).Length2D() >
          num_interval_turn * median_dist) {
        if ((*next_point - *(next_point-1)).Length2D() > 
            num_interval_turn * median_dist)
          end_num = ((int) &*next_point - (int) &*begin()) / sizeof(LaserPoint) - 1;
        lines.push_back(LaserScanLine(begin_num + first_num_in_part,
                                      end_num + first_num_in_part));
        // Store mid point for flight path reconstruction, but skip the first scan
        // line of each strip part
        if (num_scanlines_flight_path && begin_num) {
          end_point = begin() + end_num;
          flight_path_point.X() += (end_point->X() + begin_point->X()) / 2.0;
          flight_path_point.Y() += (end_point->Y() + begin_point->Y()) / 2.0;
          num_flight_path_summed++;
          // Check if a new point is to be stored
          if (num_flight_path_summed == num_scanlines_flight_path) {
            flight_path_point.X() = flight_path_point.X() / num_scanlines_flight_path;
            flight_path_point.Y() = flight_path_point.Y() / num_scanlines_flight_path;
            flight_path_point.Z() = approximate_flight_height;
            flight_path.push_back(flight_path_point);
            // Re-initialise sum
            flight_path_point.X() = 0.0;
            flight_path_point.Y() = 0.0;
            num_flight_path_summed = 0;
          }
        }
        // Initialise the new scan line
        begin_point = end_point = begin() + end_num + 1;
        begin_num = end_num = end_num + 1;
        line_length = -1.0;
      }
    }
  }

/* Terminate the last scan line */

  lines.push_back(LaserScanLine(begin_num + first_num_in_part,
                                size() - 1 + first_num_in_part));
                                
  // Erase points
  if (erase_points) ErasePoints();

// If the approximate flight height is specified, a second analysis is done,
// now using the direction vectors from the reconstructed flight line towards
// the laser points. The flight path should be a straight line for this
// algorithm to work correctly. This can be improved by extracting a multi point
// flight path description from the initially reconstructed scan lines. This is
// not yet implemented.

/* NOW DISABLED

  if (approximate_flight_height == 0.0) {
    if (erase_points) ErasePoints();
    return; 
  }

  // First reconstruct the 2D flight line from the mid points of two
  // relatively long (i.e. complete) scan lines.
  max_line_size=0;
  for (line=lines.begin(); line!=lines.end(); line++) 
    if (line->NumberOfPoints() > max_line_size)
      max_line_size = line->NumberOfPoints();
  for (line=lines.begin(), found=false; line!=lines.end() && !found; line++) {
    if (line->NumberOfPoints() > 0.95 * max_line_size) {
      long_line = line;  found = true;
    }
  }
  long_line->ScannerPosition(LaserPointsReference(), approximate_flight_height,
                             start_pos);
  for (line=lines.end()-1, found=false; line!=lines.begin() && !found; line--) {
    if (line->NumberOfPoints() > 0.95 * max_line_size) {
      long_line = line;  found = true;
    }
  }
  long_line->ScannerPosition(LaserPointsReference(), approximate_flight_height,
                             end_pos);
  flight_line = Line2D(Position2D(start_pos.vect2D()),
                       Position2D(end_pos.vect2D()));
  median_angle = long_line->ScanAngleIncrement(LaserPointsReference(),
                                               end_pos, 50.0);

  // Clear the old scan lines and redo the same analysis with angles instead
  // of distances
  lines.erase(lines.begin(), lines.end());

  // Initialise the first scan line
  begin_point = end_point = begin();
  begin_num = end_num = 0;
  u = flight_line.DistanceToPointSigned(Position2D(begin_point->vect2D()));
  begin_dir = end_dir = atan(u /(approximate_flight_height - begin_point->Z()));
  line_angle = -1.0;
  next_point = begin_point+1;
  next_num = 1;
  next_dir = begin_dir;

  // Start analysing
  for (; next_point!=end(); next_point++, next_num++) {

    if (next_point->Z() > max_terrain_height) continue;

    previous_dir = next_dir;
    u = flight_line.DistanceToPointSigned(Position2D(next_point->vect2D()));
    next_dir = atan(u / (approximate_flight_height - next_point->Z()));

    // If the current direction vector has the largest angle with the first 
    // direction vector, store this point as the potential end point of the scan
    // line.
    angle = fabs(next_dir - begin_dir);
    if (angle > line_angle) {
      line_angle = angle;
      end_point  = next_point;
      end_num    = next_num;
      end_dir    = next_dir;
    }

    // Otherwise, if the angle with the potential end point is larger than a
    // threshold, this point is stored as an end point.
    else {
      if (fabs(next_dir - end_dir) > num_interval_turn * median_angle) {
        if (fabs(next_dir - previous_dir) > num_interval_turn * median_angle) {
          end_num = next_num - 1;
          end_dir = previous_dir;
        }
        lines.push_back(LaserScanLine(begin_num + first_num_in_part,
                                      end_num + first_num_in_part));

        // Initialise the new scan line
        begin_point = end_point = begin() + end_num + 1;
        begin_num = end_num = end_num + 1;
        u = flight_line.DistanceToPointSigned(Position2D(end_point->vect2D()));
        begin_dir = atan(u / (approximate_flight_height - begin_point->Z()));
        end_dir = begin_dir;
        line_angle = -1.0;
      }
    }
  }

  // Terminate the last scan line
  lines.push_back(LaserScanLine(begin_num + first_num_in_part,
                                size() - 1 + first_num_in_part));
                                
  // Erase points if they have been read in this function
  if (erase_points) ErasePoints();
  
*/
}

/*
--------------------------------------------------------------------------------
                          Thinning of scan lines
--------------------------------------------------------------------------------
*/

void LaserSubUnit::ReduceData(const LaserScanLines &scanlines,
                             int first_num_in_part,
                             int point_reduction_factor, int random_point,
                             int scanline_reduction_factor, int random_scanline)
{

  LaserScanLines::const_iterator scanline;
  LaserPoints::iterator    store_position;
  LaserPoints::const_iterator    selected_point;
  int                      random_scanline_offset, random_point_offset,
                           last_num_in_part;
  long                     random_scanline_number, random_point_number;

/* Locate the first scan line of this part */

  for (scanline=scanlines.begin();
       scanline->Begin().Number() != first_num_in_part &&
         scanline != scanlines.end();
       scanline++);
  if (scanline == scanlines.end()) {
    fprintf(stderr, "Error locating scan line with start number %d\n",
            first_num_in_part);
    exit(0);
  }

/* Random selection of the first scan line */

  store_position = begin();
  last_num_in_part = first_num_in_part + size() - 1;
  if (random_scanline) {
#ifdef hpux
    random_scanline_number = random();
#else
    random_scanline_number = rand();
#endif
    random_scanline_offset = random_scanline_number -
                             (random_scanline_number/scanline_reduction_factor)*
                             scanline_reduction_factor;
    scanline += random_scanline_offset;
  }

/* Loop over the scan lines */

  while (scanline < scanlines.end() &&
         scanline->End().Number() <= last_num_in_part) {

/* Selection of the first point in the scan line */

    selected_point = scanline->begin(this, first_num_in_part);
    if (random_point) {
#ifdef hpux
      random_point_number = random();
#else
      random_point_number = rand();
#endif
      random_point_offset = random_point_number -
                            (random_point_number / point_reduction_factor) *
                            point_reduction_factor;
      selected_point += random_point_offset;
    }

/* Loop over the points of the scan line */

    while (selected_point < scanline->end(this, first_num_in_part)) {
      *store_position = *selected_point;
      store_position++;
      if (random_point) {
        selected_point += point_reduction_factor - random_point_offset;
#ifdef hpux
        random_point_number = random();
#else
        random_point_number = rand();
#endif
        random_point_offset = random_point_number -
                              (random_point_number / point_reduction_factor) *
                              point_reduction_factor;
        selected_point += random_point_offset;
      }
      else selected_point += point_reduction_factor;
    }
        
/* Select the next scan line */

    if (random_scanline) {
      scanline += scanline_reduction_factor - random_scanline_offset;
#ifdef hpux
      random_scanline_number = random();
#else
      random_scanline_number = rand();
#endif
      random_scanline_offset = random_scanline_number -
                             (random_scanline_number/scanline_reduction_factor)*
                               scanline_reduction_factor;
      scanline += random_scanline_offset;
    }
    else scanline += scanline_reduction_factor;
  }

/* Erase the points behind the last stored point */

  if (!empty()) erase(store_position, end());
}

/*
--------------------------------------------------------------------------------
                          Insert points into tile
--------------------------------------------------------------------------------
*/

void LaserSubUnit::InsertIntoTile(LaserPoints &pts,
                                  bool preserve_multiple_reflections)
{
  DataBoundsLaser       outer_bounds;
  FILE                  *fd;
  LaserPoints::iterator point;
  bool                  first_pulse_inside;

  // Determine the bounds of the tile with the border
  outer_bounds.SetMinimumX(tile_bounds.Minimum().X() - tile_border);
  outer_bounds.SetMaximumX(tile_bounds.Maximum().X() + tile_border);
  outer_bounds.SetMinimumY(tile_bounds.Minimum().Y() - tile_border);
  outer_bounds.SetMaximumY(tile_bounds.Maximum().Y() + tile_border);

  // Determine the XY bounds of the point set if they are still unknown
  if (!pts.Bounds().XYBoundsSet()) pts.DeriveDataBounds(0);

  // Return if the point set has no overlap with the tile
  if (!OverlapXY(pts.Bounds(), outer_bounds)) return;

  // If the tile already has a point file, read this file and check the point
  // type.
  if ((fd = Open_Compressed_File(point_file, "r")) != NULL) {
    Close_Compressed_File(fd);
    Read(PointFile(), false);
    if (scanner.PointType() != pts.Scanner().PointType()) {
      fprintf(stderr,"Warning: points in tile with different point types.\n");
      fprintf(stderr,"         Point type will be set to NormalPoint (XYZ).\n");
      scanner.SetPointType(NormalPoint);
    }
  }
  else scanner.SetPointType(pts.Scanner().PointType());

  // Choose the selection strategy
  if (!preserve_multiple_reflections ||
      !pts.begin()->HasAttribute(PulseCountTag)) {
    // Add the points to the tile if they are within the outer bounds
    for (point=pts.begin(); point!=pts.end(); point++)
      if (outer_bounds.InsideXY(&*point)) push_back(*point);
  }
  else {
    // Keep multiple reflections together and select on the first pulse
    first_pulse_inside = false;
    for (point=pts.begin(); point!=pts.end(); point++) {
      if (point->IsPulseType(FirstPulse))
        first_pulse_inside = outer_bounds.InsideXY(&*point);
      if (first_pulse_inside) push_back(*point);
    }
  }

  // Store the points if there are any and deallocate the memory
  if (size()) {
    Write(false);
    ErasePoints();
  }
}

/*
--------------------------------------------------------------------------------
                          Add tile boundaries to lists
--------------------------------------------------------------------------------
*/

void LaserSubUnit::AddTileBoundary(ObjectPoints2D &corners,
                                   LineTopologies &topology) const
{
  ObjectPoint2D corner=ObjectPoint2D(0,0,0,0,0,0);
  LineTopology tile_topology;

  tile_topology.Number() = tile_row * 1000 + tile_column;

  corner.Number() = tile_row * 1000 + tile_column;
  corner.SetXY(tile_bounds.Minimum().X(), tile_bounds.Maximum().Y());
  if (corners.FindPoint(corner.NumberRef()) == -1) corners.push_back(corner);
  tile_topology.push_back(corner.NumberRef());

  corner.Number() = tile_row * 1000 + tile_column + 1;
  corner.SetXY(tile_bounds.Maximum().X(), tile_bounds.Maximum().Y());
  if (corners.FindPoint(corner.NumberRef()) == -1) corners.push_back(corner);
  tile_topology.push_back(corner.NumberRef());

  corner.Number() = (tile_row+1) * 1000 + tile_column + 1;
  corner.SetXY(tile_bounds.Maximum().X(), tile_bounds.Minimum().Y());
  if (corners.FindPoint(corner.NumberRef()) == -1) corners.push_back(corner);
  tile_topology.push_back(corner.NumberRef());

  corner.Number() = (tile_row+1) * 1000 + tile_column;
  corner.SetXY(tile_bounds.Minimum().X(), tile_bounds.Minimum().Y());
  if (corners.FindPoint(corner.NumberRef()) == -1) corners.push_back(corner);
  tile_topology.push_back(corner.NumberRef());
  tile_topology.push_back(tile_topology.begin()->NumberRef());

  if (topology.FindLine(tile_topology.NumberRef()) == -1)
    topology.push_back(tile_topology);
}

/*
--------------------------------------------------------------------------------
                     Select points within bounds and tile bounds
--------------------------------------------------------------------------------
*/

void LaserSubUnit::Select(LaserPoints &selection,
                          const DataBoundsLaser &sel_bounds) const
{
  DataBoundsLaser new_bounds = sel_bounds;

// Make sure not to select points in the tile border

  if (data_org == StripTileWise || data_org == TileWise) {
    if (!sel_bounds.MinimumXIsSet() ||
        sel_bounds.Minimum().X() < tile_bounds.Minimum().X())
      new_bounds.SetMinimumX(tile_bounds.Minimum().X());
    if (!sel_bounds.MaximumXIsSet() ||
        sel_bounds.Maximum().X() > tile_bounds.Maximum().X())
      new_bounds.SetMaximumX(tile_bounds.Maximum().X());
    if (!sel_bounds.MinimumYIsSet() ||
        sel_bounds.Minimum().Y() < tile_bounds.Minimum().Y())
      new_bounds.SetMinimumY(tile_bounds.Minimum().Y());
    if (!sel_bounds.MaximumYIsSet() ||
        sel_bounds.Maximum().Y() > tile_bounds.Maximum().Y())
      new_bounds.SetMaximumY(tile_bounds.Maximum().Y());
  }

  LaserPoints::Select(selection, new_bounds);
}

/*
--------------------------------------------------------------------------------
               Select points within polygons and tile bounds
--------------------------------------------------------------------------------
*/

int LaserSubUnit::Select(LaserPoints &selection, const ObjectPoints &corners,
                         const LineTopologies &polygons)
{
  LineTopologies::const_iterator    polygon;
  LaserPoints::const_iterator point;
  int                         old_size = selection.size();
  DataBoundsLaser             polygon_bounds;
  bool                        delete_laser_data;

  if (data_org == StripTileWise || data_org == TileWise) {
    for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
      polygon_bounds = DataBoundsLaser(polygon->Bounds(corners));
      // Only check points if the tile bounds of the laser data and
      // polygon overlap
      if (OverlapXY(tile_bounds, polygon_bounds)) {
        if (empty()) {
          Read();
          delete_laser_data = true;
        }
        else delete_laser_data = false;
        for (point=begin(); point!=end(); point++) {
          // Check if the point is within the tile bounds
          if (tile_bounds.InsideXY(&*point))
            // Check if the point is within the polygon
            if (point->InsidePolygon(corners,
                                     polygon->PointNumberListReference()))
              selection.push_back(*point); // Add the point to the selection
        }
        if (delete_laser_data) ErasePoints();
      }
    }
    return(selection.size() - old_size);
  }
  else return LaserPoints::Select(selection, corners, polygons);
}

/*
--------------------------------------------------------------------------------
                      Compare tile locations
--------------------------------------------------------------------------------
*/

int LaserSubUnit::CompareTileLocation(int row, int column) const
{
  if (tile_row < row) return -1;
  if (tile_row > row) return 1;
  if (tile_column < column) return -1;
  if (tile_column > column) return 1;
  return 0;
}

/*
--------------------------------------------------------------------------------
                      Determine the number of points
--------------------------------------------------------------------------------
*/

long long int LaserSubUnit::NumberOfPoints() const
{
  LaserSubUnit  subunit;
  FILE          *fd; 
  int           num_pts, ret_val;
  unsigned char max_num_attributes;
  double        x_offset, y_offset, z_offset;
  
  // If points are in memory, just return the number
  if (size()) return (long long int) size();
  
  // Otherwise read header information of point file
  if (PointFile()) {
    fd = fopen(PointFile(), "rb");
    if (fd == NULL) {
      printf("Error opening point file %s\n", PointFile());
      return 0;
    }
    ret_val = subunit.ReadHeader(fd, &num_pts, &x_offset, &y_offset, &z_offset,
                            &max_num_attributes);
	if (ret_val == -1) {
      printf("Error reading header from point file %s\n", PointFile());
      fclose(fd);
      return 0;
    }
    
    // deal with LAS files
    if (ret_val == LASER_LAS) {
      printf("Support for LAS files has not yet been implemented in LaserSubUnit::NumberOfPoints()\n");
      fclose(fd);
      return 0;
    }
    fclose(fd);
    return (long long int) num_pts;
  }
  
  printf("Error: no point file for sub unit %s\n", name);
  return 0;
}
