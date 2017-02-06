
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
 Conversion of ASCII laser data to the binary laser data format

 Initial creation:
 Author : George Vosselman
 Date   : 29-09-2000

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
#include "LaserUnit.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                      The main ascii2laser function
--------------------------------------------------------------------------------
*/

void ascii2laser_cpp(char *ascii_file, char *ascii_file_filter,
                     int remove_double_points,
                     int column_x, int column_y, int column_z,
                     int column_r, int column_g, int column_b,
                     int column_p, int column_n, int column_l, int column_pl,
                     int column_pn, int column_sn, int column_poln,
                     int column_scan, int column_t, int column_a,
                     int column_int, int column_fs, int header_lines,
                     double x_offset, double y_offset, double z_offset,
                     int p_offset,
                     double x_scale, double y_scale, double z_scale,
                     int set_r, int fixed_r, int set_p, int fixed_p,
                     int set_l, int fixed_l, int set_scan, int fixed_scan,
                     int set_fs, int fixed_fs,
                     char *rootname, char *output_directory,
                     int meta_type, int max_subunit_size,
					 int rgb_scale, int int_scale)
{
  LaserUnit              strip;
  LaserSubUnit           pointset, last_reflections;
  LaserPoint             point, previous_point;
  FILE                   *ascii;
  char                   line[2048], *name, *comma, *directory, *filter, *filename;
  double                 value[21];
  int                    i, double_points, total_points, icon;
  LaserSubUnit::iterator last_first_pulse;

  // Set up the file filter for the input file(s)

  if (ascii_file_filter) filter = ascii_file_filter;
  else filter = ascii_file;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Process all input files
  total_points = double_points = 0;
  printf("processed / double points\n");
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {

    // Open the input file
    ascii = Open_Compressed_File(filename, "r");
    if (!ascii) {
      fprintf(stderr, "Error opening input file %s\n", ascii_file);
      exit(0);
    }

    // Skip the header records
    for (i=0; i<header_lines; i++) fgets(line, 2048, ascii);
    if (feof(ascii)) {
      fprintf(stderr, "Error: end of file reached after reading header lines.\n");
      exit(0);
    }

    // Set the point type
    if (column_b) {
      if (column_p) pointset.Scanner().SetPointType(MultiColourPoint);
      else pointset.Scanner().SetPointType(ColourPoint);
    }
    else if (column_r) {
      if (column_p) pointset.Scanner().SetPointType(MultiReflectancePoint);
      else pointset.Scanner().SetPointType(ReflectancePoint);
    }
    else {
      if (column_p) pointset.Scanner().SetPointType(MultiPoint);
      else pointset.Scanner().SetPointType(NormalPoint);
    }
  
    // Process all records
    do {
      if (fgets(line, 1024, ascii)) {

        // Remove the comma's
        while ((comma = strchr(line, ',')) != NULL) *comma = ' ';

        // Read the next line
        sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               value+ 1, value+ 2, value+ 3, value+ 4, value+ 5,
               value+ 6, value+ 7, value+ 8, value+ 9, value+10,
               value+11, value+12, value+13, value+14, value+15,
               value+16, value+17, value+18, value+19, value+20);

        // Copy the data to the laser point
        point.X() = value[column_x] * x_scale + x_offset;
        point.Y() = value[column_y] * y_scale + y_offset;
        point.Z() = value[column_z] * z_scale + z_offset;

        if (set_r)           // Fixed specified reflectance value
          point.Reflectance() = fixed_r;
        else if (column_g) {  // Colour point
          point.SetColour(((int) value[column_r]) / rgb_scale,
		                  ((int) value[column_g]) / rgb_scale,
                          ((int) value[column_b]) / rgb_scale);
        }
        else if (column_r)   // Reflectance point
          point.Reflectance() = ((int) value[column_r]) / int_scale;

        if (set_p)           // Fixed specified pulse count value
          point.PulseCountWithFlag() = fixed_p;
        else if (column_p) {   // Pulse count value
          point.PulseCountWithFlag() = (int) value[column_p] + p_offset;
          if (column_n)
            if ((int) value[column_n] <= (int) value[column_p])
              point.SetLastPulseFlag(true);
        }

        if (set_l)           // Fixed specified label value
          point.Label(fixed_l);
        else if (column_l)   // Label value
          point.Label((int) value[column_l]);
        
        if (set_scan) // Fixed specified scan number (strip number)
          point.SetScanNumber(fixed_scan);
        else if (column_scan) // Scan number
          point.SetScanNumber((int) value[column_scan]);

        if (set_fs) // Fixed specified filter status
          point.SetAttribute(IsFilteredTag, fixed_fs);
        else if (column_fs) // Filter status
          point.SetAttribute(IsFilteredTag,(int) value[column_fs]);

        if (column_int)  point.Reflectance()   = ((int) value[column_int]) / int_scale;
        if (column_pl)   point.PulseLength()   = (int) value[column_pl];
        if (column_pn)   point.PlaneNumber()   = (int) value[column_pn];
        if (column_sn)   point.SegmentNumber() = (int) value[column_sn];
        if (column_poln) point.PolygonNumber() = (int) value[column_poln];
        if (column_t)    point.SetDoubleAttribute(TimeTag, value[column_t]);
        if (column_a)    point.SetAttribute(AngleTag, (float) value[column_a]);
        
        // Put the laser point into the data set
        if (point != previous_point || !remove_double_points) {
          pointset.push_back(point);
          previous_point = point;
        }
        else double_points++;
        
        // Counter
        if ((pointset.size() / 10000) * 10000 == pointset.size()) {
          printf(" %d / %d\r", pointset.size(), double_points);
          fflush(stdout);
        }
      }

      // Output of subunit if maximum size is reached
      if (pointset.size() == max_subunit_size) {

        // Set the data organisation type and the file names
        if ((pointset.size() == max_subunit_size) || strip.size()) {
          if (meta_type == 1) pointset.DataOrganisation() = StripWise;
          else pointset.DataOrganisation() = UnknownOrganisation;
          name = (char *) malloc(strlen(rootname) + 5);
          sprintf(name, "%s_%3d", rootname, strip.size()+1);
          for (i=0; i<strlen(name); i++) if (name[i] == ' ') name[i] = '0';
          pointset.SetName(name);
          free(name);
          pointset.DeriveMetaDataFileName(output_directory);
        }
        else {
          pointset.DataOrganisation() = UnknownOrganisation;
          pointset.SetName(rootname);
          pointset.LaserPoints::DeriveMetaDataFileName(output_directory);
        }
        pointset.DerivePointFileName(output_directory);

        if (pointset.size() == max_subunit_size || strip.size())
          pointset.SetAsStripPart();
        else
          pointset.SetAsCompleteStrip();

        // Save first reflections of the last emitted pulse for the next point set
        if (column_p && pointset.size() == max_subunit_size) {
          // Search the last first pulse point
          last_first_pulse = pointset.end() - 1;
          while (last_first_pulse->PulseCount() != 1 &&
                 last_first_pulse != pointset.begin()) last_first_pulse--;
          if (last_first_pulse != pointset.begin()) {
            last_reflections.insert(last_reflections.begin(),
                                    last_first_pulse, pointset.end());
            pointset.erase(last_first_pulse, pointset.end());
          }
        }
      
        if (column_p && !column_n) pointset.SetLastPulseFlags();
            
        pointset.Write();
        printf("Written %s with %d points.\n", pointset.PointFile(),
               pointset.size());
             
        total_points += pointset.size();
        pointset.ErasePoints();
        strip.push_back(pointset);
      
        // Copy saved pulses back to the pointset
        if (!last_reflections.empty()) {
          pointset.insert(pointset.begin(), last_reflections.begin(),
                          last_reflections.end());
          last_reflections.ErasePoints();
        } 
      }
    } while (!feof(ascii));
    Close_Compressed_File(ascii);
  }
  
  // Save the remaining points
  if (pointset.size()) {
    // Set the data organisation type and the file names
    if (strip.size()) {
      if (meta_type == 1) pointset.DataOrganisation() = StripWise;
      else pointset.DataOrganisation() = UnknownOrganisation;
      name = (char *) malloc(strlen(rootname) + 5);
      sprintf(name, "%s_%3d", rootname, strip.size()+1);
      for (i=0; i<strlen(name); i++) if (name[i] == ' ') name[i] = '0';
      pointset.SetName(name);
      free(name);
      pointset.DeriveMetaDataFileName(output_directory);
      pointset.SetAsStripPart();
    }
    else {
      pointset.DataOrganisation() = UnknownOrganisation;
      pointset.SetName(rootname);
      pointset.LaserPoints::DeriveMetaDataFileName(output_directory);
      pointset.SetAsCompleteStrip();
    }
    pointset.DerivePointFileName(output_directory);

    if (column_p && !column_n) pointset.SetLastPulseFlags();
            
    pointset.Write();
    printf("Written %s with %d points.\n", pointset.PointFile(),
           pointset.size());

    total_points += pointset.size();
    pointset.ErasePoints();
    strip.push_back(pointset);
  }

  printf("Total of %d records processed.\n", total_points + double_points);
  if (double_points)
    printf("%d duplicate records encountered.\n", double_points);
  
// Write the meta data

  if (meta_type) {
    if (meta_type == 1 || strip.size() > 1) {   // Strip
      strip.DataOrganisation() = StripWise;
      strip.Scanner().SetPointType(pointset.Scanner().PointType());
      if (strip.size() == 1) strip.SetPointFile(pointset.PointFile());
      strip.SetName(rootname);
      strip.DeriveMetaDataFileName(output_directory);
      strip.WriteMetaData(1);
    }
    else {   // Arbitrary point set
      pointset.WriteMetaData();
    }
  }
}
