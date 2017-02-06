
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



#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: ascii2laser -i <input text file> OR\n");
  printf("                   -f <file filter for multiple input text files>\n");
  printf("                   -root <root name of output files>\n");
  
  printf("\n         General optional parameters\n");
  printf("                   -odir <output directory name, def: .\\>\n");
  printf("                   -m <meta data type 1=strip, 2=point set, def: 2>\n");
  printf("                   -max <maximum number of points per output file, def: 500000>\n");
  printf("                   -h <number of header records, def: 0>\n");

  printf("\n         Parameters for specifying columns of attributes\n");
  printf("                   -x <column number of X-coordinate, def: 1>\n");
  printf("                   -y <column number of Y-coordinate, def: 2>\n");
  printf("                   -z <column number of Z-coordinate, def: 3>\n");
  printf("                   -r <column number of reflectance or red value>\n");
  printf("                   -int <column number of intensity (reflectance) value>\n");
  printf("                   -g <column number of green value>\n");
  printf("                   -b <column number of blue value>\n");
  printf("                   -p <column number of pulse count>, see below\n");
  printf("                   -n <column number of number of echoes>, see below\n");
  printf("                   -l <column number of label>\n");
  printf("                   -pl <column number of pulse length>\n");
  printf("                   -pn <column number of plane number>\n");
  printf("                   -sn <column number of segment number>\n");
  printf("                   -poln <column number of polygon number>\n");
  printf("                   -scan <column number of scan (or strip) number>\n");
  printf("                   -t <column number of time stamp\n");
  printf("                   -a <column number of scan angle\n");
  printf("                   -fs <column number of filter status\n");

  printf("\n         Parameters for setting a fixed value for an attribute\n");
  printf("                   -sr <set value for reflectance strength>\n");
  printf("                   -sp <set value for pulse count>\n");
  printf("                   -sl <set value for label>\n");
  printf("                   -sscan <set value for scan number>\n");
  printf("                   -sfs <set value for filter status (0=terrain, 1=off-terrain)>\n");
  
  printf("\n         Transformation parameters\n");
  printf("                   -xoff <offset for X-coordinate, def: 0.0>\n");
  printf("                   -yoff <offset for Y-coordinate, def: 0.0>\n");
  printf("                   -zoff <offset for Z-coordinate, def: 0.0>\n");
  printf("                   -poff <offset for pulse count, def: 0>\n");
  printf("                   -xscale <scale factor for X-coordinate, def: 1.0>\n");
  printf("                   -yscale <scale factor for Y-coordinate, def: 1.0>\n");
  printf("                   -zscale <scale factor for Z-coordinate, def: 1.0>\n");
  printf("                   -rgbscale <devide rgb values by factor, def: 1>\n");
  printf("                   -intscale <devide intensity values by factor, def: 1>\n");
  
  printf("\nNote on echo counts:\n");
  printf("When -p is used to read the number of the echo, it is assumed that echoes of\n");
  printf("the same pulse are listed in successive records with increasing echo numbers.\n");
  printf("This is not required when the total number of echoes of an emitted pulse is\n");
  printf("provided in another column (option -n). In that case the last pulse flag will\n");
  printf("be set if the value in columns -p en -n are the same.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void ascii2laser_cpp(char *, char *, int, int, int, int, int, int, int, int, int,
                       int, int, int, int, int, int, int, int, int, int, int,
                       double, double, double, int,
                       double, double, double, int, int,
                       int, int, int, int, int, int, int, int, char *, char *,
                       int, int, int, int);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f should be specified.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are mutually exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }
  
  if (args->Contains("-n") && !args->Contains("-p")) {
    printf("Error: -n is only useful in combination with -p\n");
    PrintUsage();
    exit(0);
  }
  
  if (!args->Contains("-root")) {
    printf("Error: -root is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  ascii2laser_cpp(args->String("-i"), args->String("-f"),
                  args->Contains("-rd"),
                  args->Integer("-x", 1), args->Integer("-y", 2),
                  args->Integer("-z", 3), args->Integer("-r", 0),
                  args->Integer("-g", 0), args->Integer("-b", 0),
                  args->Integer("-p", 0), args->Integer("-n", 0),
                  args->Integer("-l", 0),
                  args->Integer("-pl", 0), args->Integer("-pn", 0),
                  args->Integer("-sn", 0), args->Integer("-poln", 0),
                  args->Integer("-scan", 0),
                  args->Integer("-t", 0), args->Integer("-a", 0),
                  args->Integer("-int", 0), args->Integer("-fs", 0),
                  args->Integer("-h", 0), args->Double("-xoff", 0.0),
                  args->Double("-yoff", 0.0), args->Double("-zoff", 0.0),
                  args->Integer("-poff", 0),
                  args->Double("-xscale", 1.0), args->Double("-yscale", 1.0),
                  args->Double("-zscale", 1.0),
                  args->Contains("-sr"), args->Integer("-sr", 0),
                  args->Contains("-sp"), args->Integer("-sp", 0),
                  args->Contains("-sl"), args->Integer("-sl", 0),
                  args->Contains("-sscan"), args->Integer("-sscan", 0),
                  args->Contains("-sfs"), args->Integer("-sfs", 0),
                  args->String("-root"), args->String("-odir"),
                  args->Integer("-m", 2), args->Integer("-max", 500000),
				  args->Integer("-rgbscale", 1), args->Integer("-intscale", 1));

  return EXIT_SUCCESS;
}
