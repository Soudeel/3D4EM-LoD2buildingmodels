
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

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 24-01-2017
 
 Simple tool to read in laz file, thin by removing points within 5 cm

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
#include "LaserBlock.h"
#include <time.h>

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

extern void timer_start(clock_t *);
extern void timer_end(clock_t, char *);

/*
--------------------------------------------------------------------------------
                      The main laser2ascii function
--------------------------------------------------------------------------------
*/

void laz2xyzlaser_cpp(char *infile, char *outfile, double thindistance)
{
//  char                   *directory, *filter, *filename;
  long int                    i;
  LaserPoints              points;
  LaserPoint			outpoint;
  LaserPoints::iterator point;
  clock_t                  timer;
  
    if (!points.Read(infile, false)) {
        fprintf(stderr, "Error reading laser data from file %s\n", infile);
        exit(0);
      }
	  printf("file read in, %d points\n", points.size());
	if (points.size()>10000000) {
	  printf("point size too big, use smaller dataset (less than 10 M points)\n");
	  exit(0);
		
	}

	points.RemoveAttributes();
	if (thindistance>0.0) {
			   timer_start(&timer);
   
				printf("start thinning, this can take a while (about 2 minutes per million points)\n");
				int presize = points.size();
				points.ReduceData(thindistance, 10);
				printf("reduced from %d to %d.\n", presize, points.size());		
   				timer_end(timer, (char *) "thinning");
   
		}
	
	points.Write(outfile, false);
	return;
}
