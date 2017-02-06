
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



/* DONE (#1#): Add declarations of prop_num_images etc., see viff.c */
// Declarations of C image I/O routines for usage in C++ programmes

#include "viff.h"
			                
extern "C" xvimage *createimage(unsigned long, unsigned long, unsigned long,
                                unsigned long, unsigned long, const char *,
                                unsigned long, unsigned long, unsigned long,
                                unsigned long, unsigned long, unsigned long);
extern "C" xvimage *readimageheader(const char *);
extern "C" xvimage *readimage(const char *);
extern "C" xvimage *readimagepart(const char *, int, int, int, int);
extern "C" int writeimage(const char *, xvimage *);

extern "C" int propertype(const char *, xvimage *, unsigned long, int);
extern "C" int proper_num_images(const char *, xvimage *, unsigned long, int);
extern "C" int proper_num_bands(const char *, xvimage *, unsigned long, int);
extern "C" int proper_map_enable(const char *, xvimage *, unsigned long, int);
extern "C" int proper_map_type(const char *, xvimage *, unsigned long, int);
extern "C" int proper_map_scheme(const char *, xvimage *, unsigned long, int);
extern "C" int proper_color_model(const char *, xvimage *, unsigned long, int);

extern "C" int freeimage(xvimage *);
extern "C" void copyheader(xvimage *, xvimage *);
extern "C" int imagesize(xvimage *, int *, int *, int *, int *, int *, int *);

