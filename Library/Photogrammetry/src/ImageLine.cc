
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
 Collection of functions for class ImageLine

 ImageLine& ImageLine::operator=(const ImageLine&)   Copy assignment
 void ImageLine::Cpp2C(ImgLine **)      Conversion of C++ class to C structure
 void ImageLine::C2Cpp(ImgLine *)       Conversion of C structure to C++ class

 Initial creation
 Author : Ildiko Suveg
 Date   : 24-11-1998

 Update #1
 Author : George Vosselman
 Date   : 18-07-2002
 Changes: Added support for image line labels

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include "ImageLine.h"
#include "LineTopology.h"
#include "ImagePoints.h"

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

ImageLine& ImageLine::operator=(const ImageLine& line)
{
   // Check for self assignment
   if (this == &line) return *this;
   if (!empty()) erase(begin(), end());
   num = line.num;
   label = line.label;
   insert(begin(), line.begin(), line.end());
   return *this;			
}

/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void ImageLine::Cpp2C(ImgLine **lineptr) const
{
  ImgLine *line;

// Allocate space if this has not been done yet 

  line = *lineptr;
  if (line == NULL) {
    line = (ImgLine *) malloc(sizeof(ImgLine));
    line->pts = (ImgPt*) malloc(size() * sizeof(ImgPt));
    *lineptr = line;
  }

// Copy the data from the C++ to the C object 

  line->num = num;
  line->num_pts = size();
  line->label = label;
  ImageLine::const_iterator i;
  int l = 0;
  ImgPt *pt; 
  for (i = begin(); i != end(); i++, l++)
  {
  	pt = &line->pts[l]; 
  	i->Cpp2C(&pt); 	
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void ImageLine::C2Cpp(ImgLine *line)
{
  if (!empty())
  	erase(begin(), end());
  reserve(line->num_pts);
  num = line->num;  
  label = line->label;

  for(int i = 0; i < line->num_pts; i++) 	
  {
	ImagePoint point(&line->pts[i]);  	
  	push_back(point);
  }
}


/*
--------------------------------------------------------------------------------
                        Convert to line topology
--------------------------------------------------------------------------------
*/

void ImageLine::Convert2LineTop(ImagePoints *imgpts, LineTopology *line) const 	
{
  ImageLine::const_iterator i;
  for(i = begin(); i != end(); i++)
  {
     imgpts->push_back(*i);
     line->push_back(*i);	
  } 
} 
