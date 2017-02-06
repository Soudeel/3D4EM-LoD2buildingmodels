
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
 Collection of functions for class ImageLines          

 void ImageLines::Cpp2C(ImgLines **    Conversion of C++ to C object
 void ImageLines::C2Cpp(ImgLines *)    Conversion of C to C++ object
 int  ImageLines::Read(char *)         Read lines from a database
 int  ImageLines::Write(char *)        Write lines points to a database
 void ImageLines::Print()              Print lines points to stdout

 Initial creation
 Author : Ildi Suveg
 Date   : 01-12-1998

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

#include <string.h>
#include "ImageLines.h"
#include "LineTopologies.h"
#include "ImagePoints.h"


/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ImageLines::Cpp2C(ImgLines **linesptr) const
{
  ImgLines *lines;

// Allocate space if this has not been done yet 

  lines = *linesptr;
  if (lines == NULL) {
    lines = (ImgLines *) malloc(sizeof(ImgLines));
    lines->lines = (ImgLine*) malloc(size() * sizeof(ImgLine)); 
    *linesptr = lines;
  }

  strcpy(lines->img_name, name);
 
// Copy the data 

  lines->num_lines = size();
  ImageLines::const_iterator i;
  ImgLine *ln; 
  int l = 0;
  for(i = begin(); i != end(); i++, l++)
  {
	lines->lines[l].pts = (ImgPt*) malloc(i->size() * sizeof(ImgPt));
	if (lines->lines[l].pts == NULL)
		printf("points allocation error\n");
	
	ln = &lines->lines[l];
	i->Cpp2C(&ln);	              
  } 
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ImageLines::C2Cpp(ImgLines *lines)
{    
  if (!empty()) erase(begin(), end());
  reserve(lines->num_lines);
  //delete [] name;	
 
  
  name = new char[strlen(lines->img_name) + 1];
  strcpy(name, lines->img_name);
 
  int l = 0;
  for(int i = 0 ; i < lines->num_lines; i++, l++)
  {	
        ImageLine line(&lines->lines[l]);
  	push_back(line);
  } 
}

/*
--------------------------------------------------------------------------------
                       Read lines from a database
--------------------------------------------------------------------------------
*/
int ImageLines::Read(char *filename)
{
  ImgLines *lines;

  lines = Get_ImgLines(filename);    // Read the database into C structure
  if (lines == NULL) return(0);
//  name = NULL;
  C2Cpp(lines);                      // Convert to C++ object 
  Free_ImgLines(lines);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write lines to a database
--------------------------------------------------------------------------------
*/
int ImageLines::Write(char *filename) const
{
  int    error;
  ImgLines *lines;

  lines = NULL;
  Cpp2C(&lines);
  error = Put_ImgLines(lines, filename);
  Free_ImgLines(lines); 
  return(error);
}

/*
--------------------------------------------------------------------------------
                        Print lines to stdout
--------------------------------------------------------------------------------
*/
void ImageLines::Print() const
{
  ImgLines *lines;

  lines = NULL;
  Cpp2C(&lines);
  Print_ImgLines(lines);
  Free_ImgLines(lines); 
}

/*
--------------------------------------------------------------------------------
                        Convert to line topologies
--------------------------------------------------------------------------------
*/

void ImageLines::Convert2LineTops(ImagePoints *imgpts, LineTopologies *lines) const 	
{
  ImageLines::const_iterator i;
  LineTopology line; 
  for(i = begin(); i != end(); i++)
  {
     line.Number() = i->Number();
     i->Convert2LineTop(imgpts, &line);
     lines->push_back(line);
     line.erase(line.begin(), line.end());	
  }
}
 
