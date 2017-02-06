
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



/*!
 * \file
 * \brief Interface to Class ImageLines
 *
 */
/*!
 * \class ImageLines
 * \ingroup Photogrammetry
 * \brief Interface to Class ImageLines
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _ImageLines_h_
#define _ImageLines_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImageLines  

--------------------------------------------------------------------------------
*/

#include "ImageLine.h"

class LineTopologies;
class ImagePoints;

//------------------------------------------------------------------------------
/// A vector of image lines
//------------------------------------------------------------------------------

class ImageLines : public std::vector<ImageLine>
{
    /// Name of the data set
    char *name;

  public:

    /// Default constructor
    ImageLines() : std::vector<ImageLine>()
      { name = NULL; } 

    /// Construct by reading image lines from a file
    /** @param filename File with the image lines
        @param success  Success status of the read function (0 - failure,
                        1 - success)
    */
    ImageLines(char *filename, int *success) : std::vector<ImageLine>()
      { *success = Read(filename); }

    /// Construct by converting a C object
    ImageLines(ImgLines *lines) : std::vector<ImageLine>()
      { C2Cpp(lines); }    

    /// Default destructor
    ~ImageLines()
	{ delete [] name; }

    /// Conversion from C to C++ object
    void C2Cpp(ImgLines *);

    /// Conversion from C++ to C object
    void Cpp2C(ImgLines **) const;

    /// Read image lines from a file
    /** @param filename File with the image lines
        @return         Success status, 0 - failure, 1 - success.
    */
    int Read(char *filename);

    /// Write image lines to a file
    /** @param filename File for the image lines
        @return         Success status, 0 - failure, 1 - success.
    */
    int Write(char *filename) const;

    /// Print the image lines to stdout
    void Print() const;

    /// Return the name of the data set
    const char *Name() const
    	{ return name; }	
    	
    /// Convert the image lines to image points with line topologies
    void Convert2LineTops(ImagePoints *imgpts, LineTopologies *tops) const; 	
};

#endif /* _ImageLines_h_ */   /* Do NOT add anything after this line */
