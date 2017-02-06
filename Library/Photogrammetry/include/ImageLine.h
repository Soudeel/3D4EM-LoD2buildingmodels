
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
 * \brief Interface to Class ImageLine
 *
 */
/*!
 * \class ImageLine
 * \ingroup Photogrammetry
 * \brief Interface to Class ImageLine
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 * \date		18-07-2002 (Update #1)
 *
 * \remark \li Update #1 - Added a label to the class
 *
 * \remark \li Image line -
 * An image line is defined by a vector of image points, a line number and
 * a line label.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _ImageLine_h_
#define _ImageLine_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImageLine                            

  Update #1
  Author  : George Vosselman
  Date    : 18-07-2002
  Changes : Added a label to the class


--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <vector>
#include "ImagePoints.h"
#include "LineNumber.h"
#include "Database4Cpp.h"

class LineTopology;
class ImagePoints;


//------------------------------------------------------------------------------
/// Image line
/** An image line is defined by a vector of image points, a line number and
    a line label.
*/
//------------------------------------------------------------------------------

class ImageLine : public std::vector<ImagePoint>, public LineNumber
{

  protected:

    /// Label of the image line
    int label;

  public:

    /// Default constructor
    ImageLine() : std::vector<ImagePoint>(), LineNumber() {}

    /// Construct by converting a C object
    ImageLine(ImgLine *line) : std::vector<ImagePoint>(), LineNumber() 
      { C2Cpp(line); }

    /// Copy constructor
    ImageLine(const ImageLine &lin) : std::vector<ImagePoint>(), LineNumber() 
      { insert(begin(), lin.begin(), lin.end()); 
        num = lin.num; 
        label = lin.label;}

    /// Destructor
    ~ImageLine() {};

    /// Copy assignment
    ImageLine & operator = (const ImageLine &);

    /// Return the readable line label
    int Label() const
      {return label;}

    /// Return the writable line label
    int &Label() 
      {return label;}

    /// Conversion from C to C++ object
    void C2Cpp(ImgLine *);

    /// Conversion from C++ to C object
    void Cpp2C(ImgLine **) const;
    
    /// Convert to image points and line topology
    /** Both arguments should point to already allocated variables */
    void Convert2LineTop(ImagePoints *imgpts, LineTopology *top) const; 	
};

#endif /* _ImageLine_h_ */   /* Do NOT add anything after this line */
