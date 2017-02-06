
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
 * \brief Interface to Class Histogram - Histogram of some data
 *
 */
/*!
 * \class Histogram
 * \ingroup PStatistic
 * \brief Interface to Class Histogram - Histogram of some data
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li One-dimensional histogram stored in an image of type VFF_TYP_4_BYTE (int)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Histogram_h_
#define _Histogram_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 Histogram                          - Histogram of some data

--------------------------------------------------------------------------------
*/

#include "Image.h"

//------------------------------------------------------------------------------
/// Histogram
/** One-dimensional histogram stored in an image of type VFF_TYP_4_BYTE (int) */
//------------------------------------------------------------------------------

class Histogram : public Image
{
  public:

    /// Default constructor
    Histogram() : Image() {};

    /// Construct an empty histogram
    /** All histogram values are set to 0. Location data is added to the image.
        This allows to visualise the histogram image with 

        % putdata -axis2d -i "histogram_name"

        @param num_bins Number of bins of the histogram
        @param minimum  Minimum data value (corresponding to the left edge of
                        bin 0
        @param maximum  Maximum data value (corresponding to thte right edge of
                        bin num_bins-1
    */
    Histogram(int num_bins, double minimum, double maximum);

    /// Return the readable reference
    const Histogram &HistogramReference() const
      {return(*this);}

    /// Return the writable reference
    Histogram &HistogramReference()
      {return(*this);}

    /// Set all frequencies to zero
    void Clear()
      {ClearImage();}

    /// Add a value to the histogram
    /** The bin of this data value is computed and the bin value is incremented
        by one. The first or the last bin values is incremented if the data
        value is below or above the histogram range.
    */
    void Add(double data_value);

    /// Return the frequencies in an int vector
    int *Frequencies() const
      {return((int *) image->imagedata);}
};
#endif /* _Histogram_h_ */  /* Don't add after this point */
