
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
 * \brief Interface to Class LaserScanLines - Information on the scanlines
 *
 */
/*!
 * \class LaserScanLines
 * \ingroup LDataOrg
 * \brief Interface to Class LaserPoint - Single laser altimetry point
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li Information on the scanline beginnings
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _LaserScanLines_h_
#define _LaserScanLines_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserScanLines                       - Information on the scanlines

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserScanLine.h"

//------------------------------------------------------------------------------
///                  Information on the scanline beginnings
//------------------------------------------------------------------------------

class LaserScanLines : public std::vector<LaserScanLine>
{
  protected:

    /// Name of the file with the scan line information
    char *scanlines_file;

  public:

    /// Default constructor
    LaserScanLines()
      {Initialise();}

    /// Default destructor
    ~LaserScanLines() {};

    /// Return the reference
    LaserScanLines & LaserScanLinesReference()
      {return(*this);}

    /// Return the const reference
    const LaserScanLines & LaserScanLinesReference() const
      {return(*this);}

    /// Copy assignment
    LaserScanLines operator = (const LaserScanLines &scanlines);

    /// Initialisation of new instant
    void Initialise();

    /// Reinitialisation of old instant
    void ReInitialise();

    /// Read the scan lines from a file
    int Read(const char *filename);

    /// Read the scan lines from "scanline_file"
    int Read();

    /// Write the scan lines to a file
    /** @param filename File for the scanline information. This file name is
                        also stored in the class variable scanline_file.
        @param compress If true, the file will be compressed with gzip
        @return 1 for success, 0 for failure
    */
    int Write(const char *filename, int compress);

    /// Write the scan lines to "scanline_file"
    /** @param compress If true, the file will be compressed with gzip
        @return 1 for success, 0 for failure
    */
    int Write(int compress) const;

    /// Write the scan lines to "scanline_file" and compress it
    /** @return 1 for success, 0 for failure
    */
    int Write() const;

    /// Print scan lines
    void Print() const;

    /// Set file name of scan lines
    void SetScanLinesFile(const char *filename);

    /// Return file name of scan lines
    char *ScanLinesFile() const
      {return(scanlines_file);}
      
    /// Find a similar scan line
    /** Simple scan line matching (used for indoor mapping) based on angle,
        distance and overlap
        &param points The point set
        &param line The scan line to be matched
        &param max_angle Maximum angle in degrees between the scan lines
        &param max_dist Maximum distance in meters between the scan lines
        &param min_overlap Minimum overlap in meters between the scan lines
        &param max_time_diff Maximum time difference between scan lines
        &param reference Reference position for checking distance between the
                         lines. This point is projected onto one line. The
                         distance of this projection to the other line is used
                         for the distance check.
        &return iterator to the matching scan line. In case of no match
                this->end() will be returned.
    */
    LaserScanLines::iterator MatchingScanLine(const LaserPoints &points,
	                                          const LaserScanLine &line,
											  double max_angle,
											  double max_dist,
											  double min_overlap,
											  double max_time_diff,
											  const Position3D &reference);
};

#endif /* _LaserScanLines_h_ */  /* Don't add after this point */
