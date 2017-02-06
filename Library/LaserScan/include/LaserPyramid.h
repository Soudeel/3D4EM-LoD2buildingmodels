
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
 * \brief Class LaserPyramid - hierarchical laser block
 *
 */
/*!
 * \class LaserPyramid
 * \ingroup LDataOrg
 * \brief Class LaserPyramid - hierarchical laser block
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date	  16-04-2007 (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _LaserPyramid_h_
#define _LaserPyramid_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 LaserPyramid                          - A laser altimetry pyramid

 Initial creation
 Author : George Vosselman
 Date   : 16-04-2007

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserBlock.h"

//------------------------------------------------------------------------------
///                       Laser altimetry pyramid
//------------------------------------------------------------------------------

class LaserPyramid: public std::vector<LaserBlock>, public LaserDataFiles
{
  protected:
    /// Point spacing on level 0
    double original_point_spacing;

    /// Data reduction factor
    int reduction_factor;
    
    /// Tiles used for point selection
    LaserTilePtrVector used_tiles;
    
    /// Used level for the tile selection
    int used_level;
    
    /// End of tile indices in point selection
    std::vector <int> end_of_tile;
    
    /// Point file descriptor
    FILE *point_fd;

  public:

    /// Default constructor
    LaserPyramid()
      {Initialise();}

    /// Construct pyramid by reading pyramid meta data
    LaserPyramid(const char *filename, int *success)
      {Initialise(); *success = ReadMetaData(filename);}

    /// Copy assignment
    LaserPyramid & operator = (const LaserPyramid &);

    /// Initialisation of new pyramid
    void Initialise();

    /// Reinitialisation of existing pyramid
    void ReInitialise();

    /// Read meta data from a file
    int ReadMetaData(const char *filename);
  
    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to a file
    int WriteMetaData(const char *filename) const;

    /// Write meta data of pyramid to "meta_file" and of (sub)units
    int WriteMetaData(int write_blocks, int write_units,
		      int write_subunits) const;

    /// Write meta data of pyramid to a file and of (sub)units
    int WriteMetaData(const char *filename, int write_blocks, int write_units,
                      int write_subunits) const;

    /// Derive the name of the meta data file
    char *DeriveMetaDataFileName(const char *directory);

    /// Create a pyramid of a tiled block
    /** @param block Tile block with the original laser points
        @param red_method Reduction method (1, 2 or 3):
               1 - Minimum 2D distance between points, based on original point spacing
               2 - Minimum 3D distance between points, based on original point spacing
               3 - Every n'th point
               4 - Random point from every n points
               5 - knn based selection of every n'th point
        @param red_factor Linear reduction factor (>1)
        @param point_spacing Point spacing of original data
        @param overwrite If true, existing tiles are overwritten
        @return 1 if successful, 0 otherwise
    */
    int Create(const LaserBlock &block, int red_method, int red_factor,
               double point_spacing, bool overwrite);
    
    /// Select tiles in the specified area with the specified point spacing
    LaserTilePtrVector & SelectTiles(const DataBounds2D &bounds,
                                     double point_spacing);

    /// Update a laser point selection
    int UpdateSelection(const DataBounds2D &bounds, double point_spacing,
                        LaserPoints &selection, int max_num_pts);
                        
    /// Invalidate used tile history
    void InvalidateUsedTiles();

    /// Return information on selected tiles
    void SelectedTileRange(int &min_row, int &max_row,
                           int &min_column, int &max_column,
                           int &current_level) const;

    /// Return the current level
    int UsedLevel() const {return used_level;}
    
    /// Return pointer to the used tiles
    LaserTilePtrVector * UsedTiles() {return &used_tiles;}
    
    /// Read tile (from a merged pyramid point file or tile point file)
    int ReadTile(LaserSubUnit &tile);
    
    /// Close the pyramid point file
    int ClosePointFile();
    
    /// Open the pyramid point file (and optionally provide a new point file)
    int OpenPointFile(char *filename=NULL);
    
    /// Return pointer to block at specified pyramid level
    const LaserBlock *BlockAtLevel(int level) const;
    
    /// Collect boundaries of used tiles
    void CollectUsedTileBoundaries(ObjectPoints2D &corners,
                                   LineTopologies &top) const;

    /// Determine the number of points in the pyramid
    long long int NumberOfPoints() const;
};

#endif /* _LaserPyramid_h_ */  /* Don't add after this point */
