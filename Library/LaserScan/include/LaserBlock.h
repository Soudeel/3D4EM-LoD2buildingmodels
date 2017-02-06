
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
 * \brief Interface to Class LaserBlock - A laser altimetry block
 *
 */
/*!
 * \class LaserBlock
 * \ingroup LDataOrg
 * \brief Interface to Class LaserBlock - A laser altimetry block
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		19-03-1999 (Created)
 * \date        19-11-1999 (Update #1) - \G_Vosselman
 *
 * \remark \li Update #1: Added support for tiled blocks
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


#ifndef _LaserBlock_h_
#define _LaserBlock_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 LaserBlock                          - A laser altimetry block

 Initial creation
 Author : George Vosselman
 Date   : 19-03-1999

 Update #1
 Author : George Vosselman
 Date   : 19-11-1999
 Changes: Added support for tiled blocks

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserUnit.h"
#include "ControlPoints.h"
#include "DataBounds2D.h"
#include "LaserTilePtrVector.h"

class Adjustment;

//------------------------------------------------------------------------------
///                       Laser altimetry block
//------------------------------------------------------------------------------

class LaserBlock: public std::vector<LaserUnit>, public LaserPointsInfo,
                  public LaserMetaFile
{
  protected:
    /// Name of the file with the control points
    char *control_points_file;

    /// Control points
    ControlPoints control_points;

  public:

    /// Default constructor
    LaserBlock()
      {Initialise();}

    /// Construct block by reading block meta data
    LaserBlock(const char *filename, int *success)
      {Initialise(); *success = ReadMetaData(filename);}

    /// Construct block from arbitrary laser meta data or point file
    LaserBlock(const char *filename, int *filetype, int *success)
      {Initialise(); *success = Create(filename, filetype);}

    /// Construct block with one unit
    LaserBlock(const LaserUnit &unit)
      {Initialise(); push_back(unit);}

    /// Construct block from filter of point file names
    LaserBlock(const char *file_filter);

    /// Copy assignment
    LaserBlock & operator = (const LaserBlock &);

    /// Initialisation of new block
    void Initialise();

    /// Reinitialisation of existing block
    void ReInitialise();

    /// Create a block from arbitrary laser meta data file or a point file
    int Create(const char *filename, int *fileclass, bool read_units=true,
               bool read_subunits=true);

    /// Read meta data from a file
    int ReadMetaData(const char *filename, bool read_units=true,
                     bool read_subunit=true);
  
    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to a file
    int WriteMetaData(const char *filename) const;

    /// Write meta data of block to "meta_file" and of (sub)units
    int WriteMetaData(int write_units, int write_subunits) const;

    /// Write meta data of block to a file and of (sub)units
    int WriteMetaData(const char *filename, int write_units,
                      int write_subunits) const;

    /// Derive the name of the meta data file
    char *DeriveMetaDataFileName(const char *directory);

    /// Add laser data to an image
    /** Conversion of the laser data to an image.
        @param image The rasterised laser data
        @param dist_image Image to keep track of the nearest laser points
        @param grid The image grid specification
        @param type HeightData, ReflectanceData or ColourData
        @param int_method Interpolation method:
                    1 - Nearest neighbour within a pixel (also called binning),
                    2 - Barycentric coordinates,
                    3 - Planar interpolation in TIN mesh,
                    4 - Minimum value within a pixel
        @param max_mesh_size Maximum mesh size to interpolate
                             (relevant for method 2 and 3 only)
    */
    void ImageData(Image &image, Image &dist_image, const DataGrid &grid,
                   ImagedDataType type, int int_method, double max_mesh_size);

    /// Derive the data bounds of a block
    /** @param use_known_bounds If true, bounds are not recalculated if they
                                are already set
        @return The determined data bounds
    */
    DataBoundsLaser DeriveDataBounds(int use_known_bounds);

    /// Write the boundary lines of block tiles
    /** Store the corner points of the tiles as object points and store the
        topology of the squares.
    */
    int WriteTileBoundaries(const char *corner_file,
                            const char *topology_file) const;

    /// Collect tile boundaries 
    void CollectTileBoundaries(ObjectPoints2D &corners,
                               LineTopologies &top,
                               const LaserTilePtrVector &tiles) const;

    /// Collect tile boundaries in a certain area
    void CollectTileBoundaries(ObjectPoints2D &corners,
                               LineTopologies &top,
                               const DataBounds2D &area_bounds);

    /// Strip adjustment
    /** Strip adjustment based on measured coordinates of tie points and
        control points. The error model can be either the 3 parameter model
        of the Survey department of Rijkswaterstaat (height offset and 2
        tilt parameters) or a 9 parameter model (3 position offsets, 3
        rotation offsets and 3 rotation drifts). In the first error model
        only the height difference is used as observation. In the second
        model, all three coordinate differences are used as observations.
        @param error_model Either 1 or 2, see above.
        @param correct_points If true, the laser points are corrected with
                              the estimated error parameters. This feature
                              is NOT YET IMPLEMENTED!
        @param output_file Text file for adjustment output
        @return 1 if succesfull, 0 otherwise.
    */
    int AdjustStrips(int error_model, int correct_points,
                     const char *output_file);

    /// Read the block control points
    int ReadControlPoints();

    /// Return the readable control points
    const ControlPoints &ControlPointsRef() const
      {return(control_points);}

    /// Return the control points file name
    const char * ControlPointsFileName() const
      {return control_points_file;}
      
    /// Estimate sigma naught from the residuals
    /** Function used by AdjustStrips. See there for explanation of the
        used error models
        @param error_model Error model for strip adjustment
        @return The estimated a posteriori standard deviation
    */
    double SigmaNaught(int error_model);

    /// Add the tie point observations to the normal equation system
    /** Function used by AdjustStrips. See there for explanation of the
        used error models
        @param adjustment Adjustment class with normal matrix
        @param error_model Error model for strip adjustment
        @return The estimated a posteriori standard deviation
    */
    int AddTiePointObs(Adjustment &adjustment, int error_model) const;

    /// Add the tie point observations to the normal equation system
    /** Function used by AdjustStrips. See there for explanation of the
        used error models
        @param adjustment Adjustment class with normal matrix
        @param error_model Error model for strip adjustment
        @return The estimated a posteriori standard deviation
    */
    int AddControlPointObs(Adjustment &, int) const;

    /// Update the estimated strip error parameters
    /** Function used by AdjustStrips. See there for explanation of the
        used error models. After each iteration the estimated errors are
        updated.
        @param increments Incremental estimated errors
        @param error_model Error model for strip adjustment
    */
    void UpdateStripErrors(const vMatrix &increments, int error_model);

    /// Erase all points
    void ErasePoints();

    /// Read the points of all block parts
    /** This function reads the points of all block parts if the total number
        of points does not exceed "max_num_pts". If the number of points in
        the block is larger, all points will be erased and the function will
        return false.
        @param max_num_pts Maximum number of points to be read (see above)
        @return The number of read points. If the number exceeded "max_num_pts",
                -1 is returned.
    */
    int ReadPoints(int max_num_pts);
    
    /// Return the number of tile rows
    int NumberOfTileRows() const;
    
    /// Return the number of tile columns
    int NumberOfTileColumns() const;
    
    /// Return the tile of the specified tile coordinates
    LaserSubUnit *Tile(int tile_row, int tile_column);
    
    /// Return the tile of the specified tile coordinates (const version)
    const LaserSubUnit *Tile(int tile_row, int tile_column) const;
    
    /// Return the tile at a position
    LaserSubUnit *Tile(const Position2D &point);

    /// Retrieve the tile row index of a Y-coordinate
    int TileRow(double Y) const;
    
    /// Retrieve the tile column index of a X-coordinate
    int TileColumn(double X) const;
    
    /// Select tiles in a specific area
    LaserTilePtrVector & SelectTiles(const DataBounds2D &bounds);
    
    /// Select data in a specific area
    void SelectPoints(const DataBounds2D &area_bounds, LaserPoints &selection);

    /// Select data in a rotated rectangle
    void SelectPoints(LaserPoints &selection,
                      const Line2D &line, double max_dist,
                      double scalar_min, double scalar_max);
                      
    /// Determine the number of points in the block
    long long int NumberOfPoints() const;
};

#endif /* _LaserBlock_h_ */  /* Don't add after this point */
