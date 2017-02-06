
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
 * \brief Interface to Class LaserSubUnit - Part of a unit of laser points
 *
 */
/*!
 * \class LaserSubUnit
 * \ingroup LDataOrg
 * \brief Interface to Class LaserSubUnit - Part of a unit of laser points
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li  The variable data_org determines the kind of data organisation:
 * <br><br>
 * <ol>
 * <li> UnknownOrganisation  - Unknown data organisation
 * <li> StripWise            - SubUnit contains the data of all or a number of
 *                        scan lines of a laser strip
 * <li> StripWise + TileWise - SubUnit contains the data within a tile of a strip
 * <li> TileWise             - SubUnit contains all data within a tile of a block,
 *                        i.e. data is aggregated over all strips
 * </ol>
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



#ifndef _LaserSubUnit_h_
#define _LaserSubUnit_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 LaserSubUnit                  - Part of a unit of laser points

 The variable data_org determines the kind of data organisation:

   UnknownOrganisation  - Unknown data organisation
   StripWise            - SubUnit contains the data of all or a number of
                          scan lines of a laser strip
   StripWise + TileWise - SubUnit contains the data within a tile of a strip
   TileWise             - SubUnit contains all data within a tile of a block,
                          i.e. data is aggregated over all strips

--------------------------------------------------------------------------------
*/

#include "LaserPoints.h"
#include "LaserScanLines.h"
#include "ObjectPoints2D.h"
#include "LineTopologies.h"

/** Types of laser data organisation.
    Possible types are:
        UnknownOrganisation,
        StripWise (block in strips, possibly also strip parts)
        TileWise (block in tiles), and
        StripTileWise (block in strips, strips in tiles)
*/
enum LaserDataOrganisation {UnknownOrganisation=0, /* Unknown organisation    */
                            StripWise,             /* Block in strip (parts)  */
                            TileWise,              /* Block in tiles          */
                            StripTileWise };       /* Block in strips in tiles*/
typedef enum LaserDataOrganisation LaserDataOrganisation;

//------------------------------------------------------------------------------
///                     A part of a unit of laser points
//------------------------------------------------------------------------------

class LaserSubUnit: public LaserPoints
{
  protected:

    /// Kind of data organisation
    LaserDataOrganisation data_org;

/* Specific variables for a part of a laser strip */

    /// Flag indicating the sub unit is a complete strip
    int whole_strip;

    /// First scan line of the strip part
    int first_scanline;

    /// Last scan line of the strip part
    int last_scanline;

/* Specific variables for a tile of laser data */

    /// Row coordinate of the tile
    int tile_row;

    /// Column coordinate of the tile
    int tile_column;

    /// Pointer to tile at the north tile edge
    LaserSubUnit *north;

    /// Pointer to tile at the east tile edge
    LaserSubUnit *east;

    /// Pointer to tile at the west tile edge
    LaserSubUnit *west;

    /// Pointer to tile at the south tile edge
    LaserSubUnit *south;

    /// Bounds of the non-overlapping tile
    DataBoundsLaser tile_bounds;

    /// Border around the non-overlapping tile
    double tile_border;

  public:

    /// Default constructor
    LaserSubUnit()
      {Initialise();}

    /// Copy constructor
    LaserSubUnit(const LaserSubUnit &);
 
    /// Copy assignment
    LaserSubUnit & operator = (const LaserSubUnit &);

    /// Construct by reading meta data
    LaserSubUnit(const char *filename, int *success)
      {Initialise(); *success = ReadMetaData(filename);}
      
    /// Construct by reading meta data
    LaserSubUnit(FILE *fd, LaserDataOrganisation organisation, int *success)
      {Initialise(); *success = ReadMetaData(fd, organisation);}

    /// Construct from a set of laser points
    LaserSubUnit(const LaserPoints &pts)
      {Initialise(); LaserPointsReference() = pts.LaserPointsReference();}

    /// Construct an empty tile
    /** @param organisation Data organisation type
        @param row Tile row number
        @param column Tile column number
        @param bounds Interior tile bounds (i.e. without border)
        @param border Size of the tile border
    */
    LaserSubUnit(LaserDataOrganisation organisation, int row, int column,
                 DataBoundsLaser &bounds, double border);

    /// Initialisation of a new instant
    void Initialise();

    /// Reinitialisation of an old instant
    void ReInitialise();

    /// Read the meta data
    int ReadMetaData(const char *filename);
  
    /// Read the meta data
    int ReadMetaData(FILE *fd, LaserDataOrganisation organisation);
    
    /// Write the meta data to "meta_file"
    int WriteMetaData() const;

    /// Write the meta data to a file
    int WriteMetaData(const char *filename) const;

    /// Write the meta data to a file
    int WriteMetaData(FILE *fd) const;

    /// Derive the name of the sub unit
    /** @param parent_name Name of the parent unit
        @param first_subunit Pointer to the first subunit. This pointer is
                             used to calculate the part number in case of
                             StripWise data organisation
        @param six_digits Use six digits for tile row or column in very large blocks
    */
    char *DeriveName(const char *parent_name, LaserSubUnit *first_subunit,
                     bool six_digits=false);

    /// Derive the name of the meta data file
    char *DeriveMetaDataFileName(const char *directory);

    /// Set the names of all files
    /** Derive the subunit name and set the names of the point and meta data
        file.
        @param parent_name Name of the parent unit
        @param first_subunit Pointer to the first subunit. This pointer is
                             used to calculate the part number in case of
                             StripWise data organisation
        @param six_digits Use six digits for tile row and column numbers
                          for very large blocks
    */
    void SetFileNames(const char *parent_name, const char *directory,
                      LaserSubUnit *first_subunit, bool six_digits=false);

    /// Return the readable data organisation type
    LaserDataOrganisation DataOrganisation() const
      {return(data_org);}

    /// Return the writable data organisation type
    LaserDataOrganisation & DataOrganisation()
      {return(data_org);}

/* Functions specific to parts of laser strips */

    /// Set the first and last scanline
    void SetScanLineSubSet(int l1, int l2)
      {first_scanline = l1; last_scanline  = l2;}

    /// Return the complete strip flag
    int IsCompleteStrip() const
      {return(whole_strip);}

    /// Mark the sub unit as a complete strip
    void SetAsCompleteStrip()
      {whole_strip = 1;}

    /// Mark the sub unit as an incomplete strip
    void SetAsStripPart()
      {whole_strip = 0;}

    /// Derive the scan lines from the laser data
    /** For the analysis high points like reflections on birds should not be
        used, since their XY coordinates may be very different from other points
        and cause the "detection" of a scan line end. Therefore, only points
        below a certain height (max_terrain_height) are used.
        An end point is detected by checking the distances of points to the
        first point of a scan line. First, this distance will increase. After
        the end of the scan line this distance will become smaller. If the
        distance between the current point and the point most distance from the
        first point is larger than a multiple of the median distance between
        consecutive points, the latter point is considered to be the end of the
        scan line.
        If the approximate flight height is specified, a second algorithm will
        be applied. The results of the first algorithm are used to construct
        the approximate flight path (as a straight line). Using this line
        a second analysis to detect the scan line ends is performed. This
        analysis does not use distances in the terrain, but direction vectors
        from the scanner to the terrain points. In this way, height differences
        do not influence the scan line end detection. However, the flight path
        should be close to a straight line for this algorithm to work correctly.
        @param lines The derived scan lines
        @param median_dist Median distance between two consecutive points. Use
                           MedianInterPointDistance to compute this.
        @param max_terrain_height Assumed maximum terrain height
        @param approximate_flight_height Approximate flight height
        @param num_scanlines_flight_path Number of scan lines to be used for
                                         reconstruction of a single flight path
                                         point (def:0 = no flight path reconstruction
        @param flight_path Points of the reconstructed flight path
        @param flight_path_point Storage of data to be transferred to the
                                 derivation of scan lines of the next strip part
        @param num_flight_path_summed Idem
    */
    void DeriveScanLines(LaserScanLines &lines, double median_dist,
                         double max_terrain_height,
                         double approximate_flight_height,
                         int num_scanlines_flight_path,
                         Positions3D &flight_path,
                         Position3D &flight_path_point,
                         int &num_flight_path_summed);

    /// Reduce the data scanline-wise
    /** One scan line out of each series of "scanline_reduction_factor" scan
        lines is selected. Of this scan line one point out of each series of
        "point_reduction_factor" consecutive points is selected.
        @param lines The scan line information
        @param first_num_in_part The point number of the first point in this
                                 subunit. In case of a strip with multiple
                                 strip parts, this number is needed to determine
                                 the appropriate scan line information
        @param point_reduction_factor See above.
        @param random_point If set, a random point is selected out of the set of
                            points, otherwise each first point is selected.
        @param scanline_reduction_factor See above.
        @param random_scanline If set, a random scan line is selected out of
                               the set of scan lines, otherwise each first scan
                               line is selected.
    */
    void ReduceData(const LaserScanLines &lines, int first_num_in_part,
                    int point_reduction_factor, int random_point,
                    int scanline_reduction_factor, int random_scanline);

    /// Return the number of points in the subunit
    /** If no points are in memory, the header of the point file will
        be read to obtain the number of points.
        @return The number of points in the subunit
    */
    long long int NumberOfPoints() const;
    
/* Functions specific to tiles */

    /// Return the bounds of the tile
    const DataBoundsLaser TileBounds() const
      {return(tile_bounds);}

    /// Return the reference to the tile bounds
    DataBoundsLaser & TileBounds()
      {return tile_bounds;}
      
    /// Return the tile coordinates
    void TileLocation(int *row, int *column) const
      {*row = tile_row; *column = tile_column;}

    /// Set the tile coordinates
    void SetTileLocation(int row, int column)
      {tile_row = row; tile_column = column;}

    /// Return the tile row
    int TileRow() const
      {return tile_row;}
      
    /// Return the tile column
    int TileColumn() const
      {return tile_column;}
      
    /// Return the tile number as 1000 * tile row + tile column
    int TileNumber() const
      {return tile_row * 1000 + tile_column;}
      
    /// Compare tile location
    /** If the location of this tile is before (row, column), -1 is returned.
        If the location equals (row, column) 0 is returned.
        If the location is after (row, column) 1 is returned.
        It is assumed that tiles are first sorted after row and then after
        column.
    */
    int CompareTileLocation(int row, int column) const;
    
    /// Return the tile border size
    double TileBorder() const
      {return(tile_border);}

    /// Set the tile border size
    void SetTileBorder(double border)
      {tile_border = border;}
      
    /// Insert points into the tile
    void InsertIntoTile(LaserPoints &points,
                        bool preserve_multiple_reflections=false);

    /// Add tile boundaries to a list of boundaries
    /** @param points Points of tile boundaries
        @param topology Topology of tile boundaries
    */
    void AddTileBoundary(ObjectPoints2D &points, 
                         LineTopologies &topology) const;

    /// Select points within the specified bounds AND the tile bounds
    /** This function is overloading the same function in the LaserPoints class.
        By also respecting the tile bounds, points in the tile border will
        only be selected once if this function is used to select from a
        tiles block.
    */
    void Select(LaserPoints &selection, const DataBoundsLaser &bounds) const;

    /// Select a subset of points within polygons and within tile bounds
    /** All laser points within the given polygons are added to the selection.
        To speed up the inside-polygon computations, it is first checked
        whether the bounds of the laser points overlap with the bounds of the
        polygons. For this purpose, the bounds of the laser points are computed
        if they are not yet available. Therefore this function is not a const
        function.
        In contrast to the same function of class LaserPoints, this function
        also makes sure that data of a tile border (in case the subunit is
        a tile) is not selected (in order to avoid double selection).
        @param selection The set of laser points to which the points within the
                         polygons are added
        @param points    The corner points of the polygons
        @param topology  The node numbers of the corners of the polygons
        @return The number of points added to the selection.
    */
    int Select(LaserPoints &selection, const ObjectPoints &points,
               const LineTopologies &polygons);


/* Functions defined in file LaserSubUnitF.cc */

    /// Delete all filtered points
    void RemoveFilteredPoints();

    /// Select all filtered points
    /** @param all_points A set of filtered and unfiltered points
    */
    void SelectFilteredPoints(const LaserSubUnit &all_points);
};

#endif /* _LaserSubUnit_h_ */  /* Don't add after this point */
