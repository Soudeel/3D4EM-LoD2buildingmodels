
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
 * \brief Interface to Class LaserUnit - A unit of a laser altimetry block
 *
 */
/*!
 * \class LaserUnit
 * \ingroup LDataOrg
 * \brief Interface to Class LaserUnit - A unit of a laser altimetry block
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		19-03-1999 (Created)
 * \date		19-11-1999 (Update #1) - \G_Vosselman
 *
 * \remark \li Update #1: Added support for tiled strips and blocks
 * \remark \li LaserUnit - A unit of a laser altimetry block
 *                                      This can be either a strip or a
 *                                     container with tiles of laser data*
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */



#ifndef _LaserUnit_h_
#define _LaserUnit_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 LaserUnit                          - A unit of a laser altimetry block
                                      This can be either a strip or a
                                      container with tiles of laser data

 Initial creation
 Author : George Vosselman
 Date   : 19-03-1999

 Update #1
 Author : George Vosselman
 Date   : 19-11-1999
 Changes: Added support for tiled strips and blocks

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserSubUnit.h"
#include "Positions3D.h"
#include "ObjectPoints.h"
#include "LaserScanLines.h"
#include "StripTransform.h"
#include "vMatrix.h"

//------------------------------------------------------------------------------
///                     A unit of a laser altimetry block
//------------------------------------------------------------------------------

class LaserUnit: public std::vector<LaserSubUnit>, public LaserPointsInfo,
                 public LaserDataFiles
{
  protected:
    /// Kind of data organisation
    LaserDataOrganisation data_org;

    /* Specific variables for a strip of laser data */

    /// Strip number
    int strip_number;

    /// Transformation between distorted strip and terrain coordinate system
    StripTransform strip_transform;

    /// Name of the file with the flight path
    char *flight_path_file;

    /// Flight path of the strip
    Positions3D flight_path;

    /// Information on the scan line beginnings
    LaserScanLines scanlines;

    /// Name of the file with the measured tie points
    char *tie_points_file;

    /// Measured tie points
    ObjectPoints tie_points;

    /// Name of the file with the measured control points
    char *control_points_file;

    /// Measured control points
    ObjectPoints control_points;

  public:

    /// Default constructor
    LaserUnit()
      {Initialise();}

    /// Construct by reading from strip or tile meta data file
    LaserUnit(const char *filename, int *success, bool read_subunits=true)
      {Initialise(); *success = ReadMetaData(filename, read_subunits);}

    /// Construct from a single sub unit
    LaserUnit(LaserSubUnit subunit)
      {Initialise(); push_back(subunit);}

    /// Construct from a point file name
    LaserUnit(const char *filename);

    /// Copy constructor
    LaserUnit(const LaserUnit &unit);

    /// Copy assignment
    LaserUnit & operator = (const LaserUnit &unit);

    /// Initialisation of a new instant
    void Initialise();

    /// Reinitialisation of an old instant
    void ReInitialise();

    /// Read meta data
    int ReadMetaData(const char *filename, bool read_subunits=true);
  
    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to a file
    int WriteMetaData(const char *filename) const;

    /// Write meta data to "meta_file" and write sub units
    /** @param write_parts If true, the meta data of the subunits is also
                           written
    */
    int WriteMetaData(int write_parts) const;

    /// Write meta data to a file and write sub units
    /** @param filename Name of the meta data file
        @param write_parts If true, the meta data of the subunits is also
                           written
    */
    int WriteMetaData(const char *filename, int write_parts) const;

    /// Derive the file name of the meta data
    char *DeriveMetaDataFileName(const char *directory);

    /// Return the readable strip number
    int StripNumber() const
      {return(strip_number);}

    /// Return the writable strip number
    int &StripNumber()
      {return(strip_number);}

    /// Return the readable strip transform
    const StripTransform &Transform() const
      {return(strip_transform);}

    /// Return the writable strip transform
    StripTransform &Transform()
      {return(strip_transform);}

    /// Return the readable data organisation type
    LaserDataOrganisation DataOrganisation() const
      {return(data_org);}

    /// Return the writable data organisation type
    LaserDataOrganisation &DataOrganisation()
      {return(data_org);}

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

    /// Derive the unit's data bounds
    /** @param use_known_bounds If true, bounds are not recalculated if they
                                are already set
        @return The determined data bounds
    */
    DataBoundsLaser DeriveDataBounds(int use_known_bounds);

    /// Determine the zero, first and second order (non central) moments of the points
    void Moments(double &m00, double &m10, double &m01,
                 double &m20, double &m11, double &m02);
                 
    /// Return the number of points
    /** If no subunit data is available, meta data will be read
        @return Number of points in the unit
    */
    long long int NumberOfPoints() const;

/* Functions specific to strips of laser data */

    /// Generate a sub unit with one complete strip
    LaserSubUnit CompleteStrip() const;

    /// Derive the file name of the scan lines
    char *DeriveScanLinesFileName(const char *directory);

    /// Return the flight path file name
    char *FlightPathFileName()
      {return(flight_path_file);}

    /// Set the flight path file name
    void SetFlightPathFileName(const char *filename);

    /// Derive the file name of the flight path
    char *DeriveFlightPathFileName(const char *directory);
    
    /// Return the writable flight path reference
    Positions3D &FlightPath()
      {return(flight_path.Positions3DRef());}

    /// Return the readable flight path reference
    const Positions3D &FlightPath() const
      {return(flight_path.Positions3DRef());}

    /// Read the flight path
    int ReadFlightPath();

    /// Write the flight path
    int WriteFlightPath() const;

    /// Erase the flight path points
    void EraseFlightPath()
      {if (!flight_path.empty())
         flight_path.erase(flight_path.begin(), flight_path.end());}

    /// Return the writable scan lines reference
    LaserScanLines &LaserScanLinesReference()
      {return(scanlines.LaserScanLinesReference());}

    /// Return the readable scan lines reference
    const LaserScanLines &LaserScanLinesReference() const
      {return(scanlines.LaserScanLinesReference());}

    /// Derive the scan lines
    /** See LaserSubUnit::MedianInterPointDistance and
        LaserSubUnit::DeriveScanLines for more information on the algorithms.
        @param max_terrain_height The assumed maximum terrain height. Points
                                  above this height are ignored by this function
        @param num_scanlines_flight_path Number of scan lines to be used for
                                         reconstruction of a single flight path
                                         point (def:0 = no flight path reconstruction)
        @param approximate_flight_height Height of reconstructed flight path
                                         
    */
    void DeriveScanLines(double max_terrain_height,
                         int num_scanlines_flight_path=0,
                         double approximate_flight_height=0.0);

    /// Read measurements for strip adjustment
    int ReadMeasurements(int read_tie_points, int read_control_points);

    /// Return the readable tie point measurements
    const ObjectPoints &TiePointMeasurements() const
      { return tie_points;}

    /// Return the readable control point measurements
    const ObjectPoints &ControlPointMeasurements() const
      { return control_points;}

    /// Update strip error parameters
    void UpdateErrors(vMatrix &increments, int error_model)
      {strip_transform.Errors().Update(increments, error_model);}
      
    /// Extract the points in the overlap with a second strip
    /** @param strip2 The strip that overlaps this
        @param overlap The container for points in the overlap
        @param gridsize Pixelsize of an image to check the overlap
        @return The number of points in the overlap
    */
    int Overlap(LaserUnit &strip2, LaserPoints &overlap, double gridsize);

    /// Extract the points in the overlap with a second strip
    /** Points in the overlap are stored in a one of more files on disk.
        File names are generated automatically based on the strip names.
        @param strip2 The strip that overlaps this
        @param gridsize Pixelsize of an image to check the overlap
        @param max_num_pts Maximum number of points in an overlap part on disk
        @param output_directory Directory for the overlap points (default:local)
        @return The reference to the overlap meta data structure
    */
    LaserUnit & Overlap(LaserUnit &strip2, double gridsize, int max_num_pts,
                        char *output_directory=NULL);
                        
    /// Derive strip axis
    /** @param start_point Mid point of the first 5000 points of the strip
        @param end_point Mid point of the last 5000 points of the strip
        @return 2D line through start and end point
    */
    Line2D StripAxis(LaserPoint &start_point, LaserPoint &end_point);

    /// Derive strip axis
    /** Just fit a line to all points. For use in case of tiled data.
        @return 2D line through start and end point
    */
    Line2D StripAxis();
    
    /// Sort all points along the strip axis
    /** Works for arbitrarily large strips but be time consuming because of
        the required disk I/O.
        @param sorted_strip_name Name of the sorted strip
        @param output_directory Directory for all meta data and point files of 
                                the sorted strip
        @param working_directory Directory for temporary files
        @param verbose Output progress if true
        @param remove_scalar_tag If true the scalar tag used for sorting will
                                 be removed before saving the points
    */
    void SortAlongStripAxis(char *sorted_strip_name, char *output_directory,
                            char *working_directory, bool verbose=false,
                            bool remove_scalar_tag=true);

/* Functions specific to tiles of laser data */

    /// Define the tiles of a unit
    /** @param organisation Data organisation type, either TileWise or
                            StripTileWise
        @param width Width of the data tile 
        @param height Height of the data tile 
        @param border Border size of the data tile overlapping with neighbouring
                      tiles
    */
    void DefineTiles(LaserDataOrganisation organisation,
                     double width, double height, double border);

    /// Set the names of the tile meta data files
    /** @param rootname Root name of all tile meta data files
        @param directory Directory for meta data files
        @param use_subdirs Use subdirectories for every row in order to avoid
                           very large numbers of files in one directory
        @param six_digits Use six digits for tile row and columns for very large blocks
    */
    void SetTileNames(const char *rootname, const char *directory,
                      bool use_subdirs=false, bool six_digits=false);

    /// Determine number of tile rows
    int NumberOfTileRows() const;
    
    /// Determine number of tile columns
    int NumberOfTileColumns() const;
    
    /// Distribute points over the tiles
    /** Put the points into the correct tiles of this unit.
        @param points Points to be distributed over the tiles
    */
    void InsertIntoTiles(LaserPoints &points,
                         bool preserve_multiple_reflections=false);

    /// Distribute points over the tiles (quick version for tiles without border)
    /** Put the points into the correct tiles of this unit.
        @param points Points to be distributed over the tiles
    */
    void InsertIntoTilesWithoutBorder(LaserPoints &pts,
                                      bool preserve_multiple_reflections=false);

    /// Remove tiles without points
    void RemoveEmptyTiles();

    /// Erase all points
    void ErasePoints();

    /// Read the points of all unit parts
    /** This function reads the points of all unit parts if the total number
        of points does not exceed "max_num_pts". If the number of points in
        the unit is larger, all points will be erased and the function will
        return false.
        @param max_num_pts Maximum number of points to be read (see above)
        @return The number of points read. If the number of points exceeded
                "max_num_pts" -1 is returned.
    */
    int ReadPoints(int max_num_pts);
};

#endif /* _LaserUnit_h_ */  /* Don't add after this point */
