
/*
                  Copyright 2010 University of Twente
 
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
 * \brief Class RoadPart - Outline, modelled objects and related laser data of a road part
 *
 */
/*!
 * \class RoadPart
 * \ingroup Photogrammetry
 * \brief Class RoadPart - Outline, modelled objects and related laser data of a road part
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          15-12-2009(Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef ROADPART_H
#define ROADPART_H

enum RoadObjectType { UnknownRoadObject, RoadMarking, RoadSide,
                      RoadBuilding, RoadSideBarrier, RoadTrafficLight,
                      RoadTrafficSign, RoadStreetLight};
typedef enum RoadObjectType RoadObjectType;


#include "FeatureNumber.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LaserTilePtrVector.h"
#include "LaserBlock.h"

#include <stdio.h>
#include <stdlib.h>

class RoadPart : public FeatureNumber
{
  protected:
    /// Name of the road part
    char *name;
    
    /// Name of the meta data file
    char *meta_file;
    
    /// Road part outline points file
    char *outline_points_file;
    
    /// Road part outline points
    ObjectPoints *outline_points;
    
    /// Road part outlines topology file
    char *outline_topology_file;
    
    /// Road part outline topology
    LineTopology *outline_top;
    
    /// Road part outline topology with overlap
    LineTopology *outline_overlap_top;
    
    /// Road part centre line topology
    LineTopology *centre_line_top;
    
    /// Road part centre line topology with overlap
    LineTopology *centre_line_overlap_top;
    
    /// Road part inventory points file
    char *inventory_points_file;
    
    /// Road part inventory points
    ObjectPoints *inventory_points;

    /// Road part inventory topology file
    char *inventory_topology_file;
    
    /// Road part object topology
    LineTopologies *inventory_tops;
    
    /// Related laser point tiles
    LaserTilePtrVector tiles;
    
    /// Laser point file of this part
    char *laser_points_file;
    
    /// Laser points within this part
    LaserPoints laser_points;

  public:
    /// Default constructor
    RoadPart();
    
    /// Construct from outline
    /** @param part_no Part number
        @param points Outline and centre line points
        @param top Topology of outline without overlap
        @param overlap_top Topology of outline with overlap
        @param line_top Topology of road part centre line
        @param line_overlap_top Topology of road part centre line with overlap
    */
    RoadPart(int part_no, ObjectPoints *points,
             LineTopology *top, LineTopology *overlap_top,
             LineTopology *line_top, LineTopology *line_overlap_top);
    
    /// Default destructor
    ~RoadPart() {};

    /// Re-initialisation
    void ReInitialise();
    
    /// Copy assignment
    RoadPart & operator = (const RoadPart &);

// Return class members (const and non-const functions)

    /// Return the pointer
    RoadPart * RoadPartPtr() {return this;}

    /// Return the part outline points
    ObjectPoints * OutlinePoints() {return outline_points;}

    /// Return the part outline points
    const ObjectPoints * OutlinePoints() const {return outline_points;}

    /// Return the part outline topology
    LineTopology * OutlineTopology()  {return outline_top;}

    /// Return the part outline topology
    const LineTopology * OutlineTopology() const {return outline_top;}

    /// Return the part outline topology with overlap
    LineTopology * OutlineOverlapTopology()  {return outline_overlap_top;}

    /// Return the part outline topology with overlap
    const LineTopology * OutlineOverlapTopology() const {return outline_overlap_top;}

    /// Return the centre line topology
    LineTopology * CentreLineTopology()  {return centre_line_top;}

    /// Return the centre line topology
    const LineTopology * CentreLineTopology() const {return centre_line_top;}

    /// Return the centre line topology with overlap
    LineTopology * CentreLineOverlapTopology()  {return centre_line_overlap_top;}

    /// Return the centre line topology with overlap
    const LineTopology * CentreLineOverlapTopology() const {return centre_line_overlap_top;}

    /// Return the road inventory points
    ObjectPoints * InventoryPoints() {return inventory_points;}

    /// Return the road inventory points
    const ObjectPoints * InventoryPoints() const {return inventory_points;}

    /// Return the topology of the road inventory
    LineTopologies * InventoryTopologies() {return inventory_tops;}

    /// Return the topology of the road inventory
    const LineTopologies * InventoryTopologies() const {return inventory_tops;}

    /// Return the tiles of this road part
    LaserTilePtrVector & Tiles() {return tiles;}

    /// Return the tiles of this road part (const version)
    const LaserTilePtrVector & Tiles() const {return tiles;}

    /// Return the laser points of this road part
    LaserPoints & LaserData() {return laser_points;}

    /// Return the laser points of this road part (const version)
    const LaserPoints & LaserData() const {return laser_points;}

// Names

    /// Derive name of road part from road name and part number
    char *DeriveName(const char *road_name);
    
    /// Derive meta data file name
    char *DeriveMetaDataFileName(const char *directory);
    
    /// Derive names of road part outline files (points and topology)
    void DeriveOutlineFileNames(const char *directory);
    
    /// Derive names of road part inventory files (points and topology)
    void DeriveInventoryFileNames(const char *directory);
    
    /// Derive name of the laesr point file
    char *DeriveLaserPointsFileName(const char *directory);
    
    /// Return road part name
    const char * Name() const {return name;}
    
    /// Return meta data file name
    const char * MetaDataFile() const {return meta_file;}
    
    /// Return outline points file name
    const char * OutlinePointsFile() const {return outline_points_file;}
    
    /// Return outline topology file name
    const char * OutlineTopologyFile() const {return outline_topology_file;}
    
    /// Return inventory points file name
    const char * InventoryPointsFile() const {return inventory_points_file;}
    
    /// Return inventory topology file name
    const char * InventoryTopologyFile() const {return inventory_topology_file;}
    
    /// Return laser points file name
    const char * LaserPointsFile() const {return laser_points_file;}
    
// Data I/O

    /// Read road part meta data
    /** @param filename File name of the road part meta data. This file name
                        is stored in the variable meta_file.
        @return true if reading was successful, otherwise false
    */
    bool ReadMetaData(const char *filename);
    
    /// Write road part meta data
    /** Writing of the road part meta data to the file specified in the 
        class variable meta_file.
        @return true if writing was successful, otherwise false
    */
    bool WriteMetaData() const;
    
    /// Read the points and topology of outline and centre line (with and without overlap)
    /** Reading of the outline points and topologies to the files specified
        in the class variables outline_points_file and outline_topology_file.
        @return true if reading was successful, otherwise false
    */
    bool ReadOutlines();
    
    /// Write the points and topology of outline and centre line (with and without overlap)
    /** Writing of the outline points and topologies to the files specified
        in the class variables outline_points_file and outline_topology_file.
        @return true if writing was successful, otherwise false
    */
    bool WriteOutlines() const;
    
    /// Read the points and topology of the road part inventory
    /** Reading of the inventory points and topologies to the files specified
        in the class variables inventory_points_file and inventory_topology_file.
        @return true if reading was successful, otherwise false
    */
    bool ReadInventory();
    
    /// Write the points and topology of the road part inventory
    /** Writing of the inventory points and topologies to the files specified
        in the class variables inventory_points_file and inventory_topology_file.
        @return true if writing was successful, otherwise false
    */
    bool WriteInventory() const;
    
    /// Read the laser points
    /** Reading the laser points from the file name specified in class
        variable laser_points_file.
        @return true if reading was successful, otherwise false
    */
    bool ReadLaserPoints()
      { return (bool) laser_points.Read(laser_points_file, false); }
    
    /// Write the laser points
    /** Writing the laser points to the file name specified in class
        variable laser_points_file.
        @return true if writing was successful, otherwise false
    */
    bool WriteLaserPoints()
      { return (bool) laser_points.Write(laser_points_file, 0, false); }
    
    /// Erase the laser points
    void EraseLaserPoints()
      { laser_points.ErasePoints(); }
    
// Collect data

   /// Determine tiles that overlap with this road part
   /** @param block Tiled laser block
       @param include_overlap If true points are also selected from
                              the overlap with other road parts
       @return Number of tiles overlapping with this block.
               A negative number is returned in case of errors
   */
   int CollectTiles(LaserBlock &block, bool include_overlap);
   
// To be implemented

   /// Extract points out of tiles overlapping with this part
   /** @param include_overlap If true points are also selected from
                              the overlap with other road parts
       @param clip_at_bounds If false, complete tiles are selected,
                             otherwise only points inside the road part bounds
       @return Number of collected points.
               A negative number is returned in case of errors
   */
   int CollectLaserPoints(bool include_overlap, bool clip_at_bounds);


// Further useful functions may be copied from BuildingPart.h
};
#endif // ROADPART_H
