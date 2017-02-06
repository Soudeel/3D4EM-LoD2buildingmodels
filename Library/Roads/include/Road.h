
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
 * \brief Class Road - Set of road parts
 *
 */
/*!
 * \class Road
 * \ingroup Photogrammetry
 * \brief Class Building - Set of road parts
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date          15-12-2009 (Created)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 */

#ifndef ROAD_H
#define ROAD_H

#include "RoadPart.h"
#include "LaserUnit.h"
#include "BNF_io.h"

class Road : public vector <RoadPart>
{
  protected:
    /// Name of the road
    char *name;
    
    /// Name of the meta data file
    char *meta_file;

  public:
    /// Default constructor
    Road() {name = meta_file = NULL;}

    /// Default destructor
    ~Road() {};

    /// Return the reference
    Road & RoadRef() {return *this;}

    /// Return the const reference
    const Road & RoadRef() const {return *this;}

    /// Return the road pointer
    Road * RoadPtr() {return this;}

    /// Return a specific road part
    RoadPart * RoadPartPtr(int number);
    
    /// Return the road name
    const char *Name() const {return name;}
    
    /// Set the road name
    void SetName(const char *road_name)
      {if (name) free(name); StringCopy(&name, road_name);}
      
    /// Derive the meta data file name
    /** @param directory Directory of the meta data file name
        @return Pointer to the file name as stored in the class variable meta_file
    */
    char *DeriveMetaDataFileName(const char *directory);

    /// Read road meta data
    /** Reading of road meta data, including meta data of road parts listed
        in the meta data file.
        @param filename File name of the road meta data
        @return true if reading was successful, otherwise false
    */
    bool ReadMetaData(const char *filename);
    
    /// Write road meta data
    /** Writing of road meta data to the file name as stored in the
        class variable meta_file. 
        @param write_road_parts If true, meta data of road parts is also written
        @return true if writing was successful, otherwise false
    */
    bool WriteMetaData(bool write_road_parts) const;
    
// Further useful functions may be copied from Building.h

    /// Initialise road parts based on the car trajectory and part size specification
    /** @param car_trajectory Sequence of GPS positions of a strip
        @param track_part_length Length of the road part polygon
        @param track_width Width of the road part polygon
        @param track_part_overlap Overlap in m between adjacent road parts
        @param track_outline_step_size Discretisation step of part outline
        @param directory Directory for road part meta data
        @return Number of initialised parts if success, negative number in case of failure
    */
    int InitialiseRoadParts(const Positions3D &car_trajectory,
                            double track_part_length, double track_width,
                            double track_part_overlap,
                            double track_outline_step_size,
                            const char *directory); 
                            
    /// Collect the outlines from all road parts
    /** Gather the outlines from all road parts. Depending on the last switch
        the outlines contain the overlaps betweeen parts or not.
        @param points Points of the road part outlines
        @param tops Topologies of the road part outlines
        @param with_overlap If true, the road parts with overlap are collected
    */
    void CollectRoadPartOutlines(ObjectPoints &points, LineTopologies &tops,
                                 bool with_overlap=false) const;
                                 
    /// Read outlines of all road parts
    /** @return true if reading was successful, otherwise false
    */
    bool ReadRoadPartOutlines();
    
    /// Write outlines of all road parts
    /** @return true if writing was successful, otherwise false
    */
    bool WriteRoadPartOutlines() const;

    /// Read inventory data of all road parts
    /** @return true if reading was successful, otherwise false
    */
    bool ReadRoadPartInventories();
    
    /// Write inventory data of all road parts
    /** @return true if writing was successful, otherwise false
    */
    bool WriteRoadPartInventories() const;
};

// functions to generate file names
// Add meta_file to all functions in roads
// Add access functions for file names
// functions for reading and writing meta data of class roads
// functions for reading and writing inventory
#endif // ROAD_H
