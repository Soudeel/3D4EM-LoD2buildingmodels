
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



#ifndef DATATYPES_H
#define DATATYPES_H

// Data types of the point cloud mapping programme

enum DataType {MapData, MapPartitionData, ModelData, LastModelData, LaserData,
               SelectedMapData, SelectedMapPartitionData, SelectedModelData,
               SelectedModelPartData, SelectedModelFaceData, SelectedLaserData,
               NumDataTypes,
               SelectionBoxData, SplitLineData, ExtensionLineData,
               SelectedPointData, TextureData, TileBoundaryData,
               NumAllDataTypes};
typedef enum DataType DataType;
#define NumObjectDataTypes 4
#define NumNormalDataTypes 5


// Line labels for map and model data

enum DataLabel {RoofLabel, MapLabel, MapPartitionLabel, WallLabel=5};


// Types of background images (texture maps)

enum BackGroundType {NoBackGroundImage, HeightImage, ShadedHeightImage,
                     OrthoImage, NumBackGroundTypes};
typedef enum BackGroundType BackGroundType;

#endif // DATATYPES_H
