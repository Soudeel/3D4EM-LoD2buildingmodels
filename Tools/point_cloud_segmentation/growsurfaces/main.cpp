
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


/*-----------------------------------------------------------
|
  segmentlaser
  
  Segmentation of laser data into planar or smooth surfaces
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "SegmentationParameters.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: growsurfaces [-i <file name> OR -f <file filter>] : input files\n");
  printf("        [-odir <directory name>]            : output directory\n");
  printf("        [-app <appendix>]                   : append string for output files\n");
  printf("        [-meta]                             : generate meta data files\n");
  printf("        [-overwrite]                        : overwrite old files\n");
  printf("        [-t]                                : store tile numbers in output\n");
  printf("        [-par <parameter file (default: segmentlaser.par)>]\n");
  printf("        [-savepar <parameter file (default: growsurfaces.par)>]\n");
  printf("Type \"growsurfaces -parameters\" to get the listing of surface growing parameters\n");
}

void PrintParameters()
{
  printf("Segmentation parameters of the growsurfaces programme\n");
  printf("        [-tin OR -octree OR -knn (default)] : neighbourhood type\n");
  printf("        [-ocbinmax <size> (def: 100)        : maximum octree bin size\n");
  printf("        [-ocbinoverlap <size> (def: 1.0)    : maximum octree bin overlap\n");
  printf("        [-knn <number> (def: 20)            : number of nearest neighbours\n");
  printf("        [-dim <2 or 3> (def: 3)             : neighbour metric dimension\n");
  printf("        [-minsegsize <size> (def: 10)       : minimum segment size\n\n");
  printf("        [-seednbh <0: direct (def), 1: dist>: seed neighbourhood definition\n");
  printf("        [-seedradius <number> (def: 1.0)    : seed neighbourhood radius\n");
  printf("        [-maxslope <degrees> (def: 90.0)    : maximum plane slope\n");
  printf("        [-binsizeslope <degrees> (def: 3.0) : Hough bin size of slope\n");
  printf("        [-binsizedist <number> (def: 0.2)   : Hough bin size of distance\n");
  printf("        [-minsizeseed <number> (def: 10)    : minimum seed size (#pts)\n");
  printf("        [-maxdistseed <number> (def: 0.2)   : maximum distance of point to seed\n\n");
  printf("        [-smooth] OR [-plane (def)]         : surface type to be grown\n");
  printf("        [-grownbh <0: direct (def), 1: dist>: growing neighbourhood definition\n");
  printf("        [-growradius <number> (def: 0.3)    : growing search radius\n");
  printf("        [-maxdistgrow <number> (def: 0.3)   : maximum distance of point to surface\n");
  printf("        [-mindistrecompute <num> (def: 0.15): minimum distance to recompute surface\n");
  printf("        [-compete]                          : let planes compete for points on edge\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  SegmentationParameters par;

  void growsurfaces(char *, char *, char *, char *, bool, bool, bool,
                    const SegmentationParameters &);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-parameters")) {
    PrintParameters();
    exit(0);
  }
  
  // Check on required input files
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <filename> or -f <filter>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f can not be used simultaneously!\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-app") && !args->Contains("-overwrite")) {
    printf("Error: no appendix for output files specified with -app <appendix>.\n");
    printf("       The input file is only overwritten if you specify -overwrite.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Try to read segmentation parameters if specified
  if (args->Contains("-par")) par.Read(args->String("-par", "growsurfaces.par"));

  // Get all individually set segmentation parameters
  // Neighbourhood storage model
  if (args->Contains("-tin")) par.NeighbourhoodStorageModel() = 0;
  else if (args->Contains("-octree")) {
    par.NeighbourhoodStorageModel() = 1;
    par.OctreeBinMaxNumberOfPoints() = args->Integer("-ocbinmax", 100);
    par.OctreeBinOverlap() = args->Double("-ocbinoverlap", 1.0);
  }
  else if (!args->Contains("-par")) {
    par.NeighbourhoodStorageModel() = 2; // knn, default
    par.NumberOfNeighbours() = args->Integer("-knn", 20);
  }
  if (args->Contains("-dim"))
    par.DistanceMetricDimension() = args->Integer("-dim", 3);
  
  // Minimum component size
  if (args->Contains("-minsegsize"))
    par.MinNumberOfPointsComponent() = args->Integer("-minsegsize", 10);

  // Seed selection parameters
  if (args->Contains("-seednbh"))
    par.SeedNeighbourhoodDefinition() = args->Integer("-seednbh", 0);
  if (args->Contains("-seedradius"))
    par.SeedNeighbourhoodRadius() = args->Double("-seedradius", 1.0);
  if (args->Contains("-maxslope"))
    par.MaxSlopeAngle() = args->Double("-maxslope", 90.0) * atan(1.0) / 45.0;
  if (args->Contains("-binsizeslope"))
    par.BinSizeSlopeAngle() = args->Double("-binsizeslope", 3.0) * atan(1.0) / 45.0;
  if (args->Contains("-binsizedist"))
    par.BinSizeDistance() = args->Double("-binsizedist", 0.2);
  if (args->Contains("-minsizeseed"))
    par.MinNumberOfPointsSeed() = args->Integer("-minsizeseed", 10);
  if (args->Contains("-maxdistseed"))
    par.MaxDistanceSeedPlane() = args->Double("-maxdistseed", 0.2);
  
  // Surface growing parameters
  if (args->Contains("-smooth")) par.SurfaceModel() = 1;
  else if (!args->Contains("-par")) par.SurfaceModel() = 0; // Plane
  if (args->Contains("-grownbh"))
    par.GrowingNeighbourhoodDefinition() = args->Integer("-grownbh", 0);
  if (args->Contains("-growradius"))
    par.GrowingRadius() = args->Double("-growradius", 0.3);
  if (args->Contains("-maxdistgrow"))
    par.MaxDistanceSurface() = args->Double("-maxdistgrow", 0.3);
  if (args->Contains("-mindistrecompute"))
    par.MinDistanceRecompute() = args->Double("-mindistrecompute", 0.15);
  if (args->Contains("-compete")) par.SurfacesCompete() = true;

  // Segment the data
  growsurfaces(args->String("-f"), args->String("-i"), args->String("-app"),
               args->String("-odir"), args->Contains("-meta"),
               args->Contains("-overwrite"), args->String("-t"), par); 

  // Save segmentation parameters
  if (args->Contains("-savepar"))
    par.Write(args->String("-savepar", "segmentlaser.par"));

  return EXIT_SUCCESS;
}
