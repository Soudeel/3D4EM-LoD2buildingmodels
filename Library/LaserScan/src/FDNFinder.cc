
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : October 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Defines a space partitioning scheme for 3D points.
*				Uses a simple cubic partition, but keeps data in a map
*				to exploit the sparsity of data. Each box in space partition
*				is represented by PartitionBox, and its data is given by
*				PartitioinBoxData.
*
*--------------------------------------------------------------------*/
#include "PointsSpacePartition.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "FDNFinder.h"

//signed short should be big enough to contain our partition indices.
typedef signed short PartitionType;
typedef PartitionBox<PartitionType> PBox;
typedef PointsSpacePartition<PartitionType,LaserPoints> LaserPointsSpacePartition;

typedef FDNFinder<LaserPoint> LaserPointFDNFinder;

int main_findfdn(int argc,char** argv)
{
	char Usage[] = "find the neighbours within a specified distance and write to a laserfile.fdn\n"
	"findfnd laser_name distance [box_count]\n"
	"Results are write to laser_name.fdn with one line per point\n";
	if(argc<3)
	{
		cerr<<Usage;
		exit(1);
	}
	char* laser_name = argv[1];
	double fdn_distance = atof(argv[2]);
	
	int neighbour_boxes = 3;
	if(argc>3)
		neighbour_boxes = atof(argv[3]);
	
	double partition_step = fdn_distance/(neighbour_boxes);
	
	LaserPoints laserPoints;
	
	laserPoints.SetPointFile(laser_name);
	laserPoints.Read();
	laserPoints.DeriveDataBounds(0);
	
	char buff[4096];
	sprintf(buff,"%s.fdn",laser_name);
	FILE* pResultFile = fopen(buff,"wt");
	
	LaserPointsSpacePartition partition(&laserPoints,1000);
	partition.UpdatePartition(partition_step);
	partition.Print();
	fprintf(stderr,"\nCalculating fdns...");
	partition.WriteFdn(pResultFile,fdn_distance,neighbour_boxes);
	
	fprintf(stderr,"Done\n");	
	fclose(pResultFile);
	
	return 0;
	
}
