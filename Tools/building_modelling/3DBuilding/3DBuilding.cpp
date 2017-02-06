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

#include <stdio.h>
#include "InlineArguments.h"
#include "ModelBldWithAnchorPnt.h"
#include "PolylineGeneralization.h"
#include "TopologyGraph.h"

using namespace std;

void PrintUsage();

//-il 1.laser -ip 1.objpts -it 1.top -op 1_bld.objpts -ot 1_bld.top
int main(int argc, char *argv[])
{
	InlineArguments args = InlineArguments(argc, argv);

	// Check on required input files
	if (args.Contains("-usage") ||
		!args.Contains("-il") ||
		!args.Contains("-ip") ||
		!args.Contains("-it") ||
		!args.Contains("-op") ||
		!args.Contains("-ot")) 
	{
		if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
		PrintUsage();
		return EXIT_SUCCESS;
	}

	//ModelBldWithTinThining modeler(&laser_points);
	LineTopologies footTops;
	ObjectPoints footPnts;
	LaserPoints laser_points;
	Buildings buildings;
	ObjectPoints model_points;
	if (!footTops.Read(args.String("-it")) || 
		!footPnts.Read(args.String("-ip")) || 
		!laser_points.Read(args.String("-il")))
	{
		return 1;
	}
	
	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(laser_points, vecSegPnls, SegmentNumberTag);
	
	BldRecPar bldRecPar = GetIEGlobelVari();
	bldRecPar.bFootPrintHei = true;//use original height
	UpdataIEGlobelVari(bldRecPar);
	
	std::vector<PointNumberList> vecSegs;
	MyDeriveSegPNL(laser_points,vecSegs);
	TopologyGraph graph();

	//ModelBldWithTinThining* pModeler;
	ModelBldWithAnchorPnt* pModeler;

	if (footTops.empty()) {
		pModeler = new ModelBldWithAnchorPnt(&laser_points);
	}
	else {
		pModeler = new ModelBldWithAnchorPnt(&laser_points, footTops, footPnts, false);
	}

	if (pModeler) {
		pModeler->DoModel();
		pModeler->GetPcmModel(buildings, model_points);	
		buildings.WriteModelData(args.String("-ot"));
		model_points.Write(args.String("-op"));
		laser_points.Write(args.String("-il"));
	}

	return EXIT_SUCCESS;
}

void PrintUsage()
{
	printf("Usage: 3dbuilding -il <laser points>\n");
	printf("                 -ip <input map points (2D map)>\n");
	printf("                 -it <input map topology>\n");
	printf("                 -ot <output points (3D model)>\n");
	printf("                 -op <output map topology>\n");
}

