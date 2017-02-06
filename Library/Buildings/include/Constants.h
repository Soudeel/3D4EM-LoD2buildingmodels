
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



#ifndef _Constants_h_
#define _Constants_h_

// Height constraint used in the matching
#define HEIGHT1	  	3
#define HEIGHT2   	15

#define MIN_DIST_MAP_POINTS	0.2
#define MIN_DIST_POINTS		0.2

#define MIN_LENGTH_ROOF		2
#define SCORE_WORST		-10000

#define DIL_DIST   10	// dilatation distance

// parameters of feature extraction
#define THRESH		5000	// highest threshold used to extract features 
#define LOW_THRESH	1000	// lowest threshold used to extract features
#define WIN_SIZE_PT	5	// window size of interesting operator 
#define WIN_SIZE_LIN	3	// window size used at line extraction
#define Q_THRESH	0.8 	// roundness threshold
#define LOCAL_MAX	9	// window size for local max selection
#define MIN_LENGTH	10	// minimum required line length
#define MAX_WIDTH	3	// maximum width of a line area

//#define ERR_PARALLEL_LINES	0.10	// degree 

#define ERR_PARALLEL_SEG	0.26	// ~15 
#define ERR_COLLINEAR_SEG1	0.2	// value used at the corners generation
#define ERR_COLLINEAR_SEG2	0.5	// value used at the graph building
#define ERR_COLLINEAR_SEG3	0.1	// value used at corner list updating
					// to check if it is the same segment 
#define ERR_EQ_POINTS		1	// test if the interest points are equal	

#define WIN_SIZE_CORNER		7	// check if a line and a point form a corner  

#define ERR_PAR_SEG	0.5	// graph building 
#define ERR_COL_SEG	0.5	// graph building
#define GAP		1	// admitted gap between collinear segments which allows
				// that the segments to be connected together
#define ARC_LENGTH	0.8	// part of the arc which has to be filled by
				// segments that the arc to be considered real
#define ERR_CONTOUR	0.5	// 2D graph updating - det new contour				
				
#define ERR_PAR_PLANES	0.20	// ~10 - test mergeability of 2 planes

#define ERR_MAP_CORNER	2	// label corner points - distance between a
				// reconstructed corner point and a map corner  


#define X_OFF	205500		// VRML
#define Y_OFF	489100

#define PROB1	0.3	// probabilities for defining house models
#define PROB2	0.5
#define PROB3	0.7

// default values of the height of the roof for incomplet house models
#define DEF_HIP		4
#define DEF_GABLE	3

#define HEIGHT_DIFF	3	// admitted height difference for 
				// horizontal constraint
/////////////////////////////////////////////////////////////////////////

#define PI       4.0 * atan(1.0)

#define ERR_PARALLEL_LINES	5 * PI / 180	// degrees      
#define ERR_COLLINEAR_LINES	1		// distance

#define UNCERTAINTY_REL_ORIENT 	10	// used to test epipolar constraint

#define ERR_POINT_ON_LINE	2

#endif
