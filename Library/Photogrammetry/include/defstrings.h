
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



#ifndef _DEFSTRINGS_H_
#define _DEFSTRINGS_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Defining the strings/keywords in the database files
*
*--------------------------------------------------------------------*/


/* string definitions for output, primitive info and parameter info */
	
#define DD						": "
#define END						"end"
#define POSITION_STR			"position"
#define X_STR					"x"
#define Y_STR					"y"
#define Z_STR					"z"
#define RX_STR					"rx"
#define RY_STR					"ry"
#define RZ_STR					"rz"
#define ROTATION_STR			"rotation"
#define R11_STR					"r11"
#define R12_STR					"r12"
#define R13_STR					"r13"
#define R21_STR					"r21"
#define R22_STR					"r22"
#define R23_STR					"r23"
#define R31_STR					"r31"
#define R32_STR					"r32"
#define R33_STR					"r33"
#define Q0_STR					"q0"
#define Q1_STR					"q1"
#define Q2_STR					"q2"
#define Q3_STR					"q3"
#define T0_STR					"t0"
#define T1_STR					"t1"
#define T2_STR					"t2"
#define ORIENTATION_STR			"orientation"
#define CAMERA_STR				"interior"
#define CAMERANAME_STR			"camera"
#define FOCAL_STR				"focal"
#define PIXELSIZE_R_STR			"pixelsize_r"
#define PIXELSIZE_C_STR			"pixelsize_c"
#define PRINCIPAL_R_STR			"principal_r"
#define PRINCIPAL_C_STR			"principal_c"
#define ROWS_STR				"rows"
#define COLUMNS_STR				"columns"
#define ROTATE_STR				"rotate"
#define SHEAR_STR				"shear"
#define K1_STR					"k1"
#define K2_STR					"k2"
#define K3_STR					"k3"
#define P1_STR					"p1"
#define P2_STR					"p2"
#define PHOTO_STR				"photo"
#define FILENAME_STR			"filename"
#define THUMBNAIL_STR			"thumbnail"
#define RECORDING_STR			"recording"
#define CYLINDER_STR			"cylinder"
#define TORUS_STR				"torus"
#define BOX_STR					"box"
#define SPHERE_STR				"sphere"
#define CONE_STR				"cone"
#define WEDGE_STR				"wedge"
#define PRISM_STR				"prism"
#define RADIUS_STR				"radius"
#define RADIUS2_STR				"radius2"
#define LENGTH_STR				"length"
#define ANGLE_STR				"angle"
#define SIZE_X_STR				"size_x"
#define SIZE_Y_STR				"size_y"
#define SIZE_Z_STR				"size_z"
#define TOP_X_STR				"top_x"
#define ID_STR					"id"
#define PIPE_END_STR			"pipe_end"
#define HEADER_STR				"header"
#define DATA_DIR_STR			"data_directory"
#define TEMP_DIR_STR			"temp_directory"
#define MMAP_IMAGES_STR			"mmap_images"
#define MMAP_IMAGE_HEADERS_STR	"mmap_image_headers"
#define PROJ_NAME_STR			"projectname"
#define VERSION_STR				"version"
#define ROT_REPR_STR			"rot_repr"
#define VECTOR_STR				"vector"
#define QUATERNION_STR			"quaternion"
#define CATALOG_STR				"catalog"
#define CAD_MODEL_STR			"cad_model"
#define CSG_TREE_STR			"csg_tree"
#define CSG_NODE_STR			"csg_node"
#define BOOL_OPER_STR			"bool_oper"
#define NAME_STR				"name"
#define NODE_ID_STR				"node_id"
#define INDEX_STR				"index"
#define AXIS_STR				"axis"
#define VALUE_STR				"value"
#define MEASUREMENTS_STR		"measurements"
#define MEASURED_MODEL_STR		"measured_model"
#define PHOTO_POINTS_STR		"photo_points"
#define PRIMITIVE_POINT_STR		"primitive_point"
#define CONTOUR_POINT_STR		"contour_point"
#define INTERSECTION_POINT_STR	"intersection_point"
#define NODE_INDEX_STR			"node_index"

// constraint vocabulaire
#define PARAMETER_CONSTRAINT_STR		"parameter_constraint"
#define ROTATION_CONSTRAINT_STR			"rotation_constraint"
#define TRANSLATION_CONSTRAINT_STR		"translation_constraint"
#define POSITION_CONSTRAINT_STR			"position_constraint"
#define DIRECTION_CONSTRAINT_STR		"direction_constraint"
#define CONNECTION_CONSTRAINT_STR		"connection_constraint"
#define WEIGHT_STR						"weight"
#define SIGN_STR						"sign"
#define SIGN1_STR						"sign1"
#define SIGN2_STR						"sign2"
#define FACTOR1_STR						"factor1"
#define FACTOR2_STR						"factor2"
#define FACTOR3_STR						"factor3"
#define OFFSET_STR						"offset"
#define PARAMETER1_STR					"parameter1"
#define PARAMETER2_STR					"parameter2"
#define PARAMETER3_STR					"parameter3"
#define ANGLE_PAR1_STR					"angle_par1"
#define ANGLE_PAR2_STR					"angle_par2"
#define RADIUS_PAR1_STR					"radius_par1"
#define RADIUS_PAR2_STR					"radius_par2"
#define NODE_AXIS1_STR					"node_axis1"
#define NODE_AXIS2_STR					"node_axis2"
#define NODE_AXIS3_STR					"node_axis3"
#define NODE_TRANS1_STR					"node_trans1"
#define NODE_TRANS2_STR					"node_trans2"
#define NODE_ROTAT1_STR					"node_rotat1"
#define NODE_ROTAT2_STR					"node_rotat2"
#define NODE_CONNECT1_STR				"node_connect1"
#define NODE_CONNECT2_STR				"node_connect2"
#define CONNECTION_STR					"connection"
#define COLOR_STR						"color"
#define RED_STR							"red"
#define GREEN_STR						"green"
#define BLUE_STR						"blue"

/// determinating a plus from a minus.
#define PLUS							'+'
#define MINUS							'-'

/// for use in entity attributes only
#define EDGE_POINT_STR					"edge_point"
#define NODE_INT_STR					"node_int"


#endif
