
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class ObjectPoints          


 Initial creation
 Author : Ildi Suveg
 Date   : 24-11-1998

 Update #1
 Author : George Vosselman
 Date   : 08-06-1999
 Changes: Added VRML functions
 
 Update #2
 Author : Sander Oude Elberink
 Data   : 16-03-2006
 Changes: Added RGB options to VRML2_Write
 
 Update #3
 Author : Adam Patrick Nyaruhuma
 Data   : 31-03-2009
 Changes: PointVisible method

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <math.h>
#include "ObjectPoints.h"
#include "ObjectPoints2D.h"
#include "CameraPoints.h"
#include "ModelPoints.h"
#include "InteriorOrientation.h"
#include "ExteriorOrientation.h"
#include "AbsoluteOrientation.h"
#include "TIN.h"
#include "VRML_io.h"
#include "dxf.h"
#include "LineSegment2D.h"
/*
--------------------------------------------------------------------------------
                     Declarations of old DigPhot functions in C
--------------------------------------------------------------------------------
*/

extern "C" void lforward(CamPts *, CamPts *, Interior *, Interior *, 
    		Exterior *, Exterior *, ObjPts *);

extern "C" void lmodel2obj(ModelPts *, Absolute *, ObjPts *);

/*
--------------------------------------------------------------------------------
                     Construct from a 2D object point
--------------------------------------------------------------------------------
*/

ObjectPoints::ObjectPoints(const ObjectPoints2D *points2D, double z)
  : VectorPoint<ObjectPoint>()
{
  ObjectPoints2D::const_iterator i;
  ObjectPoint point; 
  for(i = points2D->begin(); i != points2D->end(); i++)
  {
     point = ObjectPoint(i->ObjectPoint2DPtr(), z);
     push_back(point);
  } 
}     


/*
--------------------------------------------------------------------------------
	 	Construct object points by forward intersection  
--------------------------------------------------------------------------------
*/

ObjectPoints::ObjectPoints(const CameraPoints *campoints1, const CameraPoints *campoints2,
    		const InteriorOrientation *intor1, const InteriorOrientation *intor2, 
    		const ExteriorOrientation *extor1, const ExteriorOrientation *extor2)
  : VectorPoint<ObjectPoint>()
{
  Interior *int1 = (Interior*)malloc(sizeof(Interior));
  intor1->Cpp2C(&int1);	

  Interior *int2 = (Interior*)malloc(sizeof(Interior));
  intor2->Cpp2C(&int2);	

  Exterior *ext1 = (Exterior*)malloc(sizeof(Exterior));
  extor1->Cpp2C(&ext1);	

  Exterior *ext2 = (Exterior*)malloc(sizeof(Exterior));
  extor2->Cpp2C(&ext2);	

  CamPts *campts1;	
  campts1 = NULL;
  campoints1->Cpp2C(&campts1);

  CamPts *campts2;	
  campts2 = NULL;
  campoints2->Cpp2C(&campts2);
  
  ObjPts *objpts = (ObjPts *)malloc(sizeof(ObjPts));
  objpts->num_pts = 0;
  for(int i = 0; i < campoints1->size(); i++)
     for(int j = 0; j < campoints2->size(); j++)
       if (campoints1[0][i].Number() == campoints2[0][j].Number())
            objpts->num_pts++;
  objpts->pts = (ObjPt *)calloc(objpts->num_pts, sizeof(ObjPt));

  // call library routine
  lforward(campts1, campts2, int1, int2, ext1, ext2, objpts);

  C2Cpp(objpts);

/* ???????????????
  free(int1);
  free(int2);
  free(ext1);
  free(ext2);
  free(campts1);
  free(campts2);
  free(objpts);  
*/
}


/*
--------------------------------------------------------------------------------
	 	Construct object points by transforming model points
--------------------------------------------------------------------------------
*/

ObjectPoints::ObjectPoints(const ModelPoints *points, const AbsoluteOrientation *absor)
  : VectorPoint<ObjectPoint>()
{
  Absolute *abso = (Absolute*)malloc(sizeof(Absolute));
  absor->Cpp2C(&abso);	

  ModelPts *pts;	
  pts = NULL;
  points->Cpp2C(&pts);

  ObjPts *objpts = (ObjPts *)malloc(sizeof(ObjPts));
  objpts->num_pts = pts->num_pts;
  objpts->pts = (ObjPt *)calloc(objpts->num_pts, sizeof(ObjPt));

  // call library routine
  lmodel2obj(pts, abso, objpts);

  C2Cpp(objpts);

  free(abso);
  Free_ModelPts(pts);
  Free_ObjPts(objpts);  
}

/*
--------------------------------------------------------------------------------
	 	Construct object points from 3D positions and a point number
--------------------------------------------------------------------------------
*/

ObjectPoints::ObjectPoints(const Positions3D &positions, const PointNumber &number)
  : VectorPoint<ObjectPoint>()
{
  int                         num = number.Number();
  ObjectPoint                 point;
  Positions3D::const_iterator position;
  
  for (position=positions.begin(); position!=positions.end(); position++) {
    point.Number() = num;
    point.vect() = position->vect();
    push_back(point);
    num++;
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void ObjectPoints::Cpp2C(ObjPts **objptsptr) const
{
  ObjPts *objpts;

/* Allocate space if this has not been done yet */

  objpts = *objptsptr;
  if (objpts == NULL) {
    objpts = (ObjPts *) malloc(sizeof(ObjPts));
    objpts->pts = (ObjPt*) malloc(size() * sizeof(ObjPt));
    *objptsptr = objpts;
  }

/* Copy the data */

  objpts->num_pts = size();
  ObjectPoints::const_iterator i;
  ObjPt *pt; 
  int l;
  for(i = begin(), l = 0; i != end(); i++, l++)
  {
	pt = &objpts->pts[l];
        i->Cpp2C(&pt);
  }
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void ObjectPoints::C2Cpp(ObjPts *objpts)
{
  if (!empty()) erase(begin(), end());
  reserve(objpts->num_pts);
  	
  ObjectPoint point; 	
  for(int i = 0 ; i < objpts->num_pts; i++)
  {
  	point.C2Cpp(&objpts->pts[i]);
  	push_back(point);
  }
}

/*
--------------------------------------------------------------------------------
                       Read Object points from a database
--------------------------------------------------------------------------------
*/
int ObjectPoints::ReadC(const char *filename)
{
  ObjPts *objpts;

  objpts = Get_ObjPts(filename);        /* Read the database into C structure */
  if (objpts == NULL) return(0);
  C2Cpp(objpts);                        /* Convert to C++ object */
  Free_ObjPts(objpts);
  return(1);
}

int ObjectPoints::Read(const char *filename)
{
  FILE *fp;
  char line[MAXCHARS];
  int  file_id, num;
  double x, y, z, vx, vy, vz, cvxy, cvxz, cvyz;

  if ((fp = Open_Compressed_File(filename, "r")) == NULL) {
    fprintf( stderr, "Could not open database file %s\n", filename);
    return(0);
  }
  
  fgets(line, MAXCHARS, fp);
  sscanf(line, "%d", &file_id);
  if (file_id != OBJECT_POINTS) {
    printf("file identification error, this file ( %s ) does not contain \
object points\n", filename);
    return(0);
  }

  while (!feof(fp)) {
    fgets(line, MAXCHARS, fp);
    if (!Is_Comment(line) && !feof(fp)) {
      sscanf(line, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &num, &x, &y, &z, &vx, &vy, &vz, &cvxy, &cvxz, &cvyz);
      push_back(ObjectPoint(x, y , z, num, vx, vy, vz, cvxy, cvxz, cvyz));
    }
  }  
    
  fclose(fp);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write Object points to a database
--------------------------------------------------------------------------------
*/
int ObjectPoints::WriteC(const char *filename) const 
{
  int    error;
  ObjPts *objpts;

  objpts = NULL;
  Cpp2C(&objpts);
  error = Put_ObjPts(objpts, filename);
  Free_ObjPts(objpts);
  return(error);
}

int ObjectPoints::Write(const char *filename) 
{
  FILE *fp;
  ObjectPoints::iterator objpt;
  
  if ((fp = fopen(filename, "w")) == NULL) {
    fprintf(stderr, "Could not open %s to write object point database.\n", 
            filename);
    return(0);
  }

  Sort(); // on point number

  // Header
  fprintf(fp, "%d # file identifier\n", OBJECT_POINTS);
  fprintf(fp, "# This file contains %d object points\n", size());
  fprintf(fp, "#                       coordinates                                   variances                                    covariances\n");
  fprintf(fp, "# point        x                y              z              x              y              z               xy             xz             yz\n");
  fprintf(fp, "#-----------------------------------------------------------------------------------------------------------------------------------------------\n");
  
  // Point output
  for (objpt=begin(); objpt!=end(); objpt++)
    fprintf(fp, " %4d%17.7lf%17.7lf%15.7lf%16.7le%15.7le%15.7le%16.7le%15.7le%15.7le\n",
	        objpt->Number(), objpt-> X(), objpt->Y(), objpt->Z(),
	        objpt->Covar(0), objpt->Covar(1), objpt->Covar(2),
	        objpt->Covar(3), objpt->Covar(4), objpt->Covar(5));
    
  fclose(fp);
  return(1);
}

/*
--------------------------------------------------------------------------------
                        Write Object points(without coviance) to a database
--------------------------------------------------------------------------------
*/
int ObjectPoints::Write2(char *filename) const 
{
  int    error;
  ObjPts *objpts;

  objpts = NULL;
  Cpp2C(&objpts);
  error = Put_ObjPts2(objpts, filename);
  Free_ObjPts(objpts);
  return(error);
}
/*
--------------------------------------------------------------------------------
                        Print Object points to stdout
--------------------------------------------------------------------------------
*/
void ObjectPoints::Print() const
{
  ObjPts *objpts;

  objpts = NULL;
  Cpp2C(&objpts);
  Print_ObjPts(objpts);
  Free_ObjPts(objpts);
}

/*
--------------------------------------------------------------------------------
                 Read object points and topologies from a DXF file
--------------------------------------------------------------------------------
*/

#define DXF_NO_LABEL        1
#define DXF_LABEL_IN_COLOUR 2
#define DXF_LABEL_IN_LAYER  3

int SaveDXFObject(int select_on_colour, int wanted_colour, int current_colour,
                  int select_on_layer, int wanted_layer, int current_layer)
{
  if (select_on_colour && current_colour != wanted_colour) return(0);
  if (select_on_layer && current_layer != wanted_layer) return(0);
  return(1);
}

int ObjectPoints::ReadDXF(char *dxf_file, LineTopologies &top,
                          int label_location,
                          int select_on_colour, int wanted_colour,
                          int select_on_layer, int wanted_layer,
                          bool loose_points, bool ignore_zeros,
						  bool force_polygon_closure)
{
  FILE         *dxf;
  int          group, busy_vertex=0, busy_polyline=0, busy_line=0,
               close_polygon=0, current_colour, current_layer, stored_X=0,
               first_just_point=1, debug=0, num_records;
  char         line[200];
  ObjectPoint  point, point2, *first_point, *last_point;
  LineTopology polygon;

  ObjectPoints just_points;
  ObjectPoint  just_point;
  LineTopologies no_lines;

// Open DXF file

  if (!(dxf = fopen(dxf_file, "r"))) { 
    printf("Error opening DXF file %s\n", dxf_file);
    return 0;
  }

// Read the first two lines and start processing

  fscanf(dxf, "%d\n", &group);
  fgets(line, 200, dxf);
  num_records = 2;
  while (!feof(dxf)) {

    switch (group) {
      case DXFGRP_START:
        if (busy_vertex) { 
          if (ignore_zeros && point.X() == 0.0 && point.Y() == 0.0 &&
		      point.Z() == 0.0) {
          	if (busy_polyline) polygon.erase(polygon.end()-1);
          }
          if (SaveDXFObject(select_on_colour, wanted_colour, current_colour,
                            select_on_layer, wanted_layer, current_layer))
            push_back(point);
          busy_vertex = 0;
        }
        if (strncmp(line, "VERTEX", 6) == 0 ||
            (loose_points && strncmp(line, "POINT", 5))) { // New vertex
          busy_vertex = 1;
          point.X() = point.Y() = point.Z() = 0.0;
          point.Number() = size();
          if (busy_polyline) polygon.push_back(point.Number());
          stored_X = 0;
        }
        else {
          if (busy_polyline) {
            if (close_polygon || force_polygon_closure) { // Close current polyline if not already closed
              first_point = GetPoint(polygon.begin()->NumberRef());
              last_point  = GetPoint((polygon.end()-1)->NumberRef());
              if (first_point->X() != last_point->X() ||
                  first_point->Y() != last_point->Y() ||
                  first_point->Z() != last_point->Z())
                polygon.push_back(*polygon.begin());
            }
            if (SaveDXFObject(select_on_colour, wanted_colour, current_colour,
                              select_on_layer, wanted_layer, current_layer))
              top.push_back(polygon);
            polygon.erase(polygon.begin(), polygon.end());
            busy_polyline = 0;
          }
          if (busy_line) { // Close current line
            if (SaveDXFObject(select_on_colour, wanted_colour, current_colour,
                              select_on_layer, wanted_layer, current_layer)) {
              push_back(point);
              push_back(point2);
            }
            polygon.erase(polygon.begin(), polygon.end());
            busy_line = 0;
          }
          if (strncmp(line, "POLYLINE", 8) == 0 ||
              strncmp(line, "LWPOLYLINE", 10) == 0) { // New polyline
            polygon.Number() = top.size();
            polygon.Label() = 0;
            busy_polyline = 1;
            close_polygon = 0;
          }
          if (strncmp(line, "LINE", 4) == 0) { // New line
            polygon.Number() = top.size();
            polygon.Label() = 0;
            point.X() = point.Y() = point.Z() = 0.0;
            point.Number() = size();
            point2.X() = point2.Y() = point2.Z() = 0.0;
            point2.Number() = size() + 1;
            polygon.push_back(point.Number());
            polygon.push_back(point2.Number());
            if (SaveDXFObject(select_on_colour, wanted_colour, current_colour,
                              select_on_layer, wanted_layer, current_layer))
              top.push_back(polygon);
            busy_line = 1;
          }
        }
        break;

      case DXFGRP_X_1:
        if (first_just_point) {
          just_point.Number() = 0;
          first_just_point = 0;
        }
        else {
          just_points.push_back(just_point);
          just_point.Number()++;
        }
        // Sometimes the VERTEX command is missing...
        if (stored_X) { // Save old vertex
          if (SaveDXFObject(select_on_colour, wanted_colour, current_colour,
                            select_on_layer, wanted_layer, current_layer))
            push_back(point);
          busy_vertex = 0;
        }
        // Enforce new vertex
        if (busy_polyline && !busy_vertex) {
          busy_vertex = 1;
          point.Y() = point.Z() = 0.0;
          point.Number() = size();
          polygon.push_back(point.Number());
        }
        sscanf(line, "%lf", &just_point.X());
        if (busy_vertex || busy_line) {
          point.X() = just_point.X();
          stored_X = 1;
        }
        break;

      case DXFGRP_Y_1:
        sscanf(line, "%lf", &just_point.Y());
        if (busy_vertex || busy_line) point.Y() = just_point.Y();
        break;

      case DXFGRP_Z_1:
        sscanf(line, "%lf", &just_point.Z());
        if (busy_vertex || busy_line) point.Z() = just_point.Z();
        break;
          
      case DXFGRP_X_2:
        if (busy_line) sscanf(line, "%lf", &point2.X());
        break;

      case DXFGRP_Y_2:
        if (busy_line) sscanf(line, "%lf", &point2.Y());
        break;

      case DXFGRP_Z_2:
        if (busy_line) sscanf(line, "%lf", &point2.Z());
        break;
          
      case DXFGRP_INT_1:
        if (busy_polyline && !busy_vertex) sscanf(line, "%d", &close_polygon);
        break;
          
      case DXFGRP_LAYER_NAME:
        if (label_location == DXF_LABEL_IN_LAYER &&
            (busy_line || (busy_polyline && !busy_vertex))) {
          sscanf(line, "%d", &polygon.Label());
          current_layer = polygon.Label();
        }
        break;
          
      case DXFGRP_COLOUR:
        if (label_location == DXF_LABEL_IN_COLOUR &&
            (busy_line || (busy_polyline && !busy_vertex))) {
          sscanf(line, "%d", &polygon.Label());
          current_colour = polygon.Label();
        }
        break;
    }
   
    fscanf(dxf, "%d\n", &group);
    fgets(line, 200, dxf);
    num_records += 2;
    printf("POLYLINE %7d record %8d\r", top.size(), num_records);
  }
  
// Remove points with identical coordinates but different numbers

  RemoveDoublePoints(top, 0.0);

// Close DXF file and return success

  fclose(dxf);

  if (debug) {
    just_points.RemoveDoublePoints(no_lines, 0.0);
    just_points.Write("pts_only.objpts");
  }
  return 1;
}

/*
--------------------------------------------------------------------------------
              Write object points to a VRML file as points or crosses
--------------------------------------------------------------------------------
*/

void ObjectPoints::VRML_Write(FILE *fd) const
{
  ObjectPoints::const_iterator point;

  VRML_Write_Begin_Of_PointList(fd);
  for (point=begin(); point!=end(); point++)
    point->Position3D::VRML_Write(fd);
  VRML_Write_End_Of_PointList(fd);
}

void ObjectPoints::VRML_Write_Crosses(FILE *fd, double size) const
{
  ObjectPoints::const_iterator point;

  for (point=begin(); point!=end(); point++)
    point->VRML_Write_Cross(fd, size);
}

/*
--------------------------------------------------------------------------------
              Write lines as cylinders to a VRML file
--------------------------------------------------------------------------------
*/

void ObjectPoints::VRML_Write_Cylinders(FILE *fd, const LineTopologies & lines,
                                        double radius) const
{
  LineTopologies::const_iterator line;
  LineTopology::const_iterator pn;
  ObjectPoint *point1, *point2;
  Position3D  pos1, pos2;
  
  for (line=lines.begin(); line!=lines.end(); line++) {
    if (line->size() > 1) {
      point1 = GetPoint(*(line->begin()));
      for (pn=line->begin()+1; pn!=line->end(); pn++) {
        point2 = GetPoint(*pn);
        if (point1 && point2) {
          pos1 = *point1;
          pos2 = *point2;
          VRML_Write_Cylinder(fd, pos1, pos2, radius);
        }
        point1 = point2;
      }
    } 
  }
}

bool ComparePointNumbers2(const ObjectPoint& p1, const ObjectPoint& p2)
{
	return(p1.Number()<p2.Number());
}

/*
--------------------------------------------------------------------------------
              Remove points with nearly the same coordinates
--------------------------------------------------------------------------------
*/

void ObjectPoints::RemoveDoublePoints(LineTopologies &polygons, double min_dist, bool b2D)
{
LineTopologies::iterator polygon;
	LineTopology::iterator node;
	ObjectPoints::iterator   point1, point2;
	int                      double_number, new_number, old_number, count;

	// Sort on coordinates
	SortOnCoordinates();

	// Mark double points with point highest point number + 1
	double_number = HighestPointNumber().Number() + 1;
	count = 0;

	if (!b2D) {
		for (point1=begin(); point1!=end(); point1++) {
			if (point1->Number() == double_number) continue;
			for (point2=point1+1; point2!=end(); point2++) {
				if (fabs(point2->X()-point1->X()) > min_dist) continue;
				if (point2->Number() == double_number) continue;
				if (point1->Distance(point2->Position3DRef()) <= min_dist) {
					count++;
					new_number = point1->Number();
					old_number = point2->Number();
					for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++)
						for (node=polygon->begin(); node!=polygon->end(); node++)
							if (node->Number() == old_number) 
								node->Number() = new_number;
					point2->Number() = double_number;
				}
			}
		}
	}
	else {
		double dist;
		double dx, dy;
		double maxX, minX;
		for (point1=begin(); point1!=end(); point1++) {
			if (point1->Number() == double_number) continue;
			minX = point1->X()-min_dist;
			maxX = point1->X()+min_dist;
			for (point2=point1+1; point2!=end(); point2++) {
				//if (fabs(point2->X()-point1->X()) > min_dist) continue;
				if (point2->X()<minX || point2->X()>maxX) continue;
				if (point2->Number() == double_number) continue;
				dx = point1->X()-point2->X();
				dy = point1->Y()-point2->Y();
				dist = sqrt(dx*dx+dy*dy);
				if (dist <= min_dist) {
					count++;
					new_number = point1->Number();
					old_number = point2->Number();
					for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++)
						for (node=polygon->begin(); node!=polygon->end(); node++)
							if (node->Number() == old_number) 
								node->Number() = new_number;
					point2->Number() = double_number;
				}
			}
		}
	}


	// Sort on point numbers
	//Sort();
	std::sort(begin(), end(), ComparePointNumbers2);

	// Erase all double points
	erase(end()-count, end());
}
/*
--------------------------------------------------------------------------------
              Average all coordinates of points with nearly the same coordinates
--------------------------------------------------------------------------------
*/

void ObjectPoints::AverageDoublePoints(LineTopologies &polygons, double min_dist, double min_height_dist)
{
  LineTopologies::iterator polygon;
  LineTopology::iterator node;
  ObjectPoints::iterator   point1, point2;
  int                      double_number, new_number, old_number, count;
  
  // Sort on coordinates
  SortOnCoordinates();
  
  // Mark double points with point highest point number + 1
  double_number = HighestPointNumber().Number() + 1;
  count = 0;
  for (point1=begin(); point1!=end(); point1++) {
      for (point2=point1+1;
           point2!=end() && fabs(point2->X() - point1->X()) <= min_dist;
           point2++) {
          if (fabs(point2->Y() - point1->Y()) <= min_dist) {//;point1->Distance(point2->Position3DRef()) <= min_dist) {
             if (fabs(point1->Z() - point2->Z()) <=min_height_dist) {
               count++;
               point1->X() = (point1->X() + point2->X())/2;
               point1->Y() = (point1->Y() + point2->Y())/2;
               point1->Z() = (point1->Z() + point2->Z())/2;
               point2->X() = point1->X();
               point2->Y() = point1->Y();               
               point2->Z() = point1->Z();
               
             }
          }
      }
  }  
}


/// Remove unreferenced points. The point will be sorted according to the numbers
/// Both Points and lines will be renumbered from 0
void ObjectPoints::RemoveUnusedPnts(LineTopologies &lines)
{
	LineTopologies::iterator itrTop;
	vector<int> vecValidNum = vector<int>(this->size(), -1);
	ObjectPoints::iterator curPntItr;
	int index;

	//check valid points
	for(int i=0; i<lines.size(); ++i) {
		itrTop = lines.begin()+i;

		for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
			index = this->FindPoint((*itrTop)[iPnt]);
			if (index == -1) continue;
			vecValidNum[index] = 1;
		}
	}

	//compute the new number
	int validPntCount = 0;
	for (int iPnt=0; iPnt<vecValidNum.size(); ++iPnt) {
		if (vecValidNum[iPnt]==-1) continue;
		vecValidNum[iPnt] = validPntCount;
		validPntCount++;
	}

	int temNum;
	//renumber line topology
	//check valid points
	for(int i=0; i<lines.size(); ++i) {
		itrTop = lines.begin()+i;

		for (int iPnt=0; iPnt<itrTop->size(); ++iPnt) {
			index = this->FindPoint((*itrTop)[iPnt]);
			if (index == -1) continue;
			(*itrTop)[iPnt].Number() = vecValidNum[index];
		}
	}

	ObjectPoints temObjPnts;
	ObjectPoint temObjPnt;
	for (int iPnt=0; iPnt<this->size(); ++iPnt) {
		if (vecValidNum[iPnt] ==-1) continue;
		temObjPnt = (*this)[iPnt];
		temObjPnt.Number() = vecValidNum[iPnt];
		temObjPnts.push_back(temObjPnt);
	}

//	temObjPnts.SortOnCoordinates();
	std::swap(temObjPnts, *this);
}

/*
--------------------------------------------------------------------------------
              Triangulate object points with constraints and holes
--------------------------------------------------------------------------------
*/

TIN & ObjectPoints::Triangulate(const LineTopologies &bounds,
                                const ObjectPoints &holes) const
{
  double *coordinate_list, *coordinate, *hole_list;
  int    *edge_list, *edge_point, num_edges, remove, i_edge, i_edge2,
         *edge_point2, num_edges_org;
  ObjectPoints::const_iterator point;
  LineTopologies::const_iterator polygon;
  LineTopology::const_iterator node;
  TIN *tin;

// Create the coordinate list

  coordinate_list = coordinate = (double *) malloc(2 * sizeof(double) * size());
  for (point=begin(); point!=end(); point++) {
    *coordinate++ = point->X();
    *coordinate++ = point->Y();
  }

// Create the edge list

  for (polygon=bounds.begin(), num_edges=0; polygon!=bounds.end(); polygon++)
    num_edges += polygon->size() - 1;
  if (num_edges) {
    // Make the initial edge list
    edge_list = edge_point = (int *) malloc(num_edges * 2 * sizeof(int));
    for (polygon=bounds.begin(); polygon!=bounds.end(); polygon++) {
      for (node=polygon->begin(); node!=polygon->end()-1; node++) {
        if (node->Number() < (node+1)->Number()) {
          *edge_point++ = node->Number();
          *edge_point++ = (node+1)->Number();
        }
        else {
          *edge_point++ = (node+1)->Number();
          *edge_point++ = node->Number();
        }
      }
    }
    // Remove edges that appear twice: first replace point numbers by -1,
    // then remove point numbers -1
    for (i_edge=0, edge_point=edge_list; i_edge<num_edges-1;
         i_edge++, edge_point+=2) {
      if (*edge_point != -1) {
        for (i_edge2=i_edge+1, edge_point2=edge_point+2, remove=0;
             i_edge2<num_edges && !remove;
             i_edge2++, edge_point2+=2) {
          if (*edge_point == *edge_point2 &&
              *(edge_point+1) == *(edge_point2+1)) {
            remove = 1;
            *edge_point = *(edge_point+1) = *edge_point2 = *(edge_point2+1)= -1;
          }
        }
      }
    }
    num_edges_org = num_edges;
    num_edges = 0;
    for (i_edge=0, edge_point=edge_point2=edge_list;
         i_edge<num_edges_org; i_edge++, edge_point+=2) {
      if (*edge_point != -1) {
        num_edges++;
        *edge_point2 = *edge_point;
        *(edge_point2+1) = *(edge_point+1);
        edge_point2 += 2;
      }
    }
  }

// Create the holes list

  if (holes.size()) {
    hole_list = coordinate =
                (double *) malloc(2 * sizeof(double) * holes.size());
    for (point=holes.begin(); point!=holes.end(); point++) {
      *coordinate++ = point->X();
      *coordinate++ = point->Y();
    }
  }

// Create the constrained TIN

  tin = new TIN(coordinate_list, size(), edge_list, num_edges, hole_list,
                holes.size());

// Clear temporary data structures and return the TIN

  free(coordinate_list);
  if (num_edges) free(edge_list);
  if (holes.size()) free(hole_list);
  return(*tin);
}

/*
--------------------------------------------------------------------------------
          Duplicate all points with a fixed Z and a new point number
--------------------------------------------------------------------------------
*/

void ObjectPoints::DuplicateWithFixedZ(double z, int number_offset)
{
  ObjectPoints::iterator point;

  insert(end(), begin(), end());
  for (point=begin()+size()/2; point!=end(); point++) {
    point->Z() = z;
    point->Number() += number_offset;
  }
}

/*
--------------------------------------------------------------------------------
                   Write object faces in VRML 2.0 format
--------------------------------------------------------------------------------
*/

void ObjectPoints::VRML2_Write(FILE *fd, const LineTopologies &faces,
                               bool same_colour, double R, double G, 
                               double B) const
{
  ObjectPoints::const_iterator   point;
  LineTopologies::const_iterator face;
  LineTopology::const_iterator   node;
  int                            label, max_label;

  fprintf(fd, "Transform {\n");
  fprintf(fd, "  children Shape {\n");
  fprintf(fd, "    appearance Appearance {\n");
  fprintf(fd, "      material Material {\n");
  fprintf(fd, "        diffuseColor %2.1f %2.1f %2.1f\n", R, G, B);
  fprintf(fd, "      }\n"); // Closing material
  fprintf(fd, "    }\n"); // Closing appearance

  fprintf(fd, "    geometry IndexedFaceSet {\n");
  fprintf(fd, "      coord Coordinate {\n");
  fprintf(fd, "        point [\n");
  for (point=begin(); point!=end(); point++)
    fprintf(fd, "        %.2f %.2f %.2f,\n", point->X(), point->Y(), point->Z());
  fprintf(fd, "        ]\n"); // Closing point
  fprintf(fd, "      }\n"); // Closing coord
  fprintf(fd, "      coordIndex [\n");
  for (face=faces.begin(); face!=faces.end(); face++) {
    fprintf(fd, "       ");
    for (node=face->begin(); node!=face->end(); node++)
      fprintf(fd, " %d", node->Number());
    fprintf(fd, "-1 ,\n");
  }
  fprintf(fd, "      ]\n"); // Closing coordIndex
  fprintf(fd, "      solid FALSE\n"); // No backface culling

  if (!same_colour) {
    // Determine highest label
    for (face=faces.begin(), max_label=0; face!=faces.end(); face++)
      if (face->Label() > max_label) max_label = face->Label();
    // Write max_label+1 colours
    fprintf(fd, "      color Color {\n");
    fprintf(fd, "        color [\n");
    fprintf(fd, "          1.0 0.3 0.3,\n");                      // Red
    if (max_label >= 1) fprintf(fd, "          0.2 0.9 0.2,\n");  // Green
    if (max_label >= 2) fprintf(fd, "          0.0 0.5 1.0,\n");  // Blue
    if (max_label >= 3) fprintf(fd, "          1.0 1.0 0.8,\n");  // Yellow
    if (max_label >= 4) fprintf(fd, "          0.8 0.5 0.0,\n");  // Brown
    if (max_label >= 5) fprintf(fd, "          1.0 1.0 0.8,\n");  // Yellow
    if (max_label > 5)
      for (label=6; label<=max_label; label++)
        fprintf(fd, "          0.9 0.9 0.9,\n"); // Light grey
    fprintf(fd, "        ]\n"); // Closing color
    fprintf(fd, "      }\n"); // Closing color
    fprintf(fd, "      colorPerVertex FALSE\n");
    fprintf(fd, "      colorIndex [\n");
    fprintf(fd, "       ");
    for (face=faces.begin(); face!=faces.end(); face++)
      if (face->Label() > 0) fprintf(fd, " %d", face->Label());
      else fprintf(fd, " 0");
    fprintf(fd, "\n");
    fprintf(fd, "      ]\n"); // Closing colorIndex
  }
  fprintf(fd, "    }\n"); // Closing geometry
  fprintf(fd, "  }\n"); // Closing shape
  fprintf(fd, "}\n"); // Closing Transform
}

/*
--------------------------------------------------------------------------------
                         Sort on coordinates
--------------------------------------------------------------------------------
*/
int LargerXYZObjectPoint(const ObjectPoint *point1, const ObjectPoint *point2)
{
  if (point1->X() > point2->X() + 0.001) return(1);    /* First sort on X */
  if (point1->X() < point2->X() - 0.001) return(-1);
  if (point1->Y() > point2->Y() + 0.001) return(1);    /* Then on Y */
  if (point1->Y() < point2->Y() - 0.001) return(-1);
  if (point1->Z() > point2->Z() + 0.001) return(1);    /* Then on Z */
  if (point1->Z() < point2->Z() - 0.001) return(-1);
  return(0);                                   /* Same coordinates */
}

int LargerXYZObjectPointqsort(const void *pt1, const void *pt2)
{
  return(LargerXYZObjectPoint((const ObjectPoint *) pt1,
                              (const ObjectPoint *) pt2));
}

void ObjectPoints::SortOnCoordinates()
{
  qsort((void *) &*begin(), (int) size(), sizeof(ObjectPoint),
        LargerXYZObjectPointqsort);
}

/*
--------------------------------------------------------------------------------
          Duplicate all points with a fixed height offset and a new point number
--------------------------------------------------------------------------------
*/

void ObjectPoints::DuplicateWithFixedOffset(double z, int number_offset)
{
  ObjectPoints::iterator point;

  insert(end(), begin(), end());
  for (point=begin()+size()/2; point!=end(); point++) {
    point->Z() += z;
    point->Number() += number_offset;
  }
}

/*
--------------------------------------------------------------------------------
                  Derive the bounds of the point set
--------------------------------------------------------------------------------
*/
DataBounds3D & ObjectPoints::Bounds() const 
{
  DataBounds3D *bounds;
  ObjectPoints::const_iterator point;

  bounds = new DataBounds3D();
  for (point=begin(); point!=end(); point++)
    bounds->Update(point->Position3DRef());
  return *bounds;
}


/*
--------------------------------------------------------------------------------
                  Return point by point number
--------------------------------------------------------------------------------
*/

ObjectPoint ObjectPoints::PointByNumber(int number)
{
   for(int i=0;i<size();i++)
   {
           if((*this)[i].Number()==number)
           return (*this)[i];
   }
   
  
}


/*
--------------------------------------------------------------------------------
                  Return point Ref by point number
--------------------------------------------------------------------------------
*/

int  ObjectPoints::PointIndexByNumber(int number)
{
   for(int i=0;i<size();i++)
   {
           if((*this)[i].Number()==number)
           return i;
   }
   
  
}


/*
--------------------------------------------------------------------------------
                  Write object to a DXF file
--------------------------------------------------------------------------------
*/



void ObjectPoints::WriteDXF(FILE *dxffile, const LineTopologies &polygons, bool use_label_as_layer=false) const
{
  DataBounds3D                   bounds;
  bool                           pol_ok;
  LineTopologies::const_iterator polygon;
  LineTopology::const_iterator   node;
  ObjectPoints::const_iterator   point;
  
  bounds = Bounds();
  
  // Write the DXF header 
  fprintf(dxffile, "%3d\nSECTION\n" , DXFGRP_START);
  fprintf(dxffile, "%3d\nHEADER\n"  , DXFGRP_NAME);
  fprintf(dxffile, "%3d\n$ACADVER\n", DXFGRP_VARIABLE_NAME);
  fprintf(dxffile, "%3d\nAC1006\n"  , DXFGRP_ENTITY_TEXT);
  fprintf(dxffile, "%3d\n$INSBASE\n", DXFGRP_VARIABLE_NAME);
  fprintf(dxffile, "%3d\n0.0\n"     , DXFGRP_X_1);
  fprintf(dxffile, "%3d\n0.0\n"     , DXFGRP_Y_1);
  fprintf(dxffile, "%3d\n0.0\n"     , DXFGRP_Z_1);
  fprintf(dxffile, "%3d\n$EXTMIN\n" , DXFGRP_VARIABLE_NAME);
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_X_1, bounds.Minimum().X());
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_Y_1, bounds.Minimum().Y());
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_Z_1, bounds.Minimum().Z());
  fprintf(dxffile, "%3d\n$EXTMAX\n" , DXFGRP_VARIABLE_NAME);
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_X_1, bounds.Maximum().X());
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_Y_1, bounds.Maximum().Y());
  fprintf(dxffile, "%3d\n%f\n"      , DXFGRP_Z_1, bounds.Maximum().Z());
  fprintf(dxffile, "%3d\nENDSEC\n"  , DXFGRP_START);

  // Process all polygons
  fprintf(dxffile, "%3d\nSECTION\n" , DXFGRP_START);
  fprintf(dxffile, "%3d\nENTITIES\n", DXFGRP_NAME);
  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {

    // Check whether all points of the polygon are present
    pol_ok = true;
    for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++)
      if (GetPoint(*node) == NULL) pol_ok = false;

    // Output the polygons on layer 1 in colour 7 (white)
    if (pol_ok) {
      fprintf(dxffile, "%3d\nPOLYLINE\n"  , DXFGRP_START);
      
      if(use_label_as_layer)
          fprintf(dxffile, "%3d\n%d\n"         , DXFGRP_LAYER_NAME, polygon->Label());
      else
          fprintf(dxffile, "%3d\n1\n"         , DXFGRP_LAYER_NAME);
          
      fprintf(dxffile, "%3d\n7\n"         , DXFGRP_COLOUR);
      fprintf(dxffile, "%3d\nCONTINUOUS\n", DXFGRP_LINETYPE);
      fprintf(dxffile, "100\nAcDb3dPolyline\n");
      fprintf(dxffile, "%3d\n1\n"         , DXFGRP_ENTITIES_FOLLOW);
      if (polygon->IsClosed())
        fprintf(dxffile, "%3d\n9\n"       , DXFGRP_INT_1);
      else
        fprintf(dxffile, "%3d\n8\n"       , DXFGRP_INT_1);
        
        fprintf(dxffile, "%3d\n0\n"        , DXFGRP_X_1);
        fprintf(dxffile, "%3d\n0\n"        , DXFGRP_Y_1);
        fprintf(dxffile, "%3d\n5\n"        , DXFGRP_Z_1);
//      fprintf(dxffile, "71\n%d\n"         , polygon->size());
//      fprintf(dxffile, "72\n1\n");
      // Output each vertex of the polygon
      for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++) {
        point = ConstPointIterator(*node);
        fprintf(dxffile, "%3d\nVERTEX\n"    , DXFGRP_START);
        fprintf(dxffile, "100\nAcDb3dPolylineVertex\n");
        fprintf(dxffile, "%3d\n32\n"       , DXFGRP_INT_1);
        if(use_label_as_layer)
          fprintf(dxffile, "%3d\n%f\n"         , DXFGRP_LAYER_NAME, polygon->Label());
        else
          fprintf(dxffile, "%3d\n1\n"         , DXFGRP_LAYER_NAME);
        fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_X_1, point->X());
        fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_Y_1, point->Y());
        fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_Z_1, point->Z());
      }
      fprintf(dxffile, "%3d\nSEQEND\n"    , DXFGRP_START);
    }
    else {
      fprintf(stderr, "Warning: Points of polygon %d are not available in the points database.\n",
              polygon->Number());
    }
  }

  // End of the entities section and end of the DXF file
  fprintf(dxffile, "%3d\nENDSEC\n"  , DXFGRP_START);
  fprintf(dxffile, "%3d\nEOF\n"     , DXFGRP_START);
}

/*
--------------------------------------------------------------------------------
              Average heights of points with nearly the same coordinates
--------------------------------------------------------------------------------
*/

void ObjectPoints::AverageHeightsOfDoublePoints(LineTopologies &polygons, double min_dist, double min_height_dist)
{
  LineTopologies::iterator polygon;
  LineTopology::iterator node;
  ObjectPoints::iterator   point1, point2;
  int                      double_number, new_number, old_number, count;
  
  // Sort on coordinates
  SortOnCoordinates();
  
  // Mark double points with point highest point number + 1
  double_number = HighestPointNumber().Number() + 1;
  count = 0;
  for (point1=begin(); point1!=end(); point1++) {
    if (point1->Number() != double_number) {
      for (point2=point1+1;
           point2!=end() && fabs(point2->X() - point1->X()) <= min_dist;
           point2++) {
        if (point2->Number() != double_number) {
          if (fabs(point2->Y() - point1->Y()) <= min_dist) {//;point1->Distance(point2->Position3DRef()) <= min_dist) {
             if (fabs(point1->Z() - point2->Z()) <=min_height_dist) {
               count++;
               new_number = point1->Number();
               old_number = point2->Number();
               point1->Z() = (point1->Z() + point2->Z())/2;
               for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++)
                 for (node=polygon->begin(); node!=polygon->end(); node++)
                   if (node->Number() == old_number) node->Number() = new_number;
               point2->Number() = double_number;
             }
          }
        }
      }
    }
  }
  
  // Sort on point numbers
  Sort();
  
  // Erase all double points
  erase(end()-count, end());
}


/*
--------------------------------------------------------------------------------
                  Write object to a DXF file, simple but polygons filled
                  Works with linetopologies in triangular structure
                  (Non-triangulated polygons will be converted to TIN's).
                  Color scheme:
                        0 = dark grey
                        1 = red
                        2 = yellow
                        3 = green
                        4 = cyan
                        5 = blue
                        6 = purple
                        7 = white  
                   Optional: 
                   Write edges (Default true)                         
                   Write header (Default true)
                   Write footer (Default true)
                   (Header and footer can be switched off in case of writing 
                   multiple linetopologies to the same dxf file.)
                   
--------------------------------------------------------------------------------
*/
void ObjectPoints::WriteDXFMesh(FILE *dxffile, const LineTopologies &polygons,
                                int color, bool line, bool header, bool footer,
                                bool use_label_as_layer) const
{
  bool                           pol_ok;
  LineTopologies::const_iterator polygon, polygon2;
  LineTopology::const_iterator   node;
  ObjectPoints::const_iterator   point;
  ObjectPoints                   copy_points, sel_map_points;
  int                            numx, numy, numz;
  TIN                            tin;
  LineTopologies                 one_map_line, map_tin_lines;
  
  if (header) fprintf(dxffile,"0\nSECTION\n2\nHEADER\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\n");

  if (line) {
    for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
      // Check whether all points of the polygon are present
      pol_ok = true;
      for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++)
        if (GetPoint(*node) == NULL) pol_ok = false;
        
      if (pol_ok) {
        fprintf(dxffile, "POLYLINE\n");
        fprintf(dxffile, "100\nAcDb3dPolyline\n");
        fprintf(dxffile, "%3d\n1\n"         , DXFGRP_ENTITIES_FOLLOW);
        if (polygon->IsClosed())
          fprintf(dxffile, "%3d\n9\n"       , DXFGRP_INT_1);
        else
          fprintf(dxffile, "%3d\n8\n"       , DXFGRP_INT_1);

        // Output each vertex of the polygon
        for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++) {
          point = ConstPointIterator(*node);
          fprintf(dxffile, "%3d\nVERTEX\n"    , DXFGRP_START);
          fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_X_1, point->X());
          fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_Y_1, point->Y());
          fprintf(dxffile, "%3d\n%f\n"        , DXFGRP_Z_1, point->Z());
        }
        fprintf(dxffile, "%3d\nSEQEND\n0\n"    , DXFGRP_START);
      }
      else {
        fprintf(stderr, "Warning: Points of polygon %d are not available in the points database.\n",
                polygon->Number());
      }
    }
  }
  
  for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
    if(!sel_map_points.empty())
      sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
    // Check whether all points of the polygon are present
    pol_ok = true;
    for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++){
      if (GetPoint(*node) == NULL) pol_ok = false;
      point = ConstPointIterator(*node);
      sel_map_points.push_back(*point);
    }
    // Output the polygons
    if (pol_ok) {
      one_map_line.erase(one_map_line.begin(), one_map_line.end());
      if(!map_tin_lines.empty())
        map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
      one_map_line.push_back(*polygon);      
      sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
      tin.Erase();  
      if (sel_map_points.size()>5){
        one_map_line.ReNumber(sel_map_points, 0, 0);                             
        tin = sel_map_points.Triangulate(one_map_line);
        map_tin_lines = LineTopologies(tin); 
        for (polygon2 = map_tin_lines.begin(); polygon2!=map_tin_lines.end();polygon2++){           
          fprintf(dxffile,"3DFACE\n8\n0\n");
	      if (use_label_as_layer)
            fprintf(dxffile, "%3d\n%d\n", DXFGRP_LAYER_NAME, polygon->Label());
          else
            fprintf(dxffile, "%3d\n1\n"         , DXFGRP_LAYER_NAME);
          fprintf(dxffile, "%3d\n%3d\n"         , DXFGRP_COLOUR, color);
          fprintf(dxffile, "%3d\nCONTINUOUS\n", DXFGRP_LINETYPE);
          numx = 10;
          numy = 20;
          numz = 30;
          for (node=polygon2->begin(); node!=polygon2->end() && pol_ok; node++) {
            point = sel_map_points.ConstPointIterator(*node);
            fprintf(dxffile,"%d\n%10.4f\n%d\n%10.4f\n%d\n%10.4f\n",
                    numx, point->X(), numy, point->Y(), numz, point->Z());
            numx++;
            numy++;
            numz++;
          }
          fprintf(dxffile,"%d\n%d\n", 70, 15);
          fprintf(dxffile,"0\n");
          }
        }
      else {
        fprintf(dxffile,"3DFACE\n8\n0\n");
	    if (use_label_as_layer)
          fprintf(dxffile, "%3d\n%d\n", DXFGRP_LAYER_NAME, polygon->Label());
        else
          fprintf(dxffile, "%3d\n1\n"         , DXFGRP_LAYER_NAME);
        fprintf(dxffile, "%3d\n%3d\n"         , DXFGRP_COLOUR, color);
        fprintf(dxffile, "%3d\nCONTINUOUS\n", DXFGRP_LINETYPE);
        numx = 10;
        numy = 20;
        numz = 30;
        for (node=polygon->begin(); node!=polygon->end() && pol_ok; node++) {
          point = sel_map_points.ConstPointIterator(*node);
          fprintf(dxffile,"%d\n%10.4f\n%d\n%10.4f\n%d\n%10.4f\n",
                  numx, point->X(), numy, point->Y(), numz, point->Z());
          numx++;
          numy++;
          numz++;
        }
        fprintf(dxffile,"%d\n%d\n", 70, 15);
        fprintf(dxffile,"0\n");
      }     
    }
  }
      
  // Print End of file
      
  if (footer) fprintf(dxffile,"ENDSEC\n0\nEOF\n");
}

int ObjectPoints::MaxPointNumber()
{
  ObjectPoints::const_iterator i;
  int max=0;
 
  for(i = begin(); i != end(); i++)
        {
         if(i->Number()>max)
         max=i->Number();      
        }
     
  return max;   
}
// check if an object point is visible
/* 
    @param objpt      the point to check
    @param extor      exterior orientation
    @param top        a line topology for a line including the point
    
*/
    
bool ObjectPoints::PointVisible(ObjectPoint *objpt,
                  ExteriorOrientation* extor,
                  LineTopology* top, double max_dist){
                                                                                         
     int objpt_number=objpt->Number();
     //2D position of object point
     Position2D objpt_pos(objpt->X(),objpt->Y());
     //projection centre x,y
     Position2D centre(extor->X(),extor->Y());
     //make line from viewing point to tested point
     LineSegment2D *view_line=new LineSegment2D(objpt_pos, centre);
     //test visibility
     LineTopology::iterator mypoint_number, mypoint_number_next;     
     for (mypoint_number=top->begin();
                  mypoint_number!=top->end(); mypoint_number++){
         mypoint_number_next=mypoint_number+1;                                  
         ObjectPoint *objpt1=GetPoint(*mypoint_number);  
         ObjectPoint *objpt2=GetPoint(*mypoint_number_next);
         int objpt_number1=objpt1->Number();
         int objpt_number2=objpt2->Number();
         Position2D pos1(objpt1->X(),objpt1->Y());
         Position2D pos2(objpt2->X(),objpt2->Y());
         //line between consecutive points in line_topology
         LineSegment2D *other_line=new LineSegment2D(pos1, pos2);
         //check if two line segments intersect
         Position2D intersection_pos;
         bool intersect=Intersect2LineSegments2D(*view_line,*other_line, intersection_pos, max_dist);              
         if(intersect && objpt_number!=objpt_number1 && objpt_number!=objpt_number2 ){
              return false;
              }
         }
     return true;
     }

// check if a mid point of two object point is visible
/* 
    @param objpt1      the point one
    @param objpt2      the point two 
    @param extor      exterior orientation
    @param top        a line topology for a line including the point
    
*/
    
bool ObjectPoints::MiddlePointVisible(ObjectPoint *objpt1,
                                   ObjectPoint *objpt2,
                                   ExteriorOrientation* extor,
                                   LineTopology* top, double max_dist){
                                                                                        
     int objpt1_number=objpt1->Number();
     int objpt2_number=objpt2->Number();
     //positions of object points 1 and 2
     Position2D objpt1_pos(objpt1->X(),objpt1->Y());
     Position2D objpt2_pos(objpt2->X(),objpt2->Y());
     //segment of the two positions
     LineSegment2D *tested_line=new LineSegment2D(objpt1_pos, objpt2_pos);
     //2D position of midpoint of object points 1 and 2
     Position2D mid_objpt_pos=tested_line->MiddlePoint();
     
     //projection centre x,y
     Position2D centre(extor->X(),extor->Y());
     //make line from viewing point to tested point
     LineSegment2D *view_line=new LineSegment2D(mid_objpt_pos, centre);
     //test visibility
      
     LineTopology::iterator mypoint_number, mypoint_number_next;     
     for (mypoint_number=top->begin();
                  mypoint_number!=top->end()-1; mypoint_number++){
         mypoint_number_next=mypoint_number+1;                                  
         ObjectPoint *objpt1=GetPoint(*mypoint_number);  
         ObjectPoint *objpt2=GetPoint(*mypoint_number_next);

         int objpt_number1=objpt1->Number();
         int objpt_number2=objpt2->Number();
         Position2D pos1(objpt1->X(),objpt1->Y());
         Position2D pos2(objpt2->X(),objpt2->Y());
         //line between consecutive points in line_topology
         LineSegment2D *other_line=new LineSegment2D(pos1, pos2);
         //check if two line segments intersect
         Position2D intersection_pos;
         bool intersect=Intersect2LineSegments2D(*view_line,*other_line, intersection_pos, max_dist);

         if(intersect && !((objpt1_number==objpt_number1 && objpt2_number==objpt_number2)||
         (objpt1_number==objpt_number2 && objpt2_number==objpt_number1))){
              return false;
              }
         }
     return true;
     }

