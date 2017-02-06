
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
 Collection of functions for class LineTopologies          

 LineTopologies LineTopologies::operator = Copy assignment
   (const LineTopologies &)
 void LineTopologies::Cpp2C(LineTops **    Conversion of C++ to C object
 void LineTopologies::C2Cpp(LineTops *)    Conversion of C to C++ object
 int  LineTopologies::Read(char *)         Read lines from a database
 int  LineTopologies::Write(char *)        Write lines points to a database
 void LineTopologies::Print()              Print lines points to stdout
 void LineTopologies::VRML_Write           Write lines to VRML file
  (FILE *, int) const
 int  LineTopologies::FindLine(LineNumber) Find a line in the list
 void LineTopologies::MergePolygons()             Merge polygons with the same
                                                  label
 void LineTopologies::ReNumber                    Renumber points and polygons
   (ObjectPoints &, int)
 void LineTopologies::ReNumber                    Renumber points and polygons
   (ObjectPoints2D &, int)
 void LineTopologies::InsertNodes                 Insert nodes on segments of
   (const ObjectPoints &)                         the polygons
 void LineTopologies::RemoveCollinearNodes        Remove collinear nodes
   (const ObjectPoints &)
 LineTopologies::LineTopologies                   Construct from a TIN
   (const TIN &, int, int)
 void LineTopologies::DuplicateWithOffset(int)    Duplicate polygons
 void LineTopologies::AddWalls                    Add wall topology
   (const LineTopologies &, int) 
 void LineTopologies::Densify                     Densify points on edges
   (ObjectPoints &, double)
 DataBounds3D &LineTopologies::Bounds             Determine bounds of polygons
   (const ObjectPoints &) const


 Initial creation
 Author : Ildi Suveg
 Date   : 24-11-1998

 Update #1
 Author : George Vosselman
 Date   : 08-06-1999
 Changes: Added VRML_Write

 Update #2
 Author : George Vosselman
 Date   : 30-11-1999
 Changes: Added FindLine

 Update #3
 Author : George Vosselman
 Date   : 03-05-2001
 Changes: Added MergePolygons until Densify

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "ObjectPoints2D.h"
#include "TIN.h"
#include "DataBounds3D.h"

/*
--------------------------------------------------------------------------------
                             Copy assignment
--------------------------------------------------------------------------------
*/

LineTopologies & LineTopologies::operator = (const LineTopologies &linetops)
{
  // Check for self assignment
  if (this == &linetops) return *this;
  if (!empty()) erase(begin(), end());
  if (!linetops.empty()) insert(begin(), linetops.begin(), linetops.end());
  if (linetops.name) {
    name = (char *) realloc(name, strlen(linetops.name) + 1);
    strcpy(name, linetops.name);
  }
  else {
    if (name) free(name);
    name = NULL;
  }
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Conversion of C++ to C object
--------------------------------------------------------------------------------
*/

void LineTopologies::Cpp2C(LineTops **linesptr) const
{
   LineTops *lines;

/* Allocate space if this has not been done yet */

  lines = *linesptr;
  if (lines == NULL) {
    lines = (LineTops *) malloc(sizeof(LineTops));
    if (size()) lines->lines = (LineTop*) malloc(size() * sizeof(LineTop)); 
    else lines->lines = NULL;
    *linesptr = lines;
  }

  if (name) strcpy(lines->name, name);
  else lines->name[0] = 0;
 
/* Copy the data */

  lines->num_lines = size();
  LineTopologies::const_iterator i;
  LineTop *ln; 
  int l = 0;
  for(i = begin(); i != end(); i++, l++)
  {
	lines->lines[l].pts = (int*) malloc(i->size() * sizeof(int));
	if (lines->lines[l].pts == NULL)
		printf("points allocation error\n");
	
	ln = &lines->lines[l];
	i->Cpp2C(&ln);
  }  
}

/*
--------------------------------------------------------------------------------
                     Conversion of C to C++ object
--------------------------------------------------------------------------------
*/

void LineTopologies::C2Cpp(LineTops *lines)
{
  if (!empty())
  	erase(begin(), end());

  name = new char[strlen(lines->name) + 1];
  strcpy(name, lines->name);
  
  int l = 0;
  
  for(int i = 0 ; i < lines->num_lines; i++, l++)
  {
  	LineTopology line(&lines->lines[l]);
  	push_back(line);
          }   
for(int i=0;i<size();i++)
     (*this)[i].SetAttribute(LineLabelTag,(lines->lines[i]).label);	  

}

/*
--------------------------------------------------------------------------------
                       Read lines from a database
--------------------------------------------------------------------------------
*/
int LineTopologies::Read(const char *filename, bool printlog)
{
  ifstream in(filename, ios::in);
  int file_no;
  
  if(!in.is_open())
    {
     cout<<"Read failure.";
     return 0;
     }
  
  in>>file_no;
  in.close();
  switch(file_no)
  {
  case LINE_TOPOLOGY_V1:
       ReadV1(filename, printlog);
       break;
  default:
       ReadV0(filename);
       break;
  }
  return 1;
  
}
int LineTopologies::ReadV0(const char *filename)
{
  LineTops *lines;
  
  cout<<"Reading Line Topology Version 0."<<endl;
  lines = Get_LineTops(filename);    /* Read the database into C structure */
  if (lines == NULL) return(0); 
  C2Cpp(lines);                     /* Convert to C++ object */
  Free_LineTops(lines);
  return(1);
}

/*
int LineTopologies::ReadV1(const char *filename, bool printlog)
{
char title[256];
  
  LineTopology *top;

  int linenumber, num_pts, num_attributes;
  int attribute_tag, attribute_value;
  int pt_num;
  int i;
  ifstream in(filename, ios::in);
  if (printlog) cout<<"Reading Line Topology Version 1."<<endl;
  
  
  clear();
  //skip header
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
   
  //start reading data
  while(!in.eof())
  {
  in>>linenumber>>num_pts>>num_attributes;
  
  
  if(in.eof())
     break;
     
  
  top=new LineTopology(linenumber, num_attributes);
  
  //read attributes
  for(i=0;i<num_attributes;i++)
    {
    in>>attribute_tag>>attribute_value;
    top->SetAttribute(attribute_tag,attribute_value);
    }
  
  //read points in this linetopology
   
  for(i=0;i<num_pts;i++)
    {
    in>>pt_num;    
    top->push_back(PointNumber(pt_num));  
    }
  push_back(*top);
  top->Erase();  
  free(top);
  }

  in.close();
  //pop_back();
  if (printlog) cout<<size()<< " lines read from "<<filename<<"."<<endl;
  return(1);

}
*/

int LineTopologies::ReadV1(const char *filename, bool printlog)
{
char title[256];
  
  LineTopology *top;

  int linenumber, num_pts, num_attributes;
  int attribute_tag, attribute_value;
  int pt_num;
  int i;
  ifstream in(filename, ios::in);
  if (printlog) cout<<"Reading Line Topology Version 1."<<endl;
  LineTopologies::iterator iter;
  
  clear();
  //skip header
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
  in.getline(title,256);
   
  //start reading data
  while(!in.eof())
  {
  in>>linenumber>>num_pts>>num_attributes;
  
  
  if(in.eof())
     break;
     
  
  top=new LineTopology(linenumber, num_attributes);
  push_back(*top);
  
  iter=end()-1;
  
  iter->Number()=linenumber;
  
  //read attributes
  for(i=0;i<num_attributes;i++)
    {
    in>>attribute_tag>>attribute_value;
    iter->SetAttribute(attribute_tag,attribute_value);
    }
  
  //read points in this linetopology
   
  for(i=0;i<num_pts;i++)
    {
    in>>pt_num;    
    iter->push_back(PointNumber(pt_num));  
    }
  
  //iter->Erase();  
  free(top);
  }

  in.close();
  //pop_back();
  if (printlog) cout<<size()<< " lines read from "<<filename<<"."<<endl;
  return(1);

}




/*
--------------------------------------------------------------------------------
                        Write lines to a database
--------------------------------------------------------------------------------
*/

int LineTopologies::Write(const char *filename, bool printlog) const
 {
                                /*
 int    error;
 LineTops *lines;

 lines = NULL;
   Cpp2C(&lines);
error = Put_LineTops(lines, filename);
 Free_LineTops(lines);
  return(error);
  */
  WriteV1(filename, printlog);
 }




int LineTopologies::WriteV1(const char *filename, bool printlog) const
{
 int    error;
  
  ofstream out(filename, ios::out);
  out << LINE_TOPOLOGY_V1<<" # file ID" << "\n";
  out << "# Line topology database :"<<"\n";
  out << "# Number of lines :"<<"\t"<<size()<<"\n";
  out << "# Line number"<<"   \\   "<<"Number of points"<<"   \\   "<<"Number of attributes"<<"\n";
  out << "# Attributes Tag number"<<"   \\   "<<"Attribute value"<<"\n";
  out << "# Point numbers"<<"\n";
  out << "# -----------------------------------------------------------------------------------------------------------"<<"\n";
  
  
  LineTopologies::const_iterator line;
  int k,m;
  const unsigned char *tags;
  const int           *values;
  int num;

  for (line=begin(); line!=end(); line++)
    {
    num=(int)(line->NumAttributes());  
    out<<line->Number()<<"\t"<<line->size()<<"\t"<<num<<"\n";   
    tags=line->AttributeTags();
    values=line->AttributeValues();
    for(m=0;m<num;m++)
    {
    out<<(int)tags[m]<<"\t"<<values[m]<<"\n";
    } 
    for(m=0;m<line->size();m++)
    {
    out<<(*line)[m].Number()<<"\t";
    }
    out<<"\n";
    }
  out.close();
  if (printlog) cout<<size()<<" lines written to "<<filename<<"."<<endl;
  return 1;
}


/*
--------------------------------------------------------------------------------
                        Print lines to stdout
--------------------------------------------------------------------------------
*/
void LineTopologies::Print() const
{
  LineTops *lines;

  lines = NULL;
  Cpp2C(&lines);
  Print_LineTops(lines);
  Free_LineTops(lines);
}

/*
--------------------------------------------------------------------------------
                        Write lines to VRML file
--------------------------------------------------------------------------------
*/
void LineTopologies::VRML_Write(FILE *fd, int show_as_face_if_closed) const
{
  LineTopologies::const_iterator line;

  for (line=begin(); line!=end(); line++)
    line->VRML_Write(fd, show_as_face_if_closed);
}

/*
--------------------------------------------------------------------------------
                        Find a line in the list
--------------------------------------------------------------------------------
*/
int LineTopologies::FindLine(const LineNumber &linenumber) const
{
  LineTopologies::const_iterator line;
  int k;

  for (line=begin(), k=0; line!=end(); line++, k++)
    if (line->Number() == linenumber.Number()) return(k);
  return(-1);
}


/*
--------------------------------------------------------------------------------
                        Find a line in the list by certain tag value
--------------------------------------------------------------------------------
*/
int LineTopologies::FindLineByTagValue(const LineTopologyTag tag, const int value) const
{
    LineTopologies::const_iterator line;
    int k;

  for (line=begin(), k=0; line!=end(); line++, k++)
    {
    if(!line->HasAttribute(tag))
    continue;
    
    if (line->Attribute(tag) == value) return(k);
  
    }
  
  
  return(-1);
    
}


/*
--------------------------------------------------------------------------------
                   Merge polygons with the same label
--------------------------------------------------------------------------------
*/

void LineTopologies::MergePolygons()
{
  LineTopologies::iterator pol1, pol2, last_pol;
  LineTopology             merged_pol;
  int                      merge;

  printf("Starting MergePolygons with %d polygons.\n", size());
  for (pol1=begin(); pol1!=end(); pol1++) {
    merge = 1;
    while (merge) {
      for (pol2=pol1+1, merge=0; pol2!=end() && !merge; pol2++) {
        if (pol1->Label() == pol2->Label()) {

/* See if the polygons can be merged */

          if (pol1->Merge(pol2->LineTopologyReference(), merged_pol)) {
            printf("Merging polygons %d and %d\n", pol1->Number(),
                   pol2->Number());

/* Replace the first polygon by the merged one */

            *pol1 = merged_pol;
            merge = 1;

/* Remove the second polygon */

            last_pol = end() - 1;
            if (pol2 != last_pol) *pol2 = *last_pol;
            erase(last_pol, end());
          }
        }
      }
    }
  }
  printf("Finishing MergePolygons with %d polygons.\n", size());
}

/*
--------------------------------------------------------------------------------
                        Renumber points and polygons
--------------------------------------------------------------------------------
*/

void LineTopologies::ReNumber(ObjectPoints &points, int first_point_number,
                              int first_line_number)
{
  int                       point_index, line_index;
  LineTopologies::iterator  line;
  PointNumberList::iterator node;
  ObjectPoints::iterator    point;

/* Renumber the topology */

  for (line=begin(), line_index=0; line!=end(); line++, line_index++) {
    for (node=line->begin(); node!=line->end(); node++) {
      point_index = points.FindPoint(node->NumberRef());
      if (point_index == -1) {
        printf("Error: Pointnumber %d not in list of object points.\n",
               node->Number());
        exit(0);
      }
      node->Number() = first_point_number + point_index;
    }
    if (first_line_number >= 0) line->Number() = first_line_number + line_index;
  }

/* Renumber the points */

  for (point_index=0, point=points.begin(); point!=points.end();
       point_index++, point++)
    point->Number() = first_point_number + point_index;
}

void LineTopologies::ReNumber(ObjectPoints2D &points, int first_point_number,
                              int first_line_number)
{
  int                       point_index, line_index;
  LineTopologies::iterator  line;
  PointNumberList::iterator node;
  ObjectPoints2D::iterator  point;

/* Renumber the topology */

  for (line=begin(), line_index=0; line!=end(); line++) {
    for (node=line->begin(); node!=line->end(); node++) {
      point_index = points.FindPoint(node->NumberRef());
      if (point_index == -1) {
        printf("Error: Pointnumber %d not in list of object points.\n",
               node->Number());
        exit(0);
      }
      node->Number() = first_point_number + point_index;
    }
    line->Number() = first_line_number + line_index;
  }

/* Renumber the points */

  for (point_index=0, point=points.begin(); point!=points.end();
       point_index++, point++)
    point->Number() = first_point_number + point_index;
}

/*
--------------------------------------------------------------------------------
                  Insert nodes for points on segments of the polygons
--------------------------------------------------------------------------------
*/

void LineTopologies::InsertNodes(const ObjectPoints &points)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    polygon->InsertNodes(points);
}

/*
--------------------------------------------------------------------------------
                           Remove collinear nodes
--------------------------------------------------------------------------------
*/

void LineTopologies::RemoveCollinearNodes(const ObjectPoints &points)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    polygon->RemoveCollinearNodes(points);
}

/*
--------------------------------------------------------------------------------
               Construct by converting TIN meshes to triangular polygons
--------------------------------------------------------------------------------
*/

LineTopologies::LineTopologies(const TIN &tin, int first_line_number,
                               int label)
{
  TIN::const_iterator mesh;
  int                 line_number = first_line_number;

  name = NULL;
  for (mesh=tin.begin(); mesh!=tin.end(); mesh++, line_number++)
    push_back(LineTopology(*mesh, line_number, label));
}

/*
--------------------------------------------------------------------------------
               Duplicate polygons with a fixed point number offset
--------------------------------------------------------------------------------
*/

void LineTopologies::DuplicateWithOffset(int number_offset)
{
  LineTopologies::iterator polygon;
  LineTopology::iterator   node;

  insert(end(), begin(), end());
  for (polygon=begin()+size()/2; polygon!=end(); polygon++) {
    polygon->Number() += size() / 2;
    for (node=polygon->begin(); node!=polygon->end(); node++)
      node->Number() += number_offset;
  }
}

/*
--------------------------------------------------------------------------------
               Add walls between roof and floor points
--------------------------------------------------------------------------------
*/

void LineTopologies::AddWalls(const LineTopologies &roof_outlines,
                              int number_offset)
{
  int wall_number=0;
  LineTopologies::const_iterator roof_outline;
  LineTopology::const_iterator node;
  LineTopology wall;

  if (!empty()) wall_number = (end()-1)->Number();
  for (roof_outline=roof_outlines.begin();
       roof_outline!=roof_outlines.end(); roof_outline++) {
    wall_number++;  wall.Number() = wall_number;
    for (node=roof_outline->begin(); node!=roof_outline->end()-1; node++) {
      wall.push_back(*node);
      wall.push_back(PointNumber(node->Number() + number_offset));
      wall.push_back(PointNumber((node+1)->Number() + number_offset));
      wall.push_back(*(node+1));
      wall.push_back(*node);
      wall.Label() = roof_outline->Label();
      push_back(wall);
      wall.erase(wall.begin(), wall.end());
    }
  }
}

/*
--------------------------------------------------------------------------------
            Densify polygons by inserting new nodes on long edges
--------------------------------------------------------------------------------
*/
void LineTopologies::Densify(ObjectPoints &points, double max_dist)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    polygon->Densify(points, max_dist);
}

/*
--------------------------------------------------------------------------------
                  Determine the bounds of the polygons
--------------------------------------------------------------------------------
*/
DataBounds3D &LineTopologies::Bounds(const ObjectPoints &points) const
{
  DataBounds3D *all_bounds = new DataBounds3D(), pol_bounds;
  LineTopologies::const_iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    all_bounds->Update(polygon->Bounds(points));
  return *all_bounds;
}

/*
--------------------------------------------------------------------------------
               Test if one of the polygons contains a point number
--------------------------------------------------------------------------------
*/
int LineTopologies::Contains(const PointNumber &number) const
{
  LineTopologies::const_iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (polygon->Contains(number)) return 1;
  return 0;
}


/*
--------------------------------------------------------------------------------
               Test if a polygon is part of the polygon set
--------------------------------------------------------------------------------
*/
bool LineTopologies::Contains(const LineTopology &top) const
{
  LineTopologies::const_iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (*polygon == top) return true;
  return false;
}

/*
--------------------------------------------------------------------------------
      Test if the points of all polygons are present in a point number list
--------------------------------------------------------------------------------
*/
int LineTopologies::IsPartOf(const PointNumberList &list) const
{
  LineTopologies::const_iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (!polygon->IsPartOf(list)) return 0;
  return 1;
}

/*
--------------------------------------------------------------------------------
    Test if some points of some polygons are present in a point number list
--------------------------------------------------------------------------------
*/
int LineTopologies::OverlapsWith(const PointNumberList &list) const
{
  LineTopologies::const_iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (polygon->OverlapsWith(list)) return 1;
  return 0;
}

/*
--------------------------------------------------------------------------------
                       Delete all lines
--------------------------------------------------------------------------------
*/
void LineTopologies::Erase()
{
  LineTopologies::iterator polygon;

  if (size()) {
    for (polygon=begin(); polygon!=end(); polygon++) polygon->Erase();
    erase(begin(), end());
  }
}

/*
--------------------------------------------------------------------------------
           Make polygons counter clockwise when seen from the top
--------------------------------------------------------------------------------
*/
void LineTopologies::MakeCounterClockWise(const ObjectPoints &points)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++) {
    polygon->MakeCounterClockWise(points);
  }
}

/*
--------------------------------------------------------------------------------
           Make polygons clockwise when seen from the top
--------------------------------------------------------------------------------
*/
void LineTopologies::MakeClockWise(const ObjectPoints &points)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++) {
    polygon->MakeClockWise(points);
  }
}


/*
--------------------------------------------------------------------------------
                                Sort the lines
--------------------------------------------------------------------------------
*/

int CompareLineNumbers(const void *line1, const void *line2)
{ return ((LineTopology *) line1)->Number() -
         ((LineTopology *) line2)->Number(); }

void LineTopologies::Sort()
{
  qsort((void *) &*(this->begin()), this->size(), sizeof(LineTopology),
        CompareLineNumbers);

}

/*
--------------------------------------------------------------------------------
                 Set an attribute value for all lines
--------------------------------------------------------------------------------
*/
void LineTopologies::SetAttribute(const LineTopologyTag tag, const int value)
{
  LineTopologies::iterator polygon;

  for (polygon=begin(); polygon!=end(); polygon++)
    polygon->SetAttribute(tag, value);
}

/*
--------------------------------------------------------------------------------
                 Select attributed lines from line topologies
--------------------------------------------------------------------------------
*/
LineTopologies LineTopologies::SelectAttributedLines(const LineTopologyTag tag, const int value)
{
  LineTopologies::iterator polygon;
  LineTopologies           attributed_lines;

  for (polygon=begin(); polygon!=end(); polygon++){
    if (polygon->Attribute(tag) == value){
        attributed_lines.push_back(*polygon);
    }
  }
  return attributed_lines;
}

/*
--------------------------------------------------------------------------------
                 Select attributed lines from line topologies
--------------------------------------------------------------------------------
*/
std::vector <int> LineTopologies::AttributedValues(const LineTopologyTag tag)
{
       vector <int> *values = new vector <int>();
       vector <int>::iterator stored_value;
       int value;
       LineTopologies::iterator polygon;
       bool found;
       
       for (polygon=begin(); polygon!=end(); polygon++){
         if(polygon->HasAttribute(tag)){
           value = polygon->Attribute(tag);
           for (stored_value=values->begin(), found=false;
              stored_value!=values->end() && !found; stored_value++)
               if (value == *stored_value) found = true;
           if (!found) values->push_back(value);
         }
       }

       return *values;
}


/*
--------------------------------------------------------------------------------
               Add floor between floor points with same topology as roof
--------------------------------------------------------------------------------
*/

void LineTopologies::AddFloor(const LineTopologies &roof_outlines,
                              int number_offset)
{
  int floor_number=0;
  LineTopologies::const_iterator roof_outline;
  LineTopology::const_iterator node;
  LineTopology floor;

  if (!empty()) floor_number = (end()-1)->Number();
  for (roof_outline=roof_outlines.begin();
       roof_outline!=roof_outlines.end(); roof_outline++) {
    floor_number++;  floor.Number() = floor_number;
    for (node=roof_outline->begin(); node!=roof_outline->end(); node++) {
      floor.push_back(PointNumber(node->Number() + number_offset));  
    }
    floor.Label() = roof_outline->Label();
    push_back(floor);
    floor.erase(floor.begin(), floor.end());
  }
}

/*
--------------------------------------------------------------------------------
               Transform Linetopologies to graph vector
--------------------------------------------------------------------------------
*/

vector<int> LineTopologies::TransformToGraph(ObjectPoints points, bool label, bool number)

{
 vector<int>                  graph_vector, nullvector;
 ObjectPoints                 graphpoints;
 LineTopologies               graphlines;
 LineTopologies::iterator     map_linet; 
 LineTopology::iterator       nodet;
 int                          it, jt, iit, jjt, sn1t, sn2t;
  
  if (label && number){
            printf ("Error: only one of two attributes can be transformed in graph\n");
            return nullvector;
            }
  if (!label && !number){
             printf ("simple adjacancy graph (with 0-s and 1-s) will be created.\n");
             }
 // select only the nodes that are in these linetopologies
    for (map_linet= begin(); map_linet!=end(); map_linet++) {
         for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              graphpoints.push_back(*(points.PointIterator(*nodet)));
         }
         graphlines.push_back(*map_linet);
    } 
    graphpoints.RemoveDoublePoints(graphlines, 0.001);
 // create an adjacency matrix, only keep upper triangle (represented in a vector of 1/2 * (n x n)-n)
    graph_vector.resize(int(0.5 * ((graphpoints.size()*graphpoints.size())-graphpoints.size())),0);
    for (it=0; it<graph_vector.size();it++) graph_vector[it]=0;
    for (map_linet=graphlines.begin(); map_linet!=graphlines.end(); map_linet++){
         nodet=map_linet->begin();
         sn1t = graphpoints.PointIterator(*nodet)->Number(); nodet++;
         sn2t = graphpoints.PointIterator(*nodet)->Number();
         for (it=0; it<graphpoints.size();it++){ //find point number in location in graph_points
              if (graphpoints[it].Number()==sn1t) iit = it;
              if (graphpoints[it].Number()==sn2t) jjt = it;
         } 
         if (jjt<iit) jt=jjt, jjt=iit, iit=jt;//swap ii and jj; start in graph with the smallest
         if (label) {
                    graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = map_linet->Label(); //put label in adjacency matrix
                    }
         else {
              if (number) graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = map_linet->Number(); //put number in adjacency matrix
              else graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = 1; //put "1" in adjacency matrix
              }
        }
        graphpoints.erase(graphpoints.begin(), graphpoints.end());
        graphlines.erase(graphlines.begin(), graphlines.end());
  return graph_vector;
  }
   
/*
--------------------------------------------------------------------------------
               Transform Linetopologies to adjacency matrix
--------------------------------------------------------------------------------
*/

FRSmatrix<int> LineTopologies::TransformToGraphMatrix(ObjectPoints points, bool label, bool number)

{
 FRSmatrix<int>               graph_matrix, nullmatrix;
 ObjectPoints                 graphpoints;
 LineTopologies               graphlines;
 LineTopologies::iterator     map_linet; 
 LineTopology::iterator       nodet;
 int                          it, jt, iit, jjt, sn1t, sn2t;
  
  if (label && number){
            printf ("Error: only one of two attributes can be transformed in graph\n");
            return nullmatrix;
            }
  if (!label && !number){
             printf ("simple adjacancy graph (with 0-s and 1-s) will be created.\n");
             }
 // select only the nodes that are in these linetopologies
    for (map_linet= begin(); map_linet!=end(); map_linet++) {
         for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              graphpoints.push_back(*(points.PointIterator(*nodet)));
         }
         graphlines.push_back(*map_linet);
    } 
    graphpoints.RemoveDoublePoints(graphlines, 0.001);

 // create an adjacency matrix 
    graph_matrix = FRSmatrix<int>(graphpoints.size(),graphpoints.size());
    graph_matrix.Zero();
    
    for (map_linet=graphlines.begin(); map_linet!=graphlines.end(); map_linet++){
         nodet=map_linet->begin();
         sn1t = graphpoints.PointIterator(*nodet)->Number(); nodet++;
         sn2t = graphpoints.PointIterator(*nodet)->Number();
         for (it=0; it<graphpoints.size();it++){ //find point number in location in graph_points
              if (graphpoints[it].Number()==sn1t) iit = it;
              if (graphpoints[it].Number()==sn2t) jjt = it;
         } 
         if (jjt<iit) jt=jjt, jjt=iit, iit=jt;//swap ii and jj; start in graph with the smallest
         if (label) {
//                    graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = map_linet->Label(); //put label in adjacency matrix
                    graph_matrix[iit][jjt] = map_linet->Label(); //put label in adjacency matrix
                    graph_matrix[jjt][iit] = map_linet->Label(); //matrix is symmetric
                    graph_matrix[iit][iit] = sn1t; //put node number on diagonal; just for information purp..
                    graph_matrix[jjt][jjt] = sn2t;
                    }
         else {
    //          if (number) graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = map_linet->Number(); //put number in adjacency matrix
              if (number) {
                          graph_matrix[iit][jjt] = map_linet->Number(); //put number in adjacency matrix
                          graph_matrix[jjt][iit] = map_linet->Number(); //matrix is symmetric
                          graph_matrix[iit][iit] = sn1t; //put node number on diagonal; just for information purp..
                          graph_matrix[jjt][jjt] = sn2t;
                          }
//              else graph_vector[iit*(graphpoints.size()-1) + jjt -1 - int(0.5*(iit*iit + iit))] = 1; //put "1" in adjacency matrix
              else {
                   graph_matrix[iit][jjt] = 1; //put number in adjacency matrix
                   graph_matrix[jjt][iit] = 1; //matrix is symmetric
                   graph_matrix[iit][iit] = sn1t; //put node number on diagonal; just for information purp..
                   graph_matrix[jjt][jjt] = sn2t;
                   }
              }
        }
        graphpoints.erase(graphpoints.begin(), graphpoints.end());
        graphlines.erase(graphlines.begin(), graphlines.end());
  return graph_matrix;
  }
         
/*
--------------------------------------------------------------------------------
               Return end points of linetopologies
--------------------------------------------------------------------------------
*/

ObjectPoints LineTopologies::ReturnEndPointsOfTopologies(ObjectPoints points)

{
 FRSmatrix<int>               count_matrix, point_matrix;
 ObjectPoints                 graphpoints;
 LineTopologies               graphlines;
 LineTopologies::iterator     map_linet; 
 LineTopology::iterator       nodet;
 int                          it, jt, iit, jjt, sn1t, sn2t;
// PointNumberList              endpoints;
 ObjectPoints                    endpoints;
  
 // select only the nodes that are in these linetopologies
    for (map_linet= begin(); map_linet!=end(); map_linet++) {
         for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              graphpoints.push_back(*(points.PointIterator(*nodet)));
         }
         graphlines.push_back(*map_linet);
    } 
    graphpoints.RemoveDoublePoints(graphlines, 0.01);
 //   endpoints = PointNumberList();
    endpoints.erase(endpoints.begin(), endpoints.end());
 // create an adjacency matrix 
    count_matrix = FRSmatrix<int>(graphpoints.size(),graphpoints.size());
    count_matrix.Zero();
    point_matrix = FRSmatrix<int>(graphpoints.size(),graphpoints.size());
    point_matrix.Zero();
    for (map_linet=graphlines.begin(); map_linet!=graphlines.end(); map_linet++){
         nodet=map_linet->begin();
         if (map_linet->size()>0){
           sn1t = graphpoints.PointIterator(*nodet)->Number(); nodet++;
           sn2t = graphpoints.PointIterator(*nodet)->Number();
           for (it=0; it<graphpoints.size();it++){ //find point number in location in graph_points
                if (graphpoints[it].Number()==sn1t) iit = it;
                if (graphpoints[it].Number()==sn2t) jjt = it;
           }  
           count_matrix[iit][iit]++;
           count_matrix[jjt][jjt]++;
           point_matrix[iit][iit] = sn1t;
           point_matrix[jjt][jjt] = sn2t;
         }
    }
    for (it=0; it< graphpoints.size();it++){
        if (count_matrix[it][it]==1){
     //     endpoints.push_back(PointNumber(point_matrix[it][it]));
          endpoints.push_back(graphpoints[it]);
           }
    }
    graphpoints.erase(graphpoints.begin(), graphpoints.end());
    graphlines.erase(graphlines.begin(), graphlines.end());
  return endpoints;
  }

/*
--------------------------------------------------------------------------------
               Returns lines that contain a given pointnumber
--------------------------------------------------------------------------------
*/
LineTopologies LineTopologies::ReturnLinesWithPoint(const PointNumber &number) const
{
  LineTopologies::const_iterator polygon;
  LineTopologies                 connected_lines;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (polygon->Contains(number)) connected_lines.push_back(*polygon);
  return connected_lines;
}

/*
--------------------------------------------------------------------------------
               Returns lines that contain a given pointnumber, but not another given pointnumber
--------------------------------------------------------------------------------
*/
LineTopologies LineTopologies::ReturnLinesWithPoint1ButWithoutPoint2(const PointNumber &number, const PointNumber &number2) const
{
  LineTopologies::const_iterator polygon;
  LineTopologies                 connected_lines;

  for (polygon=begin(); polygon!=end(); polygon++)
    if (polygon->Contains(number)&& !polygon->Contains(number2)) connected_lines.push_back(*polygon);
  return connected_lines;
}

/*
--------------------------------------------------------------------------------
               Return the cost between two nodes in linetopologies
--------------------------------------------------------------------------------
*/

int LineTopologies::CostBetweenTwoNodes(ObjectPoints points, const PointNumber &number1, const PointNumber &number2)

{
 ObjectPoints                 graphpoints;
 ObjectPoints::iterator       point;
 LineTopologies               graphlines, bucketlines, connectedlines, nextconnectedlines;
 LineTopologies::iterator     map_linet; 
 LineTopology::iterator       nodet;
 int                          iter;
 PointNumber                  point1, point2;
 bool                         found;
 found = false; 
 // select only the nodes that are in these linetopologies
    for (map_linet= begin(); map_linet!=end(); map_linet++) {
         for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              graphpoints.push_back(*(points.PointIterator(*nodet)));
         }
         graphlines.push_back(*map_linet);
    } 
    graphpoints.RemoveDoublePoints(graphlines, 0.001);

          point1 = number1;
          connectedlines = graphlines.ReturnLinesWithPoint(point1);
          iter = 0;
          do{
          for (map_linet= connectedlines.begin(); map_linet!=connectedlines.end(); map_linet++) {
            iter++;
            for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              if (nodet->Number()!= point1){
                if (nodet->Number()==number2){
                  found = true;
                }
                else {
                   point2 = nodet->Number();
                   nextconnectedlines = graphlines.ReturnLinesWithPoint1ButWithoutPoint2(point2, point1);
                }
              }
            }
            bucketlines.push_back(*map_linet);
          }
          if (!found) connectedlines = nextconnectedlines;
          point1=point2;
          if (connectedlines.size()==0) iter = 100;
          }while (!found && iter<10);
    
          return iter;
  }

/*
--------------------------------------------------------------------------------
               Return the closed polygon of all lines in linetopologies
               Only works for linetopologies that indeed form a closedpolygon
--------------------------------------------------------------------------------
*/

LineTopology LineTopologies::ReturnClosedPolygon(ObjectPoints points)

{
 ObjectPoints                 graphpoints;
 ObjectPoints::iterator       point;
 LineTopologies               graphlines, connectedlines;
 LineTopologies::iterator     map_linet, next_linet;
 LineTopology                 closedline; 
 LineTopology::iterator       nodet;
 int                          iter;
 PointNumber                  startnumber, firstnumber, secnumber, newsecnumber;
 // select only the nodes that are in these linetopologies
    for (map_linet= begin(); map_linet!=end(); map_linet++) {
         for (nodet=map_linet->begin(); nodet!=map_linet->end(); nodet++) {
              graphpoints.push_back(*(points.PointIterator(*nodet)));
         }
         graphlines.push_back(*map_linet);
    } 
    graphpoints.RemoveDoublePoints(graphlines, 0.001);
    closedline.clear();
    closedline.Initialise();
    
    if (graphpoints.size()!=graphlines.size()){
                                                 closedline.SetAttribute(LineLabelTag, 0);
                                                 return closedline;
                                                 }
    else{

          map_linet= graphlines.begin(); 
          nodet=map_linet->begin();
          startnumber = nodet->Number();
          firstnumber = startnumber;
          closedline.push_back(startnumber);
          nodet++;
          secnumber = nodet->Number();
          closedline.push_back(secnumber);
          do {
              connectedlines = graphlines.ReturnLinesWithPoint1ButWithoutPoint2(secnumber, firstnumber);
              next_linet = connectedlines.begin();
              for (nodet=next_linet->begin(); nodet!=next_linet->end(); nodet++) {
                if (nodet->Number()!= secnumber){
                 newsecnumber = nodet->Number();
                 closedline.push_back(newsecnumber);
                }
              }
              firstnumber = secnumber;
              secnumber = newsecnumber;
          } while (secnumber!=startnumber);
          }
          closedline.SetAttribute(LineLabelTag, 0);
          closedline.SetAttribute(SegmentLabel, map_linet->Attribute(SegmentLabel));
          return closedline;    
  }

/*
--------------------------------------------------------------------------------
               Add walls between roof and floor points
               Same as other, but now return 2 triangles
               instead of 1 rectangle per edge
--------------------------------------------------------------------------------
*/

void LineTopologies::AddTINWalls(const LineTopologies &roof_outlines,
                              int number_offset)
{
  int wall_number=0;
  LineTopologies::const_iterator roof_outline;
  LineTopology::const_iterator node;
  LineTopology wall;

  if (!empty()) wall_number = (end()-1)->Number();
  for (roof_outline=roof_outlines.begin();
       roof_outline!=roof_outlines.end(); roof_outline++) {
    wall_number++;  wall.Number() = wall_number;
    for (node=roof_outline->begin(); node!=roof_outline->end()-1; node++) {
      wall.push_back(*node);
      wall.push_back(PointNumber(node->Number() + number_offset));
      wall.push_back(PointNumber((node+1)->Number() + number_offset));
//      wall.push_back(*(node+1)); leave this point out
      wall.push_back(*node);
      wall.Label() = roof_outline->Label();
      push_back(wall);
      wall.erase(wall.begin(), wall.end());
      wall.push_back(*node);
//      wall.push_back(PointNumber(node->Number() + number_offset)); leave this point out
      wall.push_back(PointNumber((node+1)->Number() + number_offset));
      wall.push_back(*(node+1));
      wall.push_back(*node);
      wall.Label() = roof_outline->Label();
      push_back(wall);
      wall.erase(wall.begin(), wall.end());
    }
  }
}

