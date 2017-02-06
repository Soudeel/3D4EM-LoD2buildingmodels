
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
 Collection of functions for class LineTopology

 LineTopology& LineTopology::operator=(const LineTopology&)   Copy assignment
 void LineTopology::Cpp2C(LineTop **)     Conversion of C++ class to C structure
 void LineTopology::C2Cpp(LineTop *)      Conversion of C structure to C++ class
 void LineTopology::DeletePoint(PointNumber *)       Delete a point from list
 bool LineTopology::Adjacent                    Check adjacency between polygons
   (const LineTopology &, int) const
 bool LineTopology::CommonEdge                  Check if two polygons have an
   (const LineTopology &) const                 edge in common
 const PointNumber * LineTopology::NextNode     Return pointer to next node
   (const PointNumber *) const
 PointNumber * LineTopology::NextNode           Return pointer to next node
   (const PointNumber *)
 const PointNumber * LineTopology::PreviousNode Return pointer to previous node
   (const PointNumber *) const
 PointNumber * LineTopology::PreviousNode       Return pointer to previous node
   (const PointNumber *)
 const PointNumber * LineTopology::NodePointer  Return pointer to node with
   (int) const                                  specified point number
 PointNumberList::iterator                      Return iterator to next node
   LineTopology::NextNode
   (PointNumberList::iterator)
 PointNumberList::const_iterator                Return iterator to next node
   LineTopology::NextNode
   (PointNumberList::const_iterator) const
 PointNumberList::iterator                      Return iterator to previous node
   LineTopology::PreviousNode
   (PointNumberList::iterator)
 PointNumberList::const_iterator                Return iterator to previous node
   LineTopology::PreviousNode
   (PointNumberList::const_iterator) const
 PointNumberList::const_iterator                Return iterator to node with
   LineTopology::NodeIterator(int) const        specified point number
 bool LineTopology::Merge                       Merge two polygons
   (const LineTopology &, LineTopology &) const
 bool LineTopology::Split                       Split a polygon in two open pols
   (const PointNumber &, LineTopologies &) const
 bool LineTopology::Split                       Split a polygon in two closed p.
   (const PointNumber &, const PointNumber &,
    LineTopologies &) const
 void LineTopology::InsertNodes                 Insert nodes for points on 
   (const ObjectPoints &, double)               edges of the polygon
 void LineTopology::InsertNodes                 Insert nodes of a polygon
   (const ObjectPoints &,                       located on the edges of this
    const LineTopology &, double)               polygon
 void LineTopology::RemoveCollinearNodes        Remove collinear nodes
   (const ObjectPoints &)
 DataBounds2D & LineTopology::Bounds            Determine polygon bounds
   (const ObjectPoints &) const
 void LineTopology::AddToBitmap                 Add a polygon to a bitmap image
   (const ObjectPoints &, Image &,
    const ImageGrid &) const
 int LineTopology::IntersectPolygonByLine       Intersect a polygon by a line
   (const ObjectPoints &, const Line2D &,
    LineSegments2D &, double, double, int)
 bool LineTopology::SplitPolygonByLine
   (ObjectPoints &, const Line2D &,             Split a polygon by a line
    double, double, LineTopologies &) const
 bool LineTopology::SplitPolygonByLineSegment
   (ObjectPoints &, const LineSegment2D &,      Split a polygon by a line segm.
    double, LineTopologies &) const
 LineTopology::LineTopology                     Convert a TIN mesh to a polygon
   (const TINMesh &, int, int)
 Vector3D LineTopology::Normal                  Derive surface normal from 
   (ObjectPoints &) const                       the first three polygon nodes
 void LineTopology::Densify(ObjectPoints &,     Densify a polygon by inserting
                            double)             new nodes on long edges

 Initial creation
 Author : Ildiko Suveg
 Date   : 24-11-1998

 Update #1
 Author : George Vosselman
 Date   : 10-03-1999
 Changes: Added copying of the added label.

 Update #2
 Author : George Vosselman
 Date   : 23-05-2003
 Changes: Added functions Adjacent until Densify

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
#include "LineTopology.h"
#include "LineSegments2D.h"
#include "LineSegment3D.h"
#include "ObjectPoints2D.h"
#include "DataBounds2D.h"
#include "DataBounds3D.h"
#include "Image.h"
#include "ImageGrid.h"
#include "ImagePoint.h"
#include "Line2D.h"
#include "TINMesh.h"


/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/

LineTopology& LineTopology::operator=(const LineTopology& linetop)
{
  // Check for self assignment
  if (this == &linetop) return *this;
  // Copy line node numbers
  if (!empty()) erase(begin(), end());
  reserve(linetop.size());
  insert(begin(), linetop.begin(), linetop.end());
  // Copy line number
  num = linetop.num;
  // Copy line attributes
  if (linetop.num_attributes > 0) {
    if (linetop.num_attributes > num_attributes) {
      if (num_attributes > 0) {
        free(tags);
        free(attributes);
      }
      tags = (unsigned char *) malloc(linetop.num_attributes * 
                                      sizeof(unsigned char));
      attributes = (int *) malloc(linetop.num_attributes * sizeof(int));
    }
    memcpy((void *) tags, (const void *) linetop.tags,
           linetop.num_attributes * sizeof(unsigned char));
    memcpy((void *) attributes, (const void *) linetop.attributes,
           linetop.num_attributes * sizeof(int));
  }
  else {
    if (num_attributes > 0) {
      free(tags);
      free(attributes);
    }
    tags       = NULL;
    attributes = NULL;
  }
  num_attributes = linetop.num_attributes;   
  return *this;			
}








/*
--------------------------------------------------------------------------------
                     Conversion of C++ class to C structure
--------------------------------------------------------------------------------
*/

void LineTopology::Cpp2C(LineTop **lineptr) const
{
  LineTop *line;

/* Allocate space if this has not been done yet */

  line = *lineptr;
  if (line == NULL) {
    line = (LineTop *) malloc(sizeof(LineTop));
    line->pts = (int*) malloc(size() * sizeof(int));
    *lineptr = line;
  }

/* Copy the data from the C++ to the C object */

  line->num = num;
  line->num_pts = size();
  line->label = Attribute(LineLabelTag);

  LineTopology::const_iterator i;
  int l = 0;
  for (i = begin(); i != end(); i++, l++)
  	line->pts[l] = i->Number();
}

/*
--------------------------------------------------------------------------------
                     Conversion of C structure to C++ class
--------------------------------------------------------------------------------
*/

void LineTopology::C2Cpp(LineTop *line)
{
  
  if (!empty()) erase(begin(), end());
  reserve(line->num_pts);
  
  
  num = line->num;  

  SetAttribute(LineLabelTag,line->label); 

  for(int i = 0; i < line->num_pts; i++) 	
  	push_back(line->pts[i]);
  	
}

/*
--------------------------------------------------------------------------------
                     Delete a point from the list
--------------------------------------------------------------------------------
*/

void LineTopology::DeletePoint(const PointNumber *ptnr)
{
  LineTopology::iterator i;

  int k = 0;
  for(i = begin(); i != end(); i++, k++)
  {
 	if (ptnr->Number() == i->Number())
 	    erase(i);
  }  
}

/*
--------------------------------------------------------------------------------
                     Check adjacency of two polygons
--------------------------------------------------------------------------------
*/

bool LineTopology::Adjacent(const LineTopology &polygon,
                            int min_num_common_nodes) const
{
  PointNumberList::const_iterator node1, last_node1, node2, last_node2;
  int                             num_common_nodes = 0;

  last_node1 = end() - 1;
  if (begin()->Number() == last_node1->Number()) last_node1--;
  last_node2 = polygon.end() - 1;
  if (polygon.begin()->Number() == last_node2->Number()) last_node2--;
  for (node1=begin(); node1<=last_node1; node1++)
    for (node2=polygon.begin(); node2<=last_node2; node2++)
      if (node1->Number() == node2->Number()) num_common_nodes++;
  return(num_common_nodes >= min_num_common_nodes);
}

/*
--------------------------------------------------------------------------------
                     Check if two polygons share an edge
--------------------------------------------------------------------------------
*/

bool LineTopology::CommonEdge(const LineTopology &polygon) const
{
  PointNumberList::const_iterator node, common_node, start_node1, start_node2,
                                  not_common_node;
  int                             i;

  if (size() < 2 || polygon.size() < 2) return(0);

/* Look for a node not part of the second polygon */

  for (node=begin(), not_common_node=end();
       node!=end() && not_common_node==end();
       node++)
    if (!polygon.Contains(*node)) not_common_node = node;
  if (not_common_node==end()) return(1); // All nodes are the same

/* Look for the next common node */

  for (node=not_common_node, common_node=end(), i=0;
       i<size() && common_node==end();
       node=NextNode(node), i++)
    if (polygon.Contains(*node)) common_node = node;
  if (common_node==end()) return(0); // No common node

/* Check if the next node is also part of the other polygon */

  start_node1 = common_node;
  start_node2 = polygon.NodeIterator(common_node->Number());
  if (NextNode(start_node1)->NumberRef() ==
      polygon.NextNode(start_node2)->NumberRef() ||
      NextNode(start_node1)->NumberRef() ==
      polygon.PreviousNode(start_node2)->NumberRef())
    return(1);
  else return(0);
}

/*
--------------------------------------------------------------------------------
               Return the common part of two polygons
--------------------------------------------------------------------------------
*/

LineTopology & LineTopology::CommonPart(const LineTopology &polygon) const
{
  LineTopology                 *common_part;
  LineTopology::const_iterator common_node, not_common_node, node1, node2;
  int                          i;
  bool                         same_order;
  
  common_part = new LineTopology();
  if (!CommonEdge(polygon)) return common_part->LineTopologyReference();
  
  // Look for a node not part of the second polygon
  for (node1=begin(), not_common_node=end();
       node1!=end() && not_common_node==end(); node1++)
    if (!polygon.Contains(*node1)) not_common_node = node1;
    
  // Check if all nodes are shared. If so, return a copy of the polygon
  if (not_common_node == end()) {
    common_part->insert(common_part->begin(), begin(), end());
    return common_part->LineTopologyReference();
  }  

  // Look for the next common node
  for (node1=not_common_node, common_node=end(), i=0;
       i<size() && common_node==end();
       node1=NextNode(node1), i++)
    if (polygon.Contains(*node1)) common_node = node1;

  // Store the common nodes
  node1 = common_node;
  node2 = polygon.NodeIterator(common_node->Number());
  same_order = (NextNode(node1)->Number() ==
                polygon.PreviousNode(node2)->Number());
  while (node1->Number() == node2->Number() &&
         common_part->size() != size()) {
    common_part->push_back(*node1);
    node1 = NextNode(node1);
    if (same_order) node2 = polygon.PreviousNode(node2);
    else node2 = polygon.NextNode(node2);
  }
  return common_part->LineTopologyReference();  
}

/*
--------------------------------------------------------------------------------
               Return all common parts of two polygons
--------------------------------------------------------------------------------
*/

LineTopologies LineTopology::AllCommonParts(const LineTopology &polygon) const
{
  LineTopology                 *common_part;
  LineTopologies               common_parts;
  LineTopology::const_iterator common_node, not_common_node, node1, node2,
                               keep_not_common_node, nodestore;
  int                          i, iter=0;
  bool                         same_order;
  
//  common_part = new LineTopology();
  if (!CommonEdge(polygon)) return common_parts;//->LineTopologyReference();
  keep_not_common_node = begin();
//  printf("sizes %d and %d\n", size(), polygon.size());
  do {
  // Look for a node not part of the second polygon
   iter=0;
    common_part = new LineTopology();

  for (node1=begin(), not_common_node=end();
       node1!=end() && not_common_node==end(); node1++)
    if (!polygon.Contains(*node1)) not_common_node = node1;
    
  // Check if all nodes are shared. If so, return a copy of the polygon
  if (not_common_node == end()) {
    common_part->insert(common_part->begin(), begin(), end());
    //return common_part->LineTopologyReference();
    common_parts.push_back(*common_part);
   // printf("complete common part");
    return common_parts;
  }  

  // Look for the next common node
  for (node1=not_common_node, common_node=end(), i=0;
       i<size() && common_node==end();
       node1=NextNode(node1), i++)
    if (polygon.Contains(*node1)&&!common_parts.Contains(*node1)) {
                                  common_node = node1;
 //                                 printf("i=%d ",i);
                                  }

  // Store the common nodes
  node1 = common_node;
  nodestore = common_node;
  node2 = polygon.NodeIterator(common_node->Number());
  same_order = (NextNode(node1)->Number() ==
                polygon.PreviousNode(node2)->Number());
  while (node1->Number() == node2->Number() &&
         common_part->size() != size() && common_parts.Contains(*node1)==0) {
    common_part->push_back(*node1);
    node1 = NextNode(node1);
    if (same_order) node2 = polygon.PreviousNode(node2);
    else node2 = polygon.NextNode(node2);
  }
  if (common_part->size()>1) common_parts.push_back(*common_part);
  else return common_parts;
//  node1=NextNode(node1);
//  if (keep_not_common_node == node1 || common_parts.Contains(*node1)) return common_parts;
//  keep_not_common_node = NextNode(nodestore) ;
//  keep_not_common_node = node1;  
 // printf("         number %d, size of commonpart %d, iter %d\n", keep_not_common_node->Number(), common_part->size(), iter);
 // keep_not_common_node++;
  iter++;
  } while (node1 != end() && iter<20);//somehow there was a unfinished loop, now it stops after at most 20 iterations 
  return common_parts;  
}

/*
--------------------------------------------------------------------------------
              Check the orientation of two polygons with a common edge
     @param polygon The other polygon.
        @return 1 if the two polygons have the same orientation, 0 otherwise.
    
--------------------------------------------------------------------------------
*/

int LineTopology::SameOrientation(const LineTopology &polygon) const
{
  PointNumberList::const_iterator node, common_node, start_node1, start_node2,
                                  not_common_node;
  int                             i;

  if (size() < 2 || polygon.size() < 2) return(0);

/* Look for a node not part of the second polygon */

  for (node=begin(), not_common_node=end();
       node!=end() && not_common_node==end();
       node++)
    if (!polygon.Contains(*node)) not_common_node = node;
  if (not_common_node==end()) return(1); // All nodes are the same

/* Look for the next common node */

  for (node=not_common_node, common_node=end(), i=0;
       i<size() && common_node==end();
       node=NextNode(node), i++)
    if (polygon.Contains(*node)) common_node = node;
  if (common_node==end()) return(0); // No common node

/* Check if the next node is also part of the other polygon */

  start_node1 = common_node;
  start_node2 = polygon.NodeIterator(common_node->Number());
  if (NextNode(start_node1)->NumberRef() ==
      polygon.NextNode(start_node2)->NumberRef() ||
      NextNode(start_node1)->NumberRef() ==
      polygon.PreviousNode(start_node2)->NumberRef()){
      if (NextNode(start_node1)->NumberRef() ==
      polygon.NextNode(start_node2)->NumberRef())return(0);
      if (NextNode(start_node1)->NumberRef() ==
      polygon.PreviousNode(start_node2)->NumberRef())return(1);
      }
  else return(10);
}


/*
--------------------------------------------------------------------------------
        Pointers to previous and next node and node with specified number
--------------------------------------------------------------------------------
*/

const PointNumber * LineTopology::NextNode(const PointNumber *node) const
{
  const PointNumber *next_node;

  if (size() < 2) return(NULL);
  next_node = node + 1;
  if (next_node == &*end()) {
    next_node = &*begin();
    if (*next_node == *node) next_node++;
  }
  return(next_node);
}

PointNumber * LineTopology::NextNode(PointNumber *node)
{
  PointNumber *next_node;

  if (size() < 2) return(NULL);
  next_node = node + 1;
  if (next_node == &*end()) {
    next_node = &*begin();
    if (*next_node == *node) next_node++;
  }
  return(next_node);
}

const PointNumber * LineTopology::PreviousNode(const PointNumber *node) const
{
  const PointNumber *previous_node;

  if (size() < 2) return(NULL);
  if (node == &*begin()) {
    previous_node = &*(end() - 1);
    if (*previous_node == *node) previous_node--;
  }
  else previous_node = node - 1;
  return(previous_node);
}

PointNumber * LineTopology::PreviousNode(PointNumber *node)
{
  PointNumber *previous_node;

  if (size() < 2) return(NULL);
  if (node == &*begin()) {
    previous_node = &*(end() - 1);
    if (*previous_node == *node) previous_node--;
  }
  else previous_node = node - 1;
  return(previous_node);
}

const PointNumber * LineTopology::NodePointer(int node_number) const
{
  const PointNumber *node;

  for (node=&*begin(); node!=&*end(); node++)
    if (node->Number() == node_number) return(node);
  return(NULL);
}

PointNumber * LineTopology::NodePointer(int node_number)
{
  PointNumber *node;

  for (node=&*begin(); node!=&*end(); node++)
    if (node->Number() == node_number) return(node);
  return(NULL);
}


/*
--------------------------------------------------------------------------------
        Iterators to previous and next node and node with specified number
--------------------------------------------------------------------------------
*/

PointNumberList::const_iterator
  LineTopology::NextNode(PointNumberList::const_iterator node) const
{
  PointNumberList::const_iterator next_node;

  if (size() < 2) return(end());
  next_node = node + 1;
  if (next_node == end()) {
    next_node = begin();
    if (*next_node == *node) next_node++;
  }
  return(next_node);
}

PointNumberList::iterator LineTopology::NextNode(PointNumberList::iterator node)
{
  PointNumberList::iterator next_node;

  if (size() < 2) return(end());
  next_node = node + 1;
  if (next_node == end()) {
    next_node = begin();
    if (*next_node == *node) next_node++;
  }
  return(next_node);
}

PointNumberList::const_iterator
  LineTopology::PreviousNode(PointNumberList::const_iterator node) const
{
  PointNumberList::const_iterator previous_node;

  if (size() < 2) return(end());
  if (node == begin()) {
    previous_node = end() - 1;
    if (*previous_node == *node) previous_node--;
  }
  else previous_node = node - 1;
  return(previous_node);
}

PointNumberList::iterator
  LineTopology::PreviousNode(PointNumberList::iterator node)
{
  PointNumberList::iterator previous_node;

  if (size() < 2) return(end());
  if (node == begin()) {
    previous_node = end() - 1;
    if (*previous_node == *node) previous_node--;
  }
  else previous_node = node - 1;
  return(previous_node);
}

PointNumberList::const_iterator
  LineTopology::NodeIterator(int node_number) const
{
  PointNumberList::const_iterator node;

  for (node=begin(); node!=end(); node++)
    if (node->Number() == node_number) return(node);
  return(end());
}

PointNumberList::iterator LineTopology::NodeIterator(int node_number)
{
  PointNumberList::iterator node;

  for (node=begin(); node!=end(); node++)
    if (node->Number() == node_number) return(node);
  return(end());
}
/*
--------------------------------------------------------------------------------
                              Merge two polygons
--------------------------------------------------------------------------------
*/

bool LineTopology::Merge(const LineTopology &pol2, LineTopology &pol_merged)
     const
{
  PointNumberList::const_iterator node, common_node, start_node1, start_node2,
                                  not_common_node, end_node1, end_node2;
  int                             i, partof;
  const LineTopology              *pol1ptr, *pol2ptr;
  PointNumberList::iterator       node1;
  LineTopology::const_iterator    prev_common_node, next_common_node;

/* Check if the polygons have a common edge, and clean the merged polygon */

  if (!CommonEdge(pol2)) return(0);
  if (!pol_merged.empty())
    pol_merged.erase(pol_merged.begin(), pol_merged.end());

// First check if we have a special case: the nodes of one polygon are all part
// of the nodes of the other polygon.

  partof = IsPartOf(pol2.PointNumberListReference());
  if (partof) {
    pol1ptr = &pol2;
    pol2ptr = this;
  }
  else {
    partof = pol2.IsPartOf(PointNumberListReference());
    if (partof) {
      pol1ptr = this;
      pol2ptr = &pol2;
    }
  }

// Deal with this special case first

  if (partof) {
    // Look for a node not part of the second polygon
    for (node=pol1ptr->begin(), not_common_node=pol1ptr->end();
         node!=pol1ptr->end() && not_common_node==pol1ptr->end();
         node++)
      if (!pol2ptr->Contains(*node)) not_common_node = node;
    pol_merged = *pol1ptr;
    // Check if polygons are identical
    if (not_common_node == pol1ptr->end()) return 1;
    // Starting from the not common node, locate the previous and next
    // common node number.
    prev_common_node = next_common_node = not_common_node;
    while (!pol2ptr->Contains(*prev_common_node))
      prev_common_node = pol1ptr->PreviousNode(prev_common_node);
    while (!pol2ptr->Contains(*next_common_node))
      next_common_node = pol1ptr->NextNode(next_common_node);
    // Remove all common nodes except the two nodes at the ends of the
    // common segment (as determined above)
    for (node1=pol_merged.begin(); node1!=pol_merged.end(); node1++) {
      if (pol2ptr->Contains(*node1) &&
          node1->Number() != prev_common_node->Number() &&
          node1->Number() != next_common_node->Number()) {
        pol_merged.erase(node1);
        node1--;
      }
    }
    return 1;
  }

/* Look for a node not part of the second polygon */

  for (node=begin(), not_common_node=end();
       node!=end() && not_common_node==end();
       node++)
    if (!pol2.Contains(*node)) not_common_node = node;

  if (not_common_node==end()) {
//  pol_merged = *this; // Old
//  return(1); // Old
//  not_common_node = begin(); // New
    // This should not happen anymore
    printf("Bug in LineTopology::Merge\n");
  }

/* Look for the first common node */

  for (node=not_common_node, common_node=end(), i=0;
       i<size() && common_node==end();
       node=NextNode(node), i++)
    if (pol2.Contains(*node)) common_node = node;
  if (common_node==end()) return(0); // No common node

/* Get the start nodes of the common segment in both polygons */

  start_node1 = common_node;
  start_node2 = pol2.NodeIterator(common_node->Number());

/* Distinguish two cases: same node order and reverse node order */

  // Same node order
  if (NextNode(start_node1)->NumberRef() ==
      pol2.NextNode(start_node2)->NumberRef()) {

/* Find the last common nodes in both polygons */

    end_node1 = NextNode(start_node1);
    end_node2 = pol2.NextNode(start_node2);
    while (end_node1->Number() == end_node2->Number()) {
      end_node1 = NextNode(end_node1);
      end_node2 = pol2.NextNode(end_node2);
    }
    end_node1 = PreviousNode(end_node1);
    end_node2 = pol2.PreviousNode(end_node2);

/* Construct the merged polygon */

    pol_merged.push_back(*end_node1);
    node = end_node1;
    do {
      node = NextNode(node);
      pol_merged.push_back(*node);
    } while (node->Number() != start_node1->Number());
    node = start_node2;
    do {
      node = pol2.PreviousNode(node);
      pol_merged.push_back(*node);
    } while (node->Number() != end_node2->Number());
  }

  else { // Reverse node order

/* Find the last common nodes in both polygons */

    end_node1 = NextNode(start_node1);
    end_node2 = pol2.PreviousNode(start_node2);
    while (end_node1->Number() == end_node2->Number()) {
      end_node1 = NextNode(end_node1);
      end_node2 = pol2.PreviousNode(end_node2);
    }
    end_node1 = PreviousNode(end_node1);
    end_node2 = pol2.NextNode(end_node2);

/* Construct the merged polygon */

    pol_merged.push_back(*end_node1);
    node = end_node1;
    do {
      node = NextNode(node);
      pol_merged.push_back(*node);
    } while (node->Number() != start_node1->Number());
    node = start_node2;
    do {
      node = pol2.NextNode(node);
      pol_merged.push_back(*node);
    } while (node->Number() != end_node2->Number());
  }

  pol_merged.Number() = Number();
  pol_merged.Label() = Label();
  return(1);
}

/*
--------------------------------------------------------------------------------
                  Split a polygon at a specified node number
--------------------------------------------------------------------------------
*/

bool LineTopology::Split(const PointNumber &number,
                         LineTopologies &new_polygons) const
{
  int          index;
  LineTopology new_polygon;

  // Check if splitting is possible
  index = FindPoint(number);
  if (index == -1) return false;         // No such point number in this polygon
  if (index == 0)  return false;         // Can not split at first point
  if (index == size() - 1) return false; // Can not split at last point

  // Construct first part
  new_polygon.Number() = 0;
  new_polygon.Label()  = Label();
  new_polygon.insert(new_polygon.end(), begin(), begin() + index + 1);
  new_polygons.push_back(new_polygon);

  // Construct second part
  new_polygon.erase(new_polygon.begin(), new_polygon.end());
  new_polygon.insert(new_polygon.end(), begin() + index, end());
  new_polygons.push_back(new_polygon);
 
  return true;
}

/*
--------------------------------------------------------------------------------
                  Split a closed polygon into two closed polygons
--------------------------------------------------------------------------------
*/

bool LineTopology::Split(const PointNumber &number1, const PointNumber &number2,
                         LineTopologies &new_polygons) const
{
  int                    index1, index2, index_swap;
  LineTopology           new_polygon;
  LineTopology::iterator node;

  if (!IsClosed()) return false;         // Closed polygon needed
  if (number1 == number2) return false;  // Two different node numbers needed

  index1 = FindPoint(number1);
  index2 = FindPoint(number2);
  if (index1 == -1 || index2 == -1) return false; // Node number not in polygon
  if (index1 > index2)
    {index_swap = index1;  index1 = index2;  index2 = index_swap;}

// Create the first new polygon
  new_polygon.Number() = 0;
  new_polygon.Label()  = Label();
  new_polygon.insert(new_polygon.begin(), begin() + index2, end());
  if (index1 > 0) new_polygon.insert(new_polygon.end(), begin() + 1,
                                     begin() + 1 + index1);
  new_polygon.push_back(*new_polygon.begin()); // Close polygon
  new_polygons.push_back(new_polygon);

// Create the second new polygon
  new_polygon.erase(new_polygon.begin(), new_polygon.end());
  new_polygon.Number() = 1;
  new_polygon.insert(new_polygon.end(), begin() + index1, begin() + index2 + 1);
  new_polygon.push_back(*new_polygon.begin()); // Close polygon
  new_polygons.push_back(new_polygon);

  return true;
}

/*
--------------------------------------------------------------------------------
          Insert nodes for points on or near segments of the polygon
--------------------------------------------------------------------------------
*/

void LineTopology::InsertNodes(const ObjectPoints &points, double distance)
{
  LineSegment3D                segment;
  PointNumberList::iterator    node1, node2;
  ObjectPoint                  *point1, *point2;
  ObjectPoints::const_iterator point;
  int                          inserted;

  if (size() < 2) return;

  do {
    inserted = 0;
    node1 = begin();
    point1 = points.GetPoint(node1->NumberRef());
    if (!point1) {
      fprintf(stderr, "Error: Point %d not in list of object points\n",
              node1->Number());
      exit(0);
    }
    for (node2=node1+1; node2!=end() && !inserted; node2++) {

/* Create a segment of two successive points */

      point2 = points.GetPoint(node2->NumberRef());
      if (!point2) {
        fprintf(stderr, "Error: Point %d not in list of object points\n",
                node2->Number());
        exit(0);
      }
      segment = LineSegment3D(point1->Position3DRef(), point2->Position3DRef());

/* See if there are other points nearby */

      for (point=points.begin(); point!=points.end() && !inserted; point++) {
        if (point->Number() != node1->Number() &&
            point->Number() != node2->Number()) {
          if (segment.Distance(point->Position3DRef()) <= distance &&
              segment.Scalar(point->Position3DRef()) > segment.ScalarBegin() &&
              segment.Scalar(point->Position3DRef()) < segment.ScalarEnd()) {
            insert(node2, point->NumberRef());
            inserted = 1;
          }
        }
      }
    
      node1 = node2;
      point1 = point2;
    }
  } while (inserted);
}

/*
--------------------------------------------------------------------------------
      Insert nodes of a polygon located on the edges of this polygon
--------------------------------------------------------------------------------
*/

void LineTopology::InsertNodes(const ObjectPoints &points, 
                               const LineTopology &pol, double distance)
{
  ObjectPoints                 pol_points;
  ObjectPoint                  *point;
  LineTopology::const_iterator node;

  // Create a vector with the points of both polygons only
  for (node=begin(); node!=end(); node++) {
    point = points.GetPoint(node->NumberRef());
    if (!point) {
      fprintf(stderr, "Error: point %d not found in point vector\n",
              node->Number());
      return;
    }
    // Insert the point if it's not yet in the vector
    if (!pol_points.GetPoint(node->NumberRef())) pol_points.push_back(*point);
  }
  for (node=pol.begin(); node!=pol.end(); node++) {
    point = points.GetPoint(node->NumberRef());
    if (!point) {
      fprintf(stderr, "Error: point %d not found in point vector\n",
              node->Number());
      return;
    }
    // Insert the point if it's not yet in the vector
    if (!pol_points.GetPoint(node->NumberRef())) pol_points.push_back(*point);
  }

  // Try to insert these points into the current polygon
  InsertNodes(pol_points, distance);
}

/*
--------------------------------------------------------------------------------
                          Remove collinear nodes
--------------------------------------------------------------------------------
*/
void LineTopology::RemoveCollinearNodes(const ObjectPoints &points, bool on3D, double err)
{
	//double err = 0.005;
	if (this->size()<3) return;
	ObjectPoint* pnt1, *pnt2, *pnt3;
	int i,j,k;

	bool isClosed = this->IsClosed();
	if (isClosed) this->erase(this->end()-1);
	vector<int> vecValidNodes(this->size(), true);

	//detect removable points
	if (on3D) {
		Line3D temLine;
		for (i=0; i<this->size()-1; ++i) {
			if (!vecValidNodes[i]) continue;

			j = i+1;
			if (!vecValidNodes[j]) continue;
			pnt1 = points.GetPoint((*this)[i]);
			pnt2 = points.GetPoint((*this)[j]);
			if (!pnt1||!pnt2 || pnt1->Position3DRef()==pnt2->Position3DRef()) continue;
			temLine = Line3D(pnt1->Position3DRef(), pnt2->Position3DRef());

			k = j+1;
			while (k<this->size() && 
				(pnt3=points.GetPoint((*this)[k])) &&
				temLine.DistanceToPoint(*pnt3)<err) 
			{
				vecValidNodes[k-1] = false;
				k++;
			}
		}
	}
	else {
		Line2D temLine;
		for (i=0; i<this->size()-1; ++i) {
			if (!vecValidNodes[i]) continue;

			j = i+1;
			if (!vecValidNodes[j]) continue;
			pnt1 = points.GetPoint((*this)[i]);
			pnt2 = points.GetPoint((*this)[j]);
			if (!pnt1||!pnt2 || pnt1->Position2DOnly()==pnt2->Position2DOnly()) continue;
			temLine = Line2D(pnt1->Position2DOnly(), pnt2->Position2DOnly());

			k = j+1;
			while (k<this->size() && 
				(pnt3=points.GetPoint((*this)[k])) &&
				temLine.DistanceToPoint(pnt3->Position2DOnly())<err) 
			{
				vecValidNodes[k-1] = false;
				k++;
			}
		}
	}


	//do remove
	for (i=0; i<this->size()-1; ++i) {
		if (!vecValidNodes[i]) {
			this->erase(this->begin()+i);
			vecValidNodes.erase(vecValidNodes.begin()+i);
			i--;
		}
	}

	//check start point
	if(isClosed && this->size()>3) {
		if (on3D) {
			Line3D temLine;
			pnt1 = points.GetPoint((*this)[1]);
			pnt2 = points.GetPoint((*this)[this->size()-1]);
			if (pnt1->X()!=pnt2->X() && pnt1->Y()!=pnt2->Y() && pnt1->Z()!=pnt2->Z()) {
				pnt3 = points.GetPoint((*this)[0]);
				temLine = Line3D(pnt1->Position3DRef(), pnt2->Position3DRef());
				if(temLine.DistanceToPoint(*pnt3)<err) 
					this->erase(this->begin());//remove the start point
			}
		}
		else {
			Line2D temLine;
			pnt1 = points.GetPoint((*this)[1]);
			pnt2 = points.GetPoint((*this)[this->size()-1]);
			if (pnt1->X()!=pnt2->X() && pnt1->Y()!=pnt2->Y()) {
				pnt3 = points.GetPoint((*this)[0]);
				temLine = Line3D(pnt1->Position3DRef(), pnt2->Position3DRef());
				if(temLine.DistanceToPoint(pnt3->Position2DOnly())<err) 
					this->erase(this->begin());//remove the start point
			}
		}
	}

	if (isClosed) this->push_back(*begin());
}

/*
--------------------------------------------------------------------------------
                                  Revert the node order
--------------------------------------------------------------------------------
*/

void LineTopology::RevertNodeOrder()
{
  LineTopology::iterator node1, node2;
  PointNumber            swap_node;
  int                    i;

  for (i=0, node1=begin(), node2=end()-1; i<size()/2; i++, node1++, node2--) {
    swap_node = *node1;
    *node1    = *node2;
    *node2    = swap_node;
  }
}

/*
--------------------------------------------------------------------------------
                             Determine the polygon bounds
--------------------------------------------------------------------------------
*/

DataBounds2D & LineTopology::Bounds(const ObjectPoints2D &objpts) const
{
  DataBounds2D                 *bounds = new DataBounds2D();
  LineTopology::const_iterator node;
  ObjectPoint2D                *objpt;

  for (node = begin(); node != end(); node++) {
    objpt = objpts.GetPoint(node->NumberRef());
    if (objpt == NULL)
      printf("Warning: Point %d not found in LineTopology::Bounds(ObjectPoints2D)\n",
             node->Number());
    else bounds->Update(objpt->Position2DRef());
  }
  return *bounds;
}

DataBounds3D & LineTopology::Bounds(const ObjectPoints &objpts) const
{
  DataBounds3D                 *bounds = new DataBounds3D();
  LineTopology::const_iterator node;
  ObjectPoint                  *objpt;

  for (node = begin(); node != end(); node++) {
    objpt = objpts.GetPoint(node->NumberRef());
    if (objpt == NULL)
      printf("Warning: Point %d not found in LineTopology::Bounds(ObjectPoints)\n",
             node->Number());
    else bounds->Update(objpt->Position3DRef());
  }
  return *bounds;
}

/*
--------------------------------------------------------------------------------
                        Add a polygon to a bitmap image
--------------------------------------------------------------------------------
*/

void LineTopology::AddToBitmap(const ObjectPoints &objpts,
                               Image &bitmap, const ImageGrid &grid) const
{
  int            r, c, rmin, rmax, cmin, cmax;
  DataBounds3D   bounds = Bounds(objpts);
  ImagePoint     imgpt;
  ObjectPoint2D  objpt2d;
  double         y;
  LineSegments2D intersections;
  LineSegments2D::iterator intersection;
  Line2D         line;

/*
  // Retrieve the bounds in pixels and make sure they are inside the image
  rmin = (int) ((grid.YOffset() - bounds.Maximum().Y()) / grid.Pixelsize()+0.5);
  if (rmin >= bitmap.NumRows()) return;
  if (rmin < 0) rmin = 0;
  rmax = (int) ((grid.YOffset() - bounds.Minimum().Y()) / grid.Pixelsize()-0.5);
  if (rmax < 0) return;
  if (rmax >= bitmap.NumRows()) rmax = bitmap.NumRows() - 1;
  cmin = (int) ((bounds.Minimum().X() - grid.XOffset()) / grid.Pixelsize()+0.5);
  if (cmin >= bitmap.NumColumns()) return;
  if (cmin < 0) cmin = 0;
  cmax = (int) ((bounds.Maximum().X() - grid.XOffset()) / grid.Pixelsize()-0.5);
  if (cmax < 0) return;
  if (cmax >= bitmap.NumColumns()) cmax = bitmap.NumColumns() - 1;

  // Process all pixels within the bounding box
  for (r = rmin; r <= rmax; r++) {
    for (c = cmin; c <= cmax; c++) {
      imgpt = ImagePoint(1, (double) r, (double) c, 0.0);
      objpt2d = ObjectPoint2D(&imgpt, &grid);
      if (objpt2d.InsidePolygon(objpts, *this))
        *(bitmap.Pixel(r, c)) = 255;
    }
  }
*/

  // Retrieve the range of rows of the polygon in image space
  rmin = (int) ((grid.YOffset() - bounds.Maximum().Y()) / grid.Pixelsize()+0.5);
  if (rmin >= bitmap.NumRows()) return;
  if (rmin < 0) rmin = 0;
  rmax = (int) ((grid.YOffset() - bounds.Minimum().Y()) / grid.Pixelsize()-0.5);
  if (rmax < 0) return;
  if (rmax >= bitmap.NumRows()) rmax = bitmap.NumRows() - 1;

  // Process all rows
  for (r = rmin; r <= rmax; r++) {

    // Create a 2D horizontal line of this row to intersect with the polygon
    y = grid.YOffset() - ((double) r + 0.5) * grid.Pixelsize();
    line = Line2D(1.0, 0.0, y);

    // Calculate all intersecting line segments
    IntersectPolygonByLine(objpts, line, intersections, 1e-6,
                           0.1 * grid.Pixelsize(), r != rmin);

    // Fill all segments in the bitmap
    for (intersection=intersections.begin();
         intersection!=intersections.end(); intersection++) {
      cmin = (int) ((-intersection->ScalarEnd() - grid.XOffset()) /
                    grid.Pixelsize()+0.5);
      if (cmin < 0) cmin = 0;
      cmax = (int) ((-intersection->ScalarBegin() - grid.XOffset()) /
                    grid.Pixelsize()-0.5);
      if (cmax >= bitmap.NumColumns()) cmax = bitmap.NumColumns() - 1;
      for (c = cmin; c <= cmax; c++) *(bitmap.Pixel(r, c)) = 255;
    }

    // Clear the intersection list
    if (!intersections.empty())
      intersections.erase(intersections.begin(), intersections.end());
  }
}

/*
--------------------------------------------------------------------------------
                        Intersect a polygon by a line
--------------------------------------------------------------------------------
*/

int LineTopology::IntersectPolygonByLine(const ObjectPoints &points,
                                         const Line2D &line,
                                         LineSegments2D &intersections,
                                         double err_dist, double min_length,
                                         int reuse_pol_segments) const
{
  static LineSegments2D          segments;
  LineSegments2D::const_iterator segment;
  Positions2D                    int_points;
  Position2D                     int_point;
  Positions2D::iterator          pos, pos1, pos2;
  ObjectPoint2D                  mid_point;
  vector <double>                scalars;
  vector <double>::iterator      scalar, scalar1, scalar2;
  int                            old_num_intersections;

// Clear old data and (re-)compute polygon segments from the point with topology

  if (!reuse_pol_segments) {
    if (!intersections.empty())
      intersections.erase(intersections.begin(), intersections.end());
    if (!segments.empty())
      segments.erase(segments.begin(), segments.end());
    segments = LineSegments2D(points, *this);
  }
  old_num_intersections = intersections.size();

// Collect all intersection positions

  for (segment=segments.begin(); segment!=segments.end(); segment++)
    if (segment->Intersect(line, int_point, err_dist))
      int_points.push_back(int_point);

// Compute the scalars along the line for all intersection points

  for (pos1=int_points.begin(); pos1!=int_points.end(); pos1++)
    scalars.push_back(line.Scalar(*pos1));

// Check all segments between two intersection points

  while (int_points.size() >= 2) {

    // Find the two smallest scalars
    if (scalars[0] < scalars[1]) {
      scalar1 = scalars.begin();  pos1 = int_points.begin();
      scalar2 = scalar1 + 1;      pos2 = pos1 + 1;
    }
    else {
      scalar2 = scalars.begin();  pos2 = int_points.begin();
      scalar1 = scalar2 + 1;      pos1 = pos2 + 1;
    }
    for (scalar=scalars.begin()+2, pos=int_points.begin()+2;
         scalar!=scalars.end();  scalar++, pos++) {
      if (*scalar < *scalar1) {
        scalar2 = scalar1;  pos2 = pos1;
        scalar1 = scalar;   pos1 = pos;
      }
      else if (*scalar < *scalar2) {
        scalar2 = scalar;   pos2 = pos;
      }
    }

    // Check the segment length
    if ((*scalar2 - *scalar1) >=  min_length) {

      // Check if the mid point of the line segment is inside the polygon
      mid_point.vect() = (*pos1 + *pos2) / 2.0;
      if (mid_point.InsidePolygon(points, *this)) {

        // Add the segment to the list of intersecting line segments
        intersections.push_back(LineSegment2D(*pos1, *pos2));
      }
    }

    // Remove the smallest scalar and the corresponding intersection point
    scalars.erase(scalar1);
    int_points.erase(pos1);
  }

  return(intersections.size() - old_num_intersections);
}

/*
--------------------------------------------------------------------------------
                      	Split a polygon by a line segment
--------------------------------------------------------------------------------
*/

bool LineTopology::SplitPolygonByLineSegment(ObjectPoints &points,
                                             const LineSegment2D &edge_segment,
                                             double snap_dist,
                                             LineTopologies &new_polygons) const
{
  int                            begin_index, end_index, segment_index, found,
                                 begin_segment_index, end_segment_index;
  LineSegments2D::const_iterator segment;
  ObjectPoints::const_iterator   corner;
  ObjectPoint                    new_point;
  LineTopology::const_iterator   node;
  LineSegments2D                 segments = LineSegments2D(points, *this);
  LineTopology                   polygon = *this;

  // Check if the begin or end point is already near a node of the polygon
  begin_index = end_index = -1;
  for (node=begin(); node!=end()-1; node++) {
    corner = points.ConstPointIterator(*node);
    if (Distance(edge_segment.BeginPoint(), Position2D(corner->vect2D())) <
        snap_dist)
      begin_index = points.FindPoint(*node);
    if (Distance(edge_segment.EndPoint(), Position2D(corner->vect2D())) < 
        snap_dist)
      end_index = points.FindPoint(*node);
  }

  // Determine the edges in which new nodes need to be inserted
  begin_segment_index = -1;
  if (begin_index == -1) {    // New node needed for begin of line segment
    // Add the new point
    new_point.X() = edge_segment.BeginPoint().X();
    new_point.Y() = edge_segment.BeginPoint().Y();
    new_point.Z() = 0.0;
    new_point.Number() = (points.end()-1)->Number() + 1;
    points.push_back(new_point);
    begin_index = points.size() - 1;
    // Find the segment
    for (segment=segments.begin(), found=0, segment_index=0;
         segment!=segments.end() && !found; segment++, segment_index++) {
      if (segment->Distance(edge_segment.BeginPoint()) < 0.01) {
        found = 1;
        begin_segment_index = segment_index;
      }
    }
    // Insert the new node into the polygon
    polygon.insert(polygon.begin() + begin_segment_index + 1,
                   new_point.NumberRef());
  }

  if (end_index == -1) {      // New node needed for end of line segment
    // Add the new point
    new_point.X() = edge_segment.EndPoint().X();
    new_point.Y() = edge_segment.EndPoint().Y();
    new_point.Z() = 0.0;
    new_point.Number() = (points.end()-1)->Number() + 1;
    points.push_back(new_point);
    end_index = points.size() - 1;
    // Find the segment
    for (segment=segments.begin(), found=0, segment_index=0;
         segment!=segments.end() && !found; segment++, segment_index++) {
      if (segment->Distance(edge_segment.EndPoint()) < 0.01) {
        found = 1;
        end_segment_index = segment_index;
      }
    }
    // Insert the new node into the polygon
    if (begin_segment_index != -1 &&
        end_segment_index > begin_segment_index) end_segment_index++;
    polygon.insert(polygon.begin() + end_segment_index + 1,
                   new_point.NumberRef());
  }

  return polygon.Split(points[begin_index].NumberRef(),
                       points[end_index].NumberRef(), new_polygons);
}

/*
--------------------------------------------------------------------------------
                      	Split a polygon by a line
--------------------------------------------------------------------------------
*/

bool LineTopology::SplitPolygonByLine(ObjectPoints &points, const Line2D &line,
                                      double snap_dist, double min_length, 
                                      LineTopologies &new_polygons) const
{
  LineSegments2D           intersections;
  LineSegments2D::iterator intersection, longest_segment;
  double                   longest_dist;

  // Return false if there is no intersection
  if (!IntersectPolygonByLine(points, line, intersections, snap_dist,
                              min_length)) return false;

  // Determine the longest intersecting segment
  longest_dist = 0;
  for (intersection=intersections.begin();
       intersection!=intersections.end(); intersection++) {
    if (intersection->Length() > longest_dist) {
      longest_dist = intersection->Length();
      longest_segment = intersection;
    }
  } 

  return SplitPolygonByLineSegment(points, *longest_segment, snap_dist,
                                   new_polygons);
}

/*
--------------------------------------------------------------------------------
                      	Split a polygon by a linesegment, take the nearest intersection line
--------------------------------------------------------------------------------
*/

bool LineTopology::SplitPolygonByLineSegment_nearest(ObjectPoints &points, const LineSegment2D &linesegment,
                                      double snap_dist, double min_length, 
                                      LineTopologies &new_polygons) const
{
  LineSegments2D           intersections;
  LineSegments2D::iterator intersection, nearest_segment;
  double                   dist, nearest_dist;
  Position2D               poso, posi;
  Line2D                   line;
  
  line = Line2D(linesegment.BeginPoint(), linesegment.EndPoint());

  // Return false if there is no intersection
  if (!IntersectPolygonByLine(points, line, intersections, snap_dist,
                              min_length)) return false;

  // Determine the longest intersecting segment
  nearest_dist = 10000;
  poso = linesegment.MiddlePoint();
  for (intersection=intersections.begin();
       intersection!=intersections.end(); intersection++) {
       posi = intersection->MiddlePoint();
       dist = (poso - posi).Length();
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_segment = intersection;
    }
  } 

  return SplitPolygonByLineSegment(points, *nearest_segment, snap_dist,
                                   new_polygons);
}


/*
--------------------------------------------------------------------------------
          Construct by converting a TIN mesh into a triangular polygon
--------------------------------------------------------------------------------
*/

LineTopology::LineTopology(const TINMesh &mesh, int line_number, int line_label)
{
  const PointNumber *nodes;
  
  Initialise();
  nodes = mesh.Nodes();
  push_back(nodes[0]); push_back(nodes[1]); push_back(nodes[2]);
  push_back(nodes[0]);
  Number() = line_number;
    Label()  = line_label;
   //SetAttribute(LineLabelTag,line_label);
   
}

/*
--------------------------------------------------------------------------------
      Calculate a surface normal based on the first three polygon points 
--------------------------------------------------------------------------------
*/
Vector3D LineTopology::Normal(const ObjectPoints &points, int method) const
{
  ObjectPoints::const_iterator point1, point2, point3, point, pointA, pointB,
                               pointC;
  LineTopology::const_iterator node;
  Vector3D                     difvec1, difvec2, normal;
  double                       dist, max_dist;
  Line3D                       line;

  // Return null vector in case of less than three points
  if (size() < 3) return Vector3D();

  // Select the method
  switch (method) {
    case 0: // Normal based on first three non-collinear points
      node = begin(); point1 = points.ConstPointIterator(node->Number());
      node++;         point2 = points.ConstPointIterator(node->Number());
      difvec1 = point2->vect() - point1->vect();
      while (node != end()) {
        node++;
        point3 = points.ConstPointIterator(node->Number());
        difvec2 = point2->vect() - point3->vect();
        normal  = difvec1.VectorProduct(difvec2);
        if (normal.SqLength() > 0.0) return normal.Normalize();
      }
      break;

    case 1: // Normal based on most distant points
      // Find point most distant from the first point
      node = begin(); pointA = points.ConstPointIterator(node->Number());
      max_dist = 0.0;
      for (node=begin()+1; node!=end(); node++) {
        point = points.ConstPointIterator(node->Number());
        dist = pointA->Distance(point->Position3DRef());
        if (dist > max_dist) { max_dist = dist; pointB = point; }
      }
      if (max_dist == 0.0) break;
      // Find point most distant from this point
      max_dist = 0.0;
      for (node=begin(); node!=end(); node++) {
        point = points.ConstPointIterator(node->Number());
        dist = pointB->Distance(point->Position3DRef());
        if (dist > max_dist) { max_dist = dist; pointA = point; }
      }
      // Find point most distant from the line between point 1 and point 2
      line = Line3D(pointA->Position3DRef(), pointB->Position3DRef());
      max_dist = 0.0;
      for (node=begin(); node!=end(); node++) {
        point = points.ConstPointIterator(node->Number());
        if (point != pointA && point != pointB) {
          dist = line.DistanceToPoint(point->Position3DRef());
          if (dist > max_dist) { max_dist = dist; pointC = point; }
        }
      }
      if (max_dist == 0.0) break;
      // Get the points in the original node order
      if (FindUnOrderedPoint(pointA->NumberRef()) <
          FindUnOrderedPoint(pointB->NumberRef())) {
        if (FindUnOrderedPoint(pointA->NumberRef()) <
            FindUnOrderedPoint(pointC->NumberRef())) {
          point1 = pointA;
          if (FindUnOrderedPoint(pointB->NumberRef()) <
              FindUnOrderedPoint(pointC->NumberRef())) {
            point2 = pointB;  point3 = pointC;
          }
          else {
            point2 = pointC;  point3 = pointB;
          }
        }
        else {
          point1 = pointC;
          point2 = pointA;
          point3 = pointB;
        }
      }
      else { // A > B
        if (FindUnOrderedPoint(pointB->NumberRef()) <
            FindUnOrderedPoint(pointC->NumberRef())) {
          point1 = pointB;
          if (FindUnOrderedPoint(pointA->NumberRef()) <
              FindUnOrderedPoint(pointC->NumberRef())) {
            point2 = pointA;  point3 = pointC;
          }
          else {
            point2 = pointC;  point3 = pointA;
          }
        }
        else {
          point1 = pointC;
          point2 = pointB;
          point3 = pointA;
        }
      }
      // Calculate normal
      difvec1 = point2->vect() - point1->vect();
      difvec2 = point2->vect() - point3->vect();
      normal  = difvec1.VectorProduct(difvec2);
      if (normal.SqLength() > 0.0) return normal.Normalize();
      break;

    default:
      break;
  }
  return Vector3D(); // Return null vector in case of problems
}

/*
--------------------------------------------------------------------------------
      Densify a polygon by inserting new nodes on long edges
--------------------------------------------------------------------------------
*/
void LineTopology::Densify(ObjectPoints &points, double max_dist)
{
  int point_inserted, next_number, start_index;
  LineTopology::iterator node, start_node;
  ObjectPoints::const_iterator point1, point2;
  ObjectPoint            new_point;

  if (size() < 2) return;
  if (points.empty()) return;
  next_number = (points.end() - 1)->Number() + 1;
  start_node = begin();
  do {
    point_inserted = 0;
    node = start_node;
    point1 = points.ConstPointIterator(*node);
    if (point1 == points.end()) return; // Point not found
    for (node++; node!=end() && !point_inserted; node++) {
      point2 = points.ConstPointIterator(*node);
      if (point2 == points.end()) return; // Point not found
      if ((point1->vect() - point2->vect()).Length2D() > max_dist) {
        new_point.vect() = (point1->vect() + point2->vect()) / 2.0;
        new_point.Number() = next_number;
        points.push_back(new_point);
        insert(node, new_point.NumberRef());
        next_number++;
        point_inserted = 1;
        start_index = FindPoint(new_point.NumberRef()) - 1;
        start_node = begin() + start_index;
      }
      point1 = point2;
    }
  } while (point_inserted);
}

/*
--------------------------------------------------------------------------------
      Select another polygon node as start node of a closed polygon
--------------------------------------------------------------------------------
*/
bool LineTopology::NewStartNode(const PointNumber &new_start_node)
{
  int index_new_start;

  // Several checks
  if (!IsClosed()) return false; // Polygon should be closed
  index_new_start = FindPoint(new_start_node);
  if (index_new_start == -1) return false; // Node not part of polygon
  if (index_new_start == 0) return true; // Node already is the first one
  
  insert(end(), begin() + 1, begin() + index_new_start + 1);
  erase(begin(), begin() + index_new_start);
  return true;
}

/*
--------------------------------------------------------------------------------
                  Check if two polygons are the same
--------------------------------------------------------------------------------
*/

bool operator == (const LineTopology &top1, const LineTopology &top2)
{
  LineTopology::const_iterator node1, node2;

  if (top1.size() != top2.size()) return false;
  for (node1=top1.begin(), node2=top2.begin(); node1!=top1.end();
       node1++, node2++)
    if (*node1 != *node2) return false;
  return true;
}

/*
--------------------------------------------------------------------------------
      Check if a polygon is clockwise when seen from the top
--------------------------------------------------------------------------------
*/

bool LineTopology::IsClockWise(const ObjectPoints &points) const
{
  LineTopology::const_iterator   node;
  ObjectPoints::const_iterator   point1, point2;
  Vector2D                       vec;
  ObjectPoint2D                  mid_point;
  double                         len;

  // Take the first two points of a polygon
  node = begin();  point1 = points.ConstPointIterator(*node);
  node++;          point2 = points.ConstPointIterator(*node);

  // Construct a point that is a little right of the mid point of the edge
  vec = point2->vect2D() - point1->vect2D();
  len = vec.Length();
  if (len > 0.0) vec = vec / (100.0 * len);
  mid_point.vect() = (point1->vect2D() + point2->vect2D()) / 2.0;
  mid_point.X() += vec.Y();
  mid_point.Y() -= vec.X();

  // Check if this point is inside the polygon
  return mid_point.InsidePolygon(points, *this);
}

/*
--------------------------------------------------------------------------------
      Make a polygon counter clockwise when seen from the top
--------------------------------------------------------------------------------
*/

void LineTopology::MakeCounterClockWise(const ObjectPoints &points)
{
  if (IsClockWise(points)) RevertNodeOrder();
}

/*
--------------------------------------------------------------------------------
      Make a polygon clockwise when seen from the top
--------------------------------------------------------------------------------
*/

void LineTopology::MakeClockWise(const ObjectPoints &points)
{
  if (!IsClockWise(points)) RevertNodeOrder();
}


/*
--------------------------------------------------------------------------------
                            Print the line
--------------------------------------------------------------------------------
*/

void LineTopology::Print() const
{
  LineTopology::const_iterator node;

  printf("Line %d, %d points, %d attributes\n", num, size(), num_attributes);
  printf("Point numbers\n");
  for (node=begin(); node!=end(); node++) printf("%d ", node->Number());
  printf("\n");
  if (num_attributes) {
    printf("Attribute value pairs\n");
    for (int i=0; i<num_attributes; i++)
      printf("tag %d, value %d\n", tags[i], attributes[i]);
  }
}

unsigned char LineTopology::ExistingAttributeIndex
  (const LineTopologyTag requested_tag) const
{
  const unsigned char *tag;
  unsigned char       index;
   
  for (index=0, tag=tags; index<num_attributes; index++, tag++)
    if (*tag == requested_tag) {return(index);}
  
  return(UndefinedLineTag);
}

unsigned char LineTopology::AttributeIndex(const LineTopologyTag tag)
{

  unsigned char index=ExistingAttributeIndex(tag);
 
  //int tmp;
 
  if (index == UndefinedLineTag) {
  
    //tmp=(int)num_attributes;
    //tmp++;
    //num_attributes=(unsigned char)num; // Add a new attribute
    //num=(int)num_attributes;
    
   
    num_attributes++;
    index = num_attributes - 1; // This will be the last attribute
   
    if (tags == NULL)
      tags = (unsigned char *) malloc(sizeof(unsigned char *));
    else
      tags = (unsigned char *) realloc(tags, num_attributes *
                                       sizeof(unsigned char *));
                         
  
    tags[index] = (unsigned char) tag; // Store the tag;
    
   
    if (attributes == NULL)
      {
                            
                   //attributes = (int *) malloc(sizeof(int));
                   attributes=new int[1];
                   
                  }
    else
      attributes = (int *) realloc(attributes, num_attributes * sizeof(int));
  }
  return(index);
}

int LineTopology::Attribute(const LineTopologyTag tag) const
{
    
 
  unsigned char index=ExistingAttributeIndex(tag);
  if (index == UndefinedLineTag) return(INT_MIN);
  return(attributes[index]);
}

int & LineTopology::Attribute(const LineTopologyTag tag)
{
  
  unsigned char index = AttributeIndex(tag);
 
  return attributes[index];
}

bool LineTopology::HasAttribute(const LineTopologyTag tag) const
{
  unsigned char index=ExistingAttributeIndex(tag);
  return index != UndefinedLineTag;
}

/*
--------------------------------------------------------------------------------
                      Simple attribute manipulations
--------------------------------------------------------------------------------
*/

void LineTopology::SetAttribute(const LineTopologyTag tag, const int value)
{
  Attribute(tag) = value;
}

bool LineTopology::RemoveAttribute(const LineTopologyTag tag)
{
  unsigned char index=ExistingAttributeIndex(tag), i;
  if (index == UndefinedLineTag) return false;
  for (i=index; i<num_attributes-1; i++) {
    tags[i] = tags[i+1];
    attributes[i] = attributes[i+1];
  }
  num_attributes--;
  return true;
}

bool LineTopology::RemoveAttributes()
{
  if (num_attributes == 0) return false;
  num_attributes = 0;
  free(attributes);
  free(tags);
  return true;
}

bool LineTopology::RenameAttribute(const LineTopologyTag oldtag,
                                 const LineTopologyTag newtag)
{
  unsigned char index=ExistingAttributeIndex(oldtag);
  if (index == UndefinedLineTag) return false;
  tags[index] = newtag;
  return true;
}



/*
--------------------------------------------------------------------------------
                      Return the major TOP10 class code
--------------------------------------------------------------------------------
*/

int LineTopology::TOP10MajorClass() const
{
  if (Attribute(LineLabelTag) < 1000) return TOP10_Unknown;
  if (Attribute(LineLabelTag) < 2000) return TOP10_Building;
  if (Attribute(LineLabelTag) < 5000) return TOP10_Road;
  if (Attribute(LineLabelTag) < 6000) return TOP10_Meadow;
  if (Attribute(LineLabelTag) < 7000) return TOP10_Water;
  return TOP10_Unknown;
}

bool LineTopology::TOP10Invisible() const
{
  return (Attribute(LineLabelTag) - (Attribute(LineLabelTag)/10)*10 == 2);
}

int LineTopology::Smooth_Outline_3D(ObjectPoints &objpts, double angle_thres)
{
     Line3D line_a, line_b;
     double angle;
     LineTopology::iterator iter;
     int finish=1;
    
      for(iter=begin();iter!=end()-3;)
     {
       line_a=Line3D(objpts.PointByNumber(iter->Number()),objpts.PointByNumber((iter+1)->Number()));
       line_b=Line3D(objpts.PointByNumber((iter+1)->Number()),objpts.PointByNumber((iter+2)->Number()));
       angle=Angle2Lines(line_a,line_b);
       if(angle<(angle_thres*3.1416/180.0))
           {  erase(iter+1); finish=0; }
       else
         iter++;
     }   
    
    if(finish)
     return 1;
    else
     return Smooth_Outline_3D(objpts, angle_thres);
     
}

/*
--------------------------------------------------------------------------------
            Check if a closed polygon overlaps with a rectangle
--------------------------------------------------------------------------------
*/

bool LineTopology::Overlap(const ObjectPoints &points,
                           const DataBounds2D &rectangle,
                           bool bounds_check_done) const
{
  LineTopology::const_iterator node;
  ObjectPoints::const_iterator point;
  ObjectPoint2D                corner;
  LineSegments2D               rectangle_segments, polygon_segments;
  LineSegments2D::iterator     rectangle_segment, polygon_segment;
  bool                         found_intersection;
  
  // Check if polygon is closed
  if (!IsClosed()) return false;
  
  // Check if bounding box of polygon overlaps with rectangle
  if (!bounds_check_done) {
    DataBounds2D polygon_bounds;
    for (node=begin(); node!=end()-1; node++) {
      point = points.ConstPointIterator(*node);
      if (point == points.end()) return false; // No coordinates available
      polygon_bounds.Update(point->Position2DOnly());
    }
    if (!polygon_bounds.Overlap(rectangle)) return false;
  }

  // Check if rectangle corners are inside polygon
  corner.X() = rectangle.Minimum().X();
  corner.Y() = rectangle.Minimum().Y();
  if (corner.InsidePolygon(points, *this)) return true; // xmin, ymin
  corner.Y() = rectangle.Maximum().Y();  
  if (corner.InsidePolygon(points, *this)) return true; // xmin, ymax
  corner.X() = rectangle.Maximum().X();  
  if (corner.InsidePolygon(points, *this)) return true; // xmax, ymax
  corner.Y() = rectangle.Minimum().Y();  
  if (corner.InsidePolygon(points, *this)) return true; // xmax, ymin
  
  // Check if polygon points are inside rectangle
  for (node=begin(); node!=end()-1; node++) {
    point = points.ConstPointIterator(*node);
    if (point == points.end()) return false; // No coordinates available
    if (rectangle.Inside(point->Position2DOnly())) return true;
  }
  
  // Check if edges of polygon intersect with rectangle edges
  // First create edges of rectangle
  rectangle_segments.push_back(
    LineSegment2D(Position2D(rectangle.Minimum().X(), rectangle.Minimum().Y()),
                  Position2D(rectangle.Minimum().X(), rectangle.Maximum().Y())));
  rectangle_segments.push_back(
    LineSegment2D(Position2D(rectangle.Minimum().X(), rectangle.Maximum().Y()),
                  Position2D(rectangle.Maximum().X(), rectangle.Maximum().Y())));
  rectangle_segments.push_back(
    LineSegment2D(Position2D(rectangle.Maximum().X(), rectangle.Maximum().Y()),
                  Position2D(rectangle.Maximum().X(), rectangle.Minimum().Y())));
  rectangle_segments.push_back(
    LineSegment2D(Position2D(rectangle.Maximum().X(), rectangle.Minimum().Y()),
                  Position2D(rectangle.Minimum().X(), rectangle.Maximum().Y())));
  // Create edges of polygon
  polygon_segments = LineSegments2D(points, *this);
  // Check all possible intersections
  for (polygon_segment=polygon_segments.begin(), found_intersection=false;
       polygon_segment!=polygon_segments.end() && !found_intersection;
       polygon_segment++)
    for (rectangle_segment=rectangle_segments.begin();
         rectangle_segment!=rectangle_segments.end() && !found_intersection;
         rectangle_segment++)
      if (polygon_segment->Distance(*rectangle_segment) == 0.0)
        found_intersection = true;
  // Clean up
  rectangle_segments.erase(rectangle_segments.begin(), rectangle_segments.end());
  polygon_segments.erase(polygon_segments.begin(), polygon_segments.end());
  
  return found_intersection;
}

bool LineTopology::BoundPoint(const ObjectPoints &polyPnts, const Vector3D& inPnt)
{
	if (!this->IsClosed()) return false;
	ObjectPoint2D pnt2d;
	pnt2d.X() = inPnt.X();
	pnt2d.Y() = inPnt.Y();

	return pnt2d.InsidePolygon(polyPnts, *this);
}

/*
--------------------------------------------------------------------------------
                      Calculate Area of closed polygon
                      Using approximated formulas from Green's theorem
--------------------------------------------------------------------------------
*/

double LineTopology::CalculateArea(const ObjectPoints &points)
{
if (IsClosed()){
  LineTopology::const_iterator node, nextnode;
  ObjectPoints::const_iterator   point1, point2;
  
  if (IsClockWise(points)) RevertNodeOrder();//counter clockwise gives positive results
  
  double area, offsetx, offsety, p1x, p2x, p1y, p2y;
  area = 0;
  offsetx = points.ConstPointIterator(*begin())->X();
  offsety = points.ConstPointIterator(*begin())->Y();

  for (node=begin(); node!=end()-1; node++) {
      nextnode = node+1;
      point1 = points.ConstPointIterator(*node);
      point2 = points.ConstPointIterator(*nextnode);
      p1x =  point1->X() - offsetx;
      p2x =  point2->X() - offsetx;
      p1y =  point1->Y() - offsety;
      p2y =  point2->Y() - offsety;
      
      area = area + p1x*p2y - p2x*p1y;
      }
      area = fabs(area/2);
      
      return (area);
}
else {
     printf("Error calculating surface area: linetopology is not a closed polygon.\n");
     return (-1.0);
     }       
}

/*
--------------------------------------------------------------------------------
                      Calculate centroid of closed polygon 
                      Using approximated formulas from Green's theorem
         (relative coordinates are used to minimize errors in Green's theorem)
--------------------------------------------------------------------------------
*/


Position2D LineTopology::CalculateCentroid(const ObjectPoints &points)
{
Position2D centroid;
if (IsClosed()){
  LineTopology::const_iterator node, nextnode;
  ObjectPoints::const_iterator   point1, point2;
  double cx, cy, offsetx, offsety, p1x, p2x, p1y, p2y;
//  float  cx, cy, offsetx, offsety, p1x, p2x, p1y, p2y;
  cx =0;
  cy =0;
  double area;
  area = CalculateArea(points);
  if (IsClockWise(points)) RevertNodeOrder(); //counter clockwise gives positive results
  //for (node=begin(); node!=end()-1; node++) {
      
  offsetx = points.ConstPointIterator(*begin())->X()+1;
  offsety = points.ConstPointIterator(*begin())->Y()+1;
  
  for (node=begin(); node!=end()-1; node++) {
      nextnode = node+1;
      point1 = points.ConstPointIterator(*node);
      point2 = points.ConstPointIterator(*nextnode);
      p1x =  point1->X() - offsetx;
      p2x =  point2->X() - offsetx;
      p1y =  point1->Y() - offsety;
      p2y =  point2->Y() - offsety;

      cx = cx + ((p1x+p2x)*(p1x*p2y-p2x*p1y));
      cy = cy + ((p1y+p2y)*(p1x*p2y-p2x*p1y));
      }
  //    if (IsClockWise(points)) centroid = Position2D(cx/(6*area), cy/(6*area));
      cx = offsetx + cx/(6.0*area);
      cy = offsety + cy/(6.0*area);
      centroid = Position2D(cx, cy);
            
      return (centroid);
}
else {
     printf("Error calculating centroid: linetopology is not a closed polygon.\n");
     centroid = Position2D(0.0,0.0);
     return (centroid);
     }       
}       

/*
--------------------------------------------------------------------------------
                      Short check whether TIN triangles are valid or not
--------------------------------------------------------------------------------
*/
 int LineTopology::IsValid() const
      {
      if (size() < 2) return(0);
      if (begin()->Number() != (end()-1)->Number())return (0);
      if (begin()->Number() == (end()-2)->Number())return (0);
      if (begin()->Number() == (end()-3)->Number())return (0);
      if ((end()-2)->Number() == (end()-3)->Number())return (0);
      
      return(1);
      }

/*
--------------------------------------------------------------------------------
                          Remove double nodes
--------------------------------------------------------------------------------
*/

void LineTopology::RemoveDoubleNodes()
{
  PointNumberList::iterator node1, node2;
 
//  node1 = begin();
  node2 = begin();
  for (node1 = begin(); node1!= end(); node1++){
      node2 = node1;
      node2 = NextNode(node2);
      if (node1->Number() == node2->Number()){
       erase(node1, node2);
       }
      }
return;
}
 

