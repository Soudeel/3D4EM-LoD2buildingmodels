
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
 Additional functions for class LaserPoints related to components in a TIN

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
#include "LaserDataTypes.h"
#include "LaserPoints.h"
#include "LaserOctree.h"
#include "KNNFinder.h"
#include "MyConstants.h"
#include <time.h>

/*
--------------------------------------------------------------------------------
                      Derive the edges to neighbouring points
--------------------------------------------------------------------------------
*/

void timer_start(clock_t *time1)
{
  *time1 = clock();
}

void timer_end(clock_t time1, char *string)
{
  clock_t time2;
  time2 = clock();
  printf("Time used for %s: %5.2f minutes\n", string,
         (double) (time2 - time1) / (60 * CLOCKS_PER_SEC));
}


TINEdges * LaserPoints::DeriveEdges(const SegmentationParameters &parameters)
{
  TINEdges              *tin_edges, *edges;
  LaserOctree           *octree;
  LaserPoints::iterator point;
  int                   i;
  clock_t               start;
  
  
  timer_start(&start);
  
  if (nbh_edges) EraseNeighbourhoodEdges();	

  switch (parameters.NeighbourhoodStorageModel()) {
    case 0: // Derive TIN and TIN edges
      VerifyTIN();
      tin_edges = new TINEdges();
      tin_edges->Derive(tin->TINReference());
      while (tin_edges->size() < size()) tin_edges->push_back(TINEdgeSet());
      if (parameters.MaxDistanceInComponent() > 0.0) {
        // Remove long edges
        RemoveLongEdges(tin_edges->TINEdgesRef(), 
                        parameters.MaxDistanceInComponent(), true);
      }
      nbh_edges = tin_edges;
      return nbh_edges;
      break;
      
    case 1: // Derive edges from octree
      octree = new LaserOctree();
      DeriveDataBounds(1);
      octree->Initialise(bounds.Bounds3D(),
                         parameters.OctreeBinOverlap(),
                         parameters.OctreeBinMaxNumberOfPoints());
      octree->Insert(*this);
      // Derive edges from octree
      edges = octree->NeighbourhoodEdges3D(*this,
                                           parameters.MaxDistanceInComponent());
      octree->Erase();
      nbh_edges = edges;
      return edges;
      break;
      
    case 2: // Derive edges from kd-tree
      { KNNFinder <LaserPoint> finder(*this, parameters.DistanceMetricDimension());
        vector<int>            indices;
        TINEdges::iterator     edgeset;
        int                    num_nb=parameters.NumberOfNeighbours();
        
        if (size() < num_nb) num_nb = size();
	    edges = new TINEdges();
        for (int i=0; i<(int) size(); i++) {
          indices = finder.FindIndices((*this)[i], num_nb);
          edges->push_back(TINEdgeSet());
          edgeset = edges->end() - 1;
          for (int j=0; j<num_nb; j++)
            edgeset->push_back(PointNumber(indices[j]));
          indices.erase(indices.begin(), indices.end());
        }
        finder.FreeResources();
        nbh_edges = edges;
        return nbh_edges;
      }
      break;
      
    default: // Invalid neighbourhood model
      printf("Error: Invalid neighbourhood model (%d)\n",
             parameters.NeighbourhoodStorageModel());
      break;
      
  }
  nbh_edges = NULL;
  return NULL;
}

TINEdges * LaserPoints::DeriveFDNEdges(const SegmentationParameters &parameters)
{
  TINEdges              *tin_edges, *edges;
  LaserOctree           *octree;
  LaserPoints::iterator point;
  int                   i;
  clock_t               start;
  
  
  timer_start(&start);
  
  if (nbh_edges) EraseNeighbourhoodEdges();	

  switch (parameters.NeighbourhoodStorageModel()) {
    case 0: // Derive TIN and TIN edges
      VerifyTIN();
      tin_edges = new TINEdges();
      tin_edges->Derive(tin->TINReference());
      while (tin_edges->size() < size()) tin_edges->push_back(TINEdgeSet());
      if (parameters.DistanceMetricDimension() == 2) {
      	if (parameters.GrowingNeighbourhoodDefinition() == 1) { // All points within a radius
          edges = new TINEdges();
          for (point=begin(), i=0; point!=end(); point++, i++) {
            edges->push_back(TINEdgeSet(Neighbourhood(PointNumber(i),
               parameters.GrowingRadius(), tin_edges->TINEdgesRef(),
               true, false)));
          }
          delete tin_edges;
          nbh_edges = edges;
//      	  timer_end(start, "TIN based FDN edges");
      	}
      	else if (parameters.MaxDistanceInComponent() > 0.0) {
          // Remove long edges
          RemoveLongEdges(tin_edges->TINEdgesRef(), 
                          parameters.MaxDistanceInComponent(), true);
          nbh_edges = tin_edges;
      	}

        return nbh_edges;
      }
      else {
        edges = NeighbourhoodEdges3D(tin_edges->TINEdgesRef(),
                                     parameters.MaxDistanceInComponent());
        delete tin_edges;
        nbh_edges = edges;
        return edges;
      }
      break;
      
    case 1: // Derive edges from octree
      octree = new LaserOctree();
      DeriveDataBounds(1);
      octree->Initialise(bounds.Bounds3D(),
                         parameters.OctreeBinOverlap(),
                         parameters.OctreeBinMaxNumberOfPoints());
      octree->Insert(*this);
      // Derive edges from octree
      edges = octree->NeighbourhoodEdges3D(*this,
                                           parameters.MaxDistanceInComponent());
      octree->Erase();
      nbh_edges = edges;
      return edges;
      break;
      
    case 2: // Derive edges from kd-tree
      { KNNFinder <LaserPoint> finder(*this, parameters.DistanceMetricDimension());
        vector<int>            indices;
        TINEdges::iterator     edgeset;
        int                    num_nb=parameters.NumberOfNeighbours(), index, j,
		                       num_pts;
        double                 distance;     
		vector<double>         distances;     
		bool                   done;          
        
        edges = new TINEdges();
        for (int i=0; i<(int) size(); i++) {
          edges->push_back(TINEdgeSet());
          edgeset = edges->end() - 1;
          num_pts = 50; 
          if (size() - 1 < 50) num_pts == size() - 1;
		  done = false;
          do {
            finder.FindKnn((*this)[i], num_pts, distances, indices);
            if (distances[num_pts-1] > parameters.GrowingRadius() ||
			    num_pts == size() - 1) {
              done = true;
            }
            else {
              num_pts *= 2;
              if (num_pts > size() - 1) num_pts = size() - 1;
            }
          } while (!done);
          for (j=0, done=false; j<num_pts && !done; j++) {
            if (distances[j] < parameters.GrowingRadius())
              edgeset->push_back(PointNumber(indices[j]));
            else
              done = true;
          }
        }
//     	  timer_end(start, "kd-tree based FDN edges");
        nbh_edges = edges;
        return edges;
      }
      break;
      
    default: // Invalid neighbourhood model
      printf("Error: Invalid neighbourhood model (%d)\n",
             parameters.NeighbourhoodStorageModel());
      break;
      
  }
  nbh_edges = NULL;
  return NULL;
}

TINEdges * LaserPoints::VerifyEdges(const SegmentationParameters &parameters)
{
  bool     derive_edges = true;
  
  // Check if there are edges at all
  if (GetNeighbourhoodEdges() != NULL) {
  	// Check if there are edges for every point (and not more)
  	if (NeighbourhoodEdges().size() == size()) { 
  	  derive_edges = false;
  	  // Check the number of edges for kd-tree
  	  if (parameters.NeighbourhoodStorageModel() == 2) { // kd-tree
	    if (parameters.NumberOfNeighbours() !=
	        (int) NeighbourhoodEdges().begin()->size()) {
	      derive_edges = true; 
	    }
	  }
  	}
  }
  if (derive_edges) return DeriveEdges(parameters);
  return GetNeighbourhoodEdges();
}

/*
--------------------------------------------------------------------------------
                              Neighbourhood
--------------------------------------------------------------------------------
*/

/* Neighbourhood based on label and/or filter state. Set the label of all
 * points in the neighbourhood.
 */

PointNumberList LaserPoints::Neighbourhood(const PointNumber &centre,
				           const TINEdges &edges, 
                                           int use_label, int use_filter_state,
                                           int new_label)
{
  PointNumberList           neighbourhood;
  PointNumberList::iterator nb, node;
  TINEdgeSet                neighbours;
  LaserPoints::iterator     centre_point, nb_point;
  int                       node_index, label_state, filter_state, mem_alloc=10;

/* Check if we can use something to group the points */

  if (!use_label && !use_filter_state) {
    fprintf(stderr,
  "Error: Can not use label or filter state for detection of neighbourhood.\n");
    exit(0);
  }

/* Initialise the neighbourhood with the centre */

  neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
  neighbourhood.reserve(mem_alloc);
  neighbourhood.push_back(centre);
  centre_point = begin() + centre.Number();
  label_state = centre_point->Label();
  centre_point->Label(new_label);
  filter_state = centre_point->Filtered();

/* Check the neighbours, and their neighbours, and theirs, and ... */

  for (node=neighbourhood.begin(), node_index=0;
       node!=neighbourhood.end();
       node++, node_index++) {
    neighbours = edges[node->Number()];
    for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
      nb_point = begin() + nb->Number();
      if ((nb_point->Filtered() == filter_state || !use_filter_state) &&
	  (nb_point->Label() == label_state || !use_label) &&
          nb_point->Label() != new_label) {
	switch (mem_alloc) {
	  case 10 :
	  case 100: if (neighbourhood.size() == mem_alloc) mem_alloc *= 10;
		    break;
          default : if (neighbourhood.size() == mem_alloc) mem_alloc += 1000;
	}
        neighbourhood.reserve(mem_alloc);
        neighbourhood.push_back(*nb);
	node = neighbourhood.begin() + node_index;
	nb_point->Label(new_label);
      }
    }
  }
  return(neighbourhood);
}

/*
--------------------------------------------------------------------------------
                       Contour of connected component
--------------------------------------------------------------------------------
*/

/* Some debugging functions */

void PrintNumberList(const char *string, const PointNumberList &pnl)
{
  PointNumberList::const_iterator pn;

  if (string) printf("%s: ", string);
  for (pn=pnl.begin(); pn!= pnl.end(); pn++) printf("%d ", pn->Number());
  printf("(%d)\n", pnl.size());
}

void PrintEdgeSet(PointNumber *pns, TINEdgeSet *set)
{
  TINEdgeSet::iterator pn;

  printf("EdgeSet of point %d: ", pns->Number());
  for (pn=set->begin(); pn!= set->end(); pn++) printf("%d ", pn->Number());
  printf("\n");
}

void PrintPoint(LaserPoints *pts, PointNumber *pn)
{
  LaserPoints::iterator pt;

  pt = pts->begin() + pn->Number();
  printf("Point %d: %10.2f %10.2f", pn->Number(), pt->X(), pt->Y());
}

LineTopology LaserPoints::DeriveContour(int contour_number,
                                        const PointNumberList &component,
					                    const TINEdges &edges,
                                        bool same_attribute_value,
                                        const LaserPointTag tag) const
{
  LineTopology                    contour;
  int                             foundnext, nbdir, value;
  TINEdges::const_iterator        edgeset;
  PointNumberList::const_iterator startpoint, point, prevpoint, nextpoint,
		                  curpoint, secondpoint;
  PointNumberList::iterator       conpoint;
  double                          maxX, X, dir, bestdir, anglesum, angle, pi;
  Vector3D                        p1, p2;
  LaserPoints::const_iterator     pt;

  contour.Number() = contour_number;

/* Use the point with the largest X-coordinate as the starting point */

  startpoint = component.begin();
  maxX = (begin() + startpoint->Number())->X();
  for (point=component.begin()+1; point!=component.end(); point++) {
    X = (begin() + point->Number())->X();
    if (X > maxX) {maxX = X; startpoint = point;}
  }
  contour.push_back(*startpoint);
  // Retrieve object's attribute value
  value = (begin() + startpoint->Number())->Attribute(tag);

/* Use the first point in counter-clockwise order starting from three o'clock
 * which is part of the component as the next point.
 */

  edgeset = edges.begin() + startpoint->Number();
  pi = 4.0 * atan(1.0);
  bestdir = 1.0;
  foundnext = 0;
  for (point = edgeset->begin(); point != edgeset->end(); point++) {
    if ((begin() + point->Number())->Attribute(tag) == value ||
        !same_attribute_value) {
    /* if (component.FindPoint(point->Number()) != -1) */
      dir = Direction2D(*startpoint, *point);
      if (dir > 0.0) dir -= 2.0 * pi;
      if (dir < bestdir) {bestdir = dir; nextpoint = point; foundnext = 1;}
    }
  }

/* If there is no next point, the component only has one point */

  if (!foundnext) {
    if (component.size() != 1) {
      printf("Warning: Inconsistency in deriving contour.\n");
      pt = begin() + startpoint->Number();
      printf("Start point %.2f %.2f %.2f\n", pt->X(),
             pt->Y(), pt->Z());
      printf("same_attribute_value %d\n", same_attribute_value);
      printf("Edge set of start point has %d members\n", edgeset->size());
    }
    return(contour);
  }

/* Walk around the component contour until the first and second point are
 * encountered again.
 */

  anglesum = 0.0;
  prevpoint = startpoint;
  curpoint = nextpoint;
  secondpoint = nextpoint;
  do {
    contour.push_back(*curpoint);

    /* Look for the previous point in the edge set of the current point */

    edgeset = edges.begin() + curpoint->Number();
    nbdir = edgeset->FindPoint(prevpoint->Number()) + 1;
    if (nbdir == 0) {
      printf("Error: Inconsistancy in edge set\n");
      exit(0);
    }

    /* Find the next point that is part of the component */

    for (point = edgeset->begin() + nbdir, foundnext = 0;
	 nbdir < edgeset->size() && !foundnext;
	 nbdir++, point++) {
      if ((begin() + point->Number())->Attribute(tag) == value ||
          !same_attribute_value)
      /* if (component.FindPoint(point->Number()) != -1) */
	{foundnext = 1; nextpoint = point;}
    }
    if (!foundnext) {
      for (point = edgeset->begin(), nbdir = 0;
	   nbdir < edgeset->size() && !foundnext;
	   nbdir++, point++) {
        if ((begin() + point->Number())->Attribute(tag) == value ||
            !same_attribute_value)
        /* if (component.FindPoint(point->Number()) != -1) */
	  {foundnext = 1; nextpoint = point;}
      }
    }

    /* Determine the angle at the current point */

    angle = Angle2D(*prevpoint, *curpoint, *nextpoint);
    if (angle == 0) angle = 2 * pi;
    anglesum += angle;

    /* Prepare for the next point */

    prevpoint = curpoint;
    curpoint = nextpoint;
  } while (*prevpoint != *startpoint || *curpoint != *secondpoint);

/* Check whether the contour is the outer contour */

  contour.erase(contour.end()-1);
  anglesum -= pi * (contour.size());
  if (anglesum < 0.0) {
    printf("Error in deriving contour, negative angle sum\n");
    printf("Angle sum %7.2f\n", anglesum * 200 / pi);
    PrintNumberList("Component", component);
    if (component.size() < 100) {
      LaserPoints::const_iterator laserpt;
      for (point=component.begin(); point!=component.end(); point++) {
        laserpt = begin() + point->Number();
        printf("Nr %5d  X %10.3f Y %10.3f Z %7.3f R %3d PC %3d\n",
               point->Number(),
               laserpt->X(), laserpt->Y(), laserpt->Z(),
               laserpt->Reflectance(), laserpt->PulseCount());
      }
    }
    PrintNumberList("Contour", contour);
    if (contour.size() < 100) {
      LaserPoints::const_iterator laserpt;
      for (conpoint=contour.begin(); conpoint!=contour.end(); conpoint++) {
        laserpt = begin() + conpoint->Number();
        printf("Nr %5d  X %10.3f Y %10.3f Z %7.3f R %3d PC %3d\n",
               conpoint->Number(),
               laserpt->X(), laserpt->Y(), laserpt->Z(),
               laserpt->Reflectance(), laserpt->PulseCount());
      }
    }
    exit(0);
  }
  return(contour);
}
/*
--------------------------------------------------------------------------------
Derive contours of a connnected component, allowing to delete neighbourhood edges
--------------------------------------------------------------------------------
*/

LineTopology LaserPoints::DeriveContour(int number,
                                        const PointNumberList &component, 
                                        double max_edge_length_factor,
                                        double max_edge_dist_factor,
                                        TINEdges &edges)
{
  LineTopology           contour;
  LineTopology::iterator nodeA, nodeB, nodeC;
  LaserPoints::iterator  pointA, pointB, pointC;
  bool                   deleted_edge, deleted_edgeAB;
  double                 point_spacing, distAB, distAC, distBC;
  TINEdges::iterator     edgesetA, edgesetB;
  Line2D                 line;
  
  point_spacing = MedianInterPointDistance((int) min(1000.0, (double) size()-2));
          
  // See if edges can be deleted
  do {
    // Derive the contour
    contour = DeriveContour(number, component, edges, false);
    deleted_edge = false;
    // Loop over all edges of the contour (adjacent points A and B)
    for (nodeA=contour.begin(), pointA=begin()+nodeA->Number(),
         edgesetA=edges.begin()+nodeA->Number(),
         nodeB=contour.NextNode(nodeA), pointB=begin()+nodeB->Number(),
         edgesetB=edges.begin()+nodeB->Number();
         nodeA!=contour.end()-1; 
         nodeA=nodeB, pointA=pointB, edgesetA=edgesetB,
         nodeB=contour.NextNode(nodeA), pointB=begin()+nodeB->Number(),
         edgesetB=edges.begin()+nodeB->Number()) {
      // Check if this is a relatively long edge
      distAB = (pointA->vect() - pointB->vect()).Length2D();
      if (distAB > max_edge_length_factor * point_spacing) {
        // See if A and B have a common neighbour
        for (nodeC=edgesetA->begin(), deleted_edgeAB=false;
             nodeC!=edgesetA->end() && !deleted_edgeAB; nodeC++) {
          if (edgesetB->Contains(*nodeC)) {
            pointC = begin() + nodeC->Number();
            distAC = (pointA->vect() - pointC->vect()).Length2D();
            distBC = (pointB->vect() - pointC->vect()).Length2D();
            // Check distances between the three points
            if (distAC < distAB && distBC < distAB) {
              line = Line2D(pointA->Position2DOnly(),
                            pointB->Position2DOnly());
              // Check distance of point to the line
              if (line.DistanceToPoint(pointC->Position2DOnly()) <
                  max_edge_dist_factor * point_spacing) {
                // Remove edges
                edgesetA->Remove(*nodeB);
                edgesetB->Remove(*nodeA);
                deleted_edgeAB = deleted_edge = true;
                pointC->SetAttribute(PlaneNumberTag, 6);
              }
            }
          }
        }
      }
    }
  } while (deleted_edge);

  return contour;
}

/*
--------------------------------------------------------------------------------
   	       Extract sequences of the specified attribute value
--------------------------------------------------------------------------------
*/

LineTopologies & LaserPoints::TaggedSequences(const LineTopology &polygon,
                                              int value,
                                              LaserPointTag tag,
                                              int max_num_pts_gap,
                                              double max_gap_size) const
{
  LineTopologies               *sequences = new LineTopologies();
  LineTopology::const_iterator node, start_node, end_node, prev_node;
  LineTopology                 sequence;
  bool                         found;
  LaserPoints::const_iterator  point, prev_point;
  int                          num_pts_gap;
  double                       gap_size;
  
  // Find the first gap in the sequence
  for (node=polygon.begin(), num_pts_gap=0;
       node!=polygon.end() && num_pts_gap <= max_num_pts_gap; node++) {
    point = begin() + node->Number();
    if (point->Attribute(tag) == value) num_pts_gap = 0;
    else {
      num_pts_gap++;
      if (num_pts_gap > max_num_pts_gap) {
        // The start node for searching the first sequence is a node in a gap
        start_node = node;
        // The end node for searching is the previous point in the same gap
        end_node   = node - 1;
      }
    }
  }
  
  // If there is no gap, there's only one sequence. Construct and return
  // this sequence
  if (num_pts_gap <= max_num_pts_gap) {
    for (node=polygon.begin(); node!=polygon.end(); node++) {
      point = begin() + node->Number();
      if (point->Attribute(tag) == value) sequence.push_back(*node);
    }
    sequences->push_back(sequence);
    return *sequences;
  }

  // Search the whole polygon for sequences
  do {
    // Find the first point of the next sequence
    for (node=start_node, found=false; node!=end_node && !found;
         node=polygon.NextNode(node)) {
      point = begin() + node->Number();
      if (point->Attribute(tag) == value) {
        found = true;
        start_node = node;
        sequence.Erase();
        sequence.push_back(*start_node);
      }
    }
    if (!found) return *sequences; // Return list of polygons
  
    // Search for further points
    prev_point = point;
    prev_node  = start_node;
    node = polygon.NextNode(start_node);
    point = begin() + node->Number();
    num_pts_gap = 0;
//    while (point->Attribute(tag) == value || num_pts_gap <= max_num_pts_gap) {
    while (num_pts_gap <= max_num_pts_gap) {
      if (point->Attribute(tag) != value) {
        num_pts_gap++;
      }
      else {
        if (num_pts_gap) {
          gap_size = (prev_point->vect() - point->vect()).Length2D();
          if (gap_size < max_gap_size) {
            num_pts_gap = 0;
          }
          else {
            num_pts_gap = max_num_pts_gap + 1;
            node = prev_node;
            point = prev_point;
          }
        }
        if (!num_pts_gap) {
          sequence.push_back(*node);
          prev_point = point;
          prev_node  = node;
        }
      }
      node = polygon.NextNode(node);
      point = begin() + node->Number();
    };
    
    // Reset current node in case of a unclosed gap
    if (num_pts_gap) {
      node = polygon.NextNode(prev_node);
      point = begin() + node->Number();
    }
    
    start_node = node; // Start for new search
    // To prevent looping forever in the case of 0 gap sizes:
    if (start_node-1 == end_node) start_node = end_node;

    sequences->push_back(sequence); // Store sequence
  } while(1);
  return *sequences; // We'll never get here. Just to satisfy the compiler.
}

/*
--------------------------------------------------------------------------------
	       Derive contours of holes in connnected component
--------------------------------------------------------------------------------
*/

LineTopologies LaserPoints::DeriveHoles(const PointNumberList &object,
					const TINEdges &edges,
					const PointNumberList &contour)
{
  PointNumberList::const_iterator node, nb, hole_node;
  LaserPoints::iterator     node_point, nb_point;
  TINEdgeSet                neighbours;
  PointNumberList           unfiltered_points, hole;
  LineTopology              hole_contour;
  LineTopologies            hole_contours;

/* Find all unfiltered points that are inside the contour of the object and
 * adjacent to the contour.
 */

/* CURRENTLY NOT IMPLEMENTED, SINCE THE NEXT CRITERION WILL USUALLY FIND ALL
 * POINTS.
 */

/* Find all unfiltered points that are adjacent to other points of the object */

  for (node=object.begin(); node!=object.end(); node++) {
    node_point = begin() + node->Number();
    if (node_point->Label() != 2) { /* Node is not a contour point */
      neighbours = edges[node->Number()];
      for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
        nb_point = begin() + nb->Number();
        if (!nb_point->Filtered()) { /* Unfiltered neighbour */
	  if (unfiltered_points.FindPoint(*nb) == -1)
	    unfiltered_points.push_back(*nb);
        }
      }
    }
  }

/* Determine the connected components of the unfiltered points and make a
 * list of neighbouring filtered points for each of these holes.
 */

  for (node=unfiltered_points.begin(); node!=unfiltered_points.end(); node++) {
    node_point = begin() + node->Number();
    if (!node_point->Processed()) {
      hole = Neighbourhood(*node, edges, 1, 1, 3);
      if (!hole_contour.empty())
        hole_contour.erase(hole_contour.begin(), hole_contour.end());
      hole_contour.Number() = hole_contours.size() + 1;
      for (hole_node=hole.begin(); hole_node!=hole.end(); hole_node++) {
        neighbours = edges[hole_node->Number()];
        for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
          nb_point = begin() + nb->Number();
          if (nb_point->Filtered() && hole_contour.FindPoint(*nb) == -1) {
	    hole_contour.push_back(*nb);
	    nb_point->Label(4);
	  }
	}
      }
      hole_contours.push_back(hole_contour);
    }
  }
  return(hole_contours);
}

/*
--------------------------------------------------------------------------------
                       Determine size of a neighbourhood
--------------------------------------------------------------------------------
*/

double LaserPoints::NeighbourhoodSize(const PointNumberList &component,
                                      const TINEdges &edges,
                                      int label) const
{
  PointNumberList::const_iterator node, nb_node1, nb_node2;
  LaserPoints::const_iterator     point, nb_point1, nb_point2; 
  PointNumberList                 nb_nodes;
  double                          size;

  size = 0;
  for (node=component.begin(); node!=component.end(); node++) {
    nb_nodes = edges[node->Number()];
    point    = begin() + node->Number();
    for (nb_node1=nb_nodes.end()-1, nb_node2=nb_nodes.begin();
         nb_node2!=nb_nodes.end();
         nb_node1=nb_node2, nb_node2++) {
      nb_point1 = begin() + nb_node1->Number();
      nb_point2 = begin() + nb_node2->Number();
      if (nb_point1->Label() == label && nb_point2->Label() == label)
        size += fabs( 0.5 * (
                point->X() * nb_point1->Y() +
                nb_point1->X() * nb_point2->Y() +
                nb_point2->X() * point->Y() -
                point->Y() * nb_point1->X() -
                nb_point1->Y() * nb_point2->X() -
                nb_point2->Y() * point->X()));
                
    }
  }
  return(size);
}

/*
--------------------------------------------------------------------------------
                       2D Neighbourhood based on distance
--------------------------------------------------------------------------------
*/

PointNumberList LaserPoints::Neighbourhood(const PointNumber &centre,
                                           double max_range,
                                           const TINEdges &edges,
                                           bool planimetric,
                                           bool direct_neighbours_only) const
{
  PointNumberList             neighbourhood;
  PointNumberList::iterator   nb, node;
  TINEdgeSet                  neighbours;
  LaserPoints::const_iterator centre_point, nb_point;
  int                         node_index;
  double                      range;

/* Initialise the neighbourhood with the direct neighbours of the centre */

  centre_point = begin() + centre.Number();
  neighbours   = edges[centre.Number()];
  neighbourhood.reserve(neighbours.size()+1);
  neighbourhood.push_back(centre);
  for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
  	if (centre.Number() == nb->Number()) continue;
    nb_point = begin() + nb->Number();
    if (planimetric) range = (*nb_point - *centre_point).Length2D();
    else range = (*nb_point - *centre_point).Length();
    if (range <= max_range) {
      neighbourhood.push_back(*nb);
    }
  }
  if (direct_neighbours_only) return neighbourhood;

/* Check the neighbours of the neighbours */

  for (node=neighbourhood.begin(), node_index=0;
       node!=neighbourhood.end();
       node++, node_index++) {
    neighbours = edges[node->Number()];
    for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
      if (*nb != centre) { /* Node is not the centre point */
        if (neighbourhood.FindPoint(*nb) == -1) { /* Node is not yet selected */
          nb_point = begin() + nb->Number();
          if (planimetric) range = (*nb_point - *centre_point).Length2D();
          else range = (*nb_point - *centre_point).Length();
          if (range <= max_range) {
            neighbourhood.reserve(neighbourhood.size() + 1);
            neighbourhood.push_back(*nb);
            node = neighbourhood.begin() + node_index;
          }
        }
      }
    }
  }

  return neighbourhood;
}

/*
--------------------------------------------------------------------------------
                       3D Neighbourhood based on distance
      This version should be used if the edges are based on a Delaunay TIN
--------------------------------------------------------------------------------
*/

PointNumberList LaserPoints::Neighbourhood3D(const PointNumber &centre,
                                             double range,
                                             const TINEdges &edges) const
{
  PointNumberList             neighbourhood;
  PointNumberList::iterator   node;
  LaserPoints::const_iterator centre_point, point;
  int                         node_index;

  // First select points within the 2D neighbourhood
  neighbourhood = Neighbourhood(centre, range, edges);

  // Next, remove points outside the 3D range
  centre_point = begin() + centre.Number();
  for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) {
    point = begin() + node->Number();
    if ((*point - *centre_point).Length() > range) {
      neighbourhood.erase(node);
      node--;
    }
  }

  return(neighbourhood);
}

/*
--------------------------------------------------------------------------------
  Label connected components determined by specified edges of connected points
--------------------------------------------------------------------------------
*/

void LaserPoints::LabelComponents(const TINEdges &edges, 
                                  const LaserPointTag label_tag,
								  bool erase_old_component_labels)
{
  PointNumberList                 neighbourhood;
  PointNumberList::const_iterator nb, node;
  TINEdges::const_iterator        neighbours;
  LaserPoints::iterator           point, nb_point;
  int                             node_index, number, new_label, minimum_label,
                                  maximum_label;

  // Remove possible old labels
  if (erase_old_component_labels) RemoveAttribute(label_tag);

  // Set first label number
  if (!erase_old_component_labels) {
  	/// Determine the current value range
    if (AttributeRange(label_tag, minimum_label, maximum_label))
      new_label = maximum_label + 1;
    else
      new_label = 1;
  }
  else
    new_label = 1;

  // Label all points
  for (point=begin(), number=0; point!=end(); point++, number++) {
    if (!point->HasAttribute(label_tag)) {
      neighbourhood.push_back(PointNumber(number));
      point->Attribute(label_tag) = new_label;
      for (node=neighbourhood.begin(), node_index=0;
           node!=neighbourhood.end();
           node++, node_index++) {
        neighbours = edges.begin() + node->Number();
        for (nb=neighbours->begin(); nb!=neighbours->end(); nb++) {
          nb_point = begin() + nb->Number();
          if (!nb_point->HasAttribute(label_tag)) {
            nb_point->Attribute(label_tag) = new_label;
            neighbourhood.push_back(*nb);
            node = neighbourhood.begin() + node_index;
          }
        }
      }
      neighbourhood.Erase();
      new_label++;
    }
  }
}

// Old function with labels in either reflectance or pulse count attribute
void LaserPoints::LabelComponents(const TINEdges &edges,
                                  bool store_label_in_reflectance)
{
  if (store_label_in_reflectance) LabelComponents(edges, ReflectanceTag, true);
  else LabelComponents(edges, PulseCountTag, true);
}

/*
--------------------------------------------------------------------------------
                         Remove long edges
--------------------------------------------------------------------------------
*/

void LaserPoints::RemoveLongEdges(TINEdges &edges, double max_dist,
                                  bool planimetric) const
{
  TINEdges::iterator          neighbours;
  LaserPoints::const_iterator point, nb_point;
  TINEdgeSet::iterator        nb_node;
  double                      dist;
  
  //printf("laser point size is %d", size());
  
  for (point=begin(), neighbours=edges.begin(); point!=end();
       point++, neighbours++) {
            
    for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) {
    //  printf("1---%d ", nb_node->Number());
      nb_point = begin() + nb_node->Number();
      //printf("1.5");
      if (planimetric) dist = (*point - *nb_point).Length2D();
      else dist = (*point - *nb_point).Length();
      if (dist > max_dist) {
        neighbours->erase(nb_node);
        nb_node--;
      }
    }
  }
}

/*
--------------------------------------------------------------------------------
                         Count mixed edges
--------------------------------------------------------------------------------
*/

int LaserPoints::CountMixedEdges(TINEdges &edges, LaserPointTag label_tag) const
{
  TINEdges::iterator          neighbours;
  LaserPoints::const_iterator point, nb_point;
  TINEdgeSet::iterator        nb_node;
  int                         total_mixed_edges;
  
  total_mixed_edges = 0;
  for (point=begin(), neighbours=edges.begin(); point!=end();
       point++, neighbours++) {
    for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) {
      nb_point = begin() + nb_node->Number();
      if (point->Attribute(label_tag)!=nb_point->Attribute(label_tag)){
         total_mixed_edges++;
      }
    }
  }
  return(total_mixed_edges);
}

// Possible operations:
// 0: Initial selection. Attribute value should be within specified range
// 1: AND. Point should be selected and value should be within range
// 2: OR. Point should be selected or value should be within range

void LaserPoints::Select(LaserPointTag attribute, double minimum,
                         double maximum, int operation)
{
  double                   value, min_val, max_val;
  LaserPoints::iterator    point;
  LaserPointAttributeType attribute_type;
  
  // Determine attribute type
  attribute_type = AttributeType(attribute);
  min_val = minimum;
  max_val = maximum;
  if (attribute_type == IntegerAttributeType) {
    min_val = minimum - 0.0001;
    max_val = maximum + 0.0001;
  }
  
  if (operation == 0) RemoveAttribute(IsSelectedTag);
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(attribute)) {
      switch (attribute_type) {
        case IntegerAttributeType: value = point->Attribute(attribute); break;	        
        case FloatAttributeType  : value = point->FloatAttribute(attribute); break;	        
        case DoubleAttributeType : value = point->DoubleAttribute(attribute); break;
        default: break;
      }
      switch (operation) {
        case 0: // Initial selection
        case 2: // or OR operation
          if (value >= min_val && value <= max_val) point->Select();
          break;
        case 1: // AND operation
          if (point->IsSelected())
            if (value < min_val || value > max_val) point->RemoveAttribute(IsSelectedTag);
          break;
        default:
          printf("Undefined operation type %d\n", operation);
          return;
      }
    }
  }
}

LaserPointAttributeType AttributeType(const LaserPointTag attribute)
{
  LaserPointTag point_attribute;
  
  // First deal with point attributes
  if (attribute < AverageReflectanceTag) {
    switch ((int) attribute) {
      case ResidualTag:
      case v_xTag:
      case v_yTag:
      case v_zTag:
      case cv_xyTag:
      case cv_xzTag:
      case cv_yzTag:
      case NormalXTag:
      case NormalYTag:
      case NormalZTag:
      case FlatnessTag:
      case LinearityTag:
      case ScaledNormalXTag:
      case ScaledNormalYTag:
      case ScaledNormalZTag:
      case ScalarTag:
      case AngleTag:
	  case HeightAboveGroundTag:
	  case Lambda0LocalTag:
	  case Lambda1LocalTag:
	  case Lambda2LocalTag:
	  case FlatnessLocalTag:
	  case LinearityLocalTag:
	  case Lambda0LocalScaledTag:
	  case Lambda1LocalScaledTag:
	  case Lambda2LocalScaledTag: return FloatAttributeType;
      case TimeTag:
      case XCoordinateTag:
      case YCoordinateTag:
      case ZCoordinateTag:        return DoubleAttributeType;
      case LongSegmentNumberTag:  return LongAttributeType;
      default:                    return IntegerAttributeType;
    }
  }
  
  // Determine basic point tag of a segment tag
  if (attribute < SlopeAngleVarianceTag) { // Average
    switch (attribute) {
      case AveragePointSpacingTag: return FloatAttributeType;
      default:
        point_attribute = BasicLaserPointTag(attribute);
        if (AttributeType(point_attribute) == DoubleAttributeType) return DoubleAttributeType;
        else return FloatAttributeType;
    }
  }
  else if (attribute < EdgeCountTag) { // Minimum or maximum
   point_attribute = BasicLaserPointTag(attribute);
   return AttributeType(point_attribute);
  }
  else if (attribute == EdgeCountTag || attribute == SegmentSizeTag ||
           attribute == ComponentSizeTag) // Point or edge count
    return IntegerAttributeType;
  return FloatAttributeType;
}

/*
--------------------------------------------------------------------------------
                         Return points on edges with two different labels
--------------------------------------------------------------------------------
*/
LaserPoints LaserPoints::ReturnMixedEdges(TINEdges &edges, 
                                          LaserPointTag label_tag) const
{
  TINEdges::iterator          neighbours;
  LaserPoints                 points_on_edge;
  LaserPoints::const_iterator point, nb_point;
  TINEdgeSet::iterator        nb_node;
  
  for (point=begin(), neighbours=edges.begin(); point!=end();
       point++, neighbours++) {
    for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) {
      nb_point = begin() + nb_node->Number();
      if (point->Attribute(label_tag)!=nb_point->Attribute(label_tag)){
         points_on_edge.push_back(*point);
       //  points_on_edge.push_back(*nb_point);
      }
    }
  }
  return(points_on_edge);
}

/*
--------------------------------------------------------------------------------
                         Return points on segment contours 
--------------------------------------------------------------------------------
*/

LaserPoints LaserPoints::ReturnContourPoints(int contour_number,
                                        const PointNumberList &component,
					                    const TINEdges &edges,
                                        bool same_attribute_value,
                                        const LaserPointTag tag) const
{
  LineTopology                    contour;
  LaserPoints                     edgelaserpoints;
  int                             foundnext, nbdir, value;
  TINEdges::const_iterator        edgeset;
  PointNumberList::const_iterator startpoint, point, prevpoint, nextpoint,
		                  curpoint, secondpoint;
  PointNumberList::iterator       conpoint;
  double                          maxX, X, dir, bestdir, anglesum, angle, pi;
  Vector3D                        p1, p2;

  contour.Number() = contour_number;

/* Use the point with the largest X-coordinate as the starting point */

  startpoint = component.begin();
  maxX = (begin() + startpoint->Number())->X();
  for (point=component.begin()+1; point!=component.end(); point++) {
    X = (begin() + point->Number())->X();
    if (X > maxX) {maxX = X; startpoint = point;}
  }
  contour.push_back(*startpoint);
  // Retrieve object's attribute value
  value = (begin() + startpoint->Number())->Attribute(tag);

/* Use the first point in counter-clockwise order starting from three o'clock
 * which is part of the component as the next point.
 */

  edgeset = edges.begin() + startpoint->Number();
  pi = 4.0 * atan(1.0);
  bestdir = 1.0;
  foundnext = 0;
  for (point = edgeset->begin(); point != edgeset->end(); point++) {
    if ((begin() + point->Number())->Attribute(tag) == value ||
        !same_attribute_value) {
    /* if (component.FindPoint(point->Number()) != -1) */
      dir = Direction2D(*startpoint, *point);
      if (dir > 0.0) dir -= 2.0 * pi;
      if (dir < bestdir) {bestdir = dir; nextpoint = point; foundnext = 1;}
    }
  }

/* If there is no next point, the component only has one point */

  if (!foundnext) {
    if (component.size() != 1)
      printf("Warning: Inconsistancy in deriving contour.\n");
      return(edgelaserpoints);
//    return(contour);
  }

/* Walk around the component contour until the first and second point are
 * encountered again.
 */

  anglesum = 0.0;
  prevpoint = startpoint;
  curpoint = nextpoint;
  secondpoint = nextpoint;
  do {
    contour.push_back(*curpoint);

    /* Look for the previous point in the edge set of the current point */

    edgeset = edges.begin() + curpoint->Number();
    nbdir = edgeset->FindPoint(prevpoint->Number()) + 1;
    if (nbdir == 0) {
      printf("Error: Inconsistancy in edge set\n");
      exit(0);
    }

    /* Find the next point that is part of the component */

    for (point = edgeset->begin() + nbdir, foundnext = 0;
	 nbdir < edgeset->size() && !foundnext;
	 nbdir++, point++) {
      if ((begin() + point->Number())->Attribute(tag) == value ||
          !same_attribute_value)
      /* if (component.FindPoint(point->Number()) != -1) */
	{foundnext = 1; nextpoint = point;}
    }
    if (!foundnext) {
      for (point = edgeset->begin(), nbdir = 0;
	   nbdir < edgeset->size() && !foundnext;
	   nbdir++, point++) {
        if ((begin() + point->Number())->Attribute(tag) == value ||
            !same_attribute_value)
        /* if (component.FindPoint(point->Number()) != -1) */
	  {foundnext = 1; nextpoint = point;}
      }
    }

    /* Determine the angle at the current point */

    angle = Angle2D(*prevpoint, *curpoint, *nextpoint);
    if (angle == 0) angle = 2 * pi;
    anglesum += angle;

    /* Prepare for the next point */

    prevpoint = curpoint;
    curpoint = nextpoint;
  } while (*prevpoint != *startpoint || *curpoint != *secondpoint);

/* Check whether the contour is the outer contour */

    contour.erase(contour.end()-1);
    LaserPoints::const_iterator laserpt;
    edgelaserpoints = LaserPoints(*this);
    edgelaserpoints.Label(10);
    edgelaserpoints.Label(contour, 8, LabelTag);
    for (conpoint=contour.begin(); conpoint!=contour.end(); conpoint++) {
        laserpt = begin() + conpoint->Number();
   //     edgelaserpoints.push_back(*laserpt);
    }
    //exit(0);
    return(edgelaserpoints);
}

const char * AttributeName(const LaserPointTag attribute, bool capital)
{
  switch ((int) attribute) {
    // Point attributes
    case ReflectanceTag: if (capital) return "Reflectance"; return "reflectance";
    case PulseCountWithFlagTag:
    case PulseCountTag : if (capital) return "Pulse count"; return "pulse count";
    case LabelTag: if (capital) return "Label"; return "label";
    case IsFilteredTag: if (capital) return "Is filtered"; return "is filtered";
    case IsProcessedTag: if (capital) return "Is processed"; return "is processed";
    case ColourTag: if (capital) return "Colour"; return "colour";
    case ResidualTag: if (capital) return "Residual"; return "residual";
    case IsSelectedTag: if (capital) return "Is selected"; return "is selected";
    case PlaneNumberTag: if (capital) return "Plane number"; return "plane number";
    case SegmentNumberTag: if (capital) return "Segment number"; return "segment number";
    case ScanNumberTag: if (capital) return "Scan number"; return "scan number";
    case PointNumberTag: if (capital) return "Point number"; return "point number";
    case PulseLengthTag: if (capital) return "Pulse length"; return "pulse length";
    case PolygonNumberTag: if (capital) return "Polygon number"; return "polygon number";
    case IntensityTag: if (capital) return "Intensity"; return "intensity";
    case v_xTag: return "X variance";
    case v_yTag: return "Y variance";
    case v_zTag: return "Z variance";
    case cv_xyTag: return "XY co-variance";
    case cv_xzTag: return "XZ co-variance";
    case cv_yzTag: return "YZ co-variance";
    case NumRaysTag: if (capital) return "Number of rays"; return "number of rays";
    case ScalarTag: if (capital) return "Scalar"; return "scalar";
    case TimeTag: if (capital) return "Time"; return "time";
    case AngleTag: if (capital) return "Angle"; return "angle";
    case XCoordinateTag: return "X-coordinate";
    case YCoordinateTag: return "Y-coordinate";
    case ZCoordinateTag: return "Z-coordinate";
    case ComponentNumberTag: if (capital) return "Component number"; return "component number";
    case NormalXTag: return "X-coordinate normal vector";
    case NormalYTag: return "Y-coordinate normal vector";
    case NormalZTag: return "Z-coordinate normal vector";
    case FlatnessTag: if (capital) return "Flatness"; return "flatness";
    case LinearityTag: if (capital) return "Linearity"; return "linearity";
    case ScaledNormalXTag: if (capital) return "Scaled X-coordinate normal vector";
                           return "scaled X-coordinate normal vector";
    case ScaledNormalYTag: if (capital) return "Scaled Y-coordinate normal vector";
                           return "scaled Y-coordinate normal vector";
    case ScaledNormalZTag: if (capital) return "Scaled Z-coordinate normal vector";
                           return "scaled Z-coordinate normal vector";
    case SegmentStartTileNumberTag: if (capital) return "Segment start tile number";
                                    return "segment start tile number";
    case RedTag: if (capital) return "Red"; return "red";
    case GreenTag: if (capital) return "Green"; return "green";
    case BlueTag: if (capital) return "Blue"; return "blue";
    case LongSegmentNumberTag: if (capital) return "Tile + segment number"; return "tile + segment number";
    case HeightAboveGroundTag: if (capital) return "Height above ground"; return "height above ground";
    case ConvexHullTag: if (capital) return "Convex hull point"; return "convex hull point";    
    case NearOtherSegmentTag: if (capital) return "Near other segment"; return "near other segment";    
    case ScanLineNumberTag: if (capital) return "Scan line number"; return "scan line number";
    case Lambda0LocalTag: if (capital) return "Local lambda0"; return "local lambda0";
    case Lambda1LocalTag: if (capital) return "Local lambda1"; return "local lambda1";
    case Lambda2LocalTag: if (capital) return "Local lambda2"; return "local lambda2";
    case FlatnessLocalTag: if (capital) return "Local flatness"; return "local flatness";
    case LinearityLocalTag: if (capital) return "Local linearity"; return "local linearity";
    case Lambda0LocalScaledTag: if (capital) return "Local lambda0 scaled"; return "local lambda0 scaled";
    case Lambda1LocalScaledTag: if (capital) return "Local lambda1 scaled"; return "local lambda1 scaled";
    case Lambda2LocalScaledTag: if (capital) return "Local lambda2 scaled"; return "local lambda2 scaled";
    // Averages
    case AverageReflectanceTag: if (capital) return "Average reflectance"; return "average reflectance";
    case AveragePulseCountTag: if (capital) return "Average pulse count"; return "average pulse count";
    case AverageIsFilteredTag: if (capital) return "Average filter status"; return "average filter status";
    case AverageIsProcessedTag: if (capital) return "Average processed status"; return "average processed status";
    case AverageColourTag: if (capital) return "Average colour"; return "average colour";
    case AverageResidualTag: if (capital) return "Average residual"; return "average residual";
    case AverageIsSelectedTag: if (capital) return "Average selected"; return "average selected";
    case AveragePulseLengthTag: if (capital) return "Average pulse length"; return "average pulse length";
    case AverageScalarTag: if (capital) return "Average scalar"; return "average scalar";
    case AverageTimeTag: if (capital) return "Average time"; return "average time";
    case AverageAngleTag: if (capital) return "Average angle"; return "average angle";
    case AverageXCoordinateTag: if (capital) return "Average X-coordinate"; return "average X-coordinate";
    case AverageYCoordinateTag: if (capital) return "Average Y-coordinate"; return "average Y-coordinate";
    case AverageZCoordinateTag: if (capital) return "Average Z-coordinate"; return "average Z-coordinate";
    case AverageHeightAboveGroundTag: if (capital) return "Average height above ground"; return "average height above ground";
    case AveragePointSpacingTag: if (capital) return "Average point spacing"; return "average point spacing";
    case AverageLambda0LocalTag: if (capital) return "Average local lambda0"; return "average local lambda0";
    case AverageLambda1LocalTag: if (capital) return "Average local lambda1"; return "average local lambda1";
    case AverageLambda2LocalTag: if (capital) return "Average local lambda2"; return "average local lambda2";
    case AverageFlatnessLocalTag: if (capital) return "Average local flatness"; return "average local flatness";
    case AverageLinearityLocalTag: if (capital) return "Average local linearity"; return "average local linearity";
    case AverageLambda0LocalScaledTag: if (capital) return "Average local lambda0 scaled"; return "average local lambda0 scaled";
    case AverageLambda1LocalScaledTag: if (capital) return "Average local lambda1 scaled"; return "average local lambda1 scaled";
    case AverageLambda2LocalScaledTag: if (capital) return "Average local lambda2 scaled"; return "average local lambda2 scaled";
    // Variation
    case SlopeAngleVarianceTag: if (capital) return "Slope angle variance"; return "slope angle variance";
    case Sum1Tag: if (capital) return "Sum1 (private use)"; return "sum1 (private use)";
    // Minima
    case MinReflectanceTag: if (capital) return "Minimum reflectance"; return "minimum reflectance";
    case MinPulseCountTag: if (capital) return "Minimum pulse count"; return "minimum pulse count";
    case MinLabelTag: if (capital) return "Minimum label"; return "minimum label";
    case MinResidualTag: if (capital) return "Minimum residual"; return "minimum residual";
    case MinPlaneNumberTag: if (capital) return "Minimum plane number"; return "minimum plane number";
    case MinScanNumberTag: if (capital) return "Minimum scan number"; return "minimum scan number";
    case MinPulseLengthTag: if (capital) return "Minimum pulse length"; return "minimum pulse length";
    case MinPolygonNumberTag: if (capital) return "Minimum polygon number"; return "minimum polygon number";
    case MinScalarTag: if (capital) return "Mininum scalar"; return "minimum scalar";
    case MinTimeTag: if (capital) return "Minimum time"; return "minimum time";
    case MinAngleTag: if (capital) return "Minimum angle"; return "minimum angle";
    case MinXCoordinateTag: if (capital) return "Minimum X-coordinate"; return "minimum X-coordinate";
    case MinYCoordinateTag: if (capital) return "Minimum Y-coordinate"; return "minimum Y-coordinate";
    case MinZCoordinateTag: if (capital) return "Minimum Z-coordinate"; return "minimum Z-coordinate";
    case MinHeightAboveGroundTag: if (capital) return "Minimum height above ground"; return "minimum height above ground";
    // Maxima
    case MaxReflectanceTag: if (capital) return "Maximum reflectance"; return "maximum reflectance";
    case MaxPulseCountTag: if (capital) return "Maximum pulse count"; return "maximum pulse count";
    case MaxLabelTag: if (capital) return "Maximum label"; return "maximum label";
    case MaxResidualTag: if (capital) return "Maximum residual"; return "maximum residual";
    case MaxPlaneNumberTag: if (capital) return "Maximum plane number"; return "maximum plane number";
    case MaxScanNumberTag: if (capital) return "Maximum scan number"; return "maximum scan number";
    case MaxPulseLengthTag: if (capital) return "Maximum pulse length"; return "maximum pulse length";
    case MaxPolygonNumberTag: if (capital) return "Maximum polygon number"; return "maximum polygon number";
    case MaxScalarTag: if (capital) return "Mininum scalar"; return "maximum scalar";
    case MaxTimeTag: if (capital) return "Maximum time"; return "maximum time";
    case MaxAngleTag: if (capital) return "Maximum angle"; return "maximum angle";
    case MaxXCoordinateTag: if (capital) return "Maximum X-coordinate"; return "maximum X-coordinate";
    case MaxYCoordinateTag: if (capital) return "Maximum Y-coordinate"; return "maximum Y-coordinate";
    case MaxZCoordinateTag: if (capital) return "Maximum Z-coordinate"; return "maximum Z-coordinate";
    case MaxHeightAboveGroundTag: if (capital) return "Maximum height above ground"; return "maximum height above ground";
    // Point and edge counts
    case EdgeCountTag: if (capital) return "Edge count"; return "edge count";
    case ComponentSizeTag: if (capital) return "Component size"; return "component size";
    case SegmentSizeTag: if (capital) return "Segment size"; return "segment size";
    // Percentages
    case PercFirstPulseTag: if (capital) return "Percentage first pulse"; return "percentage first pulse";
    case PercSecondPulseTag: if (capital) return "Percentage second pulse"; return "percentage second pulse";
    case PercThirdPulseTag: if (capital) return "Percentage third pulse"; return "percentage third pulse";
    case PercFourthPulseTag: if (capital) return "Percentage fourth pulse"; return "percentage fourth pulse";
    case PercLastPulseTag: if (capital) return "Percentage last pulse"; return "percentage last pulse";
    case PercNotFirstPulseTag: if (capital) return "Percentage not first pulse"; return "percentage not first pulse";
    case PercNotLastPulseTag: if (capital) return "Percentage not last pulse"; return "percentage not last pulse";
    case PercMultiPulseTag: if (capital) return "Percentage multiple pulse"; return "percentage multipe pulse";
    case PercIsFilteredTag: if (capital) return "Percentage filtered"; return "percentage filtered";
    case PercIsProcessedTag: if (capital) return "Percentage processed"; return "percentage processed";
    case PercIsSelectedTag: if (capital) return "Percentage selected"; return "percentage selected";
    case PercNearOtherSegmentTag: if (capital) return "Percentage near other segment"; return "percentage near other segment";
    // Plane attributes
    case InclinationTag: if (capital) return "Inclination"; return "inclination";
    case AzimuthTag: if (capital) return "Azimuth"; return "azimuth";
    case Lambda0Tag: if (capital) return "Lambda0"; return "lambda0";
    case Lambda1Tag: if (capital) return "Lambda1"; return "lambda1";
    case Lambda2Tag: if (capital) return "Lambda2"; return "lambda2";
    case Lambda0ScaledTag: if (capital) return "Lambda0 scaled"; return "lambda0 scaled";
    case Lambda1ScaledTag: if (capital) return "Lambda1 scaled"; return "lambda1 scaled";
    case Lambda2ScaledTag: if (capital) return "Lambda2 scaled"; return "lambda2 scaled";

    case DoubleTag: if (capital) return "Second part of double value"; return "second part of double value";
    default: if (capital) return "Unknown"; return "unknown";  
  }
}

const char * AttributeName(LaserPointTag attribute)
{ return AttributeName(attribute, true); }
    

bool KnownAttribute(const LaserPointTag attribute)
{
  return (strncmp(AttributeName(attribute), "Unknown", 7) != 0);
}
  
LaserPointTag BasicLaserPointTag(LaserPointTag segment_attribute)
{
  switch (segment_attribute) {
    case AverageReflectanceTag:
    case MinReflectanceTag:
    case MaxReflectanceTag:            return ReflectanceTag;
    
    case AveragePulseCountTag:
    case MinPulseCountTag:
    case MaxPulseCountTag:
    case PercFirstPulseTag:          
    case PercSecondPulseTag:
    case PercThirdPulseTag:
    case PercFourthPulseTag:
    case PercLastPulseTag:
    case PercNotFirstPulseTag:
    case PercNotLastPulseTag:
    case PercMultiPulseTag:            return PulseCountTag;
    
    case AverageIsFilteredTag:
    case PercIsFilteredTag:            return IsFilteredTag;
    
    case AverageIsProcessedTag:
    case PercIsProcessedTag:           return IsProcessedTag;
    
    case AverageColourTag:             return ColourTag;
    
    case AverageResidualTag:
    case MinResidualTag:
    case MaxResidualTag:               return ResidualTag;
    
    case AverageIsSelectedTag:
    case PercIsSelectedTag:            return IsSelectedTag;
    
    case PercNearOtherSegmentTag:      return NearOtherSegmentTag;

    case AveragePulseLengthTag:
    case MinPulseLengthTag:
    case MaxPulseLengthTag:            return PulseLengthTag;
    
    case AverageScalarTag:
    case MinScalarTag:
    case MaxScalarTag:                 return ScalarTag;
    
    case AverageTimeTag:
    case MinTimeTag:
    case MaxTimeTag:                   return TimeTag;
    
    case AverageAngleTag:
    case MinAngleTag:
    case MaxAngleTag:                  return AngleTag;
     
    case AverageXCoordinateTag:
    case MinXCoordinateTag:
    case MaxXCoordinateTag:            return XCoordinateTag;
    
    case AverageYCoordinateTag:
    case MinYCoordinateTag:
    case MaxYCoordinateTag:            return YCoordinateTag;
    
    case AverageZCoordinateTag:
    case MinZCoordinateTag:
    case MaxZCoordinateTag:            return ZCoordinateTag;
    
    case MinLabelTag:
    case MaxLabelTag:                  return LabelTag;
    
    case MinPlaneNumberTag:
    case MaxPlaneNumberTag:            return PlaneNumberTag;
    
    case MinScanNumberTag:
    case MaxScanNumberTag:             return ScanNumberTag;
    
    case MinPolygonNumberTag:
    case MaxPolygonNumberTag:          return PolygonNumberTag;
    
    case AverageHeightAboveGroundTag:
    case MinHeightAboveGroundTag:
    case MaxHeightAboveGroundTag:      return HeightAboveGroundTag;
    	
    case SegmentSizeTag:               return SegmentNumberTag;
    
    case ComponentSizeTag:             return ComponentNumberTag;
    
    case AverageLambda0LocalTag:       return Lambda0LocalTag;
    case AverageLambda1LocalTag:       return Lambda1LocalTag;
    case AverageLambda2LocalTag:       return Lambda2LocalTag;
    case AverageFlatnessLocalTag:      return FlatnessLocalTag;
    case AverageLinearityLocalTag:     return LinearityLocalTag;
    case AverageLambda0LocalScaledTag: return Lambda0LocalScaledTag;
    case AverageLambda1LocalScaledTag: return Lambda1LocalScaledTag;
    case AverageLambda2LocalScaledTag: return Lambda2LocalScaledTag;
    
    default: return NoTag;
  }
}

void LaserPoints::DerivePointAttribute(LaserPointTag point_attribute,
                                       const SegmentationParameters &parameters)
{
  bool scale_eigenvalues;
  
  // Derive normal vector, flatness and/or linearity
  if (point_attribute >= NormalXTag &&
      point_attribute <= LinearityTag) {
    CalculateNormals(parameters.NumberOfNeighbours());
	return;
  }
    
  // Derive scaled normal vector
  if (point_attribute >= ScaledNormalXTag &&
      point_attribute <= ScaledNormalZTag) {
    CalculateScaledNormals(parameters.NumberOfNeighbours());
    return;
  }
  
  // Derive local eigenvalues and shape parameters
  if (point_attribute >= Lambda0LocalTag &&
      point_attribute <= Lambda2LocalScaledTag) {
    scale_eigenvalues = (point_attribute >= Lambda0LocalScaledTag);
    CalculateLocalEigenValues(parameters, scale_eigenvalues);
    if (point_attribute >= FlatnessLocalTag &&
        point_attribute <= LinearityLocalTag)
      CalculateLocalShape();
	return;  	
  }
  
  
  printf("Error: Point attribute %s cannot be calculated\n",
         AttributeName(point_attribute));
}

void LaserPoints::DeriveSegmentAttribute(LaserPointTag segment_attribute,
                                         LaserPointTag default_segment_tag)
{
  LaserPointTag           point_attribute, size_tag, segment_tag;
  LaserPointAttributeType attribute_type;
  int                     attribute_category, min_segm_num, max_segm_num,
                          *counts, pulse_type, segment;
  double                  *values, value, dist, eigenvalue_sum;
  Planes                  planes;
  LaserPoints::iterator   point;
  Vector3D                normal;
  
  // Check if the segment tag is valid
  if (default_segment_tag != SegmentNumberTag &&
      default_segment_tag != ComponentNumberTag) {
    printf("Error in LaserPoints::DeriveSegmentAttribute: Segment numbers should\n");
    printf("  be in SegmentNumberTag or ComponentNumberTag\n");
    return;
  }
  
  // Average point spacing only implemented in programme blockattributes
  if (segment_attribute = AveragePointSpacingTag) {
  	printf("AveragePointSpacing is only implemented in programme blockattributes.\n");
  	return;
  }
  
  // Overrule the provided segment tag in case of segment or component size attribute
  if (segment_attribute == SegmentSizeTag) segment_tag = SegmentNumberTag;
  else if (segment_attribute == ComponentSizeTag) segment_tag = ComponentNumberTag;
  else segment_tag = default_segment_tag;
  
  // Get corresponding size tag
  if (segment_tag == SegmentNumberTag) size_tag = SegmentSizeTag;
  else size_tag = ComponentSizeTag;
  
  // Check if this is a segment attribute
  if (segment_attribute < AverageReflectanceTag) {
    printf("Tag %d is not a segment attribute tag\n", segment_attribute);
    return;
  }
  
  // Determine the category of attribute 
  if (segment_attribute < SlopeAngleVarianceTag) attribute_category = 1; // Average
  else if (segment_attribute < MinReflectanceTag) { // Variation
    printf("Variations of point attributes currently not implemented in DeriveSegmentAttribute\n");
    printf("Only used for the programme blockattributes\n");
    return;
  }
  else if (segment_attribute < MaxReflectanceTag) attribute_category = 2; // Minimum
  else if (segment_attribute < EdgeCountTag) attribute_category = 3; // Maximum
  else if (segment_attribute <= SegmentSizeTag) attribute_category = 4; // Segment or component size
  else if (segment_attribute <= PercMultiPulseTag) attribute_category = 5; // Pulse type percentage
  else if (segment_attribute < InclinationTag) attribute_category = 6; // Other percentage
  else attribute_category = 7; // Plane attribute
  
  // Base attribute
  point_attribute = BasicLaserPointTag(segment_attribute);
  
  // Attribute type (integer, float, double)
  attribute_type = AttributeType(point_attribute);
  
  // Pulse types
  if (attribute_category == 5) {
    switch (segment_attribute) {
      default:
      case PercFirstPulseTag:    pulse_type = FirstPulse; break;
      case PercSecondPulseTag:   pulse_type = SecondPulse; break;
      case PercThirdPulseTag:    pulse_type = ThirdPulse; break;
      case PercFourthPulseTag:   pulse_type = FourthPulse; break;
      case PercLastPulseTag:     pulse_type = LastPulse; break;
      case PercNotFirstPulseTag: pulse_type = NotFirstPulse; break;
      case PercNotLastPulseTag:  pulse_type = NotLastPulse; break;
      case PercMultiPulseTag:    pulse_type = MultiplePulse; break;
    }
  }
  
  // Determine maximum segment number
  if (!AttributeRange(segment_tag, min_segm_num, max_segm_num)) {
    printf("Error in LaserPoints::DeriveSegmentAttribute: Points have no segment numbers\n");
    return;
  }
  
  // Initialise segment value arrays
  values = (double *) calloc(max_segm_num+1, sizeof(double));
  counts = (int *) calloc(max_segm_num+1, sizeof(int));
  if (attribute_category == 7) { // Plane attributes
    planes.reserve(max_segm_num+1);
    for (int i=0; i<max_segm_num+1; i++) planes.push_back(Plane());
  }
  
  // Collect point information
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(segment_tag)) {
      // Determine segment number
      segment = point->Attribute(segment_tag);
      // Determine point attribute value, if required
      if (attribute_category != 7) {
        switch (attribute_type) {
          default:
          case IntegerAttributeType: 
            value = (double) point->Attribute(point_attribute); break;
          case FloatAttributeType:
            value = (double) point->FloatAttribute(point_attribute); break;
          case DoubleAttributeType:
            value = point->DoubleAttribute(point_attribute); break;
        }
      }
      // Process point value
      switch (attribute_category) {
        case 1: // Average
          values[segment] += value; break;
        case 2: // Minimum
          if (counts[segment] == 0) values[segment] = value;
          else if (value < values[segment]) values[segment] = value;
          break;
        case 3: // Maximum
          if (counts[segment] == 0) values[segment] = value;
          else if (value > values[segment]) values[segment] = value;
          break;
        default:
        case 4: // Segment size
          break; // Nothing to do, just counting
        case 5: // Pulse type percentage
          if (point->IsPulseType((LaserPulseType) pulse_type))
            values[segment] += 1.0;
          break;
        case 6: // Other percentages
          if (value > 0.01) values[segment] += 1.0;
          break;
        case 7: // Plane attribute
          planes[segment].AddPoint(point->Position3DRef(), false);
          break;
      }
      counts[segment]++;
    }
  }
  
  // Calculate segment attribute
  switch (attribute_category) {
    case 1: // Average
      for (segment=0; segment<=max_segm_num; segment++)
        if (counts[segment] > 0.0) values[segment] /= counts[segment];
      break;
    case 2: // Minimum
    case 3: // Maximum
    case 4: // Segment size
      break; // Nothing to do
    case 5: // Pulse type percentages
    case 6: // Other percentages
      for (segment=0; segment<=max_segm_num; segment++)
        if (counts[segment] > 0.0) values[segment] *= 100.0 / counts[segment];
      break;
    case 7: // Plane attributes
      // Fit the plane if there are at least three points
      for (segment=0; segment<=max_segm_num; segment++) {
        if (counts[segment] >= 3) {
          planes[segment].Recalculate();
          normal = planes[segment].Normal();
          eigenvalue_sum = planes[segment].FloatAttribute(PT_Lambda0) +
                           planes[segment].FloatAttribute(PT_Lambda1) +
                           planes[segment].FloatAttribute(PT_Lambda2);
          if (normal.Z() < 0.0) normal *= -1.0;
          switch (segment_attribute) {
            case InclinationTag:
              dist = normal.vect2D().Length();
              values[segment] = atan2(dist, fabs(normal.Z())); break;
            case AzimuthTag:
              if (normal.X() == 0.0 && normal.Y() == 0.0)
                values[segment] = 0.0;
              else {
                values[segment] = atan2(normal.X(), normal.Y());
                if (values[segment] < 0.0) values[segment] += atan(1.0) * 8.0;
              }
              break;
            case Lambda0Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda0); break;
            case Lambda1Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda1); break;
            case Lambda2Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda2); break;
            case Lambda0ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda0) / eigenvalue_sum; break;
            case Lambda1ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda1) / eigenvalue_sum; break;
            case Lambda2ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda2) / eigenvalue_sum; break;
            default: // Undefined, should not get here
              values[segment] = 999.0; break;
          }
        }
        else
          values[segment] = 999.0;
        planes[segment].Initialise();
      }
      break;
  }
      
  // Store segment attributes in points      
  attribute_type = AttributeType(segment_attribute);
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(segment_tag)) {
      segment = point->Attribute(segment_tag);
      switch (attribute_type) {
        default:
        case IntegerAttributeType:
          point->SetAttribute(segment_attribute, (int) (values[segment]+0.5));
          break;
        case FloatAttributeType:
          point->SetAttribute(segment_attribute, (float) values[segment]);
          break;
        case DoubleAttributeType:
          point->SetDoubleAttribute(segment_attribute, values[segment]);
          break;
      }
      point->SetAttribute(size_tag, counts[segment]);
    }
    else point->SetAttribute(size_tag, 0);
  }
  
  // Free local vectors
  free(values);
  free(counts);
  if (attribute_category == 7) planes.Erase();
}

// Special version for long segment numbers, including tile numbers
void LaserPoints::DeriveSegmentAttribute(LaserPointTag segment_attribute)
{
  LaserPointTag           point_attribute, size_tag;
  LaserPointAttributeType attribute_type;
  int                     attribute_category, pulse_type, index, num_segments;
  long long int           segment;
  double                  value, dist, eigenvalue_sum;
  Planes                  planes;
  LaserPoints::iterator   point;
  Vector3D                normal;
  vector<long long int>   segment_numbers;
  vector<long long int>::iterator segment_iterator;
  vector<int>             counts;
  vector<double>          values;
  
  // Check if this is a segment attribute
  if (segment_attribute < AverageReflectanceTag) {
    printf("Tag %d is not a segment attribute tag\n", segment_attribute);
    return;
  }
  
  // Average point spacing only implemented in programme blockattributes
  if (segment_attribute = AveragePointSpacingTag) {
  	printf("AveragePointSpacing is only implemented in programme blockattributes.\n");
  	return;
  }
  
  // Determine the category of attribute
  if (segment_attribute < SlopeAngleVarianceTag) attribute_category = 1; // Average
  else if (segment_attribute < MinReflectanceTag) { // Variation
    printf("Variations of point attributes currently not implemented in DeriveSegmentAttribute\n");
    printf("Only used for the programme blockattributes\n");
    return;
  }
  else if (segment_attribute < MaxReflectanceTag) attribute_category = 2; // Minimum
  else if (segment_attribute < EdgeCountTag) attribute_category = 3; // Maximum
  else if (segment_attribute <= SegmentSizeTag) attribute_category = 4; // Segment or component size
  else if (segment_attribute <= PercMultiPulseTag) attribute_category = 5; // Pulse type percentage
  else if (segment_attribute < InclinationTag) attribute_category = 6; // Other percentage
  else attribute_category = 7; // Plane attribute
  
  // Base attribute
  point_attribute = BasicLaserPointTag(segment_attribute);
  
  // Attribute type (integer, float, double)
  attribute_type = AttributeType(point_attribute);
  
  // Pulse types
  if (attribute_category == 5) {
    switch (segment_attribute) {
      default:
      case PercFirstPulseTag:    pulse_type = FirstPulse; break;
      case PercSecondPulseTag:   pulse_type = SecondPulse; break;
      case PercThirdPulseTag:    pulse_type = ThirdPulse; break;
      case PercFourthPulseTag:   pulse_type = FourthPulse; break;
      case PercLastPulseTag:     pulse_type = LastPulse; break;
      case PercNotFirstPulseTag: pulse_type = NotFirstPulse; break;
      case PercNotLastPulseTag:  pulse_type = NotLastPulse; break;
      case PercMultiPulseTag:    pulse_type = MultiplePulse; break;
    }
  }
  
  // Collect point information
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      // Determine segment number
      segment = point->LongSegmentNumber();
      
      // If this is the first point of this segment, initialise some values
      segment_iterator = std::find(segment_numbers.begin(), 
	                               segment_numbers.end(), segment);
	  if (segment_iterator == segment_numbers.end()) {
	    segment_numbers.push_back(segment);
	    counts.push_back(0);
	    values.push_back(0.0);
	    if (attribute_category == 7) planes.push_back(Plane());
	    segment_iterator = segment_numbers.end() - 1;
	  }
	  
      // Get the right index
      index = std::distance(segment_numbers.begin(), segment_iterator);
      
      // Determine point attribute value, if required
      if (attribute_category != 7) {
        switch (attribute_type) {
          default:
          case IntegerAttributeType: 
            value = (double) point->Attribute(point_attribute); break;
          case FloatAttributeType:
            value = (double) point->FloatAttribute(point_attribute); break;
          case DoubleAttributeType:
            value = point->DoubleAttribute(point_attribute); break;
        }
      }
      // Process point value
      switch (attribute_category) {
        case 1: // Average
          values[index] += value; break;
        case 2: // Minimum
          if (counts[index] == 0) values[segment] = value;
          else if (value < values[index]) values[index] = value;
          break;
        case 3: // Maximum
          if (counts[index] == 0) values[index] = value;
          else if (value > values[index]) values[index] = value;
          break;
        default:
        case 4: // Segment size
          break; // Nothing to do, just counting
        case 5: // Pulse type percentage
          if (point->IsPulseType((LaserPulseType) pulse_type))
            values[index] += 1.0;
          break;
        case 6: // Other percentages
          if (value > 0.01) values[index] += 1.0;
          break;
        case 7: // Plane attribute
          planes[index].AddPoint(point->Position3DRef(), false);
          break;
      }
      counts[index]++;
    }
  }
  
  // Calculate segment attribute
  num_segments = segment_numbers.size();
  switch (attribute_category) {
    case 1: // Average
      for (index=0; index<=num_segments; index++)
        if (counts[index] > 0.0) values[index] /= counts[index];
      break;
    case 2: // Minimum
    case 3: // Maximum
    case 4: // Segment size
      break; // Nothing to do
    case 5: // Pulse type percentages
    case 6: // Other percentages
      for (index=0; index<=num_segments; index++)
        if (counts[index] > 0.0) values[index] *= 100.0 / counts[index];
      break;
    case 7: // Plane attributes
      // Fit the plane if there are at least three points
      for (index=0; index<=num_segments; index++) {
        if (counts[index] >= 3) {
          planes[index].Recalculate();
          normal = planes[index].Normal();
          if (normal.Z() < 0.0) normal *= -1.0;
          eigenvalue_sum = planes[segment].FloatAttribute(PT_Lambda0) +
                           planes[segment].FloatAttribute(PT_Lambda1) +
                           planes[segment].FloatAttribute(PT_Lambda2);
          switch (segment_attribute) {
            case InclinationTag:
              dist = normal.vect2D().Length();
              values[segment] = atan2(dist, fabs(normal.Z())); break;
            case AzimuthTag:
              if (normal.X() == 0.0 && normal.Y() == 0.0)
                values[segment] = 0.0;
              else {
                values[segment] = atan2(normal.X(), normal.Y());
                if (values[segment] < 0.0) values[segment] += atan(1.0) * 8.0;
              }
              break;
            case Lambda0Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda0); break;
            case Lambda1Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda1); break;
            case Lambda2Tag:
              values[segment] = planes[segment].FloatAttribute(PT_Lambda2); break;
            case Lambda0ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda0) / eigenvalue_sum; break;
            case Lambda1ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda1) / eigenvalue_sum; break;
            case Lambda2ScaledTag:
              if (eigenvalue_sum == 0.0)
                values[segment] = 1.0 / 3.0;
              else
                values[segment] = planes[segment].FloatAttribute(PT_Lambda2) / eigenvalue_sum; break;
            default: // Undefined, should not get here
              values[segment] = 999.0; break;
          }
        }
        else
          values[index] = 999.0;
        planes[index].Initialise();
      }
      break;
  }
      
  // Store segment attributes in points      
  attribute_type = AttributeType(segment_attribute);
  for (point=begin(); point!=end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      segment = point->LongSegmentNumber();
      // Get the right index
      segment_iterator = std::find(segment_numbers.begin(), 
	                               segment_numbers.end(), segment);    	
      index = std::distance(segment_numbers.begin(), segment_iterator);
      switch (attribute_type) {
        default:
        case IntegerAttributeType:
          point->SetAttribute(segment_attribute, (int) (values[index]+0.5));
          break;
        case FloatAttributeType:
          point->SetAttribute(segment_attribute, (float) values[index]);
          break;
        case DoubleAttributeType:
          point->SetDoubleAttribute(segment_attribute, values[index]);
          break;
      }
      point->SetAttribute(size_tag, counts[index]);
    }
    else point->SetAttribute(size_tag, 0);
  }
  
  // Free local vectors
  values.erase(values.begin(), values.end());
  counts.erase(counts.begin(), counts.end());
  segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
  if (attribute_category == 7) planes.Erase();
}


void LaserPoints::UnlabelSmallSegments(LaserPointTag tag, int minimum_size)
{
  LaserPoints::iterator   point;
  vector<int>             tile_numbers, max_segment_numbers;
  vector<int>::iterator   tile_number_iterator, max_segment_number;
  vector<int *>           segment_number_counts;
  vector<int *>::iterator segment_number_count;
  int                     tile_number, index, count_array;
  
  // Check on availability of segment number
  if (!HasAttribute(tag)) return;

  // Determine the highest segment number for every tile number
  for (point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Update the maximum segment number for this tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
	if (tile_number_iterator == tile_numbers.end()) {
	  tile_numbers.push_back(tile_number);
	  max_segment_numbers.push_back(point->Attribute(tag));
	}
	else {
	  index = std::distance(tile_numbers.begin(), tile_number_iterator);
	  max_segment_number = max_segment_numbers.begin() + index;
      if (point->Attribute(tag) > *max_segment_number)
        *max_segment_number = point->Attribute(tag);
	}
  }
  
  // Create counter arrays for all tiles
  for (max_segment_number=max_segment_numbers.begin();
       max_segment_number!=max_segment_numbers.end();
	   max_segment_number++) {
	count_array = (int *) calloc(*max_segment_number+1, sizeof(int));
	segment_number_counts.push_back(count_array);
  }
  
  // Count the segment numbers
  for (point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Get the count array of the right tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
    index = std::distance(tile_numbers.begin(), tile_number_iterator);
    segment_number_count = segment_number_counts.begin() + index;
    
    // Update count
    (*segment_number_count)[point->Attribute(tag)]++;
  }

  // Unlabel points of small segments
  for (point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Get the count array of the right tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
    index = std::distance(tile_numbers.begin(), tile_number_iterator);
    segment_number_count = segment_number_counts.begin() + index;
    
    // Remove segment number if the count is too low
    if ((*segment_number_count)[point->Attribute(tag)] < minimum_size)
      point->RemoveAttribute(tag);
  }
  
  // Clean up all allocated memory
  for (segment_number_count=segment_number_counts.begin();
       segment_number_count!=segment_number_counts.end();
	   segment_number_count++)
	free(*segment_number_count);
  tile_numbers.erase(tile_numbers.begin(), tile_numbers.end());
  max_segment_numbers.erase(max_segment_numbers.begin(), max_segment_numbers.end());
  segment_number_counts.erase(segment_number_counts.begin(), segment_number_counts.end());
}

void LaserPoints::RemoveSmallSegments(LaserPointTag tag, int minimum_size)
{
  LaserPoints::iterator   point, good_point;
  vector<int>             tile_numbers, max_segment_numbers;
  vector<int>::iterator   tile_number_iterator, max_segment_number;
  vector<int *>           segment_number_counts;
  vector<int *>::iterator segment_number_count;
  int                     tile_number, index, count_array;
  
  // Check on availability of segment number
  if (!HasAttribute(tag)) return;

  // Determine the highest segment number for every tile number
  for (point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Update the maximum segment number for this tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
	if (tile_number_iterator == tile_numbers.end()) {
	  tile_numbers.push_back(tile_number);
	  max_segment_numbers.push_back(point->Attribute(tag));
	}
	else {
	  index = std::distance(tile_numbers.begin(), tile_number_iterator);
	  max_segment_number = max_segment_numbers.begin() + index;
      if (point->Attribute(tag) > *max_segment_number)
        *max_segment_number = point->Attribute(tag);
	}
  }
  
  // Create counter arrays for all tiles
  for (max_segment_number=max_segment_numbers.begin();
       max_segment_number!=max_segment_numbers.end();
	   max_segment_number++) {
	count_array = (int *) calloc(*max_segment_number+1, sizeof(int));
	segment_number_counts.push_back(count_array);
  }
  
  // Count the segment numbers
  for (point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Get the count array of the right tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
    index = std::distance(tile_numbers.begin(), tile_number_iterator);
    segment_number_count = segment_number_counts.begin() + index;
    
    // Update count
    (*segment_number_count)[point->Attribute(tag)]++;
  }

  // Remove points of small segments
  for (point=begin(), good_point=begin(); point!=end(); point++) {
  	// Ignore points without segment numbers
  	if (!point->HasAttribute(tag)) continue;

  	// Get the tile number
  	if (point->HasAttribute(SegmentStartTileNumberTag))
  	  tile_number = point->Attribute(SegmentStartTileNumberTag);
  	else
  	  tile_number = 0;
  	  
    // Get the count array of the right tile
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                 tile_number);
    index = std::distance(tile_numbers.begin(), tile_number_iterator);
    segment_number_count = segment_number_counts.begin() + index;
    
    // Keep this point if it belongs to a large enough segment
    if ((*segment_number_count)[point->Attribute(tag)] >= minimum_size) {
      *good_point = *point;
      good_point++;
    }
  }
  erase(good_point, end());
  
  // Clean up all allocated memory
  for (segment_number_count=segment_number_counts.begin();
       segment_number_count!=segment_number_counts.end();
	   segment_number_count++)
	free(*segment_number_count);
  tile_numbers.erase(tile_numbers.begin(), tile_numbers.end());
  max_segment_numbers.erase(max_segment_numbers.begin(), max_segment_numbers.end());
  segment_number_counts.erase(segment_number_counts.begin(), segment_number_counts.end());
}

void LaserPoints::MarkConvexHullPoints(LaserPointTag tag)
{
  TIN                 *tin=VerifyTIN();
  TIN::const_iterator mesh;
  const PointNumber   *nodes;
  const MeshNumber    *neighbour_meshes;
  
  SetAttribute(tag, 0);
  for (mesh=tin->begin(); mesh!=tin->end(); mesh++) {
  	nodes = mesh->Nodes();
  	neighbour_meshes = mesh->Neighbours();
  	if (neighbour_meshes[0].Number() == -1) { // Edge on hull
  	  (begin() + nodes[1].Number())->Attribute(tag) = 1;
  	  (begin() + nodes[2].Number())->Attribute(tag) = 1;
  	}
  	if (neighbour_meshes[1].Number() == -1) { // Edge on hull
  	  (begin() + nodes[0].Number())->Attribute(tag) = 1;
  	  (begin() + nodes[2].Number())->Attribute(tag) = 1;
  	}
  	if (neighbour_meshes[2].Number() == -1) { // Edge on hull
  	  (begin() + nodes[1].Number())->Attribute(tag) = 1;
  	  (begin() + nodes[0].Number())->Attribute(tag) = 1;
  	}
  }
}

void LaserPoints::CalculateLocalEigenValues(const SegmentationParameters &parameters,
                                            bool scale_eigenvalues)
{
  LaserPoints::iterator     point, nbh_point;
  Plane                     plane;
  int                       point_number;
  PointNumberList           nbh_nodes;
  PointNumberList::iterator nbh_node;
  bool                      delete_edges;
  TINEdges                  *edges;
  double                    sum;
  
  // Check if there are edges
  delete_edges = (GetNeighbourhoodEdges() == NULL); // Delete edges again if they're currently not stored
  edges = VerifyEdges(parameters);

  for (point=begin(), point_number=0; point!=end(); point++, point_number++) {
  	// Get the neighbourhood
  	nbh_nodes = Neighbourhood(PointNumber(point_number),
                              parameters.SeedNeighbourhoodRadius(), 
							  edges->TINEdgesRef(),
                              parameters.DistanceMetricDimension() == 2,
                              parameters.SeedNeighbourhoodDefinition() == 0);

  	// Add points to the plane
    for (nbh_node=nbh_nodes.begin(); nbh_node!=nbh_nodes.end(); nbh_node++) {
      nbh_point = begin() + nbh_node->Number();
      // Only use points of the same segment if there are segment numbers
      if (point->HasAttribute(SegmentNumberTag)) {
      	if (!nbh_point->HasAttribute(SegmentNumberTag)) continue;
      	if (point->Attribute(SegmentNumberTag) !=
		    nbh_point->Attribute(SegmentNumberTag)) continue;
	  }
	  // Additional check for long segment numbers including tile numbers
      if (point->HasAttribute(SegmentStartTileNumberTag)) {
      	if (!nbh_point->HasAttribute(SegmentStartTileNumberTag)) continue;
      	if (point->Attribute(SegmentStartTileNumberTag) !=
		    nbh_point->Attribute(SegmentStartTileNumberTag)) continue;
      }
      plane.AddPoint(nbh_point->Position3DRef(), false);
    } 
	 	
  	// Set the eigenvalues
  	plane.Recalculate();
  	if (scale_eigenvalues) {
  	  sum = plane.Eigenvalue(0) + plane.Eigenvalue(1) + plane.Eigenvalue(2);
  	  if (sum == 0.0) {
 	    point->FloatAttribute(Lambda0LocalScaledTag) = 1.0 / 3.0;
 	    point->FloatAttribute(Lambda1LocalScaledTag) = 1.0 / 3.0;
 	    point->FloatAttribute(Lambda2LocalScaledTag) = 1.0 / 3.0;
  	  }
  	  else {
 	    point->FloatAttribute(Lambda0LocalScaledTag) = plane.Eigenvalue(0) / sum;
 	    point->FloatAttribute(Lambda1LocalScaledTag) = plane.Eigenvalue(1) / sum;
 	    point->FloatAttribute(Lambda2LocalScaledTag) = plane.Eigenvalue(2) / sum;
  	  }
  	}
  	else {
 	  point->FloatAttribute(Lambda0LocalTag) = plane.Eigenvalue(0);
 	  point->FloatAttribute(Lambda1LocalTag) = plane.Eigenvalue(1);
 	  point->FloatAttribute(Lambda2LocalTag) = plane.Eigenvalue(2);
  	}
  	plane.Erase();
  }
  
  // Erase edges if computed in this function
  if (delete_edges) EraseNeighbourhoodEdges();
  nbh_nodes.Erase();
}

void LaserPoints::CalculateLocalShape()
{
  LaserPoints::iterator point;
  double sum;
  bool use_scaled_eigenvalues=false;
  
  // Check if local eigenvalues are available
  if (!HasAttribute(Lambda0LocalTag) ||
      !HasAttribute(Lambda1LocalTag) ||
	  !HasAttribute(Lambda2LocalTag)) {
	use_scaled_eigenvalues = true;
    if (!HasAttribute(Lambda0LocalScaledTag) ||
        !HasAttribute(Lambda1LocalScaledTag) ||
	    !HasAttribute(Lambda2LocalScaledTag)) {
	  printf("Error: LaserPoints::CalculateLocalShape requires precalculated local eigenvalues\n");
	  return;
    }
  }
  
  for (point=begin(); point!=end(); point++)
    if (use_scaled_eigenvalues) {
      if (point->HasAttribute(Lambda0LocalScaledTag) &&
	      point->HasAttribute(Lambda1LocalScaledTag) &&
		  point->HasAttribute(Lambda2LocalScaledTag)) {
	    sum = point->FloatAttribute(Lambda1LocalScaledTag) + 
		      point->FloatAttribute(Lambda2LocalScaledTag);
	    if (sum == 0.0) point->FloatAttribute(FlatnessLocalTag) = 1.0;
	    else
	      point->FloatAttribute(FlatnessLocalTag)  = 
	        (point->FloatAttribute(Lambda1LocalScaledTag) - 
			 point->FloatAttribute(Lambda2LocalScaledTag)) / sum;
	    point->FloatAttribute(LinearityLocalTag) =
	      (point->FloatAttribute(Lambda0LocalScaledTag) - 
		   point->FloatAttribute(Lambda1LocalScaledTag)) /
	      (point->FloatAttribute(Lambda0LocalScaledTag) +
		   point->FloatAttribute(Lambda1LocalScaledTag));
      }
    }
    else {
      if (point->HasAttribute(Lambda0LocalTag) &&
	      point->HasAttribute(Lambda1LocalTag) &&
		  point->HasAttribute(Lambda2LocalTag)) {
	    sum = point->FloatAttribute(Lambda1LocalTag) +
		      point->FloatAttribute(Lambda2LocalTag);
	    if (sum == 0.0) point->FloatAttribute(FlatnessLocalTag) = 1.0;
	    else
	      point->FloatAttribute(FlatnessLocalTag)  = 
	        (point->FloatAttribute(Lambda1LocalTag) - 
			 point->FloatAttribute(Lambda2LocalTag)) / sum;
	    point->FloatAttribute(LinearityLocalTag) =
	      (point->FloatAttribute(Lambda0LocalTag) -
		   point->FloatAttribute(Lambda1LocalTag)) /
	      (point->FloatAttribute(Lambda0LocalTag) +
		   point->FloatAttribute(Lambda1LocalTag));
	  }
    }
} 
