
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
 Additional functions for class LaserPoints for postprocessing of filtering
 results

 void LaserPoints::FilterLargeObjects        Remove all points within large
   (double, TIN &,                           objects
    TINEdges &, double)
 DataBoundsLaser LaserPoints::ObjectBounds   Determine bounds of object
   (LaserPoint &)

 Functions used for debugging

 void WriteSelection(LaserPoints *, int)     Write all points with a certain
					     label
 void WriteContour(PointNumberList &, int)   Write topology of a contour

--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "LaserPoints.h"
#include "PointNumberLists.h"

/* Label names */

#define NOT_PROCESSED 0
#define PROCESSED 1
#define CURRENT_OBJECT 2
#define NB_OBJECT 3
#define TIN_BORDER 255

extern int Compare_Double(const void *, const void *);

/*
--------------------------------------------------------------------------------
                    Remove all points of large objects
--------------------------------------------------------------------------------
*/

void LaserPoints::FilterLargeObjects(double stdev, const TIN &tin,
                                     const TINEdges &edges,
                                     double min_object_height,
                                     double max_object_size)
{
  LaserPoints::iterator      object_point, node_point, nb_point;
  TIN::const_iterator        mesh;
  const MeshNumber           *nb_mesh;
  int                        index, nh_0_perc, nh_10_perc, nh_20_perc,
                             num_contours, min_size, high_contour_sizes,
                             print_size, i, j;
  PointNumberList            object;
  PointNumberLists::iterator high_object;
  PointNumberLists           high_objects;
  LineTopology               contour;
  LineTopologies::iterator   high_contour;
  LineTopologies             high_contours;
  PointNumberList::iterator  nb, node;
  const PointNumber          *node_ptr;
  TINEdgeSet                 neighbours;
  double                     low_0_perc, low_10_perc, low_20_perc,
                             perc0, perc10, perc20, object_height,
                             median_nb_height;
  vector<double>            filtered_heights, nb_heights;
  DataBoundsLaser           object_bounds;

/* Filter parameters */

//  min_perc = 20.0; // old parameter
  min_size = 2;
  print_size = 5;

/* Initialise point labels */

  Label(NOT_PROCESSED);

/* Set all points on the convex hull of the TIN to filtered and mark them. */

  for (mesh=tin.begin(); mesh!=tin.end(); mesh++) {
    for (i=0, nb_mesh=mesh->Neighbours(); i<3; i++, nb_mesh++) {
      if (nb_mesh->Number() == -1) { /* No neighbouring mesh, thus convex hull*/
        for (j=0, node_ptr=mesh->Nodes(); j<3; j++, node_ptr++) {
          if (i != j) { /* For i=0, filter points 1 and 2, etc. */
            node_point = begin() + node_ptr->Number();
            if (node_point->Label() != TIN_BORDER) {
              node_point->Label(TIN_BORDER);
              node_point->SetFiltered();
            }
          }
        }
      }
    }
  }

/* Process all points */

  num_contours = 0;
  printf("no  size low 0 low10 low20  per 0 per10 per20  av.h  nb.s status\n");
  for (object_point=begin(), index=0;
       object_point!=end();
       object_point++, index++) {

/* Filter points without neighbours. These are points that have been discarded
 * in the triangulation, since there are other points with the same X and Y
 * coordinates. (This is also done in the newer versions of FilterOnSlope and
 * FilterOnMorphology.)
 */

    if (edges[index].size() == 0) object_point->SetFiltered();

/* Find the next unfiltered point */

    if (!object_point->Filtered() && !object_point->Processed()) {

/* Determine the connected component of the selected unfiltered point and
 * mark all points with label 2.
 */
      object = Neighbourhood(PointNumber(index), edges, 1, 1, CURRENT_OBJECT);

/* Determine the object bounds */

      object_bounds = ObjectBounds(object);

/* Analyse object if the number of object points is at least min_size and the
 * size of the object is lower than max_object_size.
 */

      if (object_bounds.XRange() > max_object_size ||
          object_bounds.YRange() > max_object_size) {
        object_height = 0; /* Set to avoid elimination in the test, later on */
        median_nb_height = 1; /* Id. */
        printf("XX %5d, size %7.2f %7.2f kept.",
               object.size(), object_bounds.XRange(), object_bounds.YRange());
        if (object.size() < print_size) printf("\n");
      }
      else if (object.size() >= min_size) {

/* Determine the lowest, 10% lowest height, and 20% lowest height of the
 * adjacent filtered points.
 */
        filtered_heights.erase(filtered_heights.begin(),
                               filtered_heights.end());
        for (node=object.begin(); node!=object.end(); node++) {
          neighbours = edges[node->Number()];
          for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
    	    nb_point = begin() + nb->Number();
            if (nb_point->Filtered() && nb_point->Label() != TIN_BORDER)
              filtered_heights.push_back(nb_point->Z());
          }
        }
        qsort((void *) &*filtered_heights.begin(), filtered_heights.size(),
              sizeof(double), Compare_Double);
        if (filtered_heights.size() > 0) {
          low_0_perc  = filtered_heights[0];
          low_10_perc = filtered_heights[(int) ((filtered_heights.size())/10)];
          low_20_perc = filtered_heights[(int) ((filtered_heights.size())/5)];
        }
        else {
          low_0_perc = low_10_perc = low_20_perc = 1000000;
        }

/* Determine the number of points of the object that is higher than the lowest
 * filtered point minus 3 times the standard deviation.
 */

        nh_0_perc = nh_10_perc = nh_20_perc = 0;
        object_height = 0.0;
        for (node=object.begin(); node!=object.end(); node++) {
          node_point = begin() + node->Number();
          object_height += node_point->Z();
          if (node_point->Z() > low_0_perc - 3 * stdev) nh_0_perc++;
          if (node_point->Z() > low_10_perc - 3 * stdev) nh_10_perc++;
          if (node_point->Z() > low_20_perc - 3 * stdev) nh_20_perc++;
        }
        object_height /= object.size();
        perc0  = 100.0 * (double) nh_0_perc / (double) object.size();
        perc10 = 100.0 * (double) nh_10_perc / (double) object.size();
        perc20 = 100.0 * (double) nh_20_perc / (double) object.size();

/* Determine the contour of the object */

        num_contours++;
        contour = DeriveContour(num_contours, object, edges);
        contour.push_back(*(contour.begin()));

/* Find the contours of all components of rejected points adjacent to the
 * outside of the object. There can be multiple of those components if the
 * object touches the TIN border.
 */

        for (node=contour.begin(); node!=contour.end(); node++) {
          neighbours = edges[node->Number()];
          for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
            nb_point = begin() + nb->Number();
            if (nb_point->Filtered() &&           /* Point should be filtered */
                nb_point->Label() != NB_OBJECT && /* and not already done and */
                nb_point->Label() != TIN_BORDER) {  /* not a TIN border point */
              if (!nb_point->InsidePolygon(LaserPointsReference(), contour)) {
                high_objects.push_back(Neighbourhood(*nb, edges,1,1,NB_OBJECT));
                high_object = high_objects.end() - 1;
                high_contours.push_back(DeriveContour(1,
                            high_object->PointNumberListReference(), edges));
              }
            }
          }
        }

/* Determine the height differences along the contours of the rejected areas
 * within one step in the TIN. 
 */

        nb_heights.erase(nb_heights.begin(), nb_heights.end());
        high_contour_sizes = 0;
        for (high_contour=high_contours.begin(),
               high_object=high_objects.begin();
             high_contour!=high_contours.end();
             high_contour++, high_object++) {
          high_contour_sizes += high_contour->size();
          for (node=high_contour->begin();
               node!=high_contour->end();
               node++) {
            neighbours = edges[node->Number()];
            node_point = begin() + node->Number();
            if (!node_point->Filtered() || node_point->Label() != NB_OBJECT) {
              fprintf(stderr, "Inconsistent high point detected.\n");
              fprintf(stderr, "Filter %d, Label %d\n", node_point->Filtered(),
                      node_point->Label());
              exit(0);
            }
            for (nb=neighbours.begin(); nb!=neighbours.end(); nb++) {
  	      nb_point = begin() + nb->Number();
              switch (nb_point->Label()) {
                case CURRENT_OBJECT: /* Current object is to the inside of the*/
                  break;             /* rejected area and therefore ignored.  */

                case NB_OBJECT:      /* Neighbouring rejected point           */
                  break;

                case TIN_BORDER:     /* Point on TIN border                   */
                  break;

                default: /* Either PROCESSED or NOT_PROCESSED unfiltered point*/

                 /* Check if point is outside the contour of the rejected area*/
                  if (!nb_point->InsidePolygon(LaserPointsReference(), 
                                         high_contour->LineTopologyReference()))
                    nb_heights.push_back(nb_point->Z());
              }
            }
          }
        }
        qsort((void *) &*nb_heights.begin(), nb_heights.size(),
              sizeof(double), Compare_Double);
        if (nb_heights.size() > 0)
          median_nb_height = nb_heights[(int) ((nb_heights.size())/2)];
        else
          median_nb_height = object_height;

/* Output of statistics */

        if (object.size() >= print_size) {
        printf("%2d %5d %5.2f %5.2f %5.2f  %5.1f %5.1f %5.1f %5.2f %5.2f %5d\n",
                 num_contours, object.size(), low_0_perc,
                 low_10_perc, low_20_perc, perc0, perc10, perc20,
                 object_height, median_nb_height, filtered_heights.size());
          printf("%2d %5d %5d %5d ",
                 num_contours, contour.size(), high_objects.size(),
                 high_contour_sizes);
        }
      }

/* Old testing */

/* Filter the object if the percentage of higher points is larger than some
 * threshold or if the object has less than min_size points.
 * Also check that no point is significantly lower than the lowest filtered
 * point. */

/*
      if (object.size() > size() / 2 && object.size() >= min_size) {
        if (object.size() >= print_size) printf(" > 50\%");
      }
      else if ((perc10 > min_perc && perc0 > 99.9)|| object.size() < min_size) {
        for (node=object.begin(); node!=object.end(); node++) {
          node_point = begin() + node->Number();
          node_point->SetFiltered();
        }
        if (object.size() >= print_size) printf(" reject");
      }
*/

/* Delete an object if:
 * - the average object height exceeds the median height of neighbouring
 *   rejected points by at least min_object_height
 * - the size of the contours of the neighbouring object is larger than the
 *   size of the contour of the current object (i.e. neighbours should surround
 *   the current object), and
 * - all object points are higher than the lowest rejected point.
 * Always reject a point if its size is smaller than min_size.
 */

      if ((object_height > median_nb_height + min_object_height &&
           high_contour_sizes > contour.size() &&
           perc0 > 99.999) ||
          object.size() < min_size) {
        for (node=object.begin(); node!=object.end(); node++) {
           node_point = begin() + node->Number();
            node_point->SetFiltered();
        }
        if (object.size() >= print_size) printf(" reject");
      }
      if (object.size() >= print_size) printf("\n");

/* Set all labels of object points to PROCESSED and all labels of neighbouring
 * high components to NOT_PROCESSED.
 */

      for (node=object.begin(); node!=object.end(); node++) {
        node_point = begin() + node->Number();
        node_point->Label(PROCESSED);
      }
      for (high_object=high_objects.begin();
           high_object!=high_objects.end();
           high_object++) {
        for (node=high_object->begin(); node!=high_object->end(); node++) {
          node_point = begin() + node->Number();
          node_point->Label(PROCESSED);
        }
      }

/* Erase all high objects and their contours */

      for (high_object=high_objects.begin(),
           high_contour=high_contours.begin();
           high_object!=high_objects.end();
           high_object++, high_contour++) {
        high_object->erase(high_object->begin(), high_object->end());
        high_contour->erase(high_contour->begin(), high_contour->end());
      }
      high_objects.erase(high_objects.begin(), high_objects.end());
      high_contours.erase(high_contours.begin(), high_contours.end());
    }
  }

/* To force output now. */

  fflush(stdout);
}

/*
--------------------------------------------------------------------------------
                       Determine object the bounds
--------------------------------------------------------------------------------
*/

DataBoundsLaser LaserPoints::ObjectBounds(const PointNumberList &object) const
{
  LaserPoints::const_iterator     point;
  PointNumberList::const_iterator node;
  DataBoundsLaser                 bounds;

  for (node=object.begin(); node!=object.end(); node++) {
    point = begin() + node->Number();
    bounds.Update(point->LaserPointRef());
  }
  return(bounds);
}

/*
--------------------------------------------------------------------------------
	                  Debugging functions
--------------------------------------------------------------------------------
*/
void WriteSelection(LaserPoints *pts, int label)
{
  LaserPoints           selection;
  LaserPoints::iterator pt;
  char                  filename[20];

  for (pt=pts->begin(); pt!=pts->end(); pt++)
    if (pt->Label() == label) selection.push_back(*pt);
  sprintf(filename, "label%d.laser.gz", label);
  selection.Write(filename, 1);
}

void WriteContour(PointNumberList &list, int number)
{
  PointNumberList::iterator node;
  FILE                      *fd;
  int                       i;
  char                      filename[80];

  sprintf(filename, "contour_%d.top", number);
  fd = fopen(filename, "w");
  fprintf(fd,"711 # Topology file\n");
  fprintf(fd,"%3d %3d 1\n", number, list.size());
  for (i=1, node=list.begin();
       node!=list.end();
       i++, node++) {
    fprintf(fd, "%6d", node->Number());
    if (i==6) {
      fprintf(fd, "\n");
      i = 0;
    }
  }
  if (i != 1) fprintf(fd, "\n");
  fclose(fd);
}
