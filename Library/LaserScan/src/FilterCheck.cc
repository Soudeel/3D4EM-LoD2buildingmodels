
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

 void LaserPoints::FilterCheck               Check filter results
   (double, Image &, double, double,
    double, TINEdges & )

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
                               Check filter results
--------------------------------------------------------------------------------
*/

void LaserPoints::FilterCheck(double max_building_size, const Image &kernel,
                              double range, double stdev, double tolerance,
                              const TINEdges &edges)
{
  LaserPoints::iterator     object_point, node_point, nb_point;
  TIN::iterator             mesh;
  MeshNumber                *nb_mesh;
  int                       index, i, j;
  PointNumberList           object;
  PointNumberList::iterator nb_it, node_it;
  PointNumber               *node_ptr;
  TINEdgeSet                neighbours;
  std::vector<double>       object_heights, nb_heights, height_differences;
  DataBoundsLaser           object_bounds;
  double                    obj_0, obj_25, obj_50, obj_75, obj_100,
                            nb_50, nb_80, nb_90, nb_95, nb_100;

/* Initialise point labels */

  Label(NOT_PROCESSED);

/* Set all points on the convex hull of the TIN to filtered and mark them. */

  for (mesh=tin->begin(); mesh!=tin->end(); mesh++) {
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

  printf(" o  # pts Xrange  Yrange    0     25     50     75    100\n");
  printf(" n  # pts Xrange  Yrange   50     80     90     95    100\n");
  for (object_point=begin(), index=0;
       object_point!=end();
       object_point++, index++) {

/* Find the next filtered point */

    if (object_point->Filtered() && !object_point->Processed()) {

/* Determine the connected component of the selected filtered point and
 * mark all points with label CURRENT_OBJECT.
 */
      object = Neighbourhood(PointNumber(index), edges, 1, 1, CURRENT_OBJECT);

/* Determine the object bounds */

      object_bounds = ObjectBounds(object);

/* Analyse object if the size of the object is greater than max_building_size.
 */

      if (object_bounds.XRange() > max_building_size ||
          object_bounds.YRange() > max_building_size) {

/* Make a statistic of the height differences with the adjacent filtered points.
 */
        object_heights.erase(object_heights.begin(), object_heights.end());
        nb_heights.erase(nb_heights.begin(), nb_heights.end());
        height_differences.erase(height_differences.begin(),
                                 height_differences.end());
        for (node_it=object.begin(); node_it!=object.end(); node_it++) {
          node_point = begin() + node_it->Number();
          object_heights.push_back(node_point->Z());
          neighbours = edges[node_it->Number()];
          for (nb_it=neighbours.begin(); nb_it!=neighbours.end(); nb_it++) {
    	    nb_point = begin() + nb_it->Number();
            if (!nb_point->Filtered() && nb_point->Label() != TIN_BORDER) {
              nb_heights.push_back(nb_point->Z());
              height_differences.push_back(nb_point->Z() - node_point->Z());
            }
          }
        }
        qsort((void *) &*object_heights.begin(), object_heights.size(),
              sizeof(double), Compare_Double);
        qsort((void *) &*nb_heights.begin(), nb_heights.size(),
              sizeof(double), Compare_Double);
        qsort((void *) &*height_differences.begin(), height_differences.size(),
              sizeof(double), Compare_Double);
        obj_0   = object_heights[0];
        obj_25  = object_heights[object_heights.size()/4];
        obj_50  = object_heights[object_heights.size()/2];
        obj_75  = object_heights[(3*object_heights.size())/4];
        obj_100 = object_heights[object_heights.size()-1];
        nb_50  = nb_heights[(50*nb_heights.size())/100];
        nb_80  = nb_heights[(80*nb_heights.size())/100];
        nb_90  = nb_heights[(90*nb_heights.size())/100];
        nb_95  = nb_heights[(95*nb_heights.size())/100];
        nb_100 = nb_heights[nb_heights.size()-1];

/* Output of statistics */

        printf(" o %5d %7.2f %7.2f %6.2f %6.2f %6.2f %6.2f %6.2f\n",
               object.size(), object_bounds.XRange(), object_bounds.YRange(),
               obj_0, obj_25, obj_50, obj_75, obj_100);
        printf(" n %5d %7.2f %7.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
               nb_heights.size(),
               object_bounds.XRange(), object_bounds.YRange(),
               nb_50, nb_80, nb_90, nb_95, nb_100);

/* Set the object to unfiltered if the average object height is clearly below
 * the 90% height value of the contour.
 */

        if (obj_50 < nb_90 - 2.0) {
          for (node_it=object.begin(); node_it!=object.end(); node_it++) {
            node_point = begin() + node_it->Number();
            node_point->SetUnFiltered();
          }
          printf(" *\n");
        }
        else printf("\n");
      }

/* Set all labels of object points to PROCESSED */

      for (node_it=object.begin(); node_it!=object.end(); node_it++) {
        node_point = begin() + node_it->Number();
        node_point->Label(PROCESSED);
      }
    }
  }

/* To force output now. */

  fflush(stdout);
}

