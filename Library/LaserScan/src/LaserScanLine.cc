
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
 Collection of functions for class LaserScanLine

 LaserPoints::iterator LaserScanLine::begin        Return the first point
   (const LaserPoints &)
 LaserPoints::iterator LaserScanLine::end          Return the last point
   (const LaserPoints &)
 LaserPoints::iterator LaserScanLine::begin        Return the first point
   (const LaserPoints *, int)
 LaserPoints::iterator LaserScanLine::end          Return the last point
   (const LaserPoints *, int)
 void LaserScanLine::Print() const                 Print scan line to screen

 Initial creation
 Author : George Vosselman
 Date   : 15-07-1999

 Update #1
 Author : 
 Date   : 
 Changes: 

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "LaserScanLine.h"
#include "Line2D.h"
#include "LSAdjust.h"
// #include "vRecord.cc"

#include "Adjustment.h"
extern "C" void dgeco_(double *, int *, int *, int *, double *, double *);
extern "C" void dgedi_(double *, int *, int *, int *, double *, double *,
                       int *);


/*
--------------------------------------------------------------------------------
                   Construct from LaserPoints iterators
--------------------------------------------------------------------------------
*/

LaserScanLine::LaserScanLine(const LaserPoints &points,
	                         LaserPoints::const_iterator b,
				             LaserPoints::const_iterator e)
{
  SetBeginAndEnd(points, b, e);
}

/*
--------------------------------------------------------------------------------
                   Set begin and end of scan line
--------------------------------------------------------------------------------
*/

void LaserScanLine::SetBeginAndEnd(const LaserPoints &points,
	                               LaserPoints::const_iterator b,
				                   LaserPoints::const_iterator e)
{
  beginning = PointNumber(std::distance(points.begin(), b));
  ending    = PointNumber(std::distance(points.begin(), e));
}

/*
--------------------------------------------------------------------------------
                   Return first or last point of scan line
--------------------------------------------------------------------------------
*/

// Based on a reference to a point set
LaserPoints::const_iterator LaserScanLine::begin(const LaserPoints &points,
                                                 int first_number) const
{
  return(points.begin() + beginning.Number() - first_number);
}

LaserPoints::const_iterator LaserScanLine::end(const LaserPoints &points,
                                               int first_number) const
{
  return(points.begin() + ending.Number() - first_number);
}

LaserPoints::iterator LaserScanLine::begin(LaserPoints &points,
                                           int first_number) const
{
  return(points.begin() + beginning.Number() - first_number);
}

LaserPoints::iterator LaserScanLine::end(LaserPoints &points,
                                         int first_number) const
{
  return(points.begin() + ending.Number() - first_number);
}

// Based on a pointer to a point set
LaserPoints::const_iterator LaserScanLine::begin(const LaserPoints *points,
                                                 int first_number) const
{
  return(points->begin() + beginning.Number() - first_number);
}

LaserPoints::const_iterator LaserScanLine::end(const LaserPoints *points,
                                               int first_number) const
{
  return(points->begin() + ending.Number() - first_number);
}

// Based on an iterator to the start of a point set
LaserPoints::const_iterator LaserScanLine::begin(LaserPoints::const_iterator first_point_of_set,
                                                 int first_number) const
{
  return(first_point_of_set + beginning.Number() - first_number);
}

LaserPoints::const_iterator LaserScanLine::end(LaserPoints::const_iterator first_point_of_set,
                                               int first_number) const
{
  return(first_point_of_set + ending.Number() - first_number);
}

LaserPoints::iterator LaserScanLine::begin(LaserPoints::iterator first_point_of_set,
                                           int first_number) const
{
  return(first_point_of_set + beginning.Number() - first_number);
}

LaserPoints::iterator LaserScanLine::end(LaserPoints::iterator first_point_of_set,
                                         int first_number) const
{
  return(first_point_of_set + ending.Number() - first_number);
}
/*
--------------------------------------------------------------------------------
                         Scan line length
--------------------------------------------------------------------------------
*/

double LaserScanLine::Length(const LaserPoints &points) const
{
  return (end(points)->vect() - begin(points)->vect()).Length();
}

double LaserScanLine::Length(LaserPoints::const_iterator first_point_of_set) const
{
  return (end(first_point_of_set)->vect() -
          begin(first_point_of_set)->vect()).Length();
}

/*
--------------------------------------------------------------------------------
                         Print scan line to screen
--------------------------------------------------------------------------------
*/

void LaserScanLine::Print() const
{
  printf("%8d%8d  (%d)\n", beginning.Number(), ending.Number(),
         ending.Number() - beginning.Number() + 1);
}

/*
--------------------------------------------------------------------------------
                      Reconstruction of the scanner position
--------------------------------------------------------------------------------
*/

bool LaserScanLine::ScannerPosition(const LaserPoints &points, double height,
                                    Position3D &scanner_pos, int method,
                                    double fixed_angle_increment) const
{
  LaserPoints::const_iterator first_point, last_point, point1, point2;
  Vector2D                    flight_dir;
  Line2D                      flight_line;
  LSAdjust                    adjustment;
  vRecord                     row_A, sol;
  double                      r1, r2, u_s, z_s, angle_s, obs, sigma, sigma_old,                               angle;
  int                         iter;

  vMatrix                     Qxx;
  double                      sqobs;

  first_point = begin(points);
  last_point  = end(points);
  
  switch (method) {
    case 0: // Middle of both end points with user-specified height
      scanner_pos = (first_point->vect() + last_point->vect()) / 2.0;
      scanner_pos.Z() = height;
      break;

    case 1: // Estimate based on unknown constant angle increment
      row_A = vRecord(3, 0.0);
    case 2: // Estimate based on fixed user-specified angle increment
      ScannerPosition(points, height, scanner_pos, 0);// As first approximation
      if (method == 2) {
        if (fixed_angle_increment <= 0.0) {
          fprintf(stderr, "Error: angle increment should be positive!\n");
          return false;
        }
        row_A = vRecord(2, 0.0);
      }
      if (NumberOfPoints() < 4) return false;
      flight_dir = (first_point->vect2D() - last_point->vect2D()).Normal();
      flight_line = Line2D(Position2D(scanner_pos.vect2D()),
                           Position2D(scanner_pos.vect2D() + flight_dir));
      adjustment.SetEPS(1.0e-20); // Allow relatively bad condition of ATA 
      sigma = 1e10;
      // Approximate values of the unknowns
      u_s = 0.0;
      z_s = height;
      if (method == 1) angle_s = ScanAngleIncrement(points, scanner_pos, 20.0);
      else angle_s = fixed_angle_increment;
      iter = 0;
      do { // Start of the interations
        iter++; adjustment.FullReset();
        sigma_old = sigma; sqobs=0.0;
        for (point1=first_point, point2=point1+1;
             point1!=last_point; point1++, point2++) {
          angle = Angle(point1->vect() - scanner_pos.vect(),
                        point2->vect() - scanner_pos.vect());
          if (angle > 0.5 * angle_s && angle < 1.5 * angle_s) {
            // Valid observation
            r1 =
              (flight_line.DistanceToPointSigned(Position2D(point1->vect2D())) -
               u_s) / (z_s - point1->Z());
            r2 = 
              (flight_line.DistanceToPointSigned(Position2D(point2->vect2D())) -
               u_s) / (z_s - point2->Z());
            obs = atan(r2) - atan(r1) - angle_s;
            sqobs += obs * obs;
            // Derivative to u (perpendicular to flight line)
            row_A[0] = 1.0 / ((1.0 + r1 * r1) * (point1->Z() - z_s)) -
                       1.0 / ((1.0 + r2 * r2) * (point2->Z() - z_s));
            // Derivative to z
            row_A[1] = r1 / ((1.0 + r1 * r1) * (point1->Z() - z_s)) -
                       r2 / ((1.0 + r2 * r2) * (point2->Z() - z_s));
            // Derivative to angle increment (only for method 1)
            if (method == 1) row_A[2] = 1.0;
            adjustment.AddRecord(row_A, obs, 1.0);
//          adjustment.Update_Normal_Equations(row_A, obs, 1.0);
          }
        }
        // Three observations is the absolute minimum, quite likely we need more
        if (adjustment.nObs() < 3) return false;
        if (!adjustment.Adjust(true)) return false; // Adjustment failed
        // Update of the unknowns
        sol = adjustment.Solution();
        u_s     += sol[0];
        z_s     += sol[1];
        if (method == 1) angle_s += sol[2];
        sigma = adjustment.Sigma();
      // Iterate at least 3 times. Stop if the relative improvement is less
      // than 0.0001 or if there is no convergence after 10 iterations.
      } while ((iter < 10 && sigma / sigma_old < 0.9999) || iter < 3);
      // Computation of the estimated scanner position
      scanner_pos.X() += u_s * flight_line.Getcosphi();
      scanner_pos.Y() += u_s * flight_line.Getsinphi();
      scanner_pos.Z() = z_s;
/* Test output
      printf("Sigma %6.3f (%6.3f)\nQxx\n", sigma * 45 / atan(1.0),
             sqrt(sqobs/(adjustment.nObs()-3)) * 45 / atan(1.0));
      sigma = sqrt(sqobs/(adjustment.nObs()-3));
      Qxx = adjustment.N1();
      Qxx *= sigma * sigma;
      Qxx.Print("%12.4e");
      if (method == 1)
        printf("u %6.2f (%8.4f)  std_z %6.2f (%8.4f)  std_a %8.4f (%8.4f)\n",
               u_s, sqrt(Qxx.Val(0,0)), z_s, sqrt(Qxx.Val(1,1)),
               angle_s * 45 / atan(1.0), sqrt(Qxx.Val(2,2)) * 45 / atan(1.0));
      else // method 2
        printf("u %6.2f (%8.4f)  std_z %6.2f (%8.4f)\n",
               u_s, sqrt(Qxx.Val(0,0)), z_s, sqrt(Qxx.Val(1,1)));
      printf("Correlation\n");
      adjustment.Correlation().Print("%8.4f");
      exit(0);
*/
      break;
  }
  return true;
}

/*
--------------------------------------------------------------------------------
                      Reconstruction of the scanner position
--------------------------------------------------------------------------------
*/

double LaserScanLine::ScanAngleIncrement(const LaserPoints &points,
                                         const Position3D &scanner_pos,
                                         double percentage) const
{
  vRecord          angles;
  std::vector<int> indices;
  int              index;
  LaserPoints::const_iterator point1, point2;

  if (NumberOfPoints() < 2) return 0.0;
  point1 = begin(points);
  for (point2 = point1 + 1; point1 != end(points); point2++) {
    angles.push_back(Angle(point1->vect() - scanner_pos.vect(),
                           point2->vect() - scanner_pos.vect()));
    point1 = point2;
  }
  angles = angles.Sort(indices);
  index = (int) (angles.size() * percentage / 100.0);
  if (index < 0) index = 0;
  if (index >= angles.size()) index = angles.size() - 1;
  return (angles[index]);
}
