
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
 Collection of functions for class DataBounds2D

 DataBounds2D DataBounds2D::operator =            Copy assignment
   (const DataBounds2D &)
 void DataBounds2D::Initialise()                  Initialise set switches
 void DataBounds2D::SetMinimumX(double)           Set minimum X coordinate
 void DataBounds2D::SetMinimumY(double)           Set minimum Y coordinate
 void DataBounds2D::SetMaximumX(double)           Set maximum X coordinate
 void DataBounds2D::SetMaximumY(double)           Set maximum Y coordinate
 int DataBounds2D::MinimumXIsSet() const          Minimum X coordinate switch
 int DataBounds2D::MinimumYIsSet() const          Minimum Y coordinate switch
 int DataBounds2D::MaximumXIsSet() const          Maximum X coordinate switch
 int DataBounds2D::MaximumYIsSet() const          Maximum Y coordinate switch
 int DataBounds2D::XYBoundsSet() const            True if XY bounds are set

 double DataBounds2D::XRange() const              X coordinate range
 double DataBounds2D::YRange() const              Y coordinate range
 DataBounds2D DataBounds2D::Update(DataBounds2D)  Update bounds
 DataBounds2D DataBounds2D::Update(Position2D &)  Update bounds
 int DataBounds2D::Inside(Position2D &)           True if within all bounds
 friend int Overlap(DataBounds2D &,               Check if two ranges overlap
                    DataBounds2D &)

 Initial creation
 Author : George Vosselman
 Date   : 17-02-2001

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
#include <string.h>
#include <strings.h>
#include "DataBounds2D.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                           Declaration of C functions
--------------------------------------------------------------------------------
*/


/*
--------------------------------------------------------------------------------
                         Copy assignment
--------------------------------------------------------------------------------
*/

DataBounds2D DataBounds2D::operator = (const DataBounds2D &b)
{
  // Check for self assignment
  if (this == &b) return *this;
  minimum = b.minimum;
  maximum = b.maximum;
  for (int i=0; i<2; i++) {
    min_is_set[i] = b.min_is_set[i];
    max_is_set[i] = b.max_is_set[i];
  }
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise bound setting switches
--------------------------------------------------------------------------------
*/
void DataBounds2D::Initialise()
{
  for (int i=0; i<2; i++) {
    min_is_set[i] = max_is_set[i] = 0;
  }
}

/*
--------------------------------------------------------------------------------
                         Set minimum or maximum value
--------------------------------------------------------------------------------
*/

void DataBounds2D::SetMinimumX(double x)
  { minimum.X() = x;  min_is_set[0] = 1; }
void DataBounds2D::SetMinimumY(double y)
  { minimum.Y() = y;  min_is_set[1] = 1; }
void DataBounds2D::SetMaximumX(double x)
  { maximum.X() = x;  max_is_set[0] = 1; }
void DataBounds2D::SetMaximumY(double y)
  { maximum.Y() = y;  max_is_set[1] = 1; }

/*
--------------------------------------------------------------------------------
                         Inquire after bound setting
--------------------------------------------------------------------------------
*/

int DataBounds2D::MinimumXIsSet() const {return(min_is_set[0]);}
int DataBounds2D::MinimumYIsSet() const {return(min_is_set[1]);}
int DataBounds2D::MaximumXIsSet() const {return(max_is_set[0]);}
int DataBounds2D::MaximumYIsSet() const {return(max_is_set[1]);}

int DataBounds2D::XYBoundsSet() const
{
  return(MinimumXIsSet() && MaximumXIsSet() &&
         MinimumYIsSet() && MaximumYIsSet());
}

int DataBounds2D::HasSomeData() const
{
  return(MinimumXIsSet() || MaximumXIsSet() ||
         MinimumYIsSet() || MaximumYIsSet());
}


/*
--------------------------------------------------------------------------------
                        Return the range of a coordinate
--------------------------------------------------------------------------------
*/

double DataBounds2D::XRange() const
{
  if (MinimumXIsSet() && MaximumXIsSet()) return(maximum.X() - minimum.X());
  else return(0.0);
}

double DataBounds2D::YRange() const
{
  if (MinimumYIsSet() && MaximumYIsSet()) return(maximum.Y() - minimum.Y());
  else return(0.0);
}

/*
--------------------------------------------------------------------------------
                            Update the bounds
--------------------------------------------------------------------------------
*/

DataBounds2D DataBounds2D::Update(const DataBounds2D &b)
{
  Position2D bmin, bmax;

  bmin = b.Minimum();
  bmax = b.Maximum();

  if (b.MinimumXIsSet()) {
    if (MinimumXIsSet()) SetMinimumX(MIN(minimum.X(), bmin.X()));
    else SetMinimumX(bmin.X());
  }
  if (b.MaximumXIsSet()) {
    if (MaximumXIsSet()) SetMaximumX(MAX(maximum.X(), bmax.X()));
    else SetMaximumX(bmax.X());
  }
  if (b.MinimumYIsSet()) {
    if (MinimumYIsSet()) SetMinimumY(MIN(minimum.Y(), bmin.Y()));
    else SetMinimumY(bmin.Y());
  }
  if (b.MaximumYIsSet()) {
    if (MaximumYIsSet()) SetMaximumY(MAX(maximum.Y(), bmax.Y()));
    else SetMaximumY(bmax.Y());
  }

  return(*this);
}

DataBounds2D DataBounds2D::Update(const Position2D &point)
{
  if (MinimumXIsSet()) SetMinimumX(MIN(minimum.X(), point.X()));
  else SetMinimumX(point.X());
  if (MaximumXIsSet()) SetMaximumX(MAX(maximum.X(), point.X()));
  else SetMaximumX(point.X());

  if (MinimumYIsSet()) SetMinimumY(MIN(minimum.Y(), point.Y()));
  else SetMinimumY(point.Y());
  if (MaximumYIsSet()) SetMaximumY(MAX(maximum.Y(), point.Y()));
  else SetMaximumY(point.Y());

  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Check if a 2D point is within the bounds
--------------------------------------------------------------------------------
*/

int DataBounds2D::Inside(const Position2D &point) const
{
  if (MinimumXIsSet() && point.X() < minimum.X()) return(0);
  if (MaximumXIsSet() && point.X() >= maximum.X()) return(0);
  if (MinimumYIsSet() && point.Y() < minimum.Y()) return(0);
  if (MaximumYIsSet() && point.Y() >= maximum.Y()) return(0);
  return(1);
}

/*
--------------------------------------------------------------------------------
                     Check if a 3D point is within the bounds
--------------------------------------------------------------------------------
*/

int DataBounds2D::Inside(const Position3D &point) const
{
  if (MinimumXIsSet() && point.X() < minimum.X()) return(0);
  if (MaximumXIsSet() && point.X() >= maximum.X()) return(0);
  if (MinimumYIsSet() && point.Y() < minimum.Y()) return(0);
  if (MaximumYIsSet() && point.Y() >= maximum.Y()) return(0);
  return(1);
}

/*
--------------------------------------------------------------------------------
                     Return the centre of the bounding box
--------------------------------------------------------------------------------
*/

Position2D DataBounds2D::MidPoint() const
{
  return Position2D((minimum.X() + maximum.X()) / 2.0,
                    (minimum.Y() + maximum.Y()) / 2.0);
}

/*
--------------------------------------------------------------------------------
                Check if two data bounds overlap in planimetry
--------------------------------------------------------------------------------
*/

int DataBounds2D::Overlap(const DataBounds2D &bounds2) const
{
  if (MinimumXIsSet() && bounds2.MaximumXIsSet() &&
      Minimum().X() > bounds2.Maximum().X()) return(0);
  if (MaximumXIsSet() && bounds2.MinimumXIsSet() &&
      Maximum().X() < bounds2.Minimum().X()) return(0);
  if (MinimumYIsSet() && bounds2.MaximumYIsSet() &&
      Minimum().Y() > bounds2.Maximum().Y()) return(0);
  if (MaximumYIsSet() && bounds2.MinimumYIsSet() &&
      Maximum().Y() < bounds2.Minimum().Y()) return(0);
  return(1);
}

int Overlap(const DataBounds2D &bounds1, const DataBounds2D &bounds2)
{
  if (bounds1.MinimumXIsSet() && bounds2.MaximumXIsSet() &&
      bounds1.Minimum().X() > bounds2.Maximum().X()) return(0);
  if (bounds1.MaximumXIsSet() && bounds2.MinimumXIsSet() &&
      bounds1.Maximum().X() < bounds2.Minimum().X()) return(0);
  if (bounds1.MinimumYIsSet() && bounds2.MaximumYIsSet() &&
      bounds1.Minimum().Y() > bounds2.Maximum().Y()) return(0);
  if (bounds1.MaximumYIsSet() && bounds2.MinimumYIsSet() &&
      bounds1.Maximum().Y() < bounds2.Minimum().Y()) return(0);
  return(1);
}
