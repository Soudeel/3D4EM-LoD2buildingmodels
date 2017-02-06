
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
 Collection of functions for class DataBounds3D

 DataBounds3D DataBounds3D::operator =            Copy assignment
   (const DataBounds3D &)
 void DataBounds3D::Initialise()                  Initialise set switches
 void DataBounds3D::SetMinimumX(double)           Set minimum X coordinate
 void DataBounds3D::SetMinimumY(double)           Set minimum Y coordinate
 void DataBounds3D::SetMinimumZ(double)           Set minimum Z coordinate
 void DataBounds3D::SetMaximumX(double)           Set maximum X coordinate
 void DataBounds3D::SetMaximumY(double)           Set maximum Y coordinate
 void DataBounds3D::SetMaximumZ(double)           Set maximum Z coordinate
 int DataBounds3D::MinimumXIsSet() const          Minimum X coordinate switch
 int DataBounds3D::MinimumYIsSet() const          Minimum Y coordinate switch
 int DataBounds3D::MinimumZIsSet() const          Minimum Z coordinate switch
 int DataBounds3D::MaximumXIsSet() const          Maximum X coordinate switch
 int DataBounds3D::MaximumYIsSet() const          Maximum Y coordinate switch
 int DataBounds3D::MaximumZIsSet() const          Maximum Z coordinate switch
 int DataBounds3D::XYBoundsSet() const            True if XY bounds are set
 int DataBounds3D::XYZBoundsSet() const           True if XYZ bounds are set

 double DataBounds3D::XRange() const              X coordinate range
 double DataBounds3D::YRange() const              Y coordinate range
 double DataBounds3D::ZRange() const              Z coordinate range
 DataBounds3D DataBounds3D::Update(DataBounds3D)  Update bounds
 DataBounds3D DataBounds3D::Update(Position3D &)  Update bounds
 int DataBounds3D::InsideXY(Position3D &)         True if within XY bounds
 int DataBounds3D::Inside(Position3D &)           True if within all bounds
 friend int Overlap(DataBounds3D &,               Check if two ranges overlap
                    DataBounds3D &)              
 friend int OverlapXY(DataBounds3D &,             Check if two ranges overlap
                    DataBounds3D &)               in planimetry

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
#include "DataBounds3D.h"
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

DataBounds3D DataBounds3D::operator = (const DataBounds3D &b)
{
  // Check for self assignment
  if (this == &b) return *this;
  minimum = b.minimum;
  maximum = b.maximum;
  for (int i=0; i<3; i++) {
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
void DataBounds3D::Initialise()
{
  for (int i=0; i<3; i++) {
    min_is_set[i] = max_is_set[i] = 0;
  }
}

/*
--------------------------------------------------------------------------------
                         Set minimum or maximum value
--------------------------------------------------------------------------------
*/

void DataBounds3D::SetMinimumX(double x)
  { minimum.SetX(x);  min_is_set[0] = 1; }
void DataBounds3D::SetMinimumY(double y)
  { minimum.SetY(y);  min_is_set[1] = 1; }
void DataBounds3D::SetMinimumZ(double z)
  { minimum.SetZ(z);  min_is_set[2] = 1; }
void DataBounds3D::SetMaximumX(double x)
  { maximum.SetX(x);  max_is_set[0] = 1; }
void DataBounds3D::SetMaximumY(double y)
  { maximum.SetY(y);  max_is_set[1] = 1; }
void DataBounds3D::SetMaximumZ(double z)
  { maximum.SetZ(z);  max_is_set[2] = 1; }


/*
--------------------------------------------------------------------------------
                         Inquire after bound setting
--------------------------------------------------------------------------------
*/

int DataBounds3D::MinimumXIsSet() const {return(min_is_set[0]);}
int DataBounds3D::MinimumYIsSet() const {return(min_is_set[1]);}
int DataBounds3D::MinimumZIsSet() const {return(min_is_set[2]);}
int DataBounds3D::MaximumXIsSet() const {return(max_is_set[0]);}
int DataBounds3D::MaximumYIsSet() const {return(max_is_set[1]);}
int DataBounds3D::MaximumZIsSet() const {return(max_is_set[2]);}

int DataBounds3D::XYBoundsSet() const
{
  return(MinimumXIsSet() && MaximumXIsSet() &&
         MinimumYIsSet() && MaximumYIsSet());
}

int DataBounds3D::XYZBoundsSet() const
{
  return(MinimumXIsSet() && MaximumXIsSet() &&
         MinimumYIsSet() && MaximumYIsSet() &&
         MinimumZIsSet() && MaximumZIsSet());
}

int DataBounds3D::HasSomeData() const
{
  return(MinimumXIsSet() || MaximumXIsSet() ||
         MinimumYIsSet() || MaximumYIsSet() ||
         MinimumZIsSet() || MaximumZIsSet());
}


/*
--------------------------------------------------------------------------------
                        Return the range of a coordinate
--------------------------------------------------------------------------------
*/

double DataBounds3D::XRange() const
{
  if (MinimumXIsSet() && MaximumXIsSet()) return(maximum.X() - minimum.X());
  else return(0.0);
}

double DataBounds3D::YRange() const
{
  if (MinimumYIsSet() && MaximumYIsSet()) return(maximum.Y() - minimum.Y());
  else return(0.0);
}

double DataBounds3D::ZRange() const
{
  if (MinimumZIsSet() && MaximumZIsSet()) return(maximum.Z() - minimum.Z());
  else return(0.0);
}

/*
--------------------------------------------------------------------------------
                            Update the bounds
--------------------------------------------------------------------------------
*/

DataBounds3D DataBounds3D::Update(const DataBounds3D &b)
{
  Position3D bmin, bmax;

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
  if (b.MinimumZIsSet()) {
    if (MinimumZIsSet()) SetMinimumZ(MIN(minimum.Z(), bmin.Z()));
    else SetMinimumZ(bmin.Z());
  }
  if (b.MaximumZIsSet()) {
    if (MaximumZIsSet()) SetMaximumZ(MAX(maximum.Z(), bmax.Z()));
    else SetMaximumZ(bmax.Z());
  }

  return(*this);
}

DataBounds3D DataBounds3D::Update(const Position3D &point)
{
  if (MinimumXIsSet()) SetMinimumX(MIN(minimum.X(), point.X()));
  else SetMinimumX(point.X());
  if (MaximumXIsSet()) SetMaximumX(MAX(maximum.X(), point.X()));
  else SetMaximumX(point.X());

  if (MinimumYIsSet()) SetMinimumY(MIN(minimum.Y(), point.Y()));
  else SetMinimumY(point.Y());
  if (MaximumYIsSet()) SetMaximumY(MAX(maximum.Y(), point.Y()));
  else SetMaximumY(point.Y());

  if (MinimumZIsSet()) SetMinimumZ(MIN(minimum.Z(), point.Z()));
  else SetMinimumZ(point.Z());
  if (MaximumZIsSet()) SetMaximumZ(MAX(maximum.Z(), point.Z()));
  else SetMaximumZ(point.Z());

  return(*this);
}

DataBounds3D DataBounds3D::Update(const ObjectPoints &points)
{
  ObjectPoints::const_iterator point;
  
  for (point=points.begin(); point!=points.end(); point++)
    Update(point->Position3DRef());
  return(*this);
}
    
/*
--------------------------------------------------------------------------------
        Check if a point is within the bounds (relaxed with a margin)
--------------------------------------------------------------------------------
*/

bool DataBounds3D::InsideXY(const Position3D &point) const
{
  if (MinimumXIsSet() && point.X() < minimum.X()) return(false);
  if (MaximumXIsSet() && point.X() >= maximum.X()) return(false);
  if (MinimumYIsSet() && point.Y() < minimum.Y()) return(false);
  if (MaximumYIsSet() && point.Y() >= maximum.Y()) return(false);
  return(true);
}

bool DataBounds3D::InsideXY(const Position3D &point, double margin) const
{
  if (MinimumXIsSet() && point.X() < minimum.X() - margin) return(false);
  if (MaximumXIsSet() && point.X() >= maximum.X() + margin) return(false);
  if (MinimumYIsSet() && point.Y() < minimum.Y() - margin) return(false);
  if (MaximumYIsSet() && point.Y() >= maximum.Y() + margin) return(false);
  return(true);
}

bool DataBounds3D::Inside(const Position3D &point) const
{
  if (!InsideXY(point)) return(false);
  if (MinimumZIsSet() && point.Z() < minimum.Z()) return(false);
  if (MaximumZIsSet() && point.Z() >= maximum.Z()) return(false);
  return(true);
}

bool DataBounds3D::Inside(const Position3D &point, double margin) const
{
  if (!InsideXY(point, margin)) return(false);
  if (MinimumZIsSet() && point.Z() < minimum.Z() - margin) return(false);
  if (MaximumZIsSet() && point.Z() >= maximum.Z() + margin) return(false);
  return(true);
}

/*
--------------------------------------------------------------------------------
                     Return the centre of the bounding box
--------------------------------------------------------------------------------
*/

Position3D DataBounds3D::MidPoint() const
{
  return Position3D((minimum.X() + maximum.X()) / 2.0,
                    (minimum.Y() + maximum.Y()) / 2.0,
                    (minimum.Z() + maximum.Z()) / 2.0);
}

/*
--------------------------------------------------------------------------------
                Check if two data bounds overlap
--------------------------------------------------------------------------------
*/

bool Overlap(const DataBounds3D &bounds1, const DataBounds3D &bounds2)
{
  // First check planimetric overlap
  if (!OverlapXY(bounds1, bounds2)) return false;
  // Next, verify the height overlap
  if (bounds1.MinimumZIsSet() && bounds2.MaximumZIsSet() &&
      bounds1.Minimum().Z() > bounds2.Maximum().Z()) return(false);
  if (bounds1.MaximumZIsSet() && bounds2.MinimumZIsSet() &&
      bounds1.Maximum().Z() < bounds2.Minimum().Z()) return(false);
  return true;
}

bool OverlapXY(const DataBounds3D &bounds1, const DataBounds3D &bounds2)
{
  if (bounds1.MinimumXIsSet() && bounds2.MaximumXIsSet() &&
      bounds1.Minimum().X() > bounds2.Maximum().X()) return(false);
  if (bounds1.MaximumXIsSet() && bounds2.MinimumXIsSet() &&
      bounds1.Maximum().X() < bounds2.Minimum().X()) return(false);
  if (bounds1.MinimumYIsSet() && bounds2.MaximumYIsSet() &&
      bounds1.Minimum().Y() > bounds2.Maximum().Y()) return(false);
  if (bounds1.MaximumYIsSet() && bounds2.MinimumYIsSet() &&
      bounds1.Maximum().Y() < bounds2.Minimum().Y()) return(false);
  return(true);
}

/*
--------------------------------------------------------------------------------
                Check if two data bounds are the same
--------------------------------------------------------------------------------
*/

bool operator == (const DataBounds3D &bounds1, const DataBounds3D &bounds2)
{
  return bounds1.Minimum() == bounds2.Minimum() &&
         bounds1.Maximum() == bounds2.Maximum();
}

/*
--------------------------------------------------------------------------------
                Extract 2D bounds
--------------------------------------------------------------------------------
*/

DataBounds2D & DataBounds3D::Bounds2D() const
{
  DataBounds2D *bounds2d = new DataBounds2D();
  
  // Copy X- and Y-bounds
  if (MinimumXIsSet()) bounds2d->SetMinimumX(Minimum().X());
  if (MinimumYIsSet()) bounds2d->SetMinimumY(Minimum().Y());
  if (MaximumXIsSet()) bounds2d->SetMaximumX(Maximum().X());
  if (MaximumYIsSet()) bounds2d->SetMaximumY(Maximum().Y());
  
  return *bounds2d;
}
