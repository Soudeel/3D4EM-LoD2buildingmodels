
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
 Collection of functions for class LaserPoint
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <string.h>
#include "ObjectPoints2D.h"
#include "ObjectPoints.h"
#include "PointNumberList.h"
#include "LaserPoints.h"
#include "ImagePoint.h"
#include "DataGrid.h"
#include "Histogram.h"

/*
--------------------------------------------------------------------------------
                         Constructors and destructor
--------------------------------------------------------------------------------
*/

LaserPoint::LaserPoint(double x, double y, double z, int r, int pc)
  : Position3D(x, y, z)
{
  tags = NULL; attributes = NULL;
  Initialise();
  attributes[AttributeIndex(ReflectanceTag)] = r;
  attributes[AttributeIndex(PulseCountTag)]  = pc;
}

LaserPoint::LaserPoint(const Vector3D& v)
: Position3D(v)
{
  tags = NULL; attributes = NULL;
  Initialise();
}


LaserPoint::LaserPoint(double x, double y, double z, int r)
  : Position3D(x, y, z)
{
  tags = NULL; attributes = NULL;
  Initialise();
  attributes[AttributeIndex(ReflectanceTag)] = r;
}

LaserPoint::LaserPoint(double x, double y, double z)
  : Position3D(x, y, z)
{
  tags = NULL; attributes = NULL;
  Initialise();
}

LaserPoint::LaserPoint(const LaserPoint &pt)
  : Position3D()
{
  tags = NULL; attributes = NULL;
  Initialise();
  *this = pt;
}

LaserPoint::~LaserPoint()
{
  if (num_attributes > 0) { free(tags); free(attributes); }
}

void LaserPoint::Initialise()
{
  num_attributes = 0;
  if (tags != NULL) {free(tags); tags = NULL;}
  if (attributes != NULL) {free(attributes); attributes = NULL;}
}

/*
--------------------------------------------------------------------------------
                              Attribute retrieval
--------------------------------------------------------------------------------
*/

unsigned char LaserPoint::ExistingAttributeIndex
  (const LaserPointTag requested_tag) const
{
  const unsigned char *tag;
  unsigned char       index;

  if (requested_tag == PulseCountWithFlagTag)
    return ExistingAttributeIndex(PulseCountTag);
  if (requested_tag == ScanNumberWithoutAFNTag ||
      requested_tag == AFNTag)
    return ExistingAttributeIndex(ScanNumberTag);
  for (index=0, tag=tags; index<num_attributes; index++, tag++)
    if (*tag == requested_tag) return(index);
  return(UndefinedTag);
}

unsigned char LaserPoint::AttributeIndex(const LaserPointTag tag,
                                         bool double_value)
{
  unsigned char index;
  
  if (tag == PulseCountWithFlagTag) return AttributeIndex(PulseCountTag);
  index=ExistingAttributeIndex(tag);
  if (index == UndefinedTag) {
    num_attributes++;  // Add a new attribute
    index = num_attributes - 1; // This will be the last attribute
    if (double_value) num_attributes++;
    if (tags == NULL)
      tags = (unsigned char *) malloc(num_attributes * sizeof(unsigned char *));
    else
      tags = (unsigned char *) realloc(tags, num_attributes *
                                       sizeof(unsigned char *));
    tags[index] = (unsigned char) tag; // Store the tag;
    if (double_value) tags[index+1] = DoubleTag;
    if (attributes == NULL)
      attributes = (int *) malloc(num_attributes * sizeof(int));
    else
      attributes = (int *) realloc(attributes, num_attributes * sizeof(int));
    // Set default value of new attribute
    attributes[index] = INT_MIN;
    if (double_value) attributes[index+1] = INT_MIN;
  }
  return(index);
}

int LaserPoint::Attribute(const LaserPointTag tag) const
{
  int           pc;
  unsigned char index;
  
  if (tag == RedTag) return Red();
  else if (tag == GreenTag) return Green();
  else if (tag == BlueTag) return Blue();
  
  index = ExistingAttributeIndex(tag);
  if (index == UndefinedTag) return(INT_MIN);

  switch (tag) {
  	// Special cases:
  	case PulseCountTag:
      pc = attributes[index];
      return pc - ((pc >> 30) << 30); // Pulse count without last pulse bit
    case ScanNumberWithoutAFNTag:
      return (attributes[index] / 10);
    case AFNTag: 
      return (attributes[index] - 10 * (attributes[index] / 10));
    // For all other attributes
    default:
      return attributes[index];
  }
  return(attributes[index]); // We don't get here, just to satisfy compiler
}

int & LaserPoint::Attribute(const LaserPointTag tag)
{
  unsigned char index;
  
  if (tag == RedTag || tag == GreenTag || tag == BlueTag) {
  	printf("Error: int & Attribute(tag) cannot be used for colour tags\n");
  	printf("       Use SetAttribute or SetColour instead\n");
  }
  index = AttributeIndex(tag);
  return attributes[index];
}

float LaserPoint::FloatAttribute(const LaserPointTag tag) const
{
  float *address;

  unsigned char index=ExistingAttributeIndex(tag);
  if (index == UndefinedTag) return(0.0);
  address = (float *) (attributes + index);
  return(*address);
}

float & LaserPoint::FloatAttribute(const LaserPointTag tag)
{
  float *address;

  unsigned char index = AttributeIndex(tag);
  address = (float *) (attributes + index);
  return (float &) *address;
}

double LaserPoint::DoubleAttribute(const LaserPointTag tag) const
{
  double *address;
  unsigned char index;
  
  if (tag >= XCoordinateTag && tag <= ZCoordinateTag)
    return X((int) tag - (int) XCoordinateTag);
  index = ExistingAttributeIndex(tag);
  if (index == UndefinedTag) return(0.0);
  address = (double *) (attributes + index);
  return(*address);
}

double & LaserPoint::DoubleAttribute(const LaserPointTag tag)
{
  double *address;
  unsigned char index;
  
  if (tag >= XCoordinateTag && tag <= ZCoordinateTag)
    return X((int) tag - (int) XCoordinateTag);
  index = AttributeIndex(tag, true);
  address = (double *) (attributes + index);
  return (double &) *address;
}

long long int LaserPoint::LongAttribute(const LaserPointTag tag) const
{
  double *address;
  unsigned char index;
  
  switch (tag) {
  	case LongSegmentNumberTag: return LongSegmentNumber();
  	default: printf("Invalid long long int tag %d\n", tag);
  }
  return INT_MIN;
}


bool LaserPoint::HasAttribute(const LaserPointTag tag) const
{
  unsigned char index;
  if (tag >= XCoordinateTag && tag <= ZCoordinateTag) return true;;
  if (tag >= RedTag && tag <= BlueTag) return HasAttribute(ColourTag);
  if (tag == LongSegmentNumberTag)
    return (HasAttribute(SegmentNumberTag) &&
	        HasAttribute(SegmentStartTileNumberTag));
  index = ExistingAttributeIndex(tag);
  return index != UndefinedTag;
}

bool LaserPoint::HasAttributeValue(const LaserPointTag tag, int value) const
{
  unsigned char index=ExistingAttributeIndex(tag);
  if (index == UndefinedTag) return false;
  return (Attribute(tag) == value);
}

/*
--------------------------------------------------------------------------------
                      Simple attribute manipulations
--------------------------------------------------------------------------------
*/

void LaserPoint::SetAttribute(const LaserPointTag tag, const int value)
{
  switch (tag) {
  	case RedTag  : SetColour(value, Green(), Blue()); break;
  	case GreenTag: SetColour(Red(), value, Blue()); break;
  	case BlueTag : SetColour(Red(), Green(), value); break;
  	default      : Attribute(tag) = value;
  }
}

void LaserPoint::SetAttribute(const LaserPointTag tag, const float value)
{
  FloatAttribute(tag) = value;
}

void LaserPoint::SetDoubleAttribute(const LaserPointTag tag, const double value)
{
  DoubleAttribute(tag) = value;
}

void LaserPoint::SetLongAttribute(const LaserPointTag tag, const long long int value)
{
  switch (tag) {
  	case LongSegmentNumberTag: SetLongSegmentNumber(value); break;
  	default: printf("Error: tag %d is not of type long long int\n"); break;
  }
}

bool LaserPoint::RemoveAttribute(const LaserPointTag tag)
{
  unsigned char index=ExistingAttributeIndex(tag), i;
  if (index == UndefinedTag) return false;
  for (i=index; i<num_attributes-1; i++) {
    tags[i] = tags[i+1];
    attributes[i] = attributes[i+1];
  }
  num_attributes--;
  // Remove the next value if the next tag is the second part of a double value
  if (index < num_attributes && num_attributes > 0) {
    if (tags[index] == DoubleTag) {
      for (i=index; i<num_attributes-1; i++) {
        tags[i] = tags[i+1];
        attributes[i] = attributes[i+1];
      }
      num_attributes--;
    }
  }  
  return true;
}

bool LaserPoint::RemoveAttributes()
{
  if (num_attributes == 0) return false;
  num_attributes = 0;
  free(attributes); attributes = NULL;
  free(tags); tags = NULL;
  return true;
}

bool LaserPoint::RenameAttribute(const LaserPointTag oldtag,
                                 const LaserPointTag newtag)
{
  unsigned char index=ExistingAttributeIndex(oldtag);
  if (index == UndefinedTag) return false;
  if (newtag == PulseCountWithFlagTag) tags[index] = PulseCountTag;
  else tags[index] = newtag;
  return true;
}

void LaserPoint::SortAttributes()
{
  unsigned char              index, *tag;
  std::vector<unsigned char> sorted_tags;
  std::vector<unsigned char>::iterator sorted_tag;
  LaserPoint                 sorted_point;
  
  // Create a sorted list of attribute values
  for (index=0, tag=tags; index<num_attributes; index++, tag++)
    if (*tag < NoTag) sorted_tags.push_back(*tag);
  std::sort(sorted_tags.begin(), sorted_tags.end());
  
  // Put all attributes in the right sequence in a new point
  for (sorted_tag=sorted_tags.begin(); sorted_tag!=sorted_tags.end();
       sorted_tag++)
    switch (AttributeType((LaserPointTag) *sorted_tag)) {
      case IntegerAttributeType:
      	sorted_point.Attribute(*sorted_tag) = Attribute(*sorted_tag); break;
      case FloatAttributeType:
      	sorted_point.FloatAttribute(*sorted_tag) = FloatAttribute(*sorted_tag); break;
      case DoubleAttributeType:
      	sorted_point.DoubleAttribute(*sorted_tag) = DoubleAttribute(*sorted_tag); break;
    }

  // Copy the tags and values
  memmove((void *) tags, (void *) sorted_point.AttributeTags(), num_attributes);
  memmove((void *) attributes, (void *) sorted_point.AttributeValues(),
          num_attributes * sizeof(int));
        
  // Erase sorted tags and values
  sorted_point.Initialise();
  sorted_tags.erase(sorted_tags.begin(), sorted_tags.end());
}

/*
--------------------------------------------------------------------------------
                      Retrieval of (specific) attributes
--------------------------------------------------------------------------------
*/

int LaserPoint::Reflectance() const { return Attribute(ReflectanceTag); }
int & LaserPoint::Reflectance() { return Attribute(ReflectanceTag); }

int LaserPoint::PulseCount() const { return Attribute(PulseCountTag); }
int & LaserPoint::PulseCount() { return Attribute(PulseCountTag); }
int LaserPoint::PulseCountWithFlag() const { return Attribute(PulseCountWithFlagTag); }
int & LaserPoint::PulseCountWithFlag() { return Attribute(PulseCountWithFlagTag); }

void LaserPoint::SetLastPulseFlag(bool last)
{
  int pc = Attribute(PulseCountWithFlagTag);
  pc -= (pc >> 30) << 30;    // Old last pulse flag removed
  if (last) pc += (1 << 30); // Set new last pulse flag if requested
  Attribute(PulseCountWithFlagTag) = pc; 
}

float LaserPoint::Residual() const { return FloatAttribute(ResidualTag); }
float & LaserPoint::Residual() { return FloatAttribute(ResidualTag); }

bool LaserPoint::IsSelected() const
{
  if (HasAttribute(IsSelectedTag)) return (bool) Attribute(IsSelectedTag);
  return false;
}
void LaserPoint::Select() { Attribute(IsSelectedTag) = 1; }
void LaserPoint::UnSelect() 
{ 
  if (HasAttribute(IsSelectedTag)) Attribute(IsSelectedTag) = 0;
}

int LaserPoint::PulseLength() const { return Attribute(PulseLengthTag); }
int & LaserPoint::PulseLength() { return Attribute(PulseLengthTag); }

int LaserPoint::PlaneNumber() const { return Attribute(PlaneNumberTag); }
int & LaserPoint::PlaneNumber() { return Attribute(PlaneNumberTag); }

int LaserPoint::SegmentNumber() const { return Attribute(SegmentNumberTag); }
int & LaserPoint::SegmentNumber() { return Attribute(SegmentNumberTag); }

int LaserPoint::PolygonNumber() const { return Attribute(PolygonNumberTag); }
int & LaserPoint::PolygonNumber() { return Attribute(PolygonNumberTag); }

/*
--------------------------------------------------------------------------------
                      Check if a pulse is of a specific type
--------------------------------------------------------------------------------
*/

bool LaserPoint::IsPulseType(LaserPulseType type) const
{
  int pc = PulseCount();
  
  if (pc == 0) return true; // No pulse count set
  switch (type) {
    case AnyPulse         : return true;
    case FirstPulse       :
    case SecondPulse      :
    case ThirdPulse       :
    case FourthPulse      :
    case FifthPulse       : return (pc == (int) type);
    case LastPulse        : return IsLastPulse();
    case NotFirstPulse    : return (pc != (int) FirstPulse);
    case NotLastPulse     : return !IsLastPulse();
    case SinglePulse      : return ((pc == (int) FirstPulse) && IsLastPulse());
    case MultiplePulse    : return ((pc > (int) FirstPulse) || !IsLastPulse());
    case FirstNorLastPulse: return ((pc > (int) FirstPulse) && !IsLastPulse());
  }
  return true; // Superfluous statement to avoid compiler warnings
}
 
/*
--------------------------------------------------------------------------------
                              Copy assignment
--------------------------------------------------------------------------------
*/

LaserPoint & LaserPoint::operator = (const LaserPoint &p)
{
  if (this == &p) return *this; // Don't copy onto itself

  x[0] = p.x[0];
  x[1] = p.x[1];
  x[2] = p.x[2];
  if (p.num_attributes > 0) {
    if (p.num_attributes > num_attributes) {
      if (num_attributes > 0) {
        free(tags);
        free(attributes);
      }
      tags = (unsigned char *) malloc(p.num_attributes * sizeof(unsigned char));
      attributes = (int *) malloc(p.num_attributes * sizeof(int));
    }
    memcpy((void *) tags, (const void *) p.tags,
           p.num_attributes * sizeof(unsigned char));
    memcpy((void *) attributes, (const void *) p.attributes,
           p.num_attributes * sizeof(int));
  }
  else {
    if (num_attributes > 0) {
      free(tags);
      free(attributes);
    }
    tags       = NULL;
    attributes = NULL;
  }
  num_attributes = p.num_attributes;
  return(*this);
}

LaserPoint & LaserPoint::operator = (const Vector3D &pt)
{
	for(int i=0;i<3;i++)
		(*this)[i]	= pt[i];
	return (*this);
}

/*
--------------------------------------------------------------------------------
                         Print a laser altimetry point
--------------------------------------------------------------------------------
*/

void LaserPoint::Print(FILE *fd) const
{
  fprintf(fd, "XYZ (%.2f, %.2f, %.2f)\n", x[0], x[1], x[2]);
  PrintAttributes(fd);
}

void LaserPoint::PrintAttributes(FILE *fd) const
{
  const unsigned char *tag;
  const int           *attribute;
  const float         *floatattribute;
  const double        *doubleattribute;
  int                 i;

  for (i=0, tag=AttributeTags(), attribute=AttributeValues();
       i<NumAttributes(); i++, tag++, attribute++) {
    if (*tag >= NoTag) continue;
    fprintf(fd, "  %s: ", AttributeName((const LaserPointTag) *tag, true));
    // get correct pointer type
    switch (AttributeType((const LaserPointTag) *tag)) {
      default:
      case IntegerAttributeType: 
        if (*tag == ColourTag || *tag == AverageColourTag)
          fprintf(fd, "(%d, %d, %d)\n", Red(), Green(), Blue());
        else if (*tag == PulseCountTag) {
          fprintf(fd, "%d", PulseCount());
          if (IsLastPulse()) fprintf(fd, " (last)\n");
          else printf(" (not last)\n");
        }
        else
          fprintf(fd, "%d\n", *attribute);
        break;
      case FloatAttributeType:
        floatattribute = (const float *) attribute;
        if (*tag == InclinationTag || *tag == AzimuthTag ||
		    *tag == AngleTag)
          fprintf(fd, "%.3f\n", *floatattribute * 45.0 / atan(1.0));
        else
          fprintf(fd, "%.3f\n", *floatattribute);
        break;
      case DoubleAttributeType:
        doubleattribute = (const double *) attribute;
        fprintf(fd, "%.6f\n", *doubleattribute);
        i++; tag++; attribute++; // Skip second part of double in the next attribute value
        break;
    }
  }
}

/*
--------------------------------------------------------------------------------
                         Convert to an image point
--------------------------------------------------------------------------------
*/

ImagePoint LaserPoint::Map_Into_Image(int number, const ImageGrid &grid) const
{
  return(ImagePoint(number,
		            (double) ((grid.YOffset() - x[1]) / grid.Pixelsize() - 0.5),
                    (double) ((x[0] - grid.XOffset()) / grid.Pixelsize() - 0.5),
	                0.0));
}

/*
--------------------------------------------------------------------------------
                 Convert height or reflectance to grey value
--------------------------------------------------------------------------------
*/

unsigned char LaserPoint::Height_To_GreyValue(const DataGrid &grid) const
{
  unsigned char gv;
  double        dgv;
  
  dgv = (x[2] - grid.DataOffset()) / grid.DataScale();
  if (dgv < 0.0) gv = 0;
  else if (dgv > 255.0) gv = 255;
  else gv = (unsigned char) dgv;
  return(gv);
}

unsigned char LaserPoint::Reflectance_To_GreyValue(const DataGrid &grid) const
{
  unsigned char gv;
  double        dgv;
  
  dgv = ((double) Reflectance() - grid.DataOffset()) / grid.DataScale();
  if (dgv < 0.0) gv = 0;
  else if (dgv > 255.0) gv = 255;
  else gv = (unsigned char) dgv;
  return(gv);
}

/*
--------------------------------------------------------------------------------
                 Add a point attribute to a histogram
--------------------------------------------------------------------------------
*/

void LaserPoint::AddToHistogram(Histogram &histo, int type) const
{
  switch (type) {
    case 1: histo.Add(x[0]); break;
    case 2: histo.Add(x[1]); break;
    case 3: histo.Add(x[2]); break;
    case 4: histo.Add((double) Reflectance()); break;
    case 5: histo.Add((double) PulseCount()); break;
    case 6: histo.Add((double) Red()); break;
    case 7: histo.Add((double) Green()); break;
    case 8: histo.Add((double) Blue()); break;
    default: fprintf(stderr,
                     "Error: wrong data type in LaserPoint::AddToHistogram\n");
             exit(0);
  }
}

void LaserPoint::AddToHistogram(Histogram &histo, int type,
                                const LaserPointTag &tag) const
{
  switch (type) {
    case 0: if (HasAttribute(tag)) histo.Add((double) Attribute(tag)); break;
    case 1: histo.Add(x[0]); break;
    case 2: histo.Add(x[1]); break;
    case 3: histo.Add(x[2]); break;
    default: fprintf(stderr,
                     "Error: wrong data type in LaserPoint::AddToHistogram\n");
             exit(0);
  }
}

/*
--------------------------------------------------------------------------------
                 Construct an object point from a laser point
--------------------------------------------------------------------------------
*/

ObjectPoint LaserPoint::ConstructObjectPoint(const PointNumber &number,
                                             const Covariance3D &Q) const
{
  return(ObjectPoint(vect(), number, Q));
}

/*
--------------------------------------------------------------------------------
                           Inside polygon test
--------------------------------------------------------------------------------
*/

int LaserPoint::InsidePolygon(const LaserPoints &pts,
                              const PointNumberList &top,
                              bool skip_polygon_check) const
{
  LaserPoints::const_iterator     pt0, pt1;
  PointNumberList::const_iterator node;
  int                             num_intersections;
  double                          Y_test;

// Check if the polygon is closed and has at least three points

  if (!skip_polygon_check) {
    if (top.begin()->Number() != (top.end()-1)->Number()) return(0);
    if (top.size() < 4) return(0); // 4 since begin and end point are the same
  }
  
// Get the first polygon point

  pt0 = pts.begin() + top.begin()->Number();

// Count the number of intersections of polygon edges with the line from the
// point (X, Y) to (X, inf).

  num_intersections = 0;
  for (node=top.begin()+1; node!=top.end(); node++) {

// Get the next polygon point

    pt1 = pts.begin() + node->Number();

// Check whether the lines intersect

    if ((pt0->X() - X()) * (pt1->X() - X()) <= 0 && pt1->X() != X()) {
      Y_test = ((pt1->X() - X()) * pt0->Y() +
                (X() - pt0->X()) * pt1->Y()) / (pt1->X() - pt0->X());
      if (Y_test > Y()) num_intersections++;
    }

// Get ready for the next edge

    pt0 = pt1;
  }

// Return 1 for an odd number of intersections

  if ((num_intersections/2)*2 == num_intersections) return(0);
  else return(1);
}

int LaserPoint::InsidePolygon(const ObjectPoints2D &pts,
                              const PointNumberList &top,
                              bool skip_polygon_check) const
{
  ObjectPoint2D                   *pt0, *pt1;
  PointNumberList::const_iterator node;
  int                             num_intersections;
  double                          Y_test;

/* Check if the polygon is closed and has at least three points */

  if (!skip_polygon_check) {
    if (top.begin()->Number() != (top.end()-1)->Number()) return(0);
    if (top.size() < 4) return(0); // 4 since begin and end point are the same
  }

/* Get the first polygon point */

  pt0 = pts.GetPoint(*(top.begin()));
  if (!pt0) {
    fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", top.begin()->Number());
    return(0);
  }

/* Count the number of intersections of polygon edges with the line from the
 * point (X, Y) to (X, inf).
 */

  num_intersections = 0;
  for (node=top.begin()+1; node!=top.end(); node++) {

/* Get the next polygon point */

    pt1 = pts.GetPoint(*node);
    if (!pt1) {
      fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", node->Number());
      return(0);
    }

/* Check whether the lines intersect */

    if ((pt0->X() - X()) * (pt1->X() - X()) <= 0 && pt1->X() != X()) {
      Y_test = ((pt1->X() - X()) * pt0->Y() +
                (X() - pt0->X()) * pt1->Y()) / (pt1->X() - pt0->X());
      if (Y_test > Y()) num_intersections++;
    }

/* Get ready for the next edge */

    pt0 = pt1;
  }

/* Return 1 for an odd number of intersections */

  if ((num_intersections/2)*2 == num_intersections) return(0);
  else return(1);
}

int LaserPoint::InsidePolygon(const ObjectPoints &pts,
                              const PointNumberList &top,
                              bool skip_polygon_check) const
{
  ObjectPoint                     *pt0, *pt1;
  PointNumberList::const_iterator node;
  int                             num_intersections;
  double                          Y_test;

/* Check if the polygon is closed and has at least three points */

  if (!skip_polygon_check) {
    if (top.begin()->Number() != (top.end()-1)->Number()) return(0);
    if (top.size() < 4) return(0); // 4 since begin and end point are the same
  }

/* Get the first polygon point */

  pt0 = pts.GetPoint(*(top.begin()));
  if (!pt0) {
    fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", top.begin()->Number());
    return(0);
  }

/* Count the number of intersections of polygon edges with the line from the
 * point (X, Y) to (X, inf).
 */

  num_intersections = 0;
  for (node=top.begin()+1; node!=top.end(); node++) {

/* Get the next polygon point */

    pt1 = pts.GetPoint(*node);
    if (!pt1) {
      fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints2D list\n", node->Number());
      return(0);
    }

/* Check whether the lines intersect */

    if ((pt0->X() - X()) * (pt1->X() - X()) <= 0 && pt1->X() != X()) {
      Y_test = ((pt1->X() - X()) * pt0->Y() +
                (X() - pt0->X()) * pt1->Y()) / (pt1->X() - pt0->X());
      if (Y_test > Y()) num_intersections++;
    }

/* Get ready for the next edge */

    pt0 = pt1;
  }

/* Return 1 for an odd number of intersections */

  if ((num_intersections/2)*2 == num_intersections) return(0);
  else return(1);
};

/*
--------------------------------------------------------------------------------
                           Inside polygon test
                           Jordan Curve Theory
--------------------------------------------------------------------------------
*/

int LaserPoint::InsidePolygonJordan(const ObjectPoints &pts,
                              const PointNumberList &top) const
{
  ObjectPoint                     *pt0, *pt1;
  PointNumberList::const_iterator node, node_i, node_j;
  int                             i, j, c;

/* Get the first polygon point */
  pt0 = pts.GetPoint(*(top.begin()));
  if (!pt0) {
    fprintf(stderr, "Error: polygon node %d is missing in the ObjectPoints3D list\n", top.begin()->Number());
    return(0);
  }
  
   c = 0;
      for (node=top.begin()+1; node!=top.end(); node++) {
        pt1 = pts.GetPoint(*node);
        if( ((pt1->Y()>Y()) != (pt0->Y()>Y())) &&
           	 (X() < (pt0->X()-pt1->X()) * (Y()-pt1->Y()) / (pt0->Y()-pt1->Y()) + pt1->X()) )
              c = !c;   
        pt0 = pt1;
      }          
 return(c);
}

/*
--------------------------------------------------------------------------------
                           Point arithmic
--------------------------------------------------------------------------------
*/

LaserPoint & LaserPoint::Add(const LaserPoint &p2) const
{
  const unsigned char *tag;
  int                 i;
  LaserPoint          *p = new LaserPoint();

  // Add coordinates
  p->vect() = vect() + p2.vect();  
  
  // Add attribute values
  for (i=0, tag=AttributeTags(); i<NumAttributes(); i++, tag++) {
  	// Add two values
    if (p2.HasAttribute(*tag)) {
      switch (AttributeType((LaserPointTag) *tag)) {
        case IntegerAttributeType:
        default:
          p->Attribute(*tag) = Attribute(*tag) + p2.Attribute(*tag);
	      break;
        case FloatAttributeType:
  	      p->FloatAttribute(*tag) = FloatAttribute(*tag) + p2.FloatAttribute(*tag);
	  	  break;
  	    case DoubleAttributeType:
          p->DoubleAttribute(*tag) = DoubleAttribute(*tag) + p2.DoubleAttribute(*tag);
          i++; tag++; // Skip second part of double
		  break;
	  }
    }
    // Otherwise assume 0 for p2 attribute
    else {
      switch (AttributeType((LaserPointTag) *tag)) {
        case IntegerAttributeType:
        default:
          p->Attribute(*tag) = Attribute(*tag);
	      break;
        case FloatAttributeType:
  	      p->FloatAttribute(*tag) = FloatAttribute(*tag);
	  	  break;
  	    case DoubleAttributeType:
          p->DoubleAttribute(*tag) = DoubleAttribute(*tag);
          i++; tag++; // Skip second part of double
		  break;
	  }
    }
  }
  
  // Check for attributes only available in p2
  for (i=0, tag=p2.AttributeTags(); i<p2.NumAttributes(); i++, tag++) {
  	// Assume 0 for p1 attribute
    if (!HasAttribute(*tag)) {
      switch (AttributeType((LaserPointTag) *tag)) {
        case IntegerAttributeType:
        default:
          p->Attribute(*tag) = p2.Attribute(*tag);
	      break;
        case FloatAttributeType:
  	      p->FloatAttribute(*tag) = p2.FloatAttribute(*tag);
	  	  break;
  	    case DoubleAttributeType:
          p->DoubleAttribute(*tag) = p2.DoubleAttribute(*tag);
          i++; tag++; // Skip second part of double
		  break;
	  }
    }
  }

  return *p;
}

LaserPoint & LaserPoint::Subtract(const LaserPoint &p2) const
{
  const unsigned char *tag;
  int                 i;

  // Lazy programmer
  return Add(p2.Multiply(-1.0));
}

LaserPoint & LaserPoint::Multiply(double d) const
{
  const unsigned char *tag;
  int                 i;
  LaserPoint          *p = new LaserPoint();

  // Multiply coordinates
  p->vect() = vect() * d;  

  // Multiply attribute values
  for (i=0, tag=AttributeTags(); i<NumAttributes(); i++, tag++) {
    switch (AttributeType((LaserPointTag) *tag)) {
      case IntegerAttributeType:
      default:
        p->Attribute(*tag) = (int) ((double) Attribute(*tag) * d);
	    break;
      case FloatAttributeType:
  	    p->FloatAttribute(*tag) = (float) ((double) FloatAttribute(*tag) * d);
	    break;
  	  case DoubleAttributeType:
        p->DoubleAttribute(*tag) = DoubleAttribute(*tag) * d;
        i++; tag++; // Skip second part of double
		break;
    }
  }

  return *p;
}

LaserPoint & LaserPoint::Divide(double d) const
{
  return Multiply(1.0/d);
}

/*
--------------------------------------------------------------------------------
                           Point comparisons
--------------------------------------------------------------------------------
*/

bool operator == (const LaserPoint &p1, const LaserPoint &p2)
{
  const unsigned char *tag;
  int                 i;

  // Compare coordinates  
  if (p1.X() != p2.X() || p1.Y() != p2.Y() || p1.Z() != p2.Z()) return false;
  
  // Compare attribute values
  for (i=0, tag=p1.AttributeTags(); i<p1.NumAttributes(); i++, tag++) {
    if (!p2.HasAttribute(*tag)) return false;
    switch (AttributeType((LaserPointTag) *tag)) {
      case IntegerAttributeType:
      default:
        if (p1.Attribute(*tag) != p2.Attribute(*tag)) return false;
		break;
      case FloatAttributeType:
  	    if (p1.FloatAttribute(*tag) != p2.FloatAttribute(*tag)) return false;
		break;
  	  case DoubleAttributeType:
        if (p1.DoubleAttribute(*tag) != p2.DoubleAttribute(*tag)) return false;
		break;
    }
  }
  
  // Check if p2 has any additional attributes
  for (i=0, tag=p2.AttributeTags(); i<p2.NumAttributes(); i++, tag++) {
    if (!p1.HasAttribute(*tag)) return false;
  }
  return true;
}

bool operator != (const LaserPoint &p1, const LaserPoint &p2)
{;
  return(!(p1 == p2));
}

/*
--------------------------------------------------------------------------------
                           Coordinate swapping
--------------------------------------------------------------------------------
*/

void LaserPoint::SwapXY()
{
  double coord;
  coord = X(); X() = Y(); Y() = coord;
}

void LaserPoint::SwapXZ()
{
  double coord;
  coord = X(); X() = Z(); Z() = coord;
}

void LaserPoint::SwapYZ()
{
  double coord;
  coord = Z(); Z() = Y(); Y() = coord;
}

/*
--------------------------------------------------------------------------------
                           Scan and Point Id 
--------------------------------------------------------------------------------
*/
/// Set the scan number of the point 
void LaserPoint::SetScanNumber(int number)
{ 
	SetAttribute(ScanNumberTag, number); 
}

/// Set the point number of the point 
void LaserPoint::SetPointNumber(int number)
{ 
	SetAttribute(PointNumberTag, number); 
}	

/// Get the scan number of the point 
int LaserPoint::ScanNumber(bool afn_in_number) const
{
  int number = Attribute(ScanNumberTag);
  if (afn_in_number) return number/10;
  return number;
}

/// Get the point number of the point 
int LaserPoint::GetPointNumber() const
{ 
	return Attribute(PointNumberTag); 
}	

/// Return both scan number and point number as a 64-bit integer.
long long int LaserPoint::GetId() const
{
	int scanId = GetScanNumber();
	int pointId = GetPointNumber();

	return ((long long int)(pointId)) + (((long long int)(scanId))<<32);
}

///Set both scan id and point id from a given 64-bit integer.
void LaserPoint::SetId(long long int id)
{
	SetScanNumber(id>>32);
	SetPointNumber((int)id);
}

/// Return both segment and tile number as a 64-bit integer.
long long int LaserPoint::LongSegmentNumber() const
{
  int tile_number;
  if (!HasAttribute(SegmentNumberTag)) return INT_MIN;
  if (!HasAttribute(SegmentStartTileNumberTag)) tile_number = 0;
  else if (Attribute(SegmentStartTileNumberTag) == INT_MIN) tile_number = 0;
  else tile_number = Attribute(SegmentStartTileNumberTag);
  return (long long int) Attribute(SegmentNumberTag) +
         1000000 * (long long int) tile_number;
}

/// Set both segment and tile number from a given 64-bit integer.
void LaserPoint::SetLongSegmentNumber(long long int long_number)
{
  Attribute(SegmentNumberTag) = long_number%1000000;
  Attribute(SegmentStartTileNumberTag) = long_number / 1000000;
}

/*
--------------------------------------------------------------------------------
                           Stream operators.
--------------------------------------------------------------------------------
*/

/// output stream insertion operator
std::ostream& operator << (std::ostream& os, const LaserPoint &pt)
{
	os<<"x: "<<pt[0]<<" y: "<<pt[1]<<" z: "<<pt[2]<<endl;
	return os;
}
	
/*
--------------------------------------------------------------------------------
                           Normals
--------------------------------------------------------------------------------
*/

/// Return the normal stored in attributes
Vector3D LaserPoint::Normal() const
{
  Vector3D normal;
  
  if (!HasAttribute(NormalXTag) || !HasAttribute(NormalYTag) ||
      !HasAttribute(NormalZTag)) return normal;
  normal.X() = FloatAttribute(NormalXTag);
  normal.Y() = FloatAttribute(NormalYTag);
  normal.Z() = FloatAttribute(NormalZTag);
  return normal;
}

/// Return the scaled normal stored in attributes
Vector3D LaserPoint::ScaledNormal() const
{
  Vector3D normal;
  
  if (!HasAttribute(ScaledNormalXTag) || !HasAttribute(ScaledNormalYTag) ||
      !HasAttribute(ScaledNormalZTag)) return normal;
  normal.X() = FloatAttribute(ScaledNormalXTag);
  normal.Y() = FloatAttribute(ScaledNormalYTag);
  normal.Z() = FloatAttribute(ScaledNormalZTag);
  return normal;
}

/// Store the normal in the point's attributes
void LaserPoint::SetNormal(const Vector3D &normal)
{
  FloatAttribute(NormalXTag) = normal.X();
  FloatAttribute(NormalYTag) = normal.Y();
  FloatAttribute(NormalZTag) = normal.Z();
}

// Flip normal
void LaserPoint::FlipNormal()
{
  if (HasAttribute(NormalXTag))
    FloatAttribute(NormalXTag) = -1.0 * FloatAttribute(NormalXTag);
  if (HasAttribute(NormalYTag))
    FloatAttribute(NormalYTag) = -1.0 * FloatAttribute(NormalYTag);
  if (HasAttribute(NormalZTag))
    FloatAttribute(NormalZTag) = -1.0 * FloatAttribute(NormalZTag);
}

// Flip scaled normal
void LaserPoint::FlipScaledNormal()
{
  if (HasAttribute(ScaledNormalXTag))
    FloatAttribute(ScaledNormalXTag) = -1.0 * FloatAttribute(ScaledNormalXTag);
  if (HasAttribute(ScaledNormalYTag))
    FloatAttribute(ScaledNormalYTag) = -1.0 * FloatAttribute(ScaledNormalYTag);
  if (HasAttribute(ScaledNormalZTag))
    FloatAttribute(ScaledNormalZTag) = -1.0 * FloatAttribute(ScaledNormalZTag);
}

