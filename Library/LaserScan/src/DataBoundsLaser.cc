
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
 Collection of functions for class DataBoundsLaser

 Initial creation
 Author : George Vosselman
 Date   : 25-03-1999

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
#include "DataBoundsLaser.h"
#include "BNF_io.h"
#include "stdmath.h"

/*
--------------------------------------------------------------------------------
                         Copy assignment
--------------------------------------------------------------------------------
*/

DataBoundsLaser DataBoundsLaser::operator = (const DataBoundsLaser &bounds)
{
  // Check for self assignment
  if (this == &bounds) return *this;
  minimum    = bounds.minimum;
  maximum    = bounds.maximum;
  min_is_set = bounds.min_is_set;
  max_is_set = bounds.max_is_set;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise bound setting switches
--------------------------------------------------------------------------------
*/
void DataBoundsLaser::Initialise()
{
  minimum.Initialise();
  maximum.Initialise();
  min_is_set.Initialise();
  max_is_set.Initialise();
  min_is_set.X() = min_is_set.Y() = min_is_set.Z() = 0.0;
  max_is_set.X() = max_is_set.Y() = max_is_set.Z() = 0.0;
}

void DataBoundsLaser::Initialise(const DataBounds2D &bounds)
{
  Initialise();
  minimum.X() = bounds.Minimum().X();
  minimum.Y() = bounds.Minimum().Y();
  maximum.X() = bounds.Maximum().X();
  maximum.Y() = bounds.Maximum().Y();
  if (bounds.MinimumXIsSet()) min_is_set.X() = 1.0;
  if (bounds.MinimumYIsSet()) min_is_set.Y() = 1.0;
  if (bounds.MaximumXIsSet()) max_is_set.X() = 1.0;
  if (bounds.MaximumYIsSet()) max_is_set.Y() = 1.0;
}

void DataBoundsLaser::Initialise(const DataBounds3D &bounds)
{
  Initialise();
  minimum.vect() = bounds.Minimum().vect();
  maximum.vect() = bounds.Maximum().vect();
  if (bounds.MinimumXIsSet()) min_is_set.X() = 1.0;
  if (bounds.MinimumYIsSet()) min_is_set.Y() = 1.0;
  if (bounds.MinimumZIsSet()) min_is_set.Z() = 1.0;
  if (bounds.MaximumXIsSet()) max_is_set.X() = 1.0;
  if (bounds.MaximumYIsSet()) max_is_set.Y() = 1.0;
  if (bounds.MaximumZIsSet()) max_is_set.Z() = 1.0;
}

/*
--------------------------------------------------------------------------------
              Set minimum or maximum value for generic attributes
--------------------------------------------------------------------------------
*/

void DataBoundsLaser::SetMinimumValue(const LaserPointTag tag, int value)
  { minimum.SetAttribute(tag, value); min_is_set.SetAttribute(tag, 1); }
void DataBoundsLaser::SetMinimumValue(const LaserPointTag tag, float value)
  { minimum.SetAttribute(tag, value); min_is_set.SetAttribute(tag, (float) 1.0); }
void DataBoundsLaser::SetMinimumValue(const LaserPointTag tag, double value)
  { minimum.SetDoubleAttribute(tag, value); min_is_set.SetDoubleAttribute(tag, 1.0); }
void DataBoundsLaser::SetMaximumValue(const LaserPointTag tag, int value)
  { maximum.SetAttribute(tag, value); max_is_set.SetAttribute(tag, 1); }
void DataBoundsLaser::SetMaximumValue(const LaserPointTag tag, float value)
  { maximum.SetAttribute(tag, value); max_is_set.SetAttribute(tag, (float) 1.0); }
void DataBoundsLaser::SetMaximumValue(const LaserPointTag tag, double value)
  { maximum.SetDoubleAttribute(tag, value); max_is_set.SetDoubleAttribute(tag, 1.0); }

/*
--------------------------------------------------------------------------------
              Set minimum or maximum value for specific attributes
--------------------------------------------------------------------------------
*/

void DataBoundsLaser::SetMinimumX(double x)
  { minimum.SetX(x);  min_is_set.SetX(1.0); }
void DataBoundsLaser::SetMinimumY(double y)
  { minimum.SetY(y);  min_is_set.SetY(1.0); }
void DataBoundsLaser::SetMinimumZ(double z)
  { minimum.SetZ(z);  min_is_set.SetZ(1.0); }
void DataBoundsLaser::SetMinimumReflectance(int r)
  { minimum.SetReflectance(r);  min_is_set.SetReflectance(1); }
void DataBoundsLaser::SetMinimumColour(int r, int g, int b)
  { minimum.SetColour(r, g, b);  min_is_set.SetAttribute(ColourTag, 1); }
void DataBoundsLaser::SetMinimumPulseCount(int pc)
  { minimum.SetPulseCount(pc);  min_is_set.SetPulseCount(1); }
void DataBoundsLaser::SetMinimumPulseLength(int pl)
  { minimum.SetPulseLength(pl);  min_is_set.SetPulseLength(1); }
void DataBoundsLaser::SetMinimumLabel(int l)
  { minimum.Label(l);  min_is_set.Label(1); }
void DataBoundsLaser::SetMinimumScalar(float s)
  { minimum.SetAttribute(ScalarTag, s);  min_is_set.SetAttribute(ScalarTag, (float) 1.0); }

void DataBoundsLaser::SetMaximumX(double x)
  { maximum.SetX(x);  max_is_set.SetX(1.0); }
void DataBoundsLaser::SetMaximumY(double y)
  { maximum.SetY(y);  max_is_set.SetY(1.0); }
void DataBoundsLaser::SetMaximumZ(double z)
  { maximum.SetZ(z);  max_is_set.SetZ(1.0); }
void DataBoundsLaser::SetMaximumReflectance(int r)
  { maximum.SetReflectance(r);  max_is_set.SetReflectance(1); }
void DataBoundsLaser::SetMaximumColour(int r, int g, int b)
  { maximum.SetColour(r, g, b);  max_is_set.SetAttribute(ColourTag, 1); }
void DataBoundsLaser::SetMaximumPulseCount(int pc)
  { maximum.SetPulseCount(pc);  max_is_set.SetPulseCount(1); }
void DataBoundsLaser::SetMaximumPulseLength(int pl)
  { maximum.SetPulseLength(pl);  max_is_set.SetPulseLength(1); }
void DataBoundsLaser::SetMaximumLabel(int l)
  { maximum.Label(l);  max_is_set.Label(1); }
void DataBoundsLaser::SetMaximumScalar(float s)
  { maximum.SetAttribute(ScalarTag, s);  max_is_set.SetAttribute(ScalarTag, (float) 1.0); }

/*
--------------------------------------------------------------------------------
                         Inquire after bound setting
--------------------------------------------------------------------------------
*/

bool DataBoundsLaser::MinimumIsSet(const LaserPointTag tag) const
{
  // First check on coordinates
  switch (tag) {
    case XCoordinateTag: return MinimumXIsSet();
    case YCoordinateTag: return MinimumYIsSet();
    case ZCoordinateTag: return MinimumZIsSet();
    default: break; // Do nothing
  }
  
  // Split out to attribute type
  switch (AttributeType(tag)) {
    default:
    case IntegerAttributeType: return min_is_set.Attribute(tag) == 1;
    case FloatAttributeType:   return min_is_set.FloatAttribute(tag) == 1.0;
    case DoubleAttributeType:  return min_is_set.DoubleAttribute(tag) == 1.0;
  }
  return 0;
}
    
bool DataBoundsLaser::MaximumIsSet(const LaserPointTag tag) const
{
  // First check on coordinates
  switch (tag) {
    case XCoordinateTag: return MaximumXIsSet();
    case YCoordinateTag: return MaximumYIsSet();
    case ZCoordinateTag: return MaximumZIsSet();
    default: break; // Do nothing
  }
  
  // Split out to attribute type
  switch (AttributeType(tag)) {
    default:
    case IntegerAttributeType: return max_is_set.Attribute(tag) == 1;
    case FloatAttributeType:   return max_is_set.FloatAttribute(tag) == 1.0;
    case DoubleAttributeType:  return max_is_set.DoubleAttribute(tag) == 1.0;
  }
  return 0;
}
    
bool DataBoundsLaser::MinimumXIsSet() const {return min_is_set.X() == 1.0;}
bool DataBoundsLaser::MinimumYIsSet() const {return min_is_set.Y() == 1.0;}
bool DataBoundsLaser::MinimumZIsSet() const {return min_is_set.Z() == 1.0;}
bool DataBoundsLaser::MinimumReflectanceIsSet() const
{ return min_is_set.Attribute(ReflectanceTag) == 1; }
bool DataBoundsLaser::MinimumColourIsSet() const
{ return min_is_set.Attribute(ColourTag) == 1; }
bool DataBoundsLaser::MinimumPulseCountIsSet() const
{ return min_is_set.Attribute(PulseCountTag) == 1; }
bool DataBoundsLaser::MinimumPulseLengthIsSet() const
{ return min_is_set.Attribute(PulseLengthTag) == 1; }
bool DataBoundsLaser::MinimumLabelIsSet() const
{ return min_is_set.Attribute(LabelTag) == 1; }

bool DataBoundsLaser::MaximumXIsSet() const {return max_is_set.X() == 1.0;}
bool DataBoundsLaser::MaximumYIsSet() const {return max_is_set.Y() == 1.0;}
bool DataBoundsLaser::MaximumZIsSet() const {return max_is_set.Z() == 1.0;}
bool DataBoundsLaser::MaximumReflectanceIsSet() const
{ return max_is_set.Attribute(ReflectanceTag) == 1; }
bool DataBoundsLaser::MaximumColourIsSet() const
{ return max_is_set.Attribute(ColourTag) == 1; }
bool DataBoundsLaser::MaximumPulseCountIsSet() const
{ return max_is_set.Attribute(PulseCountTag) == 1; }
bool DataBoundsLaser::MaximumPulseLengthIsSet() const
{ return max_is_set.Attribute(PulseLengthTag) == 1; }
bool DataBoundsLaser::MaximumLabelIsSet() const
{ return max_is_set.Attribute(LabelTag) == 1; }

bool DataBoundsLaser::XYBoundsSet() const
{
  return(MinimumXIsSet() && MaximumXIsSet() &&
         MinimumYIsSet() && MaximumYIsSet());
}

bool DataBoundsLaser::XYZBoundsSet() const
{
  return(MinimumXIsSet() && MaximumXIsSet() &&
         MinimumYIsSet() && MaximumYIsSet() &&
         MinimumZIsSet() && MaximumZIsSet());
}

bool DataBoundsLaser::ReflectanceBoundsSet() const
{
  return(MinimumReflectanceIsSet() && MaximumReflectanceIsSet());
}

bool DataBoundsLaser::ColourBoundsSet() const
{
  return(MinimumColourIsSet() && MaximumColourIsSet());
}

bool DataBoundsLaser::PulseCountBoundsSet() const
{
  return(MinimumPulseCountIsSet() && MaximumPulseCountIsSet());
}

bool DataBoundsLaser::PulseLengthBoundsSet() const
{
  return(MinimumPulseLengthIsSet() && MaximumPulseLengthIsSet());
}

bool DataBoundsLaser::LabelBoundsSet() const
{
  return(MinimumLabelIsSet() && MaximumLabelIsSet());
}

/*
--------------------------------------------------------------------------------
                       Read bounds from file in BNF format
--------------------------------------------------------------------------------
*/

void DataBoundsLaser::Read(FILE *fd)
{
  Read(fd, "enddatabounds");
}

void DataBoundsLaser::Read(FILE *fd, const char *endkeyword)
{
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "xmin", MAX(keyword_length, 4)))
          SetMinimumX(BNF_Double(line));

        else if (!strncmp(keyword, "ymin", MAX(keyword_length, 4)))
          SetMinimumY(BNF_Double(line));

        else if (!strncmp(keyword, "zmin", MAX(keyword_length, 4)))
          SetMinimumZ(BNF_Double(line));

        else if (!strncmp(keyword, "rmin", MAX(keyword_length, 4)))
          SetMinimumReflectance(BNF_Integer(line));

        else if (!strncmp(keyword, "Rmin", MAX(keyword_length, 4)))
          SetMinimumColour(BNF_Integer(line), minimum.Green(), minimum.Blue());

        else if (!strncmp(keyword, "Gmin", MAX(keyword_length, 4)))
          SetMinimumColour(minimum.Red(), BNF_Integer(line), minimum.Blue());

        else if (!strncmp(keyword, "Bmin", MAX(keyword_length, 4)))
          SetMinimumColour(minimum.Red(), minimum.Green(), BNF_Integer(line));

        else if (!strncmp(keyword, "pcmin", MAX(keyword_length, 5)))
          SetMinimumPulseCount(BNF_Integer(line));

        else if (!strncmp(keyword, "plmin", MAX(keyword_length, 5)))
          SetMinimumPulseLength(BNF_Integer(line));

        else if (!strncmp(keyword, "lmin", MAX(keyword_length, 4)))
          SetMinimumLabel(BNF_Integer(line));

	    else if (!strncmp(keyword, "xmax", MAX(keyword_length, 4)))
          SetMaximumX(BNF_Double(line));

        else if (!strncmp(keyword, "ymax", MAX(keyword_length, 4)))
          SetMaximumY(BNF_Double(line));

        else if (!strncmp(keyword, "zmax", MAX(keyword_length, 4)))
          SetMaximumZ(BNF_Double(line));

        else if (!strncmp(keyword, "rmax", MAX(keyword_length, 4)))
          SetMaximumReflectance(BNF_Integer(line));

        else if (!strncmp(keyword, "Rmax", MAX(keyword_length, 4)))
          SetMaximumColour(BNF_Integer(line), maximum.Green(), maximum.Blue());

        else if (!strncmp(keyword, "Gmax", MAX(keyword_length, 4)))
          SetMaximumColour(maximum.Red(), BNF_Integer(line), maximum.Blue());

        else if (!strncmp(keyword, "Bmax", MAX(keyword_length, 4)))
          SetMaximumColour(maximum.Red(), maximum.Green(), BNF_Integer(line));

        else if (!strncmp(keyword, "pcmax", MAX(keyword_length, 5)))
          SetMaximumPulseCount(BNF_Integer(line));

        else if (!strncmp(keyword, "plmax", MAX(keyword_length, 5)))
          SetMaximumPulseLength(BNF_Integer(line));

        else if (!strncmp(keyword, "lmax", MAX(keyword_length, 4)))
          SetMaximumLabel(BNF_Integer(line));

        else if (!strncmp(keyword, endkeyword,
                          MAX(keyword_length, strlen(endkeyword)))) {
          free(buffer);
          return;
        }
      else
        fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword %s.\n", endkeyword);
}

/*
--------------------------------------------------------------------------------
                       Write bounds to file in BNF format
--------------------------------------------------------------------------------
*/

int DataBoundsLaser::HasSomeData() const
{
  const unsigned char *tag;
  int                 i;

  // Check coordinates
  if (MinimumXIsSet() || MaximumXIsSet() ||
      MinimumYIsSet() || MaximumYIsSet() ||
      MinimumZIsSet() || MaximumZIsSet()) return true;
  // Check attributes
  for (i=0, tag=min_is_set.AttributeTags(); i<min_is_set.NumAttributes();
       i++, tag++)
    if (min_is_set.Attribute((const LaserPointTag) *tag) == 1) return true;
  for (i=0, tag=max_is_set.AttributeTags(); i<max_is_set.NumAttributes();
       i++, tag++)
    if (max_is_set.Attribute((const LaserPointTag) *tag) == 1) return true;
  return false;
}

void DataBoundsLaser::Write(FILE *fd, int indent, int colour) const
{
  Write(fd, indent, "databounds", colour);
}

void DataBoundsLaser::Write(FILE *fd, int indent, const char *boundstype,
                            int colour) const
{
  char *endstring;

  if (!HasSomeData()) return;
  BNF_Write_String(fd, boundstype, indent, NULL);
  if (MinimumXIsSet())
    BNF_Write_Double(fd, "xmin", indent+2, minimum.X(), "%15.3f");
  if (MinimumYIsSet())
    BNF_Write_Double(fd, "ymin", indent+2, minimum.Y(), "%15.3f");
  if (MinimumZIsSet())
    BNF_Write_Double(fd, "zmin", indent+2, minimum.Z(), "%15.3f");
  if (MinimumReflectanceIsSet() || MinimumColourIsSet()) {
    if (colour) {
      BNF_Write_Integer(fd, "Rmin", indent+2, minimum.Red(), "%11d");
      BNF_Write_Integer(fd, "Gmin", indent+2, minimum.Green(), "%11d");
      BNF_Write_Integer(fd, "Bmin", indent+2, minimum.Blue(), "%11d");
    }
    else
      BNF_Write_Integer(fd, "rmin", indent+2, minimum.Reflectance(), "%11d");
  }
  if (MinimumPulseCountIsSet())
    BNF_Write_Integer(fd, "pcmin", indent+2, minimum.PulseCount(), "%10d");
  if (MinimumPulseLengthIsSet())
    BNF_Write_Integer(fd, "plmin", indent+2, minimum.PulseLength(), "%10d");
  if (MinimumLabelIsSet())
    BNF_Write_Integer(fd, "lmin", indent+2, minimum.Label(), "%11d");
  if (MaximumXIsSet())
    BNF_Write_Double(fd, "xmax", indent+2, maximum.X(), "%15.3f");
  if (MaximumYIsSet())
    BNF_Write_Double(fd, "ymax", indent+2, maximum.Y(), "%15.3f");
  if (MaximumZIsSet())
    BNF_Write_Double(fd, "zmax", indent+2, maximum.Z(), "%15.3f");
  if (MaximumReflectanceIsSet() || MaximumColourIsSet()) {
    if (colour) {
      BNF_Write_Integer(fd, "Rmax", indent+2, maximum.Red(), "%11d");
      BNF_Write_Integer(fd, "Gmax", indent+2, maximum.Green(), "%11d");
      BNF_Write_Integer(fd, "Bmax", indent+2, maximum.Blue(), "%11d");
    }
    else
      BNF_Write_Integer(fd, "rmax", indent+2, maximum.Reflectance(), "%11d");
  }
  if (MaximumPulseCountIsSet())
    BNF_Write_Integer(fd, "pcmax", indent+2, maximum.PulseCount(), "%10d");
  if (MaximumPulseLengthIsSet())
    BNF_Write_Integer(fd, "plmax", indent+2, maximum.PulseLength(), "%10d");
  if (MaximumLabelIsSet())
    BNF_Write_Integer(fd, "lmax", indent+2, maximum.Label(), "%11d");
  endstring = (char *) malloc(strlen(boundstype) + 4);
  sprintf(endstring, "end%s", boundstype);
  BNF_Write_String(fd, endstring, indent, NULL);
  free(endstring);
}

/*
--------------------------------------------------------------------------------
                       Print bounds to a file
--------------------------------------------------------------------------------
*/

void DataBoundsLaser::Print() const {Print(stdout);}

void DataBoundsLaser::Print(FILE *fd) const
{
  fprintf(fd, "  Element           Minimum value         Maximum value\n");
  fprintf(fd, "-----------         -------------         -------------\n");

  fprintf(fd, "     X");
  if (MinimumXIsSet()) fprintf(fd, "%26.2f", minimum.X());
  else fprintf(fd, "                          ");
  if (MaximumXIsSet()) fprintf(fd, "%22.2f\n", maximum.X());
  else fprintf(fd, "\n");

  fprintf(fd, "     Y");
  if (MinimumYIsSet()) fprintf(fd, "%26.2f", minimum.Y());
  else fprintf(fd, "                          ");
  if (MaximumYIsSet()) fprintf(fd, "%22.2f\n", maximum.Y());
  else fprintf(fd, "\n");

  fprintf(fd, "     Z");
  if (MinimumZIsSet()) fprintf(fd, "%26.2f", minimum.Z());
  else fprintf(fd, "                          ");
  if (MaximumZIsSet()) fprintf(fd, "%22.2f\n", maximum.Z());
  else fprintf(fd, "\n");

  fprintf(fd, "Minimum attribute values\n");
  minimum.PrintAttributes(fd);
  
  fprintf(fd, "Maximum attribute values\n");
  maximum.PrintAttributes(fd);
}

/*
--------------------------------------------------------------------------------
                       Extract bounds from a meta data file
--------------------------------------------------------------------------------
*/

int DataBoundsLaser::Extract(const char *filename)
{
  FILE *fd;
  char *buffer, *line, *keyword;
  int  keyword_length;

/* Open the file */

  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening meta data file %s for reading bounds\n",
            filename);
    return(0);
  }

/* Scan the file for the keyword "bounds" and read the bounds */

  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "databounds", MAX(keyword_length, 10))) {
          Read(fd);
          free(buffer);  fclose(fd);
          return(1);
        }
      }
    }
  }

/* Keyword bounds not found */

  free(buffer);  fclose(fd);
  fprintf(stderr, "Error: no bounds in file %s\n", filename);
  return(0);
}

/*
--------------------------------------------------------------------------------
                   Return the range of coordinate or attributes
--------------------------------------------------------------------------------
*/

double DataBoundsLaser::XRange() const
{
  if (MinimumXIsSet() && MaximumXIsSet()) return(maximum.X() - minimum.X());
  else return(0.0);
}

double DataBoundsLaser::YRange() const
{
  if (MinimumYIsSet() && MaximumYIsSet()) return(maximum.Y() - minimum.Y());
  else return(0.0);
}

double DataBoundsLaser::ZRange() const
{
  if (MinimumZIsSet() && MaximumZIsSet()) return(maximum.Z() - minimum.Z());
  else return(0.0);
}

double DataBoundsLaser::MaxRange() const
{
	return std::max(XRange(),std::max(YRange(),ZRange()));
}

int DataBoundsLaser::ReflectanceRange() const
{
  if (MinimumReflectanceIsSet() && MaximumReflectanceIsSet())
    return(maximum.Reflectance() - minimum.Reflectance());
  else
    return(0);
}

int DataBoundsLaser::PulseCountRange() const
{
  if (MinimumPulseCountIsSet() && MaximumPulseCountIsSet())
    return(maximum.PulseCount() - minimum.PulseCount());
  else
    return(0);
}

int DataBoundsLaser::PulseLengthRange() const
{
  if (MinimumPulseLengthIsSet() && MaximumPulseLengthIsSet())
    return(maximum.PulseLength() - minimum.PulseLength());
  else
    return(0);
}

int DataBoundsLaser::LabelRange() const
{
  if (MinimumLabelIsSet() && MaximumLabelIsSet())
    return(maximum.Label() - minimum.Label());
  else
    return(0);
}

/*
--------------------------------------------------------------------------------
                            Update the bounds
--------------------------------------------------------------------------------
*/

DataBoundsLaser DataBoundsLaser::Update(const LaserPoint &point, bool useAtts)
{
  const unsigned char *tag;
  int                 i;
  int                 ivalue;
  float               fvalue;
  double              dvalue;

  // Coordinates
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

  if (useAtts) {
	  

  // Colour
  if (point.HasAttribute(ColourTag)) {
    if (MinimumColourIsSet()) {
      SetMinimumColour(MIN(minimum.Red(), point.Red()),
                       minimum.Green(), minimum.Blue());
      SetMinimumColour(minimum.Red(), MIN(minimum.Green(), point.Green()),
                       minimum.Blue());
      SetMinimumColour(minimum.Red(), minimum.Green(),
                       MIN(minimum.Blue(), point.Blue()));
    }
    else SetMinimumColour(point.Red(), point.Green(), point.Blue());
    if (MaximumColourIsSet()) {
      SetMaximumColour(MAX(maximum.Red(), point.Red()),
                       maximum.Green(), maximum.Blue());
      SetMaximumColour(maximum.Red(), MAX(maximum.Green(), point.Green()),
                       maximum.Blue());
      SetMaximumColour(maximum.Red(), maximum.Green(),
                       MAX(maximum.Blue(), point.Blue()));
    }
    else SetMaximumColour(point.Red(), point.Green(), point.Blue());
  }

  // Other attributes
  for (i=0, tag=point.AttributeTags(); i<point.NumAttributes(); i++, tag++) {
    if (*tag != ColourTag && *tag < NoTag) {
      switch (AttributeType(*tag)) {
      	default:
      	case IntegerAttributeType:
      	  ivalue = point.Attribute(*tag);
          if (minimum.HasAttribute(*tag))
            SetMinimumValue(*tag, MIN(minimum.Attribute(*tag), ivalue));
          else
            SetMinimumValue(*tag, ivalue);
          if (maximum.HasAttribute(*tag))
		    SetMaximumValue(*tag, MAX(maximum.Attribute(*tag), ivalue));
          else
            SetMaximumValue(*tag, ivalue);
          break;
          
        case FloatAttributeType:
      	  fvalue = point.FloatAttribute(*tag);
          if (minimum.HasAttribute(*tag))
            SetMinimumValue(*tag, MIN(minimum.FloatAttribute(*tag), fvalue));
          else
            SetMinimumValue(*tag, fvalue);
          if (maximum.HasAttribute(*tag))
		    SetMaximumValue(*tag, MAX(maximum.FloatAttribute(*tag), fvalue));
          else
            SetMaximumValue(*tag, fvalue);
          break;
          
        case DoubleAttributeType:
      	  dvalue = point.DoubleAttribute(*tag);
          if (minimum.HasAttribute(*tag))
            SetMinimumValue(*tag, MIN(minimum.DoubleAttribute(*tag), dvalue));
          else
            SetMinimumValue(*tag, dvalue);
          if (maximum.HasAttribute(*tag))
		    SetMaximumValue(*tag, MAX(maximum.DoubleAttribute(*tag), dvalue));
          else
            SetMaximumValue(*tag, dvalue);
          break;
      }
    }
  }
    }

  return(*this);
}

DataBoundsLaser DataBoundsLaser::Update(const DataBoundsLaser &b)
{
  Update(b.Minimum());
  return Update(b.Maximum());
}

/*
--------------------------------------------------------------------------------
              Define a grids for imaged data
--------------------------------------------------------------------------------
*/

DataGrid *DataBoundsLaser::HeightDataGrid(double pixelsize) const
{
  DataGrid *grid;

  grid = new DataGrid(minimum.X(), maximum.Y(), pixelsize,
		      minimum.Z(), (maximum.Z() - minimum.Z())/255.0);
  return(grid);
}

DataGrid *DataBoundsLaser::ReflectanceDataGrid(double pixelsize) const
{
  DataGrid *grid;

  grid = new DataGrid(minimum.X(), maximum.Y(), pixelsize,
		      minimum.Reflectance(),
		      (maximum.Reflectance() - minimum.Reflectance())/255.0);
  return(grid);
}

DataGrid *DataBoundsLaser::ColourDataGrid(double pixelsize) const
{
  DataGrid *grid;
  double   min_col, max_col;

  min_col = MIN(MIN(minimum.Red(), minimum.Green()), minimum.Blue());
  max_col = MAX(MAX(maximum.Red(), maximum.Green()), maximum.Blue());
  grid = new DataGrid(minimum.X(), maximum.Y(), pixelsize,
		      min_col, (max_col - min_col)/255.0);
  return(grid);
}

DataGrid *DataBoundsLaser::PulseLengthDataGrid(double pixelsize) const
{
  DataGrid *grid;

  grid = new DataGrid(minimum.X(), maximum.Y(), pixelsize,
		      minimum.PulseLength(),
		      (maximum.PulseLength() - minimum.PulseLength())/255.0);
  return(grid);
}

DataGrid *DataBoundsLaser::PointCountDataGrid(double pixelsize) const
{
  DataGrid *grid;

  grid = new DataGrid(minimum.X(), maximum.Y(), pixelsize, 0.0, 1.0);
  return(grid);
}

/*
--------------------------------------------------------------------------------
                     Check if a point is within the bounds
--------------------------------------------------------------------------------
*/

bool DataBoundsLaser::InsideXY(const LaserPoint *point) const
{
  if (MinimumXIsSet())
    if (point->X() < minimum.X()) return false;
  if (MaximumXIsSet())
    if (point->X() >= maximum.X()) return false;
  if (MinimumYIsSet())
    if (point->Y() < minimum.Y()) return false;
  if (MaximumYIsSet())
    if (point->Y() >= maximum.Y()) return false;
  return true;
}

bool DataBoundsLaser::InsideAttributes(const LaserPoint *point) const
{
  const unsigned char *tag;
  int                 i, loop, num_tags;
  int                 value;
  float               fvalue;
  double              dvalue;

  // Loop for minimum and maximum checks
  for (loop=0; loop<2; loop++) {
  	// Check minima in first loop, maxima in second
  	switch (loop) {
  	  default:
  	  case 0:
  	  	tag = minimum.AttributeTags();
  	  	num_tags = minimum.NumAttributes();
  	  	break;
  	  case 1:
  	  	tag = maximum.AttributeTags();
  	  	num_tags = maximum.NumAttributes();
  	  	break;
  	}
    // Loop over all attributes extremes
    for (i=0; i<num_tags; i++, tag++) {
      if (*tag >= NoTag) continue;
      
      // Not inside if the point does not have this attribute
      if (!point->HasAttribute(*tag)) return false;
      
      // Special case for colour checking
      if (*tag != ColourTag) {
        switch (AttributeType(*tag)) {
      	  case IntegerAttributeType:
      	    value = point->Attribute(*tag);
            if (minimum.HasAttribute(*tag))
              if (value < minimum.Attribute(*tag)) return false;
            if (maximum.HasAttribute(*tag))
              if (value > maximum.Attribute(*tag)) return false;
            break;
          
      	  case FloatAttributeType:
      	    fvalue = point->FloatAttribute(*tag);
            if (minimum.HasAttribute(*tag))
              if (fvalue < minimum.FloatAttribute(*tag)) return false;
            if (maximum.HasAttribute(*tag))
              if (fvalue > maximum.FloatAttribute(*tag)) return false;
            break;
          
      	  case DoubleAttributeType:
      	    dvalue = point->DoubleAttribute(*tag);
            if (minimum.HasAttribute(*tag))
              if (dvalue < minimum.DoubleAttribute(*tag)) return false;
            if (maximum.HasAttribute(*tag))
              if (dvalue > maximum.DoubleAttribute(*tag)) return false;
            break;
        }
      }
      else {
        if (minimum.HasAttribute(ColourTag)) {
          if (point->Red()   < minimum.Red()) return false;
          if (point->Green() < minimum.Green()) return false;
          if (point->Blue()  < minimum.Blue()) return false;
        }
        if (maximum.HasAttribute(ColourTag)) {
          if (point->Red()   > maximum.Red()) return false;
          if (point->Green() > maximum.Green()) return false;
          if (point->Blue()  > maximum.Blue()) return false;
        }
      }
    }
  }

  return true;
}

bool DataBoundsLaser::Inside(const LaserPoint *point) const
{
  const unsigned char *tag;
  int                 i;
  const int           *value;

  // Check coordinates
  if (!InsideXY(point)) return false;
  if (MinimumZIsSet())
    if (point->Z() < minimum.Z()) return false;
  if (MaximumZIsSet())
    if (point->Z() >= maximum.Z()) return false;
    
  // Check attributes
  return InsideAttributes(point);
}

/*
--------------------------------------------------------------------------------
                Check if two data bounds overlap in planimetry
--------------------------------------------------------------------------------
*/

bool OverlapXY(const DataBoundsLaser &bounds1, const DataBoundsLaser &bounds2)
{
  if (bounds1.MinimumXIsSet() && bounds2.MaximumXIsSet() &&
      bounds1.Minimum().X() > bounds2.Maximum().X()) return false;
  if (bounds1.MaximumXIsSet() && bounds2.MinimumXIsSet() &&
      bounds1.Maximum().X() < bounds2.Minimum().X()) return false;
  if (bounds1.MinimumYIsSet() && bounds2.MaximumYIsSet() &&
      bounds1.Minimum().Y() > bounds2.Maximum().Y()) return false;
  if (bounds1.MaximumYIsSet() && bounds2.MinimumYIsSet() &&
      bounds1.Maximum().Y() < bounds2.Minimum().Y()) return false;
  return true;
}

/*
--------------------------------------------------------------------------------
                Return the averages of the min/max values
--------------------------------------------------------------------------------
*/

LaserPoint DataBoundsLaser::MidPoint() const
{
  if (Minimum().HasAttribute(ReflectanceTag) &&
      Maximum().HasAttribute(ReflectanceTag) &&
	  Minimum().HasAttribute(PulseCountTag) &&
	  Maximum().HasAttribute(PulseCountTag))	
    return LaserPoint((Minimum().X() + Maximum().X()) / 2.0,
                      (Minimum().Y() + Maximum().Y()) / 2.0,
                      (Minimum().Z() + Maximum().Z()) / 2.0,
                      (Minimum().Reflectance() + Maximum().Reflectance()) / 2,
                      (Minimum().PulseCount() + Maximum().PulseCount()) / 2);
  return LaserPoint((Minimum().X() + Maximum().X()) / 2.0,
                    (Minimum().Y() + Maximum().Y()) / 2.0,
                    (Minimum().Z() + Maximum().Z()) / 2.0);
}

/*
--------------------------------------------------------------------------------
                         Return the 2D and 3D bounds
--------------------------------------------------------------------------------
*/
DataBounds3D &DataBoundsLaser::Bounds3D() const
{
  DataBounds3D *bounds3d = new DataBounds3D();
  bounds3d->Update(Minimum().Position3DRef());
  bounds3d->Update(Maximum().Position3DRef());
  return *bounds3d;
}

DataBounds2D &DataBoundsLaser::Bounds2D() const
{
  DataBounds2D *bounds2d = new DataBounds2D();
  bounds2d->Update(Minimum().Position2DOnly());
  bounds2d->Update(Maximum().Position2DOnly());
  return *bounds2d;
}
