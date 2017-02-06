
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



/* TODO (#1#): Fix bug in copy constructor used by push_back in 
               ascii2laser */
/*!
 * \file
 * \brief Interface to Class LaserPoint - Single laser altimetry point
 *
 */
/*!
 * \class LaserPoint
 * \ingroup LPointSets
 * \brief Interface to Class LaserPoint - Single laser altimetry point
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */



#ifndef _LaserPoint_h_
#define _LaserPoint_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserPoint                              - Single laser altimetry point

--------------------------------------------------------------------------------
*/
#include <ostream>
#include "Position3D.h"
#include "ImagePoint.h"
#include "Image.h"
#include "DataGrid.h"
#include "Histogram.h"
#include "ObjectPoint.h"
#include "AttributeTypes.h"

class ObjectPoints2D;
class ObjectPoints;
class PointNumberList;
class LaserPoints;
class Plane;

enum LaserPointTag { ReflectanceTag, PulseCountTag, LabelTag, IsFilteredTag,
                     IsProcessedTag, ColourTag, ResidualTag, IsSelectedTag,
                     PlaneNumberTag, SegmentNumberTag, 
                     ScanNumberTag, PointNumberTag, //Added by Tahir 
                     PulseLengthTag, PolygonNumberTag, 
                     IntensityTag, //Added by Shi Pu
                     PulseCountWithFlagTag,
                     v_xTag, v_yTag, v_zTag, cv_xyTag, cv_xzTag, cv_yzTag, //Added by Markus Gerke: 6 unique covariance parameters
		             NumRaysTag,//Added by Markus Gerke: number of rays from forward intersection
		             ScalarTag, // For sorting points along a line
		             TimeTag,   // Time of reflecting pulse
		             AngleTag,  // Scan mirror angle
		             XCoordinateTag, YCoordinateTag, ZCoordinateTag, // Just to access coordinates through tags
		             ComponentNumberTag,
		             ScanNumberWithoutAFNTag, // Scan number without FLI-MAP AFN code (=ScanNumber/10)
		             AFNTag, // FLI-MAP AFN code (=last digit of ScanNumber)
		             NormalXTag, NormalYTag, NormalZTag, // Normal vector components
		             FlatnessTag, LinearityTag, // Based on eigenvalue ratios, either local or segment based
		             // Normal vector scaled by flatness
		             ScaledNormalXTag, ScaledNormalYTag, ScaledNormalZTag,
		             SegmentStartTileNumberTag, // Tile number in which the segment growing started
		             RedTag, GreenTag, BlueTag, // To access the components of ColourTag
		             LongSegmentNumberTag, // To check for both SegmentNumberTag and SegmentStartTileNumberTag
		             HeightAboveGroundTag,
		             ConvexHullTag, // To mark points on the convex hull of the TIN (0=interior, 1=hull)
		             NearOtherSegmentTag, // 47 To mark points close to segment border (0=false, 1=true)
		             
		             ScanLineNumberTag=50,  // Why 50? GV
		             Lambda0LocalTag, Lambda1LocalTag, Lambda2LocalTag, // Eigenvalues in local neighbourhood
		             FlatnessLocalTag, LinearityLocalTag, // Based on eigenvalues in local neighbourhood
		             Lambda0LocalScaledTag, Lambda1LocalScaledTag, Lambda2LocalScaledTag, // Scaled eigenvalues in local neighbourhood
		             
                     // Attributes of segments, starting at index 100
                     // Averages
		             AverageReflectanceTag=100, AveragePulseCountTag,
		             AverageIsFilteredTag, AverageIsProcessedTag,
		             AverageColourTag, AverageResidualTag, AverageIsSelectedTag,
		             AveragePulseLengthTag, AverageScalarTag,
		             AverageTimeTag, AverageAngleTag,
		             AverageXCoordinateTag, AverageYCoordinateTag,
		             AverageZCoordinateTag,
		             AverageHeightAboveGroundTag,
		             AveragePointSpacingTag, // based on TIN edges within segment
		             AverageLambda0LocalTag, AverageLambda1LocalTag, AverageLambda2LocalTag,
		             AverageFlatnessLocalTag, AverageLinearityLocalTag,
		             AverageLambda0LocalScaledTag, AverageLambda1LocalScaledTag, AverageLambda2LocalScaledTag,
		             
		             // Variations of attribute values within segments, starting at 130
		             SlopeAngleVarianceTag=130,
		             Sum1Tag=138, Sum2Tag, // Needed for variance computations
		             
		             // Minima of attributes within segment, starting at index 140
		             MinReflectanceTag=140, MinPulseCountTag, MinLabelTag,
		             MinResidualTag, MinPlaneNumberTag, 
		             MinScanNumberTag, MinPulseLengthTag, MinPolygonNumberTag,
		             MinScalarTag, MinTimeTag, MinAngleTag,
		             MinXCoordinateTag, MinYCoordinateTag, MinZCoordinateTag,
		             MinHeightAboveGroundTag,
					 
					 PolygonIDTag=160,
		             
		             // Maxima of attributes within segment, starting at index 180
		             MaxReflectanceTag=180, MaxPulseCountTag, MaxLabelTag,
		             MaxResidualTag, MaxPlaneNumberTag, 
		             MaxScanNumberTag, MaxPulseLengthTag, MaxPolygonNumberTag,
		             MaxScalarTag, MaxTimeTag, MaxAngleTag,
		             MaxXCoordinateTag, MaxYCoordinateTag, MaxZCoordinateTag,
		             MaxHeightAboveGroundTag,
		             
		             // Several counts, starting at index 218
		             EdgeCountTag=218,
					 ComponentSizeTag, SegmentSizeTag, 
		             
		             // Percentage attributes, starting at index 221
		             PercFirstPulseTag=221, PercSecondPulseTag,
                     PercThirdPulseTag, PercFourthPulseTag, PercLastPulseTag,
                     PercNotFirstPulseTag,
                     PercNotLastPulseTag, PercMultiPulseTag, PercIsFilteredTag,
                     PercIsProcessedTag, PercIsSelectedTag,
					 PercNearOtherSegmentTag,
                     
                     // Plane attributes, starting at index 240
                     InclinationTag=240, AzimuthTag, 
					 Lambda0Tag, Lambda1Tag, Lambda2Tag, // Eigenvalues of all points in a segment
					 Lambda0ScaledTag, Lambda1ScaledTag, Lambda2ScaledTag, // Scaled eigenvalues of all points in a segment
		             
		             NoTag=253, // To indicate that no attribute has been specified
					 DoubleTag=254, // Tag for second part of a double value
                     UndefinedTag=255};
typedef enum LaserPointTag LaserPointTag;

typedef GenericAttributeType LaserPointAttributeType;

/// Determine the type of a laser point attribute
extern LaserPointAttributeType AttributeType(const LaserPointTag);

/// Return the name of a laser point attribute
extern const char * AttributeName(const LaserPointTag, bool);

/// Check whether an attribute tag is defined
extern bool KnownAttribute(const LaserPointTag);

/// Determine the corresponding point attribute of a segment attribute
extern LaserPointTag BasicLaserPointTag(LaserPointTag);

enum LaserPulseType { AnyPulse, FirstPulse, SecondPulse, ThirdPulse,
                      FourthPulse, FifthPulse, LastPulse=11, NotFirstPulse,
                      NotLastPulse, SinglePulse, MultiplePulse,
                      FirstNorLastPulse };
typedef enum LaserPulseType LaserPulseType;

//------------------------------------------------------------------------------
///                         Laser altimetry point
//------------------------------------------------------------------------------

class LaserPoint : public Position3D
{
  protected:

    /// Number of attributes
    unsigned char num_attributes;

    /// Vector of attribute tags
    unsigned char *tags;

    /// Vector of attribute values
    int *attributes;

// Functions defined in the file LaserPoint.cc 

  private:
    // Retrieve index of an existing attribute tag
    unsigned char ExistingAttributeIndex(const LaserPointTag tag) const;

    // Retrieve index of an attribute tag, create one if it does not yet exist
    unsigned char AttributeIndex(const LaserPointTag tag, bool double_value=false);

  public:
    /// Default constructor
    LaserPoint() : Position3D()
      { tags = NULL; attributes = NULL; Initialise(); }

    /// Construct point with reflectance and pulse count
    LaserPoint(double x, double y, double z, int r, int pc);

    /// Construct point with reflectance
    LaserPoint(double x, double y, double z, int r);

    /// Construct point without reflectance or pulse count
    LaserPoint(double x, double y, double z);
    
    /// Copy constructor
    LaserPoint(const LaserPoint &pt);
	
	/// Copy constructor
    LaserPoint(const Vector3D &pt);

    /// Default destructor
    ~LaserPoint();

    /// Copy assignment
    LaserPoint & operator = (const LaserPoint &pt);
	
	///assignment from Vector3D
    LaserPoint & operator = (const Vector3D &pt);
	
    /// Initialise a point
    void Initialise();

    /// Retrieve readable attribute value
    int Attribute(const LaserPointTag tag) const;

    /// Retrieve readable attribute value
    int Attribute(const int tag) const
      { return Attribute((const LaserPointTag) tag); }

    /// Retrieve writable attribute value
    int & Attribute(const LaserPointTag tag);

    /// Retrieve writable attribute value
    int & Attribute(const int tag)
      { return Attribute((const LaserPointTag) tag); }

    /// Retrieve readable float attribute value
    float FloatAttribute(const LaserPointTag tag) const;

    /// Retrieve readable float attribute value
    float FloatAttribute(const int tag) const
      { return FloatAttribute((const LaserPointTag) tag); }

    /// Retrieve writable float attribute value
    float & FloatAttribute(const LaserPointTag tag);

    /// Retrieve writable float attribute value
    float & FloatAttribute(const int tag)
      { return FloatAttribute((const LaserPointTag) tag); }

    /// Retrieve readable double attribute value
    double DoubleAttribute(const LaserPointTag tag) const;

    /// Retrieve readable double attribute value
    double DoubleAttribute(const int tag) const
      { return DoubleAttribute((const LaserPointTag) tag); }

    /// Retrieve writable double attribute value
    double & DoubleAttribute(const LaserPointTag tag);

    /// Retrieve writable double attribute value
    double & DoubleAttribute(const int tag)
      { return DoubleAttribute((const LaserPointTag) tag); }

    /// Retrieve readable long long int attribute value
    long long int LongAttribute(const LaserPointTag tag) const;

    /// Retrieve readable long long int attribute value
    long long int LongAttribute(const int tag) const
      { return LongAttribute((const LaserPointTag) tag); }

    /// Check if an attribute is present
    bool HasAttribute(const LaserPointTag tag) const;

    /// Check if an attribute value is present
    bool HasAttributeValue(const LaserPointTag tag, int value) const;

    /// Check if an attribute value is present
    bool HasAttribute(const int tag) const
      { return HasAttribute((const LaserPointTag) tag); }

    /// Check if an attribute has a label
    bool HasLabel() const
      { return HasAttribute(LabelTag); }

    /// Set an attribute value
    void SetAttribute(const LaserPointTag tag, const int value);

    /// Set an attribute value
    void SetAttribute(const int tag, const int value)
      { SetAttribute((const LaserPointTag) tag, value); }

    /// Set a float attribute value
    void SetAttribute(const LaserPointTag tag, const float value);

    /// Set a float attribute value
    void SetAttribute(const int tag, const float value)
      { SetAttribute((const LaserPointTag) tag, value); }

    /// Set a double attribute value
    void SetDoubleAttribute(const LaserPointTag tag, const double value);

    /// Set a double attribute value
    void SetDoubleAttribute(const int tag, const double value)
      { SetDoubleAttribute((const LaserPointTag) tag, value); }

    /// Set a long long int attribute value
    void SetLongAttribute(const LaserPointTag tag, const long long int value);

    /// Set a long long int attribute value
    void SetLongAttribute(const int tag, const long long int value)
      { SetLongAttribute((const LaserPointTag) tag, value); }

    /// Remove an attribute
    bool RemoveAttribute(const LaserPointTag tag);

    /// Remove all attributes
    bool RemoveAttributes();

    /// Rename an attribute
    bool RenameAttribute(const LaserPointTag oldtag,
                         const LaserPointTag newtag);
                         
    /// Sort all attributes on tag number
    void SortAttributes();

    /// Return the number of attributes
    unsigned char NumAttributes() const { return num_attributes; }

    // Return attribute tag vector based on index
    const unsigned char * AttributeTags() const { return tags; }

    // Return attribute value vector
    const int * AttributeValues() const { return attributes; }

    /// Retrieve readable reflectance value
    int Reflectance() const;

    /// Retrieve writable reflectance value
    int &Reflectance();

    /// Set reflectance value
    void SetReflectance(int r)
      { Reflectance() = r; }

    /// Retrieve readable pulse count value
    /** Note that both the last pulse flag as well as the actual pulse count
        are stored in this attribute. This function returns the pulse count
        without the last pulse flag.
    */
    int PulseCount() const;

    /// Retrieve wriable pulse count value
    /** Note that both the last pulse flag as well as the actual pulse count
        are stored in this attribute. When using this function the user is
        responsible for setting the last pulse flag in bit 31. Use
        SetLastPulseFlag for this purpose. Otherwise use SetPulseCount instead
        of this function.
    */
    int & PulseCount();

    /// Retrieve readable pulse count value
    /** Note that both the last pulse flag as well as the actual pulse count
        are stored in this attribute. This function returns the pulse count
        with the last pulse flag.
    */
    int PulseCountWithFlag() const;

    /// Retrieve writable pulse count value
    /** Note that both the last pulse flag as well as the actual pulse count
        are stored in this attribute. When using this function the user is
        responsible for setting the last pulse flag in bit 31. Use
        SetLastPulseFlag for this purpose. Otherwise use SetPulseCount instead
        of this function.
    */
    int &PulseCountWithFlag();

    /// Set pulse count value
    void SetPulseCount(int pc, bool last=false)
      { PulseCountWithFlag() = pc; SetLastPulseFlag(last); }

    /// Return the last pulse flag
    bool IsLastPulse() const
      { return ((Attribute(PulseCountWithFlagTag) >> 30) == 1); }
      
    /// Set the last pulse flag
    void SetLastPulseFlag(bool last);

    /// Check if the pulse is of a specific type
    /** If the pulse count is unknown, this function will return true for
        any specified laser pulse type
    */
    bool IsPulseType(LaserPulseType type) const;
    
    /// Red component of the point colour
    int Red() const
      { return(Attribute(ColourTag) >> 16); }

    /// Green component of the point colour
    int Green() const
      { return((Attribute(ColourTag) >> 8) -
               ((Attribute(ColourTag) >> 16) << 8)); }
    
    /// Blue component of the point colour
    int Blue() const
      { return(Attribute(ColourTag)  - ((Attribute(ColourTag) >> 8) << 8)); }

    /// Intensity of a colour point
    int Intensity() const
      { return((Red() + Green() + Blue()) / 3); }

    /// Set the colour of the point 
    /** The values of red, green and blue should be in the range 0-255. */
    void SetColour(int red, int green, int blue)
      { SetAttribute(ColourTag, (red << 16) + (green << 8) + blue); }

	/// Set the scan number of the point 
    void SetScanNumber(int number);
	  
	/// Get the scan number of the point, possibly combined with afn number
    int ScanNumber(bool afn_in_number=false) const;

	/// Get the scan number of the point 
    int GetScanNumber() const
      { return ScanNumber(false); }
	  
	/// Set the point number of the point 
    void SetPointNumber(int number);
	  
	/// Get the point number of the point 
    int GetPointNumber() const;
	  
	/// Return both scan number and point number as a 64-bit integer.
	long long int GetId()const;
	
	///Set both scan id and point id from a given 64-bit integer.
	void SetId(long long int id);
    
    /// Retrieve readable residual value
    float Residual() const;

    /// Retrieve writable residual value
    float & Residual();
    
    /// Retrieve readable pulse length value
    int PulseLength() const;
    
    /// Retrieve writable pulse length value
    int & PulseLength();
    
    /// Set pulse length value
    void SetPulseLength(int pl)
      { PulseLength() = pl; }

    /// Retrieve readable plane number
    int PlaneNumber() const;
    
    /// Retrieve writable plane number
    int & PlaneNumber();
    
    /// Set plane number
    void SetPlaneNumber(int pn)
      { PlaneNumber() = pn; }

    /// Retrieve readable segment number
    int SegmentNumber() const;
    
    /// Retrieve writable segment number
    int & SegmentNumber();
    
    /// Set segment number
    void SetSegmentNumber(int sn)
      { SegmentNumber() = sn; }

    /// Retrieve readable polygon number
    int PolygonNumber() const;
    
    /// Retrieve writable polygon number
    int & PolygonNumber();
    
    /// Set polygon number
    void SetPolygonNumber(int pn)
      { PolygonNumber() = pn; }
      
    /// Retrieve the long segment number (segment number with tile number of origin)
    long long int LongSegmentNumber() const;
    
    /// Set the long segment number (segment number with tile number of origin)
    void SetLongSegmentNumber(long long int number);

    /// Check if a point is selected
    bool IsSelected() const;
    
    /// Select a point
    void Select();
    
    /// Unselect a point
    void UnSelect();

    /// Print laser point data (coordinates and attributes)
    void Print(FILE *fd) const;

    /// Print laser point data (coordinates and attributes)
    void Print() const {Print(stdout);}
    
    /// Print attributes of a point
    void PrintAttributes(FILE *fd) const;

    /// Print attributes of a point
    void PrintAttributes() const {PrintAttributes(stdout);}

    /// Convert laser point to an image point
    ImagePoint Map_Into_Image(int point_number, const ImageGrid &grid) const;

    /// Convert height to a grey value
    unsigned char Height_To_GreyValue(const DataGrid &grid) const;

    /// Convert reflectance to a grey value
    unsigned char Reflectance_To_GreyValue(const DataGrid &grid) const;

    /// Add point to a histogram
    /** @param histogram The histogram to be updated with this point
        @param type Type of histogram: 1 - X-coordinate, 2 - Y-coordinate, 
                    3 - Z-coordinate, 4 - Reflectance, 5 - Pulse count
    */
    void AddToHistogram(Histogram &histogram, int type) const;

    /// Add point to a histogram
    /** @param histogram The histogram to be updated with this point
        @param type Type of histogram: 0 - Attribute
                    1 - X-coordinate, 2 - Y-coordinate, 
                    3 - Z-coordinate
    */

    void AddToHistogram(Histogram &histogram, int type,
                        const LaserPointTag &tag) const;


    /// Construct an object point
    ObjectPoint ConstructObjectPoint(const PointNumber &number,
                                     const Covariance3D &cov) const;

    /// Check if point is inside polygon of laser points 
    /** @param polygon_points Laser points of the polygon
        @param topology Topology of the polygon
        @param skip_polygon_check If true it is not checked whether the topology
                                  represents a correct polygon
        @return 1 if inside polygon, 0 otherwise
    */
    int InsidePolygon(const LaserPoints &polygon_points,
                      const PointNumberList &topology,
                      bool skip_polygon_check=false) const;

    /// Check if point is inside polygon of 2D object points
    /** @param polygon_points 2D object points of the polygon
        @param topology Topology of the polygon
        @param skip_polygon_check If true it is not checked whether the topology
                                  represents a correct polygon
        @return 1 if inside polygon, 0 otherwise
    */
    int InsidePolygon(const ObjectPoints2D &polygon_points,
                      const PointNumberList &topology,
                      bool skip_polygon_check=false) const;

    /// Check if point is inside polygon of 3D object points
    /** @param polygon_points 3D object points of the polygon
        @param topology Topology of the polygon
        @param skip_polygon_check If true it is not checked whether the topology
                                  represents a correct polygon
        @return 1 if inside polygon, 0 otherwise
    */
    int InsidePolygon(const ObjectPoints &polygon_points,
                      const PointNumberList &topology,
                      bool skip_polygon_check=false) const;

    /*a semi-infinite ray horizontally (increasing x, fixed y) out from the test point, 
        and count how many edges it crosses. 
        At each crossing, the ray switches between inside and outside. 
        This is called the Jordan curve theorem.
        More robust than others.
    */
    int InsidePolygonJordan(const ObjectPoints &pts,
                              const PointNumberList &top) const;
                              
   
 	/// Return the readable reference
    const LaserPoint & LaserPointRef() const
      {return(*this);}

    /// Return the writable reference
    LaserPoint & LaserPointRef()
      {return(*this);}

    /// Addition of two laser points (coordinates and attribute values)
    LaserPoint & Add(const LaserPoint &pt) const;

    /// Subtraction of two laser points (coordinates and attribute values)
    LaserPoint & Subtract(const LaserPoint &pt) const;

    /// Multiplication of a laser point by a constant(coordinates and attribute values)
    LaserPoint & Multiply(double d) const;

    /// Division of a laser point by a constant(coordinates and attribute values)
    LaserPoint & Divide(double d) const;

    /// Check if two points are the same
    friend bool operator == (const LaserPoint &pt1, const LaserPoint &pt2);

    /// Check if two points are different
    friend bool operator != (const LaserPoint &pt1, const LaserPoint &pt2);
	
	/// output stream insertion operator
    friend std::ostream& operator << (std::ostream& os, const LaserPoint &pt1);

    /// Return the point's normal if it's stored in the attributes
    //* A null vector is returned if the normal attributes are not available */
    Vector3D Normal() const;
    
    /// Return the point's scaled normal if it's stored in the attributes
    //* A null vector is returned if the normal attributes are not available */
    Vector3D ScaledNormal() const;
    
    /// Store the normal in the point's attributes
    void SetNormal(const Vector3D &normal);
    
    /// Flip normal direction
    void FlipNormal();
    
    /// Flip scaled normal direction
    void FlipScaledNormal();
    
/* Functions defined in the file FilterPoint.cc */

    /// Return the filter state
    bool Filtered() const;

    /// Set point as filtered (i.e. not in DEM)
    void SetFiltered();

    /// Set point as unfiltered (i.e. selected for DEM)
    void SetUnFiltered();

    /// Return process state
    bool Processed() const;

    /// Set point as processed
    void SetProcessed();

    /// Set point as unprocessed
    void SetUnProcessed();

    /// Return point label
    int Label() const;

    /// Store the point label
    void Label(int);

    /// Compare two points based on slope and filter the lowest point
    /** If this point is higher than the neighbouring point by more than the
        maximum slope times the distance between the points and reduced by
        2 sqrt(2) sigma, this points is set to filtered. Vice versa it is also
        checked whether the neighbouring point can be set to filtered.
        Note that the range is only used for the return value.
        @param nb_point Another laser point
        @param range See return value.
        @param slope The assumed maximum terrain slope
        @param sigma Standard deviation of the laser point heights
        @return 1 if the distance between the two points is less than "range",
                0 otherwise.
    */
    int FilterOnSlope(LaserPoint *nb_point, double range, double slope,
                      double sigma);

    /// Compare two points based on slope, but do not filter
    /** If this point is higher than the neighbouring point by more than the
        maximum slope times the distance between the points and reduced by
        2 sqrt(2) sigma, this points would be filtered by the function
        FilterOnSlope. In this case the return value is 1, otherwise this
        function returns 0.
        @param nb_point Another laser point
        @param slope The assumed maximum terrain slope
        @param sigma Standard deviation of the laser point heights
        @return 1 If this laser point can not be a terrain point,
                0 otherwise.
    */
    int WouldBeFilteredOnSlope(const LaserPoint *nb_point, double slope,
                               double stdev) const;

    /// Filter on morphology
    /** For the distance between the two points the maximum allowed difference
        in the terrain heights is extracted from the kernel and compared with
        the actual height difference. The first row of the kernel should
        contain the maximum allowed height differences as a function of the
        distance. The optional second row of the kernel can contain the
        standard deviations of these maximum height differences as obtained from
        a training. Both this point and the neighbouring point are considered
        for filtering.
        @param nb_point Another laser point
        @param kernel Image with the maximum allowed height differences (and
                      optionally also containing their standard deviations)
        @param range Only filter if the two points are within this range
        @param pixelsize_kernel The distance represented by one kernel pixel
        @param use_stdev If true, the maximum allowed height difference is
                         increased by three times the square root of the sum of
                         the squared standard deviation in the kernel and the 
                         squared specified standard deviation stdev.
        @param stdev See use_stdev
        @param tolerance Tolerance added to the maximum allowed height
                         difference of the kernel
        @return 1 if nb_point is filtered, 0 otherwise (i.e. also if "this"
                  is filtered)
    */
    int FilterOnMorphology(LaserPoint *nb_point, const Image &kernel,
                           double range, double pixelsize_kernel,
                           int use_stdev, double stdev, double tolerance);

    /// Neighbour point would have been filtered by morphological filter
    /** Check if nb_point would be filtered by the function FilterOnMorphology.
        See this function for an explanation of the parameters
    */
    int NeighbourWouldBeFiltered(const LaserPoint *nb_point,
                                 const Image &kernel,
                                 double range, double pixelsize_kernel,
                                 int use_stdev, double stdev,
                                 double tolerance) const;

    /// Swap the X and Y coordinate
    void SwapXY();

    /// Swap the X and Z coordinate
    void SwapXZ();

    /// Swap the Y and Z coordinate
    void SwapYZ();
};

#endif /* _LaserPoint_h_ */  /* Don't add after this point */
