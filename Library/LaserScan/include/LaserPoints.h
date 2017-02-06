
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



/*!
 * \file
 * \brief Interface to Class LaserPoints - Set of laser altimetry points
 *
 */
/*!
 * \class LaserPoints
 * \ingroup LPointSets
 * \brief Interface to Class LaserPoints - Set of laser altimetry points
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


#ifndef _LaserPoints_h_
#define _LaserPoints_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 LaserPoints                          - Set of laser altimetry points

--------------------------------------------------------------------------------
*/
#include <vector>
#include "LaserPoint.h"
#include "LaserPointsInfo.h"
#include "LaserDataFiles.h"
#include "ImagePoints.h"
#include "TIN.h"
#include "TINEdges.h"
#include "DataGrid.h"
#include "LineTopologies.h"
#include "LineSegments3D.h"
#include "LaserDataTypes.h"
#include "Histogram.h"
#include "ObjectPoints.h"
#include "Plane.h"
#include "HoughSpace.h"
#include "SegmentationParameters.h"
#include "OutliningParameters.h"
#include <Matrix3.h>
//#include "laswriter.hpp"

class LASheader;
class LASwriter;
class LASpoint;


// Extra includes for RoofSegm functions

#include "PointNumberLists.h"
#include "Planes.h"
#include "Circle2D.h"
#include "LaserPatch.h"
#include "LaserPatches.h"

//------------------------------------------------------------------------------
///                 Set of laser altimetry points
//------------------------------------------------------------------------------

class LaserPoints : public std::vector<LaserPoint>, public LaserPointsInfo,
                    public LaserDataFiles
{
  protected:

    /// Delaunay triangulation of the points
    TIN *tin;
    
    /// Edges of the neighbourhood definition (indices of neighbouring points)
    TINEdges *nbh_edges;

    /// Seek offset for a point file with multiple point sets
    long long seek_offset;
    
    /// Highest surface number (used to separate surfaces from other segments)
    int highest_surface_number;

/* Functions defined in file LaserPoints.cc */

  public:
    /// Default constructor
    LaserPoints()
      {Initialise();}

    /// Construct by setting point file name
    LaserPoints(const char *filename)
      {Initialise(); SetPointFile(filename);}

    /// Construct by reading point file
    LaserPoints(int *success, const char *filename)
      {Initialise(); *success = Read(filename);}

    /// Construct by reading meta data file
    LaserPoints(const char *filename, int *success)
      {Initialise(); *success = ReadMetaData(filename);}
	  
	///Construct from a vector of something that defines [] operator for [0],[1],[2].
	template<class T>
	LaserPoints(const vector<T>& v)
	{
		Initialise();
		resize(v.size());
		for(int i=0;i<v.size();i++)
			(*this)[i] = LaserPoint(v[i][0],v[i][1],v[i][2]);
	}
	
	///Construct for a given size.
	LaserPoints(int new_size)
	{
		Initialise();
		resize(new_size);
	}
	
	///Construct using an iterator.
	template <class It>
	LaserPoints(It i1,It i2)
	{
		Initialise();
		reserve(i2-i1);
		for(It i=i1;i!=i2;i++)
			push_back(LaserPoint((*i)[0],(*i)[1],(*i)[2]));
	}
	
	/// + operator, combines two laser points
    LaserPoints operator +(const LaserPoints& b)const;
	
	///Set reflectance of all points to one value.
	LaserPoints& SetReflectance(double d);
	
	///Set reflectance from a vector of values.
	template<class T>
	LaserPoints& SetReflectance(const std::vector<T>& v)
	{
		LaserPoints& pts = (*this);
		for(int i=0;i<pts.size() && i<v.size() ;i++)
			pts[i].Reflectance() = v[i];
		return pts;
	}


	///Collect ids of all points and return them as a vector.
	vector<long long int> GetIds() const;

	/// Retrieve readable attribute value
	vector<int> GetAttribute(const LaserPointTag tag) const;
	
	/// Set attribute values of the whole vector.
	template<class T>
	void SetAttribute(const LaserPointTag tag, const vector<T>& values) 
	{
		for(int i=0;i<values.size() && i<size() ;i++)
			(*this)[i].SetAttribute(tag,values[i]);
	}
	
	/// Set attribute to one value.
	template<class T,class U>
	void SetAttribute(const LaserPointTag tag, const U value) 
	{
		for(int i=0;i<size() ;i++)
			(*this)[i].SetAttribute(tag,value);
	}
	
	///select using passed indices and return the result.
	template <class T>
	LaserPoints Select(const std::vector<T>& v)const
	{
		LaserPoints sel;

		if(v.empty())
			return sel;

		sel.reserve(v.size());

		for(int i=0;i<v.size();i++)
		{
			if(v[i]>=0 && v[i]< (this->size()))
				sel.push_back((*this)[v[i]]);
		}
		return sel;
	}
	
	
	template<class T>
	bool IsValidIndex(T i)const
	{
		return (i>=0 && i<size());
	}
	
	///Project to plane.
	LaserPoints ProjectToPlane(Vector3D zv)const;
		
	///Collect reflectance of all points.
	vector<int> GetReflectance() const;

	///Transform all points and return in the form of another LaserPoints.
	LaserPoints operator*(const Rotation3D& r)const;

	///Transform all points using orientation 3D.
	LaserPoints operator*(const Orientation3D& r)const;

	///Translate all points.
	LaserPoints operator+(const Vector3D& r)const;

	///Translate all points.
	LaserPoints operator-(const Vector3D& r)const;

	///Add uniform noise to all points.
	LaserPoints AddNoise(double level)const;

	///Add gaussian noise to all points.
	LaserPoints AddNoiseG(double stdDev, double mean=0)const;

	///Rotate and translate by the given amount at the same time.
	LaserPoints Transform(const Rotation3D& rot,const Vector3D&trans)const;

	///Calculate the mean of the laser points.
	Vector3D Mean()const;

	///Calculate normal using all points. Also return rho.
	Vector3D Normal(double* pResidual=NULL,double *pRho=NULL,vector<Vector3D>* directions=NULL)const;
	
	///Calculate normal using all points. Also return eigenvalues.
	Vector3D NormalAndEigenValues(Vector3D &eigenvalues) const;
	
	///Calculate Curvature using new-mat.
	double Curvature(double* majorCurvature=NULL,double* minorCurvature=NULL,
			Vector3D* majorDirection=NULL,Vector3D* minorDirection=NULL)const;
			
	///Calculate curvature for all points. Use K nearest neighbors.
	std::vector<double> Curvatures(int KNN,
					std::vector<double>* majorCurvatures = NULL,
					std::vector<double>* minorCurvatures = NULL,
					std::vector<Vector3D>* majorDir = NULL,
					std::vector<Vector3D>* minorDir = NULL);
	///Fit a quadric. We have 10 parameters for it.
	vector<double> FitQuadric(Vector3D* pEigVectors=NULL , double* pEigenValues=NULL)const;

	///Subsample skipping every nth point.
	LaserPoints SubSampleSimple(int step=10)const;
	
	///Subsample using space partition but with constant distance along each axis.
	LaserPoints SubSample(double stepSize=1)const;

	///Saves to a passed vrml file.
	void SaveVrml(std::string fileName)const;

	///Show as vrml.
	void ShowVrml(std::string message)const;

	///Histoequalize the reflectance.
	LaserPoints Histoequalize(const int binCount=512)const;

	///Calculate normals for all points using kNN.
	vector<Vector3D> Normals(int kNN,bool showProgress=true,vector<double>* pResiduals=NULL);
	
	///Calculate normals for all points using kNN (a const version).
	vector<Vector3D> Normals(int kNN,bool showProgress=true,vector<double>* pResiduals=NULL)const
	{
		LaserPoints& pts = *(const_cast<LaserPoints*>(this));
		return pts.Normals(kNN,showProgress,pResiduals);
	}


	///Calculate normals for all points using kNN 
	///Also return rhos.
	vector<Vector3D> Normals(int kNN,vector<double>& rhos,
						bool showProgress,vector<double>* pResiduals=NULL)const;

	///Calculate normals and eigenvalues for all points using kNN.
	vector<Vector3D> NormalsAndEigenValues(int kNN,
	                                       vector<Vector3D>* eigenvalues,
										   bool showProgress=true);
	
	/// Calculate normals  (based on eigenvalues)
	/** Normals and eigenvalues are derived from knn neighbours. The
	    flatness of the point cloud is determined based on the smallest
	    and largest eigen value. The linearity is determined based on the two
		smallest eigenvalues. All values are stored in the tags
		NormalXTag, NormalYTag, NormalZTag, FlatnessTag, and LinearityTag.
	    @param knn Number of neighbours used to calculate local normal vector
	*/
	void CalculateNormals(int knn);
	
	/// Calculate eigenvalues in local, possibly segmented, neighbourhoods
	/** Eigenvalues are computed for the local neighbourhood of each point.
	    If a point contains a segment number, only points with the same
	    segment number are used for the computation.
	    @param parameters Segmentation parameters that contain the neighbourhood definition
	    @param scale_eigenvalues Scale eigenvalues to sum up to one.
	*/
    void CalculateLocalEigenValues(const SegmentationParameters &parameters,
	                               bool scale_eigenvalues=false);

    /// Calculate local flatness and linearity from local eigenvalues
    void CalculateLocalShape();
    
	/// Calculate normals scaled by flatness (based on eigenvalues)
	/** Normals and eigenvalues are derived from knn neighbours. The
	    flatness of the point cloud is determined based on the smallest
	    and largest eigen value. The linearity is determined based on the two
		smallest eigenvalues. All values are stored in the tags
		ScaledNormalXTag, ScaledNormalYTag, ScaledNormalZTag, 
		FlatnessTag, LinearityTag.
		The normal vector itself is not stored, but can be retrieved by
		re-scaling the scaled normal vector to 1.

	    @param knn Number of neighbours used to calculate local normal vector
	*/
	void CalculateScaledNormals(int knn);
	
	///Return the extents calculated by Principal Component Analysis.
	vector<double> Extents(vector<Vector3D>* pOrthogonalDirections=NULL)const;
	
	///Return the reflectance range.
	double ReflectanceRange(double* pMin=NULL, double* pMax=NULL, double* pScaleFactor=NULL)const;

	///Return the distance from the closest points from a given pointcloud. 
	vector<double> Distance(const LaserPoints& pts,double factor=1)const;

	///Distance from the closest point.
	double Distance(Vector3D v)const;
	      
    /// Copy constructor
    LaserPoints(const LaserPoints &pts) : std::vector<LaserPoint>(),
      LaserPointsInfo(), LaserDataFiles(), tin(NULL),nbh_edges(NULL)
      {*this = pts;}

    /// Default destructor
    ~LaserPoints();

    /// Copy assignment
    LaserPoints & operator = (const LaserPoints &);

    /// Return the writable reference
    LaserPoints &LaserPointsReference()
      {return(*this);}

    /// Return the readable reference
    const LaserPoints &LaserPointsReference() const
      {return(*this);}

    /// Return the writable pointer
    LaserPoints *LaserPointsPointer()
      {return(this);}

    /// Return the readable pointer
    const LaserPoints *LaserPointsPointer() const
      {return(this);}

    /// Initialise new set of laser points
    void Initialise();

    /// Reinitialise old set of laser points
    void ReInitialise();

    /// Read meta data from "meta_file"
    int ReadMetaData();

    /// Read meta data from file
    int ReadMetaData(const char *filename);

    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to file
    int WriteMetaData(const char *filename) const;

    /// Derive name of meta data file
    char *DeriveMetaDataFileName(const char *directory);
  
    /// Set seek position of laser data
    void SetSeekOffset(long long offset)
      {seek_offset = offset;}
      
    /// Return seek offset
    long long SeekOffset() const
      {return seek_offset;}
      
    /// Set highest surface number
    void SetHighestSurfaceNumber(int number)
      {highest_surface_number = number;}
      
    /// Return highest surface number
    int HighestSurfaceNumber() const
      {return highest_surface_number;}
      
    /// Read points from "point_file"
    /** @param verbose If true, the number of stored points is written to stdout
    */
    int Read();

    /// Read points from file
    /** @param filename Name of the laser point file
        @param verbose If true, the number of read points is written to stdout
        @param max_num_pts Maximum number of points to read. No limit if -1
        @return 1 in case of success, 0 in case of failure
    */
    int Read(const char *filename, bool verbose=true, int max_num_pts=-1);
		
    /// Read points from an already opened file
    /** @param fd File descriptor of opened file
        @param verbose If true, the number of read points is written to stdout
        @param max_num_pts Maximum number of points to read. No limit if -1
        @return 1 in case of success, 0 in case of failure
    */
    int Read(FILE *fd, bool verbose=false, int max_num_pts=-1);
		
	/// Load from an ascii file.
	/** @fileName name of the file to load data from
		@delimiters separating each token in the ascii file (can be multiple)
		@order a vector with column numbers for x,y,z, and reflectance. Default is column 0-4
	**/
	int LoadFromAscii(const char* fileName,const char* delimiters=" ",vector<int> order=vector<int>());
	
	
		
	///Saves to an ascii file.
	/// @param fileName name of the file.
	/// @param format format of double x,y,z (used in calling sprintf)
	/// @param delimiters (The string used to separate each entry in the text file)
	///
	int SaveToAscii(const char* fileName, const char* format=NULL, char* delimiters=NULL) const;
	
	
	///loads from a ptx file. (Ascii standard used by Cyrax-Leica scanners)
	///Contains entry about transformation in ten lines.
	///First two are the number of points.
	///Next 4 can be skipped.
	///Last 4 given full transformation matrix, that has to be pre-multiplied by points in a row vector.
	///
	/// [x y z 1] * T 
	///
	int LoadFromPtx(const char* fileName); 

  
    /// Read header data
    /** @param file File descriptor
        @param num_pts Number of laser points in the file (as read from the
                       file header)
        @param x_offset X-coordinate offset to be applied to the laser points
        @param u_offset Y-coordinate offset to be applied to the laser points
        @param z_offset Z-coordinate offset to be applied to the laser points
        @return 0 in case of failure, internal storage format ID in case of
                  success
    */
    int ReadHeader(FILE *file, int *num_pts, double *x_offset,
                   double *y_offset, double *z_offset,
                   unsigned char *max_num_attributes);

    /// Read points from "point_file" in file format version 0
    /**
        @param verbose If true, the number of stored points is written to stdout
    */
    int ReadV0(bool verbose=true);

    /// Read points from stream in file format version 1
    /** @param file File descriptor
        @param num_pts Number of laser points in the file (as read from the
                       file header)
        @param x_offset X-coordinate offset to be applied to the laser points
        @param u_offset Y-coordinate offset to be applied to the laser points
        @param z_offset Z-coordinate offset to be applied to the laser points
        @param verbose If true, the number of read points is written to stdout
        @return 0 in case of failure, 1 in case of success.
    */
    int ReadV1(FILE *file, int num_pts, double x_offset, double y_offset,
               double z_offset, bool verbose=true);

    /// Read points from stream in file format version 2
    /** @param file File descriptor
        @param num_pts Number of laser points in the file (as read from the
                       file header)
        @param x_offset X-coordinate offset to be applied to the laser points
        @param u_offset Y-coordinate offset to be applied to the laser points
        @param z_offset Z-coordinate offset to be applied to the laser points
        @param verbose If true, the number of read points is written to stdout
    @return 0 in case of failure, 1 in case of success.
    */
    int ReadV2(FILE *file, int num_pts, double x_offset, double y_offset,
               double z_offset, bool verbose=true);

    /// Read points from stream in file format version 2
    /** @param file File descriptor
        @param num_pts Number of laser points in the file (as read from the
                       file header)
        @param x_offset X-coordinate offset to be applied to the laser points
        @param u_offset Y-coordinate offset to be applied to the laser points
        @param z_offset Z-coordinate offset to be applied to the laser points
        @param max_num_attributes Maximum number of attributes of a laser point
        @param max_num_pts Maximum number of points to read. No limit if -1
        @return 0 in case of failure, 1 in case of success.
    */
    int ReadV3(FILE *file, int num_pts, double x_offset, double y_offset,
               double z_offset, unsigned char max_num_attributes,
               int max_num_pts=-1);

    /// Read points from a LAS file
    /** @param filename Name of the LAS file
        @param verbose If true, the number of read points is written to stdout
        @param max_num_pts Maximum number of points to read
        @return 0 in case of failure, 1 in case of success.
    */
    int ReadLAS(const char *filename, bool verbose, int max_num_pts=-1);

    /// Write points to file
    /** The laser points are written to a binary file.
        @param filename File name for the laser points
        @param compress If true, the file will be compressed with gzip
        @param verbose If true, the number of stored points is written to stdout
        @return 1 in case of success, 0 in case of failure.
    */
    int Write(const char *filename, int compress, bool verbose=true);
    
    /// Write points to an already opened binary file
    /** The laser points are written to a binary file.
        @param filename File name for the laser points
        @param compress If true, the file will be compressed with gzip
        @param verbose If true, the number of stored points is written to stdout
        @return Number of written bytes.
    */
    int Write(FILE *fd);
    
    /// Write points to file in old version 2
    /** The laser points are written to a binary file of outdated version 2.
        @param filename File name for the laser points
        @param compress If true, the file will be compressed with gzip
        @return 1 in case of success, 0 in case of failure.
    */
    int WriteV2(const char *filename, int compress);

    /// Write points to "point_file" and compress it
    /** The laser points are written to a binary file.
        @param verbose If true, the number of stored points is written to stdout
        @return 1 in case of success, 0 in case of failure.
    */
    int Write(bool verbose=true)
      {return(Write(point_file, 1, verbose));}

    /// Write the header of a LAS file
	/** @param filename Name of the LAS file
	    @param num_pts Number of points to write later
		@param colour  True if RGB values need to be stored
		@param laspoint Structure to store points in LAS format
		@param lasheader LAS header structure
		@param bounds Pointer to databounds
		@return Structure for writing and closing of file
    */
    LASwriter *WriteLASHeader(const char *filename, long long int num_pts,
                              bool colour, LASpoint &laspoint,
							  LASheader &lasheader,
						      const DataBoundsLaser *bounds=NULL) const;
						      
    /// Write the points to an opened LAS file
    /** @param laswriter Structure for writing and closing of file, created
                         by WriteLASHeader
        @param laspoint Structure for storing points in LAS format, created
                        by WriteLASHeader
		@param colour  True if RGB values need to be stored
	*/
    void WriteLASPoints(LASwriter *laswriter, LASpoint &laspoint,
	                    bool colour) const;

    /// Write points to a LAS file
    /** @param filename Name of the LAS file
        @param verbose If true, the number of written points is written to stdout
        @return 0 in case of failure, 1 in case of success.
    */
    int WriteLAS(const char *filename, bool verbose) const;

    /// Erase all laser points from the class instant
    void ErasePoints(bool erase_bounds=true);

    /// Erase the TIN of the laser points
    void EraseTIN()
      {if (tin) {tin->Erase(); delete tin; tin=NULL;}}
      
    /// Erase the TIN of the laser points
    void EraseNeighbourhoodEdges()
      {if (nbh_edges) {nbh_edges->Erase(); delete nbh_edges; nbh_edges=NULL;}}
      
    /// Set an attribute value for all points
    void SetAttribute(const LaserPointTag tag, const int value);
    
    /// Set a float attribute value for all points
    void SetAttribute(const LaserPointTag tag, const float value);
    
    /// Check if some points have an attribute of a specified type
    /** Note that not all points need to have this attribute. This function
        only returns false if none of the points has the specified attribute
    */
    bool HasAttribute(const LaserPointTag tag) const;
    
    /// Determine all different values of a specific attribute
    void AttributeValues(const LaserPointTag tag, vector <int> & values) const;
    
    /// Determine all different values of a specific attribute
    vector <int> & AttributeValues(const LaserPointTag tag) const;

    /// Determine all different values of a specific attribute and their frequencies
    void AttributeValueCounts(const LaserPointTag tag, vector <int> & values,
	                          vector <int> & counts) const;

    /// Determine all used long segment numbers (including tile numbers)
    void LongSegmentNumbers(vector <long long int> & values) const;

    /// Determin all different attributes
    vector <LaserPointTag> & UsedAttributes() const;
    
    /// Determine the value range of an integer attribute
    bool AttributeRange(const LaserPointTag tag, int &minimum, int &maximum) const;
    
    /// Determine the value range of a float or double attribute
    bool AttributeRange(const LaserPointTag tag, double &minimum, double &maximum) const;
    
    /// Remove an attribute from all points
    void RemoveAttribute(const LaserPointTag tag);

    /// Remove all attributes from all points
    void RemoveAttributes();

    /// Rename an attribute for all points
    void RenameAttribute(const LaserPointTag oldtag, 
                         const LaserPointTag newtag);
                         
    /// Return the data bounds
    const DataBoundsLaser &DataBounds() const
      {return(bounds);}

    /// Derive the bounds from the data
    /** @param use_known_bounds If true, bounds are not recalculated if they
                                are already set
        @return The determined data bounds
    */
    DataBoundsLaser DeriveDataBounds(int use_known_bounds);

    /// Return the pointer to the TIN
    TIN *GetTIN()
      {return(tin);}

    /// Return the reference to the TIN
    TIN &TINReference()
       {return(tin->TINReference());}

    /// Return the const reference to the TIN
    const TIN &TINReference() const
       {return(tin->TINReference());}

    /// Set a new TIN
    void SetTIN(TIN *src_tin)
       {tin = src_tin;}

    /// Set a new TIN by copying
    void SetTIN(TIN &src_tin)
       {tin = new TIN(src_tin);}

    /// Read TIN from a file
    TIN *ReadTIN(const char *filename);

    /// Read TIN from "tin_file"
    TIN *ReadTIN();

    /// Write TIN to a file and store the TIN file name
    int WriteTIN(const char *filename);

    /// Write TIN to "tin_file"
    int WriteTIN() const;

    /// Return neighbourhood edges pointer
    TINEdges *GetNeighbourhoodEdges()
      {return nbh_edges;}
      
    // Return the reference to the neighbourhood edges
    TINEdges &NeighbourhoodEdges()
      {return nbh_edges->TINEdgesRef();}
      
    /// Return the const reference to the neighbourhood edges
    const TINEdges &NeighbourhoodEdges() const
      {return nbh_edges->TINEdgesRef();}
      
    /// Set new neighbourhood edges
    void SetNeighbourhoodEdges(TINEdges *src_nbh_edges)
      {nbh_edges = src_nbh_edges;}
      
    /// Set new neighbourhood edgse by copying
    void SetNeighbourhoodEdges(TINEdges &src_nbh_edges)
      {nbh_edges = new TINEdges(src_nbh_edges);}
      
    /// Put all X and Y coordinates in a list
    /** This function is used to compose the input for the Delaunay
        triangulation with the Triangle software.
        @return Pointer to a double array with the X- and Y-coordinates
                of the laser points.
    */
    double *TriangleCoordinateList() const;

    /// Derive a TIN from the points
    TIN *DeriveTIN();

    /// Verify TIN and (re-)derive it when needed
    TIN *VerifyTIN();
    
    /// Convert laser points to image points
    ImagePoints *Map_Into_Image (const ImageGrid &grid) const;
      
    /// Add laser data to an image
    /** Conversion of the laser data to an image.
        @param image The rasterised laser data
        @param dist_image Image to keep track of the nearest laser points
        @param grid The image grid specification
        @param type HeightData, ReflectanceData or ColourData
        @param int_method Interpolation method:
                    1 - Nearest neighbour within a pixel (also called binning),
                    2 - Barycentric coordinates,
                    3 - Planar interpolation in TIN mesh,
                    4 - Minimum value within a pixel
        @param max_mesh_size Maximum mesh size to interpolate
                             (relevant for method 2 and 3 only)
    */
    void ImageData(Image &image, Image &dist_image, const DataGrid &grid,
                   ImagedDataType type, int int_method, double max_mesh_size);

    /// Add laser data to an image using nearest neighbour or minimum binning
    /** Only for usage within the function ImageData. See ImageData for an
        explanation of the parameters.
    */
    void ImageDataBinning(Image &image, Image &dist_image, const DataGrid &grid,
                          ImagedDataType type, int int_method);

    /// Add laser data to an image using interpolation in a TIN
    /** Only for usage within the function ImageData. See ImageData for an
        explanation of the parameters.
    */
    void ImageDataTriangle(Image &image, Image &dist_image, const TIN &tin,
                           const DataGrid &grid, ImagedDataType type,
                           int int_method, double max_mesh_size);

    /// Barycentric interpolation
    /** @param mesh A mesh of the TIN
        @param X    X-coordinate of the location to be interpolated
        @param Y    Y-coordinate of the location to be interpolated
        @param type Image data type, HeightData, ReflectanceData or ColourData
        @return The interpolated data value
    */
    double Inter_bary(const TINMesh *mesh, double X, double Y,
                      ImagedDataType type) const;

    /// Planar interpolation
    double Inter_plane(const TINMesh *mesh, double X, double Y,
                       ImagedDataType type) const;
    /** @param mesh A mesh of the TIN
        @param X    X-coordinate of the location to be interpolated
        @param Y    Y-coordinate of the location to be interpolated
        @param type Image data type, HeightData, ReflectanceData or ColourData
        @return The interpolated data value
    */

    /// Extract the data bounds of a TIN mesh
    DataBoundsLaser ExtractBounding(const TINMesh *mesh) const;

    /// Check if a position is inside a TIN mesh
    /** @param mesh A TIN mesh
        @param X    X-coordinate of the location to be checked
        @param Y    Y-coordinate of the location to be checked
        @return 1 if the location is inside the TIN mesh, 0 if outside.
    */
    int InsideTriangle(const TINMesh *mesh, double X, double Y) const;

    /// Select a subset of points within some bounds
    /** @param selection The laser point set to which the selected points are
                         to be added
        @param bounds    Points within these bounds are selected
    */
    void Select(LaserPoints &selection, const DataBoundsLaser &bounds) const;

    /// Select a subset of points within a rotated rectangle
    /** @param selection  The laser point set to which the selected points are
                          to be added
        @param line       Centre line of the rectangle
        @param max_dist   Maximum distance of a point to the line
        @param scalar_min Minimum scalar value
        @param scalar_max Maximum scalar value
    */
    void Select(LaserPoints &selection, const Line2D &line,
                double max_dist, double scalar_min, double scalar_max) const;

    /// Select a subset of points within polygons
    /** @param selection The laser point set to which the selected points are
                         to be added
        @param pts       2D object points of the polygons
        @param top       Topology of the polygons
    */
    void Select(LaserPoints &selection, const ObjectPoints2D &pts,
                const LineTopologies &top) const;

    /// Remove all points outside bounds
    void ReduceData(const DataBoundsLaser &bounds);

    /// Select a subset of points by a labeled image
    /** A point is selected if the value of the corresponding image pixel does
        not equal 0.
        @param selection The laser point set to which the selected points are
                         to be added
        @param image     Image
        @param grid      Image grid specification
    */
    void Select(LaserPoints &selection, const Image &image,
                const ImageGrid &grid) const;

    /// 2D Direction between two laser points
    double Direction2D(const PointNumber &n1, const PointNumber &n2) const;

    /// 2D Angle between three laser points
    double Angle2D(const PointNumber &n1, const PointNumber &n2,
                   const PointNumber &n3) const;

    /// Write points to a VRML file
    void VRML_Write(FILE *file, int version=1, bool colour=false) const;

    /// Write points as crosses to a VRML file
    void VRML_Write_Crosses(FILE *file, double size) const;	

    /// Write points as spheres to a VRML file
    void VRML_Write_Spheres(FILE *file, double size) const;	

    /// Write lines as cylinders to a VRML file
    void VRML_Write_Cylinders(FILE *file, const LineTopologies &topology,
                              double radius) const;	

    /// Add points to a histogram
    /** @param histogram The histogram to be updated with the laser points
        @param type Type of histogram: 1 - X-coordinate, 2 - Y-coordinate,
                    3 - Z-coordinate, 4 - Reflectance, 5 - Pulse count,
                    6 - Red, 7 - Green, 8 - Blue
    */
    void AddToHistogram(Histogram &histogram, int type) const;

    /// Add points to a histogram
    /** @param histogram The histogram to be updated with the laser points
        @param type Type of histogram: 0 - Attribute 
                    1 - X-coordinate, 2 - Y-coordinate,
                    3 - Z-coordinate
    */
    void AddToHistogram(Histogram &histogram, int type,
                        const LaserPointTag &tag) const;

    /// Select all points within some 2D or 3D range of a specified point
    /** Note that for a 3D neighbourhood the edges should have been generated
        with LaserOctree::NeighbourhoodEdges3D and NOT
        be the edges of a Delaunay triangulation. In the latter case, use
        function Neighbourhood3D.
        @param centre Number of the centre point of the neighbourhood to be
                      selected
        @param range  Radius of the neighbourhood
        @param edges  Edges of the TIN
        @param planimetric If true, 2D distances are used, otherwise 3D.
        @param direct_neighbours_only If true, only points connected through
                                      the centre by edges are considered
        @return       Numbers of points within the specified range of the 
                      specified centre point
    */
    PointNumberList Neighbourhood(const PointNumber &centre, double range,
                                  const TINEdges &edges,
                                  bool planimetric=true,
                                  bool direct_neighbours_only=false) const;

    /// Select all points within some 3D range of a specific point
    /** First, all points within a 2D range are selected, then all points
        outside the 3D range are removed. This version should be used if
        the edges are based on a Delaunay TIN.
        @param centre Number of the centre point of the neighbourhood to be
                      selected
        @param range  Radius of the neighbourhood
        @param edges  Edges of the TIN
        @return       Numbers of points within the specified range of the 
                      specified centre point
    */
    PointNumberList Neighbourhood3D(const PointNumber &centre, double range,
                                    const TINEdges &edges) const;
       
    /// Reduce data by thinning
    /** @param reduction_factor Select one out of each group of
                                "reduction_factor" points
        @param random If true, a random point out of each group is selected, 
                      otherwise the first point of each group is selected
        @param start_offset Offset of first point to be selected
    */
    void ReduceData(int reduction_factor, int random, int start_offset=0);

    /// Reduce data such that there are no two points within a specified distance
    /** @param dist Minimum distance between two points
        @param knn  Number of neighbours for kd-tree. If knn=0, a TIN is used and
                    the distance is only computed in 2D. If knn>0, a kd-tree is
                    computed and the distance is computed in 3D.
    */
    void ReduceData(double dist, int knn=0);

    /// Reduce data such that only one point of k nearest neighbours is kept
    void ReduceData(int knn_start, int knn_max, double reduction_factor);
    
    /// Convert laser points to object points
    ObjectPoints ConstructObjectPoints() const;

    /// Return median distance between successive points
    /** This function is used to get an estimated of the point spacing in
        a scan line.
        @param num_pairs Use the first "num_pairs" point pairs for the
                         median distance computation.
    */
    double MedianInterPointDistance(int num_pairs) const;

    /// Determine the zero, first and second order (non central) moments of the laser points
    void Moments(double &m00, double &m10, double &m01,
                 double &m20, double &m11, double &m02) const;

    /// Smooth the polygon of laser points
    /** @param polygon The initial polygon topology
        @param max_outlier Maximum allowed distance between the initial polygon
                           and the smoothed version
        @param min_dist    Minimum distance between the nodes of the smoothed
                           polygon
        @return Topology of the smoothed polygon
    */
    LineTopology SmoothOutline(const LineTopology &polygon, double max_outlier,
                               double min_dist) const;

    /// Check if a point is part of this point set
    bool Contains(const LaserPoint &point) const;
    
    /// Remove points with the same X, Y (and Z) coordinates
    /** @param only_check_X_and_Y If true, only planimetry is used
        @return number of removed points
    */
    int RemoveDoublePoints(bool only_check_X_and_Y=true);

/* Functions defined in file FilterPoints.cc */

    /// Select all points for the DEM
    void SetUnFiltered();

    /// Select all points as non-terrain points
    void SetFiltered();

    /// Set all point labels
    void Label(int label);
    
    /// Set all points as processed
    void SetProcessed();
    
    /// Set all points as not processed
    void SetUnProcessed();

    /// Change point attribute values
    int ReTag(int old_label, int new_label, const LaserPointTag tag);

    /// Conditionally change point attribute values
    /** Change value of tag label_tag from old_label to new_label if the
        value of tag test_tag equals test_value
        @return The number of relabeled points
    */
    int ConditionalReTag(int old_label, int new_label,
                         const LaserPointTag label_tag,
                         int test_value, const LaserPointTag test_tag);
    
    int MultiConditionalReTag(int new_value, const LaserPointTag new_tag, int value1,
                         const LaserPointTag tag1,
                         int value2, const LaserPointTag tag2);
                         
    /// Change point labels
    int ReLabel(int old_label, int new_label)
      { return ReTag(old_label, new_label, LabelTag); }

    /// Add points to this
    int AddPoints(const LaserPoints &additional_points);
    
    /// Add points with a specific attribute value to this
    /** @param tagged_points Point set from which a selection is added to this
        @param value Attribute value to be used for the selection
        @param tag Tag of attribute to be used for the selection
        @return The number of points added to this
    */
    int AddTaggedPoints(const LaserPoints &tagged_points, int value,
                        const LaserPointTag tag);

    /// Add points with a specific attribute value to this and set selection tag
    /** @param tagged_points Point set from which a selection is added to this
        @param value Attribute value to be used for the selection
        @param tag Tag; of attribute to be used for the selection
        @param selecttag Tag of attribute to be set to 1 in tagged_points
        @return The number of points added to this
    */
    int AddTaggedPoints(LaserPoints &tagged_points, int value,
                        const LaserPointTag tag, const LaserPointTag selecttag);

    /// Add points with two specific attribute values to this and set selection tag
    /** @param tagged_points Point set from which a selection is added to this
        @param value1 Attribute value to be used for the selection
        @param tag1 Tag; of attribute to be used for the selection
        @param value2 Attribute value to be used for the selection
        @param tag2 Tag; of attribute to be used for the selection
        @param selecttag Tag of attribute to be set to 1 in tagged_points
        @return The number of points added to this
    */
    int AddTaggedPoints(LaserPoints &tagged_points,
	                    int value1, const LaserPointTag tag1, 
	                    int value2, const LaserPointTag tag2, 
						const LaserPointTag selecttag);

    /// Add points with a specific label to this
    /** @param labeled_points Point set from which a selection is added to this
        @param label Label value to be used for the selection
        @return The number of points added to this
    */
    int AddLabeledPoints(const LaserPoints &labeled_points, int label)
      { return AddTaggedPoints(labeled_points, label, LabelTag); }

    /// Remove points with a specific attribute (no matter what value)
    /** @param tag   Tag of attribute to be used for the selection
        @return The number of removed points
    */
    int RemoveTaggedPoints(const LaserPointTag tag);

    /// Remove points without a specific attribute
    /** @param tag   Tag of attribute to be used for the selection
        @return The number of removed points
    */
    int CropTaggedPoints(const LaserPointTag tag);

    /// Remove points with a specific attribute value
    /** @param value Attribute value to be used for the selection of points to
                     be removed
        @param tag   Tag of attribute to be used for the selection
        @return The number of removed points
    */
    int RemoveTaggedPoints(int value, const LaserPointTag tag);

    /// Remove points with two specific attribute values
    /** @param value1 Attribute value to be used for the selection of points to
                      be removed
        @param tag1   Tag of attribute to be used for the selection
        @param value2 Attribute value to be used for the selection of points to
                      be removed
        @param tag2   Tag of attribute to be used for the selection
        @return The number of removed points
    */
    int RemoveTaggedPoints(int value1, const LaserPointTag tag1,
	                       int value2, const LaserPointTag tag2);

    /// Remove points with a specific label
    /** @param label Label value to be used for the selection of points to be
                     removed
        @return The number of removed points
    */
    int RemoveLabeledPoints(int label)
      { return RemoveTaggedPoints(label, LabelTag); }

    /// Remove all points without a specific attribute value
    /** @param value Attribute value to be used for the selection of points to
                     be kept
        @param tag   Tag of attribute to be used for the selection
        @return The number of removed points
    */
    int CropTaggedPoints(int value, const LaserPointTag tag);

    /// Filter all points based on slope
    /** If one point is higher than the neighbouring point by more than the
        maximum slope times the distance between the points and reduced by
        2 sqrt(2) sigma, this points is set to filtered. 
        @param range All points that are less apart than this range are checked
        @param slope The assumed maximum terrain slope
        @param sigma Standard deviation of the laser point heights
    */
    void FilterOnSlope(double range, double slope, double stdev,
                       const TINEdges &edges);

    /// Delete the filtered points
    void RemoveFilteredPoints();

    /// Delete the DEM points
    void RemoveGroundPoints();

    /// Select the filtered points
    /** Copy the filtered points of a set of laser points to this.
        @param all_points A set of laser points with filtered and unfiltered
                          points
    */
    void SelectFilteredPoints(const LaserPoints &all_points);

    /// Filter on morphology
    /** For the distance between the two points the maximum allowed difference
        in the terrain heights is extracted from the kernel and compared with
        the actual height difference. The first row of the kernel should
        contain the maximum allowed height differences as a function of the
        distance. The optional second row of the kernel can contain the
        standard deviations of these maximum height differences as obtained from
        a training. Pairs of two points within the specified range are
        checked.
        @param kernel Image with the maximum allowed height differences (and
                      optionally also containing their standard deviations)
        @param range Only filter if the two points are within this range
        @param stdev Standard deviation of the laser point heights
        @param tolerance Tolerance added to the maximum allowed height
                         difference of the kernel
        @param edges The edges of the Delaunay triangulation
    */
    void FilterOnMorphology(const Image &kernel, double range,
                            double stdev, double tolerance,
                            const TINEdges &edges);

    /// Filter extremely low points
    /** Outliers below the ground level disturb the slope based filtering
        process. This function should remove these outliers by checking whether
        a single point or a group of a few points is far below all other points
        in the neighbourhood.
        @param kernel Morphological filter. Typically with height 0 at distance
                      0 and a constant higher value for all other distances.
        @param range The size of the neighbourhood of a low point to be
                     examined
        @param stdev Standard deviation of the height of the laser points
        @param tolerace Tolerance to be added to the allowed height difference
        @param edges The edges of the Delaunay triangulation
        @param max_unfil_nbs The maximum number of points far below the
                             ground surface that may be filtered at one
                             location. If this number is exceeded, it is
                             assumed that the low points are no outliers, but
                             a true terrain object.
    */
    void FilterLowPoints(const Image &kernel, double range, double stdev,
                         double tolerance, const TINEdges &edges,
                         int max_unfil_nbs);
                         
    /// Set the last pulse flags based on the pulse counts
    void SetLastPulseFlags();

/* Functions defined in the file Components.cc */

    /// Derive edges that define the neighbourhood relationships.
    /** See class SegmentationParameters for more information. This function
        is not a const function, because a TIN may be derived that is stored
        in the LaserPoints class.
        @param parameters The parameters that define the type of neighbourhood
                          and the values of neighbourhood properties
        @return Neighbourhood edges
    */
    TINEdges *DeriveEdges(const SegmentationParameters &parameters);
    
    /// Derive fixed distance neighbourhood edges.
    /** See class SegmentationParameters for more information. This function
        is not a const function, because a TIN may be derived that is stored
        in the LaserPoints class. The growing radius is used as the
        neighbourhood radius.
        @param parameters The parameters that define the type of neighbourhood
                          and the values of neighbourhood properties
        @return Neighbourhood edges
    */
    TINEdges *DeriveFDNEdges(const SegmentationParameters &parameters);
    
    /// Verify edges and (re-)derive when needed
    /** See class SegmentationParameters for more information. This function
        is not a const function, because a TIN may be derived that is stored
        in the LaserPoints class.
        @param parameters The parameters that define the type of neighbourhood
                          and the values of neighbourhood properties
        @return Neighbourhood edges
    */
    TINEdges *VerifyEdges(const SegmentationParameters &parameters);
    
    /// Determine neighbourhood of a point based on label and/or filter state
    /** Select all points that have the same label and/or the same filter
        state as the centre point and can be reached via TIN edges of points
        with the same label and/or filter state.
        @param centre Start point for the neighbourhood selection. The label
                      and/or filter state of this point are compared with the
                      surrounding points.
        @param edges  The edges of the Delaunay triangulation
        @param use_label If set, the labels are used to decide if points belong
                         to the same neighbourhood
        @param use_filter_state If set, the filter states are used to decide
                                if points belong to the same neighbourhood
        @param new_label New label for the points of the neighbourhood
        @return List of numbers of the points of the neighbourhood
    */
    PointNumberList Neighbourhood(const PointNumber &centre,
                                  const TINEdges &edges, int use_label,
                                  int use_filter_state, int new_label);

    /// Derive a contour of a connected component
    /** @param number The number of the contour line to be derived
        @param component Set of laser points that are connected through edges
        @param edges The edges of the Delaunay triangulation
        @param same_attribute_value Constrain contour to points with same
                                    attribute value
        @param tag   Tag to be used for attribute retrieval
        @return The contour of the connected component
    */
    LineTopology DeriveContour(int number, const PointNumberList &component, 
                               const TINEdges &edges,
                               bool same_attribute_value=true,
                               const LaserPointTag tag=LabelTag) const;

    /// Derive a contour of a connected component, allowing to delete edges
    /** @param number The number of the contour line to be derived
        @param component Set of laser points that are connected through edges
                         of the Delaunay triangulation
        @param max_edge_length_factor Length of contour edge that may be deleted
                                      is derived by the product of the median
                                      point spacing and this factor
        @param max_edge_dist_factor   Maximum distance of new contour point to
                                      a contour edge is derived by the product
                                      of the median point spacing and this
                                      factor
        @param edges The edges of the Delaunay triangulation
        @return The contour of the connected component
    */
    LineTopology DeriveContour(int number, const PointNumberList &component, 
                               double max_edge_length_factor,
                               double max_edge_dist_factor, TINEdges &edges);

    /// Extract sequences of points with the same attribute
    /** @param polygon     Closed polygon of tagged laser points
        @param value       Attribute value of requested sequences
        @param tag         Attribute tag of requested sequences
        @param num_pts_gap Allowed gap size in number of points
        @param gap_size    Allowed gap size in meters
        @return            Sequences of laser point numbers with the specified
                           attribute value
    */
    LineTopologies & TaggedSequences(const LineTopology &polygon,
                                     int value,
                                     LaserPointTag tag=LabelTag,
                                     int num_pts_gap=0,
                                     double gap_size=0.0) const;
                                     
    /// Derive the holes contours of the connected component
    /** The labels of the contour points of the connected component should have
        been set to 2. The labels of points in holes are set to 3. The labels
        of the hole contours are set to 4.
        @param component A connected component of laser points
        @param edges The edges of the Delaunay triangulation
        @param contour The contour of the connected components
        @param same_label If true, only points with the same label as the first
                          point of the component are tracked.
        @return The contours of the holes in the connected component
    */
    LineTopologies DeriveHoles(const PointNumberList &component,
                               const TINEdges &edges,
                               const PointNumberList &contour);

    /// Size of a connected component
    /** Calculate the size of a component by summing up the sizes of all
        triangles in the component with the same label.
        @param component Connected component of laser points
        @param edges The edges of the Delaunay triangulation
        @param label Label of the points of the connected component
        @return The surface size of the connected component in the X0Y plane
    */
    double NeighbourhoodSize(const PointNumberList &component,
                             const TINEdges &edges, int label) const;
                             
    /// Unlabel small segments
    /** The segment tag of a point is removed if the number of points with the
        specified tag is below a given number.
        @param tag Tag used for storing the segment number
        @param minimum_size Minimum number of required points
    */
	void UnlabelSmallSegments(LaserPointTag tag, int minimum_size);
    
    /// Remove small segments
    /** A point is removed if the number of points with the
        specified tag is below a given number.
        @param tag Tag used for storing the segment number
        @param minimum_size Minimum number of required points
    */
	void RemoveSmallSegments(LaserPointTag tag, int minimum_size);
    
    /// Mark points on the convex hull
    /** The ConvexHullTag attribute is set to 1 for points on the hull and to
        0 for points in the interior of the TIN. If required, a TIN is derived.
        @param tag Tag used for storing hull marker
    */
    void MarkConvexHullPoints(LaserPointTag tag=ConvexHullTag);
    
/* Functions defined in the file LargeObjects.cc */

    /// Filter large objects
    /** The interior of buildings are usually not filtered away by morphological
        filtering. This function is to detect such situations and to filter
        the unfiltered building points by analysing the connected components
        of unfiltered points. If the difference between the average height of an
        unfiltered component (assumably on a building) and the median height of
        the unfiltered points (assumably ground points) attached to the
        attached filtered points (assumably points near the building outline)
        exceeds "min_object_height" and the linear size of the connected
        component is smaller than "max_object_size", all points in this
        component are filtered.
        @param stdev Standard deviation of the laser point heights
        @param tin Delaunay triangulation
        @param edges The edges of the Delaunay triangulation
        @param min_object_height Assumed minimum size of a roof above the ground
        @param max_object_size Assumed maximum linear size of a building
    */
    void FilterLargeObjects(double stdev, const TIN &tin, const TINEdges &edges,
                            double min_object_height, double max_object_size);

    /// Determine the bounds of an object
    DataBoundsLaser ObjectBounds(const PointNumberList &) const;

    /// Remove long edges from a TIN edge set
    /** All edges longer than a specified threshold are removed from the
        TINEdges. This function can be used to create contours with concave
        shapes out of a triangulated point cloud.
        @param edge The edges of the Delaunay triangulation
        @param max_dist Maximum distance of edges that are to be preserved
        @param planimetric Switch for using either 2D (default) or 3D distances
    */
    void RemoveLongEdges(TINEdges &edges, double max_dist,
                         bool planimetric=true) const;


    LaserPoints ExtractLongEdges(double max_dist,
                                  bool planimetric);

    /// Count the number of edges with points with two different label values
    int CountMixedEdges(TINEdges &edges, LaserPointTag label_tag) const;

    /// Return points on edges with two different label values
    LaserPoints ReturnMixedEdges(TINEdges &edges, 
                                 LaserPointTag label_tag) const;
                                                  
    /// Return points on edges of component
    LaserPoints ReturnContourPoints(int contour_number,
                                        const PointNumberList &component,
					                    const TINEdges &edges,
                                        bool same_attribute_value,
                                        const LaserPointTag tag) const;
                                        
/* Functions defined in the file AnalyseErrors.cc */

    /// Analysis of filtering errors in a histogram
    /** The erroneous DEM points are compared against the heights of "this"
        set of terrain laser points. The sizes of the errors are calculated
        and added to an histogram.
        @param errors Incorrectly classified laser points
        @param histogram Histogram of the errors
        @param error_type Either 1 or 2, for type 1 or type 2 errors.
    */
    void ErrorHistogram(const LaserPoints &errors, Image &histogram,
                        int error_type) const;

    /// Search TIN mesh around a point
    /** @param point Point for which the TIN mesh needs to be found
        @param start_mesh Mesh to start the search. Just use the first mesh
                          of the TIN if you don't have a good start point
        @param mesh_labels Array for storing search results. This array
                           should have been allocated and has a size of at
                           least the number of meshes of the TIN.
        @param meshnumber_list A vector of mesh numbers used locally.
        @return If the mesh is found a pointer to that mesh is returned,
                otherwise NULL is returned.
    */
    const TINMesh *SearchMesh(const LaserPoint *point,
                              const TINMesh *start_mesh, char *mesh_labels,
                              std::vector <MeshNumber> &meshnumber_list) const;

    /// Search mesh path to a point
    /** Function used by SearchMesh to find a mesh of the mesh corner that is
        closest to the point.
        @param point Point for which the TIN mesh needs to be found
        @param start_mesh Mesh to start the search. Just use the first mesh
        @return If a near mesh is found a pointer to that mesh is returned,
                otherwise NULL is returned.
    */
    const TINMesh *InitialSearchMesh(const LaserPoint *point,
                                     const TINMesh *start_mesh) const;

    /// Sequential search for a mesh
    /** All TIN meshes are tested. Optionally a start mesh for the search
        can be provided. If the mesh is not found in the remainder of the
        TIN, the search continues at the beginning of the TIN.
        @param point Point for which the TIN mesh needs to be found
        @param start_mesh Mesh to start the search. Just use the first mesh
    */
    const TINMesh *SimpleSearchMesh(const LaserPoint *point,
                                    const TINMesh *start_mesh) const;

/* Functions defined in the file SearchPoint.cc */

    /// Sort points on coordinates
    /** First sort after the X-coordinate. If X-coordinates are identical, 
        sort those points after the Y-coordinate. If Y-coordinates are also
        identical, sort those points after the Z-coordinate.
    */
    void SortOnCoordinates();

    /// Find point at the same position
    /** Find the pointer to the laser point with coordinates as in "position". 
        @param position Location of the laser point to be found
        @param start Suggested start point for the search (optional)
    */
    const LaserPoint *MatchXYZ(const LaserPoint *position,
                               const LaserPoint *start=NULL) const;

    /// Find point at the same position
    /** Find the pointer to the laser point with coordinates as in "position". 
        @param position Location of the laser point to be found
        @param start Suggested start point for the search (optional)
    */
    LaserPoint *MatchXYZ(LaserPoint *position, LaserPoint *start=NULL);

    /// Sort points along a line with increasing scalar
    /** @param line A 2D line along which the points should be sorted
        @param remove_scalars If true ScalarTag is removed after sorting
        @param use_given_scalars If true the line is not used for calculating scalars
    */
    void SortAlongLine(const Line2D &line, bool remove_scalars=true,
                       bool use_given_scalars=false);
    
/* Functions defined in the file FilterCheck.cc */

    /// Check the filter results
    /** The function FilterLargeObjects may sometimes remove large terrain
        parts. FilterCheck is a (too) simple way to try to correct for this.
        @param max_building_size Maximum linear size of a building
        @param kernel The morphological kernel used for filtering
        @param range The range over which the kernel has been applied
        @param stdev The standard deviation of the laser point heights
        @param tolerance Tolerance added to the maximum allowed height
                         differences
        @param edges The edges of the Delaunay triangulation
    */
    void FilterCheck(double max_building_size, const Image &kernel,
                     double range, double stdev, double tolerance,
                     const TINEdges &edges);


/* Functions defined in file LaserPointsB.cc */

    /// Fit a plane through the points with some attribute value
    /** Least squares fitting of a plane to all laser points with attribute
        values set to value1 or value2. There is no attempt to remove outliers.
        @param value1 Points with this attribute value are used to fit the plane
        @param value2 Points with this attribute value are also used to fit the 
                      plane
        @param tag    Attribute tag of which the value should be taken
        @return The fitted plane
    */
    Plane FitPlane(int value1, int value2,
                   const LaserPointTag tag=LabelTag) const;

    /// Fit a plane through the points with a specific long segment number
    /** Least squares fitting of a plane to all laser points with a specified
	    long segment number. There is no attempt to remove outliers.
        @param number The long segment number used to select the points
        @return The fitted plane
    */
    Plane FitPlane(long long int number) const;

    /// Calculates the noise of the plane parameters
	/** @param value Attribute value of points to be used
	    @param point_noise A priori standard deviation of points
	    @param tag Attribute to be used for point selection
	    @return Covariance matrix of plane parameters
	*/

    Matrix3 QualityPlane(int value, double point_noise,
                         const LaserPointTag tag=LabelTag) const;

    /// Fit a plane through the points in a list
    /** Least squares fitting of a plane to all laser points with numbers
        in the specified point number list. There is no attempt to remove
        outliers.
        @param list The list of point numbers to be used for plane fitting
        @param num  The plane number of the plane to be constructed.
        @return The fitted plane.
    */
    
    Plane FitPlane(const PointNumberList &list, int num) const;

    /// Label all points within some distance of a plane
    /** Relabel points with specific labels depending on their distance
        to a plane.
        @param plane      A plane
        @param max_dist   Threshold for the distance between a point and the
                          plane
        @param old_label1 Points with this label are to be relabelled
        @param old_label2 Points with this label also are to be relabelled
        @param near_label Points within max_dist of the plane get this label
        @param far_label  Points further than max_dist of the plane get this
                          label
        @param tag        Attribute tag to be used for storing the label
    */
    int Label(const Plane &plane, double max_dist, int old_label1,
              int old_label2, int near_label, int far_label,
              const LaserPointTag tag=LabelTag);

    /// Label all points of a point set
    /** @param list  All points of this point number list are relabelled
        @param label The new label for those points
        @param tag   Attribute tag to be used for storing the label
    */
    void Label(const PointNumberList &list, int label,
               const LaserPointTag tag=LabelTag);

    /// Label the points within a 2D polygon
    /** @param points   List of points including the nodes of the polygon
        @param topology Node numbers of the polygon
        @param label    New label for the points inside the polygon
        @param tag      Attribute tag to be used for storing the label
    */
    void LabelInside(const ObjectPoints2D &points,
                     const PointNumberList &topology, int label,
                     const LaserPointTag tag=LabelTag);

    /// Initialise the Hough space for labeled points
    /** Set up the Hough space to detect planes through laser points of a
        specified label. A maximum slope of the planes has to be specified.
        Bounds on the distance of the planar faces to the origin are extracted
        from the bounds of the points with the specified label. The Hough
        transform uses the plane equation Z = slope_x * X + slope_y * Y + d
        @param space     The Hough space that is created
        @param label     The label of the points for which the Hough space is set up
        @param slope_max Maximum slope of a plane to be extracted
        @param slope_bin Bin size for the two slope parameters
        @param dist_bin  Bin size for the distance parameter
        @param equation  Equation type for the Hough space, see class HoughSpace
        @param tag       Attribute tag to be used for retrieving the label
    */
    void InitialiseHoughSpace(HoughSpace &space, int label,
                              double slope_max, double slope_bin,
                              double dist_bin, int equation=0,
                              const LaserPointTag tag=LabelTag);

    /// Add all points to the Hough space
    void IncrementHoughSpace(HoughSpace &space) const;

    /// Add all points with some attribute value to the Hough space
    /** @param space  The Hough space
        @param value  Only add points with this attribute value
        @param tag    Attribute tag of which the value is to be used
    */
    void IncrementHoughSpace(HoughSpace &space, int value,
                             const LaserPointTag tag=LabelTag) const;

    /// Determine bounds of the points with some attribute value
    /** @param value Determine bounds of points with this attribute value
        @param tag   Attribute tag of which the value is to be used
    */
    DataBoundsLaser DeriveDataBoundsTaggedPoints
      (int value, const LaserPointTag tag=LabelTag) const;

    /// Check if points are in a plane
    /** A set of points is considered to ly in a plane if some minimum
        percentage of the points is within a specified distance from the plane.
        @param list     List of point numbers to be checked
        @param plane    A plane
        @param max_dist Distance threshold to decide if a point is in the plane
        @param min_perc Percentage threshold to decide if a sufficient number
                        of points is in the plane.
        @return 1 if at least min_perc percent of the points in the list is
                within max_dist of the plane, 0 otherwise.
    */
    int PointsInPlane(const PointNumberList &list, const Plane &plane,
                      double max_dist, double min_perc) const;

    /// Determine the centre of a point set
    /** @param list List of point numbers for which the centroid is to be
                    computed.
        @param number Point number of the centroid
        @return An object point with the coordinates of the computed centroid
    */
    ObjectPoint Centroid(const PointNumberList &list, int number) const;

    /// Intersect two faces
    /** Two planes are intersected. For the points of the corresponding two
        faces, the segment of the intersection line is determined for which
        both faces have points within a specified distance to the intersection
        line. The end points of this line segment are computed.
        @param face1    Point numbers of the points of the first planar face
        @param face2    Point numbers of the points of the second planar face
        @param plane1   The plane of the first face
        @param plane2   The plane of the second face
        @param max_dist Maximum distance of a point to the intersection line
                        used for the derivation of the end points of the
                        intersection segment
        @param sbegin   Begin position of the intersection segment
        @param send     End position of the intersection segment
        @return 1 if the planes intersect and the parts of the faces near the
                  intersection line overlap, 0 otherwise.
    */
    int IntersectFaces(const PointNumberList &face1,
                       const PointNumberList &face2,
                       const Plane &plane1, const Plane &plane2,
                       double max_dist,
                       Position3D &sbegin, Position3D &send) const;

    /// Determine line segment near face
    /** Given a set of points and a line, the begin and end position of the
        segment of this line is determined which has points within a specified
        distance. This function is used by IntersectFaces.
        @param face     Point numbers of the laser points
        @param line     The 3D line
        @param max_dist Maximum distance between a point and the line for a
                        point to be considered a nearby point
        @param sbegin   Scalar of the begin point of the line segment
        @param send     Scalar of the end point of the line segment
        @return The number of points within max_dist of the line
    */
    int FaceNearLine(const PointNumberList &face, const Line3D &line,
                     double max_dist,
                     double &sbegin, double &send) const;

    /// Derive normal vector at the position of a laser point
    /** A point list is made of the specified point and its neighbouring points
        in the Delaunay triangulation. A plane is fitted to all points of this
        list and the plane's normal is returned.
        @param node The number of the point for which the surface normal is to
                    be computed
        @param edges The edges of the Delaunay triangulation
        @return The normal vector of the fitted plane.
    */
    Vector3D NormalAtPoint(const PointNumber &node,
                           const TINEdges &edges) const;

    /// Select a subset of points within polygons
    /** All laser points within the given polygons are added to the selection.
        To speed up the inside-polygon computations, it is first checked
        whether the bounds of the laser points overlap with the bounds of the
        polygons. For this purpose, the bounds of the laser points are computed
        if they are not yet available. Therefore this function is not a const
        function.
        @param selection The set of laser points to which the points within the
                         polygons are added
        @param points    The corner points of the polygons
        @param topology  The node numbers of the corners of the polygons
        @return The number of points added to the selection.
    */
    int Select(LaserPoints &selection, const ObjectPoints &points,
               const LineTopologies &polygons, bool set_IsSelectedTag=false);

    /// Determine the nearest laser point using neighbourhood edges
    /** @param pos The 3D location where to look for the nearest laser point
        @param edges Edges (from a TIN or k-d tree) to guide the search
        @param planimetric If true, use 2D distances, otherwise 3D
        @return The index of the nearest point.
    */
    int NearestPoint(const Position3D &pos, const TINEdges &edges,
                           bool planimetric=true) const;
   
    /// Determine the distance to the nearest laser point.
    
    double Distance2NearestPoint(Position3D pos, int nearestpoint);
                           
                           

    /// Return the next point of a certain pulse type
    /** @param type Pulse type to be found
        @param start_point Start location for the search
        @return const iterator of located point of specified pulse type or
                end() if point was not found
    */
    LaserPoints::const_iterator 
      NextPointOfPulseType(LaserPulseType type,
                           LaserPoints::const_iterator start_point) const;
                           
    /// Determine the nearest laser point with a specific attribute value
    /** @param pos Position for which the nearest point is to be found
        @param value The attribute value that the nearest point should have
        @param nearest_point Const iterator to the neareast laser point with the
                             specified label
        @return 1 if a point with the specified label was found, 0 otherwise.
        @param tag Tag to be used for retrieving the attribute value
    */
    int NearestTaggedPoint(const Position2D &pos, int value,
                           const_iterator &nearest_point,
                           const LaserPointTag tag=LabelTag) const;

    /// Collect all points in the neighbourhood with a specific attribute value
    /** First all points within a specified range of the given centre point are
        selected. Second, all points with attribute values other than the
        specified value are removed from the point list.
        @param centre Point number of the laser point around which the labelled
                      neighbourhood is to be determined.
        @param value  The attribute value that the selected points in the
                      neighbourhood should have.
        @param range  The maximum radius of the neighbourhood around the centre.
        @param edges  The edges of the Delaunay triangulation.
        @param tag    Attribute tag to be used for retrieving the attribute value
        @param planimetric If true, use 2D distances for range, otherwise 3D
        @param direct_neighbours_only If true, only points connected through
                                      the centre by edges are considered
        @return The points within range of centre with the specified label.
    */
    PointNumberList TaggedNeighbourhood(const PointNumber &centre, int value,
                                        double range, const TINEdges &edges,
                                        const LaserPointTag tag=LabelTag,
                                        bool planimetric=true,
                                        bool direct_neighbours_only=false) const;

    /// Collect all points with a specific attribute value
    void TaggedPointNumberList(PointNumberList &list,
                               const LaserPointTag tag, int value) const;
    
    /// Collect all points with a specific attribute value
    PointNumberList & TaggedPointNumberList(const LaserPointTag tag,
                                            int value) const;
    
    /// Derive TIN edges for 3D connectivity of a point cloud
    TINEdges * TINEdges3D(double range);

    /// Derive 3D neighbourhoods for each point
    TINEdges * NeighbourhoodEdges3D(const TINEdges &edges, double range) const;

    /// Swap all X and Y coordinates
    void SwapXY();

    /// Swap all X and Z coordinates
    void SwapXZ();

    /// Swap all Y and Z coordinates
    void SwapYZ();

    /// Label connected components determined by specified edges of connected points (old function)
    void LabelComponents(const TINEdges &edges,
                         bool store_label_in_reflectance);

    /// Label connected components determined by specified edges of connected points
    /** @param edges Edges of the TIN
        @param tag   Tag to be used for storing the component id's
        @param erase_old_component_labels If false, only points without a label are used to
                      determine new connected components
    */
    void LabelComponents(const TINEdges &edges,
                         const LaserPointTag tag=LabelTag,
						 bool erase_old_component_labels=true);

    /// Select points based on range of attribute value
    /** The IsSelectedTag is set for points that are between the minimum and
        maximum values for a specified attribute. Three operations allow
        combinations with previous selections.
        @param attribute Attribute of which the value should be checked
        @param mimimum Minimum attribute value
        @param maximum Maximum attribute value
        @param operation 0: Initial selection, 1: AND with current selection
                         2: OR with current selection
    */
    void Select(LaserPointTag attribute, double minimum, double maximum,
                int operation=0);

    /// Derive attributes of points 
    /** Used to compute one of the attributes normal vector components
	    NormalXTag, NormalYTag, NormalZTag, or derived attributes
	    FlatnessTag, LinearityTag (ased on eigenvalue ratios) and normals
	    scaled by flatness: ScaledNormalXTag, ScaledNormalYTag, ScaledNormalZTag.
	    @param attribute Attribute to be derived
	    @param parameters Segmentation parameters
    */
    void DerivePointAttribute(LaserPointTag attribute,
	                          const SegmentationParameters &parameters);
    
    /// Derive attributes of points with the same segment number
    /** @param attribute Segment attribute to be computed
        @param segment_tag Tag with the segment numbers
    */
    void DeriveSegmentAttribute(LaserPointTag attribute,
	                            LaserPointTag segment_tag);

    /// Derive attributes of points with the same long segment number, including tile number
    /** @param attribute Segment attribute to be computed
    */
    void DeriveSegmentAttribute(LaserPointTag attribute);

/* Functions defined in library RoofSegm */

    /// Reconstruct a building model from laser data and a ground plan
    int ReconstructBuilding(
      // Intermediate and final results
      TINEdges &, Planes &, PointNumberLists &,
      LineTopologies &, Planes &, PointNumberLists &, LineTopologies &,
      ObjectPoints2D &, LineTopologies &, ObjectPoints2D &, LineTopologies &,
      ObjectPoints2D &, LineTopologies &, ObjectPoints2D &, LineTopologies &,
      ObjectPoints &, LineTopologies &,
      // Control parameters
      bool, int, double, double, double, double, int, int, double, double, 
      double, double, double, double, double, double, double, double, double,
      double, double,
      // Ground height
      double, 
      // Output files
      char *, char *, char *, char *, char *, char *, char *);

    /// Output of rough contours of faces
    void Rough_Contours(PointNumberLists &, Planes &, TINEdges &, char *,
                        char *, LineTopologies &);

    /// Extract the best face from the Hough space
    int ExtractFace(HoughSpace &, int, double, int, TINEdges &,
                    int, Plane &, PointNumberList &);

    /// Determine adjacency of two connected components
    int Adjacent(PointNumberList &, int, TINEdges &);

    /// Merge faces if they are adjacent and in the same plane
    void MergeFaces(PointNumberLists &, Planes &, double, double, TINEdges &);

    /// Grow all faces
    void FaceGrowing(PointNumberLists &, Planes &, double, int, TINEdges &);

    /// Grow a face
    void GrowFace(int, PointNumberLists &, Planes &, double, TINEdges &);

    /// Fit face bounds to partition bounds and intersection lines
    void FitFacesToPartitions(PointNumberLists &, Planes &, double,
                              ObjectPoints2D &, LineTopologies &,
                              ObjectPoints &, LineTopologies &,
                              TINEdges &);

    /// Fit face bounds to one partition bound and intersection lines
    void FitFacesToPartition(PointNumberLists &, Planes &, double,
                             ObjectPoints2D &, LineTopology &,
                             ObjectPoints &, LineTopologies &,
                             TINEdges &);

    /// Construct the walls of a building
    void ConstructWalls(ObjectPoints &, LineTopologies &, int, double);

    /// Align the face bounds to the partition edges
    void AlignFacesToPartitions(PointNumberLists &, Planes &,
                                ObjectPoints2D &, LineTopologies &,
                                ObjectPoints &, LineTopologies &,
                                TINEdges &, double, double, double, double,
                                double, double, double, double, double, double,
                                ObjectPoints2D &, LineTopologies &,
                                ObjectPoints2D &, LineTopologies &);

    /// Locate height jump edges in a partition
    LineSegments2D HeightJumpsInPartition(ObjectPoints2D &, LineTopology &,
                                          LineSegments2D &, LineTopologies &,
                                          Planes &, TINEdges &,
                                          double, double, double, double);

    /// Locate intersection edges in the partitions
    LineSegments2D IntersectionEdgesInPartitions
       (PointNumberLists &, Planes &, ObjectPoints2D &, LineTopologies &, 
        double, double, double, double);

    /// Try to split a partition based on intersection and height jump lines
    bool SubPartition(ObjectPoints2D &, LineTopologies &, int,
                      LineSegments2D &, LineTopologies &, Planes &,
                      TINEdges &, LineSegments2D &, double, double,
                      double, double, double, double);
                  
    /// Construct a roof face
    bool ConstructRoofFace(ObjectPoints2D &, LineTopology &, Planes &,
                           ObjectPoints &, LineTopologies &);

    /// Estimate parallel and horizontal faces
    void ParallelFaces(PointNumberLists &, Planes &, double, double, double);
	LineSegment2D FitLineSegment(const PointNumberList& pnlPnts) const;
	LineSegment2D FitLineSegment(const PointNumberList& pnlPnts, const Vector2D& dir) const;

    /// Fit an edge model to the laser points
    LineSegment2D FitEdge(const LineSegment2D, double, double &, double &, int);

    /// Robustly fit a polynomial through the points in a list
    double FitPolynomial(const Position2D &origin, const PointNumberList &list,
                         double &coef, int order, const Vector2D &direction,
                         double lambda1, double lambda2) const;

    /// Fit a plane with a fixed orientation
    Plane FitPlane(const PointNumberList &face, const Vector3D &orientation,
                   int plane_number) const;

    /// Fit a roof model to the laser oints
    vMatrix FitRoof(const LineSegments2D &walls, int model,
                    double &rms, int max_iter, bool &success) const;

    /// Determine the minimum enclosing rectangle
    bool EnclosingRectangle(double max_edge_dist, ObjectPoints &points,
                            LineTopology &polygon) const;

    /// Determine the enclosing polygon
    bool EnclosingPolygon(ObjectPoints &points, 
                          LineTopology &enclosing_polygon,
                          const SegmentationParameters &segmentation_pars,
                          const OutliningParameters &outlining_pars,
                          bool use_sloped_segments);

    /// Determine the smallest enclosing circle
    Circle2D EnclosingCircle() const;

    /// Fit a plane through the points in a list
    /** Function used for new fitting algorithm based on eigen vector analysis
    */
    /** Least squares fitting of a plane to all laser points with numbers
        in the specified point number list. There is no attempt to remove
        outliers.
        @param list The list of point numbers to be used for plane fitting
        @return The fitted plane.
    */
    Plane FitPlane(const PointNumberList &list) const;

    /// Fit a line through a sequence of points
    /** Least squares fit of a line through a sequence of points
        @param first_point Iterator of first point
        @param last_point  Iterator of last point
        @return The fitted line
    */
    Line3D FitLine(LaserPoints::const_iterator first_point,
	               LaserPoints::const_iterator last_point) const;
	               
    /// Return the centre of gravity of the points in the list
    Position3D CentreOfGravity(const PointNumberList &list) const;
    
    /// Segmentation of the point cloud by surface growing
    /** Segmentation of a point cloud into planar or smooth surfaces.
        @param parameters See the SegmentationParameters class for explanation
        @param delete_edges If false, derived neighbourhood edges will be preserved
        @param verbose If true, progress output will be written to stdout
        @param segmentation_planes If not NULL, estimated segment planes
               can be accessed through this pointer. Note that the Planes
               variable should have been allocated.
        @return true if successful
    */ 
    bool SurfaceGrowing(const SegmentationParameters &parameters,
                        bool delete_edges=true,
                        bool verbose=true,
                        Planes *segment_planes=NULL);
    
    /// Segmentation of the point cloud by grouping points with similar attribute values
    /** Segmentation of a point cloud segments with similar attribute values.
        @param parameters See the SegmentationParameters class for explanation.
        @param first_segment_number Number of the first detected segment
        @param delete_edges If false, derived neighbourhood edges will be preserved
        @param verbose If true, progress output will be written to stdout
        @param tile_number Will be set as SegmentStartTileNumberTag attribute
                           for new segments if >= 0
        @return true if successful
    */ 
    bool SegmentGrowing(const SegmentationParameters &parameters,
                        int first_segment_number,
                        bool delete_edges=true, bool verbose=true,
						int tile_number=-1);


    /// Mean shift segmentation
    /** Mean shift segmentation of a point cloud. Attributes and bandwidths of
        attributes and coordinates are passed through the variable parameters.
        For larger point clouds, mean shift can be very time consuming.
        @param parameters See the SegmentationParameters class for explanation.
        @param first_segment_number Number of the first detected segment
        @param verbose If true, some output on progress is given.
    */
    void MeanShift(const SegmentationParameters &parameters,
                   int first_segment_number, bool verbose);
                        
    /// Determine tag value for points without this tag based on tags in their neighbourhood
    /** Majority filtering on an attribute value for those points that do not have
        a value of the specified attribute
        @param parameters See the SegmentationParameters class for explanation.
        @param edges Edges to the neighbouring points
    */
    void MajorityFilter(const SegmentationParameters &parameters,
                        const TINEdges &edges);

    /// Merge nearly coinciding neighbouring planar segments
    /** Merge neighbouring segments if the angle between the normal
        vectors is small and the points of both segments (within a neighbourhood)
        are close to the plane of the other segment.
        @param parameters See the SegmentationParameters class for explanation.
    */
    void MergeSurfaces(const SegmentationParameters &parameters);

    /// Select points with a certain tag
    /** @param tag Only points with this tag will be selected
        @return Number of selected points
    */
    int SelectByTag(const LaserPointTag tag, LaserPoints &selection) const;
    
    /// Select points without a certain tag
    /** @param tag Only points without this tag will be selected
        @return Number of selected points
    */
    int SelectByNotTag(const LaserPointTag tag, LaserPoints &selection) const;
    
    /// Select by Tag with certain integer value
    LaserPoints SelectTagValue(const LaserPointTag tag, int value) const;
    
    /// Select by Tag with certain integer value, return PointNumberList
    PointNumberList SelectTagValueList(const LaserPointTag tag, int value) const;
    
    /// Select by PulseType
    LaserPoints SelectLastPulse() const;
    
    /// Determine the most frequent attribute value of the specified attribute
    /** @param tag Attribute for which the most frequent value is to be
                   determined
        @param count The count of the most frequent attribute value
        @return The most frequent value
    */
    int MostFrequentAttributeValue(LaserPointTag tag, int &count) const;

    /// Determine the most frequent attribute value of the specified attribute
    /** @param tag Attribute for which the most frequent value is to be
                   determined
        @param list List of point numbers to be considered
        @param count The count of the most frequent attribute value
        @param minimum_value Ignore attribute values lower than this value
        @return The most frequent value
    */
    int MostFrequentAttributeValue(LaserPointTag tag,
	                               const PointNumberList &list,
	                               int &count,
	                               int minimum_value=0) const;
    
    /// Determine the most frequent attribute value of the specified long attribute
    /** @param tag Attribute for which the most frequent value is to be
                   determined
        @param list List of point numbers to be considered
        @param count The count of the most frequent attribute value
        @param minimum_value Ignore attribute values lower than this value
        @return The most frequent value
    */
    long long int MostFrequentLongAttributeValue(LaserPointTag tag,
	                                             const PointNumberList &list,
	                                             int &count,
	                                             int minimum_value=0) const;
    
    /// Check if laserpoints are segmented and have RGB values; give median value to whole segment
    LaserPoints ReColorPointsPerSegment();
    
    double ReturnHeightOfPercentilePoint(int perc);
    
    double ReturnDifferenceToGutterHeight(int segment_number, double gutterheight);
    
    /// Calculate the convex hull of a laser segment 
    /** @param objpts Output ObjectPoints
        @param tops Output LineTopologies
        @param max the biggest segment number
    */

    void ConvexHull(ObjectPoints &objpts, LineTopologies &tops, int max) const;
    
    /// Determine the contour points in 3D
    /** @param contour_obj Output ObjectPoints
        @param contour_top Output LineTopologies
        @param max_edge_dist filter for TIN edges, bigger gives more detail
    */
    void DeriveContour3D(ObjectPoints &contour_obj, LineTopology &contour_top, double max_edge_dist=0.2) const;
    
    
    /// Detection of a line using RANSAC
    /** Detection of a 3D line in a point cloud using RANSAC.
        @param max_dist_to_line Maximum distance of a point to the detected line
        @param min_num_hits Minimum number of points close to the line for successful detection
        @param max_num_tries Maximum number of RANSAC iterations
        @param num_hits Number of points near the line if at least min_num_hits
        @param select_tag If unequal UndefinedTag, only points with value
                          select_value of tag select_tag are used for the line detection
        @param select_value See select_tag
    */
    Line3D RANSAC3DLine(double max_dist_to_line, int min_num_hits,
                        int max_num_tries, int &num_hits, 
                        double max_dist_between_points, LineSegments3D &segments,
                        LaserPointTag select_tag=UndefinedTag,
                        int select_value=0) const;
    
	Line3D RANSAC3DRailTrack(double max_dist_to_line, double gauge_distance,
		int min_num_hits,
		int max_num_tries, int &num_hits, 
		double max_dist_between_points, LineSegments3D &segments,
		LineSegments3D &secondsegments,
		LaserPointTag select_tag=UndefinedTag,
		int select_value=0) const;

	LineSegments2D RANSAC2DLineParallel(double max_dist_to_line, int min_num_hits,
		int max_num_tries, int &num_hits,
		double max_dist_between_points, double minimum_length,
		LineSegment2D prefline,
		LaserPointTag select_tag=UndefinedTag,
		int select_value=0) ;
	
    ///Determine the minimum bounding circle
    /**Determine the minimum bounding circle based on the gravity centre, assuming a equally distributed points
    */
    Circle2D MBC();
   
    /// Return topological relations between two neighbouring segments
    void ReturnTopologicalRelations(TINEdges &edges, 
                        LaserPointTag label_tag, ObjectPoints &obj, 
                        LineTopologies &top) const;
                        
     void DeriveTopologicalRelations(ObjectPoints &obj, LineTopologies &top, double max_dist) const;
    
/// Return number of topological relations in output laserpoints
    void ReturnNumberOfTopologicalRelations(LaserPointTag inlabel_tag, 
                        LaserPoints &outpoints,
                        ObjectPoints objpts,
                        LineTopologies tops) const;
                                                                      
   
	///mesh simplification; by checking if point can me removed
  	LaserPoints SimplifyMesh(double max_dist);   

	///mesh simplification; but keep the points with a certain labelvalue
    LaserPoints SimplifyMesh_KeepLabel(double max_dist, const LaserPointTag tag, int keeplabel);       

	///remove points that are withing a certain distance    
    int RemoveAlmostDoublePoints(bool only_check_X_and_Y=true, double dist=0);
  
/// TEMPORARY FUNCTIONS FROM SUDAN

    int ReTag(int new_value, const LaserPointTag new_tag, 
	          int value, const LaserPointTag tag);
    
    bool DeriveConvexhull(LaserPoints &points);
    
    bool EnclosingRectangle_3D(double max_height,ObjectPoints &points, 
                               LineTopology &polygon);
                               
/// END OF SUDAN'S FUNCTIONS
    
    private:
            
    void FindHull(LaserPatches &lps_tmp,LaserPatch &sk, Position3D P, Position3D Q, int num, int flag, int vertical) const;

    ///Iteration function for ConvexHull, you should never use this.       
  //  void FindHull(LaserPatches &lps_tmp,LaserPatch &sk, Position3D P, Position3D Q, int num, int flag, int vertical) const;
    
 

};

#endif /* _LaserPoints_h_ */  /* Don't add after this point */
