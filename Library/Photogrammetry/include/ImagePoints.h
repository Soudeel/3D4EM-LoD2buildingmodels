
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
 * \brief Interface to Class ImagePoints - A set of stochastic image points
 *
 */
/*!
 * \class ImagePoints
 * \ingroup Photogrammetry
 * \brief Interface to Class ImagePoints - A set of stochastic image points
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _ImagePoints_h_
#define _ImagePoints_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  ImagePoints  - A set of stochastic image points

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                  Include files
--------------------------------------------------------------------------------
*/

#include "VectorPoint.h"
#include "ImagePoint.h"
#include "Database4Cpp.h"

class CameraPoints;
class InteriorOrientation;

//------------------------------------------------------------------------------
/// A vector of image points
//------------------------------------------------------------------------------

class ImagePoints : public VectorPoint<ImagePoint>
{
    /// Name of the data set
    char *name;
    	
  public:

    /// Default constructor
    ImagePoints() : VectorPoint<ImagePoint>()
      { name = NULL; } 
    
    /// Construct by reading image points from a file
    /** @param filename File with image points
        @param success  Success status of the read function,
                        0 - failure, 1 - success.
    */
    ImagePoints(char *filename, int *success) : VectorPoint<ImagePoint>()
      { *success = Read(filename); }

    /// Construct by converting a C object
    ImagePoints(ImgPts *imgpts) : VectorPoint<ImagePoint>()
      { C2Cpp(imgpts); }
    
    /// Construct by transforming camera points into image points
    /** This transformation converts the mm coordinates to coordinates in
        pixels and adds the lens distortion.
        @param campts A vector of camera points
        @param intor  Interior orientation
    */
    ImagePoints(const CameraPoints *campts, const InteriorOrientation *intor)
      : VectorPoint<ImagePoint>()
      { Camera2ImagePoints(campts, intor); } 

    /// Destructor
    ~ImagePoints() 
       { delete [] name; }

    /// Conversion from C to C++ object
    void C2Cpp(ImgPts *);

    /// Conversion from C++ to C object
    void Cpp2C(ImgPts **) const;

    /// Read the image points from a file
    /** @param filename File with image points
        @return         Success status, 0 - failure, 1 - success
    */
    int Read(char *filename);

    /// Write the image points to a file
    /** @param filename File for the image points
        @return         Success status, 0 - failure, 1 - success
    */
    int Write(char *filename) const;

    /// Print the image points to stdout
    void Print() const;
    
    /// Return the name of the data set
    const char *Name() const
    	{ return name; }	    

    /// Transform the camera points into image points
    /** This transformation converts the mm coordinates to coordinates in
        pixels and adds the lens distortion.
        @param campts A vector of camera points
        @param intor  Interior orientation
    */
    void Camera2ImagePoints(const CameraPoints *campts,
                            const InteriorOrientation *intor);

    /// Transform the image points into camera points
    /** This transformation converts the pixel coordinates to
        coordinates in mm and corrects for the lens distortion.
        @param intor  Interior orientation
        @return       A pointer to a vector of camera points
    */
    CameraPoints *Image2CameraPoints(const InteriorOrientation *intor) const;
};

#endif /* _ImagePoints_h_ */   /* Do NOT add anything after this line */
