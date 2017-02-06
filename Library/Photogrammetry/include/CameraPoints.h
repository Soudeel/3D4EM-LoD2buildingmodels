
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
 * \brief Interface to Class CameraPoints - A set of stochastic camera points
 *
 */
/*!
 * \class CameraPoints
 * \ingroup Photogrammetry
 * \brief Interface to Class CameraPoints - A set of stochastic camera points
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


#ifndef _CameraPoints_h_
#define _CameraPoints_h_

/*
--------------------------------------------------------------------------------
  CameraPoints  - A set of stochastic camera points
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                  Include files
--------------------------------------------------------------------------------
*/
#include "Database4Cpp.h"
#include "CameraPoint.h"
#include "VectorPoint.h"

class ObjectPoints;
class ControlPoints;
class ImagePoints;
class InteriorOrientation;

//------------------------------------------------------------------------------
/// A vector of camera points
//------------------------------------------------------------------------------

class CameraPoints : public VectorPoint<CameraPoint>{

  protected:

  public:

    /// Default constructor
    CameraPoints() : VectorPoint<CameraPoint>() {}

    /// Construct by reading camera points from a file
    /** @param filename Name of the camera points file
        @param success  Return value of read function: 0 = failure, 1 = success
    */
    CameraPoints(char *filename, int *success)
      {*success = Read(filename);};

    /// Construct by converting a C object
    CameraPoints(CamPts *campts)
      {C2Cpp(campts);};

    /// Construct by projecting object points into the camera coordinate system
    /** The collinearity equations are used for the projection.
        @param objpts Object points to be projected into the camera coordinate
                      system
        @param intor  Interior orientation of the camera
        @param extor  Exterior orientation of the camera
    */
    CameraPoints(const ObjectPoints *objpts, const InteriorOrientation *intor,
                 const ExteriorOrientation *extor);

    /// Construct by projecting control points into the camera coordinate system
    /** The collinearity equations are used for the projection.
        @param ctrlpts Object points to be projected into the camera coordinate
                       system
        @param intor   Interior orientation of the camera
        @param extor   Exterior orientation of the camera
    */
    CameraPoints(const ControlPoints *ctrlpts, const InteriorOrientation *intor,
                 const ExteriorOrientation *extor);

    /// Construct by transforming image points into the camera coordinate system
    /** This transformation converts the pixel coordinates to
        coordinates in mm and corrects for the lens distortion.
        @param imgpts Image points
        @param intor  Interior orientation
    */
    CameraPoints(const ImagePoints *imgpts, const InteriorOrientation *intor);

    /// Copy constructor
    CameraPoints(const CameraPoints &pts)
       { insert(begin(), pts.begin(), pts.end()); }

    /// Default destructor
    ~CameraPoints() {};

    /// Copy assignament	
    CameraPoints& operator=(const CameraPoints &pts)
       { if (!empty()) erase(begin(), end());
         insert(begin(), pts.begin(), pts.end());
         return *this;  }

    /// Conversion from C to C++ object
    void C2Cpp(CamPts *);

    /// Conversion from C++ to C object
    void Cpp2C(CamPts **) const;

    /// Read camera points from a file
    /** @param filename Name of the file with the camera points
        @return 0 = failure, 1 = success.
    */
    int Read(char *filename);

    /// Write camera points to a file
    /** @param filename Name of the file to be written
        @return 0 = failure, 1 = success.
    */
    int Write(char *filename) const;

    /// Print the camera points to stdout
    void Print() const;

    /// Project object points into the camera coordinate system
    /** The collinearity equations are used for the projection.
        @param objpts Object points to be projected into the camera coordinate
                      system
        @param intor  Interior orientation of the camera
        @param extor  Exterior orientation of the camera
    */
    void Object2CameraPoints(const ObjectPoints *objpts,
                             const InteriorOrientation *intor, 
    	                     const ExteriorOrientation *extor);

    /// Project control points into the camera coordinate system
    /** The collinearity equations are used for the projection.
        @param ctrlpts Object points to be projected into the camera coordinate
                       system
        @param intor   Interior orientation of the camera
        @param extor   Exterior orientation of the camera
    */
    void Control2CameraPoints(const ControlPoints *ctrlpts,
                              const InteriorOrientation *intor, 
    	                      const ExteriorOrientation *extor);

    /// Transform image points into the camera coordinate system
    /** This transformation converts the pixel coordinates to
        coordinates in mm and corrects for the lens distortion.
        @param imgpts Image points
        @param intor  Interior orientation
    */
    void Image2CameraPoints(const ImagePoints *imgpts,
                            const InteriorOrientation *intor); 

    /// Transform camera points into the image coordinate system
    /** This transformation converts the mm coordinates to coordinates in 
        pixels and adds the lens distortion.
        @param intor  Interior orientation
        @return       A pointer to a vector of image points
    */
    ImagePoints *Camera2ImagePoints(const InteriorOrientation *intor) const;   
};
#endif /* _CameraPoints_h_ */   /* Do NOT add anything after this line */
