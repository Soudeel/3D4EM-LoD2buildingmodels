
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
 * \brief Interface to Class Circle2D
 *
 */
/*!
 * \class Circle2D
 * \ingroup Photogrammetry
 * \brief Interface to Class Circle2D
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

#ifndef _Circle2D_h_
#define _Circle2D_h_

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///                            A plane
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <math.h>
#include "Position2D.h"
#include "FeatureNumber.h"
#include "ObjectPoints.h"
#include "LineTopology.h"
		
class Circle2D : public FeatureNumber
{
  protected:

        /// Centre
  	Position2D centre;

        /// Radius
  	double     radius;

        /// Label of the circle
        int      label;
 	
  public:
        /// Default constructor
  	Circle2D() : FeatureNumber()
 	  {centre = Position2D(0.0, 0.0); radius = 1.0; label=0;}

	/// Copy constructor
        Circle2D(const Circle2D &circle) : FeatureNumber()
	  { *this = circle; } 
	  		
        /// Construct circle from three positions
        Circle2D(const Position2D &p1, const Position2D &p2,
                 const Position2D &p3, bool &success);

        /// Construct from a centre position and a radius
        Circle2D(const Position2D &pos, double rad);

        /// Default destructor
  	~Circle2D() {};

        /// Copy assignment
        Circle2D& operator=(const Circle2D& circle); 
  	
        /// Return the readable reference
        const Circle2D &Circle2DReference() const
          {return(*this);}

        /// Return the writable reference
        Circle2D &Circle2DReference()
          {return(*this);}

        /// Return the readable label
        int Label() const
          { return label; }

        /// Return the writable label
        int &Label()
          { return label; }

        /// Return the readable radius
        double Radius() const
          { return radius; }

        /// Return the writable radius
        double &Radius()
          { return radius; }

        /// Return the readable centre
        const Position2D& Centre() const
          { return centre; }

        /// Return the writable centre
        Position2D &Centre()
          { return centre; }

        /// Determine the signed distance of a point to the circle
        /** If the distance is negative, the point is inside the circle.
        */
        double SignedDistance(const Position2D &pos) const;

        /// Determine the distance of a point to the circle
        double Distance(const Position2D &pos) const
          {return(fabs(SignedDistance(pos)));}

        /// Determine if a position is inside the circle
        bool Inside(const Position2D &pos) const
          {return(SignedDistance(pos) <= 0.0);}

        /// Create a polygon approximating the circle
        /** @param num_nodes Number of points that should be generated to
                             approximate the circle
            @param points    Object points that approximate the circle are added
                             to this point vector. 
            @param polygon   Node numbers of the polygon are added to this
                             LineTopology. If polygon already contains node
                             numbers, these numbers are erased.
        */
        void Polygon(int num_nodes, ObjectPoints &points,
                     LineTopology &polygon) const;

};
#endif /* _Circle2D_h_ */  /* Don't add after this point */
