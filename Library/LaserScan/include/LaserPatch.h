
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
 * \brief ?????
 *
 */
/*!
 * \ingroup LMetaData
 * \brief ?????
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \S_Pu
 * \date		---- 2007-2-2
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

/*
 * $Log$
 * Revision 1.6  2010/07/09 08:39:59  WINDOWSNT\rutzinger
 * update of folders Library and Tools
 * - removal of NR tags in Makefiles and dev-files
 * - adding GNU Licence header to *.dev, *.cc, *.cpp, *.c, *.win, *.linux, *.h
 *
 * Revision 1.5  2010/06/15 14:47:15  WINDOWSNT\vosselman
 * Prepared for adding copyright texts
 *
 * Revision 1.4  2008/10/08 14:05:00  WINDOWSNT\vosselman
 * Bug fix, comment to previous update caused compiler error
 *
 * Revision 1.3  2008/10/08 13:46:30  WINDOWSNT\vosselman
 *
 * Added missing and of comment block *
 * Revision 1.2  2008/06/26 12:16:01  WINDOWSNT\spu
 * no message
 *
 * Revision 1.1  2007/02/02 16:45:43  WINDOWSNT\spu
 * Add of LaserPatch and LaserPatches (combination of objectpoint, linetopologies and laserpoints)
 */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<<
   >>>>
   >>>>       Purpose: Definitions and prototypes for library LaserData
   >>>>
   >>>>       Toolbox: $ALTIMETRY
   >>>>
   >>>>    Written By: Shi Pu
   >>>>
   >>>>          Date: 
   >>>>
   >>>> Modifications:
   >>>>
   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>> <<<<<<<<<<<<<<<<<<<<<<<<<< */




#include <vector>
#include "Positions3D.h"
#include "Plane.h"
#include "LineTopology.h"
#include "ObjectPoints.h"

#ifndef _LaserPatch_h_
#define _LaserPatch_h_


class LaserPatch : public Positions3D
{
      public:
             /// Default constructor       
             LaserPatch() {};
             
             LaserPatch(int new_size)
	         {
		      resize(new_size);
             }
        
             /// Copy constructor
	         LaserPatch(const LaserPatch &p) : Positions3D()
             {
             insert(begin(), p.begin(), p.end());             
             myplane=p.getPlane();
             //if(p.points.size())
             //{points=p.points;}
             } 
             ///Construct from objectpoints and linetopology
             LaserPatch( const ObjectPoints &objpts,const LineTopology &top)
             {
             int size;
             
             for(unsigned int i=0;i<top.size();i++)
             {
                push_back(objpts[top[i].Number()]);     
             }
             size=top.size();
             
             if(size>=3)
              {
              for(int i=0;i<size;i++)
                 myplane.AddPoint((*this)[i],false);
              myplane.Recalculate();  
              }           
             }
      
             
             ///Construct from objectpoint and linetopology, then add laserpoints info 
            /* LaserPatch( const ObjectPoints &objpts,const LineTopology &top)
             {
                LaserPatch(objpts,top);
                this->AddLaserPoints(lpts);    
             }
             */
             ///Copy from another , then add laserpoints info 
            /* LaserPatch(const LaserPatch &p, const LaserPoints &lpts) : Positions3D()
             {
             insert(begin(), p.begin(), p.end());             
             myplane=p.getPlane();
             //this->AddLaserPoints(lpts);
             }*/ 
             
             /// Default destructor 
             ~LaserPatch() {};
             
             
             /// return the 3D area of this patch
             double area3D() const;
             
             /// return the plane of this patch
             Plane getPlane() const;
             /// set the plane of this patch
             void setPlane(Plane &p);
             
             
             
             /// Whether 2 laserpatches intersect
             /** @param pie The other laserpatch to detect intersection with
        @param error threshold value
    */
    
   
             bool Intersect(LaserPatch pie, double error);
             
             /// Distance between 2 laserpatches
               /** @param pie The other laserpatch to detect distance with
    */
             double Distance(LaserPatch pie);
             
             ///Whether this laserpatch is vertical
               /** @param tolerance tolerance value
    */
             
             ///Whether this laserpatch is on ground
              /** @param ground The height value of ground
              @param tolerance tolerance value
    */
             bool OnGround(double ground,double error);
             
             ///Attach (new) laserpoints to this laserpatch
              /** @param lpts The laserpoints to be added.
    */
            // void AddLaserPoints(const LaserPoints &lpts);
            // LaserPoints points; 
       private:
             Plane myplane;     
             
                    
};
#endif /* _LaserPatch_h_ */
