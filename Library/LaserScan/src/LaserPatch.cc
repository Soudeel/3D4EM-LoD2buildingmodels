
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



#include "LaserPatch.h"
#include "Plane.h"
#include "Vector3D.h"
#include "Positions3D.h"

 
Plane LaserPatch::getPlane() const
{
      return myplane;
}


void LaserPatch::setPlane(Plane &p)
{
     myplane=p;
 }
 
bool LaserPatch::Intersect(LaserPatch pie, double error)
{
    
    if(Distance(pie)<error)
     return true;
    else
    return false;
     
} 

bool LaserPatch::OnGround(double ground,double error)
{
    
    for(int i=0;i<size();i++)
     {
            if((*this)[i].GetZ()<ground+error)
               return true;
     }
    return false; 
}

 
double LaserPatch::Distance(LaserPatch pie)
{
       
       double min=32768999;
       double dis;
            
       for(int i=0;i<size();i++)
          {
                  dis=fabs(pie.getPlane().Distance((*this)[i]));
                  min=min<dis?min:dis;
          }   
       return min;    
}
 
 
 double LaserPatch::area3D() const
 {
 float area = 0;
    float an, ax, ay, az;  // abs value of normal and its coords
    int   coord;           // coord to ignore: 1=x, 2=y, 3=z
    int   i, j, k;         // loop indices
    Vector3D N;
    LaserPatch tmp(*this);
    
    tmp.push_back(tmp[1]);  //make sure Point[n-1]=Point[1] and Point[n-2]=Point[0]
    
    N=myplane.Normal();
    // select largest abs coordinate to ignore for projection
    ax = (N.X()>0 ? N.X() : -N.X());     // abs x-coord
    ay = (N.Y()>0 ? N.Y() : -N.Y());     // abs y-coord
    az = (N.Z()>0 ? N.Z() : -N.Z());     // abs z-coord

    coord = 3;                     // ignore z-coord
    if (ax > ay) {
        if (ax > az) coord = 1;    // ignore x-coord
    }
    else if (ay > az) coord = 2;   // ignore y-coord

    // compute area of the 2D projection
    for (i=1, j=2, k=0; i<=tmp.size()-2; i++, j++, k++)
        switch (coord) {
        case 1:
            area += (tmp[i].GetY() * (tmp[j].GetZ() - tmp[k].GetZ()));
            continue;
        case 2:
            area += (tmp[i].GetX() * (tmp[j].GetZ() - tmp[k].GetZ()));
            continue;
        case 3:
            area += (tmp[i].GetX() * (tmp[j].GetY() - tmp[k].GetY()));
            continue;
        }
        
        
        

    // scale to get area before projection
    an = sqrt( ax*ax + ay*ay + az*az);  // length of normal vector
    switch (coord) {
    case 1:
        area *= (an / (2*ax));
        break;
    case 2:
        area *= (an / (2*ay));
        break;
    case 3:
        area *= (an / (2*az));
    }
    return fabs(area);
 
 }
 /*
void LaserPatch::AddLaserPoints(const LaserPoints &lpts)
{
     points=lpts;
}
*/



