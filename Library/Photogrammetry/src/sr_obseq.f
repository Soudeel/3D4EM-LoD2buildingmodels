c
c    Copyright 2010 University of Twente and Delft University of Technology
c 
c       This file is part of the Mapping libraries and tools, developed
c  for research, education and projects in photogrammetry and laser scanning.
c
c  The Mapping libraries and tools are free software: you can redistribute it
c    and/or modify it under the terms of the GNU General Public License as
c  published by the Free Software Foundation, either version 3 of the License,
c                   or (at your option) any later version.
c
c The Mapping libraries and tools are distributed in the hope that it will be
c    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
c        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
c                GNU General Public License for more details.
c
c      You should have received a copy of the GNU General Public License
c          along with the Mapping libraries and tools.  If not, see
c                      <http://www.gnu.org/licenses/>.
c
c---------------------------------------------------------------------------*/

      subroutine sr_obseq(xmod,ymod,zmod,xim,yim,cc,x0,r,fk,w,
     .                    al,xqz,yqz)
        
      implicit real*8 (a-h,o-z)
      real*8   x0(3),r(3,3),w(3),al(12)
      xx=xmod-x0(1)
      yy=ymod-x0(2)
      zz=zmod-x0(3)
      xq1=r(1,1)*xx+r(2,1)*yy+r(3,1)*zz
      xq2=r(1,2)*xx+r(2,2)*yy+r(3,2)*zz
      xq3=r(1,3)*xx+r(2,3)*yy+r(3,3)*zz
      xq3=1./xq3
      x2=xim
      y2=yim
      fkz=fk*xq3
      xqz=xq1*xq3
      yqz=xq2*xq3
      a1=xx*w(1)+yy*w(2)+zz*w(3)
      a2=xx*w(3)-yy*2.-zz*w(1)
      a3=xx*2.+yy*w(3)-zz*w(2)
      a4=xx*w(2)-yy*w(1)+zz*2.
      al(1)=fkz*(a1-xqz*a2)
      al(7)=fkz*(a4-yqz*a2)
      al(2)=fkz*(-a4-xqz*a3)
      al(8)=fkz*(a1-yqz*a3)
      al(3)=fkz*(-a2-xqz*a1)
      al(9)=fkz*(-a3-yqz*a1)
      al(4)=xq3*(r(1,3)*x2+r(1,1)*cc)
      al(10)=xq3*(r(1,3)*y2+r(1,2)*cc)
      al(5)=xq3*(r(2,3)*x2+r(2,1)*cc)
      al(11)=xq3*(r(2,3)*y2+r(2,2)*cc)
      al(6)=xq3*(r(3,3)*x2+r(3,1)*cc)
      al(12)=xq3*(r(3,3)*y2+r(3,2)*cc)
      xqz=x2+xqz*cc
      yqz=y2+yqz*cc
      return
      end
