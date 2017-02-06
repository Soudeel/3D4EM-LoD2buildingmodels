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

      subroutine srvv_ls(xim,yim,npoi,cc,xmod,ymod,zmod,x0,quat,
     .                 auitw,yuitw)

c photogrammetric space resection

      integer*4   npoi
      real*8      xim(npoi),yim(npoi),
     .            xmod(npoi),ymod(npoi),zmod(npoi),
     .            cc, x0(3), quat(3),
     .            w(3),r(3,3),al(12),
     .            zz,
     .            di,aa,bb,ccc,xx,yy,
     .            dk,fk,xqz,yqz,
     .            auitw(2*npoi*6),yuitw(2*npoi) 
      integer*4   a_index, y_index

c setting of the approximate values for the quaternion elements

      do i=1,3
        w(i) = 2.0 * quat(i)
      enddo

c aufstellen und loesen der normalgleichungen

      di=.00000001
      
 5170 aa=w(1)*w(1)*.25
      bb=w(2)*w(2)*.25
      ccc=w(3)*w(3)*.25
      xx=w(1)*w(2)*.5
      yy=w(1)*w(3)*.5
      zz=w(2)*w(3)*.5
      dk=1./(1.+aa+bb+ccc)
      fk=-.5*cc*dk
      r(1,1)=(1.+aa-bb-ccc)*dk
      r(2,2)=(1.-aa+bb-ccc)*dk
      r(3,3)=(1.-aa-bb+ccc)*dk
      r(1,2)=(xx-w(3))*dk
      r(1,3)=(yy+w(2))*dk
      r(2,1)=(xx+w(3))*dk
      r(3,1)=(yy-w(2))*dk
      r(2,3)=(zz-w(1))*dk
      r(3,2)=(zz+w(1))*dk

      a_index = 1
      y_index = 1
      
      do 150 i=1,npoi
        call sr_obseq(xmod(i),ymod(i),zmod(i),xim(i),yim(i),
     .                cc,x0,r,fk,w,al,xqz,yqz)

c        write(6,2010) al(1), al(2), al(3) ,al(4) ,al(5), al(6)
c        write(6,2010) al(7), al(8) ,al(9), al(10), al(11) ,al(12)
c	
c 2010   format(f15.5,1x,f15.5,1x,f15.5,1x,f15.5,1x,f15.5,1x,f15.5)

c Opstellen A matrix auitw, en waarnemingsfile yuitw
        do 155 j=1,12
          auitw(a_index) = al(j)
	  a_index = a_index + 1
  155   continue
	  
        yuitw(y_index) = xqz
        y_index = y_index + 1
	
        yuitw(y_index) = yqz
        y_index = y_index + 1
 
  150 continue
  
 1305 return
      end
