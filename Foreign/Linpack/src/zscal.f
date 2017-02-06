
Caveat receptor.  (Jack) dongarra@cs.utk.edu, (Eric Grosse) research!ehg
Careful! Anything free comes with no guarantee.
*
*PLEASE NOTE THAT netlib HAS MOVED, THE NEW ADDRESS IS netlib@ornl.gov.
*THE OLD ADDRESS, netlib@mcs.anl.gov, WILL BE TURNED OFF SOON.
*
*** from netlib, Tue Sep 11 13:39:53 EDT 1990 ***
      subroutine  zscal(n,za,zx,incx)
c
c    scales a vector by a constant.
c    jack dongarra, 3/11/78.
c
      double complex za,zx(1)
      if(n.le.0)return
      if(incx.eq.1)go to 20
c
c        code for increments not equal to 1
c
      ix = 1
      if(incx.lt.0)ix = (-n+1)*incx + 1
      do 10 i = 1,n
        zx(ix) = za*zx(ix)
        ix = ix + incx
   10 continue
      return
c
c        code for increments equal to 1
c
   20 do 30 i = 1,n
        zx(i) = za*zx(i)
   30 continue
      return
      end
