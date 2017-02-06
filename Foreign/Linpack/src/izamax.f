
Caveat receptor.  (Jack) dongarra@cs.utk.edu, (Eric Grosse) research!ehg
Careful! Anything free comes with no guarantee.
*
*PLEASE NOTE THAT netlib HAS MOVED, THE NEW ADDRESS IS netlib@ornl.gov.
*THE OLD ADDRESS, netlib@mcs.anl.gov, WILL BE TURNED OFF SOON.
*
*** from netlib, Mon Oct 29 04:27:06 EST 1990 ***
      integer function izamax(n,zx,incx)
c
c     finds the index of element having max. absolute value.
c     jack dongarra, 1/15/85.
c
      double complex zx(1)
      double precision smax
      integer i,incx,ix,n
      double precision dcabs1
c
      izamax = 0
      if(n.lt.1)return
      izamax = 1
      if(n.eq.1)return
      if(incx.eq.1)go to 20
c
c        code for increment not equal to 1
c
      ix = 1
      smax = dcabs1(zx(1))
      ix = ix + incx
      do 10 i = 2,n
         if(dcabs1(zx(ix)).le.smax) go to 5
         izamax = i
         smax = dcabs1(zx(ix))
    5    ix = ix + incx
   10 continue
      return
c
c        code for increment equal to 1
c
   20 smax = dcabs1(zx(1))
      do 30 i = 2,n
         if(dcabs1(zx(i)).le.smax) go to 30
         izamax = i
         smax = dcabs1(zx(i))
   30 continue
      return
      end
