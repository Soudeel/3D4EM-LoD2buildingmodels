
Caveat receptor.  (Jack) dongarra@cs.utk.edu, (Eric Grosse) research!ehg
Careful! Anything free comes with no guarantee.
*
*PLEASE NOTE THAT netlib HAS MOVED, THE NEW ADDRESS IS netlib@ornl.gov.
*THE OLD ADDRESS, netlib@mcs.anl.gov, WILL BE TURNED OFF SOON.
*
*** from netlib, Tue Sep 11 13:40:42 EDT 1990 ***
      subroutine zrotg(ca,cb,c,s)
      double complex ca,cb,s
      double precision c,dcabs1
      double precision norm,scale
      double complex alpha
      if (dcabs1(ca) .ne. 0.0d0) go to 10
         c = 0.0d0
         s = (1.0d0,0.0d0)
         ca = cb
         go to 20
   10 continue
         scale = dcabs1(ca) + dcabs1(cb)
         norm = scale*dsqrt((dcabs1(ca/dcmplx(scale,0.0d0)))**2 +
     *                      (dcabs1(cb/dcmplx(scale,0.0d0)))**2)
         alpha = ca /dcabs1(ca)
         c = dcabs1(ca) / norm
         s = alpha * dconjg(cb) / norm
         ca = alpha * norm
   20 continue
      return
      end
