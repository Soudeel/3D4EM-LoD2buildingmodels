c
c                Copyright 2010 Delft University of Technology
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

      integer*4 function hpadjust(m,n,n1,a,y,qy,qyd,iqycor,depth,
     +    var,varest,detqy,detn,x,qx,qx2,
     +    e,qe,qed,w,naby,lamy,lamx,lamx1, tpar)
c     **************
c
c*ver ADJUST version 2.0  d.d. 07-11-1989  modified: 1-02-1993 
c*sys ms-dos 5.0, MS-FORTRAN 5.1
c*aut frank kenselaar, delft geodetic computing centre
c*aut ADJUST modified to subroutine PCSCAN by G.J.Husti
c
c*aut Additional changes to the code made by R.Th. Ursem to include
c*aut this routine in the software written for the photogrammetry
c*aut group.
c
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol program for the linear least-squares adjustment of observation
c*rol equations for non-correlated or correlated observations
c*rol interactive input or file input of observations, variance-matrix
c*rol and designmatrix.
c*rol file output of adjusted observations and unknowns, residuals,
c*rol datasnooping and int.- and ext. reliability.
c
c*fil 5/6: screen, 7: output-file: PCSCAN.DOC  [8: inputfile in old version]
c
c referenced routines    : fmint1 (interactive matrix input)  [deleted]
c                          fmprtr/fmprts (print matrix)
c                          fmlsa3 (ls adjustment uncorr. obs.)
c                          fmlsa4 (ls adjustment correl. obs.)
c                          fmcovd (matrix inversion and determinant)
c                          terel3 (testing and reliab. uncorr. obs.)
c                          terel4 (testing and reliab. correl. obs.)
c                          bmeth1 (computation statistical parameters)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par m       in        number of observations
c*par n       in        number of unknowns
c*par n1      in        number first part unknowns (for ext. reliab.)
c*par var     in        variance-factor (a priori)
c*par a       in        designmatrix
c*par y       in        observations
c*par qy      in        variance matrix observations (diag. or undertr.)
c*par iqycor  in        qy: 0 = no correlation; 1 = unit; 2 = correlation
c*par depth   in        0 = do everything, 1 = do only adjustment
c*par varest  out       var, estimated = testg (sigma^ after adjustment)
c*par detqy   out       determinant qy
c*par detn    out       determinant normal matrix
c*par x       out       adjusted unknowns
c*par qx      int       variance matrix adj. unknowns (undertriangle)
c*par qx2     int       variance matrix last n-n1 unknowns
c*par e       out       least squares residuals
c*par qe      int       variance matrix residuals (undertr.or diag:qed)
c*par w       out       vector w-tests datasnooping
c*par naby    out       vector nablas int. reliab.
c*par lamy    out       vector (sqrt) lambda-y
c*par lamx    out       vector (sqrt) lambda-x
c*par lamx1   out       vector (sqrt) lambda-x first n1 unknowns
c*par tpar    out       statistical parameters (gamma, lambda, alpha etc)
c
c?  remarks: (1) lines starts with "c?" should be revized
cx           (2) lines starts with "cx" can be deleted
      double precision  epsdet
      parameter ( epsdet=1.d+20)
      integer           m, n, n1, iqycor
      integer           i, j, in, ii, depth
      double precision  a(m*n), y(m), qy(m*(m+1)/2), qyd(m)
      double precision  x(n), qx(n*(n+1)/2),
     #                  qx2(n*(n+1)/2), e(m), 
     #                  qe(m*(m+1)/2), qed(m), w(m),
     #                  naby(m), lamy(m), lamx(m), lamx1(m),var, 
     #                  tpar(7), testg, crit1, varest,
     #                  detqy, detn, lndetq, lnepsd
      character         cfail*72,choice*1
      logical           check, qual, zero, corr, unit, error
c
        cfail = ' '
        corr = .false.
        unit = .false.
        if(iqycor.eq.1) unit = .true.
        if(iqycor.gt.1) corr = .true.
c
c      ----- CREATE UNIT (- OR DIAGONAL) VARIANCE-MATRIX
c
      if (unit) then
         do 25 i = 1,m
            in = i*(i-1)/2
            qyd(i) = 1.d0
            qy(in+i) = 1.d0
            do 26 ii = 1,i-1
               qy(in+ii) = 0.d0
   26       continue
   25    continue
         go to 4
      end if
c
c --------------------------------------------------------------------
c ---------- CHOICE OF COMPUTATION -----------------------------------
c --------------------------------------------------------------------
c
    4 choice = 'a'
      check  = .true.
      qual   = .true.
c
c --------------------------------------------------------------------
c ---------- ADJUSTMENT ----------------------------------------------
c --------------------------------------------------------------------
c
      error = .false.
c
c      ----- CHECK NUMBER OF OBSERVATIONS AND UNKNOWNS
c  
c      ----- too many unknowns for fmcovd, terel3 and terel4
c
      if (n.gt.100) then
         write (6,1400) ' err adjust: TOO MANY UNKNOWNS (n > 100)'
      end if
c  
c      ----- too many observations for fmcovd, terel3 and terel4
c
      if (m.gt.100) then
         cfail = ' err adjust: TOO MANY OBSERVATIONS (m > 100)'
         write (6,1400) cfail
	 cfail = ' ***** USING DIAGONAL VARIANCE MATRIX ******'
	 write (6,1400) cfail
	 cfail = ' '
	 write (6,1400) cfail
c  
c      ----- create diagonal matrix from full cov. matrix
c
	 if (unit.neqv..false.) then
	   do 30 i = 1,m
	     qyd(i) = qy(i*(i-1)/2+i)
   30	   continue
         unit = .false.
         end if
	 corr = .false.
      end if
c  
c      ----- more unknowns than observations
c
      if (n.gt.m) then
         cfail = ' err adjust: MORE UNKNOWNS THAN OBSERVATIONS (n > m)'
         write (6,1400) cfail
         error = .true.
      end if
      if (error) go to 99
c
c      ----- CHECK DESIGNMATRIX FOR ZERO ROWS OR COLUMNS
c
      do 40 i = 1,m
         zero = .true.
         in   = n*(i-1)
         do 41 j = 1,n
            if (a(in+j).ne.0.d0) then
               zero = .false.
               go to 40
            end if
   41    continue
         if (zero) then
            write (cfail,'('' err adjust: ZERO ROW (NO.'',i3,
     #                     '') IN DESIGN MATRIX'')') i
            write (6,1400) cfail
            error = .true.
         end if
   40 continue
c
      do 42 j = 1,n
         zero = .true.
         do 43 i = 1,m
            if (a(n*(i-1)+j).ne.0.d0) then
               zero = .false.
               go to 42
            end if
   43    continue
         if (zero) then
            write (cfail,'('' err adjust: ZERO COLUMN (NO.'',i3,
     #                     '') IN DESIGN MATRIX'')') j
            write (6,1400) cfail
            error = .true.
         end if
   42 continue
c
      if (error) go to 99
c
c      ----- CHECK qy, COMPUTE AND CHECK DETERMINANT qy
c
      if (unit) then
         detqy = 1.d0
         go to 5
      end if
c
      do 45 i = 1,m
         if (qyd(i).eq.0.d0) then
            write (cfail,'('' err adjust: ZERO DIAGONAL ELEMENT'',
     #                     '' (NO.'',i3,'') IN VAR. MAT. qy'')') i
            write (6,1400) cfail
            error = .true.
         end if
         if (qyd(i).lt.0.d0) then
            write (cfail,'('' err adjust: NEGATIVE DIAGONAL ELEMENT'',
     #                     '' (NO.'',i3,'') IN VAR. MAT. qy'')') i
            write (6,1400) cfail
            error = .true.
         end if
   45 continue
      if (error) go to 99
c
      if (corr) go to 5
c
      lndetq = 0.d0
      do 46 i = 1,m
         lndetq = lndetq + dlog(qyd(i))
   46 continue
      lnepsd = dlog(epsdet)
      detqy = dexp(lndetq)
cr      if (lndetq.lt.-lnepsd) then
cr         write (cfail,'('' err adjust: DETERMINANT qy TOO SMALL'',
cr     #                  '' ('',d8.1,'')'')') 1.d0/epsdet
cr         write (6,1400) cfail
cr         error = .true.
cr      end if
cr      if (lndetq.gt.lnepsd) then
cr         write (cfail,'('' err adjust: DETERMINANT qy TOO LARGE'',
cr     #                  '' ('',d8.1,'' > '',d8.1,'')'')') detqy, epsdet
cr         write (6,1400) cfail
cr         error = .true.
cr      end if
      if (error) go to 99
c
c      ============================
c         ADJUSTMENT
c      =============================
    5 if (.not.corr) then
         call fmlsa3 (m,n,n1,y,qyd,a,x,qx,qx2,e,qed,detn,cfail)
      else
         call fmlsa4 (m,n,n1,y,qy,a,x,qx,qx2,e,qe,detqy,detn,cfail)
      end if
      if (cfail.ne.' ') then
         write (6,1400) cfail
         go to 99
      end if
c
c --------------------------------------------------------------------
c ---------- TESTING AND QUALITY-ANALYSIS ----------------------------
c --------------------------------------------------------------------
c
      if (depth.eq.0) go to 99
      if (.not.qual) go to 99
c
      if (.not.corr) then
         call terel3 (m,n,n1,qyd,e,qed,a,qx2,var,
     #                                tpar,w,naby,lamy,lamx,lamx1)
      else
         call terel4 (m,n,n1,qy,e,qe,a,qx2,var,
     #                                tpar,w,naby,lamy,lamx,lamx1)
      end if
c  note: tpar(7) = ee/(b*var)     ; used for global test
c  note: varest = ee/b            ; 'variance estimated' 
      varest = tpar(7)*var
      testg  = tpar(7)
c
      crit1 = tpar(4)
c
c --------------------------------------------------------------------
c ---------- FORMATS -------------------------------------------------
c --------------------------------------------------------------------
c
 1000 format (' ********************************* HPADJUST ****',
     #           '*****************************',
     #         /,' ****           Least squares adjustment of ',
     #           'linear observation           ****',
     #         /,' ****           equations for (non-) correlated ',
     #           'observations.            ****',
     #         /,' ************************************************',
     #           '****************************')
 1010 format (//,' ******************************** input data ***',
     #           '*****************************')
 1020 format (//,' **************************** adjustment results',
     #           ' ****************************')
 1030 format (//,' ***************************** quality analysis ',
     #           '*****************************')
 1050 format ( /,' ****************************** end of program *',
     #           '*****************************')
 1400 format (a72)
c
 3530 format (//,' number of observations (m)     : ',i3,
     #         /,' number of unknowns (n)         : ',i3,
     #         /,' part unknowns (n1) ext. reliab.: ',i3,
     #         /,' variance factor (sigma**2)     :  ',d12.5)
 3540 format (//,' observations and variances (excl. variance factor)',
     #        //,'  nr.       y[i]             qy[i]',
     #         /,' ------------------------------------')
 3550 format (i4,3x,d15.8,3x,d12.5)
 3560 format (//,' observations',
     #        //,'  nr.       y[i]',
     #         /,' ------------------')
 3570 format (i4,3x,d12.5)
 3580 format (//,' design matrix a(m,n)')
 3590 format (//,' variance matrix observations qy(m,m)',
     #           ' (excl. variance factor)')
c
 4000 format (//,' determinant variance matrix (qy)   : ',d12.5,
     #         /,' determinant normal matrix (qx-INV) : ',d12.5)
 4010 format (//,' adjusted unknowns and variances',
     #           ' (excl. variance factor)',
     #        //,'  nr.       x[j]            var-x[j]',
     #         /,' ------------------------------------')
 4020 format (i4,3x,d15.8,3x,d12.5)
 4030 format (//,' adjusted observations and variances',
     #           ' (excl. variance factor)',
     #        //,'  nr.     adj-y[i]       var-adj-y[i]',
     #         /,' ------------------------------------')
c
 5000 format (//,' test parameters',
     #         /,'      number of observations (m) :',i3,
     #         /,'      number of unknowns (n)     :',i3,
     #         /,'      degrees of freedom (m-n)   :',i3,
     #         /,'      variance factor (sigma**2) : ',d12.5,
     #         /,'      gamma-0                    :',f8.4,
     #         /,'      lambda-0                   :',f8.4,
     #         /,' one-dimensional w-test',
     #         /,'      level of significance      :',f8.4,
     #         /,'      critical value             :',f8.4,
     #         /,' global test',
     #         /,'      level of significance      :',f8.4,
     #         /,'      critical value             :',f8.4)
 5010 format (//,' global test :',f11.4,'  accepted')
 5020 format (//,' global test :',f11.4,'  rejected')
 5030 format (//,' testing and reliability based on conventional',
     #           ' alternative hypothesis',
     #         /,'   residuals, w-test (datasnooping)',
     #         /,'   internal reliability,  Minimal Detectable Bias',
     #               6x,': nabla-y',
     #         /,25x,' norm (sqrt)                  : lam-y',
     #         /,'   external reliability,  norm all unknowns',
     #               6x,'(sqrt): lam-x',
     #         /,25x,' norm first',i3,' unknowns (sqrt): lam-x1')
 5040 format ( /,'  nr.     residual    w-test  reject? ',
     #           '  nabla-y     lam-y      lam-x   lam-x1',
     #         /,' ---------------------------------------',
     #           '-------------------------------------')
 5050 format (i4,3x,d12.5,2x,'    -               ',
     #           '(nearly) free observation')
 5060 format (i4,3x,d12.5,1x,f8.3,'       ',
     #        2x,d12.5,1x,f7.2,4x,f7.2,1x,f7.2)
 5070 format (i4,3x,d12.5,1x,f8.3,'   yes ',
     #        2x,d12.5,1x,f7.2,4x,f7.2,1x,f7.2)
c
c --------------------------------------------------------------------
c ---------- END OF PROGRAM ------------------------------------------
c --------------------------------------------------------------------
c
   99 continue
c     write (7,1400) cfail
c     write (7,1050)
c     close (7)
c
c     HPADJUST = (cfail.ne.' ')
      HPADJUST = 1
      if (cfail.eq.' ') HPADJUST = 0
      return
      end
c *********************************************************************
      subroutine terel3 (m,n,n1,qyd,e,qed,a,qx2,var,
     #                   tpar,w,naby,lamy,lamx,lamx1)
c     *****************
c
c*ver version 2.0  d.d. 28-01-90           modified:
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol computation of test- and reliability-quantities according
c*rol to the b-method of testing for uncorrelated observations.
c*rol the test-parameters, global test, datasnooping and internal
c*rol and external reliabitity are computed.
c*rol this routine is an extension of terel1. also the external
c*rol reliability on the first n1 unknowns is computed.
c
c     referenced routines: bmeth1 (test parameters)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par m       in        number of observations
c*par n       in        number of unknowns
c*par n1      in        (first) part unknowns (0<=n1<=n)
c*par qyd     in        diagonal variance matrix observations (m)
c*par e       in        least squares residuals (m)
c*par qed     in        diagonal variance matrix residuals (m)
c*par a       in        design matrix
c*par qx2     in        undertr. variance matrix last n-n1 unknowns
c*par var     in        variance-factor
c*par tpar    out       test-parameters: gamma0, lambda0,
c                                        alpha + crit.val. 1 dim test,
c                                        alpha + crit.val. + test-
c                                        quantity global test
c*par w       out       vector w-quantities (m)
c*par naby    out       vector nablas (m)
c*par lamy    out       vector lambda-y (sqrt) (m)
c*par lamx    out       vector lambda-x (sqrt) (m)
c*par lamx1   out       vector lambda-x (sqrt) first n1 unknowns (m)
c
      double precision eps, alpha1, gamma0
      parameter        (eps=1.d-5, alpha1=0.001, gamma0=0.80, max=100)
      integer          m, n, n1,   b, i, n2, j, jn, jj, in
      double precision qyd(m), e(m), qed(m), var, a(m*n),
     #                 qx2((n-n1)*(n-n1+1)/2), tpar(7), w(m), naby(m),
     #                 lamy(m), lamx(m), lamx1(m), a2qyi(max)
c  a2qyi(m)  FOUTMELDING, want deze variable komt niet voor in de SUBR.list  
      double precision lambda0, crit1, alphag, critg,
     #                 ee, ei, qyi, qei, nom,
     #                 nab, lam, a2i, sum
      logical          part
c
c
      part = .false.
      if (n1.ge.1.and.n1.le.n) part = .true.
c
c ---------- TEST PARAMETERS ------------------------------------------
c
      b      = m-n
      call bmeth1 (alpha1,gamma0,b,crit1,lambda0,alphag,critg)
c
      tpar(1) = gamma0
      tpar(2) = lambda0
      tpar(3) = alpha1
      tpar(4) = dsqrt(crit1)
      tpar(5) = alphag
      tpar(6) = critg
c
c ---------- GLOBAL TEST, DATASNOOPING, RELIABILITY -------------------
c
      ee = 0.d0
      do 10 i = 1,m
         ei  = e(i)
         qyi = qyd(i)
         qei = qed(i)
c
c      ----- NECESSARY FOR EXT. RELIAB. PART n1 UNKNOWNS
c
         a2i = 0.d0
         if (part) then
            n2 = n - n1
            in = n*(i-1)
            do 20 j = 1,n2
               a2qyi(j) = a(in+n1+j)/qyi
   20       continue
            a2i = 0.d0
            do 30 j = 1,n2
               sum = 0.d0
               jn = j*(j-1)/2
               do 40 jj = 1,j
                  sum = sum + qx2(jn+jj)*a2qyi(jj)
   40          continue
               do 45 jj = j+1,n2
                  sum = sum + qx2(jj*(jj-1)/2+j)*a2qyi(jj)
   45          continue
               a2i = a2i + a2qyi(j)*sum
   30       continue
         end if
c
c     ----- w, nABLA AND lAMBDA'S FOR EACH OBSERVATION
c
         ee = ee + ei**2/qyi
c
         w(i)     = 0.d0
         naby(i)  = 0.d0
         lamy(i)  = 0.d0
         lamx(i)  = 0.d0
         lamx1(i) = 0.d0
c
         nom = qei/qyi**2
         if (nom.gt.eps) then
            qyi = qyi*var
            qei = qei*var
            nom = nom/var
            a2i = a2i/var
            nab = lambda0/nom
            lam = nab/qyi
c
            w(i)    = dabs(ei)/dsqrt(qei)
            naby(i) = dsqrt(nab)
            lamy(i) = dsqrt(lam)
            lamx(i) = dsqrt(lam - lambda0)
            if (part) lamx1(i) = dsqrt(lam - lambda0 - nab*a2i)
         end if
   10 continue
      tpar(7) = ee/(b*var)
c
      return
      end
c *********************************************************************
      subroutine terel4 (m,n,n1,qy,e,qe,a,qx2,var,
     #                                 tpar,w,naby,lamy,lamx,lamx1)
c     *****************
c
c*ver version 2.0  d.d. 28-01-1990         modified:
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol computation of test- and reliability-quantities according
c*rol to the b-method of testing for correlated observations.
c*rol the test-parameters, global test, datasnooping and internal
c*rol and external reliabitity are computed.
c*rol this routine is an extension of terel2. also the external
c*rol reliability on the first n1 unknowns is computed.
c
c     referenced routines: bmeth1 (test parameters)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par m       in        number of observations
c*par n       in        number of unknowns
c*par n1      in        (first) part unknowns (0<=n1<=n)
c*par qy      in        weight matrix observations (m(m+1)/2)
c*par e       in        least squares residuals (m)
c*par qe      in        variance matrix residuals (m(m+1)/2)
c*par a       in        design matrix (m*n)
c*par qx2     in        variance matrix first n1 unknowns
c*par var     in        variance-factor
c*par tpar    out       test-parameters: gamma0, lambda0,
c                                        alpha + crit.val. 1 dim test,
c                                        alpha + crit.val. + test-
c                                        quantity global test
c*par w       out        vector w-quantities (m)
c*par naby    out        vector nablas (m)
c*par lamy    out        vector lambda-y (sqrt) (m)
c*par lamx    out        vector lambda-x (sqrt) (m)
c*par lamx1   out        vector lambda-x (sqrt) first n1 unknowns (m)
c
c
c      implicit double precision (a-h,o-z)
      double precision eps, alpha1, gamma0
      parameter        (eps=1.d-5, alpha1=0.001, gamma0=0.80, max=100)
      integer          b
      integer        m, n, i, im, ii, ni, k, in, j, jj, jn, n2
      double precision qy(m*(m+1)/2), e(m), qe(m*(m+1)/2), var,
     #                 a(m*n), qx2((n-n1)*(n-n1+1)/2), tpar(7), w(m),
     #                 naby(m), lamy(m), lamx(m), lamx1(m)
      double precision lambda0, crit1, alphag, critg,
     #                 ee, nom, qyei, nab, lam,
     #                 qyi(max), qeqyi(max), qeqyii, a2qyi(max),
     #                 sum, a2i, qyii
      logical          part
c
c
      part = .false.
      if (n1.ge.1.and.n1.le.n) part = .true.
c
c ---------- TEST PARAMETERS ------------------------------------------
c
      b      = m-n
      call bmeth1 (alpha1,gamma0,b,crit1,lambda0,alphag,critg)
c
      tpar(1) = gamma0
      tpar(2) = lambda0
      tpar(3) = alpha1
      tpar(4) = dsqrt(crit1)
      tpar(5) = alphag
      tpar(6) = critg
c
c ---------- GLOBAL TEST, DATASNOOPING, RELIABILITY -------------------
c
      ee = 0.d0
      do 70 im = 1,m
c
c      ----- qyi = COLUMN I FROM qyINV
c
      ni = im*(im-1)/2
         do 10 i = 1,im
            k = ni+i
            qyi(i) = qy(k)
   10    continue
         do 11 i = im+1,m
            k = i*(i-1)/2 + im
            qyi(i) = qy(k)
   11    continue
c
c      ----- qeqyi = qe*qyi
c
         do 20 i = 1,m
            qeqyii   = 0.d0
            qeqyi(i) = 0.d0
            in = i*(i-1)/2
            do 21 ii = 1,i
               qeqyii = qeqyii + qe(in+ii)*qyi(ii)
   21       continue
            do 22 ii = i+1,m
               qeqyii = qeqyii + qe(ii*(ii-1)/2+i)*qyi(ii)
   22       continue
            qeqyi(i) = qeqyii
   20    continue
c
c      ----- nom  = qyi*qeqyi
c      ----- qyei = qyi*e
c
c
         nom  = 0.d0
         qyei = 0.d0
         do 30 i = 1,m
            nom  = nom  + qyi(i)*qeqyi(i)
            qyei = qyei + qyi(i)*e(i)
	    if (im.gt.30.and.im.lt.34) then
cr       write (*,5000) im, i, nom, qyi(i), qeqyi(i), qyi(i)*qeqyi(i)
cr 5000  format ('nom (',i3,',',i3,') = ',d12.5,
cr     + ' (',d12.5,'*',d12.5,'=',d12.5,')')
       end if
   30    continue
c
c      ----- a2qyi = a(2)*qyi
c      ----- a2i = a2qyi*qx2*a2qyi
c
         a2i = 0.d0
         if (part) then
            n2 = n - n1
            do 45 j = 1,n2
               sum = 0.d0
               do 40 i = 1,m
                  in = n*(i-1)
                  sum = sum + a(in+n1+j)*qyi(i)
   40          continue
               a2qyi(j) = sum
   45       continue
            a2i = 0.d0
            do 60 j = 1,n2
               sum = 0.d0
               jn = j*(j-1)/2
               do 50 jj = 1,j
                  sum = sum + qx2(jn+jj)*a2qyi(jj)
   50          continue
               do 55 jj = j+1,n2
                  sum = sum + qx2(jj*(jj-1)/2+j)*a2qyi(jj)
   55          continue
               a2i = a2i + a2qyi(j)*sum
   60       continue
         end if
c
c      ----- w, NABLA AND LAMBDA'S FOR EACH OBSERVATION
c
         ee = ee + e(im)*qyei
c
         w(im)     = 0.d0
         naby(im)  = 0.d0
         lamy(im)  = 0.d0
         lamx(im)  = 0.d0
         lamx1(im) = 0.d0
c
         if (nom.gt.eps) then
            nom  = nom/var
            qyei = qyei/var
            qyii = qyi(im)/var
            a2i  = a2i/var
            nab  = lambda0/nom
            lam  = nab*qyii
c
            w(im)     = dabs(qyei)/dsqrt(nom)
            naby(im)  = dsqrt(nab)
            lamy(im)  = dsqrt(lam)
            lamx(im)  = dsqrt(lam - lambda0)
            if (part) lamx1(im) = dsqrt(dabs(lam - lambda0 - nab*a2i))
c
         end if
   70 continue
c
c      ----- GLOBAL TEST
c
      tpar(7) = ee/(b*var)
c
      return
      end
c *********************************************************************
      subroutine fmprtr(ilu,a,irowsa,icolsa,iprint,ipage,iline,cfail)
c     *****************
c
c*ver version 1.1  d.d. 01-01-87          modified: 22-6-87
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut martin salzmann, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol spools a full matrix a to unit ilu
c
c*fil ilu
c
c     referenced routines:
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par ilu     input     logical unit number
c*par a       input     matrix to be printed (irowsa*icolsa)
c*par irowsa  input     number of rows a
c*par icolsa  input     number of columns a
c*par iprint  input     chosen format: 1  f11.6
c*par                                  2  d11.5
c*par ipage   input     number of lines per page
c*par iline   input     number of elements per line (max: 10)
c*par cfail   output    error message
 
      double precision a(*)
      integer          ilu
      integer          irowsa
      integer          icolsa
      integer          iprint
      integer          iline
      integer          ipage 
      integer          m1,m2,ntot,itel
      character*(*)    cfail

      cfail = '  '
      if((iprint.lt.1).or.(iprint.gt.2)) then
        cfail  = 'war fmprtr: format set to d11.5'
        iprint = 2
      end if
      if(iline.gt.10) then
        iline = 10
        cfail = 'war fmprtr: number of elements per line set to 10'
      end if
 
      m1      = 1
      m2      = iline
      ntot    = icolsa/iline
 
      do 101 i=1,ntot   
         write(ilu,2100) (j,j=m1,m2)
         do 102 j=1,irowsa
            if (j.eq.ipage+1) write(ilu,2100)(k,k=m1,m2)
            if(iprint.eq.1) then
               write(ilu,2201) j,(a(icolsa*(j-1)+k),k=m1,m2)
            else
               write(ilu,2202) j,(a(icolsa*(j-1)+k),k=m1,m2)
            endif
102      continue
         m1 = m1+iline
         m2 = m2+iline
101    continue
 
       if((icolsa-(ntot*iline)).ge.1) then
         itel = icolsa - m1 + 1
         write(ilu,2101) (j,j=m1,icolsa)
         do 103 j=1,irowsa
            if (j.eq.ipage+1) write(ilu,2101)(k,k=m1,icolsa)
            if(iprint.eq.1) then
               write(ilu,2203) j,(a(icolsa*(j-1)+k),k=m1,icolsa)
            else
               write(ilu,2204) j,(a(icolsa*(j-1)+k),k=m1,icolsa)
            endif
103      continue
      end if

      return
  
2100  format('1',6x,10(6x,i3,3x),/)
2101  format('1',6x,10(6x,i3,3x),/)
2201  format(' ',i3,3x,10(1x,f11.6))
2202  format(' ',i3,3x,10(1x,d11.5))
2203  format(' ',i3,3x,10(1x,f11.6))
2204  format(' ',i3,3x,10(1x,d11.5))
 
      end
c *********************************************************************
      subroutine fmprts(ilu,a,n,iprint,ipage,iline,cfail)
c     *****************
c
c*ver version 1.1  d.d. 01-01-87          modified: 22-6-87
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut martin salzmann, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol a symmetric row wise stored upper triangle matrix
c*rol (n(n+1)/2 vector) is printed as a lower triangular matrix.
c
c*fil ilu
c
c     referenced routines:
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par ilu     input     logical unit number
c*par a       input     printed matrix (dim: n)
c*par n       input     number of rows/columns
c*par iprint  input     chosen format: 1  f11.6
c*par                                  2  d11.5
c*par ipage   input     number of lines per page
c*par iline   input     number of elements per line (max: 10)
c*par cfail   output    error message
c 
c
      double precision a(*)
      integer          ilu
      integer          n
      integer          iprint
      integer          ipage
      integer          iline
      character*(*)    cfail

      cfail = ' '
      if((iprint.lt.1).or.(iprint.gt.2)) then
         cfail  = 'war fmprts: format set to d11.5'
         iprint = 2
      end if
      if((iline.lt.0).or.(iline.gt.10)) then
         cfail  = 'war fmprts: no. elements per line set to 10'
         iline  = 10
      end if

      n1      = 1
      n2      = iline
      m2      = n
      ntot    = n/iline
 
      do 101 i=1,ntot
         write(ilu,2100) (j,j=n1,n2)
         do 102 j=n1,n2
            if(iprint.eq.1) then
               write(ilu,2200) j,(a(j*(j-1)/2+k),k=n1,j)
            else
               write(ilu,2300) j,(a(j*(j-1)/2+k),k=n1,j)
            end if
102      continue
         do 103 j=i*iline+1,m2
          ilen = j-(i-1)*iline
            if((dfloat(ilen/ipage)).eq.
     1      (dfloat(ilen)/ipage))
     1                        write(ilu,2100) (k,k=n1,n2)
            if(iprint.eq.1) then
               write(ilu,2200) j,(a(j*(j-1)/2+k),k=n1,n2)
            else
               write(ilu,2300) j,(a(j*(j-1)/2+k),k=n1,n2)
            end if
103       continue
         n1 = n1+iline
         n2 = n2+iline
101   continue
      write(ilu,2100) (j,j=n1,n)
      do 104 j=n1,n
         if(iprint.eq.1) then
            write(ilu,2200) j,(a(j*(j-1)/2+k),k=n1,j)
         else
            write(ilu,2300) j,(a(j*(j-1)/2+k),k=n1,j)
         end if
104   continue
       
      return
 
2100  format('1',6x,12(6x,i3,3x),//)
2200  format(' ',i3,3x,12(1x,f11.6))
2300  format(' ',i3,3x,12(1x,d11.5))
 
      end
c *********************************************************************
      subroutine fmlsa3 (m,n,n1,y,qyd,a,x,qx,qx2,e,qed,det,cfail)
c     *****************
c
c*ver version 1.0  d.d. 30-10-1989         modified:
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol least squares adjustment of a linear system of equations with
c*rol m uncorrelated observations and n unknowns.
c*rol this routine is an extention of fmlsa1. also the variance matrix
c*rol (qx2) of the last n2 unknowns is given for computation of the
c*rol external reliability on the first n1 unknowns (e.g. by terel3).
c*rol in : observations (y), diagonal variance matrix (qy),
c*rol      design matrix (a)
c*rol out: ls-est. unknowns (x), undertr. variance matrix (qx),
c*rol      ls-residuals (e), diagonal variance matrix (qe),
c*rol      undertr. variance matrix (qx2) last n-n1 unknowns
c*rol row-wise stored full matrices. the normal matrix determinant
c*rol and eventually error messages are given.
c
c     referenced routines: fmcovd (inversion)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par m       in        number of observations
c*par n       in        number of unknowns
c*par n1      in        (first) part unknowns (for part. ext. reliab.)
c*par y       in        observations (m)
c*par qyd     in        diagonal elements variance matrix observ. (m)
c*par a       in        designmatrix (m*n)
c*par x       out       least squares estimates unknowns (n)
c*par qx      out       variance matrix unknowns (n(n+1)/2)
c*par qx2     out       variance matrix last n-n1 unknowns
c*par e       out       least squares residuals (m)
c*par qed     out       diagonal elements variance matrix resid. (m)
c*par det     out       determinant normal matrix
c*par cfail   out       error message
c
c
      double precision  eps, epsdet
      parameter         (eps=1.d-10, epsdet=1.d+30)
      integer           m, n, n1
      integer           i, j, in, jj, k, jn, n2, jn2
      double precision  y(m), qyd(m), a(m*n),
     #                  x(n), qx(n*(n+1)/2), e(m), qed(m),
     #                  qx2((n-n1)*(n-n1+1)/2),
     #                  lndet, det, lnepsd
      double precision  qxatji, xj, yadj
      character*(*)     cfail
      logical           part
c
c ---------- CHECK m AND n --------------------------------------------
c
      cfail = ' '
      if (n.gt.m) then
         cfail = ' err fmlsa3: more unknowns than observ. (m > n)'
         go to 99
      end if
      if (n1.gt.0.and.n1.lt.n) then
         part = .true.
         n2 = n - n1
      else
         part = .false.
      end if
c
c ---------- NORMAL MATRIX qxINV = aT*qyINV*a ------------------------
c
      do 30 j = 1,n
         jn = j*(j-1)/2
         do 31 jj = 1,j
            k = jn + jj
            qx(k) = 0.d0
            do 32 i = 1,m
               in = n*(i-1)
               qx(k) = qx(k) + a(in+j)*a(in+jj)/qyd(i)
   32       continue
   31    continue
   30 continue
c
c ---------- PARTIAL NORMAL MATRIX AND INVERSE ------------------------
c
      if (part) then
         do 40 j = 1,n2
            jn2 = j*(j-1)/2
            jn  = (n1+j)*(n1+j-1)/2 + n1
            do 45 jj = 1,j
               qx2(jn2+jj) = qx(jn+jj)
   45       continue
   40    continue
         call fmcovd (qx2,n2,lndet,eps,cfail)
         if (cfail.ne.' ') go to 99
      end if
c
c ---------- INVERSE AND DETERMINANT ----------------------------------
c
      call fmcovd (qx,n,lndet,eps,cfail)
      if (cfail.ne.' ') go to 99
      lnepsd = dlog(epsdet)
      if (lndet.lt.-lnepsd) then
         write (cfail,'('' err fmlsa3: determinant normal matrix too'',
     #                  '' small (<'',d8.1,'')'')') 1.d0/epsdet
         go to 99
      end if
c      if (lndet.gt.lnepsd) then
c         write (cfail,'('' err fmlsa3: determinant normal matrix too'',
c     #                  '' large (>'',d8.1,'')'')') epsdet
c         go to 99
c      end if
      det = dexp(lndet)
c
c ---------- ls-ESTIMATES UNKNOWNS x = qx*aT*qyINV*y ------------------
c ---------- DIAG.EL. VAR.MAT. RESIDUALS qe = qy - a*qx*aT ------------
c ---------- ls-RESIDUALS e = y - a*x ---------------------------------
c
      do 50 i = 1,m
         qed(i) = qyd(i)
   50 continue
c
      do 60 j = 1,n
         xj = 0.d0
         do 61 i = 1,m
            qxatji = 0.d0
            in = n*(i-1)
c
c           COMPUTE ONE ELEMENT qxatji OF qx*aT
c
            do 62 jj = 1,j
               qxatji = qxatji + qx(j*(j-1)/2+jj)*a(in+jj)
   62       continue
            do 63 jj = j+1,n
               qxatji = qxatji + qx(jj*(jj-1)/2+j)*a(in+jj)
   63       continue
c
c           UPDATE x(J) AND qed(I) FOR THIS ELEMENT
c
            xj    = xj    + qxatji*y(i)/qyd(i)
            qed(i) = qed(i) - a(in+j)*qxatji
   61    continue
         x(j) = xj
   60 continue
c
c           COMPUTE e(i)
c
      do 70 i = 1,m
         yadj = 0.d0
         in = n*(i-1)
         do 71 j = 1,n
            yadj = yadj + a(in+j)*x(j)
   71    continue
         e(i) = y(i) - yadj
   70 continue
c
   99 return
      end
c *********************************************************************
      subroutine fmlsa4 (m,n,n1,y,qy,a,x,qx,qx2,e,qe,detqy,detn,cfail)
c     *****************
c
c*ver version 1.0  d.d. 30-10-1989         modified:
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol least squares adjustment of a linear set of equations with
c*rol m correlated observations and n unknowns.
c*rol this routine is an extention of fmlsa2. also the variance matrix
c*rol (qx2) of the last n2 unknowns is given for computation of the
c*rol external reliability on the first n1 unknowns (e.g. by terel4).
c*rol in : observations (y), undertriangle variance matrix (qy),
c*rol      design matrix (a)
c*rol out: ls-est. unknowns (x), undertr. variance matrix (qx),
c*rol      ls-residuals (e), undertr. variance matrix (qe),
c*rol      undertr. variance matrix (qx2) last n-n1 unknowns
c*rol row-wise stored full matrices. the determinants of qy and
c*rol the normal matrix are computed and eventually error
c*rol messages are given.
c
c     referenced routines: fmcovd (inversion)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par m       in        number of observations
c*par n       in        number of unknowns
c*par n1      in        (first) part unknowns (for part. ext. reliab.)
c*par y       in        observations (m)
c*par qy      in        undertriangle variance matrix y (m(m+1)/2)
c             out       undertriangle weight matrix y
c*par a       in        design matrix (m*n)
c*par x       out       least squares estimates unknowns (n)
c*par qx      out       undertriangle variance matrix x (n(n+1)/2)
c*par qx2     out       variance matrix last n-n1 unknowns
c*par e       out       least squares estimates residuals (m)
c*par qe      out       undertriangle variance matrix e (m(m+1)/2)
c*par detqy   out       determinant qy
c*par detn    out       determinant normal matrix.
c*par cfail   out       error message
c
      double precision  eps, epsdet
      parameter         (eps=1.d-10, mmax=100, nmax=100, epsdet=1.d+30)
      integer           m, n, n1
      integer           i, ii, in, iin, j, jj, jn, k, n2, jn2
      double precision  y(m), qy(m*(m+1)/2), a(m*n),
     #                  x(n), qx(n*(n+1)/2), e(m), qe(m*(m+1)/2),
     #                  qx2((n-n1)*(n-n1+1)/2),
     #                  detqy, detn,
     #                  lndetq, lndetn, lnepsd,
     #                  qxk, qyai, yi, xj, qxatj, qek,
     #                  bbi, bb(mmax), bj, b(nmax)
      character*(*)     cfail
      logical           part
c
c ---------- CHECK m AND n --------------------------------------------
c  m,n with mmax and nmax must be checked in the main program
      if (n.gt.m) then
         cfail = ' err fmlsa4: more unknowns than observ. (n>m)'
         go to 99
      end if
      if (n1.gt.0.and.n1.lt.n) then
         part = .true.
         n2 = n - n1
      else
         part = .false.
      end if
c
c ---------- INVERSE + DETERMINANT qy --------------------------------
c
      do 10 k = 1,m*(m+1)/2
         qe(k) = qy(k)
   10 continue
c
      call fmcovd (qy,m,lndetq,eps,cfail)
      if (cfail.ne.' ') go to 99
c
      lnepsd = dlog(epsdet)
cr      if (lndetq.lt.-lnepsd) then
cr         write (cfail,'('' err fmlsa4: determinant QY too small'',
cr     #                  '' (<'',d8.1,'')'')') 1.d0/epsdet
cr         go to 99
cr      end if
cr      if (lndetq.gt.lnepsd) then
cr         write (cfail,'('' err fmlsa4: determinant QY too large'',
cr     #                  '' (<'',d8.1,'')'')') epsdet
cr         go to 99
cr      end if
      detqy = dexp(lndetq)
c
c ---------- NORMAL MATRIX, INVERSE + DETERMINANT --------------------
c
c      ----- qxINV = aT*qyINV*a
c
      do 20 j = 1,n
         jn = j*(j-1)/2
         do 21 jj = 1,j
            k   = jn+jj
            qxk = 0.d0
c
            do 22 i = 1,m
               qyai = 0.d0
               in    = i*(i-1)/2
               do 23 ii = 1,i
                  qyai = qyai + qy(in+ii)*a(n*(ii-1)+jj)
   23          continue
               do 24 ii = i+1,m
                  qyai = qyai + qy(ii*(ii-1)/2+i)*a(n*(ii-1)+jj)
   24          continue
               qxk = qxk + a(n*(i-1)+j)*qyai
   22       continue
c
            qx(k) = qxk
   21    continue
   20 continue
c
c      ----- PARTIAL NORMAL MATRIX AND INVERSE
c
      if (part) then
         do 30 j = 1,n2
            jn2 = j*(j-1)/2
            jn  = (n1+j)*(n1+j-1)/2 + n1
            do 35 jj = 1,j
               qx2(jn2+jj) = qx(jn+jj)
   35       continue
   30    continue
         call fmcovd (qx2,n2,lndetn,eps,cfail)
         if (cfail.ne.' ') go to 99
      end if
c
c      ----- INVERSION OF NORMALMATRIX
c
      call fmcovd (qx,n,lndetn,eps,cfail)
      if (cfail.ne.' ') go to 99
c
c
cr      call fmprts (6,qx,n,2,200,10,cfail)
cr      write(*,*) 'det qxinv ', dexp(lndetn)
c
      if (lndetn.lt.-lnepsd) then
         write (cfail,'('' err fmlsa4: determinant normal matrix too'',
     #                  '' small (<'',d8.1,'')'')') 1.d0/epsdet
         go to 99
      end if
      if (lndetn.gt.lnepsd) then
         write (cfail,'('' err fmlsa4: determinant normal matrix too'',
     #                  '' large (<'',d8.1,'')'')') epsdet
         go to 99
      end if
      detn = dexp(lndetn)
c
c ---------- RIGHT-HAND SIDE  b = aT*qyINV*y --------------------------
c
c      ----- bb = qyINV*y
c
      do 40 i = 1,m
         bbi   = 0.d0
         bb(i) = 0.d0
         in = i*(i-1)/2
         do 41 ii = 1,i
            bbi = bbi + qy(in+ii)*y(ii)
   41    continue
         do 42 ii = i+1,m
            bbi = bbi + qy(ii*(ii-1)/2+i)*y(ii)
   42    continue
         bb(i) = bbi
   40 continue
c
c      ----- b = aT*bb
c
      do 50 j = 1,n
         bj = 0.d0
         do 51 i = 1,m
            bj = bj + a(n*(i-1)+j)*bb(i)
   51    continue
         b(j) = bj
   50 continue
c
c ---------- ls-ESTIMATES UNKNOWNS x = qx*b ---------------------------
c
      do 60 j = 1,n
         xj   = 0.d0
         x(j) = 0.d0
         jn = j*(j-1)/2
         do 61 jj = 1,j
            xj = xj + qx(jn+jj)*b(jj)
   61    continue
         do 62 jj = j+1,n
            xj = xj + qx(jj*(jj-1)/2+j)*b(jj)
   62    continue
         x(j) = xj
   60 continue
c
c ---------- ls-RESIDUALS e = y - a*x ---------------------------------
c
      do 70 i = 1,m
         e(i) = 0.d0
         yi   = 0.d0
         in   = n*(i-1)
         do 71 j = 1,n
            yi = yi + a(in+j)*x(j)
   71    continue
         e(i) = y(i) - yi
   70 continue
c
c ---------- VARIANCE MATRIX RESIDUALS qe = qy - a*qx*aT ------------
c
      do 80 i = 1,m
         in = i*(i-1)/2
         do 81 ii = 1,i
            k   = in+ii
            iin = n*(ii-1)
            qek = qe(k)
c
            do 82 j = 1,n
               qxatj = 0.d0
               jn    = j*(j-1)/2
               do 83 jj = 1,j
                  qxatj = qxatj + qx(jn+jj)*a(iin+jj)
   83          continue
               do 84 jj = j+1,n
                  qxatj = qxatj + qx(jj*(jj-1)/2+j)*a(iin+jj)
   84          continue
               qek = qek - a(n*(i-1)+j)*qxatj
   82       continue
c
            qe(k) = qek
   81    continue
   80 continue
c
cr      call fmprts (6,qe,m,2,200,10,cfail)
   99 return
      end
c *********************************************************************
      integer function ilen(string)
c*ver version 1, d.d. 07-09-85
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. dutch foundation for pure scientific    *
c     *                     research  (z.w.o.)                      *
c     *                                                             *
c     ***************************************************************  
c*rol compute the "true" length of a string
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c*par string input    character string with trailing blanks
c*par ilen   output   true length of the character string
c
      character string*(*)
c
      length=len(string)
      do 10 ilen=length,1,-1
         if (string(ilen:ilen).gt.' ') goto 20
   10 continue
c
   20 return
      end
c ******************************************************************
      double precision function entd2(prompt,def)
c*ver version 1, d.d. 15-06-86
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. netherlands organisation for the        *
c     *                     advancement of pure research  (z.w.o.)  *
c     *                                                             *
c     ***************************************************************  
c*rol interactive input of a d.p. number with a default
c*rol enter <cr> with no other input to get the default
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c     prompt input    character string with the question (prompt)
c     def    input    d.p. number with the default
c     entd2  function d.p. number with the answer
c
      implicit double precision (a-h,o-z)
      character prompt*(*),entc2*14,defc*14,val*14
c
      write (defc,fmt='(g14.5)') def
   10 val=entc2(prompt,defc,l)
      read  (val(1:l),fmt='(g20.14)',err=10,end=10) entd2
c
      return
      end
c ******************************************************************
      logical function entl2(prompt,ldef)
c*ver version 1, d.d. 15-06-86
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. netherlands organisation for the        *
c     *                     advancement of pure research  (z.w.o.)  *
c     *                                                             *
c     ***************************************************************  
c*rol interactive input of a boolean with a default
c*rol enter <cr> with no other input to get the default
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c     prompt input    character string with the question (prompt)
c     ldef   input    default value for the boolean
c     entl2  function boolean with the answer
c
      character prompt*(*),entc2*3,defc*3,val*3
      logical ldef
c
      defc='n'
      if (ldef) defc='y'
   10 val=entc2(prompt,defc,l)
      if ((val(1:1).eq.'Y').or.(val(1:1).eq.'y')) then
         entl2=.true.
      elseif ((val(1:1).eq.'N').or.(val(1:1).eq.'n')) then
         entl2=.false.
      else
         goto 10
      endif
c
      return
      end
c *********************************************************************
      character*(*) function entc1(prompt,lena)
c*ver version 1, d.d. 15-06-86
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. netherlands organisation for the        *
c     *                     advancement of pure research  (z.w.o.)  *
c     *                                                             *
c     ***************************************************************  
c*rol interactive input of a character string
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c     prompt input    character string with the question (prompt)
c     lena   output   length of the answer string
c     entc1  function string with the answer
c
      character prompt*(*),blank*60
      data blank(1:40) /'                                        '/
      data blank(41:60)/'                    '/
c
      lenp=len(prompt)
      lenm=len(entc1)
      lentx=(lenp/10+1)*10
      lentx=max(lentx,60)
      lb=lentx-lenp+1
      lb=max(lb,1)
      lb=min(lb,60)
      ierr=0
   10 write (6,1000) prompt,blank(1:lb)
      read  (5,1001,iostat=iret) entc1
      lena=ilen(entc1)
      if (iret.ne.0) then
         ierr=ierr+1
         if (ierr.le.10) goto 10
      endif
      if (lena.gt.lenm) then
         write (6,*) ' input longer than',LENM,' characters,'
         write (6,*) ' please reenter:'
         ierr=ierr+1
         if (ierr.le.10) goto 10
      endif
      return
 1000 format(1x,a,a,'==> ')
 1001 format(a)
      end
c *********************************************************************
      character*(*) function entc2(prompt,deflt,lena)
c*ver version 1, d.d. 15-06-86
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. netherlands organisation for the        *
c     *                     advancement of pure research  (z.w.o.)  *
c     *                                                             *
c     ***************************************************************  
c*rol interactive input of a character string with a default
c*rol enter <cr> with no other input to get the default
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c     prompt input    character string with the question (prompt)
c     deflt  input    character string with the default string
c     lena   output   length of the answer string
c     entc2  function string with the answer
c
      character prompt*(*),deflt*(*),blank*60
      data blank(1:40) /'                                        '/
      data blank(41:60)/'                    '/
c
      lenp=len(prompt)
      lend=len(deflt)
      lenm=len(entc2)
c     remove leading and trailing blanks from the default string
      j=0
      do 10 i=1,lend
         if (deflt(i:i).ne.' ') goto 20
   10 continue
   20 do 30 j=lend,i,-1
         if (deflt(j:j).ne.' ') goto 40
   30 continue
   40 lend=j-i+1
      if (lend.gt.lenm) lend=lenm
      entc2(1:lend)=deflt(i:j)
      lent=lenp+lend+4
      lentx=(lenp/10+1)*10
      lentx=max(lentx,60)
      lb=lentx-lent+1
      lb=max(lb,1)
      lb=min(lb,60)
      ierr=0
   50 write (6,1000) prompt,entc2(1:lend),blank(1:lb)
      read  (5,1001,iostat=iret) entc2
      lena=ilen(entc2)
      if (iret.ne.0) then
         ierr=ierr+1
         if (ierr.le.10) goto 50
      endif
      if (lena.eq.0) then
         entc2(1:lend)=deflt(i:j)
         lena=lend
      endif
      if (lena.gt.lenm) then
         write (6,*) ' input longer than',LENM,' characters,'
         write (6,*) ' please reenter:'
         ierr=ierr+1
         if (ierr.le.10) goto 50
      endif
      return
 1000 format(1x,a,' [',a,']',a,' ==> ')
 1001 format(a)
      end
c *********************************************************************
      integer function ient1(prompt)
c*ver version 1, d.d. 15-06-86
c*aut delft geodetic institute, h. van der marel
c     ***************************************************************
c     *                                                             *
c     *   copyright by:  1. delft university of technology,         *
c     *                     department of geodesy                   *
c     *                  2. netherlands organisation for the        *
c     *                     advancement of pure research  (z.w.o.)  *
c     *                                                             *
c     ***************************************************************  
c*rol interactive input of a integer number
c
c     name   i/o      parameter description
c     ------ -------- -------------------------------------------------
c     prompt input    character string with the question (prompt)
c     ient1  function integer with the answer
c
      character prompt*(*),entc1*10,val*10,cfmt*5
c
   10 val=entc1(prompt,l)
      write (cfmt,fmt='(''(i'',i2,'')'')') l
      read  (val(1:l),fmt=cfmt,err=10,end=10) ient1
c
      return
      end
c *********************************************************************
      integer function iposlt(i,j)
c*ver version 1.0  d.d.                    modified:
c*sys ms-dos 3.20, rm-fortran 2.11
c*aut martin salzmann
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           section mathematical and physical geodesy   *
c     *********************************************************
c
c*rol compute position of element i,j in lower triangular matrix
c
c     <name> <access_type> <data_type> <description>
c
c*par i      input   row number element
c*par j      input   column number element
c*par iposlt output  position of element i,j
c
      integer i,j

       if(i.ge.j) then
          iposlt = (i*(i-1))/2 + j
       else
          iposlt = (j*(j-1))/2 + i
       endif

       return
       end
c *********************************************************************
      subroutine fmcovd (a,n,lndet,eps,cfail)
c     *****************
c
c*ver version 1.0  d.d. 29-08-1989         modified:
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol matrix inversion symmetric matrix with in-place algorithm
c*rol (with pivotting).
c*rol the logarithm of the determinant of matrix a is computed.
c*rol this routine equals m. salzmann's routine fmcove exept for the
c*rol determinant computation
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par a       in        symmetric matrix to be inverted (m(m+1)/2)
c*par n       in        number of rows (columns) a
c*par lndet   out       logarithm determinant a
c*par eps     in        stopping criterion pivots
c*par cfail   in        error message
c
c
      parameter         (idim=100)
      double precision  a(((n+1)*n)/2)
      double precision  eps
      double precision  pivmax
      double precision  mult
      double precision  lndet
      integer           n,numpiv,iposa,iposa2,ak,akk
      logical           pivot(idim)
      character*(*)     cfail

      do 100 i=1,idim
         pivot(i) = .false.
100   continue
      cfail = ' '

      if(n.gt.idim) then
         write (cfail,'('' err fmcovd: dimension matrix too large (>'',
     #                  i3,'')'')') idim
         goto 999     
      end if
c                      search for largest pivot
      lndet = 0.d0
      do 101 k=1,n 
         pivmax = 0.0d0
         do 102 l=1,n
            iposa = ((l+1)*l)/2  
            if(dabs(a(iposa)).ge.pivmax) then
               if(.not.pivot(l)) then
                  pivmax = dabs(a(iposa))
                  if(pivmax.lt.eps) then
                     write (cfail,'('' err fmcovd: pivot too small (<'',
     #                              d8.1,'')'')') eps
                     goto 999
                  end if
                  numpiv   = l
               end if
            end if
102      continue
         pivot(numpiv) = .true.
         lndet = lndet + dlog(pivmax)

         ak = ((numpiv-1)*numpiv)/2
         akk= ((numpiv+1)*numpiv)/2

         do 200 i=1,n
            if(i.ne.numpiv) then
               if(i.ge.numpiv) then
                  iposa = ((i-1)*i)/2+numpiv
               else
                  iposa = ak +i
               endif
               mult = a(iposa)/a(akk)
               do 201 j=1,i
                  if(j.ne.numpiv) then
                     iposa = ((i-1)*i)/2+j
                     if(numpiv.ge.j) then
                        iposa2 = ak+j
                     else
                        iposa2 = ((j-1)*j)/2+numpiv
                     endif
                     a(iposa) = a(iposa)-a(iposa2)*mult
                  end if
201            continue
            end if
200      continue

         do 300 j=1,numpiv-1
            a(ak+j)= -a(ak+j)/a(akk)
300      continue
 
         do 400 i=numpiv+1,n
            iposa = ((i-1)*i)/2+numpiv
            a(iposa) = -a(iposa)/a(akk)
400      continue
       
         a(akk) = -1.0/a(akk)

101   continue

      iposa = ((n+1)*n)/2
      do 107 i=1,iposa
         a(i) = -a(i)
107   continue     

999   return
      end                           
c *********************************************************************
      subroutine bmeth1 (alpha1,gamma0,n,x1,lambda0,alphan,xn)
c     *****************
c
c*ver version 1.0  d.d. 23-02-88               modified: 30-08-89
c*sys ms-dos 3.30, rm-fortran 2.11
c*aut frank kenselaar, delft geodetic computing centre
c     *********************************************************
c     *   (c)     delft university of technology              *
c     *           faculty of geodesy                          *
c     *           geodetic computing centre                   *
c     *********************************************************
c
c*rol if alpha1,gamma0 and n are given all other statistical parameters
c*rol are computed according to the b-method of testing.
c*rol if n>=100 approximations are used.
c*rol the necessary stat__ routines are included in this
c*rol file to obtain one object file.
c
c     referenced routines: stat00,stat10,stat11,stat20,stat21,
c                          stat22,stat23,stat31,stat32,stat34,
c                          stat35,stat36 (all included)
c
c     name    i/o       parameter description
c     ------  --------  -----------------------------------------------
c*par alpha1  in        level of significance 1 degree of freedom
c*par gamma0  in        power of the test
c*par n       in        degrees of freedom multi dim. test
c*par x1      out       critical value 1 degree of freedom
c*par lambda0 out       shifting variate
c*par alphan  out       level of significance n degrees of freedom
c*par xn      out       critical value n degrees of freedom
c
c
	implicit double precision (a-h,k-m,o-z)
c
	if (alpha1.eq.0.001.and.gamma0.eq.0.80) then
		x1 = 10.8278
		lambda0 = 17.0749
		go to 10
	end if
c
c ---- COMPUTATION x1 -----------------------------------------------
c
	x1 = stat23(alpha1,1)
c
c ---- COMPUTATION lambda0 ------------------------------------------
c
	lambda0 = stat36(x1,alpha1,gamma0,1)
c
c ---- COMPUTATION alphan AND xn ------------------------------------
c
   10 	if (n.lt.100) then
		call stat34(xn,alphan,gamma0,lambda0,n)
	else
		xn = stat32(gamma0,lambda0,n)
		alphan = stat20(xn,n)
	end if
c
	return
	end

c ******************************************************************
 	real*8 function stat00 (x)
c	********************
c
c	purpose  : approximation of the error function
c	           2/sqrt(phi).integral(0,x)[exp(-t**2)]dt
c	           abramowitz/stegun 1965 (7.1.26)
c	           eps <= 1.5e-7
c	---------------------------------------------------------------
c	variables : x      input    r*8   upper boundary integral
c	            stat00 output   r*8   integral error function
c ---------------------------------------------------------------------
c
c
	implicit double precision (a-h,k-m,o-z)
	parameter (p = 0.3275911 , a1 = 0.254829592 , a2 = -0.284496736 ,
     #       a3 = 1.421413741 , a4 = -1.453152027 , a5 = 1.061405429)
c
	y  = 1.0/(1+p*x)
	y2 = y*y
	y3 = y2*y
	y4 = y3*y
	y5 = y4*y
c
	stat00 = 1.0 - (a1*y + a2*y2 + a3*y3 + a4*y4 + a5*y5)*exp(-x**2)
c
	return
	end
c
c *******************************************************************
        real*8 function stat10 (x)
c	**************************
c
c	purpose  : computes the upper-tail level of significance of the
c	           standard normal distribution.
c	           alpha = 1/sqrt(2*phi).integral(x,inf)[exp(-t**2/2)]dt
c	                 = (1 - erf(x/sqrt(2))/2
c	           the error function is computed by stat00 (1.5e-7)
c	---------------------------------------------------------------
c	referenced routines : stat00
c	variables : x       input    r*8    critical value (upper-tail)
c	            stat10  output   r*8    level of significance
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
c
	y = x/dsqrt(2.d0)
	erf = stat00(y)
	stat10 = (1.d0 - erf)/2.d0
c
	return
	end
c
c
c *******************************************************************
	real*8 function stat11 (alpha)
c	******************************
c
c     purpose  : approximation of the critical value of the
c	           standard normal distribution.
c	           approximation from abramowitz/stegun 1965 26.2.23
c	           eps <= 4.5e-4 .
c	---------------------------------------------------------------
c	variables : alpha   input   r*8   level of significance
c	            stat11  output  r*8   approximation critical value
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
	parameter (c0 = 2.515517, c1 = 0.802853, c2 = 0.010328,
     #	     d1 = 1.432788, d2 = 0.189269, d3 = 0.001308)
c
	if (alpha.gt.0.5) then
		alp = 1.0 - alpha
		sign = -1.0
	else
		alp = alpha
		sign = 1.0
	end if
c
	t = sqrt(log(1/alp**2))
	t2 = t*t
	t3 = t2*t
c
	c = c0 + c1*t + c2*t2
	d = 1.0 + d1*t + d2*t2 + d3*t3
c
	stat11 = sign*(t - c/d)
c
	return
	end
c
c *******************************************************************
	real*8 function stat20 (x,n)
c	****************************
c
c	purpose  : approximates the upper-tail level of significance of
c	           the chisquare/n (fN,INF) distribution in x for large
c	           degrees of freedom. (4 decimals if n>=100)
c	           abramowitz/stegun 1965 (26.4.14)
c	---------------------------------------------------------------
c	referenced routines : stat10 (stat00)
c	variables : x      input    r*8   critical value
c	            n      input    i     degrees of freedom
c	            stat20 output   r*8   level of significance
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
c
	p = 2.d0/(9.d0*n)
	y = (x**(1.d0/3.d0) - 1.d0 + p)/dsqrt(p)
c
c ---- STANDARD NORMAL DISTRIBUTION WITH y AS CRITICAL VALUE --------
c
	stat20 = stat10(y)
c
	return
	end
c
c *******************************************************************
	double precision function stat21 (x,n)
c	**************************************
c
c     purpose  : computes the upper-tail level of significance of
c	           the chisquare/n- (or fN,INF-) distribution.
c	           alpha = (n/2)**2/gamma(n/2)*integral(x,inf)
c	                   [y**(b/2-1)*exp(-b/2*y)]dy
c	           de kruijf 1967 (lgr r55)
c	---------------------------------------------------------------
c	referenced routines : stat00
c	variables : x      input   r*8   critical value
c	            n      input   i     degrees of freedom
c	            stat21 output  r*8   level of significance
c ---------------------------------------------------------------------
c
c
	implicit double precision (a-h,k-m,o-z)
	parameter (phi = 3.1415926536)
c
c ---- DIFFERENT SOLUTIONS FOR N = ODD OR N = EVEN ------------------
c
    5 if ((2*int(n/2.0)).eq.n) then
		go to 10
	else
		go to 20
	end if
c
c ---- N = EVEN -----------------------------------------------------
c
   10 z = n/2*x
	a = dexp(-z)
      sum = a
	do 100 i=1,(n/2-1)
		a = z/i*a
            sum = sum + a
  100 continue
	stat21 = sum
	go to 30
c
c ---- N = ODD ------------------------------------------------------
c
   20 z = n/2.d0*x
	a = dexp(-z)/dsqrt(z*phi)
	sum = 1.d0 - stat00(dsqrt(z))
	do 200 i=1,(nint(n/2.d0-0.5))
		a = z/(i-0.5)*a
		sum = sum + a
  200 continue
	stat21 = sum
c
   30 return
	end
c
c *******************************************************************
	real*8 function stat22 (alpha,n)
c	********************************
c
c	purpose   : approximation of the critical value of the
c	            chisquare/n (fN,INF) distribution by the standard
c	            normal distribution.
c	            abramowitz/stegun 1965 (26.4.17). for n >= 100 the
c	            approximation suits to 3 decimals.
c	---------------------------------------------------------------
c	referenced routines : stat11
c	variables : alpha   input   r   level of significance
c	            n       input   i   degrees of freedom
c	            stat22  output  r   approximation critical value
c ---------------------------------------------------------------------
c
c
	implicit double precision (a-h,k-m,o-z)
c
c ---- CRITICAL VALUE FROM THE STANDARD NORMAL APPROXIMATION stat11 ---
c
	y = stat11(alpha)
c
c ---- aBRAMOWITZ/sTEGUN 26.4.17 --------------------------------------
c
	p = 2.0/(9.0*n)
	x = (1.0 - p + y*sqrt(p))**3
	stat22 = x
c
	return
	end
c
c *******************************************************************
	double precision function stat23 (alpha,n)
c	******************************************
c
c	purpose  : computes the critical value from the chisquare/n
c	           (fN,INF) distribution by newton raphson iteration
c	           of the integration of the probability-function.
c	           de kruif 1967 (lgr r55)
c	---------------------------------------------------------------
c	referenced routines : stat21, stat22
c	variables : alpha   input   r*8   level of significance
c	            n       input   i     degrees of freedom
c	            stat23  output  r*8   critical value
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
c
c ---- APPROXIMATION BY stat22 ----------------------------------------
c
	x = stat22(alpha,n)
c
c ---- NEWTON-RAPHSON ITERATION --------------------------------------
c
   10 f  = stat21(x,n) - alpha
	df = deriv(x,n)
	dx = f/df
	x  = x - dx
c
	if (abs(dx).gt.1.0e-4) go to 10
      stat23 = x
c
	return
	end
c *******************************************************************
	real*8 function deriv (x,n)
c	***************************
c
c	purpose : computes the derivative of the level of significance
c	          of the chisquare/n- (or fN,INF-) distribution in
c	          point x.
c	          the derivative is necessary for the newton/raphson
c	          iteration proces to compute the critical value.
c	          de kruijf 1967 (lgr r55)
c
	implicit double precision (a-h,k-m,o-z)
	parameter (phi = 3.1415926536)
c
c ---- DIFFERENT SOLUTIONS FOR N = ODD OR N = EVEN ------------------
c
	if ((2*int(n/2.0)).eq.n) then
		go to 10
	else
		go to 20
	end if
c
c ---- N = EVEN -----------------------------------------------------
c
   10 z = n/2*x
	sum = -dexp(-z)
      b = -sum/z
	do 100 i=1,(n/2-1)
		b = z/i*b
            sum = sum + b*(i-z)
  100 continue
	deriv = n/2*sum
	go to 30
c
c ---- N = ODD ------------------------------------------------------
c
   20 z = n/2.d0*x
	sum = -dexp(-z)/dsqrt(z*phi)
	b = -sum/z
	do 200 i=1,(nint(n/2.d0-0.5))
		b = z/(i-0.5)*b
		sum = sum + b*(i-0.5-z)
  200 continue
	deriv = n/2.d0*sum
c
   30 return
	end
c
c *******************************************************************
	double precision function stat31 (x,alpha,lambda,n)
c	***************************************************
c
c	purpose   : computes the probability gamma of the non-central
c	            chisquare/n (fN,INF) distribution.
c	            de kruif 1967 (lgr r55)
c	---------------------------------------------------------------
c	variables : x       input  r*8  critical value
c	            alpha   input  r*8  level of significance
c	            lambda  input  r*8  shifting variate
c	            n       input  i    degrees of freedom
c	            stat31  output r*8  probability (gamma)
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
	parameter (phi = 3.1415926536)
c
c ---- INITIAL VALUES RECURSIVE COMPUTATION ---------------------------
c
     	z = n/2.d0*x
	y = lambda/2.d0
c
	if ((2*int(n/2.0)).eq.n) then
		d = dexp(-z)
		do 10 i=1,n/2
			d = d*z/i
   10 	continue
	else
		d = dexp(-z)*2.d0*dsqrt(z/phi)
		do 20 i=1,(nint(n/2.0 - 0.5))
			d = d*z/(i + 0.5)
   20 	continue
	end if
c
	c = d
	b = y
	dsom = b*c
	som = dsom
	j = 1
c
c ---- RECURSIVE COMPUTATION ------------------------------------------
c
   30 j = j+1
	b = y/j*b
	d = z/(n/2.d0 + j - 1.d0)*d
	c = c+d
	dsom = b*c
	som = som + dsom
c
	if (dabs(dsom).gt.(som*1.0e-8)) go to 30
	gamma = dexp(-y)*som + alpha
c
     	stat31 = gamma
	return
	end
c
c *******************************************************************
	real*8 function stat32 (gamma,lambda,n)
c	***************************************
c
c	purpose   : approximation of the critical value of the non-central
c	            chisquare/n (fN,INF) distribution from the standard
c	            normal distribution.
c	            abramowitz/stegun 1965 (26.4.32)
c	            for n >= 100 this approximation suits at 3 decimals.
c	---------------------------------------------------------------
c	referenced routines : stat11
c	variables : gamma    input   r*8   probability
c	            lambda  input   r*8   shifting variate
c	            n       input   i     degrees of freedom
c	            stat32  output  r*8   critical value
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
c
	a = n + lambda
	b = lambda/a
	p = 2.0/9.0*(1.0+b)/a
c
	y = stat11(gamma)
c
	x = a/n*(y*sqrt(p) +1.0 - p)**3
	stat32 = x
c
	return
	end
c
c *******************************************************************
	subroutine stat34 (x,alpha,gamma,lambda,n)
c	******************************************
c
c	purpose   : computes the critical value x and the level of
c	            significance alpha of the non-central chisquare/n
c  	            (fN,INF) distribution by regula-falsi iteration
c	            of the gamma-computation (stat31).
c	            x is iterated, alpha is computed by stat21. if alpha
c	            is known stat33 can be used.
c	---------------------------------------------------------------
c	referenced routines : stat31, stat32 (stat11),
c                           stat21 (stat10, stat00)
c     variables :	gamma   input   r*8  probability non-central chi/n
c	            lambda  input   r*8  shifting variate
c	            n       input   i    degrees of freedom
c	            x       output  r*8  critical value
c	            alpha   output  r*8  level of significance
c ---------------------------------------------------------------------
c
c
	implicit double precision (a-h,k-m,o-z)
c
c ---- APPROXIMATION BY stat32 TO OBTAIN INITIAL VALUES ---------------
c
	x0 = stat32(gamma,lambda,n)
	alpha0 = stat21(x0,n)
	f0 = stat31(x0,alpha0,lambda,n) - gamma
c
	if (n.lt.5) then
		p = 0.05
	else
		if (n.lt.20) then
			p = 0.01
		else
			if (n.lt.50) then
				p = 0.001
			else
				p = 0.0001
			end if
		end if
	end if
c
	x1 = x0 - p
	alpha1 = stat21(x1,n)
	f1 = stat31(x1,alpha1,lambda,n) - gamma
c
	if (f0.lt.0.d0) then
		if (f1.gt.f0) go to 10
      else
		if (f1.lt.f0) go to 10
	end if
		p = -p
		x1 = x0 - p
		alpha1 = stat21(x1,n)
		f1 = stat31(x1,alpha1,lambda,n) - gamma
c
   10	if (f1*f0.lt.0.d0) go to 20
		x1 = x1 - p
		alpha1 = stat21(x1,n)
		f1 = stat31(x1,alpha1,lambda,n) - gamma
		go to 10
c
c ---- ITERATION PROCES ------------------------------------------------
c
   20	x = (x0*f1 - x1*f0)/(f1 - f0)
	alpha = stat21(x,n)
	f = stat31(x,alpha,lambda,n) - gamma
c
	if (abs(f).lt.1.0e-6) go to 30
c
	if (f*f0.lt.0.d0) then
		x1 = x
		f1 = f
	else
		x0 = x
		f0 = f
	end if
	go to 20
c
   30	return
	end
c
c *******************************************************************
	real*8 function stat35 (x,gamma,n)
c	**********************************
c
c	purpose   : approximation of the shifting variate lambda of the
c	            non-central chisquare/n (fN,INF) distribution.
c	            the approximation of the critical value (abramowitz/
c	            stegun 1965 26.4.32) is used in an newton/raphson
c	            iteration.
c	            for n >= 100 the approximation suits at 2 decimals.
c	---------------------------------------------------------------
c	referenced routines : stat11
c	variables : x      r  input   critical value
c	            gamma  r  input   probability
c	            n      i  input   degrees of freedom
c			stat34 r  output  shifting variate
c ---------------------------------------------------------------------
c
	implicit double precision (a-h,k-m,o-z)
c
	y = stat11(gamma)
	lam = 16.0 + 1.5*sqrt(real(n))
c
   10 a = lam + n
	b = lam/a
	p = 2.0/9.0*(1.0+b)/a
	q = 2.0/9.0*b/a**2
	r = y*sqrt(p) + 1.0 - p
c
	f  = a/n*r**3 - x
	df = x/a + (3*a/n*r**2)*(-y*q/sqrt(p) + 2.0*q)
c
	dlam = f/df
	lam = lam - dlam
c
	if (dlam.gt.1.0e-3) go to 10
	stat35 = lam
c
	return
	end
c
c *******************************************************************
	double precision function stat36 (x,alpha,gamma,n)
c	**************************************************
c
c	purpose   : computes the shifting variate lambda of the non-
c	            central chisquare/n (fN,INF) distribution by newton/
c	            raphson iteration of the gamma-computation (stat31)
c	---------------------------------------------------------------
c	referenced routines : stat31, stat35 (stat11)
c	variables : x	  input  r*8  critical value
c	            alpha   input  r*8  level of significance
c	            gamma   input  r*8  probability
c	            n       input  i    degrees of freedom
c	            stat35  output r*8  shifting variate
c ---------------------------------------------------------------------
c
c
	implicit double precision (a-h,k-m,o-z)
c
c ---- INITIAL VALUE BY APPROXIMATION stat35 --------------------------
c
	lambda = stat35(x,gamma,n)
c
c ---- NEWTON/RAPHSON ITERATION ---------------------------------------
c
   10 f  = stat31(x,alpha,lambda,n) - gamma
	df = derivl(x,alpha,gamma,lambda,n)
c
	dlambda = f/df
	lambda = lambda - dlambda
c
	if (abs(dlambda).gt.1.0e-4) go to 10
     	stat36 = lambda
c
	return
	end
c
	real*8 function derivl (x,alpha,gamma,lambda,n)
c	********************
c
c	computes the derivative to lambda of the probability of the
c	non-central chisquare/n (fN,INF) distribution.
c	this derivative is necesarry for the newton/raphson iteration
c	proces to compute lambda from the non-central chisquare/n
c	distribution.
c
	implicit double precision (a-h,k-m,o-z)
	parameter (phi = 3.1415926536)
c
c ---- INITIAL VALUES RECURSIVE COMPUTATION ---------------------------
c
     	z = n/2.d0*x
	y = lambda/2.d0
c
	if ((2*int(n/2.0)).eq.n) then
		d = dexp(-z)
		do 10 i=1,n/2
			d = d*z/i
   10 	continue
	else
		d = dexp(-z)*2.d0*dsqrt(z/phi)
		do 20 i=1,(nint(n/2.0 - 0.5))
			d = d*z/(i + 0.5)
   20 	continue
	end if
c
	c = d
	b = 1.d0
	dsom = b*c
	som = dsom
	j = 1
c
c ---- RECURSIVE COMPUTATION ------------------------------------------
c
   30 j = j+1
	b = y/(j-1)*b
	d = z/(n/2.d0 + j - 1.d0)*d
	c = c+d
	dsom = b*c
	som = som + dsom
c
	if (dabs(dsom).gt.(som*1.0e-8)) go to 30
	derivl = (dexp(-y)*som + alpha - gamma)/2.d0
c
	return
	end
c ******************* end of PCSCAN *********************************

