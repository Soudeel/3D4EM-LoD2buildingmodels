
/*
                    Copyright 2014 University of Twente 
 
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

extern "C" void Update_Normal_Eq(double *a, double y, double w, double *ata, 
                                 double *aty, int np);
extern "C" void Partial_Update_Normal_Eq1(double *a, double y, double w, double *ata, 
                                          double *aty, int np, int start1, int end1);
extern "C" void Partial_Update_Normal_Eq2(double *a, double y, double w, double *ata, 
                                          double *aty, int np, int start1, int end1,
							              int start2, int end2);
extern "C" void Solve_Normal_Eq(double *ata, double *aty, int np);
extern "C" void Solve_Normal_Eq_Cond(double *ata, double *aty, int np, double *cond);
extern "C" void Invert_And_Solve_Normal_Eq(double *ata, double *aty, int np, double *cond);
extern "C" void Convert_To_Correlations(double *cov, int np);
extern "C" void Solve_Normal_Eq_Cholesky(double *ata, double *aty, int np);
extern "C" void Solve_Normal_Eq_Cholesky_Band(double *ata, double *aty, int np,
                                              int nb, int hbw);
