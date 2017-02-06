
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



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : June 2002
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Definition of various functions related to palettes.
*
*--------------------------------------------------------------------*/
#ifndef _PALETTES_H_
#define _PALETTES_H_

#include "MyConstants.h"

//declaration of enum for GL color schemes.
enum GLColorScheme
{
	GLColorSchemeScaledGray=0,
	GLColorSchemeScaledRed=7,
	GLColorSchemeScaledGreen=8,
	GLColorSchemeScaledBlue=9,
	GLColorSchemeHot=2,
	GLColorSchemeCool=3,
	GLColorSchemeJet=10,
	GLColorSchemeHSV=11,
	GLColorSchemeStandard=1,
	GLColorSchemeGrayBlue=12,
	GLColorSchemeGrayGreen=13,
	GLColorSchemePastel=14,
	GLColorSchemeArmy=4,
	GLColorSchemeElevation=5,
	GLColorSchemeThresholdedPalette=6
};	


//Load AppropriatePalette.
void LoadSelectedPalette(BYTE data[][3],int size,GLColorScheme glColorScheme);
#endif //_PALETTES_H_
