
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
 * \brief Interface to Class Timer - Timing functions
 *
 */
/*!
 * \class Timer
 * \brief Interface to Class Timer - Timing functions
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \G_Vosselman
 * \date	11-Oct-2004 (Created)
 *
 * \remark \li Number of any kind of feature (point, line, triangle, etc.)
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _Timer_h_
#define _Timer_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Timer      - Timing functions

--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//------------------------------------------------------------------------------
/// Timer
/** Timing functions */
//------------------------------------------------------------------------------

class Timer {

  protected:
    /// Start time
    clock_t start_time;

  public:

    /// Default constructor with initialisation
    Timer()
      { Reset(); }

    /// Reset the start time
    void Reset()
      { start_time = clock(); }

    /// Number of minutes passed since last timer reset
    double Minutes() const;

    /// Number of seconds passed since last timer reset
    double Seconds() const;

    /// Time passed in format HH:MM:SS.ss
    char * TimeString() const;
};
#endif /* _Timer_h_ */   /* Do NOT add anything after this line */
