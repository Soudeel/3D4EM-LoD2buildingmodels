
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



#include "Database.h"

/*
---------------------------------------------------------------------------
  Returns True if the string is a comment string (if first non space is
  equal to the character defined in comment_char) otherwise returns false
---------------------------------------------------------------------------
*/
int Is_Comment(const char *input_string)
  {
  int i = 0;
  
  while (( (input_string[i] == ' ' ) ||
           (input_string[i] == '\t') ) && (input_string[i] != 0)) i++;
  
  if ((int) input_string[i] == 10) /* newline char preserved by fgets */
    return(True);                  /* Skip this line if it is blank   */
   
  if (input_string[i] == comment_char) 
    return (True);
  else
    return (False);
  }

