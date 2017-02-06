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

#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <dirent.h>
#include <errno.h>
#ifdef irix
#  include <sys/types.h>
#endif

/*--- Global declaration ----------------------------------------------------*/

DIR    *DIRECTORY[100];                    /* Array with directory pointers  */

/* The function key_location does the same as the standard strstr function,
   but allows the key string to contain ? wildcards (that match any character).
   It returns the first location of the key in the string to be analysed.
   If the key is not found NULL is returned.
   If shift_key is zero, the key is only matched against the start of the
   string to be analysed. Hence, in this case the only possible returns are the
   address of string and NULL.
*/
const char *key_location(const char *string, const char *key, int shift_key)
{
  int shift, num_shifts, key_length, i, match;
  const char *ch_string, *ch_key;
  
  key_length = strlen(key);
  num_shifts = strlen(string) - key_length + 1;
  if (num_shifts < 1) return NULL;
  for (shift=0; shift<num_shifts; shift++) {
    ch_key = key;
    ch_string = string+shift;
    for (i=0, match=1, ch_key=key, ch_string=string+shift;
         i<key_length && match; i++, ch_key++, ch_string++)
      if (*ch_string != *ch_key && *ch_key != '?') match = 0;
    if (match) return string+shift;
    if (!shift_key) return NULL;
  }
  return NULL;
}

/* Check if the name of a file matches with the specified filter. The filter
   may contain * and ? wildcards, but no further expressions (like [0-9]).
*/
int name_matches_filter(const char *name, const char *filter)
{
  char *name_pos, *filter_pos, *filter_copy;
  const char *remainder, *match_start;
  int  pos, match=1, *key_starts, num_keys, key, len; 
  
  // Checks on name and filter
  if (filter == NULL) return 1;
  if (strlen(filter) == 0) return 1;
  if (name == NULL) return 0;
  if (strlen(name) == 0) return 0;
  if (strcmp(name, ".") == 0 ||           // Current directory
      strcmp(name, "..") == 0) return 0;  // Parent directory

  // Local copy and key array allocation  
  filter_copy = (char *) malloc(strlen(filter)+1);
  strcpy(filter_copy, filter);
  key_starts = (int *) malloc(strlen(filter) * sizeof(int));
  
  // Collect key starts
  num_keys = 0;
  if (filter[0] != '*') {
    key_starts[0] = 0;
    num_keys++;
  }
  for (pos=0; pos<strlen(filter); pos++) {
    if (filter[pos] == '*') {
      filter_copy[pos] = 0; // Replace wildcard by end of string in copy
      if (pos < strlen(filter)-1) { // Check if more characters follow
        key_starts[num_keys] = pos+1; // Store start of next key
        num_keys++;
      }
    }
  }
  
  // Check name start if no wildcard
  if (filter[0] != '*')
    if (key_location(name, filter_copy, 0) == NULL) match = 0;
  
  // Check name end if no wildcard
  if (match && filter[strlen(filter)-1] != '*' && num_keys) {
    len = strlen(filter_copy + key_starts[num_keys-1]);
    if (key_location(name + strlen(name) - len,
                     filter_copy + strlen(filter) - len, 0) == NULL) match = 0; 
    num_keys--;
  }
  
  // Check all keys in sequence
  if (match) {
    remainder = name;
    for (key=0; key!=num_keys && match; key++) {
      match_start = key_location(remainder, filter_copy + key_starts[key], 1);
      if (match_start == NULL) match = 0;
      else remainder = match_start + strlen(filter_copy + key_starts[key]);
    }
  }
   
  free(key_starts);
  free(filter_copy);
  return match;
}

/*--- Read filenames from the directory ------------------------------*/

char *get_filename(const char *dirname, const char *filter_with_quotes,
                   int *icon)
{
  char          buf[BUFSIZ],*t, *filter;
  const char    *s;
  struct dirent *entry;

/* If there is no context, determine a context number, translate the expression
   of the filter, and try to open the directory */

  if (*icon == 0) {
    *icon = 1;
    while (*icon < 100 && DIRECTORY[*icon] != NULL) (*icon)++;
    if (*icon == 100) {*icon = 0; return(NULL);}
 
/* Try to open the directory */

    DIRECTORY[*icon] = opendir(dirname);
    if (DIRECTORY[*icon] == NULL) {
      *icon = 0;
      return(NULL);
    }
  }

// Remove quotes if present
  filter = (char *) malloc(strlen(filter_with_quotes) + 1);
  if (filter_with_quotes[0] == 39) strcpy(filter, filter_with_quotes+1);
  else strcpy(filter, filter_with_quotes);
  int len = strlen(filter);
  if (filter[len-1] == 39) filter[len-1] = 0;

/* Read the next entry, check if it matches the compiled expression, and
   append a blank for end-of-string detection */

  entry = readdir(DIRECTORY[*icon]);
  while (entry != NULL) {
    // d_reclen is always 0 under windows, use strlen instead!
    if (strlen(entry->d_name) != 0) {
      if (name_matches_filter(entry->d_name, filter)) {
        if (entry->d_name[0] != '.')       /* Name does not start with a dot */
          return entry->d_name ;
      }
    }
    entry = readdir(DIRECTORY[*icon]);
  }
  
  closedir(DIRECTORY[*icon]);
  DIRECTORY[*icon] = NULL;
  free(filter);
  return(NULL);
}

char *get_full_filename(const char *dirname, const char *filter, int *icon)
{
  char *shortname, *fullname;
  int  len;

  shortname = get_filename(dirname, filter, icon);
  if (!shortname) return(NULL);
  len = strlen(shortname) + 1;
  if (dirname) len += strlen(dirname);
  fullname = (char *) malloc(len);
  fullname[0] = 0;
  if (dirname) strcat(fullname, dirname);
  strcat(fullname, shortname);
  return(fullname);
}
