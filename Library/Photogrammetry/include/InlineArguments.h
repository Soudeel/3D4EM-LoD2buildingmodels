
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




class InlineArguments {

  protected:
            
    /// Number of inline arguments
    int argc;
    
    /// Inline arguments
    char **argv;
    
  public:
    /// Default constructor
    InlineArguments()
      {argc = 0; argv = NULL;}
      
    /// Constructor
    InlineArguments(int c, char **v)
      {argc = c; argv = v;}
      
    /// Return the index of an argument
    int Index(const char *wanted_arg) const;
    
    /// Check if an argument exists
    bool Contains(const char *wanted_arg) const;
    
    /// Return the integer value of an argument
    int Integer(const char *wanted_arg, int default_value) const;

    /// Return the float value of an argument
    float Float(const char *wanted_arg, float default_value) const;

    /// Return the double value of an argument
    double Double(const char *wanted_arg, double default_value) const;

    /// Return the string value of an argument
    const char * String(const char *wanted_arg,
                        const char *default_value) const;

    /// Return the string value of an argument
    char * String(const char *wanted_arg) const;
};
