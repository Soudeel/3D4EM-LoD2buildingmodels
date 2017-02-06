
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



/*
--------------------------------------------------------------------------------
 Collection of functions for file compression and decompression

 int  Compression_Type(char *)               Determine type of compression
 char *Name_Compressed_File(char *)          Name of existant compressed file
 char *Name_Uncompressed_File(char *)        Name of uncompressed file
 FILE *Open_Compressed_File(char *, char *)  Uncompress a file to a stream
 int  Close_Compressed_File(FILE *)          Close the file and kill child proc.
 int  Decompress_File(char *, char *)        Uncompress a file to a file
 int  Compress_File(char *, char *, int)     Compress a file to a file

 Initial creation
 Author : George Vosselman
 Date   : 05-03-1999

 Update #1
 Author : Ildiko Suveg and George Vosselman
 Date   : 17-03-1999
 Changes: Added decompression to a stream for direct reading of compressed files

 Update #2
 Author : George Vosselman
 Date   : 28-04-1999
 Changes: Added Close_Compressed_File which kills the child process
          used by gunzip

 Update #3
 Author : George Vosselman
 Date   : 20-03-2005
 Changes: Added switch for MSWindows which can not handle pipe and fork.
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

//#include "digphot_arch.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <signal.h>
#if defined irix || defined linux
#  include <errno.h>
#endif

#include "compression.h"

/*
--------------------------------------------------------------------------------
                             Global declarations
--------------------------------------------------------------------------------
*/

int num_running_gunzips = 0;
int num_malloced_ids = 0;
FILE **open_compressed_file_id = NULL;
pid_t *running_gunzip_id = NULL;

/*
--------------------------------------------------------------------------------
                       Determine the type of file compression
--------------------------------------------------------------------------------
*/

int Compression_Type(const char *zipped_file)
{
  if (strcmp(zipped_file + strlen(zipped_file) - 3, ".gz") == 0)
    return(COMPRESSION_GNU);
  else if (strcmp(zipped_file + strlen(zipped_file) - 2, ".Z") == 0)
    return(COMPRESSION_LZW);
  else
    return(COMPRESSION_NONE);
}

/*
--------------------------------------------------------------------------------
                   Determine the name of the compressed file
--------------------------------------------------------------------------------
*/

char *Name_Compressed_File(const char *name)
{
  char *zipped_file;
  FILE *fd;
  
  zipped_file = (char *) malloc(strlen(name) + 4);
  strcpy(zipped_file, name);
  
/* If the file name has the extension of a compressed file, return this name */

  if (Compression_Type(name)) return(zipped_file);

/* Otherwise, check if there are compressed versions */

  else {

/* Check for a GNU ZIP'ped version */

    strcat(zipped_file, ".gz");
    fd = fopen(zipped_file, "r");
    if (fd) {
      fclose(fd);
      return(zipped_file);
    }

/* Check for a compressed version */

    strcpy(zipped_file, name);
    strcat(zipped_file, ".Z");
    fd = fopen(zipped_file, "r");
    if (fd) {
      fclose(fd);
      return(zipped_file);
    }
  }
  
/* If there are no compressed versions, return the original file name */

  strcpy(zipped_file, name);
  return(zipped_file);
}

/*
--------------------------------------------------------------------------------
                   Determine the name of the uncompressed file
--------------------------------------------------------------------------------
*/

char *Name_Uncompressed_File(const char *name)
{
  char *unzipped_file;
  int  compression_type, len;

/* If the file name does not have the extension of a compressed file,
 * return this name
 */

  compression_type = Compression_Type(name);
  if (!compression_type) {
    unzipped_file = (char *) malloc(strlen(name) + 1);
    strcpy(unzipped_file, name);
    return(unzipped_file);
  }

  switch (compression_type) {
    case COMPRESSION_GNU: len = strlen(name) - 3;
                          break;
    case COMPRESSION_LZW: len = strlen(name) - 2;
                          break;
    default             : fprintf(stderr, "Error: unknown compression type.\n");
                          exit(0);
  }
  unzipped_file = (char *) malloc(len+1);
  strncpy(unzipped_file, name, len);
  unzipped_file[len] = 0;
  return(unzipped_file);
}

/*
--------------------------------------------------------------------------------
                      Decompress a file to a stream
--------------------------------------------------------------------------------
*/

FILE *Open_Compressed_File(const char *filename_in, const char *flag)
{
  char  *command;
  const char *filename;
  FILE  *stream;
  int   compression_type, readfile;

/* Prevent segmentation violations later on */

  if (!filename_in) {
    fprintf(stderr, "Error: no file name specified!\n");
    return(NULL);
  }

/* Get the name of the compressed file */

  if (strcmp(flag, "r") == 0 || strcmp(flag, "rb") == 0) {
    /* Look for a compressed version when reading */
    filename = Name_Compressed_File(filename_in);
    readfile = 1;
  }
  else if (strcmp(flag, "w") == 0 || strcmp(flag, "wb") == 0) {
    /* Use GNU compression when writing */
    filename = filename_in;
    readfile = 0;
    compression_type = COMPRESSION_GNU;
    fprintf(stderr, "Writing to compressed files is not yet implemented.\n");
    return(NULL);
  }
  else { /* Invalid flag */
    fprintf(stderr, "Invalid opening flag (%s), should be r or w.\n", flag);
    return(NULL);
  }

/* If the file is not compressed, simply open it and return the descriptor */

  if (readfile) compression_type = Compression_Type(filename);
  if (compression_type == COMPRESSION_NONE) {
    stream = fopen(filename, flag);
    return(stream);
  }
  else if (readfile) { /* Check the existance of the file */
    stream = fopen(filename, flag);
    if (!stream) return(NULL);
    fclose(stream);
  }

#ifdef windows
/*
  fprintf(stderr, "Reading of compressed files is not implemented for Windows.\n");
*/
  return NULL;

#else
/* Compose the (de)compression command */

  command = (char *) malloc(17 + strlen(filename));
  if (readfile) {
    if (compression_type == COMPRESSION_GNU)
      strcpy(command, "gunzip -c ");
    else if (compression_type == COMPRESSION_LZW)
      strcpy(command, "uncompress -c ");
  }
  else {
    if (compression_type == COMPRESSION_GNU)
      strcpy(command, "gzip -c > ");
    else if (compression_type == COMPRESSION_LZW)
      strcpy(command, "compress -c > ");
  }
  strcat(command, filename);

/* Create an interprocess channel */

  stream = (FILE *) popen(command, flag);
  if (stream == NULL) {
    fprintf(stderr, "Error creating pipe for (de)compressing file %s.\n",
            filename);
    return(NULL);
  }
  
/* Return the stream */

  return(stream);
#endif
}

#ifndef windows
FILE *Open_Compressed_File_Old(char *filename_in, char *flag)
{
  char  *filename;
  FILE  *stream;
  int   compression_type, pipe_fd[2], readfile, new_fd;
  pid_t childpid;

/* Prevent segmentation violations later on */

  if (!filename_in) {
    fprintf(stderr, "Error: no file name specified!\n");
    return(NULL);
  }

/* Get the name of the compressed file */

  if (strcmp(flag, "r") == 0) { /* Look for a compressed version when reading */
    filename = Name_Compressed_File(filename_in);
    readfile = 1;
  }
  else if (strcmp(flag, "w") == 0) { /* Take whatever version when writing */
    filename = filename_in;
    readfile = 0;
    fprintf(stderr, "Writing to compressed files is not yet implemented.\n");
    return(NULL);
  }
  else { /* Invalid flag */
    fprintf(stderr, "Invalid opening flag (%s), should be r or w.\n", flag);
    return(NULL);
  }

/* If the file is not compressed, simply open it and return the descriptor */

  compression_type = Compression_Type(filename);
  if (compression_type == COMPRESSION_NONE) {
    stream = fopen(filename, flag);
    return(stream);
  }
  else if (readfile) { /* Check the existance of the file */
    stream = fopen(filename, flag);
    if (!stream) return(NULL);
    fclose(stream);
  }

/* Create an interprocess channel */

  if (pipe(pipe_fd) == -1) {
    fprintf(stderr, "Error creating pipe for (de)compressing file %s.\n",
            filename);
    return(NULL);
  }
  
/* Create a process to execute the decompression or compression */

  switch ( fork() )
  {
    case 1: fprintf(stderr, "Error in fork\n"); exit(0);
    case 0: /* Child process */
    {
      /* Send the child process id to the parent process */

      childpid = getpid();
      write(pipe_fd[1], (const void *) &childpid, sizeof(pid_t));

      /* Uncompress data for reading */

      if (readfile) {
        close(pipe_fd[0]);           /* there is no reading from the pipe */
        close(1);	             /* close stdout */
        new_fd = dup(pipe_fd[1]);    /* duplicate file descriptor on stdout */
        if (new_fd == -1) {
          if (errno == EMFILE)
            fprintf(stderr, "Error: Too many open file descriptors.\n");
          fprintf(stderr, "Error: Could not uncompress file %s\n", filename);
          exit(0);
        }
        close(pipe_fd[1]);
        if (compression_type == COMPRESSION_GNU)
          execlp("gunzip", "gunzip", "-c", filename, NULL);
        else if (compression_type == COMPRESSION_LZW)
          execlp("uncompress", "uncompress", "-c", filename, NULL);
        printf("Error executing decompression.\n");
        exit(1);
      }

      /* Compress written data */
      else {
      }
    }
    default: /* Parent process */
    {
      /* Get the child process id from the pipe */

      read(pipe_fd[0], (void *) &childpid, sizeof(pid_t));

      /* Create a file id for reading uncompressed data from the pipe */

      if (readfile) {
        close(pipe_fd[1]);          /* there is no writing to the pipe */
        close(0);                   /* close stdin */ 
        new_fd = dup(pipe_fd[0]);   /* duplicate file descriptor on stdin */
        if (new_fd == -1) {
          if (errno == EMFILE)
            fprintf(stderr, "Error: Too many open file descriptors.\n");
          fprintf(stderr, "Error: Could not uncompress file %s\n", filename);
          exit(0);
        }
        stream = fdopen(pipe_fd[0], "r");
      }

      /* Create a file id for writing uncompressed data to the pipe */

      else {
      }

      /* Store the child process id together with the file descriptor */

      if (num_running_gunzips == num_malloced_ids) {
        num_malloced_ids += 5;
        open_compressed_file_id =
          (FILE **) realloc(open_compressed_file_id,
                            num_malloced_ids * sizeof(FILE *));
        running_gunzip_id =
          (pid_t *) realloc(running_gunzip_id,
                            num_malloced_ids * sizeof(pid_t));
      }
      open_compressed_file_id[num_running_gunzips] = stream;
      running_gunzip_id[num_running_gunzips] = childpid;
      num_running_gunzips++;
      fprintf(stderr, "Created child process %d\n", childpid);

      /* Return the file descriptor */

      return(stream);
    }
  }
}
#endif
/*
--------------------------------------------------------------------------------
                 Close a stream attached to a compressed file
--------------------------------------------------------------------------------
*/

int Close_Compressed_File(FILE *fd)
{
  int status;

  status = pclose(fd);
  switch (status) {
    case -1:  return(fclose(fd));

    case 127: fprintf(stderr,
                      "Error:(De)Compression command could not be executed.\n");
              return(1);
    default:  return(0);
  }
}

#ifndef windows
int Close_Compressed_File_Old(FILE *fd)
{
  int i, j, success;

/* Look the file id up in the list of compressed file id's */

  for (i=0; i<num_running_gunzips; i++) {

/* If it is found, close the file id, kill the associated child process and
 * remove the id's from the list.
 */

    if (fd == open_compressed_file_id[i]) {
      success = fclose(fd);
      fprintf(stderr, "Pipe closed with status %d\n", success);
      success = kill(running_gunzip_id[i], SIGKILL);
      fprintf(stderr, "Killed process %d with status %d\n", 
              running_gunzip_id[i], success);
      for (j=i; j<num_running_gunzips-1; j++) {
        open_compressed_file_id[j] = open_compressed_file_id[j+1];
        running_gunzip_id[j] = running_gunzip_id[j+1];
      }
      num_running_gunzips--;
      return(success);
    }
  }

/* Otherwise, the file was uncompressed and therefore was not stored
 * in the list. Simply close the file.
 */

  return(fclose(fd));
}
#endif

/*
--------------------------------------------------------------------------------
                      Decompress a file to a file
--------------------------------------------------------------------------------
*/

int Decompress_File(char *zipped_file_in, char *unzipped_file)
{
  char *command, *new_file, *zipped_file;
  FILE *fd;
  int  compression_type;

/* Get the file name of the compressed file */

  zipped_file = Name_Compressed_File(zipped_file_in);

/* Check if file is compressed */

  compression_type = Compression_Type(zipped_file);
  if (compression_type == COMPRESSION_NONE) {
    fprintf(stderr, "File %s is not compressed.\n", zipped_file);
    return(0);
  }

#ifdef windows
/*
  fprintf(stderr, "File compression is not implemented for Windows.\n");
*/
  return 0;

#else
/* Compose the command */

  command = (char *) malloc((20 + strlen(zipped_file) + strlen(unzipped_file))
			    * sizeof(char));
  if (command == NULL) {
    printf("Error allocating space for command string.\n");
    return(0);
  }
  if (compression_type == COMPRESSION_GNU) strcpy(command, "gunzip ");
  else if (compression_type == COMPRESSION_LZW) strcpy(command, "uncompress ");
  else return(0);
  if (unzipped_file) strcat(command, "-c ");
  strcat(command, "-c ");
  strcat(command, zipped_file);
  if (unzipped_file) {
    strcat(command, " > ");
    strcat(command, unzipped_file);
  }

/* Uncompress the file */

  system(command);

/* Try to open the uncompressed file */

  if (unzipped_file) new_file = (char *) unzipped_file;
  else {
    new_file = (char *) malloc(sizeof(zipped_file) * sizeof(char));
    strcpy(new_file, zipped_file);
    if (compression_type == COMPRESSION_GNU) 
      new_file[strlen(new_file) - 3] = 0;
    else if (compression_type == COMPRESSION_LZW) 
      new_file[strlen(new_file) - 2] = 0;
  }
  if ((fd = fopen(new_file, "r")) == NULL) {
      printf("Could not open decompressed file %s\n", new_file);
      free(command);  free(new_file);
      return(0);
  }

/* Return success */

  fclose(fd);
  free(command);  free(new_file);
  return(1);
#endif
}

/*
--------------------------------------------------------------------------------
                      Compress a file to a file
--------------------------------------------------------------------------------
*/

int Compress_File(const char *unzipped_file, const char *zipped_file,
		          int compression_type)
{
  char *command, *new_file;
  int  len;
  FILE *fd;

/* Check if file is compressed */

  if (Compression_Type(unzipped_file) != COMPRESSION_NONE) {
    fprintf(stderr, "File %s is already compressed.\n", unzipped_file);
    return(0);
  }

#ifdef windows
/*
  fprintf(stderr, "File compression is not implemented for Windows.\n");
*/
  return 0;

#else
/* Compose the command */

  len = 20 + strlen(unzipped_file);
  if (zipped_file) len += strlen(zipped_file);
  command = (char *) malloc(len * sizeof(char));
  if (command == NULL) {
    printf("Error allocating space for command string.\n");
    return(0);
  }
  if (compression_type == COMPRESSION_GNU) strcpy(command, "gzip ");
  else if (compression_type == COMPRESSION_LZW) strcpy(command, "compress ");
  else {
    fprintf(stderr, "Unrecognised compression type (%d).\n",
	    compression_type);
    return(0);
  }
  if (zipped_file) strcat(command, "-c ");
  else strcat(command, "-f ");
  strcat(command, unzipped_file);
  if (zipped_file) {
    strcat(command, " > ");
    strcat(command, zipped_file);
  }

/* Compress the file */

  system(command);

/* Try to open it */

  if (zipped_file) new_file = (char *) zipped_file;
  else {
    new_file = (char *) malloc((strlen(unzipped_file) + 4) * sizeof(char));
    strcpy(new_file, unzipped_file);
    if (compression_type == COMPRESSION_GNU) strcat(new_file, ".gz");
    else if (compression_type == COMPRESSION_LZW) strcat(new_file, ".Z");
  }
  if ((fd = fopen(new_file, "r")) == NULL) {
    printf("Could not open compressed file %s\n", new_file);
    free(command);  free(new_file);
    return(0);
  }

/* Return success */

  fclose(fd);
  free(command);  free(new_file);
  return(1);
#endif
}
