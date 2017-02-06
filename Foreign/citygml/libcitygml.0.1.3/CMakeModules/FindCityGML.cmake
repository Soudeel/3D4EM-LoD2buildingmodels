# Locate libcitygml
# This module defines
# CITYGML_LIBRARY
# CITYGML_FOUND, if false, do not try to link to CITYGML 
# CITYGML_INCLUDE_DIR, where to find the headers
#
# $CITYGML_DIR is an environment variable that would
# correspond to the ./configure --prefix=$CITYGML_DIR


FIND_PATH( CITYGML_INCLUDE_DIR citygml.h
	./include
	../include
	$ENV{CITYGML_DIR}/include
	~/Library/Frameworks
	/Library/Frameworks
	/usr/local/include
	/usr/include
	/sw/include # Fink
	/opt/local/include # DarwinPorts
	/opt/csw/include # Blastwave
	/opt/include
	/usr/freeware/include
)

FIND_LIBRARY( CITYGML_LIBRARY 
	NAMES citygml
	PATHS
	./lib
	../lib
	$ENV{CITYGML_DIR}/lib
	$ENV{CITYGML_DIR}
	~/Library/Frameworks
	/Library/Frameworks
	/usr/local/lib
	/usr/lib
	/sw/lib
	/opt/local/lib
	/opt/csw/lib
	/opt/lib
	/usr/freeware/lib64
)

SET( CITYGML_FOUND "NO" )

IF(CITYGML_LIBRARY AND CITYGML_INCLUDE_DIR)
	SET(CITYGML_FOUND "YES")
ENDIF(CITYGML_LIBRARY AND CITYGML_INCLUDE_DIR)


