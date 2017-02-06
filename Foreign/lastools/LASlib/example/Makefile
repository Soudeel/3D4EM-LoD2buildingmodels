# makefile for lasexample
#
#COPTS    = -g -Wall
COPTS     = -O3 -Wall
COMPILER  = g++
LINKER  = g++
#BITS     = -64

#LIBS     = -L/usr/lib32
#LIBS     = -L/usr/lib64
INCLUDE  = -I/usr/include

LASLIBS     = -L../lib
LASINCLUDE  = -I../../LASzip/src -I../inc

all: lasexample

lasexample: lasexample.o
	${LINKER} ${BITS} ${COPTS} lasexample.o -llas -o $@ ${LIBS} ${LASLIBS} $(INCLUDE) $(LASINCLUDE)

.cpp.o: 
	${COMPILER} ${BITS} -c ${COPTS} ${INCLUDE} $(LASINCLUDE) $< -o $@

.c.o: 
	${COMPILER} ${BITS} -c ${COPTS} ${INCLUDE} $(LASINCLUDE) $< -o $@

clean:
	rm -rf *.o
	rm -rf lasexample

clobber:
	rm -rf *.o
	rm -rf lasexample
	rm -rf *~
