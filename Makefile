#############################################################################
#
# Makefile for librf24 examples on Linux
#
# License: GPL (General Public License)
# Author:  gnulnulf <arco@appeltaart.mine.nu>
# Date:    2013/02/07 (version 1.0)
#
# Description:
# ------------
# use make all and make install to install the examples
#

DRIVER=SPIDEV
CPUFLAGS=-march=armv7-a -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard
CFLAGS=-march=armv7-a -g -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -Ofast -Wall -pthread 
PREFIX=/usr/local
LIB=rf24
LIBNAME=librf24.so.1.1.7
LIB_VERSION=1.1.7
LIBSYMLINKS=librf24.so.1.1 librf24.so.1 librf24.so
LIBDEPRECATE=librf24-bcm.so
HEADER_DIR=/usr/local/include
LIB_DIR=/usr/local/lib
CC=arm-linux-gnueabihf-gcc
CXX=arm-linux-gnueabihf-g++

SHARED_LINKER_FLAGS= -pthread -shared -Wl,-soname,librf24.so.1
LDCONFIG=ldconfig

LIBS=-l$(LIB)

PROGRAMS := controller
SOURCES := main.cpp cflie.cpp
HEADERS := cflie.h cflie_packets.h

all: $(PROGRAMS)

built_source: $(SOURCES)
	$(CXX) $(CFLAGS) -I$(HEADER_DIR) -L$(LIB_DIR) $^  $(LIBS) -o $@


$(PROGRAMS): $(HEADERS) built_source
	mv built_source $@

clean:
	@echo "[Cleaning]"
	rm -rf $(PROGRAMS)

