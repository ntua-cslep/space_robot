#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building driver
#
#	PROJECT NAME: Linux DM6814 Driver, Library, and Example Programs
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	Copyright 2005 RTD Embedded Technologies, Inc.  All Rights Reserved.
#


#
# $(src) points to the relative path where this Makefile exists.  This must be
# used instead of getting the current working directory via pwd when telling
# the preprocessor where to find the driver header files.
#

EXTRA_CFLAGS := -I$(src)/../include

#
# Kernel build environment directory.  Supposedly it is safer to use this
# method of referring to it than using /usr/src.
#

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := rtd-dm6814.o

DRIVER_MODULE=rtd-dm6814.ko
FILES_TO_CLEAN= \
	*.o \
	rtd-dm6814.ko \
	.*.cmd \
	*.ko \
	.tmp_versions \
	rtd-dm6814.mod.c \
	Module.symvers \
	Module.markers \
	modules.order \
	*~

driver:		rtd-dm6814.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` modules


clean:
	rm -rf $(FILES_TO_CLEAN)

devices:
	@echo "Removing old device files ..."; \
	rm -f /dev/rtd-dm6814-[0-3] \
	echo ""; \
	echo -n "Obtaining device major number ... "; \
	major=`awk "\\$$2==\"rtd-dm6814\" {print \\$$1}" /proc/devices`; \
	if [ -z "$${major}" ]; \
	then \
	    echo ""; \
	    echo "FAILED: The kernel driver doesn't seem to be loaded."; \
	    echo ""; \
	else \
	    echo "$${major}"; \
	    echo ""; \
	    echo "Creating device files ..."; \
	    minor=0; \
	    while [ $${minor} -lt 4 ]; \
	    do \
		echo "   /dev/rtd-dm6814-$${minor}"; \
		mknod --mode=0666 /dev/rtd-dm6814-$${minor} c $${major} $${minor}; \
		minor=`expr $${minor} + 1`; \
	    done; \
	fi;

insmod:
	insmod ./$(DRIVER_MODULE) io=0x300,0x200 irq=5,0

rmmod:
	rmmod rtd-dm6814
