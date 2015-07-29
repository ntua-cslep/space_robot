Linux Software (DM6814)

SWP-700010036 rev D

Version 02.02.00.87650

Copyright (C), RTD Embedded Technologies, Inc. All Rights Reserved.

Using this software indicates acceptance of the RTD End-User Software 
License Agreement.  See the file LICENSE.TXT.



------------------
Supported Hardware
------------------

This software supports the RTD DM5814 and DM6814 dataModules.



-------------------------
Supported Kernel Versions
-------------------------

This software has been tested with the following Linux distributions and kernel
versions:

    * openSUSE 13.1 (unmodified kernel)


Note that due to kernel API differences between kernel versions, RTD cannot
guarantee cmopatibility with kernels and distributions not listed above.  It may
be necessary to modify the kernel code and/or Makefiles for your specific Linux 
environment.



-------------------
Supported Compilers
-------------------

The driver software was compiled using the GNU gcc compiler under openSUSE 13.1.

The library and example programs were compiled using the GNU g++ compiler under
openSUSE 13.1.



------
Driver
------

The directory driver/ contains source code related to the driver.


In order to use a driver, one must first compile it, load it into the kernel,
and create device files for the board(s).  To do this, issue the following
commands while sitting in the driver/ directory:

    * make
    * make insmod
	NOTE:	This target assumes that 1) a single DM6814 is installed,
		2) its base address is set to the factory default of 0x300, and
		3) the board uses IRQ 5.  You may need to edit the Makefile and
		change this rule to reflect your board configuration or
		manually issue an appropriate insmod command.
    * make devices


The driver make description file driver/Makefile has been written in such a way
that the above commands work for compiling both on a 2.4 kernel and on a 2.6
kernel.


For the 2.4 kernel, the driver object file is named rtd-dm6814.o.  When you
load the kernel driver, the message "Warning: loading rtd-dm6814.o will taint
the kernel: non-GPL license - Proprietary" will be printed.  You can safely
ignore this message since it pertains to GPL licensing issues rather than to
driver operation.


For the 2.6 kernel, the driver object file is named rtd-dm6814.ko.  When you
load the kernel driver, no warnings will appear on your screen.  However, the
warning "module license 'Proprietary' taints kernel." will be written to the
system log when the module is loaded.  You can safely ignore this message since
it pertains to GPL licensing issues rather than to driver operation.


The following arguments to insmod are accepted when loading the driver:

    * io
	List of 4 base I/O addresses.  An example of using this argument is
	"io=0x300,0x340" which indicates that 2 boards are present at base
	addresses of 0x300 and 0x340 respectively.  Note that multiple
        addresses must be separated by commas.  The default base I/O address is
	zero; a value of zero means either no device is present or it is
	present but should be ignored.  When the address is zero, no resources
	are allocated for the device.
    * irq
	List of 4 IRQ line numbers.  An example of using this argument is
	"irq=5,6,7" which indicates that 3 boards are present using IRQ
	numbers of 5, 6 and 7 respectively.  Note that multiple IRQ numbers
	must be separated by commas.  The default IRQ line number is zero; a
	value of zero means that interrupt functionality will not be used.
	When the IRQ is zero, no interrupt resources are allocated for the
	device.



-----------------
Library Interface
-----------------

The directory lib/ contains source code related to the user library.


The DM6814 library is created with a file name of librtd-dm6814.a and is a
statically linked library.  Since the library makes use of C++ language
facilities, it is STRONGLY SUGGESTED to use the C++ compiler to compile any
source code which will make use of library functions.


Please refer to the software manual for details on using the user level
library functions.  These functions are prototyped in the file
include/dm6814_library.h; this header file must be included in any code which
wishes to call library functions.


Because of changes made in version 2.0.0 of the driver, existing source code
which uses the library will not compile.  Users upgrading from a driver version
prior to 2.0.0 will need to modify source code to reflect the changes.


To build the library, issue the command "make" within lib/.



----------------
Example Programs
----------------

The directory examples/ contains source code related to the example programs,
which demonstrate how to use features of the DM6814 boards or test the driver
and/or library.


The following example programs are provided:

    * basic-test
	Program which tests the basic functionality of the driver and library.
    * digital-io
	Example program which demonstrates reading from and writing to the
	digital I/O ports.
    * dio-test
	Program which tests the library functions related to digital I/O.
    * encoder-int
	Example program which demonstrates how to use incremental encoder
	interrupts.
    * encoders
	Example program which demonstrates reading the incremental encoders.
    * external-int
	Example program which demonstrates how to use P14 external interrupts.
    * interrupt-test
	Program which tests the library interrupt-related functions.
    * int-wait-via-read
	Example program which demonstrates waiting for interrupt notification
	via blocking I/O and the read(2) system call.
    * int-wait-via-select
	Example program which demonstrates waiting for interrupt notification
	via the select(2) system call.
    * timer-int
	Example program which demonstrates how to use P14 timer interrupts.
    * timers
	Example program which demonstrates using the 8254 timer/counters in
	rate generator and event count modes.
    * timer-test
	Program which tests the basic functionality of the 8254 timer/counter
	library functions.


To build the example programs, issue the command "make" within examples/.



------------
Header Files
------------

The directory include/ contains all header files needed by the driver, library,
example programs, and user applications.



---------------
IOCTL Interface
---------------

Please refer to the software manual for details on how to communicate with the
driver using the ioctl() system call.



-----------------
Known Limitations
-----------------

 1. The major argument on the insmod command is no longer supported by the
    driver.  Instead, the driver now asks the kernel to dynamically assign a
    character major number when the kernel module is loaded.

 2. The debug argument on the insmod command is no longer supported by the
    driver.

 3. The force argument on the insmod command is no longer supported by the
    driver.

 4. Interrupt notification via signals is no longer supported.  Only a handful
    of signals exist which are not normally used by most Linux programs.  If
    several drivers that use these signals are loaded, the likelihood of
    interference between them is high.  Instead, the driver now allows a
    process to wait in the kernel until an interrupt occurs.  The device file
    provides access to this feature, thus greatly reducing the possibility of
    two drivers getting in the way of each other.

 5. Many conditions affect performance of user-level interrupt notification.
    These include system activity, process scheduling priority, page faults,
    and other interrupts.  Acceptable performance at high interrupt rates
    likely requires going beyond mere waiting for an interrupt to occur.
    Please see the software manual for details.

 6. The driver now acknowledges all interrupts in the interrupt handler.  The
    process of acknowledging interrupts clears the associated interrupt status
    bits.  Therefore, direct board status is no longer available to an
    application.  However, the driver caches the IRQ Status Register value
    which applications can obtain.

 7. The driver is not designed to process interrupts shared between devices.
    To avoid unpredictable behavior or worse, do not share an interrupt between
    devices.

 8. If you are using the interrupt wait mechanism, be aware that signals
    delivered to the application can cause the sleep to awaken prematurely.
    Interrupts may be missed if signals are delivered rapidly enough or at
    inopportune times.

 9. Be aware that GetIntStatus6814() disables interrupts on the associated
    DM6814 device for a very brief time to obtain accurate status information.
    If you call the function repeatedly in a loop (such as when busy-waiting
    for an interrupt to occur), this can interfere with DM6814 interrupts.  It
    is strongly suggested that you do not busy-wait for interrupts.



-------------------------
Getting Technical Support
-------------------------

If you require additional support with this product, or any other products
from RTD Embedded Technologies, contact us using the information below:

	RTD Embedded Technologies, Inc.
	103 Innovation Boulevard
	State College, PA 16803 USA

	Telephone: (814) 234-8087
	Fax: (814) 234-5218
	Sales Information and Quotes: sales@rtd.com
	Technical Assistance: techsupport@rtd.com
	Web Site: http://www.rtd.com
