# Path to your FreeBSD source tree (where you see sys/, include/, etc.)

# Name of the module (this will produce if_e1000pol.ko)
SYSDIR = /usr/src/sys
 
KMOD   = uart_test

# Your driver source file(s)
#SRCS   = e1000pol.c device_if.h bus_if.h pci_if.h opt_platform.h ofw_bus_if.h
SRCS   = uart.c

# If you need additional include paths:
.include <bsd.kmod.mk>
