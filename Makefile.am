# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

SUBDIRS = \
	src 	\
	doc	\
	include 

if UNITTESTING
SUBDIRS += \
	unitTesting
endif

kwsPlusdir = $(libdir)/pkgconfig
kwsPlus_DATA = kwsPlus.pc

install-data-hook: documentation

documentation:
	cd doc; $(MAKE) install
