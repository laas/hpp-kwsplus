# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

lib_LTLIBRARIES = libkwsPlus.la
libkwsPlus_la_SOURCES = \
	flicDirectPath.cpp \
	flicSteeringMethod.cpp \
	kwsPlusDirectPath.cpp \
	directPathVector.cpp \
	reedsSheppDirectPath.cpp \
	reedsSheppSteeringMethod.cpp \
	kitMat3.cpp \
	hppConfigurationShooter.cpp \
	hppShooterActiveDof.cpp \
	hppVisRdmBuilder.cpp \
	kwsPlusDPLinear.cpp \
	kwsPlusSMLinear.cpp \
	kwsPlusRoadmap.cpp


libkwsPlus_la_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS)  $(LAPACK_CFLAGS)
libkwsPlus_la_LDFLAGS =\
	$(KWSIO_LIBS) 	\
	$(LAPACK_LIBS)  \
	-release ${PACKAGE_VERSION}

