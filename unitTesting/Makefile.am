# 
#  Copyright CNRS-LAAS Florent Lamiraux
# 

include $(top_srcdir)/build-aux/init.mk

if UNITTESTING
TESTS = \
	testFlicMaxDeriv 

noinst_PROGRAMS = \
	testFlicMaxDeriv 	\
	testFlicBoundObj 	\
	testFlicStraightLine 	\
	testFlicApproximateDistance \
	testFlicCopyAndExtract \
	testFlicDerivative

license: clean-license 			\
	testFlicMaxDeriv.kab 		\
	testFlicBoundObj.kab 		\
	testFlicStraightLine.kab	\
	testFlicApproximateDistance.kab \
	testFlicCopyAndExtract.kab \
	testFlicDerivative.kab

clean-license: 
	rm -rf bintestFlicMaxDeriv
	rm -rf bintestFlicBoundObj
	rm -rf bintestFlicStraightLine
	rm -rf bintestFlicApproximateDistance
	rm -rf bintestFlicCopyAndExtract
	rm -rf bintestFlicDerivative

#
#   Test method maxAbsoluteDerivative of CflicDirectPath
#

testFlicMaxDeriv_SOURCES = \
	testFlicDirectPath.h \
	testFlicDirectPath.cpp \
	testFlicDirectPathMaxAbsDeriv.cpp

testFlicMaxDeriv_CPPFLAGS = $(AM_CPPFLAGS) $(TESTKINEOWORKS_CFLAGS) 
testFlicMaxDeriv_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 


#
#  Plot bounding objects of CflicDirectPath
#

testFlicBoundObj_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicDirectPathBoundingObjects.cpp

testFlicBoundObj_CPPFLAGS = $(AM_CPPFLAGS) $(TESTKINEOWORKS_CFLAGS) 
testFlicBoundObj_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 


#
#  Test a direct path between two aligned configurations
#
testFlicStraightLine_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicStraightLine.cpp

testFlicStraightLine_CPPFLAGS = $(AM_CPPFLAGS)  $(TESTKINEOWORKS_CFLAGS) 
testFlicStraightLine_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 

#
#  Test approximation of distance related to  flic steering method
#
testFlicApproximateDistance_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicApproximateDistance.cpp

testFlicApproximateDistance_CPPFLAGS = $(AM_CPPFLAGS)  $(TESTKINEOWORKS_CFLAGS) 
testFlicApproximateDistance_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 

#
#  Test copy extraction and reversion of CflicDirectPath
#
testFlicCopyAndExtract_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicCopyAndExtract.cpp

testFlicCopyAndExtract_CPPFLAGS = $(AM_CPPFLAGS)  $(TESTKINEOWORKS_CFLAGS) 
testFlicCopyAndExtract_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 

#
#  Test derivative along CflicDirectPath
#

testFlicDerivative_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicDerivative.cpp

testFlicDerivative_CPPFLAGS = $(AM_CPPFLAGS)  $(TESTKINEOWORKS_CFLAGS) 
testFlicDerivative_LDFLAGS = ../src/.libs/libkwsPlus.a $(TESTKINEOWORKS_LIBS) 


%.kab:
	mkdir -p bin$*
	cp $* bin$*
	cd bin$*; LD_LIBRARY_PATH=@KINEO_LIBDIR@  @KINEO_ADDON_BUILD@ -a ./$*
	rm -f $*

endif
