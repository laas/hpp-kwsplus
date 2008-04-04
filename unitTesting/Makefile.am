# 
#  Copyright CNRS-LAAS Florent Lamiraux
# 

TESTS = \
	testFlicMaxDeriv 

noinst_PROGRAMS = \
	testFlicMaxDeriv 	\
	testFlicBoundObj 	\
	testFlicStraightLine 	\
	testFlicApproximateDistance

license: clean-license 			\
	testFlicMaxDeriv.kab 		\
	testFlicBoundObj.kab 		\
	testFlicStraightLine.kab	\
	testFlicApproximateDistance.kab

clean-license: 
	rm -rf bintestFlicMaxDeriv
	rm -rf bintestFlicBoundObj
	rm -rf bintestFlicStraightLine
	rm -rf bintestFlicApproximateDistance
#
#   Test method maxAbsoluteDerivative of CflicDirectPath
#

testFlicMaxDeriv_SOURCES = \
	testFlicDirectPath.h \
	testFlicDirectPath.cpp \
	testFlicDirectPathMaxAbsDeriv.cpp

testFlicMaxDeriv_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) 
testFlicMaxDeriv_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) 


#
#  Plot bounding objects of CflicDirectPath
#

testFlicBoundObj_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicDirectPathBoundingObjects.cpp

testFlicBoundObj_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) 
testFlicBoundObj_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) 


#
#  Test a direct path between two aligned configurations
#
testFlicStraightLine_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicStraightLine.cpp

testFlicStraightLine_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) 
testFlicStraightLine_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) 

#
#  Test approximation of distance related to  flic steering method
#
testFlicApproximateDistance_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicApproximateDistance.cpp

testFlicApproximateDistance_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) 
testFlicApproximateDistance_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) 


# FIXME: this doesn't compile
#testProjectionConstraint_SOURCES = \
#	testProjectionConstraint.cpp

#testProjectionConstraint_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) 
#testProjectionConstraint_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) 

%.kab: %
	mkdir -p bin$<
	cp $< bin$<
	cd bin$<; LD_LIBRARY_PATH=@KINEO_LIBDIR@  @KINEO_ADDON_BUILD@ -a ./$< 
	rm -f $<
