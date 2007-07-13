# 
#  Copyright CNRS-LAAS Florent Lamiraux
# 

TESTS = \
	testFlicMaxDeriv 

noinst_PROGRAMS = \
	testFlicMaxDeriv \
	testFlicBoundObj \
	testFlicStraightLine

license: clean-license \
	testFlicMaxDeriv.kab \
	testFlicBoundObj.kab \
	testFlicStraightLine.kab

clean-license: 
	rm -rf bintestFlicMaxDeriv
	rm -rf bintestFlicBoundObj
	rm -rf bintestFlicStraightLine
#
#   Test method maxAbsoluteDerivative of CflicDirectPath
#

testFlicMaxDeriv_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicDirectPathMaxAbsDeriv.cpp

testFlicMaxDeriv_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) $(FLATCARDIRECTPATH_CFLAGS)
testFlicMaxDeriv_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) $(FLATCARDIRECTPATH_LIBS)


#
#  Plot bounding objects of CflicDirectPath
#

testFlicBoundObj_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicDirectPathBoundingObjects.cpp

testFlicBoundObj_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) $(FLATCARDIRECTPATH_CFLAGS)
testFlicBoundObj_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) $(FLATCARDIRECTPATH_LIBS)


#
#  Test a direct path between two aligned configurations
#
testFlicStraightLine_SOURCES = \
	testFlicDirectPath.cpp \
	testFlicStraightLine.cpp

testFlicStraightLine_CPPFLAGS = -I$(srcdir)/../include $(KWSIO_CFLAGS) $(FLATCARDIRECTPATH_CFLAGS)
testFlicStraightLine_LDFLAGS = ../src/.libs/libkwsPlus.a $(KWSIO_LIBS) $(FLATCARDIRECTPATH_LIBS)

%.kab: %
	mkdir -p bin$<
	cp $< bin$<
	cd bin$<; LD_LIBRARY_PATH=@KINEO_LIBDIR@  @KINEO_ADDON_BUILD@ -a ./$< 
	rm -f $<

