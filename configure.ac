dnl autoconf script for kwsPlus libs
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([kwsPlus],[1.7.99],[hpp@laas.fr])
AC_PREREQ(2.59)

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

if test "$CFLAGS$CXXFLAGS" = ""; then
  setdefaultflags=yes
else
  setdefaultflags=no
fi

AM_INIT_AUTOMAKE
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_PROG_CXX

if test "$setdefaultflags" = "yes"; then
  if test "$GCC" = "yes"; then
    CFLAGS="$CFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c flags])
  fi
  if test "$GXX" = "yes"; then
    CXXFLAGS="$CXXFLAGS -Wall"
    AC_MSG_NOTICE([adding default "-Wall" c++ flags])
  fi
fi

define(kwsio_reqd, [kwsIO >= 1.0.1])
AC_SUBST(KWSIO_REQD, "kwsio_reqd")
PKG_CHECK_MODULES(KWSIO, kwsIO)
KWSIO_PREFIX=`$PKG_CONFIG kwsIO --variable=prefix`
AC_SUBST(KWSIO_CFLAGS)
AC_SUBST(KWSIO_LIBS)
AC_SUBST(KWSIO_PREFIX)

dnl --- kineo  --------------

PKG_CHECK_MODULES(KINEOCONTROLLER, KineoController)
AC_SUBST(KINEOCONTROLLER_CFLAGS)
AC_SUBST(KINEOCONTROLLER_LIBS)

KINEOCONTROLLER_PREFIX=`$PKG_CONFIG KineoController --variable=prefix`
AC_SUBST(KINEOCONTROLLER_PREFIX)

AC_SUBST(KINEO_LIBDIR,[${KINEOCONTROLLER_PREFIX}/lib])
AC_SUBST(KINEO_ADDON_BUILD,[${KINEOCONTROLLER_PREFIX}/bin/KineoAddonBuilder])

dnl --- Search for Libtool -
AC_CONFIG_MACRO_DIR([build-aux])
AC_PROG_LIBTOOL

dnl --- Boost --------------

BOOST_UBLAS


dnl lapack path
AC_ARG_WITH(lapack, AC_HELP_STRING([--with-lapack],
	[prefix of lapack installation]),,with_lapack=no)
if test x${with_lapack} != xno ; then
   if test x${with_lapack} != xyes; then
	LAPACK_CFLAGS="-I${with_lapack}/include"
	CXXFLAGS="${CXXFLAGS} ${LAPACK_CFLAGS}"
	LAPACK_LIBS="-Wl,-R${with_lapack} -L${with_lapack}/lib -llapack"
   else
	LAPACK_LIBS="-llapack"
   fi
   else
   AC_MSG_ERROR([please configure --with-lapack(=PATH_TO_LAPACK)])
fi

AC_SUBST(LAPACK_CFLAGS)
AC_SUBST(LAPACK_LIBS)

dnl ---If unitTesting enables, make dependence to testKineoWorks
AC_ARG_ENABLE(unitTesting, AC_HELP_STRING([--enable-unitTesting],
	[Implement unitTesting. In this case, a dependence to testKineoWorks is added]),,[enable_unitTesting=no])

AC_SUBST(TESTKINEOWORKS_REQD)
AC_SUBST(TESTKINEOWORKS_CFLAGS)
AC_SUBST(TESTKINEOWORKS_LIBS)
AC_SUBST(TESTKINEOWORKS_TAGFILE, [])
AC_SUBST(TESTKINEOWORKS_PREFIX)

if test x${enable_unitTesting} = xyes; then
	UNITTESTING_CFLAGS="UNITTESTING=1"
	PKG_CHECK_MODULES(TESTKINEOWORKS, [testKineoWorks >= 1.0])
	AC_SUBST(TESTKINEOWORKS_CFLAGS)
	AC_SUBST(TESTKINEOWORKS_LIBS)
	TESTKINEOWORKS_PREFIX=`$PKG_CONFIG --variable=prefix testKineoWorks`
	TESTKINEOWORKS_TAGFILE="$TESTKINEOWORKS_PREFIX/share/doc/doxytag/testKineoWorks.doxytag=$TESTKINEOWORKS_PREFIX/share/doc/testKineoWorks"
	TESTKINEOWORKS_REQD=", testKineoWorks >= 1.0"
	AC_MSG_NOTICE([Implement direct path unit testing])
else
	UNITTESTING_CFLAGS="UNITTESTING=0"
	AC_MSG_NOTICE([Direct path unit testing not implemented])
fi

AM_CONDITIONAL(UNITTESTING, test x${enable_unitTesting} = xyes)

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	unitTesting/Makefile
	doc/kwsPlus.dox
	kwsPlus.pc
)
