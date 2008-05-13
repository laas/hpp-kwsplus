dnl autoconf script for kwsPlus libs
dnl
dnl   Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([kwsPlus],[1.4.99],[hpp@laas.fr])
AC_PREREQ(2.59)

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

dnl --- Boost --------------
AC_ARG_WITH(boost,
	AS_HELP_STRING([--with-boost], [boost installation prefix]),
	BOOST_CFLAGS="-I${withval}/include"
	CXXFLAGS="${CXXFLAGS} ${BOOST_CFLAGS}",
	)

AC_SUBST(BOOST_CFLAGS)


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

AC_OUTPUT(Makefile
	src/Makefile
	include/Makefile
	doc/Makefile
	doc/kwsPlus.dox
	kwsPlus.pc
 	unitTesting/Makefile
)
