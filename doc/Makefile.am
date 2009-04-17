# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

kwsPlusdir = $(datadir)/doc/kwsPlus/html

EXTRA_DIST = \
	footer.html \
	header.html \
	kwsPlus.css \
	additionalDoc/kwsPlus.h \
	figures/UtoSandUtoS.png \
	figures/boundsD2gammaOverDs2.png \
	figures/boundsD2gammaOverDu2.png \
	figures/boundsDgammaOverDs.png \
	figures/boundsDgammaOverDu.png \
	figures/combinaison-convexe.png \
	figures/equation-combinaison-convexe.png \
	figures/flatCar.fig \
	figures/flatCar.pdf \
	figures/flatCar.png	\
	figures/pca.png \
	figures/pcaDiff.png \
	figures/diagrammeVisibility.png \
	figures/LocalTreesProblem.png \
	papers/rapport-mathieu.pdf \
	papers/rapport-sebastien.pdf 

install: html-doc
	mkdir -p $(kwsPlusdir)
	$(INSTALL_DATA) html/* $(kwsPlusdir)
	mkdir -p $(kwsPlusdir)/papers
	$(INSTALL_DATA) $(srcdir)/papers/* $(kwsPlusdir)/papers
	$(INSTALL_DATA) kwsPlus.doxytag $(kwsPlusdir)

html-doc: kwsPlus.dox
	doxygen $<
