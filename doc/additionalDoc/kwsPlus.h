/**
   \mainpage
   \section intro Introduction

This package includes general extensions to KineoWorks software. These extensions are divided into modules.

\defgroup kwsPlus_DP Direct Paths and Steering Methods
  @{
*/

/**
  \defgroup RS Reeds and Shepp local steering method

  \defgroup flic Flatness-based local steering method for cart-like robot

*/

/**
  @}
*/

/**
   \defgroup kwsPlus_rdm Roadmaps and roadmap builders
   @{
*/

/**

\defgroup visi Visibility roadmap builder

This module implements a version of the Visibility roadmap builder.

The goal is to allow the user to use a visibility roadmap builder.
The visibilityRdmBuilder will use a hppShooterActiveDof to shoot the
configuration. Thus when the roadmapBuilder needs a new configuration,
a new configuration will be created using the ChppConfigurationShooter
of the attribute of the hppShooterActiveDof, latter the configuration
will be modified using the hppActiveDofSetters that the
ChppShooterActiveDof has.

\image html diagrammeVisibility.png

The VisibilityRdmBuilder inherits from the CkwsRoadmapBuilder. To use
the RLG we have to change the configuration after it has been
shoot. So, the VisilibityRdmBuilder uses an ChppActiveDofShooter. It
will have an CgooConfigurationShooter that will generate a random
configuration of the configuration of the device. If we whan to change
the way the configurations are shooted we have to chage this
class. Latter, the configuration shooted will be used by all the
ActiveDofSetters that will change the values of the actives
joints. The ChppActiveDofShooter have a list of ChppActiveDofSetter
that will change the values of the atives joints of the configurations
after it has been shooted by the ChppConfigurationShooter.

The DifussionShooterActiveDofSetter inherits from CkwsDiffusionShooter
and from ChppActiveDofShooter. So it could be use to do a diffussing
roadmap with RLG.

\defgroup PCA PCA Roadmap Builder


*/

/**
   \defgroup kwsPlus_graphic_rdm Displaying roadmaps

   This module provides tools to display a roadmap in KPP-interface.

*/

/**
@}
*/

/**
   \defgroup kwsPlus_misc Miscellanous tools

*/
