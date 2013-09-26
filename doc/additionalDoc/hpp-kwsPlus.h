/**
   \mainpage
   \section kwsPlus_intro Introduction

This package includes general extensions to KineoWorks software. 
Go to <a href="./modules.html">Modules</a> to see the organization of this package.

\defgroup kwsPlus_DP Direct Paths Steering Methods and Distances
  @{
*/

/**
  \defgroup RS Reeds and Shepp local steering method

  \defgroup flic Flatness-based local steering method for cart-like robot

  \defgroup kwsplusdplinear Extension of linear steering method

   These classes implement an extension of KineoWorks linear direct
   path and steering method. The main extensions are described
   below.

   \li CkwsPlusDPLinear derives from CkwsPlusDirectPath and thus 
   inherits from all fonctionalities of this class.
   \li Direct paths of class CkwsPlusDPLinear enable users to overestimate 
   the variations of the degrees of freedom of a robot along intervals of 
   parameter. 

   In some applications, for instance when constraints impose relations 
   between some degrees of freedom, computing upper-bounds on the above 
   variations as requested by CkwsDirectPath::maxAbsoluteDerivative()
   is not an easy task. 

   To overcome this difficulty, classes of this group propose to over-estimate
   the variation of degrees-of-freedom by multiplying the result without 
   constraints as computed by CkwsDPLinear::maxAbsoluteDerivative() by ratios 
   stored in a vector passed at construction of steering method 
   CkwsPlusSMLinear.

  \defgroup smfactory Steering method factory

  \defgroup distanceFactory Distance function factory

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

\defgroup kwsPlusEnhancedRoadmapManagement Enhanced roadmap management

*/

/**
   \defgroup LT Local Trees Roadmap Builder

*/

/**
   \defgroup diffusionNodePickerFactory Diffusion node picker factory

   Diffusion node picker are objects defined in KineoWorks. They define
   the way a diffusing roadmap builder picks a node to extend in the current roadmap.
*/

/**
   \defgroup diffusionShooterFactory Diffusion shooter factory

   Diffusion shooter are objects defined in KineoWorks. They define
   the way a diffusing roadmap builder randomly shoots configurations.
*/

/**
@}
*/

/**
   \defgroup kwsPlus_misc Miscellanous tools

*/
