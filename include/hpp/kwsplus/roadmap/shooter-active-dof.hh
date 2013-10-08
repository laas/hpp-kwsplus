/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)
*/

#ifndef HPP_SHOOTERACTIVEDOF_H
#define HPP_SHOOTERACTIVEDOF_H

/*************************************
INCLUDE
**************************************/

#include <list>

#include <hpp/kwsplus/roadmap/configuration-shooter.hh>
#include <hpp/kwsplus/roadmap/active-dof-setter.hh>

HPP_KIT_PREDEF_CLASS(ChppShooterActiveDof);

/**
   \addtogroup visi
   @{
*/

/**
 \brief This class have:
    \li a Configurationshooter
    \li a list of ChppActiveDofSetters:

The goal of this class is to shoot a configuration allowing the construction of Roadmaps with the VisibilityRdmBuilder. It allows to
shoot a configuration but having the capacity of modify the values of the actives dofs.
Thus, if we use a ChppShooterActiveDof as shooter, when the roadmapBuilder needs a new configuration, a
new configuration will be created using the ChppConfigurationShooter of the attribute and latter the configuration will
be passed throw all the ChppActiveDofSetters that the ChppShooterActiveDof has.

The ChppActiveDofSetters will be called in the order they are added to the ChppDiffusionShooterActiveDofSetter.

\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*/

class ChppShooterActiveDof {

public:

  /**
     \brief Constructor.
     \param i_confshooter Configuration shooter used to create the new configuration.
  */
  static ChppShooterActiveDofShPtr create(const ChppConfigurationShooterShPtr& i_confshooter=ChppConfigurationShooter::create());

  /**
     \brief Destructor.
  */
  virtual ~ChppShooterActiveDof() {};

  /**
     \brief Function to shoot a new configuration. This configuration will be shooted used the ConfigurationShooter and
latter will be passed throw all the ActiveDofSetters to allow them to change the values of the configuration. The order
of call to the ActiveDofSetters will be the order of addition of them to the ChppShooterActiveDof.
    \param i_node The input node to shoot the new configuration
    \param o_cfg The output configuration shooted
    \return true if the confituration has been shooted. False otherwise.
  */
  virtual bool shoot(const CkwsNodeShPtr& i_node,
                         CkwsConfig& o_cfg);

  /**
     \brief allows to add a ChppActiveDofSetter to the ChppShooterActiveDof.
     \param actdofsetter ActiveDofSetter that have to be added to the ChppShooterActiveDof.
   */
  void addActDofSetter(const ChppActiveDofSetterShPtr& actdofsetter);


protected:

  /**
     \brief Initialization.
  */
  ktStatus init(ChppConfigurationShooterShPtr i_shooter);

private:

  /**
     \brief The Shooter used to create the configuration.
  */
  ChppConfigurationShooterShPtr attConfigurationShooter;

  /**
     \brief List of ChppActiveDofSetters.
  */
  std::list<ChppActiveDofSetterShPtr> attListActDofSetters;

};

/**
   @}
*/

#endif /*HPP_SHOOTERACTIVEDOF_H*/
