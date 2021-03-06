/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)
*/

#ifndef HPP_CONFIGURATIONSHOOTER_H
#define HPP_CONFIGURATIONSHOOTER_H

/*************************************
INCLUDE
**************************************/

#include <list>
#include "KineoWorks2/kwsNode.h"

#include <hpp/util/kitelab.hh>

#include <hpp/kwsplus/roadmap/configuration-shooter.hh>
#include <hpp/kwsplus/roadmap/active-dof-setter.hh>

HPP_KIT_PREDEF_CLASS(ChppConfigurationShooter);

/**
   \addtogroup visi
   @{
*/

/**
 \brief

The goal of this class is to shoot a configuration.

\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*/

class ChppConfigurationShooter {

public:

  /**
     \brief Constructor.
 */
  static ChppConfigurationShooterShPtr create();

  /**
     \brief Destructor.
  */
  virtual ~ChppConfigurationShooter() {};

  /**
     \brief Function to shoot a new configuration.
    \param i_node The input node to shoot the new configuration
    \param o_cfg The output configuration shooted
  */
  virtual bool shoot(const CkwsNodeShPtr& i_node,
                         CkwsConfig& o_cfg);

protected:

  /**
     \brief Initialization.
  */
  ktStatus init();

};

/**
   @}
*/

#endif /*HPP_CONFIGURATIONSHOOTER_H*/
