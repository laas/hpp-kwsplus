/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)
*/

#ifndef HPP_ACTIVEDOFSETTER_H
#define HPP_ACTIVEDOFSETTER_H

/*************************************
INCLUDE
**************************************/
# include "KineoUtility/kitDefine.h"
# include "KineoWorks2/kwsConfig.h"

KIT_PREDEF_CLASS(ChppActiveDofSetter);

/**
   \addtogroup visi
   @{
*/

/**
 \brief This is an abstract class that works as an interface. 
It's goal is to allow the definition of ActiveDofSetters that modify the values of the active dofs of a configuration.

\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm 
*/

class ChppActiveDofSetter {

public:
  virtual ~ChppActiveDofSetter() {};
 /**
     \brief The only function of ChppActiveDofSetter is the abstract funcion that modify the values of the actives dofs.
     \param io_cfg Configuration to be modified.
     \return bool
 */
  virtual bool setActiveDof(CkwsConfig& io_cfg)=0;

};

/**
   @}
*/

#endif
