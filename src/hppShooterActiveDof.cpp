/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)

*/


#include "hppShooterActiveDof.h"
#include "KineoWorks2/kwsNode.h"
#include <iostream>


// ==========================================================================

  ChppShooterActiveDofShPtr ChppShooterActiveDof::create(const ChppConfigurationShooterShPtr& i_confshooter)
  {
	ChppShooterActiveDof* ptr= new ChppShooterActiveDof();
	ChppShooterActiveDofShPtr shPtr(ptr);
	if(shPtr->init(i_confshooter)!=KD_OK)
	{
	  shPtr.reset();
	}
	return shPtr;
  }


// ==========================================================================

  bool ChppShooterActiveDof::shoot(const CkwsNodeShPtr& i_node,
                                                 CkwsConfig& o_cfg)
  {

    bool res = attConfigurationShooter->shoot(i_node,o_cfg);

    std::list<ChppActiveDofSetterShPtr>::iterator i;
    for(i=attListActDofSetters.begin();
        i != attListActDofSetters.end() && res;
        i++)
    {
      res=(*i)->setActiveDof(o_cfg);
    }
    return res;
  }


// ==========================================================================

  ktStatus ChppShooterActiveDof::init(ChppConfigurationShooterShPtr i_shooter)
  {
    attConfigurationShooter=i_shooter;
    return KD_OK;
  }


// ==========================================================================

  void ChppShooterActiveDof::addActDofSetter(const ChppActiveDofSetterShPtr& actdofsetter)
  {
    attListActDofSetters.push_back(actdofsetter);
  }

