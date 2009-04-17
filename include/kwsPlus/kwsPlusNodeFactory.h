/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#ifndef KWS_PLUS_NODE_FACTORY_H
#define KWS_PLUS_NODE_FACTORY_H

#warning "deprecated header file. Please include kwsPlus/roadmap/kwsNodeFactory.h instead."

#include "KineoWorks2/kwsNodeFactory.h"
#include "kwsPlus/roadmap/kwsPlusNode.h"

KIT_PREDEF_CLASS(CkwsPlusNodeFactory);


/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/

/**
   \brief Factory for kwsPlus Nodes
   
   to use kwsPlusNodes in your algorithms you should pass this factory to your roadmap builder :
   \code roadmapBuilder()->nodeFactory(CkwsPlusNodeFactory::create()); \endcode

 */
class CkwsPlusNodeFactory : public CkwsNodeFactory
{

 public :

  /**
     \brief Destructor
   */
  ~CkwsPlusNodeFactory();

  /**
     \brief Create Method.
     \return A shared Pointer on the newly created instance of CkwsPlusNodeFactory.
   */
  static CkwsPlusNodeFactoryShPtr create();

  /**
     \brief Factory method that creates a new node. Inherited method.
     \param inCfg configuration of the node
     \return Shared pointer to a new instance of CkwsNode 
   */
  virtual CkwsNodeShPtr makeNode(const CkwsConfig &inCfg) const;
 protected :

  /**
     \brief Constructor
   */
  CkwsPlusNodeFactory();

  /**
     \brief Intitalisation Method.
     \param inWeakPtr A weak pointer on the object itself
     \return KD_OK | KD_ERROR.
   */
  ktStatus init(const CkwsPlusNodeFactoryWkPtr &inWeakPtr);
 private :

};

/**
   @}
*/
#endif
