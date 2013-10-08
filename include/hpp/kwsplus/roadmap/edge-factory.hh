/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#ifndef KWS_PLUS_EDGE_FACTORY_H
#define KWS_PLUS_EDGE_FACTORY_H

#include "KineoWorks2/kwsEdgeFactory.h"
#include <hpp/kwsplus/roadmap/edge.hh>

HPP_KIT_PREDEF_CLASS(CkwsPlusEdgeFactory);

/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/

/**
   \brief Factory for kwsPlus Edges
   
   to use kwsPlusEdges in your algorithms you should pass this factory to your roadmap builder :
   \code roadmapBuilder()->nodeFactory(CkwsPlusEdgeFactory::create()); \endcode

 */
class CkwsPlusEdgeFactory : public CkwsEdgeFactory
{

 public :
  /**
     \brief Destructor
   */
  ~CkwsPlusEdgeFactory();

  /**
     \brief Create Method.
     \return A shared Pointer on the newly created instance of CkwsPlusEdgeFactory.
   */
  static CkwsPlusEdgeFactoryShPtr create();

  /**
     \brief Factory method that creates a new edge. Inherited method.
     \param inDirectPath direct path for the edge.
     \return Shared pointer to a new instance of CkwsEdge
   */
  virtual CkwsEdgeShPtr makeEdge(const CkwsDirectPathConstShPtr &inDirectPath) const;
 protected :

  /**
     \brief Constructor
   */
  CkwsPlusEdgeFactory();

  /**
     \brief Intitalisation Method.
     \param inWeakPtr A weak pointer on the object itself
     \return KD_OK | KD_ERROR.
   */
  ktStatus init(const CkwsPlusEdgeFactoryWkPtr &inWeakPtr);
 private :

};

/**
   @}
*/
#endif
