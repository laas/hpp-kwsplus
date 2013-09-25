/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/


#ifndef KWS_PLUS_EDGE_H
#define KWS_PLUS_EDGE_H

#include "KineoWorks2/kwsEdge.h"
#include "KineoModel/kppColor.h"

KIT_PREDEF_CLASS(CkwsPlusEdge);

/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/
/**
   This class implements a classic kineo edge augmented with a color attribute, for graphical purposes, and an activation
   flag. Since there is no way to remove a node from a roadmap at run-time, this flag can be used to simulate a remove process.
   When it is false the edge will not be taken into account. The advantage is that you can desactivate and reactivate the edge
   when needed. The drawback is that a desactivated node is not deleted so it takes memory.
   \note You shouldn't use this class without CkwsPlusNode class for nodes.
*/
class CkwsPlusEdge : public CkwsEdge
{

 public :

  /**
     \brief Destructor
   */
  ~CkwsPlusEdge();

  /**
     \brief Creates an instance of a CkwsPlusEdge
     \param inDirectPath Direct path to create the edge from
     \return A shared pointer to the newly created edge.
   */
  static CkwsPlusEdgeShPtr create(const CkwsDirectPathConstShPtr &inDirectPath);

  /**
     \brief Sets the activation flag state (FALSE or TRUE)
     \param inState boolean giving the future state of the edge (activated if true, desactivated if false)
     \return KD_OK
   */
  ktStatus activate(bool inState); //activate or desactivate the edge

  /**
     \brief Gets the current state of the edge
     \return TRUE if the edge is activated, FALSE otherwise.
   */
  bool isEdgeActivated(); //retrieve whether edge is activated or not
  
  /**
     \brief Sets the color of the edge
     \param inColor New color
     \return KD_OK
   */
  void setColor(const CkppColor &inColor){attColor = inColor;}

  /**
     \brief Gets the current color of the edge
     \return The current color
  */
  CkppColor getColor(){return attColor;}

 protected :

  /**
     \brief Constructor
   */
  CkwsPlusEdge();

  /**
     \brief init method
     \param inDirectPath Direct path to create the edge from
     \param inWeakPtr Weak pointer on the new CkwsPlusEdge
     \return KD_OK | KD ERROR
   */
  ktStatus init(const CkwsDirectPathConstShPtr &inDirectPath,const CkwsPlusEdgeWkPtr &inWeakPtr);

 private :

  bool attIsActivated; //activation state of the edge
  CkppColor attColor; //color of the edge (default at black)

};

/**
   @}
*/
#endif /* KWS_PLUS_EDGE_H */
