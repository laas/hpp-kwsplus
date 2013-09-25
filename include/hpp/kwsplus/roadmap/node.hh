/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne (LAAS-CNRS)

*/

#ifndef KWS_PLUS_NODE_H
#define KWS_PLUS_NODE_H

#include "KineoWorks2/kwsNode.h"
#include "KineoModel/kppColor.h"


KIT_PREDEF_CLASS(CkwsPlusNode);

/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/

/**
   This class implements a classical kineo node augmented with several attributes :
   - The node's color, for graphical purposes (simple kwsNode is not aimed to be displayed)
   - The node's weight. The default implementation just set it with the distance to the goal (see CkwsPlusNodeFactory)
   - The node's collision probability. It is setted according to the success of the extension process
   - The node's retropropagation coefficient. Tells whether you should propagate the node's caracteritics to the parents.
   - The node's influence radius. Used for a specific shooter based on the roadmap nodes shooter principle.
   - The node's activation state. Since we cannot easily remove a node, this flag simulate a remove process.
   - The node's rank. Tells how far (in number of node) you are at maximum from a leaf.
   - The node's number of selections for extension. Tells how many times the node has been selected for extension (including times when the extension has failed).

   These attributes are aimed to have a better management of the roadmap at run-time, selecting nodes of interest (low collision probability and high weight) and displaying
   useful information on the nodes (with the color).
   Method that computes these attributs are made virtual to allow user to define their own.
   
   \note You shouldn't use this class without CkwsPlusEdge class for edges.
*/
class CkwsPlusNode : public CkwsNode
{

 public :

  /**
     \brief Destructor
   */
  ~CkwsPlusNode();

  /**
     \brief Creates an instance of a CkwsPlusNode
     \param inConfig Configuration of the new node
     \return A shared pointer on the newly created node
   */
  static CkwsPlusNodeShPtr create(const CkwsConfig& inConfig);

  /**
     \brief Computes the weight of the node. The default implementation only sets the attribute with the given value. 
     \param inWeight Weight of the node. Default is 1.
     \return KD_OK:
   */
  virtual ktStatus computeWeight(double inWeight = 1);

  /**
     \brief Computes the collision probability of the node. The default implementation only sets the attribute with the given value. 
     \param inCollision Collision probability of the node. Default is 1.
     \return KD_OK:
  
   */
  virtual ktStatus computeCollisionProbability(double inCollision = 1); 

  /**
     \brief Computes the number of collision times of the node. The default implementation only sets the attribute with the given value. 
     \param inCollision Collision times of the node. Default is 0.
     \return KD_OK:
  
   */
  virtual ktStatus computeCollisionTimes(double inCollision = 0); 

  /**
     \brief Computes the Color of the node. The default implementation only sets the attribute with the given value. 
     \param inColor Color of the node. Default is Black.
     \return KD_OK:
  
   */
  virtual ktStatus computeColor(CkppColor inColor = CkppColor(1,1,1,1));

  /**
     \brief Computes the Rank of the node. The default implementation only sets the attribute with the given value. 
     \param inRank Rank of the node. Default is 0.
     \return KD_OK:
  
   */
  virtual ktStatus computeRank(int inRank = 0);

  /**
     \brief Computes the RetroPropagation Coefficient of the node. The default implementation only sets the attribute with the given value. 
     \param inRetropropagation RetroPropagation Coefficient of the node. Default is 1.
     \return KD_OK:
  
   */
  virtual ktStatus computeRetroPropagationCoefficient(double inRetropropagation = 1);

  /**
     \brief Computes the Influence Radius of the node. The default implementation only sets the attribute with the given value. 
     \param inInfluenceradius Influence Radius of the node. Default is 1.
     \return KD_OK:
  
   */
  virtual ktStatus computeInfluenceRadius(double inInfluenceradius = 1 );

  /**
     \brief Computes the Number of Extension Times of the node. The default implementation only sets the attribute with the given value. 
     \param inNbExtensionTimes Number of Extension Times of the node. Default is 0.
     \return KD_OK:
  
   */
  virtual ktStatus computeNbExtensionTimes(int inNbExtensionTimes = 0 );

  /**
     \brief Get the current color of the node.
     \return The current color of the node.
   */
  CkppColor getColor(){return attColor;}

  /**
     \brief Get the current weight of the node.
     \return The current weight of the node.
   */
  double getWeight(){return attWeight;}

  /**
     \brief Get the current collision probability of the node.
     \return The current collision probability of the node.
   */
  double getCollisionProbability(){return attCollision;}

  /**
     \brief Get the current collision times of the node.
     \return The current collision times of the node.
   */
  double getCollisionTimes(){return attCollisionTimes;}

  /**
     \brief Get the current RetroPropagation Coefficient of the node.
     \return The current RetroPropagation Coefficient of the node.
   */
  double getRetroPropagationCoefficient(){return attRetropropagation;}

  /**
     \brief Get the current Influence Radius of the node.
     \return The current Influence Radius of the node.
   */
  double getInfluenceRadius(){return attInfluenceradius;}

  /**
     \brief Get the current rank of the node.
     \return The current rank of the node.
   */
  int getRank(){return attRank;}

  /**
     \brief Get the current number of extensions of the node.
     \return The current number of extensions of the node.
   */
  int getNbExtension(){return attNbExtensionTimes;}

  /**
     \brief Activate or desactivate the node, desactivating edges accordingly.
     \param inState State of the node. If true the node will be activated, otherwise it will be desactivated.
     \return KD_OK
   */
  ktStatus activate(bool inState); //activate or desactivate the node, desactivating edges accordingly.

  /**
     \brief Desactivate the node and create edges between the parent and the sons.
     \param inState State of the node. If true the node will be activated, otherwise it will be desactivated.
     \return KD_OK
   */
  ktStatus setCatch(bool inState); //desactivate the node and create edges beetween the parent and the sons.

  /**
     \brief Gets the current state of the node
     \return TRUE if the node is activated, FALSE otherwise.
   */
  bool isNodeActivated(){return attIsActivated;}

  /**
     \brief Gets the catch state of the node
     \return TRUE if a link is done between the parent node and the children in case of desactivation, FALSE otherwise.
   */
  bool getCatch(){return attCatch;}

 protected :

  /**
     \brief Constructor
     \param inConfig Configuration of the node
   */
  CkwsPlusNode(const CkwsConfig& inConfig);
  
  /**
     \brief Initializes the node.
     \param inWeakPtr Weak pointer on the node
     \return KD_OK | KD_ERROR
   */
  ktStatus init(const CkwsPlusNodeWkPtr &inWeakPtr);

 private :
  
  CkwsPlusNodeWkPtr attWeakPtr;
  CkppColor attColor;
  double attWeight; //weight of the node. Currently, inverse of the distance to the goal
  double attCollision; //collision probability
  double attCollisionTimes; //number of collision times
  double attRetropropagation; //coefficient for retropropagation
  double attInfluenceradius; //for a later shooter based on roadmap nodes
  bool attIsActivated;
  bool attCatch; //tells if in case of desactivation a link must be done between the parent and the children
  int attRank; //minimum rank from leafs
  int attNbExtensionTimes; //nb of times the node has been selected for extension

};

/**
   @}
*/
#endif /* KWS_PLUS_NODE_H */
