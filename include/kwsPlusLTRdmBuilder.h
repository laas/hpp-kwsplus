/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigné (LAAS-CNRS)
*/


#ifndef KWS_PLUS_LT_ROADMAP_BUILDER
#define KWS_PLUS_LT_ROADMAP_BUILDER

/*************************************
INCLUDE
**************************************/

#include "KineoWorks2/kwsDiffusingRdmBuilder.h"
#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsDistance.h"
#include "KineoWorks2/kwsDevice.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsDiffusionNodePicker.h"
#include "KineoWorks2/kwsDiffusionShooter.h"
#include "KineoWorks2/kwsConnectedComponent.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsDof.h"
#include "KineoWorks2/kwsEdge.h"

#include "KineoUtility/kitInterface.h"
#include "KineoWorks2/kwsDefine.h"

#include <vector>

using namespace std;

KIT_PREDEF_CLASS( CkwsRoadmap );


template<class T = CkwsDiffusingRdmBuilder > class CkwsPlusLTRdmBuilder; 

class CkwsDiffusingRdmBuilder;
class CkwsIPPRdmBuilder;

#if 0
typedef  CkwsPlusLTRdmBuilder<CkwsDiffusingRdmBuilder> CkwsPlusLTDiffusingRdmBuilder;
typedef  CkwsPlusLTRdmBuilder<CkwsIPPRdmBuilder> CkwsPlusLTIPPRdmBuilder;

KIT_POINTER_DEFS(  CkwsPlusLTDiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusLTIPPRdmBuilder );
#endif

/*************************************
CLASS
**************************************/

/**
   \addtogroup LT
   @{
   This part implements the local trees algorithm presented by Molten Strandberg in "Augmenting RRT-Planners with Local Trees"
   \section loctree Local Trees Algorithm
   The principle of Local Trees is based on the idea of bi-diffuse algorithms. For diffusion algorithms, a common implementation is
   to grow trees from the start configuration AND from the goal, in order to increase success probability. Local Trees purpose is creating
   new trees in areas that can't be reached directly from the start tree nor the goal tree. The result is that short narrow passages are
   found easier (Short narrow passages means doors between two large empty rooms and not corridors).
   
   \image html LocalTreesProblem.png "Typical Problem for which Local Trees Algorithm is well-suited" 

   \section useLT Using Local Trees in Diffusing Algorithms
   The Local Trees method works well in problems where there large empty rooms separated by short narrow passages. For these, the classical Local Trees algorithm
   (means Local Trees executed with Classical RRT as the template class : CkwsDiffusingRdmBuilder). Executed with the Iterative Path Planner (IPP) method, it can
   overcome the problem of corridors and constrained environements.
   Local Trees method can also be executed with the PCA, but to take advantages of both algorithms one must do the template process like this:
   - Give Local Trees Roadmap builder as the template class for PCA.
   - Give any diffusing algorithm as template class for Local Trees.

   This order (PCA -> LT -> any Diffusing) is due to the specific implementation of both classes.
   

*/
/**
   \brief This template class inherits from any diffusing roadmap builder class,
   and it augment diffusing algorithms by creating local trees in areas that are
   unreachable by the current roadmap.
*/

template<class T >
class CkwsPlusLTRdmBuilder : public T
{ 
 public :
  /**
     \brief Destructor
   */
  ~CkwsPlusLTRdmBuilder();
  
  /**
     \brief Creates an instance of a Local Trees roadmap builder
     \param i_roadmap roadmap to work with
     \param i_penetration dynamic penetration
     \param i_evaluator distance evaluator
     \param i_picker diffusion node picker
     \param i_shooter diffusion shooter
     \return Shared pointer to a newly created roadmap builder
   */
  static 
    KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) 
    create(const CkwsRoadmapShPtr& i_roadmap, 
	   double i_penetration, 
	   const CkwsDistanceShPtr& i_evaluator=CkwsDistance::create(), 
	   const CkwsDiffusionNodePickerShPtr& i_picker=CkwsDiffusionNodePickerShPtr(), 
	   const CkwsDiffusionShooterShPtr &i_shooter=CkwsDiffusionShooterShPtr());
  
  /**
     \brief Updates the connectivity of the roadmap by checking if the new node can be linked with a Connected Component. When a connection happens, it updates the list of diffusion nodes (trees' roots), removing when necessary (You can't have 2 diffusion nodes in the same connected component)
     \param i_node The newly created node (after an extension step).
     \param i_picked The picked diffusion node for this step (For removing it if a connection happens).
     \param isAShootedNode boolean saying if i_node is a shooted node (extension step reached the shooted node) or not.
     \return True if the update have succeed, false otherwise.
   */ 
  bool updateRoadmapConnectivity(CkwsNodeShPtr i_node, CkwsNodeShPtr i_picked, bool isAShootedNode);

  /**
     \brief Adding a diffusion node if the shooted configuration is not colliding, and if it cannot be connected with another component (this last is tested with updateConnectivity).
     \param i_newnode The newly created node.
     \param i_shootednode The shooted node.
     \param i_picked The picked diffusion node for this step.
     \return KD_OK
   */
  ktStatus addingDiffusionNodes(CkwsNodeShPtr i_newnode, CkwsNodeShPtr i_shootednode, CkwsNodeShPtr i_picked);

 protected :

  /**
     \brief Processing an extension step for a diffusion algorithm (RRT). It calls the base class' extend method
     \param i_node to extend from/to
     \param i_cfg to extend to/from
     \param i_direction of extension:
     * ROADMAP_TO_NODE: from i_node to i_cfg
     * NODE_TO_ROADMAP: from i_cfg to i_node
     * BIDIRECTIONAL: forbidden value, does nothing (throws an assert in debug mode)
     \return created node (can be null; failure is normal)

   */
  virtual CkwsNodeShPtr extend (const CkwsNodeShPtr& i_node, const CkwsConfig& i_cfg, CkwsRoadmapBuilder::EDirection i_direction);

  /**
     \brief initialization
     \param i_weakPtr
     \param i_penetration
     \param i_picker
     \param i_shooter
     \return KD_OK | KD_ERROR 
   */
  ktStatus init(const KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) &i_weakPtr, double i_penetration, CkwsDiffusionNodePickerShPtr i_picker=CkwsDiffusionNodePickerShPtr(), CkwsDiffusionShooterShPtr i_shooter=CkwsDiffusionShooterShPtr());

  /**
     \brief Constructor
     \param i_roadmap roadmap to work with
     \param i_evaluator distance evaluator (defaults to CkwsDistance)
   */
  CkwsPlusLTRdmBuilder(const CkwsRoadmapShPtr &i_roadmap, const CkwsDistanceShPtr &i_evaluator=CkwsDistance::create());

 private :

  unsigned int oldStep;

};

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
bool CkwsPlusLTRdmBuilder<T>::updateRoadmapConnectivity(CkwsNodeShPtr i_node, CkwsNodeShPtr i_picked,bool isAShootedNode){

  CkwsNodeShPtr linkNode;
  CkwsDirectPathShPtr o_dp_in;
  CkwsDirectPathShPtr o_dp_out;
  bool success = false;
  
  for(unsigned int i=0;i<T::roadmap()->countConnectedComponents();i++){ //Checking if one or more of the existing connected components can be reached with i_node
    if(!success){
      if(i_node){ 
	if(T::roadmap()->connectedComponent(i) != i_node->connectedComponent()){
	  if(T::canLinkNodeWithComponent ( CkwsRoadmapBuilder::NODE_TO_ROADMAP ,i_node, T::roadmap()->connectedComponent(i), linkNode,o_dp_in) || T::canLinkNodeWithComponent ( CkwsRoadmapBuilder::ROADMAP_TO_NODE ,i_node, T::roadmap()->connectedComponent(i), linkNode,o_dp_out)){
	    
	    bool continuer = false;
	  
	    CkwsDirectPathConstShPtr c_dp_in(CkwsDirectPath::createCopy(o_dp_in));

	    if(i_node && c_dp_in){
	      if(KD_OK != T::roadmapLink(i_node,linkNode,c_dp_in)) ;
	      else{ continuer=true; success = true; }
	    }

	    int sum=0;
	    for(unsigned int i=0;i<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);i++){
	      
	      if(i_picked->connectedComponent() == T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i)->connectedComponent()){
		if(i_picked != T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i)) sum++;
	      }
	      
	    }
	    	    
	    if(continuer || sum){//then update diffusion nodes list
	      if(!isAShootedNode){
		std::vector<CkwsNodeShPtr> diffusion_nodes_list;
		for(unsigned int i=0; i<T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);i++){
		  diffusion_nodes_list.push_back(T::diffusionNode(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE,i));
		}
		T::resetDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE);
		
		int found = false;
		
		for(unsigned int i=0; i<diffusion_nodes_list.size();i++){
		  if(!found){//to avoid deletion of 2 diffusion nodes, and then having a connected component without a diffusion node.
		    if(!diffusion_nodes_list[i]->config().isEquivalent(i_picked->config())){
		      if(diffusion_nodes_list[i]->connectedComponent() != i_picked->connectedComponent() ){
			if(KD_OK != T::addDiffusionNode(diffusion_nodes_list[i],CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)) ;
		      }else{
			found = true;
		      }
		    }else{
		      found = true;
		    }
		  }else if(KD_OK != T::addDiffusionNode(diffusion_nodes_list[i],CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)) ;
		  
		}
		
		return true;
	      }
	    }
	  }
	}
      }
    }
  }
  return success;
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


template <class T>
CkwsPlusLTRdmBuilder<T>::~CkwsPlusLTRdmBuilder(){
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
ktStatus CkwsPlusLTRdmBuilder<T>::addingDiffusionNodes(CkwsNodeShPtr i_newnode, CkwsNodeShPtr i_shootednode, CkwsNodeShPtr i_picked){

  if(i_newnode){
    if(i_newnode != i_shootednode){ 
      bool linked = false;
      bool success = false;
      if(T::countDiffusionNodes(CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)<10){
	if(KD_OK == T::addNode(i_shootednode)){
	  success = true; 
	  linked = updateRoadmapConnectivity(i_shootednode,i_picked,true);
	  if(!linked){ 
	    if(KD_OK == T::addDiffusionNode(i_shootednode,CkwsDiffusingRdmBuilder::WAYPOINT_LIKE)){
	      if(KD_OK != T::roadmap()->device()->setCurrentConfig(i_shootednode->config())) cout<<"unable to set current config"<<endl;
	     }
	    if(KD_OK != T::roadmap()->removeNode(i_shootednode)) ;
	    
	  }
	}
      }
    }
  }
  


    return KD_OK;
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
CkwsNodeShPtr CkwsPlusLTRdmBuilder<T>::extend (const CkwsNodeShPtr& i_node, const CkwsConfig& i_cfg, CkwsRoadmapBuilder::EDirection i_direction){


  bool collision = true;

  if(T::roadmap()->countNodes() < oldStep){
    oldStep=T::roadmap()->countNodes();
    T::injectWaypointNodes();
  }

  CkwsNodeShPtr newNode = T::extend(i_node,i_cfg,i_direction);
  if(!newNode){;
  }else{
    updateRoadmapConnectivity(newNode,i_node,false);
  }
  if(KD_OK == i_cfg.isColliding(collision)){
    if(!collision){
      addingDiffusionNodes(newNode,CkwsNode::create(i_cfg),i_node);
    }
  }
  oldStep=T::roadmap()->countNodes();
  
  return newNode;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
CkwsPlusLTRdmBuilder<T>::CkwsPlusLTRdmBuilder(const CkwsRoadmapShPtr &i_roadmap, const CkwsDistanceShPtr &i_evaluator): T(i_roadmap,i_evaluator){
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
ktStatus CkwsPlusLTRdmBuilder<T>::init(const KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) &i_weakPtr, double i_penetration, CkwsDiffusionNodePickerShPtr i_picker, CkwsDiffusionShooterShPtr i_shooter){

  ktStatus res = KD_OK;
  oldStep = 0;
  res = T::init(i_weakPtr,i_penetration,i_picker,i_shooter);

  return res;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

template <class T>
KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) CkwsPlusLTRdmBuilder<T>::create(const CkwsRoadmapShPtr &i_roadmap, double i_penetration, const CkwsDistanceShPtr &i_evaluator, const CkwsDiffusionNodePickerShPtr &i_picker, const CkwsDiffusionShooterShPtr &i_shooter){

  cout<<endl;

  CkwsPlusLTRdmBuilder<T>* ptr = new CkwsPlusLTRdmBuilder(i_roadmap,i_evaluator);
  KIT_SHARED_PTR(CkwsPlusLTRdmBuilder<T>) shPtr(ptr);
  KIT_WEAK_PTR(CkwsPlusLTRdmBuilder<T>) wkPtr(shPtr);

  if(KD_OK != ptr->init(wkPtr,i_penetration,i_picker,i_shooter)){
    shPtr.reset();
  }

  return shPtr;

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

typedef  CkwsPlusLTRdmBuilder<CkwsDiffusingRdmBuilder> CkwsPlusLTDiffusingRdmBuilder;
typedef  CkwsPlusLTRdmBuilder<CkwsIPPRdmBuilder> CkwsPlusLTIPPRdmBuilder;

KIT_POINTER_DEFS(  CkwsPlusLTDiffusingRdmBuilder );
KIT_POINTER_DEFS(  CkwsPlusLTIPPRdmBuilder );

#endif //KWS_PLUS_LT_ROADMAP_BUILDER
