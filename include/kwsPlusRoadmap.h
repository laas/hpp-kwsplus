/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by David Flavigne

*/

#ifndef HPP_ROADMAP_H_
#define HPP_ROADMAP_H_

/*------------------------------------*/
/*              INCLUDES              */
/*------------------------------------*/

#include "KineoKCDModel/kppKCDPolyhedron.h"
#include "KineoKCDModel/kppKCDAssembly.h"
#include "KineoUtility/kitPoint3.h"
#include "KineoWorks2/kwsRoadmap.h"

KIT_PREDEF_CLASS(CkwsPlusRoadmap);

/**
   \brief This class allows users to graphically display a roadmap.
   It shows only a 3D-space projection of the roadmap, if the robot have more than
   3 dof. Dofs should be in this order: X, Y, Z, ...  Linked devices must inherits 
   from CkppDeviceComponent, and be present in the model tree.

   To display the roadmap after a solution has been found, you can use methods compute()
   and display().

   You can compute and display the roadmap on run-time using the method "lastEdge" and a 
   roadmap builder delegate. In order to refresh the screen on run-time, you should also 
   use a progress delegate (A new class that inherits from both   CkwsRdmBuilderDelegate 
   and CkitProgressDelegate can be useful).

   Warning : if you have too many nodes in your roadmap, it will seriously slow KPP...

*/
class CkwsPlusRoadmap : public CkwsRoadmap, public CkppKCDAssembly {

 public:

  /**
     \brief creates an empty CkwsPlusRoadmap
  */
  static CkwsPlusRoadmapShPtr create(const CkwsDeviceShPtr & i_device,const std::string &inName = "");

  /**
     \brief computes the graphical roadmap. The roadmap should have nodes and edges. 
     Means that you can only use this function after the roadmap has been built  by 
     a roadmap builder.
  */
  void compute();

  /**
     \brief insert the roadmap in the modeltree. you can't see any roadmap without calling this function
   */
  void display();

  /**
     \name Run-Time display functions
     @{
   */

  /**
     \brief computes a graphical edge in the roadmap. Use this method only if you plan
     to display the roadmap on run time. In this case, you should use it in a class that inherits from a roadmap 
     builder delegate class (CkwsRdmBuilderDelegate). 
     This is actually not enough to see the roadmap building in real time. For this, your delegate class should 
     inherit from CkitProgressDelegate too, or a class that implements CkitProgressDelegate. You should also 
     call the display method before any calls to this method.

  */
  void lastEdge();

  /**
     }@
   */

  /**
     \brief Constructor
  */
  CkwsPlusRoadmap();

  /**
     \brief Destructor
  */
  virtual ~CkwsPlusRoadmap();

 protected:

  void init(const CkwsDeviceShPtr & i_device,const CkwsPlusRoadmapWkPtr& inRoadmapWkPtr ,const std::string &i_name);

 private:

  bool isInModelTree;

};

#endif /*HPP_ROADMAP_H*/
