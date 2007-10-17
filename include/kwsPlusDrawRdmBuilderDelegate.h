#ifndef KWSPLUS_DRAWRDMBUILDERDELEGATE_H
#define KWSPLUS_DRAWRDMBUILDERDELEGATE_H

#include "KineoWorks2/kwsRdmBuilderDelegate.h"
#include "KineoUtility/kitProgressDelegate.h"

KIT_PREDEF_CLASS(CkwsPlusDrawRdmBuilderDelegate);

/**
   \addtogroup kwsPlus_graphic_rdm
   @{

*/
/**
   \brief Roadmap builder delegate that draws the roadmap.
*/

class CkwsPlusDrawRdmBuilderDelegate : public CkwsRdmBuilderDelegate, public CkitProgressDelegate {
 public:
  CkwsPlusDrawRdmBuilderDelegate() {};
  ~CkwsPlusDrawRdmBuilderDelegate() {};

  /**
     \brief Initialization
  */
  virtual void willStartBuilding(const CkwsRoadmapBuilderShPtr& inRdmBuilder, CkwsPathShPtr& inOutPath);

  /**
     \brief Draw the new edge of the roadmap
  */
  virtual void didAddEdge(const CkwsRoadmapBuilderConstShPtr &inRdmBuilder,
			  const CkwsEdgeConstShPtr & inEdge);

  virtual void didAddNode(const CkwsRoadmapBuilderConstShPtr& inRdmBuilder,
			  const CkwsNodeConstShPtr& inNode);

  virtual bool shouldStopBuilding(const CkwsRoadmapBuilderConstShPtr& inRdmBuilder);

  /**
     \brief Clone the CkitProgressDelegate object
  */
  virtual CkitProgressDelegateShPtr clone() const;

  virtual bool nextStep() {return true;};

  virtual void report(float, const std::string&){};

  virtual void refresh(){};

  virtual void start(){};

  virtual void finish(){};

  virtual bool shouldReport(){return true;};

  virtual bool cancelled(){return false;};

};

/**
   @}
*/

#endif
