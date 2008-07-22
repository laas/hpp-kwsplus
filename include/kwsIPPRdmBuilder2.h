#ifndef KWSIPPRRDMBUILDER_H
#define KWSIPPRRDMBUILDER_H

#include "KineoWorks2/kwsIPPRdmBuilder.h"
#include "KineoWorks2/kwsPickerBasic.h"
#include "KineoWorks2/kwsShooterConfigSpace.h"

KIT_PREDEF_CLASS(CkwsIPPRdmBuilder2);

class CkwsIPPRdmBuilder2 : public CkwsIPPRdmBuilder {
public:
  static CkwsIPPRdmBuilder2ShPtr create(const CkwsRoadmapShPtr &inRoadmap, 
					double inPenetration, 
					const CkwsDiffusionNodePickerShPtr &inPicker=CkwsPickerBasic::create(), 
					const CkwsDiffusionShooterShPtr &inShooter=CkwsShooterConfigSpace::create());
  ktStatus buildOneStep();

protected:
  ktStatus init (const CkwsIPPRdmBuilder2WkPtr &inWeakPtr, 
		 CkwsDiffusionNodePickerShPtr inPicker, 
		 CkwsDiffusionShooterShPtr inShooter);

  CkwsIPPRdmBuilder2(const CkwsRoadmapShPtr& inRoadmap);

private:
  CkwsIPPRdmBuilder2WkPtr attWeakPtr;
};


#endif
