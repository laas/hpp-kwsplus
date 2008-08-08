/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Luis Delgado (LAAS-CNRS)
*/

#ifndef HPP_VISIBILITY_RDM_BUILDER_H
#define HPP_VISIBILITY_RDM_BUILDER_H

/*************************************
INCLUDE
**************************************/
#include "KineoWorks2/kwsRoadmapBuilder.h"
#include "hppShooterActiveDof.h"


KIT_PREDEF_CLASS(ChppVisRdmBuilder);

/**
   \addtogroup visi
   @{
*/

/**
 \brief This class is an implementation of a visibility roadmap Builder.
  This class inherits from CkwsRoadmapBuilder.

\sa Smart pointers documentation: http://www.boost.org/libs/smart_ptr/smart_ptr.htm
*/

class ChppVisRdmBuilder : public CkwsRoadmapBuilder {

public:

 /**
     \brief Constructor.
     \param inRoadmap: The roadmap to construct.
     \param inPenetration: The penetration allowed.
     \param inShooter: The shooter used to generate the configurations.
     \param inEvaluator: The class used to evaluate the distance between two configurations.
 */
  static ChppVisRdmBuilderShPtr create (const CkwsRoadmapShPtr &inRoadmap,
                                        double inPenetration,
                                        const ChppShooterActiveDofShPtr& inShooter=ChppShooterActiveDof::create(),
                                        const CkwsDistanceShPtr &inEvaluator=CkwsDistance::create());

 /**
     \brief Destructor.
 */
  virtual ~ChppVisRdmBuilder() {};


protected:

  /**
    \brief Constructor
    \param inRoadmap: The roadmap of the roadmapBuilder.
  */
  ChppVisRdmBuilder(const CkwsRoadmapShPtr& inRoadmap);

 /**
     \brief Init function.

     \param inWeakPtr: The weak pointer to the visibiltyRdmBuilder.
     \param inShooter: The shooter to generate the configurations.

     \return ktStatus KD_OK or KD_ERROR
 */
  ktStatus init(const ChppVisRdmBuilderWkPtr& inWeakPtr, const ChppShooterActiveDofShPtr &inShooter);

  /**
    \brief This function is inherited and is modified here in order to give the visibility property to the roadmapbuilder.
    \return ktStatus KD_OK or KD_ERROR
  */
  virtual ktStatus buildOneStep();

  /**
    \brief This function is inherited and is modified here to determine when stop building the roadmap to decide that there's not solution.
    \return true if the roadmapbuilder should stop building the roadmap. False otherwise.
  */
  virtual bool plannerShouldStopPlanning(const CkwsPathPlannerConstShPtr &i_planner) const;

  /**
    \brief This function is inherited.
    \return 0.
  */
  virtual double difficulty() const;

private:

  /**
     \brief This function is used to add a node to the roadmap. After generate a node it's called to decide if the node has to be added or not to the roadmap. If it has to be added it will be.
     \param node: The node to add to the roadmap.
 */
  void addVisibilityNode(CkwsNodeShPtr node);

  /**
    \brief The shooter to generate the configurations.
  */
  ChppShooterActiveDofShPtr att_shooter;

  /**
    \brief Attributed used to count the number of iterations done in order to stop the construction of the roadmap.
  */
  unsigned int att_n_iterations;


};


/**
   @}
*/

#endif /*HPP_VISIBILITY_RDM_BUILDER_H*/














