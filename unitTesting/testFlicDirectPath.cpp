/*
  Copyright CNRS 2006

  Author: Florent Lamiraux
*/

#include <iostream>
#include <fstream>

#include "angle/angle.h"

#include "testFlicDirectPath.h"
#include "kwsioConfig.h"

#include "KineoController/kppSetPropertyCommand.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"
#include "KineoModel/kppSolidComponentRef.h"
#include "KineoModel/kppFreeFlyerJointComponent.h"
#include "KineoKCDModel/kppKCDBox.h"

/*--------------------------------------------------
 *
 *                P U B L I C    M E T H O D S
 *
 *-------------------------------------------------- */


ktStatus CtestFlicDirectPath::init()
{
  if (createDevice() != KD_OK) {
    cerr << "CtestFlicDirectPath::init: Could not create device." << endl;
    return KD_ERROR;
  }

  if (createSteeringMethod() != KD_OK) {
    cerr << "CtestFlicDirectPath::init: Could not create steering method." << endl;
    return KD_ERROR;
  }

  if (createFlicDistance() != KD_OK) {
    cerr << "CtestFlicDirectPath::init: Could not create distance." << endl;
    return KD_ERROR;
  }
  
  return KD_OK;
}


ktStatus CtestFlicDirectPath::plotMappingsAndBoundingStruct(unsigned int nbRndDirectPath)
{
  CkwsConfig initConfig(device()), goalConfig(device());
  CkwsDirectPathShPtr kwsDirectPath;

  // Loop over random direct paths.
  for (unsigned int iDP=0; iDP < nbRndDirectPath; iDP++) {

    // Pick two random configurations.
    initConfig.randomize();
    goalConfig.randomize();

    // Set non planar dof to 0
    initConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
    initConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
    initConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);

    goalConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
    goalConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
    goalConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);

    // Build direct path in between.
    kwsDirectPath = steeringMethod()->makeDirectPath(initConfig, goalConfig);

    CflicDirectPathShPtr flicDirectPath;
    double minNormGamma1du, minNormGamma1ds;

    if (flicDirectPath = boost::dynamic_pointer_cast<CflicDirectPath>(kwsDirectPath)) {
      plotMappingDefaultToArcLengthParam("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      plotMappingArcLengthToDefaultParam("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      minNormGamma1du = plotBoundsDgammaOverDu("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      minNormGamma1ds = plotBoundsDgammaOverDs("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      plotBoundsD2gammaOverDu2("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      plotBoundsD2gammaOverDs2("/home/florent/robots/hrp2/humPathPlanner/testHpp/plot", flicDirectPath);
      
      if (minNormGamma1ds < .5) {
	cerr << "lowerBound of dgamma/ds = " << minNormGamma1ds << endl;
      }
      if (minNormGamma1du < .5) {
	cerr << "lowerBound of dgamma/du = " << minNormGamma1du << endl;
      }
      unsigned int i=1;
      cout << "0 to stop, 1 to continue: ";
      cin >> i;
      
      if (i==0) {
	break;
      }
    }
  }
  return KD_OK;
}


void CtestFlicDirectPath::plotMappingDefaultToArcLengthParam(std::string dirName, 
							     CflicDirectPathShPtr flicDirectPath)
{
  std::string filename; 
  CflicArcLengthManagerShPtr arcLengthManager;
  arcLengthManager = flicDirectPath->attArcLengthManager;

  unsigned int nbSamples = 10*arcLengthManager->attNbSampleIntervals;
  std::ofstream fileStream;
  //
  // Plot mapping u to s
  //
  
  // Open file
  filename = dirName + "/UtoS.plot";
  fileStream.open(filename.c_str());
  for (unsigned int iSample=0; iSample <= nbSamples; iSample++) {
    double u = (iSample*1.0/nbSamples);
    double s = arcLengthManager->attDefaultToArcLength.value(u);
    fileStream << u << "\t" << s << endl;
  }
  // Close file.
  fileStream.close();
  
  //
  // Plot mapping u to \dot{s}
  //
  
  // Open file
  
  filename = dirName + "/UtoSdot.plot";
  fileStream.open(filename.c_str());
  for (unsigned int iSample=0; iSample <= nbSamples; iSample++) {
    double u = (iSample*1.0/nbSamples);
    double sDot = flicDirectPath->normGammaDeriv1(u);
    fileStream << u << "\t" << sDot << endl;
  }
  // Close file.
  fileStream.close();
}


void CtestFlicDirectPath::plotMappingArcLengthToDefaultParam(std::string dirName, 
							     CflicDirectPathShPtr flicDirectPath)
{
  std::string filename; 
  CflicArcLengthManagerShPtr arcLengthManager;
  arcLengthManager = flicDirectPath->attArcLengthManager;

  unsigned int nbSamples = 10*arcLengthManager->attNbSampleIntervals;
  std::ofstream fileStream;

  //
  // Plot mapping u to s
  //
  
  // Open file
  filename = dirName + "/StoU.plot";
  fileStream.open(filename.c_str());
  for (unsigned int iSample=0; iSample <= nbSamples; iSample++) {
    double s = (iSample*flicDirectPath->length()/nbSamples);
    double u = arcLengthManager->attArcLengthToDefault.value(s);
    fileStream << s << "\t" << u << endl;
  }
  // Close file.
  fileStream.close();
}

double CtestFlicDirectPath::plotBoundsDgammaOverDu(std::string dirName, 
						   CflicDirectPathShPtr flicDirectPath)
{
  double minNormGamma1 = 1e8;
  std::string filename; 
  CflicBoundManagerDefParamShPtr boundManager = 
    flicDirectPath->attBoundManagerDefParam;
  
  if (boundManager->boundListDeriv1.size() == 0) {
    return minNormGamma1;
  }
  std::list<TflicBoundInterval>::iterator iter;
  std::list<TflicBoundInterval>::iterator iterEnd = boundManager->boundListDeriv1.end();

  // Open file
  std::ofstream fileStream;
  filename = dirName + "/boundsDgammaOverDu.plot";
  fileStream.open(filename.c_str());

  for (iter = boundManager->boundListDeriv1.begin();
       iter != iterEnd; iter++) {
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;

    // Update min of gamma' norm
    if (iter->valueMin < minNormGamma1) {
      minNormGamma1 = iter->valueMin;
    }
  }
  // Close file.
  fileStream.close();
  return minNormGamma1;
}

void CtestFlicDirectPath::plotBoundsD2gammaOverDu2(std::string dirName, 
						   CflicDirectPathShPtr flicDirectPath)
{
  std::string filename; 
  CflicBoundManagerDefParamShPtr boundManager = 
    flicDirectPath->attBoundManagerDefParam;
  
  std::list<TflicBoundInterval>::iterator iter;
  std::list<TflicBoundInterval>::iterator iterEnd = boundManager->boundListDeriv2.end();

  std::ofstream fileStream;
  // Open file
  filename = dirName + "/boundsD2gammaOverDu2.plot";
  fileStream.open(filename.c_str());

  for (iter = boundManager->boundListDeriv2.begin();
       iter != iterEnd; iter++) {
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;

  }
  // Close file.
  fileStream.close();

}

double CtestFlicDirectPath::plotBoundsDgammaOverDs(std::string dirName, 
						   CflicDirectPathShPtr flicDirectPath)
{
  double minNormGamma1 = 2.0;
  std::string filename; 
  CflicBoundManagerArcLengthParamShPtr boundManager = 
    flicDirectPath->attBoundManagerArcLengthParam;
  
  if (boundManager->boundVectorDeriv1.size() == 0) {
    return 2.0;
  }
  std::vector<TflicBoundInterval>::iterator iter;
  std::vector<TflicBoundInterval>::iterator iterEnd = boundManager->boundVectorDeriv1.end();

  std::ofstream fileStream;
  // Open file
  filename = dirName + "/boundsDgammaOverDs.plot";
  fileStream.open(filename.c_str());

  for (iter = boundManager->boundVectorDeriv1.begin();
       iter < iterEnd; iter++) {
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;

    // Update min of gamma' norm
    if (iter->valueMin < minNormGamma1) {
      minNormGamma1 = iter->valueMin;
    }
  }
  // Close file.
  fileStream.close();
  return minNormGamma1;
}

void CtestFlicDirectPath::plotBoundsD2gammaOverDs2(std::string dirName, 
						   CflicDirectPathShPtr flicDirectPath)
{
  std::string filename; 
  CflicBoundManagerArcLengthParamShPtr boundManager = 
    flicDirectPath->attBoundManagerArcLengthParam;
  
  std::vector<TflicBoundInterval>::iterator iter;
  std::vector<TflicBoundInterval>::iterator iterEnd = boundManager->boundVectorDeriv2.end();

  std::ofstream fileStream;
  // Open file
  filename = dirName + "/boundsD2gammaOverDs2.plot";
  fileStream.open(filename.c_str());

  for (iter = boundManager->boundVectorDeriv2.begin();
       iter < iterEnd; iter++) {
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMax << endl;
    fileStream << iter->uMin << "\t" << iter->valueMin << endl;
    fileStream << iter->uMax << "\t" << iter->valueMin << endl;

  }
  // Close file.
  fileStream.close();
}


ktStatus CtestFlicDirectPath::testStraightLineDirectPath()
{
  CkwsConfig initConfig(device()), goalConfig(device());
  CkwsDirectPathShPtr kwsDirectPath;

  // Set non planar dof to 0
  initConfig.dofValue(CflicDirectPath::CURV_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::X_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::Y_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::RZ_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
  initConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);
  
  goalConfig.dofValue(CflicDirectPath::CURV_COORD, 0.0);
  goalConfig.dofValue(CflicDirectPath::X_COORD, 3.0);
  goalConfig.dofValue(CflicDirectPath::Y_COORD, 0.0);
  goalConfig.dofValue(CflicDirectPath::RZ_COORD, 0.0);
  goalConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
  goalConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
  goalConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);
  
  kwsDirectPath = steeringMethod()->makeDirectPath(initConfig, goalConfig);
  if (kwsDirectPath) {
    double pathLength = kwsDirectPath->length();
    std::cout << "  path length = " << pathLength << std::endl;
    std::cout << std::endl;
    
    for (double s=0; s<pathLength; s+=0.01) {
      CkwsConfigShPtr q = kwsDirectPath->configAtDistance(s);
      std::cout << "s=" << s << "; " << *q << std::endl;
    }
    return KD_OK;    
  }
  else {
    std::cerr << "CtestFlicDirectPath::testStraightLineDirectPath: failed to build a straight line direct path." 
	      << std::endl;
    return KD_ERROR;
  }
  
  return KD_OK;
}

ktStatus CtestFlicDirectPath::testApproximateLength(unsigned int nbRndDirectPath)
{
  CkwsConfig initConfig(device()), goalConfig(device());
  CkwsDirectPathShPtr kwsDirectPath;
  double pathLength;

  // Loop over random direct paths.
  for (unsigned int iDP=0; iDP < nbRndDirectPath; iDP++) {
    // Pick two random configurations.
    initConfig.randomize();
    goalConfig.randomize();

    // Set non planar dof to 0
    initConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
    initConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
    initConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);

    goalConfig.dofValue(CflicDirectPath::Z_COORD, 0.0);
    goalConfig.dofValue(CflicDirectPath::RX_COORD, 0.0);
    goalConfig.dofValue(CflicDirectPath::RY_COORD, 0.0);
  
    if (iDP == 0) {
      initConfig.dofValue(CflicDirectPath::X_COORD, 0.0);
      initConfig.dofValue(CflicDirectPath::Y_COORD, 0.0);
      initConfig.dofValue(CflicDirectPath::RZ_COORD, 0.0);

      goalConfig.dofValue(CflicDirectPath::X_COORD, 2.0);
      goalConfig.dofValue(CflicDirectPath::Y_COORD, 0.0);
      goalConfig.dofValue(CflicDirectPath::RZ_COORD, 0.0);
    }

    if (iDP == 1) {
      initConfig.dofValue(CflicDirectPath::X_COORD, 0.0);
      initConfig.dofValue(CflicDirectPath::Y_COORD, 0.0);
      initConfig.dofValue(CflicDirectPath::RZ_COORD, .5*M_PI);

      goalConfig.dofValue(CflicDirectPath::X_COORD, 0.0);
      goalConfig.dofValue(CflicDirectPath::Y_COORD, 3.0);
      goalConfig.dofValue(CflicDirectPath::RZ_COORD, .5*M_PI);
    }

    if (iDP < nbRndDirectPath/2) {
      initConfig.dofValue(CflicDirectPath::CURV_COORD, 0.0);
      goalConfig.dofValue(CflicDirectPath::CURV_COORD, 0.0);
    }      

    kwsDirectPath = steeringMethod()->makeDirectPath(initConfig, goalConfig);
    if (kwsDirectPath) {
      pathLength = kwsDirectPath->length();
    }
    else {
      pathLength = 1e10;
    }
    double approximateLength = attFlicDistance->distance(initConfig, goalConfig);
    
    std::cout << "Init config = (" << initConfig << ")" << std::endl;
    std::cout << "Goal  config = (" << goalConfig << ")" << std::endl;
    std::cout << "Length direct path = " << pathLength << std::endl;
    std::cout << "Approximate distance = " << approximateLength << std::endl;
    std::cout << std::endl;
  }
  return KD_OK;
}

/*--------------------------------------------------
 *
 *                P R I V A T E    M E T H O D S
 *
 *-------------------------------------------------- */

ktStatus CtestFlicDirectPath::createDevice()
{

  // Create empty device and store it in object
  CkppDeviceComponentShPtr kppDevice=CkppDeviceComponent::create("Cart");

  CkitMat4 absPos;
  CkppKCDBoxShPtr polyBox = CkppKCDBox::create("CartBox",0.40,0.85,1.10);
  polyBox->diffuseColor( CkppColor( 0.2f, 0.2f, 0.6f, 0.4f ) );
  absPos.translate(0.0,0.0,0.55);
  polyBox->setAbsolutePosition( absPos );
  polyBox->makeCollisionItem();
  
  
  // create ExtraDof
  CkwsDofShPtr kwsExtraDof ;
  kwsExtraDof = CkwsDof::create(false);
  // Bound curvature
  kwsExtraDof->bounds(-5.0, 5.0);
  kwsExtraDof->isBounded(true);
  kppDevice->addExtraDof(kwsExtraDof);
  
  
  // build plan root joint
  CkppJointComponentShPtr kppJoint = CkppFreeFlyerJointComponent::create( "ROOTJOINT.BB" );
  // Bound joint
  // X
  kppJoint->kwsJoint()->dof(0)->isBounded(true);
  kppJoint->kwsJoint()->dof(0)->bounds(-10.0,10.0); 
  // Y
  kppJoint->kwsJoint()->dof(1)->isBounded(true);
  kppJoint->kwsJoint()->dof(1)->bounds(-10.0,10.0); 
  // Z
  kppJoint->kwsJoint()->dof(2)->isLocked(true);
  // Rx
  kppJoint->kwsJoint()->dof(3)->isLocked(true);
  // Ry
  kppJoint->kwsJoint()->dof(4)->isLocked(true);
  // Rz
  kppJoint->kwsJoint()->dof(5)->isBounded(false);
  kppJoint->kwsJoint()->dof(5)->bounds(-M_PI,M_PI);

  //attach the body to the joint 
  kppJoint->addSolidComponentRef( CkppSolidComponentRef::create(polyBox) ); 
  kppJoint->doesDisplayPath( true );
  
  //attach the joint to the device
  kppDevice->rootJointComponent(kppJoint);

  // create to config for test
  CkwsConfigShPtr startConfig=CkwsConfig::create(kppDevice->kwsDevice()) ;
  CkwsConfigShPtr endConfig  =CkwsConfig::create(kppDevice->kwsDevice()) ;

  startConfig->dofValue(0, 0.0) ;  // Kappa
  startConfig->dofValue(1, 0.0) ;  // x  
  startConfig->dofValue(2, 0.0) ;  // y
  startConfig->dofValue(6, 0.0) ;  // theta
  
  endConfig->dofValue(0, 0.0) ; // Kappa  
  endConfig->dofValue(1, 2.0) ; // x
  endConfig->dofValue(2, 1.0) ; // y
  endConfig->dofValue(6, 0.0) ; // theta 
  
  device(kppDevice);
  
  return KD_OK;
}

ktStatus CtestFlicDirectPath::createSteeringMethod()
{
  // Create steering method : Flat Interpolation 
  CflicSteeringMethodShPtr kwsSteeringMethod = CflicSteeringMethod::create(false);
  if (!kwsSteeringMethod) {
    return KD_ERROR;
  }
  steeringMethod(kwsSteeringMethod);
  return KD_OK;
}

ktStatus CtestFlicDirectPath::createFlicDistance()
{
  if (!steeringMethod()) {
    std::cerr << "CtestFlicDirectPath::createFlicDistance: create steering method first." 
	      << std::endl;
    return KD_ERROR;
  }
  attFlicDistance = CflicDistance::create(steeringMethod());
  if (!attFlicDistance) {
    return KD_ERROR;
  }
  return KD_OK;
}

