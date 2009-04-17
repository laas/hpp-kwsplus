/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Mathieu Poirier (LAAS-CNRS)
 
*/

/*****************************************
            DEFINES
*******************************************/

#define EPS3 0.001
#define EPS4 0.0001
#define EPS5 0.00001
#define EPS6 0.000001

#define EPSILON 1e-10

#define infini 10000000.

#define M_2PI 2*M_PI
#define M_PI2 0.5*M_PI

#define EQ(x,y)      (ABS((x)-(y)) < EPS5)
#define ABS(x)       (((x) >= 0.0) ? (x) : -(x))
#define FEQ(x,y,f)   (ABS((x)-(y)) < f)

/*****************************************
            INCLUDES
*******************************************/

#include "kwsPlus/directPath/reedsSheppDirectPath.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsJoint.h"
#include "KineoWorks2/kwsDevice.h" // without this causes error when calling functions of kwsDevice

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CreedsSheppDirectPath:" << x << std::endl
#define ODEBUG2(x) std::cout << "CreedsSheppDirectPath:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CreedsSheppDirectPath:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CreedsSheppDirectPath:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CreedsSheppDirectPath:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CreedsSheppDirectPath:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif




// ==============================================================================
//
//  METHOD OF CLASS ChppCmpPivoting
//
// ==============================================================================

CreedsSheppDirectPath::~CreedsSheppDirectPath()
{
  // nothing to do
}

// ==============================================================================

CreedsSheppDirectPathShPtr CreedsSheppDirectPath::create(const CkwsConfig &inStartCfg,const CkwsConfig &inEndCfg,
							 const CkwsSteeringMethodShPtr &inSteeringMethod, double inRadius, ERsCurveType inType )
{



  CreedsSheppDirectPath* pathPtr = new CreedsSheppDirectPath(inStartCfg, inEndCfg,inSteeringMethod, inType);
  CreedsSheppDirectPathShPtr pathShPtr(pathPtr);
  CreedsSheppDirectPathWkPtr pathWkPtr(pathShPtr) ;

       
  // test the root joint of the Device //
  if ( ((inStartCfg.device()->rootJoint()->countDofs() != 3) && (inStartCfg.device()->rootJoint()->countDofs() != 6)) || (inStartCfg.device()->countExtraDofs() != 0 ) ) {
    ODEBUG1(" ERROR - CreedsSheppDirectPath::create failed : Reeds&Shepp implemented only for PLAN or FREEFLYER root joint without ExtraDof") ;
    pathShPtr.reset() ;
    return pathShPtr ;
  }

  //selected the type of config
  if (inStartCfg.device()->rootJoint()->countDofs() == 3) {
    pathPtr->attTypeConfig = PLAN ;
  }
  if (inStartCfg.device()->rootJoint()->countDofs() == 6) {
    pathPtr->attTypeConfig = FREEFLYER ;
  }

  if (pathPtr->computeRSCurve( inStartCfg , inEndCfg , inRadius) != KD_OK) {
    ODEBUG1(" ERROR - CreedsSheppDirectPath::create failed !!! ") ;
    pathShPtr.reset()	;
    return pathShPtr ;
  }

  if (pathPtr->init(pathWkPtr) != KD_OK) {
    pathShPtr.reset()	;
    return pathShPtr ;
  }

 

  return pathShPtr ;


}
// ==============================================================================


CreedsSheppDirectPathShPtr CreedsSheppDirectPath::createCopy (const CreedsSheppDirectPathConstShPtr &inPivotingDP)
{

  if(inPivotingDP != NULL) {
    CreedsSheppDirectPath* pathPtr = new CreedsSheppDirectPath(*inPivotingDP) ;
    CreedsSheppDirectPathShPtr pathShPtr(pathPtr) ;
    CreedsSheppDirectPathWkPtr pathWkPtr(pathShPtr) ;

    if(pathPtr->init(pathWkPtr) != KD_OK) {
      pathShPtr.reset() ;
    }

    return pathShPtr;

  }

  return CreedsSheppDirectPathShPtr() ;

}


// ==============================================================================

double CreedsSheppDirectPath::angleBetween2Vector( CkitVect3 startVect , CkitVect3 endVect)
{

  double prodNorm ;

  prodNorm = startVect.norm() * endVect.norm() ;
  
 
  double sinA, cosA ;

  cosA = startVect.scalarProduct(endVect) / prodNorm ;

  sinA = ((startVect[0] * endVect[1]) - (startVect[1] * endVect[0])) / prodNorm ;
  
  return atan2(sinA, cosA) ;
}

// ==============================================================================

void CreedsSheppDirectPath::computeNewVal(TrsCurve &curCurve)
{
  CkitPoint3 Start, End ;

  Start[0] = curCurve.cd->dofValue(0) ;
  Start[1] = curCurve.cd->dofValue(1) ;
  End[0] = curCurve.cf->dofValue(0) ;
  End[1] = curCurve.cf->dofValue(1) ;


  if (curCurve.type == STRAIGHT) {
    curCurve.val = Start.distanceFrom(End) ;
  }
  else {
    CkitPoint3 center ;
    
    center[0] = curCurve.centre_x ;
    center[1] = curCurve.centre_y ;
    
    CkitVect3 center_Start, center_End ;

    center_Start = Start - center ;
    center_End   = End - center ;

    double angle = angleBetween2Vector(center_Start, center_End) ;

    curCurve.val = fabs(angle * curCurve.r) ;
    
  }
}

// ==============================================================================

std::vector< TrsCurve >CreedsSheppDirectPath::getInfoRsCurveVector()  {

  std::vector<TrsCurve> vectorCopy  ;
  
  double ustart = uStart() ;
  double uend = uEnd() ;

  unsigned int iStart = findRsCurveNumInVector(ustart) ;
  unsigned int iEnd = findRsCurveNumInVector(uend) ;
  
  for ( unsigned int i = iStart ; i <= iEnd ; i ++) {
    vectorCopy.push_back(attRsCurveVector[i]) ;
  }
  
  vectorCopy[0].cd = configAtStart() ;
  vectorCopy[vectorCopy.size()-1].cf = configAtEnd() ;
    
  computeNewVal(vectorCopy[0]) ;
  computeNewVal(vectorCopy[vectorCopy.size()-1]) ;
  
  //DEBUG
  //cout << " ustart : " << ustart << " uend : " << uend << endl ;
  //cout << " iStart : " << iStart << " iEnd : " << iEnd << endl ;
  //cout << "vector size : " << attRsCurveVector.size() << endl  ;
  //cout << "vector COPY size : "<< vectorCopy.size() << endl ;
  //cout << endl ;
  //cout << " =========== attRsCurveVector =========== " << endl ;
  //printDebug(attRsCurveVector) ;
  //cout << " =========== vectorCopy =========== " << endl ;
  //printDebug(vectorCopy) ;
  //cout << endl ;

  return vectorCopy ;
}


// ==============================================================================

CreedsSheppDirectPath::CreedsSheppDirectPath(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
					     const CkwsSteeringMethodShPtr &inSteeringMethod, const ERsCurveType inType)
  : CkwsPlusDirectPath(inStartCfg, inEndCfg, inSteeringMethod)
{
  attTypeCurve = inType ;
  attRsCurveVector.clear() ;
}

// ==============================================================================

CreedsSheppDirectPath::CreedsSheppDirectPath(const CreedsSheppDirectPath &inDirectPath)
  : CkwsPlusDirectPath(inDirectPath)
{
  attTypeCurve = inDirectPath.attTypeCurve ;
  attRsCurveVector = inDirectPath.attRsCurveVector ;
}
// ==============================================================================

ktStatus CreedsSheppDirectPath::init(const CreedsSheppDirectPathWkPtr &inWeakPtr)
{

  ktStatus success = CkwsPlusDirectPath::init(inWeakPtr) ;

  if (KD_OK == success)
    attWeakPtr = inWeakPtr;

  return success ;

}

// ==============================================================================

CkwsAbstractPathShPtr CreedsSheppDirectPath::clone() const
{

  return CreedsSheppDirectPath::createCopy(attWeakPtr.lock());

}

// ==============================================================================

double CreedsSheppDirectPath::computePrivateLength() const
{
  double sum = 0  ;

  for (unsigned int count =0 ; count < attRsCurveVector.size() ;count ++ ) {
    sum = sum + attRsCurveVector[count].val ;
  }

  return  sum;

}

// ==============================================================================

void CreedsSheppDirectPath::interpolate(double u , CkwsConfig & outCfg) const
{
  kwsConfigAtLengthParam( u , outCfg) ;
}

// ==============================================================================

void CreedsSheppDirectPath::maxAbsoluteDerivative(double inFrom, double inTo,
						  std::vector< double > & outVectorDeriv) const
{

  unsigned int istart = findRsCurveNumInVector(inFrom) ;
  unsigned int iend   = findRsCurveNumInVector(inTo) ;

  double maxDerivative = 1 ;

  for ( unsigned int i=istart ; i < iend  ; i++ ){
    if (attRsCurveVector[i].type != STRAIGHT) {
      maxDerivative = 1 / attRsCurveVector[i].r ;
    }
  }

  outVectorDeriv.clear() ;
  // X
  outVectorDeriv.push_back(1) ;
  // Y
  outVectorDeriv.push_back(1) ;
  
  if ( attTypeConfig == PLAN ) {
    // RZ
    outVectorDeriv.push_back(maxDerivative) ;
  }
  else {
    // Z
    outVectorDeriv.push_back(0) ;
    // RX
    outVectorDeriv.push_back(0) ;
    // RY
    outVectorDeriv.push_back(0) ;
    // RZ
    outVectorDeriv.push_back(maxDerivative) ; 
  }

}

// ==============================================================================

bool CreedsSheppDirectPath::isOriented () const
{
  return steeringMethod()->isOriented();
}

// ==============================================================================

ktStatus CreedsSheppDirectPath::computeRSCurve(CkwsConfig  inStartCfg , CkwsConfig inEndCfg , double inRadius)
{
  int      numero;
  double    longueur_rs=0 , t , u , v;
  CkwsConfig curConfig(inStartCfg);

  int dofRotz ;
  if ( attTypeConfig == PLAN )
    dofRotz = 2 ;
  else
    dofRotz = 5 ;

  // x
  curConfig.dofValue(0, inStartCfg.dofValue(0));
  // y
  curConfig.dofValue(1, inStartCfg.dofValue(1));
  // rot z
  curConfig.dofValue(dofRotz, inStartCfg.dofValue(dofRotz));


  switch(attTypeCurve) {
  case RS_ALL:
    longueur_rs = reed_shepp(inStartCfg , inEndCfg , inRadius, numero , t , u , v);
    break;
    //case RS_DUBINS:
    //longueur_rs = dubins(inStartCfg , inEndCfg , inRadius, numero , t , u , v);
    //break;
    //case RS_NO_CUSP:
    //longueur_rs = reed_shepp_no_cusp(inStartCfg,inEndCfg, inRadius, numero, t, u, v);
    //break;
    //case RS_WITH_CUSP:
    //longueur_rs = reed_shepp_with_cusp(inStartCfg,inEndCfg, inRadius , numero, t, u, v);
    //break;
  default:
    ODEBUG1(" ERROR - CreedsSheppDirectPath::computeRSCurve : TypeCurve UNKNOWN") ;
    break;
  }

  if(longueur_rs == infini)
    return KD_ERROR;

  /* TYPE DE COURBE CCSCC|| PAR R&S */

  if(computeRsCurveVector(numero , t , u , v , inRadius, curConfig ) == KD_OK) {
    return KD_OK;
  } else {
    return KD_ERROR;
  }
}

// ==============================================================================

double CreedsSheppDirectPath::reed_shepp(CkwsConfig &c1 , CkwsConfig &c2, double radius , int  &numero , double &t_r , double &u_r , double &v_r)
{
  double x , y , phi;
  double t , u , v , t1 , u1 , v1;
  int num;
  double var , var_d, theta , alpha , dx , dy , r, longueur;


  int dofRotz ;
  if ( attTypeConfig == PLAN )
    dofRotz = 2 ;
  else
    dofRotz = 5 ;

  /* Changement de repere,les courbes sont toujours calculees
     de (0 0 0)--> (x , y , phi) */
  dx       = (c2.dofValue(0) - c1.dofValue(0));
  dy       = (c2.dofValue(1) - c1.dofValue(1));
  var      = (c2.dofValue(dofRotz) - c1.dofValue(dofRotz));


  r        = radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (c1.dofValue(dofRotz));
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;


  t1 = u1 = v1 = 0.0;

  if (fabs(var) <= M_PI)
    phi = var;
  else {

    if (c2.dofValue(dofRotz) >= c1.dofValue(dofRotz))
      phi = var - M_2PI;
    else
      phi = mod2pi(var);

  }

  if(FEQ(r,0.0,EPS4)) {

    longueur = csca(x , y , phi , r , t1 , u1 , v1); /* l+ s+ l+ */
    var = t1+v1;
    num = 9;
    t = t1;
    u = u1;
    v = v1;

    csca(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ r+ */
    if((t1+v1) < (t+v)) {
      num = 10;
      t = t1;
      u = u1;
      v = v1;
    }
    csca(-x , y , -phi , r , t1 , u1 , v1); /* l- s- l- */
    if((t1+v1) < (t+v)) {
      num = 11;
      t = t1;
      u = u1;
      v = v1;
    }
    csca(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- r- */
    if((t1+v1) < (t+v)) {
      num = 12;
      t = t1;
      u = u1;
      v = v1;
    }
    cscb(x , y , phi , r , t1 , u1 , v1); /* l+ s+ r+ */
    if((t1+v1) < (t+v)) {
      num = 13;
      t = t1;
      u = u1;
      v = v1;
    }
    cscb(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ l+ */
    if((t1+v1) < (t+v)) {
      num = 14;
      t = t1;
      u = u1;
      v = v1;
    }
    cscb(-x , y , -phi , r , t1 , u1 , v1); /* l- s- r- */
    if((t1+v1) < (t+v)) {
      num = 15;
      t = t1;
      u = u1;
      v = v1;
    }
    cscb(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- l- */
    if((t1+v1) < (t+v)) {
      num = 16;
      t = t1;
      u = u1;
      v = v1;
    }

    t_r =  t;
    u_r = u;
    v_r = v;
    numero = num;
    if(longueur == infini) {
      ODEBUG1("ERROR - CreedsSheppDirectPath::reed_shepp : infini");
    }
    return(longueur);
  }
  /****  C | C | C ***/

  longueur = c_c_c(x , y , phi , r , t1 , u1 , v1); /* l+ r- l+ */
  num = 1;
  t = t1;
  u = u1;
  v = v1;

  var = c_c_c(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 2;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c_c(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 3;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c_c(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 4;
    t = t1;
    u = u1;
    v = v1;
  }

  /****  C | C C  ***/

  var = c_cc(x , y , phi , r , t1 , u1 , v1); /* l+ r- l- */
  if (var < longueur) {
    longueur = var;
    num = 5;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cc(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 6;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cc(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- r- */
  if (var < longueur) {
    longueur = var;
    num = 7;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cc(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 8;
    t = t1;
    u = u1;
    v = v1;
  }

  /****  C S C ****/

  var = csca(x , y , phi , r , t1 , u1 , v1); /* l+ s+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 9;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csca(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 10;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csca(-x , y , -phi , r , t1 , u1 , v1); /* l- s- l- */
  if (var < longueur) {
    longueur = var;
    num = 11;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csca(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- r- */
  if (var < longueur) {
    longueur = var;
    num = 12;
    t = t1;
    u = u1;
    v = v1;
  }


  var = cscb(x , y , phi , r , t1 , u1 , v1); /* l+ s+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 13;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cscb(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 14;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cscb(-x , y , -phi , r , t1 , u1 , v1); /* l- s- r- */
  if (var < longueur) {
    longueur = var;
    num = 15;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cscb(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- l- */
  if (var < longueur) {
    longueur = var;
    num = 16;
    t = t1;
    u = u1;
    v = v1;
  }

  /*** C Cu | Cu C ***/
  var = ccu_cuc(x , y , phi , r , t1 , u1 , v1); /* l+ r+ l- r- */
  if (var < longueur) {
    longueur = var;
    num = 17;
    t = t1;
    u = u1;
    v = v1;
  }

  var = ccu_cuc(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l+ r- l- */
  if (var < longueur) {
    longueur = var;
    num = 18;
    t = t1;
    u = u1;
    v = v1;
  }

  var = ccu_cuc(-x , y , -phi , r , t1 , u1 , v1); /* l- r- l+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 19;
    t = t1;
    u = u1;
    v = v1;
  }

  var = ccu_cuc(-x ,-y , phi , r , t1 , u1 , v1); /* r- l- r+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 20;
    t = t1;
    u = u1;
    v = v1;
  }

  /*** C | Cu Cu | C  ***/
  var = c_cucu_c(x , y , phi , r , t1 , u1 , v1); /* l+ r- l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 21;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cucu_c(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- r- l+ */
  if (var < longueur) {
    longueur = var;
    num = 22;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cucu_c(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 23;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_cucu_c(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 24;
    t = t1;
    u = u1;
    v = v1;
  }

  /*** C | C2 S C  ***/
  var = c_c2sca(x , y , phi , r , t1 , u1 , v1); /* l+ r- s- l- */
  if (var < longueur) {
    longueur = var;
    num = 25;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sca(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- s- r- */
  if (var < longueur) {
    longueur = var;
    num = 26;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sca(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ s+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 27;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sca(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ s+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 28;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2scb(x , y , phi , r , t1 , u1 , v1); /* l+ r- s- r- */
  if (var < longueur) {
    longueur = var;
    num = 29;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2scb(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- s- l- */
  if (var < longueur) {
    longueur = var;
    num = 30;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2scb(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ s+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 31;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2scb(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ s+ l+ */
  if (var < longueur) {
    longueur = var;
    num = 32;
    t = t1;
    u = u1;
    v = v1;
  }

  /*** C | C2 S C2 | C  ***/

  var = c_c2sc2_c(x , y , phi , r , t1 , u1 , v1); /* l+ r- s- l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 33;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sc2_c(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l- s- r- l+ */
  if (var < longueur) {
    longueur = var;
    num = 34;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sc2_c(-x , y , -phi , r , t1 , u1 , v1); /* l- r+ s+ l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 35;
    t = t1;
    u = u1;
    v = v1;
  }

  var = c_c2sc2_c(-x ,-y , phi , r , t1 , u1 , v1); /* r- l+ s+ r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 36;
    t = t1;
    u = u1;
    v = v1;
  }

  /***  C C | C  ****/

  var = cc_c(x , y , phi , r , t1 , u1 , v1); /* l+ r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 37;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cc_c(x ,-y , -phi , r , t1 , u1 , v1); /* r+ l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 38;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cc_c(-x , y , -phi , r , t1 , u1 , v1); /* l- r- l+ */
  if (var < longueur) {
    longueur = var;
    num = 39;
    t = t1;
    u = u1;
    v = v1;
  }

  var = cc_c(-x ,-y , phi , r , t1 , u1 , v1); /* r- l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 40;
    t = t1;
    u = u1;
    v = v1;
  }

  /*** C S C2 | C  ***/

  var = csc2_ca(x , y , phi , r , t1 , u1 , v1); /* l+ s+ r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 41;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_ca(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 42;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_ca(-x , y , -phi , r , t1 , u1 , v1); /* l- s- r- l+ */
  if (var < longueur) {
    longueur = var;
    num = 43;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_ca(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 44;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_cb(x , y , phi , r , t1 , u1 , v1); /* l+ s+ l+ r- */
  if (var < longueur) {
    longueur = var;
    num = 45;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_cb(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ r+ l- */
  if (var < longueur) {
    longueur = var;
    num = 46;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_cb(-x , y , -phi , r , t1 , u1 , v1); /* l- s- l- r+ */
  if (var < longueur) {
    longueur = var;
    num = 47;
    t = t1;
    u = u1;
    v = v1;
  }

  var = csc2_cb(-x ,-y , phi , r , t1 , u1 , v1); /* r- s- r- l+ */
  if (var < longueur) {
    longueur = var;
    num = 48;
    t = t1;
    u = u1;
    v = v1;
  }

  t_r = t;
  u_r = u;
  v_r = v;
  numero = num;

  return(longueur);
}


// ==============================================================================

ktStatus  CreedsSheppDirectPath::computeRsCurveVector(int num , double t, double u, double v , double r, CkwsConfig &conf )
{


  int gauche , droite , ligne , av ,ar;

  droite = 1;
  gauche = 2;
  ligne = 3;
  av = 1;
  ar = -1;


  switch(num) {
    /**         C | C | C   ****/
  case 1 :
    /*PrintInfo(("Courbe de type = L+ R- L+ \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 2 :
    /*PrintInfo(("Courbe de type = L- R+ L- \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 3 :
    /*PrintInfo(("Courbe de type = R+ L- R+ \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 4 :
    /*PrintInfo(("Courbe de type = R- L+ R- \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
    /**         C | C C      ****/
  case 5 :
    /*PrintInfo(("Courbe de type = L+ R- L- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 6 :
    /*PrintInfo(("Courbe de type = L- R+ L+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 7 :
    /*PrintInfo(("Courbe de type = r+ l- r-  \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 8 :
    /*PrintInfo(("Courbe de type = r- l+ r+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
    /**           C S C       *****/
  case 9 :
    /*PrintInfo(("Courbe de type = l+ s+ l+  \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,ligne ,  av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 10 :
    /*PrintInfo(("Courbe de type = r+ s+ r+ \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,ligne ,  av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 11 :
    /*PrintInfo(("Courbe de type = l- s- l- \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,ligne ,  ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 12 :
    /*PrintInfo(("Courbe de type = r- s- r- \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,ligne ,  ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 13 :
    /*PrintInfo(("Courbe de type = l+ s+ r+ \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,ligne ,  av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 14 :
    /*PrintInfo(("Courbe de type =  r+ s+ l+ \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,ligne ,  av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 15 :
    /*PrintInfo(("Courbe de type = l- s- r- \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,ligne ,  ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 16 :
    /*PrintInfo(("Courbe de type = r- s- l- \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,ligne ,  ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
    /*** C Cu | Cu C ***/
  case 17 :
    /*PrintInfo(("Courbe de type = l+ ru+ lu- r- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 18 :
    /*PrintInfo(("Courbe de type = r+ lu+ ru- l- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 19 :
    /*PrintInfo(("Courbe de type = l- ru- lu+ r+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 20 :
    /*PrintInfo(("Courbe de type = r- lu- ru+ l+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
    /*** C | Cu Cu | C  ***/
  case 21 :
    /*PrintInfo(("Courbe de type = l+ ru- lu- r+ \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 22 :
    /*PrintInfo(("Courbe de type = r+ lu- ru- l+ \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , ar , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 23 :
    /*PrintInfo(("Courbe de type = l- ru+ lu+ r- \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 24 :
    /*PrintInfo(("Courbe de type = r- lu+ ru+ l- \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av , u , conf );
    addRsCurveToVector(r,droite , av , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
    /*** C | C2 S C  ***/
  case 25 :
    /*PrintInfo(("Courbe de type = l+ r2- s- l- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 26 :
    /*PrintInfo(("Courbe de type = r+ l2- s- r- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 27 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ l+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 28 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ r+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 29 :
    /*PrintInfo(("Courbe de type = l+ r2- s- r- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 30 :
    /*PrintInfo(("Courbe de type = r+ l2- s- l- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 31 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ r+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 32 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ l+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
    /*** C | C2 S C2 | C  ***/
  case 33 :
    /*PrintInfo(("Courbe de type = l+ r2- s- l2- r+ \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 34 :
    /*PrintInfo(("Courbe de type = r+ l2- s- r2- l+  \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 35 :
    /*PrintInfo(("Courbe de type = l- r2+ s+ l2+ r- \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 36 :
    /*PrintInfo(("Courbe de type = r- l2+ s+ r2+ l- \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
    /***  C C | C  ****/
  case 37 :
    /*PrintInfo(("Courbe de type = l+ r+ l- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,droite , av , u, conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 38 :
    /*PrintInfo(("Courbe de type = r+ l+ r- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,gauche , av , u, conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 39 :
    /*PrintInfo(("Courbe de type = l- r- l+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,droite , ar , u, conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 40 :
    /*PrintInfo(("Courbe de type = r- l- r+\n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,gauche , ar , u , conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
    /*** C S C2 | C  ***/
  case 41 :
    /*PrintInfo(("Courbe de type = l+ s+ r2+ l- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 42 :
    /*PrintInfo(("Courbe de type = r+ s+ l2+ r- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 43 :
    /*PrintInfo(("Courbe de type = l- s- r2- l+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  case 44 :
    /*PrintInfo(("Courbe de type = r- s- l2- r+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 45 :
    /*PrintInfo(("Courbe de type = l+ s+ l2+ r- \n"));*/
    addRsCurveToVector(r,gauche , av , t , conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,gauche , av ,M_PI2, conf );
    addRsCurveToVector(r,droite , ar , v , conf );
    break;
  case 46 :
    /*PrintInfo(("Courbe de type = r+ s+ r2+ l- \n"));*/
    addRsCurveToVector(r,droite , av , t , conf );
    addRsCurveToVector(r,ligne  , av , u , conf );
    addRsCurveToVector(r,droite , av ,M_PI2, conf );
    addRsCurveToVector(r,gauche , ar , v , conf );
    break;
  case 47 :
    /*PrintInfo(("Courbe de type = l- s- l2- r+ \n"));*/
    addRsCurveToVector(r,gauche , ar , t , conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,gauche , ar ,M_PI2, conf );
    addRsCurveToVector(r,droite , av , v , conf );
    break;
  case 48 :
    /*PrintInfo(("Courbe de type = r- s- r2- l+ \n"));*/
    addRsCurveToVector(r,droite , ar , t , conf );
    addRsCurveToVector(r,ligne  , ar , u , conf );
    addRsCurveToVector(r,droite , ar ,M_PI2, conf );
    addRsCurveToVector(r,gauche , av , v , conf );
    break;
  default:
    ODEBUG1("ERROR - CreedsSheppDirectPath::computeRsCurveVector : type " << num << " UNKNOWN");
  }

  return KD_OK ;
}

// ==============================================================================

void CreedsSheppDirectPath::addRsCurveToVector(double r, int ty , int se , double val , CkwsConfig &curConfig )
{
  TrsCurve  curCurve;
 
  curCurve.cd = CkwsConfig::create(curConfig.device()) ;
  curCurve.cf  = CkwsConfig::create(curConfig.device()) ;

  double   va, curTheta;

  /* CAS OU LA VALEUR = 0 */

  if ((ty == 3)||(EQ(r,0.0)))
    va = val;
  else
    va = val * r;

  if(!FEQ(va , 0.0 , EPS5)) {

    int dofRotz ;
    if ( attTypeConfig == PLAN )
      dofRotz = 2 ;
    else
      dofRotz = 5 ;

    /** config initiale **/
    curCurve.cd->dofValue(0, curConfig.dofValue(0));
    curCurve.cd->dofValue(1, curConfig.dofValue(1));
    curCurve.cd->dofValue(dofRotz, curConfig.dofValue(dofRotz));
    curCurve.sens = se;
    curCurve.val  = va;
    curCurve.r    = r;
    curCurve.valid = true;

    switch(ty ) {
      /*******   cas d'un arc du cercle a droite  ***********/
    case RIGHT :
      curCurve.type = RIGHT;

      /** calcul du centre du cercle **/
      curCurve.centre_x = curConfig.dofValue(0) + r*sin(curConfig.dofValue(dofRotz));
      curCurve.centre_y = curConfig.dofValue(1) - r*cos(curConfig.dofValue(dofRotz));

      /** calcul du point extremite pour un arc de longueur val **/
      va = atan2((curCurve.cd->dofValue(1) -curCurve.centre_y),(curCurve.cd->dofValue(0) -curCurve.centre_x));
      if( se == 1 )
	va = ( va - val);
      else
	va = ( va + val);
      va = mod2pi(va);

      curCurve.cf->dofValue(0,  curCurve.centre_x + r*cos(va));
      curCurve.cf->dofValue(1, curCurve.centre_y + r*sin(va));
      val       =  curCurve.cd->dofValue(dofRotz) - se*val;
      curTheta = mod2pi(val) ;
      if ((curTheta - M_PI) > 0) curTheta = - (M_2PI - curTheta) ;
      curCurve.cf->dofValue(dofRotz, curTheta );
      break;

      /******   cas d'un arc du cercle a gauche  ***********/
    case LEFT :
      curCurve.type = LEFT;

      /** calcul du centre du cercle **/
      curCurve.centre_x = curConfig.dofValue(0) - r*sin(curConfig.dofValue(dofRotz));
      curCurve.centre_y = curConfig.dofValue(1) + r*cos(curConfig.dofValue(dofRotz));

      /** calcul du point extremite pour un arc de longueur val **/
      va = atan2((curCurve.cd->dofValue(1) -curCurve.centre_y),(curCurve.cd->dofValue(0) -curCurve.centre_x));
      if( se == 1 )
	va =( va + val);
      else
	va =( va - val);
      va = mod2pi(va);

      curCurve.cf->dofValue(0, curCurve.centre_x + r*cos(va));
      curCurve.cf->dofValue(1, curCurve.centre_y + r*sin(va));
      val       = curCurve.cd->dofValue(dofRotz) + se*val;
      curTheta = mod2pi(val) ;
      if ((curTheta - M_PI) > 0) curTheta = - (M_2PI - curTheta) ;
      curCurve.cf->dofValue(dofRotz, curTheta );
      break;

      /******   cas d'un segment de droite      ***********/
    case STRAIGHT :
      curCurve.type = STRAIGHT;
      curCurve.centre_x = curCurve.centre_y = 0.;
      curCurve.cf->dofValue(0, curCurve.cd->dofValue(0) + se*val*cos(curCurve.cd->dofValue(dofRotz)) );
      curCurve.cf->dofValue(1, curCurve.cd->dofValue(1) + se*val*sin(curCurve.cd->dofValue(dofRotz)) );
      curCurve.cf->dofValue(dofRotz, curCurve.cd->dofValue(dofRotz) );
      break;
    }

    /* on met a jour la nouvelle config de depart
       pour la prochaine portion de courbe + misea jour pointeur */
    curConfig.dofValue(0, curCurve.cf->dofValue(0));
    curConfig.dofValue(1, curCurve.cf->dofValue(1));
    curConfig.dofValue(dofRotz, curCurve.cf->dofValue(dofRotz));

    //add to the Vector
    attRsCurveVector.push_back(curCurve) ;

  }
}

// ==============================================================================

unsigned int CreedsSheppDirectPath::findRsCurveNumInVector(double &param) const
{
  unsigned int irs = 0 ;

  if (param < 0) {
    param = 0;
  }

  double EPS = 1e-6 ;

  while ( ((param - attRsCurveVector[irs].val) > EPS) && (irs < attRsCurveVector.size() )) {

    param = param - attRsCurveVector[irs].val;
    irs ++ ;
    
  }

  return irs ;
}


// ==============================================================================
void CreedsSheppDirectPath::kwsConfigAtLengthParam(double u, CkwsConfig &outCfg) const
{
  double paramLength = u ;

  unsigned int irs = findRsCurveNumInVector(paramLength) ;

  CkwsConfig curConfig(outCfg);
  double a,a1,a2 ;
  double lengthPartition = attRsCurveVector[irs].val ;
  double radius = attRsCurveVector[irs].r;
  double theta_init, theta_end;

  int dofRotz ;
  if ( attTypeConfig == PLAN )
    dofRotz = 2 ;
  else
    dofRotz = 5 ;

 

  theta_init = attRsCurveVector[irs].cd->dofValue(dofRotz);
  theta_end = attRsCurveVector[irs].cf->dofValue(dofRotz);

  switch(attRsCurveVector[irs].type) {
  case RIGHT:
    if(attRsCurveVector[irs].sens > 0) {
      a1 = mod2pi(theta_init+M_PI2);
      a2 = mod2pi(theta_end+M_PI2);
      if(a1<a2) {
	a1=a1+2*M_PI;
      }
    } else {
      a1 = mod2pi(theta_init+M_PI2);
      a2 = mod2pi(theta_end+M_PI2);
      if(a1>a2) {
	a1=a1-2*M_PI;
      }
    }
    a = (1-(paramLength/lengthPartition))*a1+ (paramLength/lengthPartition)*a2;

    curConfig.dofValue(0, attRsCurveVector[irs].centre_x + radius * cos(a) );
    curConfig.dofValue(1, attRsCurveVector[irs].centre_y + radius * sin(a) );
    curConfig.dofValue(dofRotz, a-M_PI2);
    curConfig.dofValue(dofRotz, angleLimitPi(curConfig.dofValue(dofRotz)) );
    break;
  case LEFT:
    if(attRsCurveVector[irs].sens > 0) {
      a1 = mod2pi(theta_init-M_PI2);
      a2 = mod2pi(theta_end-M_PI2);
      if(a1>a2) {
	a1=a1-2*M_PI;
      }
    } else {
      a1 = mod2pi(theta_init-M_PI2);
      a2 = mod2pi(theta_end-M_PI2);
      if(a1<a2) {
	a1=a1+2*M_PI;
      }
    }
    a = (1-(paramLength/lengthPartition))*a1+ (paramLength/lengthPartition)*a2;
    curConfig.dofValue(0, attRsCurveVector[irs].centre_x + radius * cos(a));
    curConfig.dofValue(1, attRsCurveVector[irs].centre_y + radius * sin(a));
    curConfig.dofValue(dofRotz, a+M_PI2);
    curConfig.dofValue(dofRotz, angleLimitPi(curConfig.dofValue(dofRotz)));
    break;
  case STRAIGHT:
    curConfig.dofValue(0,  (1-(paramLength/lengthPartition))*attRsCurveVector[irs].cd->dofValue(0)+ (paramLength/lengthPartition)*attRsCurveVector[irs].cf->dofValue(0) );
    curConfig.dofValue(1, (1-(paramLength/lengthPartition))*attRsCurveVector[irs].cd->dofValue(1)+(paramLength/lengthPartition)*attRsCurveVector[irs].cf->dofValue(1));
    curConfig.dofValue(dofRotz, (1-(paramLength/lengthPartition))*theta_init+ (paramLength/lengthPartition)*theta_end);
    break;
  default:
    ODEBUG1(" ERROR -  CreedsSheppDirectPath::interpolate : Rs type curve UNKNOWN");
    return;
  }

  outCfg = curConfig ;
}

/*********************************************************************************/

/****  C | C | C ***/
double CreedsSheppDirectPath::c_c_c(double x , double y , double phi , double radius , double &t , double &u ,double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4))
    return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > (4.0 * radius))
    return(infini);
  else {
    theta = atan2(b , a);
    alpha = acos(u1/(4.0 * radius));
    va = M_PI_2 + alpha;
    t = mod2pi(va + theta);
    u = mod2pi(2.0 * (M_PI - va));
    v = mod2pi(phi - (t) - (u));
    long_rs = radius*(t + u + v);
    return(long_rs);
  }
}

/*********************************************************************************/

/****  C | C C  *****/
double CreedsSheppDirectPath::c_cc(double x , double y , double phi , double radius , double &t , double &u , double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4))
    return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > 4*radius)
    return(infini);
  else {
    theta = atan2(b , a);
    alpha = acos(u1/(4.0 * radius));
    va = M_PI_2 + alpha;
    t = mod2pi(va + theta);
    u = mod2pi(2.0 * (M_PI - va));
    v = mod2pi(t + u - phi);
    long_rs = radius*(t +  u + v);
    return(long_rs);
  }
}

/*********************************************************************************/

/****  C S C  ****/
double CreedsSheppDirectPath::csca(double x , double y , double phi , double radius , double &t , double &u ,double& v)
{
  double a , b , long_rs;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  u = sqrt(a*a + b*b);
  t = mod2pi(atan2(b , a));
  v = mod2pi(phi - t);
  long_rs = radius*(t + v) + u;
  return(long_rs);
}

/*********************************************************************************/

double CreedsSheppDirectPath::cscb(double x ,  double y , double phi , double radius , double &t , double &u ,double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = sqrt(u1*u1 - va*va);
    alpha = atan2(va , u);
    t = mod2pi(theta + alpha);
    v = mod2pi(t - phi);
    long_rs = radius*(t + v) + u;
    return(long_rs);
  }
}

/*********************************************************************************/

/*** C Cu | Cu C  ***/
double CreedsSheppDirectPath::ccu_cuc(double x , double y , double phi , double radius , double &t , double &u ,double& v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4))
    return(infini);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 > va)
    return(infini);
  else {
    theta = atan2(b , a);
    if (u1 > (2.0 *radius)) {
      alpha = acos(u1/(4.0 * radius) - 0.5);
      t = mod2pi(M_PI_2 + theta - alpha);
      u = mod2pi(M_PI - alpha);
      v = mod2pi(phi - t + 2.0 * (u));
    } else {
      alpha = acos(0.5 + u1/(4.0 * radius));
      t = mod2pi(M_PI_2 + theta + alpha);
      u = mod2pi(alpha);
      v = mod2pi(phi - t + 2.0 * (u));
    }
    long_rs = radius * (2.0 *u + t + v);
    return(long_rs);
  }
}

/*********************************************************************************/

/****************  C | Cu Cu | C  **************/
double CreedsSheppDirectPath::c_cucu_c(double x , double y , double phi , double radius , double &t , double &u , double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  double toto;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4))
    return(infini);
  u1 = sqrt(a*a + b*b);
  if (u1 > 6.0 * radius)
    return(infini);
  else {
    theta = atan2(b , a);
    va = 1.25 -  (u1*u1) / (16.0 * radius * radius);
    if ((va<0.)||(va>1.))
      return(infini);
    else {
      u = acos(va);
      toto = sin(u);
      if (FEQ(toto ,0.0 ,EPS3))
	toto = 0.;
      alpha = asin(radius * toto * 2.0/u1);
      t = mod2pi(M_PI_2 + theta + alpha);
      v = mod2pi(t - phi);
      long_rs = radius*(2.0 *u + t + v);
      return(long_rs);
    }
  }
}

/*********************************************************************************/

/****************  C | C2 S C ******************/
double CreedsSheppDirectPath::c_c2sca(double x , double y , double phi , double radius , double &t , double &u ,double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) - 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = sqrt(u1*u1 - va*va) - va;
    if (u < 0.0)
      return(infini);
    else {
      alpha = atan2(va , (u + va));
      t = mod2pi(M_PI_2 + theta + alpha);
      v = mod2pi(t +M_PI_2 - phi);
      long_rs = radius*(t + M_PI_2 + v) + u;
      return(long_rs);
    }
  }
}


/*********************************************************************************/

double CreedsSheppDirectPath::c_c2scb(double x , double y , double phi , double radius , double &t , double &u , double &v)
{
  double a , b , u1 , theta , long_rs , va;
  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) +1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    t = mod2pi(M_PI_2 + theta);
    u = u1 - va;
    v = mod2pi(phi - t - M_PI_2);
    long_rs = radius*(t + M_PI_2 + v) + u;
    return(long_rs);
  }
}

/*********************************************************************************/

/****************  C | C2 S C2 | C  ***********/
double CreedsSheppDirectPath::c_c2sc2_c(double x , double y , double phi , double radius , double &t , double &u, double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = sqrt(u1*u1 - va*radius) - va;
    if (u < 0.0)
      return(infini);
    else {
      alpha = atan2(2*radius , (u + va));
      t = mod2pi(M_PI_2 + alpha + theta);
      v = mod2pi(t - phi);
      long_rs = radius*(t + M_PI + v) + u;
      return(long_rs);
    }
  }
}

/*********************************************************************************/

/****************  C C | C  ****************/
double CreedsSheppDirectPath::cc_c(double x , double y , double phi , double radius , double &t , double &u , double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;
  double toto;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) -1.0);
  if (FEQ(a, 0.,EPS4)&&FEQ(b , 0.,EPS4))
    return(infini);
  u1 = sqrt(a*a + b*b);
  va = 4.0 * radius;
  if (u1 > va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = acos(1.0 -  (2.0 * u1 * u1)/(va*va));
    toto = sin(u);
    if (FEQ(toto ,0.0 ,EPS3))
      toto = 0.0;
    /*BMIC*/
    if (FEQ(toto ,0.0 ,EPS3) &&FEQ(u1 ,0.0 ,EPS3))
      return(infini);
    alpha = asin((2.0 * radius * toto)/(u1));
    /*EMIC*/
    t = mod2pi(M_PI_2 + theta - alpha);
    v = mod2pi(t - u - phi);
    long_rs = radius*(t + u + v);
    return(long_rs);
  }
}

/*********************************************************************************/

/****************  C S C2 | C  ************/
double CreedsSheppDirectPath::csc2_ca(double x , double y , double phi , double radius , double &t , double &u , double &v)
{
  double a , b , u1 , theta , alpha , long_rs , va;

  a = x - radius*sin(phi);
  b = y + radius*(cos(phi) -1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = sqrt(u1*u1 - va*va) - va;
    if (u < 0.0)
      return(infini);
    else {
      alpha = atan2((u + va), va);
      t = mod2pi(M_PI_2 + theta - alpha);
      v = mod2pi(t - M_PI_2 - phi);
      long_rs = radius * (t + M_PI_2 + v) + u;
      return(long_rs);
    }
  }
}

/*********************************************************************************/

double CreedsSheppDirectPath::mod2pi(double a) const
{
  return((double)((a=fmod(a, M_2PI))>0.0)?a:(FEQ(a,0.0,EPSILON)?0.0:(a + M_2PI)));
}


/*********************************************************************************/

double CreedsSheppDirectPath::angleLimitPi(double angle) const
{
  while (angle < -M_PI){
    angle += 2*M_PI;
  }
  while (angle > M_PI){
    angle -= 2*M_PI;
  }
  return angle;

}


/*********************************************************************************/

double CreedsSheppDirectPath::csc2_cb(double x , double y , double phi , double radius , double &t , double &u ,double &v)
{
  double a , b , u1 , theta , long_rs , va;

  a = x + radius*sin(phi);
  b = y - radius*(cos(phi) + 1.0);
  u1 = sqrt(a*a + b*b);
  va = 2.0 * radius;
  if (u1 < va)
    return(infini);
  else {
    theta = atan2(b , a);
    u = u1 - va;
    t = mod2pi(theta);
    v = mod2pi(phi - M_PI_2 - t);
    long_rs = radius*(t + M_PI_2 + v) + u;
    return(long_rs);
  }

}

// ==============================================================================

void CreedsSheppDirectPath::printTrsCurve( const TrsCurve inCurve)
{
  int dofRotz ;
  if ( attTypeConfig == PLAN )
    dofRotz = 2 ;
  else
    dofRotz = 5 ;

  ODEBUG2( " curve.type :" << inCurve.type);
  ODEBUG2( " curve.cd :" << inCurve.cd->dofValue(0) << "\t" << inCurve.cd->dofValue(1) << "\t"<< inCurve.cd->dofValue(dofRotz));
  ODEBUG2( " curve.cf :" << inCurve.cf->dofValue(0) << "\t" <<  inCurve.cf->dofValue(1) << "\t"<< inCurve.cf->dofValue(dofRotz));
  ODEBUG2( " curve.centre_x :" << inCurve.centre_x);
  ODEBUG2( " curve.centre_y :" << inCurve.centre_y);
  ODEBUG2( " curve.r :" << inCurve.r);
  ODEBUG2( " curve.sens :" << inCurve.sens);
  ODEBUG2( " curve.val :" << inCurve.val );
  ODEBUG2( " curve.valid :" << inCurve.valid );
}

// ==============================================================================


void CreedsSheppDirectPath::printDebug(std::vector<TrsCurve> RSvector)
{

  for (unsigned int count=0 ; count < RSvector.size() ; count++) {
    ODEBUG2( " // ---------- " << count << " ----------- // ");
    printTrsCurve(RSvector[count]) ;
  }

}


