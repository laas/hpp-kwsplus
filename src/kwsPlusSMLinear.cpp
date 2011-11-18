/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Developed by Eiichi Yoshida (AIST/LAAS-CNRS)

*/

/*! \addtogroup hpp
 *@{
 */

/*****************************************
INCLUDES
*******************************************/

#include "kwsPlus/directPath/kwsPlusSMLinear.h"
#include "kwsPlus/directPath/kwsPlusDPLinear.h"

#if DEBUG==3
#define ODEBUG3(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG2(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#elif DEBUG==2
#define ODEBUG3(x)
#define ODEBUG2(x) std::cout << "CkwsPlusSMLinear:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#elif DEBUG==1
#define ODEBUG3(x)
#define ODEBUG2(x) 
#define ODEBUG1(x) std::cerr << "CkwsPlusSMLinear:" << x << std::endl
#else
#define ODEBUG3(x)
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

/*****************************************
PUBLIC METHODS
*******************************************/
// The following macro registers the class with the class registry,
// assigns the static CLASS attribute
// and implements the classObject() virtual method.
KIT_DEFINE_CLASS( CkwsPlusSMLinear );
// =========================================================================================

CkwsPlusSMLinear::CkwsPlusSMLinear(const std::vector<double> &inRatioVector, 
				   bool inOriented) 
  : attOriented(inOriented), attRatioVector(inRatioVector)
{
	//nothing to do
}

// =========================================================================================

CkwsPlusSMLinear::CkwsPlusSMLinear(const CkwsPlusSMLinear& i_steeringMethod) 
  : attOriented(i_steeringMethod.isOriented())
{
	i_steeringMethod.getRatioVector(attRatioVector);
}

// =========================================================================================

CkwsPlusSMLinear::CkwsPlusSMLinear() 
  : attOriented(false)
{
	//nothing to do
}

// =========================================================================================

CkwsPlusSMLinear::~CkwsPlusSMLinear() {
	//nothing to do
}

// =========================================================================================

ktStatus CkwsPlusSMLinear::init(const CkwsSteeringMethodWkPtr& inSmWeakPtr) {
 
    ktStatus  success = CkwsSteeringMethod::init(inSmWeakPtr);

    if(KD_OK == success) attWeakPtr = inSmWeakPtr;
    
    return success;
}

// ==========================================================================================

CkwsPlusSMLinearShPtr CkwsPlusSMLinear::create(const std::vector<double> &inRatioVector,
					       bool inOriented)
{
  CkwsPlusSMLinear*  flatPtr = new CkwsPlusSMLinear(inRatioVector, inOriented);
  CkwsPlusSMLinearShPtr flatShPtr(flatPtr);
  CkwsPlusSMLinearWkPtr flatWkPtr(flatShPtr);

  ODEBUG2( "steering method create" );

  if(flatPtr->init(flatWkPtr) != KD_OK) flatShPtr.reset();

  return flatShPtr;
}



// =========================================================================================

CkwsPlusSMLinearShPtr CkwsPlusSMLinear::createCopy(const CkwsPlusSMLinearConstShPtr& i_steeringmethod)
{
  CkwsPlusSMLinear*  flatPtr = new CkwsPlusSMLinear(*i_steeringmethod);
  CkwsPlusSMLinearShPtr flatShPtr(flatPtr);

  ODEBUG2( "steering method create" );

  if(flatPtr->init(flatShPtr) != KD_OK)
		flatShPtr.reset();

  return flatShPtr;
}

// =========================================================================================

CkwsDirectPathShPtr CkwsPlusSMLinear::makeDirectPath (const CkwsConfig &inStartConfig, const CkwsConfig &inEndConfig) const {
	
  // bool oriented = isOriented();
  return CkwsPlusDPLinear::create(inStartConfig, inEndConfig, attRatioVector, attWeakPtr.lock()) ;
 
}

// ==========================================================================================

bool CkwsPlusSMLinear::isOriented() const {
  return attOriented ;
}

// ==========================================================================================

void CkwsPlusSMLinear::getRatioVector(std::vector<double>& o_ratioVect)const
{
	o_ratioVect = attRatioVector;
}

// ==========================================================================================

CkwsSteeringMethodShPtr CkwsPlusSMLinear::clone()const
{
	return CkwsPlusSMLinear::createCopy(KIT_DYNAMIC_PTR_CAST(const CkwsPlusSMLinear, attWeakPtr.lock()));
}

// ==========================================================================================

void CkwsPlusSMLinear::encodeWithCoder(const CkitCoderShPtr& i_coder) const
{
  // Always call parent class coding:
  CkwsSteeringMethod::encodeWithCoder(i_coder);
  
  // Encode object attributes.
  // By convention, use identifier strings that match
  // attribute names.
  i_coder->encodeBool(attOriented, "oriented");
  i_coder->encodeDoubleVector(attRatioVector, "ratioVector");
}

// ==========================================================================================

ktStatus CkwsPlusSMLinear::initWithCoder(const CkitCoderShPtr& i_coder, const CkitCodableShPtr& i_self)
{
  ktStatus success = CkwsSteeringMethod::initWithCoder(i_coder, i_self);
  if(KD_OK == success)
  {
    attWeakPtr = KIT_DYNAMIC_PTR_CAST( CkwsPlusSMLinear, i_self );
    attOriented = i_coder->decodeBool("oriented");
    i_coder->decodeDoubleVector("ratioVector", attRatioVector);
  }
}


/** @}
 */
