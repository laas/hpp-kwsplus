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

#include "reedsSheppDirectPath.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsJoint.h"
#include "KineoWorks2/kwsDevice.h" // without this causes error when calling functions of kwsDevice





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
    cerr << " ERROR - CreedsSheppDirectPath::create failed : Reeds&Shepp implemented only for PLAN or FREEFLYER root joint without ExtraDof" << endl ;
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
    cerr << " ERROR - CreedsSheppDirectPath::create failed !!! " << endl ;
    pathShPtr.reset()	;
    return pathShPtr ;
  }

  if (pathPtr->init(pathWkPtr) != KD_OK) {
    pathShPtr.reset()	;
    return pathShPtr ;
  }

  // TODO
  //cout << endl << " implementer la possibilite de changer le type de courbe ALL/DUBINS/NOCUPS/WITHCUPS " << endl ;

  //debug
  //pathPtr->printDebug() ;

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

CreedsSheppDirectPath::CreedsSheppDirectPath(const CkwsConfig &inStartCfg, const CkwsConfig &inEndCfg,
					     const CkwsSteeringMethodShPtr &inSteeringMethod, const ERsCurveType inType)
  : CkwsDirectPath(inStartCfg, inEndCfg, inSteeringMethod)
{
  attTypeCurve = inType ;
  attRsCurveVector.clear() ;
}

// ==============================================================================

CreedsSheppDirectPath::CreedsSheppDirectPath(const CreedsSheppDirectPath &inDirectPath)
  : CkwsDirectPath(inDirectPath)
{
  attTypeCurve = inDirectPath.attTypeCurve ;
  attRsCurveVector = inDirectPath.attRsCurveVector ;
}
// ==============================================================================

ktStatus CreedsSheppDirectPath::init(const CreedsSheppDirectPathWkPtr &inWeakPtr)
{

  ktStatus success = CkwsDirectPath::init(inWeakPtr) ;

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
  //TODO
  cout << " DANGER !!!!!! TODO  CreedsSheppDirectPath :    maxAbsoluteDerivative -- inFrom " << inFrom << " inTo " << inTo << endl ;

  outVectorDeriv.clear() ;
  outVectorDeriv.push_back(1) ;
  outVectorDeriv.push_back(1) ;
  outVectorDeriv.push_back(1) ;
  outVectorDeriv.push_back(1) ;
  outVectorDeriv.push_back(1) ;
  outVectorDeriv.push_back(1) ;

  
  /*
   *  Input:  the robot,
   *          the local path,
   *          the parameter along the curve,
   *          the direction of motion,
   *          the array of maximal distances moved by all the points of 
   *          each body of the robot .
   *
   *  Output: distance the robot can move forward along the path without 
   *          moving by more than the input distance
   *
   *  Description:
   *          From a configuration on a local path, this function
   *          computes an interval of parameter on the local path on
   *          which all the points of the robot move by less than the
   *          distance given as input.  The interval starts at the
   *          configuration given as input. The function returns the
   *          length of the interval.
   *          Sketch of algorithm: the function computes sucessively
   *          for the plateform and then for each body an upper
   *          bound of the linear and angular velocities at the reference 
   *          point. For each body, the maximal range of parameter along 
   *          the curve is given by the minimal distance of the body to 
   *          the obstacles (stored in array distances) divided by the upper
   *          bound of the velocity of all its points. The minimal value of
   *          these parameter ranges is returned.
   *          
   */


  /*double p3d_rs_stay_within_dist(struct rob* robotPt,
    struct localpath* localpathPt,
    double parameter, whichway dir, 
    double* distances)*/

  //         TrsCurve curCurve ;
  //         double irs = 0 ;
  //
  //
  //
  //
  //
  //         int base_joint = 0;
  //         int trailer_joint = 1;
  //
  //         double max_param, min_param, range_param, tot_max_range;
  //         p3d_rs_data *curCurve=NULL;
  //         /* array of maximal linear and angular velocities of each body wrt
  //            the body it is attached to */
  //         int i, j, k;
  //         //p3d_jnt *cur_jntPt, *prev_jntPt;
  //         configPt q_max_param, q_param;
  //         p3d_stay_within_dist_data * stay_within_dist_data;
  //         //lm_reeds_shepp_str *rs_paramPt    = lm_get_reeds_shepp_lm_param(robotPt);
  //         //pflat_trailer_str trailer_paramPt = lm_get_trailer_lm_param(robotPt);
  //         int njnt = robotPt->njoints;
  //         int *other_jnt, nb_other_jnt;
  //         int x_coord, y_coord, z_coord;
  //
  //
  //
  //         curCurve = attRsCurveVector[irs];
  //
  //         /* store the data to compute the maximal velocities at the
  //            joint for each body of the robot */
  //         stay_within_dist_data = MY_ALLOC(p3d_stay_within_dist_data, njnt+2);
  //         p3d_init_stay_within_dist_data(stay_within_dist_data);
  //
  //         if (trailer_paramPt != NULL) {
  //                 other_jnt = trailer_paramPt->other_jnt;
  //                 nb_other_jnt = trailer_paramPt->nb_other_jnt;
  //                 base_joint = trailer_paramPt->numjnt[JNT_BASE];
  //                 trailer_joint = trailer_paramPt->numjnt[JNT_TRAILER];
  //                 x_coord = trailer_paramPt->numdof[DOF_X];
  //                 y_coord = trailer_paramPt->numdof[DOF_Y];
  //         } else {
  //                 other_jnt = rs_paramPt->other_jnt;
  //                 nb_other_jnt = rs_paramPt->nb_other_jnt;
  //                 base_joint = rs_paramPt->numjnt;
  //                 x_coord = rs_paramPt->numdof[DOF_RS_X];
  //                 y_coord = rs_paramPt->numdof[DOF_RS_Y];
  //         }
  //         z_coord = rs_paramPt->numdof[DOF_RS_Z];
  //
  //         /* Get the current config to have the modifications of the constraints */
  //         q_param = p3d_get_robot_config(robotPt);
  //
  //         /* get the RS segment on which the input configuration is */
  //         curCurve = rs_segment_at_parameter(robotPt, curCurve,
  //                          &parameter);
  //         range_param = curCurve->val_rs;
  //         tot_max_range = 0.;
  //         /* range_max is the maximal range possible whitout reaching bounds
  //            of local path. Notive that ranges are always positive */
  //         if (dir == FORWARD) {
  //                 min_param = max_param = range_param - parameter;
  //                 q_max_param = curCurve->q_end;
  //         } else {
  //                 min_param = max_param = parameter;
  //                 q_max_param = curCurve->q_init;
  //         }
  //
  //         /* loop on the Reeds and Shepp segments */
  //         while ((min_param>=0) && (curCurve != NULL)) {
  //                 k = 0;
  //                 for (i=0; i <= njnt; i++) {
  //                         cur_jntPt = robotPt->joints[i];
  //                         prev_jntPt = cur_jntPt->prev_jnt;
  //
  //                         if (i == base_joint) {
  //                                 /* We could compute the stay_within_dist for the Reed & Shepp method */
  //
  //                                 /* j = index of the joint to which the current joint is attached */
  //                                 prev_jntPt = cur_jntPt;
  //                                 j = -2;
  //                                 do {
  //                                         if (prev_jntPt == NULL) {
  //                                                 j = -1;
  //                                         } else {
  //                                                 if ((prev_jntPt->index_dof <= x_coord) &&
  //                                                                 (prev_jntPt->index_dof <= y_coord) &&
  //                                                                 ((prev_jntPt->index_dof <= z_coord) || (z_coord<0))) {
  //                                                         if (prev_jntPt->prev_jnt == NULL) {
  //                                                                 j = -1;
  //                                                         } else {
  //                                                                 j = prev_jntPt->prev_jnt->num;
  //                                                         }
  //                                                 } else {
  //                                                         prev_jntPt = prev_jntPt->prev_jnt;
  //                                                 }
  //                                         }
  //                                 } while(j==-2);
  //                                 if (trailer_paramPt!=NULL) {
  //                                         p3d_jnt_rs_trailer_stay_within_dist(&(stay_within_dist_data[j+1]),
  //                                                                             robotPt, curCurve,
  //                                                                             &(stay_within_dist_data[base_joint+1]),
  //                                                                             &(distances[base_joint]),
  //                                                                             &(stay_within_dist_data[trailer_joint+1]),
  //                                                                             &(distances[trailer_joint]),
  //                                                                             q_param, q_max_param, max_param, &min_param);
  //                                 } else {
  //                                         p3d_jnt_rs_stay_within_dist(&(stay_within_dist_data[j+1]),
  //                                                                     robotPt, curCurve,
  //                                                                     &(stay_within_dist_data[base_joint+1]),
  //                                                                     &(distances[base_joint]),
  //                                                                     q_param, q_max_param, max_param, &min_param);
  //                                 }
  //                                 /* All the joints which compose the trailer_joint have the same
  //                                    stay_within_dist (this majoration is too strong, but normaly,
  //                                    we couldn't have other link than base_joint, so we don't care) */
  //                                 while((cur_jntPt!=prev_jntPt) && (cur_jntPt->prev_jnt!=NULL)) {
  //                                         cur_jntPt = cur_jntPt->prev_jnt;
  //                                         stay_within_dist_data[cur_jntPt->num+1] =
  //                                                 stay_within_dist_data[base_joint+1];
  //                                 }
  //                         } else  if ((k < nb_other_jnt) && (i == other_jnt[k])) {
  //                                 k ++;
  //
  //                                 /* j = index of the joint to which the current joint is attached */
  //                                 if (prev_jntPt==NULL) {
  //                                         j = -1;
  //                                 } /* environment */
  //                                 else {
  //                                         j = prev_jntPt->num;
  //                                 }
  //
  //                                 p3d_jnt_stay_within_dist(&(stay_within_dist_data[j+1]), cur_jntPt,
  //                                                          &(stay_within_dist_data[i+1]),&(distances[i]),
  //                                                          q_param, q_max_param, max_param, &min_param);
  //                                 /* Rem: stay_within_dist_data[0] is bound to the environment */
  //                         }
  //                 }
  //
  //                 tot_max_range += min_param;
  //                 if ((dir == FORWARD) && (parameter+min_param>range_param-EPS6)) {
  //                         /* Test du chemin local suivant */
  //                         curCurve = curCurve->next_rs;
  //                         if (curCurve != NULL) {
  //                                 range_param = curCurve->val_rs;
  //                                 min_param = max_param = range_param;
  //                                 parameter = 0.;
  //                                 p3d_copy_config_into(robotPt, curCurve->q_init, &q_param);
  //                                 q_max_param = curCurve->q_end;
  //                         } else {
  //                                 max_param = -1.;
  //                         }
  //                 } else if ((dir == BACKWARD) && (parameter-min_param<EPS6)) {
  //                         /* Test du chemin local suivant */
  //                         curCurve = curCurve->prev_rs;
  //                         if (curCurve != NULL) {
  //                                 range_param = curCurve->val_rs;
  //                                 min_param = max_param = range_param;
  //                                 parameter = range_param;
  //                                 p3d_copy_config_into(robotPt, curCurve->q_end, &q_param);
  //                                 q_max_param = curCurve->q_init;
  //                         } else {
  //                                 min_param = -1.;
  //                         }
  //                 } else {
  //                         min_param = -1.;
  //                 }
  //         }
  //         MY_FREE(stay_within_dist_data, p3d_stay_within_dist_data, njnt+2);
  //         p3d_destroy_config(robotPt, q_param);
  //
  //         return tot_max_range;


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
  double    longueur_rs , t , u , v;
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
  case RS_DUBINS:
    longueur_rs = dubins(inStartCfg , inEndCfg , inRadius, numero , t , u , v);
    break;
  case RS_NO_CUSP:
    longueur_rs = reed_shepp_no_cusp(inStartCfg,inEndCfg, inRadius, numero, t, u, v);
    break;
  case RS_WITH_CUSP:
    longueur_rs = reed_shepp_with_cusp(inStartCfg,inEndCfg, inRadius , numero, t, u, v);
    break;
  default:
    cerr << " ERROR - CreedsSheppDirectPath::computeRSCurve : TypeCurve UNKNOWN" << endl ;
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
      cerr << "ERROR - CreedsSheppDirectPath::reed_shepp : infini" << endl;
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

double CreedsSheppDirectPath::dubins(CkwsConfig &c1 , CkwsConfig &c2, double radius , int &numero , double &t_r , double &u_r , double &v_r)
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

    t_r = t;
    u_r = u;
    v_r = v;
    numero = num;
    if(longueur == infini)
      cerr << "ERROR - CreedsSheppDirectPath::reed_shepp : infini" << endl;
    return(longueur);
  }

  /****  C S C ****/

  longueur = csca(x , y , phi , r , t1 , u1 , v1); /* l+ s+ l+ */
  num = 9;
  t = t1;
  u = u1;
  v = v1;

  var = csca(x ,-y , -phi , r , t1 , u1 , v1); /* r+ s+ r+ */
  if (var < longueur) {
    longueur = var;
    num = 10;
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


  t_r = t;
  u_r = u;
  v_r = v;
  numero = num;

  return(longueur);
}

// ==============================================================================

double CreedsSheppDirectPath::reed_shepp_no_cusp(CkwsConfig &c1 , CkwsConfig &c2, double radius , int &numero , double &t_r , double &u_r , double &v_r)
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
  var      = (c2.dofValue(dofRotz) -c1.dofValue(dofRotz));
  r        = radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (c1.dofValue(dofRotz));
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;


  if (fabs(var) <= M_PI)
    phi = var;
  else {
    if (c2.dofValue(dofRotz) >= c1.dofValue(dofRotz))
      phi = var - M_2PI;
    else
      phi = mod2pi(var);
  }


  t1 = u1 = v1 = 0.0;

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

    t_r = t;
    u_r = u;
    v_r = v;
    numero = num;
    return(longueur);
  }

  /****  C S C ****/

  longueur = csca(x , y , phi , r , t1 , u1 , v1); /* l+ s+ l+ */
  num = 9;
  t = t1;
  u = u1;
  v = v1;

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

  t_r = t;
  u_r = u;
  v_r = v;
  numero = num;

  return(longueur);
}


// ==============================================================================

double CreedsSheppDirectPath::reed_shepp_with_cusp(CkwsConfig &c1 , CkwsConfig &c2, double radius , int &numero , double &t_r , double &u_r , double &v_r)
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
  var      = (c2.dofValue(dofRotz) -c1.dofValue(dofRotz));
  r        = radius;

  theta    = atan2(dy , dx);
  alpha    = theta - (c1.dofValue(dofRotz));
  var_d    = sqrt(dx*dx + dy*dy);
  x        = cos(alpha)*var_d;
  y        = sin(alpha)*var_d;



  if (fabs(var) <= M_PI)
    phi = var;
  else {
    if (c2.dofValue(dofRotz) >= c1.dofValue(dofRotz))
      phi = var- 2*M_PI;
    else
      phi = mod2pi(var);
  }


  t1 = u1 = v1 = 0.0;

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

    t_r = t;
    u_r = u;
    v_r = v;
    numero = num;
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
    cerr << "ERROR - CreedsSheppDirectPath::computeRsCurveVector : type " << num << " UNKNOWN" << endl;
  }

  return KD_OK ;
}

// ==============================================================================

void CreedsSheppDirectPath::addRsCurveToVector(double r, int ty , int se , double val , CkwsConfig &curConfig )
{
  TrsCurve  curCurve;
  // WARNING Asign CKwsConfigShPtr
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

  while ( (param > attRsCurveVector[irs].val) && (irs < attRsCurveVector.size() )) {
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
    cerr << " ERROR -  CreedsSheppDirectPath::interpolate : Rs type curve UNKNOWN" << endl;
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

  cout << " curve.type :" << inCurve.type << endl ;
  cout << " curve.cd :" << inCurve.cd->dofValue(0) << "\t" << inCurve.cd->dofValue(1) << "\t"<< inCurve.cd->dofValue(dofRotz) << endl ;
  cout << " curve.cf :" << inCurve.cf->dofValue(0) << "\t" <<  inCurve.cf->dofValue(1) << "\t"<< inCurve.cf->dofValue(dofRotz) << endl ;
  cout << " curve.centre_x :" << inCurve.centre_x << endl ;
  cout << " curve.centre_y :" << inCurve.centre_y << endl ;
  cout << " curve.r :" << inCurve.r << endl ;
  cout << " curve.sens :" << inCurve.sens << endl ;
  cout << " curve.val :" << inCurve.val << endl ;
  cout << " curve.valid :" << inCurve.valid << endl ;
}

void CreedsSheppDirectPath::printDebug()
{

  for (unsigned int count=0 ; count < attRsCurveVector.size() ; count++) {
    cout << " // ---------- " << count << " ----------- // " << endl ;
    printTrsCurve(attRsCurveVector[count]) ;
  }

}


