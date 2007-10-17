/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Developed by Florent Lamiraux And Mathieu Poirier (LAAS-CNRS)
 
*/

#ifndef  __KITMAt3_H__
#define  __KITMAT3_H__


#include "KineoUtility/kitMat4.h"

/**
   \addtogroup kwsPlus_misc
   @{
*/

/**
   \brief Rotation matrices.
*/

class CkitMat3
{
public:
        /**
           \brief Construct an identity rotation matrix
        */
        CkitMat3();

        /**
           \brief Copy constructor
        */
        CkitMat3(const CkitMat3& inMat);

        /**
            \brief Constructor by coefficients
        */
        CkitMat3(const double inCoef[3][3]);

        /**
            \brief Access to the coefficients
        */
        double & operator()(const unsigned int i, const unsigned int j);

        /**
            \brief Read only access to the coefficients
        */
        double operator()(const unsigned int i, const unsigned int j) const;

        /**
           \brief Inverse fonction (equivalent to transpose).
        */
        CkitMat3 inverse(const CkitMat3& inMat) const;

        /**
           \brief Multiplication by a matrix
        */

        CkitMat3 operator*(const CkitMat3& inMat) const;

        /**
           \brief Multiplication by a vector
        */

        CkitVect3 operator*(const CkitVect3& inVect) const;

        /**
        \brief Get the Bryant angle (Angle used by KineoWorks) from a kitMat3 
         */
        void mat3ToBryant(double &Rx, double &Ry, double &Rz) ;


        /**
        \brief  set kitMat3 from the Bryant angle (Angle used by KineoWorks) 
         */
        void bryantToMat3(double Rx, double Ry, double Rz) ;

        /**
        \brief Put a rotation matrix mat3 in a kitMat4 (kineoworks) 
         */
        CkitMat4 mat3ToMat4() ;

        /**
        \brief set a kitMat3 from kitMat4 (kineoWorks) : extract the rotation matrix
         */
        void mat4ToMat3(const CkitMat4 inMat4);

private:

        double attCoefficients[3][3];
};

/**
   @}
*/

#endif
