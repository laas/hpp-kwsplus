
/*
   Copyright (c) JRL CNRS-AIST,
   @author : Alireza Nakhaei (LAAS-CNRS)

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
modification,
   are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
notice,
   this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its
contributors
   may be used to endorse or promote products derived from this software
without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY,
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/



#ifndef KWS_PLUS_ROADMAP_H
#define KWS_PLUS_ROADMAP_H

#warning "deprecated header file. Please include kwsPlus/roadmap/kwsPlusRoadmap.h instead."

/*************************************
INCLUDE
**************************************/
#include <iostream>
#include <fstream>


#include "KineoWorks2/kwsRoadmap.h"
#include "KineoWorks2/kwsConfig.h"
#include "KineoWorks2/kwsNode.h"
#include "KineoWorks2/kwsEdge.h"
#include "KineoWorks2/kwsRoadmapBuilder.h"


#include <time.h>
#include <sys/time.h>
# include <set>


#define RDEBUG(x)
//#define RDEBUG(x) std::cerr << "kwsPlusRoadmap :" << x << std::endl

using std::set;

KIT_PREDEF_CLASS ( CkwsPlusRoadmap );


/*************************************
CLASS
**************************************/

/**
   \addtogroup kwsPlusEnhancedRoadmapManagement
   @{
*/

/**
   \brief Implement a roadmap which enable the user to remove nodes or edges.

   kwsPlusRoadmap (managable roadmap) is herited from kwsRoadmap and enable users to remove some nodes and edges.

*/

class CkwsPlusRoadmap : public CkwsRoadmap
{
	public:
		// Constructor
		CkwsPlusRoadmap() ;

		/**
		\brief Constructor 
		*/
		static CkwsPlusRoadmapShPtr create ( CkwsDeviceShPtr inDevice );

		/**
		\brief Remove a set of nodes and a set of edges from roadmap
		*/
		ktStatus removeNodeEdge ( std::set<CkwsNodeShPtr> inNodeList , std::set<CkwsEdgeShPtr> inEdgeList );

		/**
		\brief Set attRoadmapBuilderWkPtr (roadmap builder) 
		*/
		ktStatus setRoadmapBuilder ( const CkwsRoadmapBuilderShPtr &i_roadmapBuilder );



	private:
		/**
		   \brief A weak pointer to the roadmap builder
		*/
		CkwsRoadmapBuilderWkPtr attRoadmapBuilderWkPtr;


};

/**
   @}
*/
#endif
