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




/*****************************************
 INCLUDES
*******************************************/

#include <iostream>
#include <hpp/kwsplus/roadmap/roadmap.hh>

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cerr << "CkwsPlusRoadmap:" << x << std::endl
#define ODEBUG1(x) std::cerr << "CkwsPlusRoadmap:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "CkwsPlusRoadmap:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

unsigned int CkwsPlusRoadmap::nbObject = 0;

CkwsPlusRoadmap::CkwsPlusRoadmap() :CkwsRoadmap()
{
	nbObject++;
	ODEBUG2 ( "Constructor of class CkwsPlusRoadmap. Number of objects: "<< nbObject );
}
// ==========================================================================
CkwsPlusRoadmapShPtr CkwsPlusRoadmap::create ( CkwsDeviceShPtr inDevice )
{

	CkwsPlusRoadmap*  ptr = new CkwsPlusRoadmap();

	CkwsPlusRoadmapShPtr shPtr ( ptr );

	if ( KD_ERROR == ptr->init ( inDevice->configSpace(), shPtr ) )
	{
		shPtr.reset();
	}
	return shPtr;

}
// ==========================================================================
ktStatus CkwsPlusRoadmap::removeNodeEdge ( std::set<CkwsNodeShPtr> inNodeList , std::set<CkwsEdgeShPtr> inEdgeList )
{

	CkwsPlusRoadmapShPtr roadmapB = CkwsPlusRoadmap::create ( device() ) ;
	std::map<CkwsNodeShPtr,CkwsNodeShPtr > mapOld2New;
	std::map<CkwsNodeShPtr,CkwsNodeShPtr > :: iterator it;

	int nbOfNodeInRmA= countNodes () ;
	ODEBUG2 ( "removeNodeEdge:NUMBER OF NODE in RoadmapA at the begining=  " << countNodes() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF EDGE in RoadmapA at the begining=  " << countEdges() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF ConnectedComponent in RoadmapA at the begining=  " << countConnectedComponents() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF START NODE in RoadmapA at the begining =  " << attRoadmapBuilderWkPtr.lock()->countStartNodes() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF GOAL NODE in RoadmapA at the begining =  " << attRoadmapBuilderWkPtr.lock()->countGoalNodes() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF NODE in RoadmapA Which should be removed=  " << inNodeList.size() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF EDGE in RoadmapA Which should be removed=  " << inEdgeList.size() );

//Adding valid nodes to the new roadmap roadmapB

	int i=0;
	int temp=0;
	while ( i<nbOfNodeInRmA )
	{
		CkwsNodeShPtr inNode=node ( i ) ;

		if ( inNodeList.find ( inNode ) == inNodeList.end() )
		{
			CkwsNodeShPtr newNode= CkwsNode::create ( inNode->config() );
			mapOld2New[inNode]=newNode;
			roadmapB->addNode ( newNode );
			temp++;
		}
		i++;
	}
//add valid edge to the new rodmap(rodmapB)
	for ( it = mapOld2New.begin(); it!=mapOld2New.end(); it++ )
	{
		for ( unsigned int i=1;i<= it->first->countOutEdges() ;i++ )
		{
			CkwsEdgeShPtr outEdge=it->first->outEdge ( i-1 ) ;

			if ( inEdgeList.find ( outEdge ) == inEdgeList.end() )
			{
				CkwsNodeShPtr endNodeOutEdge= outEdge->endNode();
				if ( !endNodeOutEdge )
				{
					ODEBUG2 ( "removeNodeEdge:Error : endNodeOutEdge" );
					return KD_ERROR;
				}
				if ( inNodeList.find ( endNodeOutEdge ) == inNodeList.end() )
				{
					CkwsDirectPathConstShPtr directPathOld=outEdge->directPath();
					if ( !directPathOld )
					{
						ODEBUG2 ( "removeNodeEdge:Error : directPathOld" );
						return KD_ERROR;
					}
					CkwsEdgeShPtr newEdge=CkwsEdge::create ( directPathOld );
					if ( !newEdge )
					{
						ODEBUG2 ( "removeNodeEdge:Error : newEdge" );
						return KD_ERROR;
					}
					roadmapB->addEdge ( mapOld2New[outEdge->startNode() ],mapOld2New[outEdge->endNode() ],newEdge );
				}
			}
		}
	}

//Getting the information of the  start nodes and goal nods in the RoadmapBuilder (replacing old nodes with new nods)
	std::vector <CkwsNodeShPtr> startNodeVector;
	std::vector <CkwsNodeShPtr> goalNodeVector;
	CkwsRoadmapBuilder::TNodeList::const_iterator startEndListIt;

	for ( startEndListIt=attRoadmapBuilderWkPtr.lock()->beginStartNodes();startEndListIt != attRoadmapBuilderWkPtr.lock()->endStartNodes(); startEndListIt++ )
	{
		startNodeVector.push_back ( mapOld2New[*startEndListIt] );
	}

	for ( startEndListIt=attRoadmapBuilderWkPtr.lock()->beginGoalNodes();startEndListIt != attRoadmapBuilderWkPtr.lock()->endGoalNodes(); startEndListIt++ )
	{
		goalNodeVector.push_back ( mapOld2New[*startEndListIt] );
	}

	attRoadmapBuilderWkPtr.lock()->resetStartNodes ();
	attRoadmapBuilderWkPtr.lock()->resetGoalNodes ();


//erasing the old road map (roadmapA)
	clear();
// copy roadmapB in the roadmap A which is empty now
	std::map<CkwsNodeShPtr,CkwsNodeShPtr > mapNew2Old;
	for ( unsigned int i=0; i<roadmapB->countNodes ();i++ )
	{
		CkwsNodeShPtr inNode=roadmapB->node ( i ) ;
		CkwsNodeShPtr newNode= CkwsNode::create ( inNode->config() );
		if ( newNode )
			mapNew2Old[roadmapB->node ( i ) ]=newNode;
		if ( addNode ( newNode ) !=KD_OK )
		  {
			ODEBUG1 ( "removeNodeEdge: ERROR: addNode failed" );
		  }
	}

//Copy Edges
	for ( it = mapNew2Old.begin(); it!=mapNew2Old.end(); it++ )
	{
		for ( unsigned int i=1;i<= it->first->countOutEdges() ;i++ )
		{
			CkwsEdgeShPtr outEdge=it->first->outEdge ( i-1 ) ;
			CkwsEdgeShPtr newEdge=CkwsEdge::create ( outEdge->directPath() );
			if ( addEdge ( mapNew2Old[outEdge->startNode() ],mapNew2Old[outEdge->endNode() ],newEdge ) !=KD_OK )
			  {
				ODEBUG1 ( "removeNodeEdge: ERROR: addEdge failed" );
			  }
		}
	}

	//setting the start nodes and goal nods in the RoadmapBuilder
	for ( unsigned int i=0; i<startNodeVector.size();i++ )
	{
		if ( inNodeList.find ( startNodeVector[i] ) == inNodeList.end() )
		{
			if ( attRoadmapBuilderWkPtr.lock()->addStartNode ( mapNew2Old[startNodeVector[i] ] ) !=KD_OK )
			{
				ODEBUG1 ( "removeNodeEdge: ERROR: adding init nodes failed" );
				return KD_ERROR;
			}
		}
	}

	for ( unsigned int i=0; i<goalNodeVector.size();i++ )
	{
		if ( inNodeList.find ( goalNodeVector[i] ) == inNodeList.end() )
		{
			if ( attRoadmapBuilderWkPtr.lock()->addGoalNode ( mapNew2Old[goalNodeVector[i]] ) !=KD_OK )
			{
				ODEBUG1 ( "removeNodeEdge: ERROR: adding goal nodes failed" );
				return KD_ERROR;
			}
		}
	}

	ODEBUG2 ( "removeNodeEdge:NUMBER OF NODE in RoadmapA at END=  " << countNodes() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF EDGE in RoadmapA at END=  " << countEdges() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF ConnectedComponent in RoadmapA at END=  " << countConnectedComponents() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF START NODE in RoadmapA at END =  " << attRoadmapBuilderWkPtr.lock()->countStartNodes() );
	ODEBUG2 ( "removeNodeEdge:NUMBER OF GOAL NODE in RoadmapA at END =  " << attRoadmapBuilderWkPtr.lock()->countGoalNodes() );
	return KD_OK;
}
// ==========================================================================
ktStatus CkwsPlusRoadmap::setRoadmapBuilder ( const CkwsRoadmapBuilderShPtr &i_roadmapBuilder )
{
	attRoadmapBuilderWkPtr=i_roadmapBuilder;
	return KD_OK;
}


