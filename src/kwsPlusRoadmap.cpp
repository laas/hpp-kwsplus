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
#include "kwsPlusRoadmap.h"

KIT_PREDEF_CLASS ( CkppComponent );


// ==========================================================================


CkwsPlusRoadmap::CkwsPlusRoadmap() :CkwsRoadmap()
{
	RDEBUG ( "Constructor of class CkwsPlusRoadmap." );

}

//create method
CkwsPlusRoadmapShPtr CkwsPlusRoadmap::create ( CkwsDeviceShPtr inDevice )
{

	CkwsPlusRoadmap*  ptr = new CkwsPlusRoadmap();

	CkwsPlusRoadmapShPtr shPtr ( ptr );

	if ( KD_ERROR == ptr->init ( inDevice, shPtr ) )
	{
		shPtr.reset();
	}
	return shPtr;

}
// ==========================================================================
// ==========================================================================


ktStatus CkwsPlusRoadmap::removeNodeEdge ( std::set<CkwsNodeShPtr> inNodeList , std::set<CkwsEdgeShPtr> inEdgeList )
{

	CkwsPlusRoadmapShPtr roadmapB = CkwsPlusRoadmap::create ( device() ) ;

	std::map<CkwsNodeShPtr,CkwsNodeShPtr > mapOld2New;
	std::map<CkwsNodeShPtr,CkwsNodeShPtr > :: iterator it;

	int nbOfNodeInRmA= countNodes () ;

	RDEBUG ( "NUMBER OF NODE in RoadmapA at the begining=  " << nbOfNodeInRmA );
	RDEBUG ( "NUMBER OF NODE in RoadmapA Which should be removed=  " << inNodeList.size() );
	RDEBUG ( "NUMBER OF EDGE in RoadmapA Which should be removed=  " << inEdgeList.size() );

//Adding valid nodes to the new roadmap roadmapB

	int i=0;
	int temp=0;
	while ( ( nbOfNodeInRmA>0 ) && ( i <nbOfNodeInRmA ) )
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

	RDEBUG ( "Add Valid Node to roadmapB was done - number of added node = " << temp );

//add valid edge to the new rodmap(rodmapB)
	temp=0;
	for ( it = mapOld2New.begin(); it!=mapOld2New.end(); it++ )
	{
		int nbOfOutEdges=it->first->countOutEdges();
		i=0;
		while ( ( nbOfOutEdges>0 ) && ( i< nbOfOutEdges ) )
		{

			CkwsEdgeShPtr outEdge=it->first->outEdge ( i ) ;

			if ( inEdgeList.find ( outEdge ) == inEdgeList.end() )
			{

				CkwsNodeShPtr endNodeOutEdge= outEdge->endNode();
				if ( !endNodeOutEdge )
					cout << "Error : endNodeOutEdge"<<endl;

				if ( inNodeList.find ( endNodeOutEdge ) == inNodeList.end() )
				{


					CkwsDirectPathConstShPtr directPathOld=outEdge->directPath();
					if ( !directPathOld )
						cout << "Error : directPathOld"<<endl;

					
					CkwsEdgeShPtr newEdge=CkwsEdge::create ( directPathOld );
					if ( !newEdge )
						cout << "Error : newEdge"<<endl;

					roadmapB->addEdge ( mapOld2New[outEdge->startNode() ],mapOld2New[outEdge->endNode() ],newEdge );
					temp++;
				}
			}

			i++;
		}

	}
	RDEBUG ( "Add Valid Edge to roadmapB was done - number of added edge = " << temp );

//Getting the information of the  start nodes and goal nods in the RoadmapBuilder (replacing old nodes with new nods)
	std::set <CkwsNodeShPtr> startNodeList;
	std::set <CkwsNodeShPtr> goalNodeList;
	CkwsRoadmapBuilder::TNodeList::const_iterator startEndListIt;
	startEndListIt=attRoadmapBuilderWkPtr.lock()->beginStartNodes();

	startEndListIt = attRoadmapBuilderWkPtr.lock()->endStartNodes();


	for ( startEndListIt=attRoadmapBuilderWkPtr.lock()->beginStartNodes();startEndListIt != attRoadmapBuilderWkPtr.lock()->endStartNodes();startEndListIt++ )
	{

		if ( inNodeList.find ( *startEndListIt ) == inNodeList.end() )
		{
			startNodeList.insert ( *startEndListIt );

		}
	}

	for ( startEndListIt=attRoadmapBuilderWkPtr.lock()->beginGoalNodes();startEndListIt != attRoadmapBuilderWkPtr.lock()->endGoalNodes();startEndListIt++ )
	{

		if ( inNodeList.find ( *startEndListIt ) == inNodeList.end() )
		{
			goalNodeList.insert ( *startEndListIt );

		}
	}

	attRoadmapBuilderWkPtr.lock()->resetStartNodes ();

	attRoadmapBuilderWkPtr.lock()->resetGoalNodes ();

	RDEBUG ( "informatin of start and end nodes were get - Start Nods= " << startNodeList.size() << " , goal Nods= "<< goalNodeList.size() );


//erasing the old road map (roadmapA)
	clear();
	RDEBUG ( "roadmap A cleared " );

// copy roadmapB in the roadmap A which is empty now
	int nbOfNodeInRmB= roadmapB->countNodes () ;

	RDEBUG ( "NUMBER OF NODE in RoadmapB at the begining =  " << nbOfNodeInRmB );
	RDEBUG ( "NUMBER OF NODE in RoadmapA after clearing =  " << countNodes () );

	i=0;
	std::map<CkwsNodeShPtr,CkwsNodeShPtr > mapNew2Old;
//Copy Nodes
	while ( ( nbOfNodeInRmB>0 ) && ( i <nbOfNodeInRmB ) )
	{
		CkwsNodeShPtr inNode=roadmapB->node ( i ) ;
		CkwsNodeShPtr newNode= CkwsNode::create ( inNode->config() );
		mapNew2Old[inNode]=newNode;
		addNode ( newNode );
		i++;
	}
//Copy Edges

	for ( it = mapNew2Old.begin(); it!=mapNew2Old.end(); it++ )
	{
		int nbOfOutEdges=it->first->countOutEdges();
		i=0;
		while ( ( nbOfOutEdges>0 ) && ( i< nbOfOutEdges ) )
		{
			CkwsEdgeShPtr outEdge=it->first->outEdge ( i ) ;

			CkwsEdgeShPtr newEdge=CkwsEdge::create ( outEdge->directPath() );
			addEdge ( mapNew2Old[outEdge->startNode() ],mapNew2Old[outEdge->endNode() ],newEdge );

			i++;
		}

	}


	RDEBUG ( "NUMBER OF NODE in RoadmapA after copying B inside A =  " << countNodes () );

//setting the start nodes and goal nods in the RoadmapBuilder
	std::set <CkwsNodeShPtr>:: iterator nodeIt;
	for ( nodeIt=startNodeList.begin();nodeIt != startNodeList.end();nodeIt++ )
	{
		attRoadmapBuilderWkPtr.lock()->addStartNode ( *nodeIt );
	}

	for ( nodeIt=goalNodeList.begin();nodeIt != goalNodeList.end();nodeIt++ )
	{
		attRoadmapBuilderWkPtr.lock()->addGoalNode ( *nodeIt );
	}


	return KD_OK;
}

ktStatus CkwsPlusRoadmap::setRoadmapBuilder ( const CkwsRoadmapBuilderShPtr &i_roadmapBuilder )
{
	attRoadmapBuilderWkPtr=i_roadmapBuilder;
	return KD_OK;
}


