/*
Copyright (c) 2016 Bastien Durix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 *  @file SpherePropagation2D.cpp
 *  @brief Defines functions to compute 2d skeleton with sphere propagation algorithm
 *  @author Bastien Durix
 */

#include "SpherePropagation2D.h"
#include "BoundaryOperations.h"
#include "MovingCenter.h"
#include <time.h>

#include <iostream>
#include <fstream>

#include <unordered_map>

struct PropagData
{
	unsigned int skelInd; // corresponding indice in the skeleton
	algorithm::skeletonization::propagation::MovingCenter mov; // circle structure
	unsigned int dir; // next direction to take
};

skeleton::GraphSkel2d::Ptr algorithm::skeletonization::propagation::SpherePropagation2D(const boundary::DiscreteBoundary<2>::Ptr disbnd,
																						const OptionsSphProp &options)
{
	skeleton::GraphSkel2d::Ptr skel(new skeleton::GraphSkel2d(skeleton::model::Classic<2>()));
	
	// creation of the optimized boundary structure
	OptiBnd optiBnd;
	OptiUsedBnd optiUsedBnd;
	createOptiBnd(disbnd,optiBnd,optiUsedBnd);
	
	// estimation of a point inside the shape
	Eigen::Vector2d firstPt = firstPoint(optiBnd);
	
	// estimation of the first Voronoi point
	std::list<unsigned int> lclosest;
	algorithm::skeletonization::propagation::firstVoroPoint(optiBnd,firstPt,lclosest);
	
	// first moving center instance
	algorithm::skeletonization::propagation::MovingCenter mov(firstPt);
	mov.computeContactData(optiBnd,options.epsilon);
	
	// search for a local maximum in the Voronoi diagram to be (quite) sure that we have a skeletal point (can actually be improved...)
	bool maximised = true;
	do
	{
		algorithm::skeletonization::propagation::MovingCenter movNext;
		maximised = false;
		for(unsigned int i = 0; i < mov.getOpen().size(); i++)
		{
			if(mov.getOpen()[i])
			{
				algorithm::skeletonization::propagation::MovingCenter movNextcur;
				mov.propagate(optiBnd,i,options.epsilon,movNextcur);
				
				if(!maximised)
				{
					if(movNextcur.getRadius() > mov.getRadius())
					{
						movNext = movNextcur;
						maximised = true;
					}
				}
				else
				{
					if(movNextcur.getRadius() > movNext.getRadius())
					{
						movNext = movNextcur;
					}
				}
			}
		}
		if(maximised)
			mov = movNext;
	}while(maximised);
	
	// first moving center
	mov = algorithm::skeletonization::propagation::MovingCenter(mov.getCenter());
	mov.computeContactData(optiBnd,options.epsilon);
	
	// add it to the skeleton
	unsigned int ind = skel->addNode(Eigen::Vector3d(mov.getCenter().x(),mov.getCenter().y(),mov.getRadius()));
	
	// clean boundary points
	cleanOptiBnd(optiBnd,optiUsedBnd,mov.getToErase(),mov.getClosestInds());
	
	std::list<PropagData> lctr;
	for(unsigned int i = 0; i < mov.getOpen().size(); i++)
	{
		if(mov.getOpen()[i])
		{
			PropagData curData{ind,mov,i};
			lctr.push_back(curData);
		}
	}
	
	unsigned int cpt = options.iter_max;
	
	if(lctr.size() != 0)
	{
		do
		{
			PropagData curData = *(lctr.begin());
			lctr.pop_front();
			
			algorithm::skeletonization::propagation::MovingCenter movNext;
			if(curData.mov.propagate(optiBnd,curData.dir,options.epsilon,movNext))
			{
				cleanOptiBnd(optiBnd,optiUsedBnd,movNext.getToErase(),movNext.getClosestInds());

				unsigned int indcur = skel->addNode(Eigen::Vector3d(movNext.getCenter().x(),movNext.getCenter().y(),movNext.getRadius()));
				skel->addEdge(indcur,curData.skelInd);
				
				for(unsigned int i = 0; i < movNext.getOpen().size(); i++)
				{
					if(movNext.getOpen()[i])
					{
						bool skip = false;
						for(std::list<PropagData>::iterator it = lctr.begin(); it != lctr.end() && !skip; it++)
						{
							if(algorithm::skeletonization::propagation::MovingCenter::neighbors(movNext,i,it->mov,it->dir))
							{
								skel->addEdge(indcur,it->skelInd);
								it = lctr.erase(it);
								skip = true;
							}
						}
						
						if(!skip)
						{
							PropagData nextData{indcur,movNext,i};
							lctr.push_back(nextData);
						}
					}
				}
			}
			else
			{
				cpt = 1;
			}
			
			cpt--;
		}while(!lctr.empty() && cpt != 0);
	}
    int i = 0;
	for(auto checkUsed : optiUsedBnd)
	{

	    if(!checkUsed.second)
            std::cout << "Count of non failures:" << i << std::endl;
			//continue;
	    //throw std::logic_error("Error while computing the skeleton");
	    i++;
	}
	return skel;
}
