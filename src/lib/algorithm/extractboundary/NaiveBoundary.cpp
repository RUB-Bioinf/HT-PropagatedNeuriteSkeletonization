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
 *  \file NaiveBoundary.cpp
 *  \brief Extracts a boundary with a naive algorithm
 *  \author Bastien Durix
 */

#include "NaiveBoundary.h"

boundary::DiscreteBoundary<2>::Ptr algorithm::extractboundary::NaiveBoundary(const shape::DiscreteShape<2>::Ptr dissh)
{
	boundary::DiscreteBoundary<2>::Ptr bnd(new boundary::DiscreteBoundary<2>(dissh->getFrame()));
	
	std::list<std::pair<unsigned int,unsigned int> > list_edg; //list of edges

	//cf: https://en.wikipedia.org/wiki/Marching_squares
	//first step: vertices adjacency computation
	#pragma omp parallel for
	for(unsigned int c = 0; c < dissh->getWidth(); c++)
	{
		#pragma omp parallel for
		for(unsigned int l = 0; l < dissh->getHeight(); l++)
		{
			if(dissh->getContainer()[c + dissh->getWidth() * l])
			{
				unsigned int ind1 = c   + (dissh->getWidth()+1) * l;
				unsigned int ind2 = c+1 + (dissh->getWidth()+1) * l;
				unsigned int ind3 = c+1 + (dissh->getWidth()+1) * (l+1);
				unsigned int ind4 = c   + (dissh->getWidth()+1) * (l+1);

				bool down = false;
				bool up = false;
				bool left = false;
				bool right = false;
				
				if(c != 0)                    if(dissh->getContainer()[(c-1) + dissh->getWidth() * l]) left = true;
				if(l != 0)                    if(dissh->getContainer()[c + dissh->getWidth() * (l-1)]) up = true;
				if(c != dissh->getWidth()-1)  if(dissh->getContainer()[(c+1) + dissh->getWidth() * l]) right = true;
				if(l != dissh->getHeight()-1) if(dissh->getContainer()[c + dissh->getWidth() * (l+1)]) down = true;


				if(!left || !right || !up || !down)
				{
					std::list<std::pair<unsigned int,unsigned int> > local_edg;

					if(!left)
					{
						local_edg.push_back(std::pair<unsigned int,unsigned int>(ind4,ind1));
					}

					if(!up)
					{
						local_edg.push_back(std::pair<unsigned int,unsigned int>(ind1,ind2));
					}

					if(!right)
					{
						local_edg.push_back(std::pair<unsigned int,unsigned int>(ind2,ind3));
					}

					if(!down)
					{
						local_edg.push_back(std::pair<unsigned int,unsigned int>(ind3,ind4));
					}

					#pragma omp critical
					list_edg.insert(list_edg.end(),local_edg.begin(),local_edg.end());
				}
			}
		}
	}

	std::map<unsigned int, unsigned int> map_neigh(list_edg.begin(),list_edg.end()); //map of nodes, linked in direct order

	//second step: link all the vertices on the boundary
	while(map_neigh.size() != 0)
	{
		std::list<Eigen::Vector2d> list_vert;
		
		std::map<unsigned int,unsigned int>::iterator it = map_neigh.begin();
		
		do
		{
			unsigned int l = ((it->first)/(dissh->getWidth()+1));
			unsigned int c = ((it->first)%(dissh->getWidth()+1));
			Eigen::Vector2d vec((double)c,(double)l);
			list_vert.push_back(vec);
			unsigned int val = it->second;
			map_neigh.erase(it);
			it = map_neigh.find(val);
		}while(it != map_neigh.end());
		
		double dist = (*(list_vert.begin()) - *(list_vert.rbegin())).norm();

		if(dist > 1.5)
			throw std::logic_error("Wrong boundary...");

		bnd->addVerticesVector(list_vert);
	}

	return bnd;
	
}
