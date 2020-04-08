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
 *  @file MovingCenter.cpp
 *  @brief Defines object containing informations about moving centers
 *  @author Bastien Durix
 */

#include "MovingCenter.h"

algorithm::skeletonization::propagation::MovingCenter::MovingCenter() {}

algorithm::skeletonization::propagation::MovingCenter::MovingCenter(const Eigen::Vector2d &center) : m_center(center) {}


const Eigen::Vector2d& algorithm::skeletonization::propagation::MovingCenter::getCenter() const
{
	return m_center;
}

const double& algorithm::skeletonization::propagation::MovingCenter::getRadius() const
{
	return m_rad;
}

const std::vector<unsigned int>& algorithm::skeletonization::propagation::MovingCenter::getClosestInds() const
{
	return m_closestOrdered;
}

const std::vector<bool>& algorithm::skeletonization::propagation::MovingCenter::getOpen()
{
	return m_openDir;
}

const std::list<unsigned int>& algorithm::skeletonization::propagation::MovingCenter::getToErase() const
{
	return m_toErase;
}


void algorithm::skeletonization::propagation::MovingCenter::computeContactData(const OptiBnd &optiBnd, double epsilon)
{
	// computation of the closest indices
	std::list<unsigned int> closestInds;
	double radMin = closestInd(optiBnd,m_center,closestInds);
	
	// sort the closest indices around the center
	angOrder(optiBnd,m_center,closestInds,m_closestOrdered);
	// computation of the contact sets
	contactSets(optiBnd,m_center,radMin+epsilon,m_closestOrdered,m_openDir,m_toErase);
	
	// computation of the radius
	if(epsilon != 0.0)
	{
		double dmax = radMin;
		for(auto indErase : m_toErase)
		{
			Eigen::Vector2d pt = optiBnd.find(indErase)->second.coords;
			double dist = (m_center - pt).norm();
			if(dist > dmax)
				dmax = dist;
		}
		m_rad = (radMin + dmax)/2.0;
	}
	else
	{
		m_rad = radMin;
	}
}

bool algorithm::skeletonization::propagation::MovingCenter::propagate(const OptiBnd &optiBnd,
																	  unsigned int dir,
																	  double epsilon,
																	  MovingCenter &mov) const
{
	unsigned int indBeg = dir;
	unsigned int indEnd = (dir+1)%m_closestOrdered.size();

	unsigned int indP1 = m_closestOrdered[indBeg];
	unsigned int indP2 = m_closestOrdered[indEnd];
	
	Eigen::Vector2d P1 = optiBnd.find(indP1)->second.coords;
	Eigen::Vector2d P2 = optiBnd.find(indP2)->second.coords;
	
	// research set: bissector of [P1 P2]
	Eigen::Vector2d mid = (P1 + P2)*0.5;
	double sqMid = (P1 - mid).squaredNorm();
	
	Eigen::Vector2d nor(P2.y() - P1.y(),P1.x() - P2.x());
	nor.normalize();
	
	// searching for the next point on a subpart of the bissector
	unsigned int indP3;
	Eigen::Vector2d P3;
	double lambdaMin = 0.0;
	bool found = false;
	for(auto curPt : optiBnd)
	{
		if(std::find(m_closestOrdered.begin(),m_closestOrdered.end(),curPt.first) == m_closestOrdered.end())
		{
			Eigen::Vector2d pt = curPt.second.coords;
			if((mid - pt).dot(nor) != 0.0)
			{
				double lambda = (sqMid - (pt - mid).squaredNorm() - 2.0*(mid - pt).dot(m_center - mid))/(2.0*(mid - pt).dot(nor));
				
				if(lambda > 0.0)
				{
					if(!found || lambda < lambdaMin)
					{
						lambdaMin = lambda;
						indP3 = curPt.first;
						P3 = curPt.second.coords;
						found = true;
					}
				}
			}
		}
	}
	
	if(!found) // no point found: bug?
	{
		return false;
	}
	
	Eigen::Vector2d center = circleCenter(P1,P2,P3);
	bool converged = true;
	
	if(converged)
	{
		mov = MovingCenter(center);
		mov.computeContactData(optiBnd,epsilon);
		
		// direction selection
		unsigned int dirmov = 0;
		bool found = false;
		for(unsigned int i = 0; i < mov.m_closestOrdered.size() && !found; i++)
		{
			if(neighbors(mov,i,*this,dir))
			{
				dirmov = i;
				found = true;
			}
		}
		if(found)
			mov.m_openDir[dirmov] = false;
		else
			converged = false;
	}
	
	return converged;
}

bool algorithm::skeletonization::propagation::MovingCenter::neighbors(const MovingCenter &mov1, unsigned int dir1, const MovingCenter &mov2, unsigned int dir2)
{
	unsigned int indBeg1 = dir1;
	unsigned int indEnd1 = (dir1 + 1)%mov1.m_closestOrdered.size();

	unsigned int indBeg2 = dir2;
	unsigned int indEnd2 = (dir2 + 1)%mov2.m_closestOrdered.size();

	bool eq1 = (mov1.m_closestOrdered[indBeg1] == mov2.m_closestOrdered[indEnd2]);
	bool eq2 = (mov2.m_closestOrdered[indBeg2] == mov1.m_closestOrdered[indEnd1]);
	
	return (eq1 && eq2);
}
