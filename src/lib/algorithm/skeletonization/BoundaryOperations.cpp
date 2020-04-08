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
 *  @file BoundaryOperations.cpp
 *  @brief Defines the set of operations on the boudary
 *  @author Bastien Durix
 */

#include "BoundaryOperations.h"

double algorithm::skeletonization::propagation::closestInd(const OptiBnd &optiBnd,
														   const Eigen::Vector2d &center,
														   std::list<unsigned int> &closestInds)
{
	// computation of the minimal distance between points of the boundary and C
	double radMin = -1.0;
	for(auto &curPt : optiBnd)
	{
		double dist = (curPt.second.coords - center).norm();
		
		if(dist < radMin || radMin == -1.0)
		{
			radMin = dist;
		}
	}
	
	// search for the points at the minimal distance
	closestInds.clear();
	for(auto &curPt : optiBnd)
	{
		float dist = (curPt.second.coords - center).norm();

		if(dist == (float)radMin) // trick, the machine precision is not 0
		{
			closestInds.push_back(curPt.first);
		}
	}
	
	return radMin;
}

Eigen::Vector2d algorithm::skeletonization::propagation::circleCenter(const Eigen::Vector2d &P1, const Eigen::Vector2d &P2, const Eigen::Vector2d &P3)
{
	// I'm sure there is a justified equation behind those lines, but I don't remember it...
	Eigen::Matrix<double,3,2> mat;
	mat.block<1,2>(0,0) = (P2 - P1).transpose();
	mat.block<1,2>(1,0) = (P3 - P2).transpose();
	mat.block<1,2>(2,0) = (P1 - P3).transpose();
	
	Eigen::Vector3d vec;
	vec.x() = 0.5*(P2 - P1).squaredNorm() + (P2 - P1).dot(P1);
	vec.y() = 0.5*(P3 - P2).squaredNorm() + (P3 - P2).dot(P2);
	vec.z() = 0.5*(P1 - P3).squaredNorm() + (P1 - P3).dot(P3);
	
	return mat.colPivHouseholderQr().solve(vec);
}

Eigen::Vector2d algorithm::skeletonization::propagation::firstPoint(const OptiBnd &optiBnd)
{
	Eigen::Vector2d corner = optiBnd.begin()->second.coords;
	
	// bounding box corner computation
	for(auto &curPt : optiBnd)
	{
		const Eigen::Vector2d &coords = curPt.second.coords;
		if(corner.x() > coords.x())
			corner.x() = coords.x();
		if(corner.y() > coords.y())
			corner.y() = coords.y();
	}
	// little motion to be sure to be outside
	corner.x() -= 1;
	corner.y() -= 1;
	

	// search for the closest point to the corner of the bounding box
	double distmin = -1.0;
	unsigned int indClosest;
	Eigen::Vector2d coordsClosest;
	for(auto &curPt : optiBnd)
	{
		const Eigen::Vector2d &coords = curPt.second.coords;
		double dist = (corner - coords).norm();
		if(dist < distmin || distmin == -1.0)
		{
			distmin = dist;
			indClosest = curPt.first;
			coordsClosest = curPt.second.coords;
		}
	}
	
	// moving the point such that it stays inside the shape
	// -> distance computation
	double distanceStayingInside = -1.0;
	for(auto &curPt : optiBnd)
	{
		if(curPt.first != indClosest)
		{
			const Eigen::Vector2d &coords = curPt.second.coords;
			double distinter = (coordsClosest - coords).norm();
			if(distinter < distanceStayingInside || distanceStayingInside == -1.0)
			{
				distanceStayingInside = distinter;
			}
		}
	}
	// -> moving the point
	Eigen::Vector2d center = coordsClosest + 0.1*distanceStayingInside*((coordsClosest - corner).normalized());
	
	return center;
}

void algorithm::skeletonization::propagation::firstVoroPoint(const OptiBnd &optiBnd,
															 Eigen::Vector2d &center,
															 std::list<unsigned int> &closestInds)
{
	double radius = closestInd(optiBnd,center,closestInds);
	
	// if there is only 1 closest neighbor, move the point to have two closest neighbors
	if(closestInds.size() == 1)
	{
		unsigned int ind1 = *(closestInds.begin());
		Eigen::Vector2d P1 = optiBnd.find(ind1)->second.coords;
		
		// direction in which move the point
		Eigen::Vector2d dir = (center - P1).normalized();
		
		double distMin = -1.0;
		for(auto curPt : optiBnd)
		{
			Eigen::Vector2d P2 = curPt.second.coords;
			if(ind1 != curPt.first)
			{
				if((P2 - P1).dot(dir) > 0.0)
				{
					double dist = 0.5*(P2 - P1).norm()/(dir.dot((P2 - P1).normalized()));
					if(dist < distMin || distMin == -1.0)
					{
						distMin = dist;
						center = P1 + dist*dir;
					}
				}
			}
		}
		radius = closestInd(optiBnd,center,closestInds);
	}
	
	if(closestInds.size() < 2)
		throw std::logic_error("firstVoroPoint : not enough points after the first step");
	
	if(closestInds.size() == 2)
	{
		std::list<unsigned int>::iterator it = closestInds.begin();
		unsigned int ind1 = *it;
		it++;
		unsigned int ind2 = *it;
		
		Eigen::Vector2d P1 = optiBnd.find(ind1)->second.coords;
		Eigen::Vector2d P2 = optiBnd.find(ind2)->second.coords;
		
		// direction of the research
		Eigen::Vector2d dir(P2.y() - P1.y(),P1.x() - P2.x());
		dir.normalize();

		// two possible points to compute (+/-dir)
		double radMin1 = -1.0;
		double ind3_1;
		double radMin2 = -1.0;
		double ind3_2;
		Eigen::Vector2d C1, C2;

		for(auto curPt : optiBnd)
		{
			Eigen::Vector2d P3 = curPt.second.coords;
			double scal = (P3 - P1).dot(dir);
			// center in the first direction
			if(scal > 0.0 && curPt.first != ind1 && curPt.first != ind2)
			{
				Eigen::Vector2d ctr = circleCenter(P1,P2,P3);
				double rad = (ctr - P3).norm();
				if(rad < radMin1 || radMin1 == -1.0)
				{
					radMin1 = rad;
					ind3_1 = curPt.first;
					C1 = ctr;
				}
			}
			// center in the second direction
			if(scal < 0.0 && curPt.first != ind1 && curPt.first != ind2)
			{
				Eigen::Vector2d ctr = circleCenter(P1,P2,P3);
				double rad = (ctr - P3).norm();
				if(rad < radMin2 || radMin2 == -1.0)
				{
					radMin2 = rad;
					ind3_2 = curPt.first;
					C2 = ctr;
				}
			}
		}
		
		//selection of the center
		if(optiBnd.find(ind1)->second.prev == ind2 || optiBnd.find(ind1)->second.next == ind2) // are neighbors
		{
			if(dir.dot(P1 - center)*dir.dot(P1 - C1) > dir.dot(P1 - center)*dir.dot(P1 - C2))
			{
				center = C1;
			}
			else
			{
				center = C2;
			}
		}
		else
		{
			if(radMin1 != -1.0 && radMin2 != -1.0)
			{
				if(radMin1 > radMin2)
					center = C1;
				else
					center = C2;
			}
			else if(radMin1 != -1.0)
			{
				center = C1;
			}
			else if(radMin2 != -1.0)
			{
				center = C2;
			}
			else
			{
				throw std::logic_error("arf");
			}
		}
		radius = closestInd(optiBnd,center,closestInds);
	}
	
	if(closestInds.size() < 3)
		throw std::logic_error("firstVoroPoint : not enough points after the second step");
}

void algorithm::skeletonization::propagation::angOrder(const OptiBnd &optiBnd,
													  const Eigen::Vector2d &center,
													  const std::list<unsigned int> &closestInds,
													  std::vector<unsigned int> &closestIndsOrdered)
{
	// computation of the angle associated to each point
	std::list<std::pair<unsigned int,double> > lisIndAng;
	for(auto ind : closestInds)
	{
		Eigen::Vector2d vec = optiBnd.find(ind)->second.coords - center;
		double ang = atan2(vec.y(),vec.x());
		lisIndAng.push_back(std::make_pair(ind,ang));
	}
	
	// sort
	lisIndAng.sort([](const std::pair<unsigned int,double> &lp1, const std::pair<unsigned int,double> &lp2)
			{
				return lp1.second < lp2.second;
			});
	
	// put the indices in a vector
	closestIndsOrdered.resize(0);
	closestIndsOrdered.reserve(closestInds.size());
	for(auto indang : lisIndAng)
	{
		closestIndsOrdered.push_back(indang.first);
	}
}

bool algorithm::skeletonization::propagation::contactSet(const OptiBnd &optiBnd,
													 	 const Eigen::Vector2d &center,
													 	 double distMax,
													 	 unsigned int indBeg,
													 	 unsigned int indEnd,
													 	 std::list<unsigned int> &toErase)
{
	auto itBeg = optiBnd.find(indBeg);
	auto itCur = optiBnd.find(itBeg->second.next);
	toErase.clear();

	bool fini = false;
	bool open = true;

	while(!fini)
	{
		if(itCur == optiBnd.end()) // no neighbor: bug?
		{
			fini = true;
		}
		else if(itCur->first == indEnd) // reached the last point
		{
			open = false;
			fini = true;
		}
		else if(itCur->first == indBeg) // loop has been done until going back to the first point
		{
			open = true;
			fini = true;
		}
		else
		{
			double dist = (center - itCur->second.coords).norm();
			if(dist > distMax)
			{
				fini = true;
			}
			else
			{
				toErase.push_back(itCur->first);
				itCur = optiBnd.find(itCur->second.next);
			}
		}
	}
	
	if(open)
	{
		toErase.clear();
	}
	
	return open;
}

void algorithm::skeletonization::propagation::contactSets(const OptiBnd &optiBnd,
														  const Eigen::Vector2d &center,
														  const double &distMax,
														  const std::vector<unsigned int> &closestIndsOrdered,
														  std::vector<bool> &open,
														  std::list<unsigned int> &toErase)
{
	open.resize(closestIndsOrdered.size());
	for(unsigned int i = 0; i < closestIndsOrdered.size(); i++)
	{
		std::list<unsigned int> toErase_i;
		open[i] = contactSet(optiBnd,center,distMax,closestIndsOrdered[i],closestIndsOrdered[(i+1)%closestIndsOrdered.size()],toErase_i);
		toErase.insert(toErase.end(),toErase_i.begin(),toErase_i.end());
	}
	
}

void algorithm::skeletonization::propagation::createOptiBnd(const boundary::DiscreteBoundary<2>::Ptr &disBnd, OptiBnd &optiBnd, OptiUsedBnd &optiUsedBnd)
{
	for(unsigned int i = 0; i < disBnd->getNbVertices(); i++)
	{
		OptiPt pt{disBnd->getCoordinates(i),disBnd->getPrev(i),disBnd->getNext(i)}; // be sure it's in trigonometric order
		optiBnd.insert(std::make_pair(i,pt));
		optiUsedBnd.insert(std::make_pair(i,false));
	}
}

void algorithm::skeletonization::propagation::cleanOptiBnd(OptiBnd &optiBnd, OptiUsedBnd &optiUsedBnd, const std::list<unsigned int> &toErase, const std::vector<unsigned int> &closestInds)
{
	for(auto indErase : toErase)
	{
		optiBnd.erase(indErase);
		optiUsedBnd[indErase] = true;
	}

	for(auto indClosest : closestInds)
	{
		optiUsedBnd[indClosest] = true;
	}
}
