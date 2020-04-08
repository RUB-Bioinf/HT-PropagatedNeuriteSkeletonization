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
 *  \file MovingCenter.h
 *  @brief Defines object containing informations about moving centers
 *  \author Bastien Durix
 */

#ifndef _MOVINGCENTER_H_
#define _MOVINGCENTER_H_

#include "BoundaryOperations.h"

/**
 *  @brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  @brief skeletonization algorithms
	 */
	namespace skeletonization
	{
		/**
		 * @brief Skeletonization by propagation algorithm
		 */
		namespace propagation
		{
			/**
			 *  @brief Contains properties about moving centers
			 */
			class MovingCenter
			{
				protected:
					/**
					 *  @brief Position
					 */
					Eigen::Vector2d m_center;

					/**
					 *  @brief Radius of the circle
					 */
					double m_rad;
					
					/**
					 * @brief Closest points to the center, ordered
					 */
					std::vector<unsigned int> m_closestOrdered;
					
					/**
					 * @brief Open propagation directions
					 */
					std::vector<bool> m_openDir;
					
					/**
					 * @brief Set of points to erase
					 */
					std::list<unsigned int> m_toErase;
					
				public:
					/**
					 *  @brief Default constructor
					 */
					MovingCenter();
					
					/**
					 *  @brief Constructor
					 *  @param center Position of the center
					 */
					MovingCenter(const Eigen::Vector2d &center);
					
					
					/**
					 * @brief  Center getter
					 * @return Center
					 */
					const Eigen::Vector2d& getCenter() const;
					
					/**
					 * @brief  Radius getter
					 * @return Radius
					 */
					const double& getRadius() const;
					
					/**
					 * @brief  Gets the indices of the closest points
					 * @return Indices of the closest points
					 */
					const std::vector<unsigned int>& getClosestInds() const;
					
					/**
					 * @brief  Get the open states of the directions
					 * @return Open states
					 */
					const std::vector<bool>& getOpen();
					
					/**
					 * @brief  Gets the indices of the erased points
					 * @return Indices of the erased points
					 */
					const std::list<unsigned int>& getToErase() const;
					
					
					/**
					 * @brief Computes the contact sets associated to the circle, and the next directions of propagation
					 * @param optiBnd Unused boundary points
					 * @param epsilon Pruning parameter
					 */
					void computeContactData(const OptiBnd &optiBnd, double epsilon);

					/**
					 * @brief  Propagates the center in a given direction
					 * @param  optiBnd Unused boundary points
					 * @param  dir     Direction in which propagate
					 * @param  epsilon Pruning parameter
					 * @param  mov     [out] Next moving center
					 * @return True if a center has been computed
					 */
					bool propagate(const OptiBnd &optiBnd,
								   unsigned int dir,
								   double epsilon,
								   MovingCenter &mov) const;
					
					/**
					 * @brief Checks if two circles are neighbors for a given direction
					 * @param mov1 First circle
					 * @param dir1 Direction of the first circle
					 * @param mov2 Second circle
					 * @param dir2 Direction of the second circle
					 */
					static bool neighbors(const MovingCenter &mov1, unsigned int dir1, const MovingCenter &mov2, unsigned int dir2);
			};
		}
	}
}

#endif //_MOVINGCENTER_H_
