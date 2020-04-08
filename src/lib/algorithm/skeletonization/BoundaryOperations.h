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
 *  @file BoundaryOperations.h
 *  @brief Defines the set of operations on the boudary
 *  @author Bastien Durix
 */

#ifndef _BOUNDARYOPERATIONS_H_
#define _BOUNDARYOPERATIONS_H_

#include <unordered_map>
#include <boundary/DiscreteBoundary2.h>

/**
 *  @brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  @brief Skeletonization algorithms
	 */
	namespace skeletonization
	{
		/**
		 * @brief Skeletonization by propagation algorithm
		 */
		namespace propagation
		{
			/**
			 * @brief Point of the boundary, with the indices of the next and previous points
			 */
			struct OptiPt
			{
				Eigen::Vector2d coords;
				unsigned int prev;
				unsigned int next;
			};
			
			/**
			 * @brief Structure to optimize the computation of the skeleton
			 */
			using OptiBnd = std::unordered_map<unsigned int, OptiPt>;
			
			/**
			 * @brief Structure to determine if all points have been used
			 */
			using OptiUsedBnd = std::map<unsigned int, bool>;

			/**
			 * @brief  Finds the closest points on the boundary to a given center
			 * @param  optiBnd     Unused boundary points
			 * @param  center      Current center
			 * @param  closestInds [out] Indices of the closest points
			 * @return Minimal distance to the boundary
			 */
			double closestInd(const OptiBnd &optiBnd,
							  const Eigen::Vector2d &center,
							  std::list<unsigned int> &closestInds);
			
			/**
			 * @brief Computes a center from a set of 3 boundary points
			 * @param P1 First point
			 * @param P2 Second point
			 * @param P3 Third point
			 * @return Computed circle center
			 */
			Eigen::Vector2d circleCenter(const Eigen::Vector2d &P1, const Eigen::Vector2d &P2, const Eigen::Vector2d &P3);
			
			/**
			 * @brief  Finds a first point inside a given shape, to initialize the algorithm
			 * @param  optiBnd Unused boundary points
			 * @return One point inside the shape
			 */
			Eigen::Vector2d firstPoint(const OptiBnd &optiBnd);
			
			/**
			 * @brief Computes a Voronoi center from a given point inside the shape
			 * @param optiBnd     Unused boundary points
			 * @param center      [in/out] One point inside the shape
			 * @param closestInds [out] Associated closest indices
			 */
			void firstVoroPoint(const OptiBnd &optiBnd,
								Eigen::Vector2d &center,
								std::list<unsigned int> &closestInds);
			
			/**
			 * @brief Orders a set of points in direct angular order around a given center
			 * @param optiBnd            Unused boundary points
			 * @param center             Center around which order the points
			 * @param closestInds        Indices to order
			 * @param closestIndsOrdered [out] Ordered indices
			 */
			void angOrder(const OptiBnd &optiBnd,
						  const Eigen::Vector2d &center,
						  const std::list<unsigned int> &inds,
						  std::vector<unsigned int> &indsOrder);
			
			/**
			 * @brief  Computes the contact set between a pair of closest points
			 * @param  optiBnd Unused boundary points
			 * @param  center  Current circle center
			 * @param  distMax Maximum distance to the center
			 * @param  indBeg  First boundary indice
			 * @param  indEnd  Last boundary indice
			 * @param  toErase [out] List of points to erase in the optimised boundary
			 * @return True if the direction between ind1 and ind2 is open
			 */
			bool contactSet(const OptiBnd &optiBnd,
						    const Eigen::Vector2d &center,
							double distMax,
							unsigned int indBeg,
							unsigned int indEnd,
							std::list<unsigned int> &toErase);
			
			/**
			 * @brief Computes the contact sets between each pair of closest points
			 * @param optiBnd     Unused boundary points
			 * @param center      Current circle center
			 * @param distMax     Maximum distance to the center
			 * @param closestInds Indices of the closest points
			 * @param open        [out] List of open directions
			 * @param toErase     [out] List of points to erase in the optimised boundary
			 */
			void contactSets(const OptiBnd &optiBnd,
							 const Eigen::Vector2d &center,
							 const double &distMax,
							 const std::vector<unsigned int> &closestIndsOrdered,
							 std::vector<bool> &open,
							 std::list<unsigned int> &toErase);
			
			/**
			 * @brief Creates a structure to optimize the computation of the skeleton
			 * @param disBnd      Boundary of the shape
			 * @param optiBnd     [out] Optimised structure
			 * @param optiUsedBnd [out] Structure telling used points
			 */
			void createOptiBnd(const boundary::DiscreteBoundary<2>::Ptr &disBnd, OptiBnd &optiBnd, OptiUsedBnd &optiUsedBnd);
			
			/**
			 * @brief Cleans the boundary structure from used points
			 * @param optiBnd     [in/out] Optimized structure
			 * @param optiUsedBnd [in/out] Structure telling used points
			 * @param toErase     Indices to erase
			 * @param closestInds Indice of the closest points
			 */
			void cleanOptiBnd(OptiBnd &optiBnd, OptiUsedBnd &optiUsedBnd, const std::list<unsigned int> &toErase, const std::vector<unsigned int> &closestInds);
		}
	}
}

#endif //_BOUNDARYOPERATIONS_H_
