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
 *  \file  Intersection.h
 *  \brief Defines intersections between geometric objects
 *  \author Bastien Durix
 */

#ifndef _INTERSECTION_H_
#define _INTERSECTION_H_

#include <mathtools/affine/Point.h>
#include "Projection.h"
#include "HyperSphere.h"
#include "HyperPlane.h"
#include "HyperCircle.h"

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Geometric objects
	 */
	namespace geometry
	{
		/**
		 *  \brief Euclidian geometry
		 */
		namespace euclidian
		{
			/**
			 *  \brief Intersection between an hyperplane and an hypersphere
			 *
			 *  \tparam Dim dimension of the space
			 *  
			 *  \param hplane  hyperplane to intersect
			 *  \param hsphere hypersphere to intersect
			 *  \param hcircle out hypercircle result (defined only if it exists)
			 *  
			 *  \return true if the intersection exists
			 */
			template<unsigned int Dim>
			bool Intersection(const HyperPlane<Dim> &hplane, const HyperSphere<Dim> &hsphere, HyperCircle<Dim> &hcircle)
			{
				affine::Point<3> circle_center = Projection<3>(hplane,hsphere.getCenter());

				Eigen::Matrix<double,Dim,1> vecproj_in_basis = hsphere.getBasis()->getMatrixInverse()*(circle_center - hsphere.getCenter());
				
				double rad_sq_in_basis = hsphere.getRadius()*hsphere.getRadius();

				double dist_sq_in_basis = vecproj_in_basis.squaredNorm();

				bool intersect = rad_sq_in_basis>=dist_sq_in_basis;

				if(intersect)
				{
					double radius = sqrt(rad_sq_in_basis-dist_sq_in_basis);

					hcircle = HyperCircle<Dim>(circle_center, radius, hplane.getNormal());
				}

				return intersect;
			}
		}
	}
}

#endif //_INTERSECTION_H_
