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
 *  \file  Projection.h
 *  \brief Defines projection of a geometric object on another
 *  \author Bastien Durix
 */

#ifndef _PROJECTION_H_
#define _PROJECTION_H_

#include <mathtools/affine/Point.h>
#include "HyperPlane.h"

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
			 *  \brief Projection of a point on an hyperplane
			 *
			 *  \tparam Dim dimension of the space
			 *
			 *  \param hplane hyperplane of projection
			 *  \param point  point to project
			 *
			 *  \return projected point 
			 */
			template<unsigned int Dim>
			affine::Point<Dim> Projection(const HyperPlane<Dim> &hplane, const affine::Point<Dim> &point)
			{
				Eigen::Matrix<double,Dim,1> coords = point.getCoords();

				double fact = (hplane.getFactor() - coords.dot(hplane.getNormal()))/(hplane.getNormal().squaredNorm());

				return affine::Point<Dim>(coords + fact*hplane.getNormal());
			}
		}
	}
}

#endif //_PROJECTION_H_

