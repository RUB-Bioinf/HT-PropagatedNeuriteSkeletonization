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
 *  \file  HyperPlane.h
 *  \brief Defines an hyperplane in an euclidian space
 *  \author Bastien Durix
 */

#ifndef _HYPERPLANE_H_
#define _HYPERPLANE_H_

#include <Eigen/Dense>
#include <mathtools/affine/Point.h>

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
			 *  \brief Describes an hyperplane in an euclidian space
			 *
			 *  \tparam Dim dimension of the space in which the hyperplane is
			 */
			template<unsigned int Dim>
			class HyperPlane
			{
				protected:
					/**
					 *  \brief HyperPlane factor
					 */
					double m_factor;

					/**
					 *  \brief HyperPlane normal
					 */
					Eigen::Matrix<double,Dim,1> m_normal;
				public:
					/**
					 *  \brief Constructor
					 *
					 *  \param factor  hyperPlane origin
					 *  \param normal  hyperPlane normal
					 */
					HyperPlane(const double &factor, const Eigen::Matrix<double,Dim,1> &normal) :
						m_factor(factor), m_normal(normal)
					{};

					/**
					 *  \brief Constructor
					 *
					 *  \param point   point on the hyperplane
					 *  \param normal  hyperplane normal
					 */
					HyperPlane(const affine::Point<Dim> &point = affine::Point<Dim>(), const Eigen::Matrix<double,Dim,1> &normal = Eigen::Matrix<double,Dim,1>::Zero()) :
						m_factor(point.getCoords().dot(normal)), m_normal(normal)
					{};

					/**
					 *  \brief Const factor accessor
					 *
					 *  \return HyperPlane factor
					 */
					const double& getFactor() const
					{
						return m_factor;
					}

					/**
					 *  \brief Const normal accessor
					 *
					 *  \return HyperPlane normal
					 */
					const Eigen::Matrix<double,Dim,1>& getNormal() const
					{
						return m_normal;
					}
			};
		}
	}
}

#endif //_HYPERPLANE_H_
