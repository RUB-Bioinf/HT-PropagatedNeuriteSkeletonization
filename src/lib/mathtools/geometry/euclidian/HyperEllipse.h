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
 *  \file  HyperEllipse.h
 *  \brief Defines an hyperellipse in an euclidian space
 *  \author Bastien Durix
 */

#ifndef _HYPERELLIPSE_H_
#define _HYPERELLIPSE_H_

#include <Eigen/Dense>
#include <mathtools/vectorial/Basis.h>
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
			 *  \brief Describes an hyperellipse in an euclidian space
			 *
			 *  \tparam Dim dimension of the space in which the hyperellipse is
			 */
			template<unsigned int Dim>
			class HyperEllipse
			{
				private:
					/**
					 *  \brief Axes of the hyperellipse
					 */
					Eigen::Matrix<double,Dim,Dim> m_axes;
					
					/**
					 *  \brief Center of the hyperellipse
					 */
					affine::Point<Dim> m_center;

				public:
					/**
					 *  \brief Constructor
					 *
					 *  \param center center of the hyperellipse
					 *  \param axes   axes of the hyperellipse
					 */
					HyperEllipse(const affine::Point<Dim> &center = affine::Point<Dim>(), const Eigen::Matrix<double,Dim,Dim> axes = Eigen::Matrix<double,Dim,Dim>::Identity()) :
						m_axes(axes), m_center(center)
					{}

					/**
					 *  \brief Center getter
					 *
					 *  \return center point
					 */
					inline const affine::Point<Dim>& getCenter() const
					{
						return m_center;
					}

					/**
					 *  \brief Axes getter
					 *
					 *  \return axes of the ellipse
					 */
					inline const Eigen::Matrix<double,Dim,Dim> getAxes() const
					{
						return m_axes;
					}
			};
		}
	}
}

#endif //_HYPERELLIPSE_H_
