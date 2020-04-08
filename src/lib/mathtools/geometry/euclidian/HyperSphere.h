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
 *  \file  HyperSphere.h
 *  \brief Defines an hypersphere in an euclidian space
 *  \author Bastien Durix
 */

#ifndef _HYPERSPHERE_H_
#define _HYPERSPHERE_H_

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
			 *  \brief Describes an hypersphere in an euclidian space
			 *
			 *  \tparam Dim dimension of the space in which the hypersphere is
			 */
			template<unsigned int Dim>
			class HyperSphere
			{
				private:
					/**
					 *  \brief Basis of the hypersphere (to now the scalar product)
					 */
					typename vectorial::Basis<Dim>::Ptr m_basis;
					
					/**
					 *  \brief Center of the hypersphere
					 */
					affine::Point<Dim> m_center;
					
					/**
					 *  \brief Radius of the hypersphere
					 */
					double m_radius;

				public:
					/**
					 *  \brief Constructor
					 *
					 *  \param center center of the hypersphere
					 *  \param radius radius of the hypersphere, in the frame
					 *  \param basis  basis of the hypersphere
					 */
					HyperSphere(const affine::Point<Dim> &center = affine::Point<Dim>(), double radius = 0.0, const typename vectorial::Basis<Dim>::Ptr basis = vectorial::Basis<Dim>::CanonicBasis()) :
						m_basis(basis), m_center(center), m_radius(radius)
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
					 *  \brief Radius getter
					 *
					 *  \return radius
					 */
					inline const double& getRadius() const
					{
						return m_radius;
					}

					/**
					 *  \brief Basis getter
					 *
					 *  \return basis of the sphere
					 */
					inline const typename vectorial::Basis<Dim>::Ptr getBasis() const
					{
						return m_basis;
					}
			};
		}
	}
}

#endif //_HYPERSPHERE_H_
