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
 *  \file  HyperCircle.h
 *  \brief Defines an hypercircle in an euclidian space
 *  \author Bastien Durix
 */

#ifndef _HYPERCIRCLE_H_
#define _HYPERCIRCLE_H_

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
			 *  \brief Describes an hypercircle in an euclidian space
			 *
			 *  \tparam Dim dimension of the space in which the hypercircle is
			 *
			 *  \details An hypercircle is a Dim-2 variety
			 */
			template<unsigned int Dim>
			class HyperCircle
			{
				private:
					/**
					 *  \brief Basis of the hypercircle (to now the scalar product)
					 */
					typename vectorial::Basis<Dim>::Ptr m_basis;
					
					/**
					 *  \brief Center of the hypercircle
					 */
					affine::Point<Dim> m_center;
					
					/**
					 *  \brief Radius of the hypercircle
					 */
					double m_radius;

					/**
					 *  \brief Normal of the hypercircle
					 */
					Eigen::Matrix<double,Dim,1> m_normal;

				public:
					/**
					 *  \brief Constructor
					 *
					 *  \param center center of the hypercircle
					 *  \param radius radius of the hypercircle, in the basis
					 *  \param normal normal of the hypercircle (in canonic basis)
					 *  \param basis  basis of the hypercircle
					 */
					HyperCircle(const affine::Point<Dim> &center = affine::Point<Dim>(), double radius = 0.0, const Eigen::Matrix<double,Dim,1> &normal = Eigen::Matrix<double,Dim,1>::Zero(),
								const typename vectorial::Basis<Dim>::Ptr basis = vectorial::Basis<Dim>::CanonicBasis()) :
						m_basis(basis), m_center(center), m_radius(radius), m_normal(normal)
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
					 *  \brief Normal getter (in canonic basis)
					 *
					 *  \return normal
					 */
					inline const Eigen::Matrix<double,Dim,1>& getNormal() const
					{
						return m_normal;
					}

					/**
					 *  \brief Basis getter
					 *
					 *  \return basis of the circle
					 */
					inline const typename vectorial::Basis<Dim>::Ptr getBasis() const
					{
						return m_basis;
					}
			};
		}
	}
}

#endif //_HYPERCIRCLE_H_
