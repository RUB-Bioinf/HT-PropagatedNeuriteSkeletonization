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
 *  \file  Frame.h
 *  \brief Defines a frame of an affine space
 *  \author Bastien Durix
 */

#ifndef _FRAME_H_
#define _FRAME_H_

#include <memory>
#include <Eigen/Dense>
#include <mathtools/vectorial/Basis.h>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Affine tools
	 */
	namespace affine
	{
		/**
		 *  \class Frame
		 *  \brief Frame of an affine space
		 *  \tparam Dim dimension of the affine space
		 */
		template<unsigned int Dim>
		class Frame
		{
			public:
				/**
				 *  \brief Frame shared pointer
				 */
				using Ptr = std::shared_ptr<Frame<Dim> >;
			
			private:
				/**
				 *  \brief Pointer to vectorial basis
				 */
				typename vectorial::Basis<Dim>::Ptr m_basis;

				/**
				 *  \brief Frame origin
				 */
				Eigen::Matrix<double,Dim,1> m_origin;

				/**
				 *  \brief Constructor
				 *
				 *  \param origin frame origin
				 *  \param basis  pointer to vectorial basis of the frame
				 */
				Frame(const Eigen::Matrix<double,Dim,1> &origin, const typename vectorial::Basis<Dim>::Ptr &basis) :
					m_basis(basis), m_origin(origin) {}
			
			public:
				/**
				 *  \brief Vectorial basis getter
				 *
				 *  \return Current basis
				 */
				inline const typename vectorial::Basis<Dim>::Ptr getBasis() const
				{
					return m_basis;
				}

				/**
				 *  \brief Frame origin coordinates getter
				 *
				 *  \return Current origin coordinates in canonic frame
				 */
				inline const Eigen::Matrix<double,Dim,1>& getOrigin() const
				{
					return m_origin;
				}

				/**
				 *  \brief Frame inverse getter
				 *
				 *  \return Pointer to inverse frame
				 */
				Ptr getFrameInverse() const
				{
					return CreateFrame(-1.0*getBasis()->getMatrixInverse()*getOrigin(),getBasis()->getBasisInverse());
				}

			private:
				/**
 				 *  \brief Pointer to canonic frame
 				 */
				static Ptr canonicframe;

			public:
				/**
 				 *  \brief Canonic frame getter
 				 *
 				 *  \return Pointer to current canonic frame
 				 */
				static const Ptr CanonicFrame()
				{
					return canonicframe;
				}

				/**
				 *  \brief Frame creator
				 *
				 *  \param origin frame origin
				 *  \param basis  pointer to vectorial basis of the frame
				 */
				static const Ptr CreateFrame(const Eigen::Matrix<double,Dim,1> &origin,
											 const typename vectorial::Basis<Dim>::Ptr &basis = vectorial::Basis<Dim>::CanonicBasis())
				{
					return Ptr(new Frame<Dim>(origin,basis));
				}

				/**
				 *  \brief 2d frame creator
				 *
				 *  \param origin frame origin
				 *  \param vec1   first basis vector
				 *  \param vec2   second basis vector
				 */
				static const Ptr CreateFrame(const Eigen::Vector2d &origin, const Eigen::Vector2d &vec1, const Eigen::Vector2d &vec2)
				{
					return Ptr(new Frame<Dim>(origin,vectorial::Basis<Dim>::CreateBasis(vec1,vec2)));
				}

				/**
				 *  \brief 3d frame creator
				 *
				 *  \param origin frame origin
				 *  \param vec1   first basis vector
				 *  \param vec2   second basis vector
				 *  \param vec3   third basis vector
				 */
				static const Ptr CreateFrame(const Eigen::Vector3d &origin, const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &vec3)
				{
					return Ptr(new Frame<Dim>(origin,vectorial::Basis<Dim>::CreateBasis(vec1,vec2,vec3)));
				}
		};
	}
}

#endif //_FRAME_H_
