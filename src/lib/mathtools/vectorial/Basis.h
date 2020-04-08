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
 *  \file  Basis.h
 *  \brief Defines a basis of a vectorial space
 *  \author Bastien Durix
 */

#ifndef _BASIS_H_
#define _BASIS_H_

#include <memory>
#include <stdexcept>
#include <Eigen/Dense>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Vectorial tools
	 */
	namespace vectorial
	{
		/**
		 *  \class Basis
		 *  \brief Basis of a vectorial space
		 *  \tparam Dim: dimension of the vectorial space
		 */
		template<unsigned int Dim>
		class Basis
		{
			public:
				/**
 				 *  \brief Shared pointer to basis
 				 */
				using Ptr = std::shared_ptr<Basis<Dim> >;

			private:
				/**
				 *  \brief Basis coordinates in canonic basis
				 */
				Eigen::Matrix<double,Dim,Dim> m_matrix;

				/**
				 *  \brief Canonic basis coordinates in basis
				 */
				Eigen::Matrix<double,Dim,Dim> m_matrix_inv;

				/**
				 *  \brief Constructor
				 *
				 *  \param matrix  full rank matrix
				 *
				 *  \throws logic_error if matrix parameter not invertible, or if it contains nan
				 */
				Basis(const Eigen::Matrix<double,Dim,Dim> &matrix) : m_matrix(matrix)
				{
					if(m_matrix != m_matrix)
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter is not a number");
					}
					double det = m_matrix.determinant();
					if(det*det <= Eigen::NumTraits<double>::dummy_precision())
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter not invertible");
					}
					m_matrix_inv = m_matrix.inverse();
				}

				/**
				 *  \brief 2d basis constructor 
				 *
				 *  \param vec1 first basis vector
				 *  \param vec2 second basis vector
				 *
				 *  \throws logic_error if matrix parameter not invertible, or if it contains nan
				 */
				Basis(const Eigen::Vector2d &vec1, const Eigen::Vector2d &vec2)
				{
					m_matrix << vec1, vec2;
					if(m_matrix != m_matrix)
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter is not a number");
					}
					double det = m_matrix.determinant();
					if(det*det <= Eigen::NumTraits<double>::dummy_precision())
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter not invertible");
					}
					m_matrix_inv = m_matrix.inverse();
				}

				/**
				 *  \brief 3d basis constructor 
				 *
				 *  \param vec1 first basis vector
				 *  \param vec2 second basis vector
				 *  \param vec3 third basis vector
				 *
				 *  \throws logic_error if matrix parameter not invertible, or if it contains nan
				 */
				Basis(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &vec3)
				{
					m_matrix << vec1, vec2, vec3;
					if(m_matrix != m_matrix)
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter is not a number");
					}
					double det = m_matrix.determinant();
					if(det*det <= Eigen::NumTraits<double>::dummy_precision())
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter not invertible");
					}
					m_matrix_inv = m_matrix.inverse();
				}
			
			public:
				/**
				 *  \brief Basis matrix accessor
				 *
				 *  \return Basis matrix in canonic basis
				 */
				inline const Eigen::Matrix<double,Dim,Dim>& getMatrix() const
				{
					return m_matrix;
				}

				/**
				 *  \brief Inverse basis matrix accessor
				 *
				 *  \return Canonic basis matrix in actual basis
				 */
				inline const Eigen::Matrix<double,Dim,Dim>& getMatrixInverse() const
				{
					return m_matrix_inv;
				}

				/**
				 *  \brief Tests if the basis is in direct order or not
				 *
				 *  \return True if the basis is in direct order
				 */
				bool isDirect() const
				{
					return (m_matrix.determinant()>0);
				}

				/**
				 *  \brief Inverse basis getter
				 *
				 *  \return Pointer to inverse basis
				 */
				const Ptr getBasisInverse() const
				{
					return CreateBasis(m_matrix_inv);
				}
			
			private:
				/**
 				 *  \brief Pointer to canonic basis
 				 */
				static Ptr canonicbasis;

			public:
				/**
 				 *  \brief Canonic basis getter
 				 *
 				 *  \return pointer to current canonic basis
 				 */
				static const Ptr CanonicBasis()
				{
					return canonicbasis;
				}

				/**
				 *  \brief Basis creator
				 *
				 *  \param matrix full rank matrix
				 */
				static const Ptr CreateBasis(const Eigen::Matrix<double,Dim,Dim> &matrix)
				{
					return Ptr(new Basis(matrix));
				}

				/**
				 *  \brief 2d basis creator
				 *
				 *  \param vec1 first basis vector
				 *  \param vec2 second basis vector
				 */
				static const Ptr CreateBasis(const Eigen::Vector2d &vec1, const Eigen::Vector2d &vec2)
				{
					return Ptr(new Basis(vec1,vec2));
				}

				/**
				 *  \brief 3d basis creator
				 *
				 *  \param vec1 first basis vector
				 *  \param vec2 second basis vector
				 *  \param vec3 third basis vector
				 */
				static const Ptr CreateBasis(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2, const Eigen::Vector3d &vec3)
				{
					return Ptr(new Basis(vec1,vec2,vec3));
				}
		};
	}
}

#endif //_BASIS_H_
