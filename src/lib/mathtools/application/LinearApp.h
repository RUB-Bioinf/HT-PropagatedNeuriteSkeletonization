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
 *  \file LinearApp.h
 *  \brief Defines linear application
 *  \author Bastien Durix
 */

#ifndef _LINEARAPPLICATION_H_
#define _LINEARAPPLICATION_H_

#include "Application.h"
#include <Eigen/Dense>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Application tools
	 */
	namespace application
	{
		/**
		 * \brief Linear application
		 *
		 * \tparam DimIn  In vector dimension
		 * \tparam DimOut Out vector dimension
		 */
		template<unsigned int DimOut, unsigned int DimIn>
		class LinearApp : public Application<Eigen::Matrix<double,DimOut,1>,Eigen::Matrix<double,DimIn,1> >
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = Eigen::Matrix<double,DimOut,1>;
				/**
				 *  \brief In type of result funtion
				 */
				using inType = Eigen::Matrix<double,DimIn,1>;

			protected:
				/**
				 *  \brief Linear application matrix
				 */
				Eigen::Matrix<double,DimOut,DimIn> m_matrix;

			public:
				/**
				 *  \brief Construtor
				 *
				 *  \param matrix Application matrix
				 */
				LinearApp(const Eigen::Matrix<double,DimOut,DimIn> &matrix = Eigen::Matrix<double,DimOut,DimIn>::Zero()) :
					Application<Eigen::Matrix<double,DimOut,1>,Eigen::Matrix<double,DimIn,1> >(), m_matrix(matrix)
				{}

				/**
				 *  \brief Copy construtor
				 *
				 *  \param linapp LinearApplication to copy in
				 */
				LinearApp(const LinearApp<DimOut,DimIn> &linapp) :
					Application<Eigen::Matrix<double,DimOut,1>,Eigen::Matrix<double,DimIn,1> >(), m_matrix(linapp.m_matrix)
				{}

				/**
				 *  \brief Application call
				 *
				 *  \param vec application input
				 * 
				 *  \return application output 
				 */
				virtual inline Eigen::Matrix<double,DimOut,1> operator()(const Eigen::Matrix<double,DimIn,1> &vec) const
				{
					return m_matrix * vec;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param vec Input of the application
				 *
				 *  \returns First derivative associated to input vec
				 */
				virtual inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const Eigen::Matrix<double,DimIn,1> &vec) const
				{
					typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,DimOut,DimIn> >((double*)arr.data()) = m_matrix;

					return arr;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param vec Input of the application
				 *
				 *  \returns First derivative associated to input vec
				 */
				virtual inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const Eigen::Matrix<double,DimIn,1> &vec) const
				{
					typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,DimOut,DimIn*DimIn> >((double*)arr.data()) = Eigen::Matrix<double,DimOut,DimIn*DimIn>::Zero();
					
					return arr;
				}
				
				/**
				 *  \brief Jacobian matrix
				 *
				 *  \param vec Input of the application
				 *
				 *  \returns Jacobian matrix associated to input vec
				 */
				virtual inline typename Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value>
					jac(const inType &vec) const
				{
					return m_matrix;
				}

				/**
				 *  \brief Matrix getter
				 *
				 *  \return Linera applicatino matrix
				 */
				inline Eigen::Matrix<double,DimOut,DimIn>& getMatrix() const
				{
					return m_matrix;
				}
		};
	}
}

#endif //_LINEARAPPLICATION_H_
