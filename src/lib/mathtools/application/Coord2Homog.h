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
 * \file Coord2Homog.h
 * \brief Defines traduction from space to homogenenous space
 * \author Durix Bastien
 */

#ifndef _COORD2HOMOG_H_
#define _COORD2HOMOG_H_

#include "Application.h"
#include <memory>
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
		 *  \brief Defines traduction from space to homogeneous space
		 */
		template<unsigned int Dim>
		class Coord2Homog : public Application<Eigen::Matrix<double,Dim+1,1>,Eigen::Matrix<double,Dim,1> >
		{
			public:
				/**
				 *  \brief Coord2Homog shared pointer
				 */
				using Ptr = std::shared_ptr<Coord2Homog<Dim> >;
				
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = Eigen::Matrix<double,Dim+1,1>;
				
				/**
				 *  \brief In type of result funtion
				 */
				using inType = Eigen::Matrix<double,Dim,1>;
				
			protected:
				/**
				 *  \brief Jacobian matrix
				 */
				Eigen::Matrix<double,Dim+1,Dim> m_jac;

			public:
				/**
				 *  \brief Construtor
				 */
				Coord2Homog() :
					Application<Eigen::Matrix<double,Dim+1,1>,Eigen::Matrix<double,Dim,1> >(),
					m_jac(Eigen::Matrix<double,Dim+1,Dim>::Zero())
				{
					m_jac.template block<Dim,Dim>(0,0) = Eigen::Matrix<double,Dim,Dim>::Identity();
				}

				/**
				 *  \brief Application call
				 *
				 *  \param vec application input
				 * 
				 *  \return application output 
				 */
				virtual inline Eigen::Matrix<double,Dim+1,1> operator()(const Eigen::Matrix<double,Dim,1> &vec) const
				{
					Eigen::Matrix<double,Dim+1,1> res;
					res.template block<Dim,1>(0,0) = vec;
					res(Dim,0) = 1.0;
					return res;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param vec Input of the application
				 *
				 *  \returns First derivative associated to input vec
				 */
				virtual inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const Eigen::Matrix<double,Dim,1> &vec) const
				{
					typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,Dim+1,Dim> >((double*)arr.data()) = m_jac;

					return arr;
				}

				/**
				 *  \brief Function second derivative
				 *
				 *  \param vec Input of the application
				 *
				 *  \returns Second derivative associated to input vec
				 */
				virtual inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const Eigen::Matrix<double,Dim,1> &vec) const
				{
					typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,Dim+1,Dim*Dim> >((double*)arr.data()) = Eigen::Matrix<double,Dim+1,Dim*Dim>::Zero();
					
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
					return m_jac;
				}
		};
	}
}


#endif //_COORD2HOMOG_H_

