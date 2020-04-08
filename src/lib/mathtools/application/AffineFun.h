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
 *  \file AffineFun.h
 *  \brief Defines affine function
 *  \author Bastien Durix
 */

#ifndef _AFFINEFUNCTION_H_
#define _AFFINEFUNCTION_H_

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
		 * \brief Affine function
		 */
		class AffineFun : public Application<double,double>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = double;
				/**
				 *  \brief In type of result funtion
				 */
				using inType = double;

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<AffineFun>;


			protected:
				/**
				 *  \brief slope
				 */
				double m_a;

				/**
				 *  \brief y-intercept
				 */
				double m_b;

			public:
				/**
				 *  \brief Construtor
				 *
				 *  \brief a slope
				 *  \brief b y-intercept
				 */
				AffineFun(double a, double b) : m_a(a), m_b(b)
				{}

				/**
				 *  \brief Copy construtor
				 *
				 *  \param afffun LinearApplication to copy in
				 */
				AffineFun(const AffineFun &afffun) : m_a(afffun.m_a), m_b(afffun.m_b)
				{}

				/**
				 *  \brief Application call
				 *
				 *  \param t application input
				 * 
				 *  \return application output 
				 */
				virtual inline double operator()(const double &t) const
				{
					return m_a * t + m_b;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				virtual inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const double &t) const
				{
					return {m_a};
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				virtual inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const double &t) const
				{
					return {0};
				}

				/**
				 *  \brief Slope getter
				 *
				 *  \return slope
				 */
				inline const double& getSlope() const
				{
					return m_a;
				}

				/**
				 *  \brief Y-intercept getter
				 *
				 *  \return y-intercept
				 */
				inline const double& getYIntercept() const
				{
					return m_b;
				}
		};
	}
}

#endif //_AFFINEFUN_H_
