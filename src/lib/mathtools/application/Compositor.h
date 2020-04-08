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
 *  \file Compositor.h
 *  \brief Defines application compositor
 *  \author Bastien Durix
 */

#ifndef _COMPOSITOR_H_
#define _COMPOSITOR_H_

#include <Eigen/Dense>
#include <memory>
#include "Application.h"

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
		 * \brief Recursive template composition of n function
		 *
		 * \tparam Fct  First function type
		 * \tparam Args Next functions type
		 */
		template<typename Fct, typename... Args>
		class Compositor : public Application<typename Fct::outType, typename Fct::inType>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = typename Fct::outType;

				/**
				 *  \brief In type of result funtion
				 */
				using inType = typename Fct::inType;

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<Compositor>;

			protected:
				/**
				 *  \brief Current function
				 */
				typename Fct::Ptr m_fct;

			public:
				/**
				 *  \brief Constructor
				 */
				Compositor() : m_fct(new Fct()) {};

				/**
				 *  \brief Constructor
				 *
				 *  \param fct reference to function to compose
				 */
				Compositor(const typename Fct::Ptr fct) : m_fct(fct) {};

				/**
				 *  \brief Constructor
				 *
				 *  \param fct reference to function to compose
				 */
				Compositor(const Fct &fct) : Compositor(typename Fct::Ptr(new Fct(fct))) {};

				/**
				 *  \brief Function call
				 *
				 *  \param t input variable
				 *
				 *  \return function evaluation at t
				 */
				inline outType operator()(const inType &t) const
				{
					return (*m_fct)(t);
				};

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t input variable
				 *
				 *  \returns first derivative associated to input t
				 */
				inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const inType &t) const
				{
					return m_fct->der(t);
				}

				/**
				 *  \brief Function second derivative
				 *
				 *  \param t input of the application
				 *
				 *  \returns second derivative associated to input t
				 */
				inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const inType &t) const
				{
					return m_fct->der2(t);
				}
				
				/**
				 *  \brief Function third derivative
				 *
				 *  \param t input of the application
				 *
				 *  \returns third derivative associated to input t
				 */
				inline typename derivativematrix<3,dimension<outType>::value,dimension<inType>::value>::type
					der3(const inType &t) const
				{
					return m_fct->der3(t);
				}
				
				/**
				 *  \brief Function getter
				 *
				 *  \return current function associated to compositor
				 */
				inline const typename Fct::Ptr getFun() const
				{
					return m_fct;
				}
		};

		/**
		 * \brief Recursive template composition of n function
		 *
		 * \tparam Fct  First function type
		 * \tparam Fct_ Second function type
		 * \tparam Args Next functions type
		 */
		template<typename Fct, typename Fct_, typename... Args>
		class Compositor<Fct,Fct_,Args...> : public Application<typename Fct::outType,typename Compositor<Fct_,Args...>::inType>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = typename Fct::outType;

				/**
				 *  \brief In type of result funtion
				 */
				using inType = typename Compositor<Fct_,Args...>::inType;

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<Compositor>;

			protected:
				/**
				 *  \brief Current function
				 */
				typename Fct::Ptr m_fct;

				/**
				 * \brief Compositor contain all the next functions
				 */
				Compositor<Fct_,Args...> m_next;

			public:
				/**
				 *  \brief Constructor
				 */
				Compositor() : m_fct(new Fct()), m_next() {};

				/**
				 *  \brief Constructor
				 *
				 *  \param fct   reference to first function to compose
				 *  \param fct_  reference to second function to compose
				 *  \param fcts  next function arguments
				 */
				Compositor(const typename Fct::Ptr &fct, const typename Fct_::Ptr &fct_, const Args&... fcts) :  m_fct(fct), m_next(fct_, fcts...) {};

				/**
				 *  \brief Constructor
				 *
				 *  \param fct   reference to first function to compose
				 *  \param fct_  reference to second function to compose
				 *  \param fcts  next function arguments
				 */
				Compositor(const Fct &fct, const Fct_ &fct_, const Args&... fcts) :  m_fct(new Fct(fct)), m_next(fct_, fcts...) {};

				/**
				 *  \brief Function call
				 *
				 *  \param t input variable
				 *
				 *  \return function evaluation at t
				 */
				inline outType operator()(const inType &t) const
				{
					return (*m_fct)(  m_next(t) );
				};

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t input variable
				 *
				 *  \returns first derivative associated to input t
				 */
				inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const inType &t) const
				{
					typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> >((double*)arr.data()) = 
						Eigen::Map<Eigen::Matrix<double,dimension<typename Fct::outType>::value,dimension<typename Fct::inType>::value > >(
							(double*)m_fct->der( m_next(t) ).data())*
						Eigen::Map<Eigen::Matrix<double,dimension<typename Compositor<Fct_,Args...>::outType>::value,dimension<typename Compositor<Fct_,Args...>::inType>::value> >(
							(double*)m_next.der(t).data());
					
					return arr;
				}

				/**
				 *  \brief Function second derivative
				 *
				 *  \param t input of the application
				 *
				 *  \returns second derivative associated to input t
				 */
				inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const inType &t) const
				{
					//(f_i o g)'' = Jg^t . Hf_i . Jg + sum( df_i/dyj . Hgj ) 
					typename derivativematrix<2,dimension<typename Fct::outType>::value,dimension<typename Fct::inType>::value>::type 
						f_sec = m_fct->der2( m_next(t) );
					
					typename derivativematrix<1,dimension<typename Fct::outType>::value,dimension<typename Fct::inType>::value>::type 
						f_prim = m_fct->der( m_next(t) );
					
					typename derivativematrix<2,dimension<typename Compositor<Fct_,Args...>::outType>::value,dimension<typename Compositor<Fct_,Args...>::inType>::value>::type 
						g_sec = m_next.der2(t);
					
					typename derivativematrix<1,dimension<typename Compositor<Fct_,Args...>::outType>::value,dimension<typename Compositor<Fct_,Args...>::inType>::value>::type 
						g_prim = m_next.der(t);
					Eigen::Map<Eigen::Matrix<double,dimension<typename Compositor<Fct_,Args...>::outType>::value,dimension<typename Compositor<Fct_,Args...>::inType>::value> > Jg((double*)g_prim.data());
					
					typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type fog_der2;
					
					for(unsigned int i = 0; i < dimension<outType>::value; i++)
					{
						Eigen::Map<Eigen::Matrix<double,dimension<inType>::value,dimension<inType>::value>,0,Eigen::InnerStride<dimension<outType>::value> > fog_der2_i((double*)fog_der2.data() + i);
						
						Eigen::Map<Eigen::Matrix<double,dimension<typename Fct::inType>::value,dimension<typename Fct::inType>::value>,0,Eigen::Stride<dimension<typename Fct::outType>::value*dimension<typename Fct::inType>::value,dimension<typename Fct::outType>::value> > Hf_i((double*)f_sec.data() + i);
						
						fog_der2_i = Jg.transpose() * Hf_i * Jg;

						for(unsigned int j = 0; j < dimension<typename Compositor<Fct_,Args...>::outType>::value; j++)
						{
							Eigen::Map<Eigen::Matrix<double,dimension<typename Compositor<Fct_,Args...>::inType >::value,dimension<typename Compositor<Fct_,Args...>::inType >::value>,0,Eigen::InnerStride<dimension<typename Compositor<Fct_,Args...>::outType>::value> > Hg_j((double*)g_sec.data() + j);
							fog_der2_i += f_prim[j][i] * Hg_j;
						}
					}
					
					return fog_der2;
				}

				/**
				 *  \brief Next function getter
				 *
				 *  \return next function
				 */
				inline const Compositor<Fct_,Args...>& next() const
				{
					return m_next;
				}
				
				/**
				 *  \brief Function getter
				 *
				 *  \return current function associated to compositor
				 */
				inline const typename Fct::Ptr getFun() const
				{
					return m_fct;
				}
		};
	}
}

#endif //_COMPOSITOR_H_
