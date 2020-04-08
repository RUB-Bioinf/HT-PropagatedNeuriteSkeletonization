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
 *  \file Application.h
 *  \brief Defines generic application
 *  \author Bastien Durix
 */

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <Eigen/Dense>
#include <memory>
#include <type_traits>
#include <array>

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
		 *  \brief Type dimension getter
		 *
		 *  \tparam Type: out or in type of an application
		 */
		template<typename Type> struct dimension;

		/**
		 *  \brief Eigen vector dimension getter
		 *
		 *  \tparam Dim Eigen vector dimension
		 */
		template<int Dim> struct dimension<Eigen::Matrix<double,Dim,1> >
		{
			/**
			 *  \brief Dimension value
			 */
			static constexpr unsigned int value = Dim;
		};
		
		/**
		 *  \brief Double dimension getter
		 */
		template<>
		struct dimension<double>
		{
			/**
			 *  \brief Dimension value
			 */
			static constexpr unsigned int value = 1;
		};

		/**
		 *  \brief Recursively defined derivative matrix
		 *
		 *  \tparam Der    derivative level (greater or equal to 1)
		 *  \tparam OutDim application out dimension
		 *  \tparam InDim  application in dimension
		 */
		template<unsigned int Der, unsigned int OutDim, unsigned int InDim>
		struct derivativematrix
		{
			/**
			 *  \brief Derivative matrix type
			 *
			 *  \details Defines order (Der+1) array of size InDim x InDim x ... x InDim x OutDim
			 */
			using type = std::array<typename derivativematrix<Der-1,OutDim,InDim>::type,InDim>;
		};
		
		/**
		 *  \brief Recursively defined derivative matrix
		 *
		 *  \tparam OutDim application out dimension
		 *  \tparam InDim  application in dimension
		 */
		template<unsigned int OutDim, unsigned int InDim>
		struct derivativematrix<1,OutDim,InDim>
		{
			/**
			 *  \brief Derivative matrix type
			 *
			 *  \details Defines order 2 array of size InDim x OutDim
			 */
			using type = std::array<std::array<double,OutDim>,InDim>;
		};
		
		/**
		 *  \brief Main application class
		 *
		 *  \tparam OutType out type of the application
		 *  \tparam InType  in type of the application
		 */
		template<typename OutType, typename InType>
		class Application
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = OutType;

				/**
				 *  \brief In type of result funtion
				 */
				using inType = InType;

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<Application>;

			public:
				/**
				 *  \brief Applies the function to the parameter
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Output associated by the application
				 */
				virtual outType operator()(const inType &t) const = 0;
				
				/**
				 *  \brief Function derivative
				 *
				 *  \tparam Der derivative level
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Derivative at level Der, associated to input t
				 */
				template<unsigned int Der>
				inline typename derivativematrix<Der,dimension<outType>::value,dimension<inType>::value>::type&
					der(const inType &t) const
				{
					return der(t,Eigen::Matrix<double,Der,1>());
				}
				
				/**
				 *  \brief Jacobian matrix
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Jacobian matrix associated to input t
				 */
				virtual inline typename Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value>
					jac(const inType &t) const
				{
					return Eigen::Map<typename Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> >((double*)der(t).data());
				}
				
				/**
				 *  \brief Hessian matrix
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Hessian matrix associated to input t
				 */
				virtual inline typename Eigen::Matrix<double,dimension<inType>::value,dimension<inType>::value>
					hess(const inType &t) const
				{
					if(dimension<outType>::value != 1)
						throw std::logic_error("Hessian matrices are only defined for function of which out dimension is 1");
					return Eigen::Map<typename Eigen::Matrix<double,dimension<inType>::value,dimension<inType>::value> >((double*)der2(t).data());
				}
				
			private:
				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type& 
					der(const inType &t, const Eigen::Matrix<double,1,1>&) const
				{
					return der(t);
				}
				
				/**
				 *  \brief Function second derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Second derivative associated to input t
				 */
				inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type& 
					der(const inType &t, const Eigen::Matrix<double,2,1>&) const
				{
					return der2(t);
				}
				
				/**
				 *  \brief Function third derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Third derivative associated to input t
				 */
				inline typename derivativematrix<3,dimension<outType>::value,dimension<inType>::value>::type& 
					der(const inType &t, const Eigen::Matrix<double,3,1>&) const
				{
					return der3(t);
				}
				
			public:
				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				virtual inline typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const inType &t) const
				{
					throw std::logic_error("Not implemented");
				}
				
				/**
				 *  \brief Function second derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Second derivative associated to input t
				 */
				virtual inline typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const inType &t) const
				{
					throw std::logic_error("Not implemented");
				}
				
				/**
				 *  \brief Function third derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Third derivative associated to input t
				 */
				virtual inline typename derivativematrix<3,dimension<outType>::value,dimension<inType>::value>::type
					der3(const inType &t) const
				{
					throw std::logic_error("Not implemented");
				}
				
				/**
				 *  \brief Pure virtual destructor
				 */
				virtual ~Application<OutType,InType>() {};
		};
	}
}


#endif //_APPLICATION_H_


