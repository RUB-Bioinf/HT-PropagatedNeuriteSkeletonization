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
 *  \file Nurbs.h
 *  \brief Defines NURBS function
 *  \author Bastien Durix
 */

#ifndef _NURBS_H_
#define _NURBS_H_

#include <Eigen/Dense>
#include <memory>
#include "Application.h"
#include "BsplineUtils.h"

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
		 *  \brief Defines Nurbs curve
		 *
		 *  \tparam Dim Curve dimension
		 */
		template<unsigned int Dim>
		class Nurbs : public Application<Eigen::Matrix<double,Dim,1>,double>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = Eigen::Matrix<double,Dim,1>;

				/**
				 *  \brief In type of result funtion
				 */
				using inType = double;

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<Nurbs>;

			protected:
				/**
				 *  \brief Control points
				 */
				Eigen::Matrix<double,Dim,Eigen::Dynamic> m_ctrlpt;

				/**
				 *  \brief Weight associated to control points
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_weight;

				/**
				 *  \brief Node vector
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_nodevec;

				/**
				 *  \brief Control points for first derivative
				 */
				Eigen::Matrix<double,Dim,Eigen::Dynamic> m_ctrlptder;

				/**
				 *  \brief Weight for first derivative
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_weightder;

				/**
				 *  \brief Node vector for first derivative
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_nodevecder;

				/**
				 *  \brief Control points for second derivative
				 */
				Eigen::Matrix<double,Dim,Eigen::Dynamic> m_ctrlptder2;

				/**
				 *  \brief Weight for second derivative
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_weightder2;

				/**
				 *  \brief Node vector for second derivative
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_nodevecder2;

				/**
				 *  \brief Nurbs degree
				 */
				unsigned int m_degree;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param ctrlpt   Control points
				 *  \param nodevec  Node vector
				 *  \param degree   Nurbs degree
				 *
				 *  \throws std::logic_error if number of control points, nodes and degree does not match
				 */
				Nurbs(const Eigen::Matrix<double,Dim,Eigen::Dynamic> &ctrlpt,
					  const Eigen::Matrix<double,1,Eigen::Dynamic>   &weight,
					  const Eigen::Matrix<double,1,Eigen::Dynamic>   &nodevec,
					  const unsigned int &degree) : 
					Application<Eigen::Matrix<double,Dim,1>,double>(),
					m_ctrlpt(ctrlpt), m_weight(weight), m_nodevec(nodevec),
					m_ctrlptder(Dim,1), m_weightder(1,1), m_nodevecder(1,1),
					m_ctrlptder2(Dim,1), m_weightder2(1,1), m_nodevecder2(1,1),
					m_degree(degree)
				{
					if( ctrlpt.cols() + degree != nodevec.cols()+1  )
						throw std::logic_error("Nurbs : not verified #CtrlPt + degree = #NodeVec + 1");
					if(m_degree>0)
					{
						Eigen::Matrix<double,Dim+1,Eigen::Dynamic> mat(Dim+1,m_ctrlpt.cols());
						Eigen::Matrix<double,Dim+1,Eigen::Dynamic> dermat(Dim+1,1);
						for(unsigned int i = 0; i < m_ctrlpt.cols(); i++)
						{
							mat.template block<Dim,1>(0,i) = m_ctrlpt.template block<Dim,1>(0,i)*m_weight(i);
						}
						mat.block(Dim,0,1,m_ctrlpt.cols()) = m_weight;
						
						ComputeDerivative<Dim+1>(m_degree, m_nodevec, mat, m_nodevecder, dermat);
						
						m_ctrlptder.resize(Dim,dermat.cols());
						m_weightder.resize(1,dermat.cols());
						m_weightder = dermat.block(Dim,0,1,dermat.cols());
						for(unsigned int i = 0; i < m_ctrlptder.cols(); i++)
						{
							m_ctrlptder.template block<Dim,1>(0,i) = dermat.template block<Dim,1>(0,i)/m_weightder(i);
						}
					}
					if(m_degree>1)
					{
						Eigen::Matrix<double,Dim+1,Eigen::Dynamic> mat(Dim+1,m_ctrlptder.cols());
						Eigen::Matrix<double,Dim+1,Eigen::Dynamic> dermat(Dim+1,1);
						for(unsigned int i = 0; i < m_ctrlptder.cols(); i++)
						{
							mat.template block<Dim,1>(0,i) = m_ctrlptder.template block<Dim,1>(0,i)*m_weightder(i);
						}
						mat.block(Dim,0,1,m_ctrlptder.cols()) = m_weightder;
						
						ComputeDerivative<Dim+1>(m_degree-1, m_nodevecder, mat, m_nodevecder2, dermat);
						
						m_ctrlptder2.resize(Dim,dermat.cols());
						m_weightder2.resize(1,dermat.cols());
						m_weightder2 = dermat.block(Dim,0,1,dermat.cols());
						for(unsigned int i = 0; i < m_ctrlptder2.cols(); i++)
						{
							m_ctrlptder2.template block<Dim,1>(0,i) = dermat.template block<Dim,1>(0,i)/m_weightder2(i);
						}
					}
				}

				/**
				 *  \brief Copy constructor
				 *
				 *  \param nurbs Nurbs to copy
				 */
				Nurbs(const Nurbs<Dim> &nurbs) : 
					Application<Eigen::Matrix<double,Dim,1>,double>(),
					m_ctrlpt(nurbs.m_ctrlpt), m_weight(nurbs.weight), m_nodevec(nurbs.m_nodevec),
					m_ctrlptder(nurbs.m_ctrlptder), m_weightder(nurbs.weightder), m_nodevecder(nurbs.m_nodevecder),
					m_ctrlptder2(nurbs.m_ctrlptder2), m_weightder2(nurbs.weightder2), m_nodevecder2(nurbs.m_nodevecder2),
					m_degree(nurbs.m_degree) {};
				
				/**
				 *  \brief Nurbs call
				 *
				 *  \param t Nurbs parameter
				 * 
				 *  \return Nurbs evaluation at t
				 */
				Eigen::Matrix<double,Dim,1> operator()(const double &t) const
				{
					Eigen::Matrix<double,Dim,1> res = Eigen::Matrix<double,Dim,1>::Zero();
					double weight = 0.0;       // weight function
					Eigen::Matrix<double,Dim,1> comb = Eigen::Matrix<double,Dim,1>::Zero();         // linear combination of coordinates
					
					for(unsigned int ind = 0; ind < m_ctrlpt.cols(); ind++)
					{
						double basis_ind = BsplineBasis(t,m_degree,ind,m_nodevec);
						comb += m_ctrlpt.block(0,ind,Dim,1)*basis_ind*m_weight(ind);
						weight += basis_ind*m_weight(ind);
					}

					res = comb * (1.0/weight);
					
					return res;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				virtual typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const double &t) const
				{
					typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> > res((double*)arr.data());

					res.setZero();
					if(m_degree > 0)
					{
						double weight = 0.0;       // weight function
						double weight_prime = 0.0; // derivative of weight function
						Eigen::Matrix<double,Dim,1> comb = Eigen::Matrix<double,Dim,1>::Zero();         // linear combination of coordinates
						Eigen::Matrix<double,Dim,1> comb_prime = Eigen::Matrix<double,Dim,1>::Zero();   // derivative of linear combination of coordinates
						
						for(unsigned int ind = 0; ind < m_ctrlpt.cols(); ind++)
						{
							double basis_ind = BsplineBasis(t,m_degree,ind,m_nodevec);
							comb += m_ctrlpt.block(0,ind,Dim,1)*basis_ind*m_weight(ind);
							weight += basis_ind*m_weight(ind);
						}

						for(unsigned int ind = 0; ind<m_ctrlptder.cols(); ind++)
						{
							double basis_ind = BsplineBasis(t,m_degree-1,ind,m_nodevecder);
							comb_prime += m_ctrlptder.block(0,ind,Dim,1)*basis_ind*m_weightder(ind);
							weight_prime += basis_ind*m_weightder(ind);
						}
						res = (comb_prime*weight - comb*weight_prime)*(1.0/(weight*weight));
					}

					return arr;
				}

				/**
				 *  \brief Function second derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Second derivative associated to input t
				 */
				virtual typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const double &t) const
				{
					typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> > res((double*)arr.data());
					
					res.setZero();
					
					if(m_degree > 1)
					{
						double weight = 0.0;        // weight function
						double weight_prime = 0.0;  // derivative of weight function
						double weight_second = 0.0; // second derivative of weight function
						Eigen::Matrix<double,Dim,1> comb = Eigen::Matrix<double,Dim,1>::Zero();          // linear combination of coordinates
						Eigen::Matrix<double,Dim,1> comb_prime = Eigen::Matrix<double,Dim,1>::Zero();    // derivative of linear combination of coordinates
						Eigen::Matrix<double,Dim,1> comb_second = Eigen::Matrix<double,Dim,1>::Zero();   // second derivative of linear combination of coordinates
						
						for(unsigned int ind = 0; ind < m_ctrlpt.cols(); ind++)
						{
							double basis_ind = BsplineBasis(t,m_degree,ind,m_nodevec);
							comb += m_ctrlpt.block(0,ind,Dim,1)*basis_ind*m_weight(ind);
							weight += basis_ind*m_weight(ind);
						}

						for(unsigned int ind = 0; ind<m_ctrlptder.cols(); ind++)
						{
							double basis_ind = BsplineBasis(t,m_degree-1,ind,m_nodevecder);
							comb_prime += m_ctrlptder.block(0,ind,Dim,1)*basis_ind*m_weightder(ind);
							weight_prime += basis_ind*m_weightder(ind);
						}

						for(unsigned int ind = 0; ind<m_ctrlptder2.cols(); ind++)
						{
							double basis_ind = BsplineBasis(t,m_degree-2,ind,m_nodevecder2);
							comb_second += m_ctrlptder2.block(0,ind,Dim,1)*basis_ind*m_weightder2(ind);
							weight_second += basis_ind*m_weightder2(ind);
						}
						res = (comb_second*weight*weight - comb*weight_second*weight - 2.0*comb_prime*weight_prime*weight + 2.0*comb*weight_prime*weight_prime)*(1.0/(weight*weight*weight));
					}

					return arr;
				}

				/**
				 *  \brief Inferior boundary accessor
				 *
				 *  \return Inferior boundary
				 */
				double getInfBound() const
				{
					return m_nodevec(0,m_degree-1);
				}
				
				/**
				 *  \brief Superior boundary accessor
				 *
				 *  \return Superior boundary
				 */
				double getSupBound() const
				{
					return m_nodevec(0,m_nodevec.cols()-m_degree);
				}
		};
	}
}


#endif //_NURBS_H_
