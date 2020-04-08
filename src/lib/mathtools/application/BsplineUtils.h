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
 *  \file BsplineUtils.h
 *  \brief Defines utilitary bspline functions
 *  \author Bastien Durix
 */

#ifndef _BSPLINEUTILS_H_
#define _BSPLINEUTILS_H_

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
		 *  \brief Evaluate the bspline basis function
		 *
		 *	\param t      Parameter for which evaluate the basis
		 *	\param degree Bspline degree
		 *	\param indice Indice of bspline basis
		 *	\param node   Node vector of the basis
		 *
		 *  \return evaluation of the basis for parameter t
		 *
		 *  \throws std::logic_error if basis indice is out of node vector
		 */
		double BsplineBasis(double t, unsigned int degree, unsigned int indice, const Eigen::Matrix<double,1,Eigen::Dynamic> &node);

		/**
 		 *  \brief Computes derivative base of nodes and control points for a bspline
 		 *
 		 *  \tparam Dim    Control points dimension
 		 *
 		 *  \param degree  Current degree of the bspline
 		 *  \param nodein  In node vector
 		 *  \param ctrlin  In control vector
 		 *  \param nodeout Out node vector
 		 *  \param ctrlout Out control vector
 		 */
		template<unsigned int Dim>
		void ComputeDerivative(unsigned int degree,
							   const Eigen::Matrix<double,1,Eigen::Dynamic> &nodein,
							   const Eigen::Matrix<double,Dim,Eigen::Dynamic> &ctrlin,
							   Eigen::Matrix<double,1,Eigen::Dynamic> &nodeout,
							   Eigen::Matrix<double,Dim,Eigen::Dynamic> &ctrlout)
		{
			if(ctrlout.cols() != ctrlin.cols() - 1 )
			{
				ctrlout.resize(Dim,ctrlin.cols() - 1);
			}
			ctrlout.block(0,0,Dim,ctrlin.cols() - 1) = degree * (ctrlin.block(0,1,Dim,ctrlin.cols()-1) - ctrlin.block(0,0,Dim,ctrlin.cols()-1));

			for(unsigned int i = 0; i < Dim; i++)
				ctrlout.block(i,0,1,ctrlout.cols()) =
					ctrlout.block(i,0,1,ctrlout.cols()).cwiseQuotient(nodein.block(0, degree, 1,ctrlout.cols())-nodein.block(0, 0, 1, ctrlout.cols()));

			if(nodeout.cols() != nodein.cols() - 2)
			{
				nodeout.resize(1,nodein.cols() - 2);
			}
			nodeout.block(0,0,1,nodein.cols() - 2) = nodein.block(0,1,1,nodein.cols() - 2);
		}
	}
}

#endif //_BSPLINEUTILS_H_
