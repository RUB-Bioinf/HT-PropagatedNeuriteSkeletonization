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
 *  \file BsplineUtils.cpp
 *  \brief Defines bspline utilitary functions
 *  \author Bastien Durix
 */

#include "Bspline.h"

double mathtools::application::BsplineBasis(double t, unsigned int degree, unsigned int indice, const Eigen::Matrix<double,1,Eigen::Dynamic> &node)
{
	if(indice + degree > node.cols())
		throw std::logic_error("BsplineBasis : Basis indice is out of node vector");
	double res = 0.0;
	/*
	 *  if degree = 0
	 *  B(t) = 1 if t_{indice-1} <= t < t_{indice}
	 *  with t_{-1}          = -infinity
	 *       t_{node.cols()} = +infinity
	 *  
	 *  example:
	 *
	 *  node vector: node = {0}
	 *
	 *  B_0(t) = 1 if t <  0
	 *           0 if t >= 0
	 *
	 *  B_1(t) = 0 if t <  0
	 *           1 if t >= 0
	 *
	 */
	if(node.cols() == 0)
	{
		res = 1.0;
	}
	else if(degree == 0)
	{
		if(indice == 0)
		{
			if(t < node(0,0))
				res = 1.0;
		}
		else if(indice == node.cols())
		{
			if(node(0,indice-1) <= t)
				res = 1.0;
		}
		else
		{
			if(node(0,indice-1) <= t && t < node(0,indice))
				res = 1.0;
		}
	}
	/*
	 *  if degree != 0
	 *  classical recursive definition of bspline basis
	 *  
	 *  B_{d,i}(t) = B_{d-1,i}(t) * a_1(t) + B_{d-1,i+1}(t) * a_2(t) 
	 *
	 *  with a_1(t) = 1                                           if i == 0
	 *	              (t - node[i-1])/(node[i+d-1] - node[i-1])   if i >  0
	 *                
	 *
	 *  with a_2(t) = (node[i+d] - t)/(node[i+d] - node[i])       if i + d <  node.cols()
	 *                1                                           if i + d == node.cols()
	 */
	else
	{
		if(indice > 0)
		{
			if(node(0, indice-1) <= t && t < node(0, indice + degree - 1))
			{
				double num1 = t                            - node(0, indice - 1);
				double den1 = node(0, indice + degree - 1) - node(0, indice - 1);

				if(den1 != 0)
					res += (num1/den1) * BsplineBasis(t, degree-1, indice, node);
			}
		}
		else
		{
			if(t < node(0,indice + degree - 1))
			{
				res += BsplineBasis(t, degree-1, indice, node);
			}
		}

		if(indice + degree < node.cols())
		{
			if(node(0, indice) <= t && t < node(0, indice + degree))
			{
				double num2 = node(0, indice + degree) - t;
				double den2 = node(0, indice + degree) - node(0, indice);

				if(den2 != 0)
					res += (num2/den2) * BsplineBasis(t, degree-1, indice+1, node);
			}
		}
		else
		{
			if(node(0, indice) <= t)
			{
				res += BsplineBasis(t, degree-1, indice+1, node);
			}
		}
	}	
	return res;
}
