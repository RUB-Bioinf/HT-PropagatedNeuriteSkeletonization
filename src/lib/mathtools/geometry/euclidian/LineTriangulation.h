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
 *  \file  LineTriangulation.h
 *  \brief Defines line triangulation functions
 *  \author Bastien Durix
 */

#ifndef _LINETRIANGULATION_H_
#define _LINETRIANGULATION_H_

#include <mathtools/affine/Point.h>
#include "Line.h"

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Geometric objects
	 */
	namespace geometry
	{
		/**
		 *  \brief Euclidian geometry
		 */
		namespace euclidian
		{
			/**
			 *  \brief Compute the triangulation of two lines, and distance between them
			 *
			 *  \param line1 first line to triangulate
			 *  \param line2 second line to triangulate
			 *
			 *  \return triangulated point and associated distance 
			 */
			template<unsigned int Dim>
			std::pair<affine::Point<Dim>,double> Triangulate(const Line<Dim> &line1, const Line<Dim> &line2)
			{
				Eigen::Matrix<double,Dim,1> ori1 = line1.getOrigin().getCoords();
				Eigen::Matrix<double,Dim,1> ori2 = line2.getOrigin().getCoords();
				
				Eigen::Matrix<double,Dim,1> dir1 = line1.getVecDir();
				Eigen::Matrix<double,Dim,1> dir2 = line2.getVecDir();
				
				Eigen::Matrix2d A;
				A <<  dir1.dot(dir1) , -dir1.dot(dir2) ,
				     -dir1.dot(dir2) ,  dir2.dot(dir2) ;
				
				Eigen::Vector2d b;
				b << (ori2 - ori1).dot(dir1) ,
				     (ori1 - ori2).dot(dir2) ;
				
				Eigen::Matrix<double,Dim,1> coordsP;
				
				double dist;
				if(A.determinant() != 0)
				{
					Eigen::Vector2d X = A.householderQr().solve(b);
					
					coordsP = (ori1+ori2 + X(0)*dir1 + X(1)*dir2)*0.5;
					dist = (ori1-ori2 + X(0)*dir1 - X(1)*dir2).norm();
				}
				else
				{
					coordsP = (ori1+ori2)*0.5;
					dist = (ori1-ori2).norm();
				}
				
				return std::pair<affine::Point<Dim>,double>(coordsP,dist);
			}

			/**
			 *  \brief Compute the triangulation of a set of lines
			 *
			 *  \param line lines to triangulate
			 *
			 *  \return triangulated point
			 */
			template<unsigned int Dim>
			affine::Point<Dim> Triangulate(const std::vector<Line<Dim> > &line)
			{
				Eigen::Matrix<double,Dim,Dim> A = Eigen::Matrix<double,Dim,Dim>::Zero();
				Eigen::Matrix<double,Dim,1>   b = Eigen::Matrix<double,Dim,1>::Zero();
				
				for(unsigned int i = 0; i<line.size();i++)
				{
					Eigen::Matrix<double,Dim,1> vec = line[i].getVecDir();
					Eigen::Matrix<double,Dim,1> ori = line[i].getOrigin().getCoords();
					
					Eigen::Matrix<double,Dim,Dim> Idmodif = Eigen::Matrix<double,Dim,Dim>::Identity() - vec*vec.transpose();
					
					A = A + Idmodif;
					b = b + Idmodif*ori;
				}
				
				Eigen::Matrix<double,Dim,1> Pcoo = A.householderQr().solve(b);
				
				return affine::Point<Dim>(Pcoo);
			}
		}
	}
}

#endif //_LINETRIANGULATION_H_
