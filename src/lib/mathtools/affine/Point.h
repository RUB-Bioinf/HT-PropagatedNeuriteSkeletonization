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
 *  \file  Point.h
 *  \brief Defines a point in an affine space
 *  \author Bastien Durix
 */

#ifndef _POINT_H_
#define _POINT_H_

#include <Eigen/Dense>
#include "Frame.h"

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Affine tools
	 */
	namespace affine
	{
		/**
		 *  \brief Point in an affine space
		 *
		 *  \tparam Dimension of the affine space
		 */
		template<unsigned int Dim>
		class Point
		{
			protected:
				/**
				 *  \brief Coordinates in canonic frame
				 */
				Eigen::Matrix<double,Dim,1> m_coords;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param coords coordinates of the point
				 *  \param frame  frame in which the coordinates are expressed
				 */
				Point(const Eigen::Matrix<double,Dim,1> &coords = Eigen::Matrix<double,Dim,1>::Zero(), const typename Frame<Dim>::Ptr &frame = Frame<Dim>::CanonicFrame())
				{
					m_coords = frame->getBasis()->getMatrix() * coords + frame->getOrigin();
				}

				/**
				 *  \brief Constructor
				 *
				 *  \param x      x-coordinate of the point
				 *  \param y      y-coordinate of the point
				 *  \param frame  frame in which the coordinates are expressed
				 */
				Point(double x, double y, const typename Frame<Dim>::Ptr &frame = Frame<Dim>::CanonicFrame()) : Point(Eigen::Vector2d(x,y),frame)
				{}

				/**
				 *  \brief Constructor
				 *
				 *  \param x      x-coordinate of the point
				 *  \param y      y-coordinate of the point
				 *  \param z      z-coordinate of the point
				 *  \param frame  frame in which the coordinates are expressed
				 */
				Point(double x, double y, double z, const typename Frame<Dim>::Ptr &frame = Frame<Dim>::CanonicFrame()) : Point(Eigen::Vector3d(x,y,z),frame)
				{}

				/**
				 *  \brief Coordinates getter
				 *
				 *  \param frame frame in which get the coordinates
				 *
				 *  \return coordinates vector
				 */
				inline const Eigen::Matrix<double,Dim,1> getCoords(const typename Frame<Dim>::Ptr &frame) const
				{
					return frame->getBasis()->getMatrixInverse() * (m_coords - frame->getOrigin());
				}

				/**
				 *  \brief Coordinates getter in canonic frame
				 *
				 *  \return coordinates vector
				 */
				inline const Eigen::Matrix<double,Dim,1>& getCoords() const
				{
					return m_coords;
				}

				/**
				 *  \brief Translation operator
				 *
				 *  \param vec translation vector in canonic frame
				 *
				 *  \return result point of the translation
				 */
				inline const Point<Dim> operator+(const Eigen::Matrix<double,Dim,1> &vec) const
				{
					return Point<Dim>(m_coords + vec);
				}

				/**
				 *  \brief Translation getter
				 *
				 *  \param point origin of the translation vector
				 *
				 *  \return translation vector in canonic frame
				 */
				inline const Eigen::Matrix<double,Dim,1> operator-(const Point<Dim> &point) const
				{
					return m_coords - point.m_coords;
				}
		};
	}
}




#endif //_POINT_H_
