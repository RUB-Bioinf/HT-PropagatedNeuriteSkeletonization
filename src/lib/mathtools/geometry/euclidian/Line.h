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
 *  \file  Line.h
 *  \brief Defines a line in an euclidian space
 *  \author Bastien Durix
 */

#ifndef _LINE_H_
#define _LINE_H_

#include <Eigen/Dense>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

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
			 * \brief Defines a line in an euclidian space
			 *
			 *  \tparam Dim dimension of the space in which the line is
			 */
			template<unsigned int Dim>
			class Line
			{
				protected:
					/**
					 *  \brief Origin of the line
					 */
					affine::Point<Dim> m_origin;

					/**
					 *  \brief Line vector
					 */
					Eigen::Matrix<double,Dim,1> m_vec_dir;
				public:
					/**
					 *  \brief Constructor
					 *
					 *  \param origin     Line origin
					 *  \param vec_dir    Line vector
					 */
					Line(const affine::Point<Dim> &origin = affine::Point<Dim>(), const Eigen::Matrix<double,Dim,1> &vec_dir = Eigen::Matrix<double,Dim,1>::Zero()) :
						m_origin(origin), m_vec_dir(vec_dir)
					{};

					/**
					 *  \brief Constructor
					 *
					 *  \param origin     Line origin
					 *  \param vec_dir    Line vector
					 */
					Line(const Eigen::Matrix<double,Dim,1> &origin, const Eigen::Matrix<double,Dim,1> &vec_dir) :
						m_origin(origin), m_vec_dir(vec_dir)
					{};

					/**
					 *  \brief Const origin accessor
					 *
					 *  \return Line origin
					 */
					const affine::Point<Dim>& getOrigin() const
					{
						return m_origin;
					}

					/**
					 *  \brief Const director vector accessor
					 *
					 *  \return Line vec_dir
					 */
					const Eigen::Matrix<double,Dim,1>& getVecDir() const
					{
						return m_vec_dir;
					}
			};
		}
	}
}

#endif //_LINE_H_

