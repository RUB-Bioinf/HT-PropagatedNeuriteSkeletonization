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
 *  \file DiscreteShape.h
 *  \brief Defines discrete shape
 *  \author Bastien Durix
 */

#ifndef _DISCRETESHAPE_H_
#define _DISCRETESHAPE_H_

#include <vector>
#include <memory>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

/**
 *  \brief Defines shape tools
 */
namespace shape
{
	/**
	 *  \brief Discrete shape
	 *
	 *  \tparam Dim espace dimension
	 */
	template<unsigned int Dim>
	class DiscreteShape
	{};

	/**
	 *  \brief Discrete shape in dimension 2
	 */
	template<>
	class DiscreteShape<2>
	{
		public:
			/**
			 *  \brief Boundary shared pointer
			 */
			using Ptr = std::shared_ptr<DiscreteShape<2> >;

		protected:
			/**
			 *  \brief Shape frame
			 */
			typename mathtools::affine::Frame<2>::Ptr m_frame;

			/**
			 *  \brief Vector containing discrete shape data, row wise
			 */
			std::vector<unsigned char> m_disc;

			/**
			 *  \brief Shape width
			 */
			unsigned int m_width;

			/**
			 *  \brief Shape height
			 */
			unsigned int m_height;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param width  width of the discrete shape
			 *  \param height height of the discrete shape
			 *  \param frame  frame of the discrete shape
			 */
			DiscreteShape<2>(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame());

			/**
			 *  \brief Test if a point is in the discrete shape
			 *
			 *  \param point point to test
			 *
			 *  \return true is the point is in the discrete shape
			 */
			virtual bool isIn(const mathtools::affine::Point<2> &point) const;

			/**
			 *  \brief Frame getter
			 *
			 *  \return frame of the shape
			 */
			const typename mathtools::affine::Frame<2>::Ptr getFrame() const;

			/**
			 *  \brief Width getter
			 *
			 *  \return width of the shape
			 */
			unsigned int getWidth() const;

			/**
			 *  \brief Height getter
			 *
			 *  \return height of the shape
			 */
			unsigned int getHeight() const;

			/**
			 *  \brief Container getter
			 *
			 *  \return container of the shape
			 */
			const std::vector<unsigned char>& getContainer() const;

			/**
			 *  \brief Container getter
			 *
			 *  \return container of the shape
			 */
			std::vector<unsigned char>& getContainer();
	};
}

#endif //_DISCRETESHAPE_H_
