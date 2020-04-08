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
 *  \file DiscreteShape2.cpp
 *  \brief Defines discrete shape in dimension 2
 *  \author Bastien Durix
 */

#include "DiscreteShape.h"

using namespace shape;

shape::DiscreteShape<2>::DiscreteShape(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame) :
	m_frame(frame), m_disc(width*height,0), m_width(width), m_height(height)
{}

bool shape::DiscreteShape<2>::isIn(const mathtools::affine::Point<2> &point) const
{
	Eigen::Vector2d coords = point.getCoords(m_frame);
	
	bool isin=false;
	
	if(coords.x() >= 0 && coords.y() >= 0 && coords.x() < m_width && coords.y() < m_height)
	{
		unsigned int ind = (unsigned int)coords.x() + m_width * (unsigned int)coords.y();

		if(m_disc[ind]) isin = true;
	}

	return isin;
}


const typename mathtools::affine::Frame<2>::Ptr shape::DiscreteShape<2>::getFrame() const
{
	return m_frame;
}

unsigned int shape::DiscreteShape<2>::getWidth() const
{
	return m_width;
}

unsigned int shape::DiscreteShape<2>::getHeight() const
{
	return m_height;
}

const std::vector<unsigned char>& shape::DiscreteShape<2>::getContainer() const
{
	return m_disc;
}

std::vector<unsigned char>& shape::DiscreteShape<2>::getContainer()
{
	return m_disc;
}
