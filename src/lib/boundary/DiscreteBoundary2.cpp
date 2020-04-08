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
 *  \file  DiscreteBoundary2.cpp
 *  \brief Defines 2d discrete boundary of a shape
 *  \author Bastien Durix
 */

#include "DiscreteBoundary2.h"

using namespace boundary;

boundary::DiscreteBoundary<2>::DiscreteBoundary(const mathtools::affine::Frame<2>::Ptr frame) : m_frame(frame), m_vecvert(0) {}

const mathtools::affine::Frame<2>::Ptr boundary::DiscreteBoundary<2>::getFrame() const
{
	return m_frame;
}

unsigned int boundary::DiscreteBoundary<2>::getNext(unsigned int index) const
{
	std::map<unsigned int,unsigned int>::const_iterator it = m_neigh.find(index);
	if(it == m_neigh.end())
		throw std::logic_error("boundary::DiscreteBoundary<2>::getNext: index is not in the skeleton");
	return it->second;
}

unsigned int boundary::DiscreteBoundary<2>::getPrev(unsigned int index) const
{
	std::map<unsigned int,unsigned int>::const_iterator it = m_neigh_prev.find(index);
	if(it == m_neigh_prev.end())
		throw std::logic_error("boundary::DiscreteBoundary<2>::getPrev: index is not in the skeleton");
	return it->second;
}

mathtools::affine::Point<2> boundary::DiscreteBoundary<2>::getVertex(unsigned int index) const
{
	if(index >= m_vecvert.size()) throw std::logic_error("boundary::DiscreteBoundary<2>::getVertex : index out of bounds");
	return mathtools::affine::Point<2>(m_vecvert[index]);
}

Eigen::Vector2d boundary::DiscreteBoundary<2>::getCoordinates(unsigned int index) const
{
	if(index >= m_vecvert.size()) throw std::logic_error("boundary::DiscreteBoundary<2>::getCoordinates : index out of bounds");
	return m_vecvert[index];
}

unsigned int boundary::DiscreteBoundary<2>::getNbVertices() const
{
	return m_vecvert.size();
}
