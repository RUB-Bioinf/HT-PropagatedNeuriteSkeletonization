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
 *  \file ReprojError.h
 *  \brief Computes the reprojection error of a 3d reconstructed skeleton
 *  \author Bastien Durix
 */

#ifndef _REPROJERROR_H_
#define _REPROJERROR_H_

#include <skeleton/Skeletons.h>
#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary2.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Evaluation algorithms
	 */
	namespace evaluation
	{
		double SymDiffArea(const shape::DiscreteShape<2>::Ptr &shpref, const shape::DiscreteShape<2>::Ptr &shpcmp);

		double HausDist(const skeleton::GraphSkel2d::Ptr grskl, const boundary::DiscreteBoundary<2>::Ptr disbnd, const mathtools::affine::Frame<2>::Ptr frame);
	}
}

#endif //_REPROJERROR_H_
