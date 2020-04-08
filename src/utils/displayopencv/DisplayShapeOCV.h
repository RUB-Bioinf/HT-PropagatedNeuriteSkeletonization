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
 *  \file DisplayShapeOCV.h
 *  \brief Displays shape with opencv
 *  \author Bastien Durix
 */

#ifndef _DISPLAYSHAPEOCV_H_
#define _DISPLAYSHAPEOCV_H_

#include <shape/DiscreteShape.h>
#include <mathtools/affine/Frame.h>
#include <opencv2/core/core.hpp>

/**
 *  \brief Display with opencv functions
 */
namespace displayopencv
{
	/**
	 *  \brief Displays shape with opencv
	 *  
	 *  \param dissh Shape to display
	 *  \param img   Image where display the shape
	 *  \param frame Image frame
	 *  \param color Diplay color of the shape
	 */
	void DisplayDiscreteShape(const shape::DiscreteShape<2>::Ptr dissh, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color);
}


#endif //_DISPLAYSHAPEOCV_H_
