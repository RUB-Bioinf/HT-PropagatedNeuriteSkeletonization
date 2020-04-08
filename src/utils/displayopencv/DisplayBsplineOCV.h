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
 *  \file DisplayBspline.h
 *  \brief Displays bspline with opencv
 *  \author Bastien Durix
 */

#ifndef _DISPLAYBSPLINEOCV_H_
#define _DISPLAYBSPLINEOCV_H_

#include <opencv2/core/core.hpp>

#include <mathtools/affine/Frame.h>
#include <mathtools/application/Bspline.h>

/**
 *  \brief Display with opencv functions
 */
namespace displayopencv
{
	/**
	 *  \brief Displays Bspline with opencv
	 *  
	 *  \param bspline  bspline to display
	 *  \param img      image where display the Bspline
	 *  \param frame    image frame
	 *  \param color    diplay color of the Bspline
	 */
	void DisplayBspline(const mathtools::application::Bspline<2> &bspline, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color, const unsigned int &nb_pts = 100);
}


#endif //_DISPLAYBSPLINEOCV_H_


