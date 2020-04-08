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
 *  \file DisplayShapeOCV.cpp
 *  \brief Displays shape with opencv
 *  \author Bastien Durix
 */

#include "DisplayShapeOCV.h"

void displayopencv::DisplayDiscreteShape(const shape::DiscreteShape<2>::Ptr dissh, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	#pragma omp parallel for
	for(int x=0;x<img.cols;x++)
	{
		#pragma omp parallel for
		for(int y=0;y<img.rows;y++)
		{
			if(dissh->isIn(mathtools::affine::Point<2>(x,y,frame)))
				img.at<cv::Vec3b>(y,x) = cv::Vec3b(color[0],color[1],color[2]);
		}
	}
}
