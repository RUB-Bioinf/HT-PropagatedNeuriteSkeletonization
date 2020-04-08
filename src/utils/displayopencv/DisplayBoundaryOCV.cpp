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
 *  \file DisplayBoundaryOCV.cpp
 *  \brief Displays boundary with opencv
 *  \author Bastien Durix
 */

#include "DisplayBoundaryOCV.h"

#include <opencv2/imgproc/imgproc.hpp>

void displayopencv::DisplayDiscreteBoundary(const boundary::DiscreteBoundary<2>::Ptr disbnd, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	std::vector<mathtools::affine::Point<2> > vec_pt;
	disbnd->getVerticesPoint(vec_pt);

	std::vector<Eigen::Vector2d> vec_pt_cvt(vec_pt.size());

	for(unsigned int i = 0; i < vec_pt.size(); i++)
	{
		vec_pt_cvt[i] = vec_pt[i].getCoords(frame);
	}

	for(unsigned int ind1 = 0; ind1 < vec_pt.size(); ind1++)
	{
		unsigned int ind2 = disbnd->getNext(ind1);
		cv::Point pt1(vec_pt_cvt[ind1].x(),vec_pt_cvt[ind1].y());
		cv::Point pt2(vec_pt_cvt[ind2].x(),vec_pt_cvt[ind2].y());
		
		cv::line(img,pt1,pt2,color);
	}
}
