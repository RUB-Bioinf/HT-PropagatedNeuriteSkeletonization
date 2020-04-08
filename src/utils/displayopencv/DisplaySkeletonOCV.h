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
 *  \file DisplaySkeletonOCV.h
 *  \brief Displays skeleton with opencv
 *  \author Bastien Durix
 */

#ifndef _DISPLAYSKELETONOCV_H_
#define _DISPLAYSKELETONOCV_H_

#include <opencv2/core/core.hpp>

#include <skeleton/Skeletons.h>
#include <mathtools/affine/Frame.h>

/**
 *  \brief Display with opencv functions
 */
namespace displayopencv
{
	/**
	 *  \brief Displays classic skeleton with opencv
	 *  
	 *  \param grskel Skeleton to display
	 *  \param img    Image where display the skeleton
	 *  \param frame  Image frame
	 *  \param color  Diplay color of the skeleton
	 */
	void DisplayGraphSkeleton(const skeleton::GraphSkel2d::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color);

	/**
	 *  \brief Displays projective skeleton with opencv
	 *  
	 *  \param grskel Skeleton to display
	 *  \param img    Image where display the skeleton
	 *  \param frame  Image frame
	 *  \param color  Diplay color of the skeleton
	 */
	void DisplayGraphSkeleton(const skeleton::GraphProjSkel::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color);

	/**
	 *  \brief Displays filled circles of classic skeleton with opencv
	 *  
	 *  \param grskel Skeleton to display
	 *  \param img    Image where display the skeleton
	 *  \param frame  Image frame
	 *  \param color  Diplay color of the skeleton
	 */
	void DisplayFilledGraphSkeleton(const skeleton::GraphSkel2d::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color);
	
	void DisplayContBranch(const skeleton::BranchContProjSkel::Ptr contbr, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color);
}

#endif //_DISPLAYSKELETONOCV_H_
