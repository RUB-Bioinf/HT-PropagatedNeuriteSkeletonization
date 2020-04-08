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
 *  \file DisplaySkeletonOCV.cpp
 *  \brief Displays skeleton with opencv
 *  \author Bastien Durix
 */

#include <opencv2/imgproc/imgproc.hpp>
#include "DisplaySkeletonOCV.h"

template<typename Model>
void DisplayGraphSkeleton_helper(const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	std::list<std::pair<unsigned int,unsigned int> > edges;
	grskel->getAllEdges(edges);
	for(std::list<std::pair<unsigned int,unsigned int> >::iterator it = edges.begin(); it != edges.end(); it++)
	{
		Eigen::Vector2d vec1 = grskel->template getNode<mathtools::affine::Point<2> >(it->first).getCoords(frame);
		Eigen::Vector2d vec2 = grskel->template getNode<mathtools::affine::Point<2> >(it->second).getCoords(frame);
		
		cv::Point pt1(vec1.x(),vec1.y());
		cv::Point pt2(vec2.x(),vec2.y());

		cv::line(img,pt1,pt2,color);
	}
}

void displayopencv::DisplayGraphSkeleton(const skeleton::GraphSkel2d::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	DisplayGraphSkeleton_helper<skeleton::model::Classic<2> >(grskel,img,frame,color);
}

void displayopencv::DisplayGraphSkeleton(const skeleton::GraphProjSkel::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	DisplayGraphSkeleton_helper<skeleton::model::Projective>(grskel,img,frame,color);
}

void displayopencv::DisplayFilledGraphSkeleton(const skeleton::GraphSkel2d::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	std::list<unsigned int> nods;
	grskel->getAllNodes(nods);
	for(std::list<unsigned int>::iterator it = nods.begin(); it != nods.end(); it++)
	{
		mathtools::geometry::euclidian::HyperSphere<2> cir1 = grskel->getNode<mathtools::geometry::euclidian::HyperSphere<2> >(*it);
		Eigen::Vector2d pt = cir1.getCenter().getCoords(frame);
		
		cv::circle(img,cv::Point2i((int)pt.x(),(int)pt.y()),(int)cir1.getRadius(),color,-1);
		std::cout.precision(5);
		std::cout << (int)pt.x() << " " << (int)pt.y() << " " << cir1.getRadius() << std::endl;
	}
}

void displayopencv::DisplayContBranch(const skeleton::BranchContProjSkel::Ptr contbr, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	for(unsigned int i =0; i < 100; i++)
	{
		double t1 = (double)i/100.0;
		double t2 = (double)(i+1)/100.0;

		Eigen::Vector2d vec1 = contbr->getNode<mathtools::affine::Point<2> >(t1).getCoords(frame);
		Eigen::Vector2d vec2 = contbr->getNode<mathtools::affine::Point<2> >(t2).getCoords(frame);
		
		cv::Point pt1(vec1.x(),vec1.y());
		cv::Point pt2(vec2.x(),vec2.y());

		cv::line(img,pt1,pt2,color);
	}
}
