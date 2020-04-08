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
 *  \file ReprojError.cpp
 *  \brief Computes the reprojection error of a 3d reconstructed skeleton
 *  \author Bastien Durix
 */

#include "ReprojError.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

double algorithm::evaluation::SymDiffArea(const shape::DiscreteShape<2>::Ptr &shpref, const shape::DiscreteShape<2>::Ptr &shpcmp)
{
	cv::Mat img(shpref->getHeight(),shpref->getWidth(),CV_8U);
	cv::Mat im_shape(shpref->getHeight(),shpref->getWidth(),CV_8U,&shpref->getContainer()[0]);
	cv::Mat im_proj(shpcmp->getHeight(),shpcmp->getWidth(),CV_8U,&shpcmp->getContainer()[0]);
	cv::absdiff(im_shape,im_proj,img);

	double dist = cv::sum(img)(0)/cv::sum(im_shape)(0);

	return dist;
}

double algorithm::evaluation::HausDist(const skeleton::GraphSkel2d::Ptr grskl, const boundary::DiscreteBoundary<2>::Ptr disbnd, const mathtools::affine::Frame<2>::Ptr frame)
{
	std::list<unsigned int> lnod;
	std::list<mathtools::geometry::euclidian::HyperSphere<2> > lnodcir;
	grskl->getAllNodes(lnod);
	grskl->getNodes<mathtools::geometry::euclidian::HyperSphere<2>,std::list<unsigned int>,std::list<mathtools::geometry::euclidian::HyperSphere<2> > >(lnod,lnodcir);
	double distmax = 0.0;
#pragma omp parallel for
	for(unsigned int i = 0; i < disbnd->getNbVertices(); i++)
	{
		Eigen::Vector2d pt = disbnd->getVertex(i).getCoords();
		double distcurmin = -1.0;
		for(std::list<mathtools::geometry::euclidian::HyperSphere<2> >::iterator it = lnodcir.begin(); it != lnodcir.end() && distcurmin != 0.0; it++)
		{
			Eigen::Vector2d ctr = it->getCenter().getCoords(frame);
			
			double distpt = (pt - ctr).norm();
			
			double distcur = 0.0;

			if(distpt > it->getRadius())
				distcur = distpt - it->getRadius();
			
			if(distcur < distcurmin || distcurmin == -1.0)
			{
				distcurmin = distcur;
			}
		}
#pragma omp critical
		{
			if(distcurmin > distmax)
				distmax = distcurmin;
		}
	}
	return distmax;
}
