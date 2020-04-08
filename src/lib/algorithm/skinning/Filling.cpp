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
 *  \file Filling.h
 *  \brief Computes a discrete shape associated to a continuous skeleton
 *  \author Bastien Durix
 */

#include "Filling.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mathtools/geometry/euclidian/HyperSphere.h>
#include <mathtools/affine/Point.h>

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::GraphSkel2d::Ptr grskl)
{
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	
	std::list<unsigned int> lind;
	grskl->getAllNodes(lind);

	for(std::list<unsigned int>::iterator it = lind.begin(); it != lind.end(); it++)
	{
		mathtools::geometry::euclidian::HyperSphere<2> sph = grskl->getNode<mathtools::geometry::euclidian::HyperSphere<2> >(*it);
		
    	cv::circle(im_shape,cv::Point2i(sph.getCenter().getCoords(shape->getFrame()).x()+0.5,sph.getCenter().getCoords(shape->getFrame()).y()+0.5),sph.getRadius(),255,-1);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::BranchContSkel2d::Ptr contbr, const OptionsFilling &options)
{
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	
	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);
		mathtools::geometry::euclidian::HyperSphere<2> sph = contbr->getNode<mathtools::geometry::euclidian::HyperSphere<2> >(t);
		
    	cv::circle(im_shape,cv::Point2i(sph.getCenter().getCoords(shape->getFrame()).x()+0.5,sph.getCenter().getCoords(shape->getFrame()).y()+0.5),sph.getRadius(),255,-1);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::CompContSkel2d::Ptr contskl, const OptionsFilling &options)
{
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);
	
	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = contskl->getExtremities(edge[i]);
		
		Filling(shape,contskl->getBranch(ext.first,ext.second),options);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::GraphProjSkel::Ptr grskl)
{
	cv::Mat im_shapetimes2(shape->getHeight()*2,shape->getWidth()*2,CV_8U,cv::Scalar(0));
	
	std::list<unsigned int> lind;
	grskl->getAllNodes(lind);

	for(std::list<unsigned int>::iterator it = lind.begin(); it != lind.end(); it++)
	{
		mathtools::geometry::euclidian::HyperEllipse<2> ell = grskl->getNode<mathtools::geometry::euclidian::HyperEllipse<2> >(*it);
		
		Eigen::Vector2d vec_ctr = ell.getCenter().getCoords(shape->getFrame());

		cv::RotatedRect rect(cv::Point2f(vec_ctr.x()*2.0,vec_ctr.y()*2.0),
							 cv::Size2f(2.0*2.0*ell.getAxes().block<2,1>(0,0).norm(),2.0*2.0*ell.getAxes().block<2,1>(0,1).norm()),
							 atan2(ell.getAxes()(1,0),ell.getAxes()(0,0))*180/M_PI);
		
		cv::ellipse(im_shapetimes2,rect,255,-1);
	}
	
	cv::Mat imtmp(shape->getHeight(),shape->getWidth(),CV_8U,cv::Scalar(0));
	cv::resize(im_shapetimes2,imtmp,cv::Size(shape->getWidth(),shape->getHeight()));
	
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	imtmp.copyTo(im_shape);
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::BranchContProjSkel::Ptr contbr, const OptionsFilling &options)
{
	cv::Mat im_shapetimes2(shape->getHeight()*2,shape->getWidth()*2,CV_8U,cv::Scalar(0));

	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);
		mathtools::geometry::euclidian::HyperEllipse<2> ell = contbr->getNode<mathtools::geometry::euclidian::HyperEllipse<2> >(t);
		
		Eigen::Vector2d vec_ctr = ell.getCenter().getCoords(shape->getFrame());

		cv::RotatedRect rect(cv::Point2f(vec_ctr.x()*2.0,vec_ctr.y()*2.0),
							 cv::Size2f(2.0*2.0*ell.getAxes().block<2,1>(0,0).norm(),2.0*2.0*ell.getAxes().block<2,1>(0,1).norm()),
							 atan2(ell.getAxes()(1,0),ell.getAxes()(0,0))*180/M_PI);
		
		cv::ellipse(im_shapetimes2,rect,255,-1);
	}
	
	cv::Mat imtmp(shape->getHeight(),shape->getWidth(),CV_8U,cv::Scalar(0));
	cv::resize(im_shapetimes2,imtmp,cv::Size(shape->getWidth(),shape->getHeight()));
	
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	imtmp.copyTo(im_shape);
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::CompContProjSkel::Ptr contskl, const OptionsFilling &options)
{
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);
	
	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = contskl->getExtremities(edge[i]);
		
		Filling(shape,contskl->getBranch(ext.first,ext.second),options);
	}
}
