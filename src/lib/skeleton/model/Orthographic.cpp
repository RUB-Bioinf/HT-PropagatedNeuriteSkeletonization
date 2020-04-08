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
 *  \file Orthographic.cpp
 *  \brief Defines orthographic projective skeleton
 *  \author Bastien Durix
 */

#include "Orthographic.h"
#include <mathtools/application/Compositor.h>
#include <mathtools/application/LinearApp.h>
#include <mathtools/application/Coord2Homog.h>

using namespace mathtools::application;

constexpr unsigned int skeleton::model::meta<skeleton::model::Orthographic>::stordim;

skeleton::model::Orthographic::Orthographic(const mathtools::affine::Frame<2>::Ptr frame2, const mathtools::affine::Frame<3>::Ptr frame3) : skeleton::model::Projective(frame2,frame3)
{
	Eigen::Matrix3d basis3 = frame3->getBasis()->getMatrix();
	Eigen::Vector3d ori3   = frame3->getOrigin();
	Eigen::Matrix<double,8,4> retromat;
	retromat << basis3(0,0)  , basis3(0,1)  , 0.0  , ori3(0)	 ,
				basis3(1,0)  , basis3(1,1)  , 0.0  , ori3(1)	 ,
				basis3(2,0)  , basis3(2,1)  , 0.0  , ori3(2)	 ,
				0.0 		 , 0.0  	    , 1.0  , 0.0     	 ,
				0.0			 , 0.0 		    , 0.0  , basis3(0,2) ,
				0.0 		 , 0.0  	    , 0.0  , basis3(1,2) ,
				0.0 		 , 0.0  	    , 0.0  , basis3(2,2) ,
				0.0			 , 0.0		    , 0.0  , 0.0         ;
	
	Compositor<LinearApp<8,4>,Coord2Homog<3> >::Ptr comp(new Compositor<LinearApp<8,4>,Coord2Homog<3> >(LinearApp<8,4>(retromat),Coord2Homog<3>()));
	
	m_r8fun = std::static_pointer_cast<Application<Eigen::Matrix<double,8,1>,Eigen::Vector3d> >(comp);
}

skeleton::model::Orthographic::Orthographic(const Orthographic &model) : skeleton::model::Orthographic(model.m_frame2,model.m_frame3)
{}

skeleton::model::Orthographic::Type skeleton::model::Orthographic::getType() const
{
	return skeleton::model::Orthographic::Type::orthographic;
}

double skeleton::model::Orthographic::getSize(const Eigen::Matrix<double,meta<Orthographic>::stordim,1> &vec) const
{
	return vec(2);
}

Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Orthographic>::stordim,1> skeleton::model::Orthographic::resize(const Eigen::Matrix<double,meta<Orthographic>::stordim,1> &vec, double size) const
{
	Eigen::Matrix<double,meta<Orthographic>::stordim,1> resized = vec;
	resized(meta<Orthographic>::stordim-1,0) *= size;
	return resized;
}

bool skeleton::model::Orthographic::included(const Eigen::Matrix<double,meta<Orthographic>::stordim,1> &vec1, const Eigen::Matrix<double,meta<Orthographic>::stordim,1> &vec2) const
{
	return (vec1.template block<meta<Orthographic>::stordim-1,1>(0,0) - vec2.template block<meta<Orthographic>::stordim-1,1>(0,0)).norm()
		+ vec2(meta<Orthographic>::stordim-1,0)
		<=
		vec1(meta<Orthographic>::stordim-1,0);
}

mathtools::affine::Point<2> skeleton::model::Orthographic::toObj(
		const Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Projective>::stordim,1> &vec,
		const mathtools::affine::Point<2> &) const
{
	return mathtools::affine::Point<2>(vec(0),vec(1),m_frame2);
}

mathtools::geometry::euclidian::HyperEllipse<2> skeleton::model::Orthographic::toObj(
		const Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Projective>::stordim,1> &vec,
		const mathtools::geometry::euclidian::HyperEllipse<2> &) const
{
	mathtools::affine::Point<2> pt(vec(0),vec(1),m_frame2);
	
	Eigen::Matrix2d mat;
	mat << 1.0/vec(2) , 0.0,
		   0.0 , 1.0/vec(2);
	
	Eigen::Matrix2d axes = m_frame2->getBasis()->getMatrix()*mat*m_frame2->getBasis()->getMatrixInverse();

	return mathtools::geometry::euclidian::HyperEllipse<2>(pt,axes);
}

mathtools::geometry::euclidian::Line<4> skeleton::model::Orthographic::toObj(
		const Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Projective>::stordim,1> &vec,
		const mathtools::geometry::euclidian::Line<4> &) const
{
	Eigen::Vector4d origin;
	origin.block<3,1>(0,0) = m_frame3->getBasis()->getMatrix()*Eigen::Vector3d(vec(0),vec(1),0.0) + m_frame3->getOrigin();
	origin(3) = vec(2);

	Eigen::Vector4d vecdir;
	vecdir.block<3,1>(0,0) = m_frame3->getBasis()->getMatrix()*Eigen::Vector3d(0.0,0.0,1.0);
	vecdir(3) = 0.0;
	vecdir.normalize();

	return mathtools::geometry::euclidian::Line<4>(origin,vecdir);
}
