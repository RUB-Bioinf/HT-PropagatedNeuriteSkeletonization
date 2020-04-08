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
 *  \file Projective.h
 *  \brief Defines skeleton projective model
 *  \author Bastien Durix
 */

#ifndef _PROJECTIVEMODEL_H_
#define _PROJECTIVEMODEL_H_

#include <memory>
#include "MetaModel.h"
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>
#include <mathtools/geometry/euclidian/HyperSphere.h>
#include <mathtools/geometry/euclidian/HyperEllipse.h>
#include <mathtools/geometry/euclidian/Line.h>
#include <mathtools/application/Application.h>

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Skeletal models
	 */
	namespace model
	{
		class Projective;
		
		/**
		 *  \brief Describe Projective model meta data
		 */
		template<>
		struct meta<Projective>
		{
			/**
			 *  \brief Describe storage dimension of the model
			 */
			static constexpr unsigned int stordim = 3;
		};

		
		/**
		 *  \brief Describe projective skeleton model
		 */
		class Projective
		{
			public:
				/**
				 *  \brief Shared pointer to the model
				 */
				using Ptr = std::shared_ptr<Projective>;

				/**
				 *  \brief Storage type
				 */
				using Stor = Eigen::Matrix<double,meta<Projective>::stordim,1>;
				
				/**
				 *  \brief Projective skeleton type
				 */
				enum class Type
				{
					perspective,
					orthographic
				};

			protected:
				/**
				 *  \brief 2D frame of the skeleton
				 */
				mathtools::affine::Frame<2>::Ptr m_frame2;

				/**
				 *  \brief 3D frame of the skeleton
				 */
				mathtools::affine::Frame<3>::Ptr m_frame3;

				/**
				 *  \brief Conversion function to R^4 vector (representing a line in R^4)
				 */
				mathtools::application::Application<Eigen::Matrix<double,8,1>,Eigen::Vector3d>::Ptr m_r8fun;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param frame2 skeleton 2d frame
				 *  \param frame3 skeleton 3d frame
				 */
				Projective(const mathtools::affine::Frame<2>::Ptr frame2 = mathtools::affine::Frame<2>::CanonicFrame(),
						   const mathtools::affine::Frame<3>::Ptr frame3 = mathtools::affine::Frame<3>::CanonicFrame());

				/**
				 *  \brief 2d frame getter
				 *
				 *  \return skeleton 2d frame
				 */
				const mathtools::affine::Frame<2>::Ptr getFrame2() const;

				/**
				 *  \brief 3d frame getter
				 *
				 *  \return skeleton 3d frame
				 */
				const mathtools::affine::Frame<3>::Ptr getFrame3() const;
				
				/**
				 *  \brief R8 conversion function getter
				 *
				 *  \return R8 conversion function
				 */
				const mathtools::application::Application<Eigen::Matrix<double,8,1>,Eigen::Vector3d>::Ptr getR8Fun() const;

				/**
				 *  \brief Converts an object into a vector
				 *
				 *  \tparam TypeObj type of the object to convert
				 *
				 *  \param obj object to convert
				 *
				 *  \return vector associated to obj
				 */
				template<typename TypeObj>
				Eigen::Matrix<double,meta<Projective>::stordim,1> toVec(const TypeObj &obj) const
				{
					return toVec(obj);
				}

				/**
				 *  \brief Converts a vector into an object
				 *
				 *  \tparam TypeObj type of the object to convert in
				 *
				 *  \param vec vector to convert
				 *
				 *  \return object associated to vec
				 */
				template<typename TypeObj>
				TypeObj toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const
				{
					return toObj(vec,TypeObj{});
				}
				
				/**
				 *  \brief Computes an object on continusou branch using derivatives
				 *
				 *  \tparam TypeObj type of the object to convert in
				 *
				 *  \param appli  application to use
				 *  \param t      parameter value
				 *
				 *  \return object associated to appli at t
				 */
				template<typename TypeObj>
				TypeObj toObj(const typename mathtools::application::Application<Eigen::Matrix<double,meta<Projective>::stordim,1>,double>::Ptr appli,
							  const double t) const
				{
					return toObj(appli,t,TypeObj{});
				}

				/**
				 *  \brief Skeleton type getter
				 *
				 *  \return Skeleton type
				 */
				virtual Type getType() const = 0;

				/**
				 *  \brief Size getter (used in nodes comparison)
				 *
				 *  \param vec voctor to evaluate the size
				 *
				 *  \return size associated to vec
				 */
				virtual double getSize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const = 0;

				/**
				 *  \brief Resize an object from its vector
				 *
				 *  \param vec  vector of object to resize
				 *  \param size new relative size
				 *
				 *  \return resized vector
				 */
				virtual Eigen::Matrix<double,meta<Projective>::stordim,1> resize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec, double size) const = 0;

				/**
				 *  \brief Tests if a object is included into another one
				 *
				 *  \param vec1 first object vector
				 *  \param vec2 second object vector
				 *
				 *  \return true is first object is in second object
				 */
				virtual bool included(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec1, const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec2) const = 0;

			protected:
				/**
				 *  \brief Conversion function from hypersphere to vector
				 *
				 *  \param sph hypersphere to convert
				 *
				 *  \return vector associated to hyperpshere
				 */
				virtual Eigen::Matrix<double,meta<Projective>::stordim,1> toVec(const mathtools::geometry::euclidian::HyperSphere<2> &sph) const;

				/**
				 *  \brief Associate the center of the sphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return center associated to vec
				 */
				virtual mathtools::affine::Point<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
														  const mathtools::affine::Point<2> &) const;

				/**
				 *  \brief Associate an hypersphere to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return hypersphere
				 */
				virtual mathtools::geometry::euclidian::HyperSphere<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
																			 const mathtools::geometry::euclidian::HyperSphere<2> &) const;

				/**
				 *  \brief Associate an hyperellipse to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return hyperellipse
				 */
				virtual mathtools::geometry::euclidian::HyperEllipse<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
																			  const mathtools::geometry::euclidian::HyperEllipse<2> &) const;

				/**
				 *  \brief Associate a line to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return line
				 */
				virtual mathtools::geometry::euclidian::Line<4> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
																	  const mathtools::geometry::euclidian::Line<4> &) const;

				/**
				 *  \brief Associate a 8d-vector to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return 8d-vector
				 */
				virtual Eigen::Matrix<double,8,1> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
														const Eigen::Matrix<double,8,1>&) const;

				/**
				 *  \brief Associate a derivative 8d-vector to a parameter value
				 *
				 *  \param appli  application describing the branch
				 *  \param t      parameter value
				 *
				 *  \return derivative 8d-vector
				 */
				virtual Eigen::Matrix<double,8,1> toObj(
														const typename mathtools::application::Application<Eigen::Matrix<double,meta<Projective>::stordim,1>,double>::Ptr appli,
														const double t,
														const Eigen::Matrix<double,8,1>&) const;

		};
	}
}


#endif //_PROJECTIVEMODEL_H_
