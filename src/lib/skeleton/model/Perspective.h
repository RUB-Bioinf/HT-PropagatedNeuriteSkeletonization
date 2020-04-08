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
 *  \file Perspective.h
 *  \brief Defines perspective projective skeleton
 *  \author Bastien Durix
 */

#include <memory>
#include "MetaModel.h"
#include "Projective.h"

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
		class Perspective;
		
		/**
		 *  \brief Describe Perspective model meta data
		 */
		template<>
		struct meta<Perspective>
		{
			/**
			 *  \brief Describe storage dimension of the model
			 */
			static constexpr unsigned int stordim = 3;
		};


		/**
		 *  \brief Describe perspective skeleton model
		 */
		class Perspective : public Projective
		{
			public:
				/**
				 *  \brief Shared pointer to the model
				 */
				using Ptr = std::shared_ptr<Perspective>;

				/**
				 *  \brief Storage type
				 */
				using Stor = Eigen::Matrix<double,meta<Perspective>::stordim,1>;
				
			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param frame2 skeleton 2d frame
				 *  \param frame3 skeleton 3d frame
				 */
				Perspective(const mathtools::affine::Frame<2>::Ptr frame2 = mathtools::affine::Frame<2>::CanonicFrame(),
						    const mathtools::affine::Frame<3>::Ptr frame3 = mathtools::affine::Frame<3>::CanonicFrame());

				/**
				 *  \brief Copy constructor
				 *
				 *  \param model model to copy
				 */
				Perspective(const Perspective &model);

				/**
				 *  \brief Skeleton type getter
				 *
				 *  \return Skeleton type
				 */
				virtual Type getType() const;

				/**
				 *  \brief Size getter (used in nodes comparison)
				 *
				 *  \param vec voctor to evaluate the size
				 *
				 *  \return size associated to vec
				 */
				virtual double getSize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const;

				/**
				 *  \brief Resize an object from its vector
				 *
				 *  \param vec  vector of object to resize
				 *  \param size new relative size
				 *
				 *  \return resized vector
				 */
				virtual Eigen::Matrix<double,meta<Perspective>::stordim,1> resize(const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec, double size) const;

				/**
				 *  \brief Tests if a object is included into another one
				 *
				 *  \param vec1 first object vector
				 *  \param vec2 second object vector
				 *
				 *  \return true is first object is in second object
				 */
				virtual bool included(const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec1, const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec2) const;

			protected:
				/**
				 *  \brief Associate the center of the ellips to a vector
				 *
				 *  \param vec vector to convert
				 *
				 *  \return center associated to vec
				 */
				virtual mathtools::affine::Point<2> toObj(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec,
														  const mathtools::affine::Point<2> &) const;

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
