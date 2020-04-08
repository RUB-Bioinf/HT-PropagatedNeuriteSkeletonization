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
 *  \file  DiscreteBoundary2.h
 *  \brief Defines discrete boundary of a shape
 *  \author Bastien Durix
 */

#ifndef _DISCRETEBOUNDARY2_H_
#define _DISCRETEBOUNDARY2_H_

#include <memory>
#include <list>
#include <map>
#include <vector>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>
#include "DiscreteBoundary.h"

/**
 *  \brief Boundary representations
 */
namespace boundary
{
	/**
	 *  \brief Describe discrete boundary in 2d space
	 */
	template<>
	class DiscreteBoundary<2>
	{
		public:
			/**
			 *  \brief Boundary shared pointer
			 */
			using Ptr = std::shared_ptr<DiscreteBoundary<2> >;

		protected:
			/**
			 *  \brief Boundary frame
			 */
			typename mathtools::affine::Frame<2>::Ptr m_frame;

			/**
			 *  \brief Vector of vertices composing the boundary
			 */
			std::vector<Eigen::Vector2d> m_vecvert;

			/**
			 *  \brief Neighbors of each vertex
			 */
			std::map<unsigned int,unsigned int> m_neigh;

			/**
			 *  \brief Previous neighbors of each vertex
			 */
			std::map<unsigned int,unsigned int> m_neigh_prev;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			DiscreteBoundary<2>(const mathtools::affine::Frame<2>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame());
			
			/**
			 *  \brief Adds new string of vertices
			 *
			 *  \tparam Container vertices container
			 *  
			 *  \param vertstr    container of vertices to add
			 */
			template<typename Container>
			void addVerticesPoint(const Container &vertstr)
			{
				unsigned int ind = m_vecvert.size();
				m_vecvert.resize(m_vecvert.size() + vertstr.size());
				unsigned int beg = ind;
				for(typename Container::const_iterator it = vertstr.begin(); it != vertstr.end(); it++)
				{
					m_vecvert[ind] = it->getCoords(m_frame);
					m_neigh[ind] = ind+1;
					m_neigh_prev[ind+1] = ind;
					ind++;
				}
				m_neigh[ind-1] = beg;
				m_neigh_prev[beg] = ind-1;
			}

			/**
			 *  \brief Adds new string of vertices
			 *
			 *  \tparam Container vertices container
			 *  
			 *  \param vertstr    container of vertices to add
			 */
			template<typename Container>
			void addVerticesVector(const Container &vertstr)
			{
				unsigned int ind = m_vecvert.size();
				m_vecvert.resize(m_vecvert.size() + vertstr.size());
				unsigned int beg = ind;
				for(typename Container::const_iterator it = vertstr.begin(); it != vertstr.end(); it++)
				{
					m_vecvert[ind] = *it;
					m_neigh[ind] = ind+1;
					m_neigh_prev[ind+1] = ind;
					ind++;
				}
				m_neigh[ind-1] = beg;
				m_neigh_prev[beg] = ind-1;
			}

			/**
			 *  \brief Frame getter
			 *
			 *  \return frame of the boundary
			 */
			const mathtools::affine::Frame<2>::Ptr getFrame() const;

			/**
			 *  \brief Vertices getter
			 *  
			 *  \tparam Container vertices container
			 *  \param  cont      container in which add the vertices
			 */
			template<typename Container>
			void getVerticesPoint(Container &cont) const
			{
				for(unsigned int i=0; i < m_vecvert.size(); i++)
				{
					cont.push_back(mathtools::affine::Point<2>(m_vecvert[i],m_frame));
				}
			}

			/**
			 *  \brief Vertices getter
			 *  
			 *  \tparam Container vertices container
			 *  \param  cont      container in which add the vertices
			 */
			template<typename Container>
			void getVerticesVector(Container &cont) const
			{
				cont.insert(cont.end(),m_vecvert.begin(),m_vecvert.end());
			}

			/**
			 *  \brief Get next vertex indice
			 *
			 *  \param index vertex index
			 *
			 *  \return next index
			 */
			unsigned int getNext(unsigned int index) const;

			/**
			 *  \brief Get previous vertex indice
			 *
			 *  \param index vertex index
			 *
			 *  \return previous index
			 */
			unsigned int getPrev(unsigned int index) const;

			/**
			 *  \brief Get vertex associated to an indice
			 *
			 *  \param index  index of the vertex
			 *
			 *  \return vertex associated to index
			 */
			mathtools::affine::Point<2> getVertex(unsigned int index) const;

			/**
			 *  \brief Get coordinates associated to an indice
			 *
			 *  \param index  index of the vertex
			 *
			 *  \return coordinates associated to index
			 */
			Eigen::Vector2d getCoordinates(unsigned int index) const;

			/**
 			 *  \brief Number of vertices getter
 			 *
 			 *  \return number of vertices
 			 */
			unsigned int getNbVertices() const;
	};
}

#endif //_DISCRETEBOUNDARY2_H_
