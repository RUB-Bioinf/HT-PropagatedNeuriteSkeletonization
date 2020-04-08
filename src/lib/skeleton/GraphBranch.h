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
 *  \file GraphBranch.h
 *  \brief Defines graph skeletal branch
 *  \author Bastien Durix
 */

#ifndef _GRAPHBRANCH_H_
#define _GRAPHBRANCH_H_

#include <memory>
#include <Eigen/Dense>
#include "model/MetaModel.h"

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes graph skeletal branch
	 *
	 *  \tparam Model Class giving a meaning to the skeleton (dimensions, geometric interpretation...)
	 */
	template<typename Model>
	class GraphBranch
	{
		public:
			/**
			 *  \brief Graph skeletal branch shared pointer
			 */
			using Ptr = std::shared_ptr<GraphBranch<Model> >;
			
			/**
			 *  \brief Storage type of the objects in the skeleton
			 */
			using Stor = Eigen::Matrix<double,model::meta<Model>::stordim,1>;
			
		protected:
			/**
			 *  \brief Model used to give a meaning to the skeleton
			 *
			 *  \details This class will not be modified inside the skeleton:
			 *			 once it is created, it should not be changed
			 */
			typename Model::Ptr m_model;
			
			/**
 			 *  \brief Vector of nodes in the branch
 			 */
			std::shared_ptr<std::vector<Stor> > m_nodes;

			/**
 			 *  \brief Revert the order of the nodes in the branch
 			 */
			bool m_reverted;
			
		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 *  \param nodes nodes of the branch
			 */
			GraphBranch(const typename Model::Ptr model, const std::vector<Stor> &nodes = std::vector<Stor>(0)) :
				m_model(model), m_nodes(new std::vector<Stor>(nodes)), m_reverted(false) {}
			
			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 *  \param nodes nodes of the branch
			 */
			GraphBranch(const Model &model, const std::vector<Stor> &nodes = std::vector<Stor>(0)) :
				GraphBranch(typename Model::Ptr(new Model(model)),nodes) {}
			
			/**
			 *  \brief Copy constructor
			 *
			 *  \param grbr skeleton to copy
			 */
			GraphBranch(const GraphBranch<Model> &grbr) :
				m_model(grbr.m_model), m_nodes(grbr.m_nodes), m_reverted(grbr.m_reverted) {}
			
			/**
			 *  \brief Model getter
			 *
			 *  \return Const shared pointer to skeleton model
			 */
			inline const typename Model::Ptr getModel() const
			{
				return m_model;
			}
			
			/**
			 *  \brief Give node number
			 *
			 *  \return node number
			 */
			unsigned int getNbNodes() const
			{
				return m_nodes->size();
			}
			
			/**
			 *  \brief Node getter by index
			 *
			 *  \param index index of the node to get
			 *
			 *  \return storage associated to the node
			 */
			const Stor& getNode(unsigned int index) const
			{
				if(index >= getNbNodes())
				{
					throw new std::logic_error("skeleton::GraphBranch::getNode(): Node index is not in the skeleton");
				}
				if(m_reverted)
					index = getNbNodes()-index-1;
				return (*m_nodes)[index];
			}
			
			/**
			 *  \brief Node getter by index
			 *
			 *  \tparam TypeNode type of the node to get
			 *
			 *  \param index index of the node to get
			 *
			 *  \return node associated to index
			 */
			template<typename TypeNode>
			const TypeNode getNode(unsigned int index) const
			{
				return m_model->template toObj<TypeNode>(getNode(index));
			}
			
			/**
			 *  \brief Get the indices of all nodes
			 *
			 *  \tparam Container container type
			 *
			 *  \param  cont container in which store the indices
			 */
			template<typename Container>
			void getAllNodes(Container &cont) const
			{
				if(m_reverted)
				{
					for(typename std::vector<Stor>::const_reverse_iterator it = m_nodes->rbegin(); it != m_nodes->rend(); it++)
					{
						cont.push_back(*it);
					}
				}
				else
				{
					for(typename std::vector<Stor>::const_iterator it = m_nodes->begin(); it != m_nodes->end(); it++)
					{
						cont.push_back(*it);
					}
				}
			}

			/**
 			 *  \brief Reverted branch getter
 			 *
 			 *  \return Reverted branch
 			 */
			const GraphBranch<Model>::Ptr reverted() const
			{
				GraphBranch<Model>::Ptr rev(new GraphBranch<Model>(*this));
				rev->m_reverted = !m_reverted;
				return rev;
			}
	};
}

#endif //_GRAPHBRANCH_H_
