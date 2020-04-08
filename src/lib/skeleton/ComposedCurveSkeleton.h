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
 *  \file ComposedCurveSkeleton.h
 *  \brief Defines composed curve skeleton
 *  \author Bastien Durix
 */

#ifndef _COMPOSEDCURVESKELETON_H_
#define _COMPOSEDCURVESKELETON_H_

#include <memory>
#include <boost/graph/adjacency_list.hpp>

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes composed curve skeleton
	 *
	 *  \tparam BranchType Branch class composing the skeleton
	 */
	template<typename BranchType>
	class ComposedCurveSkeleton
	{
		public:
			/**
			 *  \brief Composed curve skeleton shared pointer
			 */
			using Ptr = std::shared_ptr<ComposedCurveSkeleton<BranchType> >;

		protected:
			/**
			 *  \brief Vertex property
			 */
			struct VertexProperty 
			{
				/**
				 *  \brief Vertex index
				 */
				unsigned int index;
			};
			
			/**
			 *  \brief Edge property
			 */
			struct EdgeProperty 
			{
				/**
				 *  \brief Edge index
				 */
				unsigned int index;

				/**
 				 *  \brief Branch associated to the edge
 				 */
				typename BranchType::Ptr branch;
			};
			
			/**
			 *  \brief Graph type
			 *
			 *  \details Creates a undirected graph with vertices using VertexProperty
			 */
			using GraphType = boost::adjacency_list<boost::listS,boost::listS,boost::bidirectionalS,VertexProperty,EdgeProperty>;

			/**
			 *  \brief Graph of the composed skeleton
			 */
			GraphType m_graph;
			
			/**
			 *  \brief Last added node, to compute the key of the next added node
			 */
			unsigned int m_nodelast;
			
			/**
			 *  \brief Last added edge, to compute the key of the next added edge
			 */
			unsigned int m_edgelast;

		public:
			/**
			 *  \brief Constructor
			 */
			ComposedCurveSkeleton() : m_graph(), m_nodelast(0), m_edgelast(0) {}
		
		protected:
			/**
			 *  \brief Get the vertex descriptor associated to a vertex index
			 *
			 *  \param index  vertex index
			 *  \param v_desc out vertex descriptor
			 *
			 *  \return false if the index is not in the skeleton
			 */
			bool getDesc(unsigned int index, typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc) const
			{
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				bool v_found = false;
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end && !v_found; vi++)
				{
					if(m_graph[*vi].index == index)
					{
						v_desc = *vi;
						v_found = true;
					}
				}
				
				return v_found;
			}

			/**
			 *  \brief Get the vertex descriptors associated to two vertex indices
			 *
			 *  \param ind1    first vertex index
			 *  \param ind2    second vertex index
			 *  \param v_desc1 out first vertex descriptor
			 *  \param v_desc2 out second vertex descriptor
			 *
			 *  \return false if one of the indices is not in the skeleton
			 */
			bool getDesc(unsigned int ind1, unsigned int ind2,
						 typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc1,
						 typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc2) const
			{
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				bool v1_found = false, v2_found = false;
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end && (!v1_found || !v2_found); vi++)
				{
					if(m_graph[*vi].index == ind1)
					{
						v_desc1 = *vi;
						v1_found = true;
					}
					if(m_graph[*vi].index == ind2)
					{
						v_desc2 = *vi;
						v2_found = true;
					}
				}
				
				return v1_found && v2_found;
			}

			/**
			 *  \brief Get the edge descriptor associated to an edge index
			 *
			 *  \param index  edge index
			 *  \param e_desc out edge descriptor
			 *
			 *  \return false if the index is not in the skeleton
			 */
			bool getDesc(unsigned int index, typename boost::graph_traits<GraphType>::edge_descriptor &e_desc) const
			{
				typename boost::graph_traits<GraphType>::edge_iterator ei, ei_end;
				bool e_found = false;
				for(boost::tie(ei,ei_end) = boost::edges(m_graph); ei != ei_end && !e_found; ei++)
				{
					if(m_graph[*ei].index == index)
					{
						e_desc = *ei;
						e_found = true;
					}
				}
				
				return e_found;
			}

		public://modifying functions
			/**
			 *  \brief Adding node function
			 *
			 *  \return node index
			 */
			unsigned int addNode()
			{
				unsigned int index = m_nodelast++;
				//Adds the vertex in the graph, with index and storage information
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc = boost::add_vertex(m_graph);
				m_graph[v_desc].index = index;
				return index;
			}

			/**
			 *  \brief Adding node function, with given index
			 *
			 *  \param index new node index
			 *
			 *  \return node index
			 */
			unsigned int addNode(unsigned int index)
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				if(!getDesc(index,v_desc))
				{
					//Adds the vertex in the graph, with index and storage information
					typename boost::graph_traits<GraphType>::vertex_descriptor v_desc = boost::add_vertex(m_graph);
					m_graph[v_desc].index = index;
					if(m_nodelast<=index)
						m_nodelast = index+1;
				}
				return index;
			}

			/**
			 *  \brief Adding edge function
			 *
			 *  \param ind1   first node index
			 *  \param ind2   second node index
			 *  \param branch branch to add
			 *
			 *  \return false if one of the two nodes is not in the skeleton
			 */
			bool addEdge(unsigned int ind1, unsigned int ind2, const BranchType &branch)
			{
				bool v_found = false;
				if(ind1 != ind2)
				{
					// first step, get the descriptors corresponding to indices
					typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
					v_found = getDesc(ind1,ind2,v_desc1,v_desc2);

					// second step, add the edge
					if(v_found)
					{
						typename boost::graph_traits<GraphType>::edge_descriptor e_desc;
						bool added;
						boost::tie(e_desc,added) = boost::add_edge(v_desc1,v_desc2,m_graph);
						if(added)
						{
							m_graph[e_desc].index = m_edgelast++;
							m_graph[e_desc].branch = typename BranchType::Ptr(new BranchType(branch));
						}
					}
				}
				return v_found;
			}

			/**
			 *  \brief Adding edge function
			 *
			 *  \param ind1   first node index
			 *  \param ind2   second node index
			 *  \param branch branch to add
			 *
			 *  \return false if one of the two nodes is not in the skeleton
			 */
			bool addEdge(unsigned int ind1, unsigned int ind2, const typename BranchType::Ptr branch)
			{
				bool v_found = false;
				if(ind1 != ind2)
				{
					// first step, get the descriptors corresponding to indices
					typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
					v_found = getDesc(ind1,ind2,v_desc1,v_desc2);

					// second step, add the edge
					if(v_found)
					{
						typename boost::graph_traits<GraphType>::edge_descriptor e_desc;
						bool added;
						boost::tie(e_desc,added) = boost::add_edge(v_desc1,v_desc2,m_graph);
						if(added)
						{
							m_graph[e_desc].index = m_edgelast++;
							m_graph[e_desc].branch = branch;
						}
					}
				}
				return v_found;
			}

			/**
			 *  \brief Removing edge function
			 *
			 *  \param ind1   first node index
			 *  \param ind2   second node index
			 *  \param branch removed branch
			 *
			 *  \return false if one of the two nodes is not in the skeleton, or if they are not neighbors
			 */
			bool remEdge(unsigned int ind1, unsigned int ind2, typename BranchType::Ptr &branch)
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
				if(!getDesc(ind1,ind2,v_desc1,v_desc2))
					throw std::logic_error("skeleton::ComposedCurveSkeleton::remEdge(): Node index is not in the skeleton");

				typename boost::graph_traits<GraphType>::edge_descriptor ei;
				
				bool areneigh = false;
				branch = NULL;

				boost::tie(ei,areneigh) = boost::edge(v_desc1,v_desc2,m_graph);
				
				if(areneigh)
				{
					branch = m_graph[ei].branch;
					boost::remove_edge(ei,m_graph);
				}
				if(!areneigh)
				{
					boost::tie(ei,areneigh) = boost::edge(v_desc2,v_desc1,m_graph);
					if(areneigh)
					{
						branch = m_graph[ei].branch->reverted();
						if(!branch)
						{
							branch = m_graph[ei].branch;
						}
						boost::remove_edge(ei,m_graph);
					}
				}
				
				return areneigh;
			}

			/**
			 *  \brief Branch getter
			 *
			 *  \param index edge index
			 *  
			 *  \return branche associated to index
			 *  
			 *  \throws std::logic_error if index is not in the skeleton
			 */
			typename BranchType::Ptr getBranch(unsigned int index)
			{
				typename boost::graph_traits<GraphType>::edge_descriptor e_desc;
				if(!getDesc(index,e_desc))
					throw std::logic_error("skeleton::ComposedCurveSkeleton::getBranch(): Edge index is not in the skeleton");

				return m_graph[e_desc].branch;
			}

		public://non modifying functions
			/**
			 *  \brief Get the number of nodes in the skeleton
			 *
			 *  \return Number of nodes
			 */
			unsigned int getNbNodes() const
			{
				return boost::num_vertices(m_graph);
			}
			
			/**
			 *  \brief Tests if the node index is in the skeleton
			 *
			 *  \return true if the node is in the skeleton
			 */
			bool isNodeIn(unsigned int index) const
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				return getDesc(index,v_desc);
			}
			
			/**
			 *  \brief Tests if the nodes are neighors
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 *
			 *  \return true if they are neighors
			 *
			 *  \throws std::logic_error if one of the two nodes is not in the skeleton
			 */
			bool areNeighbors(unsigned int ind1, unsigned int ind2) const
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
				if(!getDesc(ind1,ind2,v_desc1,v_desc2))
					throw std::logic_error("skeleton::ComposedCurveSkeleton::areNeighbors(): Node index is not in the skeleton");

				typename boost::graph_traits<GraphType>::edge_iterator ei;
				
				bool areneigh = false;

				boost::tie(ei,areneigh) = boost::edge(v_desc1,v_desc2,m_graph);
				if(!areneigh)
				{
					boost::tie(ei,areneigh) = boost::edge(v_desc1,v_desc2,m_graph);
				}

				return areneigh;
			}

			/**
			 *  \brief Node degree getter by index
			 *
			 *  \param index index of the node to get
			 *
			 *  \return degree of the node
			 */
			unsigned int getNodeDegree(unsigned int index) const
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				if(!getDesc(index,v_desc))
				{
					throw new std::logic_error("skeleton::ComposedCurveSkeleton::getNodeDegree(): Node index is not in the skeleton");
				}
				
				return (unsigned int)boost::out_degree(v_desc,m_graph) + (unsigned int)boost::in_degree(v_desc,m_graph);
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
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end; vi++)
				{
					cont.push_back(m_graph[*vi].index);
				}
			}

			/**
			 *  \brief Get the indices of all nodes, by degree
			 *
			 *  \tparam Container container type
			 *
			 *  \param degree node degree to get
			 *  \param cont   container in which store the indices
			 */
			template<typename Container>
			void getNodesByDegree(unsigned int degree, Container &cont) const
			{
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end; vi++)
				{
					if(boost::out_degree(*vi,m_graph)+boost::in_degree(*vi,m_graph) == degree)
						cont.push_back(m_graph[*vi].index);
				}
			}
			
			/**
			 *  \brief Neighbors accessor
			 *
			 *  \tparam Container container type
			 *  
			 *  \param  index index of the node
			 *  \param  cont  container in which store the indices
			 */
			template<typename Container>
			void getNeighbors(unsigned int index, Container &cont) const
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				if(getDesc(index,v_desc))
				{
					typename boost::graph_traits<GraphType>::adjacency_iterator ai, ai_end;

					for(boost::tie(ai,ai_end) = boost::adjacent_vertices(v_desc,m_graph); ai != ai_end; ai++)
					{
						cont.push_back(m_graph[*ai].index);
					}
				
					typename boost::graph_traits<GraphType>::in_edge_iterator aiinv, aiinv_end;

					for(boost::tie(aiinv,aiinv_end) = boost::in_edges(v_desc,m_graph); aiinv != aiinv_end; aiinv++)
					{
						cont.push_back(m_graph[source(*aiinv,m_graph)].index);
					}
				}
			}

			/**
			 *  \brief Branch getter
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 *  
			 *  \return branch from node ind1 to node ind2
			 *  
			 *  \throws std::logic_error if ind1 or ind2 or edge (ind1,ind2) is not in the skeleton
			 */
			const typename BranchType::Ptr getBranch(unsigned int ind1, unsigned int ind2) const
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
				if(!getDesc(ind1,ind2,v_desc1,v_desc2))
					throw std::logic_error("skeleton::ComposedCurveSkeleton::getBranch(): Node index is not in the skeleton");

				typename boost::graph_traits<GraphType>::edge_descriptor ei;
				
				typename BranchType::Ptr br(NULL);

				bool areneigh = false;

				boost::tie(ei,areneigh) = boost::edge(v_desc1,v_desc2,m_graph);
				
				if(areneigh)
				{
					br = m_graph[ei].branch;
				}
				if(!areneigh)
				{
					boost::tie(ei,areneigh) = boost::edge(v_desc2,v_desc1,m_graph);
					if(areneigh)
					{
						br = m_graph[ei].branch->reverted();
						if(!br)
						{
							br = m_graph[ei].branch;
						}
					}
				}
				
				if(!areneigh)
					throw std::logic_error("skeleton::ComposedCurveSkeleton::getEdge(): Edge is not in the skeleton");

				return br;
			}

			/**
			 *  \brief Get the extremities of an edge
			 *
			 *  \param index index of the edge
			 *
			 *  \return pair containing index of extremities
			 *
			 *  \throws std::logic_error if edge is not in the skeleton
			 */
			std::pair<unsigned int,unsigned int> getExtremities(unsigned int index) const
			{
				typename boost::graph_traits<GraphType>::edge_descriptor e_desc;
				if(!getDesc(index,e_desc))
					throw std::logic_error("skeleton::ComposedCurveSkeleton::getExtremities(): Edge index is not in the skeleton");

				std::pair<unsigned int,unsigned int> ext;

				ext.first  = m_graph[boost::source(e_desc,m_graph)].index;
				ext.second = m_graph[boost::target(e_desc,m_graph)].index;
				
				return ext;
			}

			/**
			 *  \brief Get the indices of all edges
			 *
			 *  \tparam Container container type
			 *
			 *  \param  cont container in which store the indices
			 */
			template<typename Container>
			void getAllEdges(Container &cont) const
			{
				typename boost::graph_traits<GraphType>::edge_iterator ei, ei_end;
				for(boost::tie(ei,ei_end) = boost::edges(m_graph); ei != ei_end; ei++)
				{
					cont.push_back(m_graph[*ei].index);
				}
			}

	};
}


#endif //_COMPOSEDCURVESKELETON_H_
