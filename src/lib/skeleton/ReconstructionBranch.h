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
 *  \file ReconstructionBranch.h
 *  \brief Defines reconstruction branch
 *  \author Bastien Durix
 */

#ifndef _RECONSTRUCTIONBRANCH_H_
#define _RECONSTRUCTIONBRANCH_H_

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include "model/MetaModel.h"

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes reconstruction branch
	 */
	class ReconstructionBranch
	{
		public:
			/**
			 *  \brief Shared pointer definition
			 */
			typedef std::shared_ptr<ReconstructionBranch> Ptr;

		protected:
			/**
			 *  \brief Indices of used skeletons
			 */
			std::vector<unsigned int> m_indskel;

			/**
			 *  \brief First extremity associated in each skeleton
			 */
			std::vector<unsigned int> m_firstext;
			
			/**
			 *  \brief Last extremity associated in each skeleton
			 */
			std::vector<unsigned int> m_lastext;
			
			/**
			 *  \brief Matching vector
			 */
			std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > m_match;
			
			/**
			 *  \brief Tell if the branches have been matched or not
			 */
			bool m_matched;

			/**
			 *  \brief Condition vector
			 */
			std::vector<double> m_cond;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param indskel   skeleton used in reconstruction
			 *  \param firstext  first extremities of projective skeletons
			 *  \param lastext   last extremities of projective skeletons
			 */
			ReconstructionBranch(const std::vector<unsigned int> &indskel, const std::vector<unsigned int> &firstext, const std::vector<unsigned int> &lastext);
		
		public://non modifying functions
			/**
			 *  \brief Indices of used skeletons getter
			 *
			 *  \return Indices of used skeletons
			 */
			const std::vector<unsigned int>& getIndSkel() const;

			/**
			 *  \brief First extremities getter
			 *
			 *  \return First extremities
			 */
			const std::vector<unsigned int>& getFirstExt() const;

			/**
			 *  \brief Last extremities getter
			 *
			 *  \return Last extremities
			 */
			const std::vector<unsigned int>& getLastExt() const;

			/**
			 *  \brief Matching vector getter
			 *
			 *  \return Matching vector
			 */
			const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> >& getMatch() const;
			
			/**
			 *  \brief Tells if the branches have been matched or not
			 *
			 *  \return true if the matching is done
			 */
			bool isMatched() const;

			/**
			 *  \brief Condition vector getter
			 *
			 *  \return Condition vector
			 */
			const std::vector<double>& getCond() const;

			/**
 			 *  \brief Reverted branch getter
 			 *
 			 *  \return Reverted branch
 			 */
			const ReconstructionBranch::Ptr reverted() const;

		public://modifying functions
			/**
			 *  \brief Matching vector setter
			 *
			 *  \param match matching vector
			 */
			void setMatch(const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> >& match);

			/**
			 *  \brief Condition vector setter
			 *
			 *  \param cond condition vector
			 */
			void setCond(const std::vector<double>& cond);
	};
}

#endif //_RECONSTRUCTIONBRANCH_H_
