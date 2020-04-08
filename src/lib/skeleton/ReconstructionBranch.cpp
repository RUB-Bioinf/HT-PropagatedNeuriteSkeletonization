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
 *  \file ReconstructionBranch.cpp
 *  \brief Defines reconstruction branch
 *  \author Bastien Durix
 */

#include "ReconstructionBranch.h"

skeleton::ReconstructionBranch::ReconstructionBranch(const std::vector<unsigned int> &indskel, const std::vector<unsigned int> &firstext, const std::vector<unsigned int> &lastext) :
	m_indskel(indskel), m_firstext(firstext), m_lastext(lastext), m_matched(false) 
{}


const std::vector<unsigned int>& skeleton::ReconstructionBranch::getIndSkel() const
{
	return m_indskel;
}

const std::vector<unsigned int>& skeleton::ReconstructionBranch::getFirstExt() const
{
	return m_firstext;
}

const std::vector<unsigned int>& skeleton::ReconstructionBranch::getLastExt() const
{
	return m_lastext;
}

const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> >& skeleton::ReconstructionBranch::getMatch() const
{
	return m_match;
}

const std::vector<double>& skeleton::ReconstructionBranch::getCond() const
{
	return m_cond;
}

bool skeleton::ReconstructionBranch::isMatched() const
{
	return m_matched;
}

void skeleton::ReconstructionBranch::setMatch(const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> >& match)
{
	m_matched = true;
	m_match = match;
}

void skeleton::ReconstructionBranch::setCond(const std::vector<double>& cond)
{
	m_cond = cond;
}

const skeleton::ReconstructionBranch::Ptr skeleton::ReconstructionBranch::reverted() const
{
	skeleton::ReconstructionBranch::Ptr recrevert(new skeleton::ReconstructionBranch(m_indskel,m_lastext,m_firstext));
	
	if(m_matched)
	{
		std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > vecmatch(m_match.size());
		
		for(unsigned int i = 0; i < m_match.size(); i++)
		{
			Eigen::Matrix<double,Eigen::Dynamic,1> match = Eigen::Matrix<double,Eigen::Dynamic,1>::Ones(m_match[i].rows(),m_match[i].cols()) - m_match[i];
			vecmatch[i] = match;
		}
		
		recrevert->setMatch(vecmatch);
	}
	
	return recrevert;
}
