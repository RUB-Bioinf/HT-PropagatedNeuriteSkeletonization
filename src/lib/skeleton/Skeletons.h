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
 *  \file Skeletons.h
 *  \brief Defines some skeleton types
 *  \author Bastien Durix
 */

#ifndef _SKELETONS_H_
#define _SKELETONS_H_

#include "GraphCurveSkeleton.h"
#include "ComposedCurveSkeleton.h"
#include "GraphBranch.h"
#include "ContinuousBranch.h"
#include "ReconstructionBranch.h"
#include "model/Classic.h"
#include "model/Projective.h"

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Defines classical 2d curve skeleton
	 */
	using GraphSkel2d = GraphCurveSkeleton<model::Classic<2> >;

	/**
	 *  \brief Defines classical 3d curve skeleton
	 */
	using GraphSkel3d = GraphCurveSkeleton<model::Classic<3> >;

	/**
	 *  \brief Defines projective skeleton, used in reconstruction
	 */
	using GraphProjSkel = GraphCurveSkeleton<model::Projective>;



	/**
	 *  \brief Defines classical 2d branch
	 */
	using BranchGraphSkel2d = GraphBranch<model::Classic<2> >;

	/**
	 *  \brief Defines classical 3d branch
	 */
	using BranchGraphSkel3d = GraphBranch<model::Classic<3> >;

	/**
	 *  \brief Defines projective branch, used in reconstruction
	 */
	using BranchGraphProjSkel = GraphBranch<model::Projective>;



	/**
	 *  \brief Defines classical 2d composed skeleton
	 */
	using CompGraphSkel2d = ComposedCurveSkeleton<GraphBranch<model::Classic<2> > >;

	/**
	 *  \brief Defines classical 3d composed skeleton
	 */
	using CompGraphSkel3d = ComposedCurveSkeleton<GraphBranch<model::Classic<3> > >;

	/**
	 *  \brief Defines projective composed skeleton, used in reconstruction
	 */
	using CompGraphProjSkel = ComposedCurveSkeleton<GraphBranch<model::Projective> >;



	/**
	 *  \brief Defines classical 2d continuous branch
	 */
	using BranchContSkel2d = ContinuousBranch<model::Classic<2> >;

	/**
	 *  \brief Defines classical 3d continuous branch
	 */
	using BranchContSkel3d = ContinuousBranch<model::Classic<3> >;

	/**
	 *  \brief Defines projective continuous branch, used in reconstruction
	 */
	using BranchContProjSkel = ContinuousBranch<model::Projective>;



	/**
	 *  \brief Defines classical 2d composed skeleton, with continuous branches
	 */
	using CompContSkel2d = ComposedCurveSkeleton<ContinuousBranch<model::Classic<2> > >;

	/**
	 *  \brief Defines classical 3d composed skeleton, with continuous branches
	 */
	using CompContSkel3d = ComposedCurveSkeleton<ContinuousBranch<model::Classic<3> > >;

	/**
	 *  \brief Defines projective composed skeleton, used in reconstruction, with continuous branches
	 */
	using CompContProjSkel = ComposedCurveSkeleton<ContinuousBranch<model::Projective> >;


	/**
	 *  \brief Defines composed reconstruction skeleton
	 */
	using ReconstructionSkeleton = ComposedCurveSkeleton<ReconstructionBranch>;
}

#endif //_SKELETONS_H_
