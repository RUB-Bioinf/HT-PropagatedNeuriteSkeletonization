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
 *  \file  Frame.cxx
 *  \brief Defines static declarations of canonic frames
 *  \author Bastien Durix
 */

#include "Frame.h"

template<>
mathtools::affine::Frame<2>::Ptr mathtools::affine::Frame<2>::canonicframe /** Canonic frame of dimension 2 */
									= mathtools::affine::Frame<2>::CreateFrame(Eigen::Vector2d::Zero(),mathtools::vectorial::Basis<2>::CanonicBasis());

template<>
mathtools::affine::Frame<3>::Ptr mathtools::affine::Frame<3>::canonicframe /** Canonic frame of dimension 3 */ 
									= mathtools::affine::Frame<3>::CreateFrame(Eigen::Vector3d::Zero(),mathtools::vectorial::Basis<3>::CanonicBasis());

template<>
mathtools::affine::Frame<4>::Ptr mathtools::affine::Frame<4>::canonicframe /** Canonic frame of dimension 4 */ 
									= mathtools::affine::Frame<4>::CreateFrame(Eigen::Vector4d::Zero(),mathtools::vectorial::Basis<4>::CanonicBasis());

