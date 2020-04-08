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
 *  \file  Basis.cxx
 *  \brief Defines static declarations of canonic basis
 *  \author Bastien Durix
 */

#include "Basis.h"

template<>
mathtools::vectorial::Basis<2>::Ptr mathtools::vectorial::Basis<2>::canonicbasis /** Canonic basis of dimension 2 */
										= mathtools::vectorial::Basis<2>::CreateBasis(Eigen::Matrix2d::Identity());


template<>
mathtools::vectorial::Basis<3>::Ptr mathtools::vectorial::Basis<3>::canonicbasis /** Canonic basis of dimension 3 */
										= mathtools::vectorial::Basis<3>::CreateBasis(Eigen::Matrix3d::Identity());


template<>
mathtools::vectorial::Basis<4>::Ptr mathtools::vectorial::Basis<4>::canonicbasis /** Canonic basis of dimension 4 */
										= mathtools::vectorial::Basis<4>::CreateBasis(Eigen::Matrix4d::Identity());
