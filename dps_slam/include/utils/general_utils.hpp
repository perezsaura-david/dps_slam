// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/********************************************************************************************
 *  \file       utils.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef UTILS__GENERAL_UTILS_HPP_
#define UTILS__GENERAL_UTILS_HPP_

#define RESET_COLOR "\033[0m"
#define RED_COLOR "\033[0;31m"
#define GREEN_COLOR "\033[1;32m"
#define YELLOW_COLOR "\033[1;33m"
#define MAGENTA_COLOR "\033[0;35m"
#define CYAN_COLOR "\033[0;36m"
#define BLUE_COLOR "\033[0;34m"

#define ERROR(x) \
  std::cerr << RED_COLOR << "[ERROR] " << x << std::endl \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl

#define WARN(x) std::cout << YELLOW_COLOR << "[WARN] " << x << RESET_COLOR << std::endl

#define FLAG(x) // std::cout << CYAN_COLOR << "[FLAG] " << x << RESET_COLOR << std::endl
#define DEBUG(x) // std::cout << GREEN_COLOR << "[DEBUG] " << x << RESET_COLOR << std::endl
#define DEBUG_VERBOSE(x) // \
  std::cout << GREEN_COLOR << "[DEBUG] " << x << std::endl \
            << "\t  at line " << __LINE__ << " in function " << __func__ << RESET_COLOR \
            << std::endl
#define INFO(x) // std::cout << x << std::endl
#define PARAM(x) \
  std::cout << CYAN_COLOR << "[PARAMS] " << x << RESET_COLOR << std::endl

#define NAME(x) "[" << x << "] "

#define FLAG_GRAPH(x) FLAG(NAME(name_) << x)
#define WARN_GRAPH(x) WARN(NAME(name_) << x)
#define INFO_GRAPH(x) INFO(NAME(name_) << x)
#define DEBUG_GRAPH(x) DEBUG(NAME(name_) << x)
#define ERROR_GRAPH(x) ERROR(NAME(name_) << x)

#define PRINT_VAR(var) #var << " = " << var

#endif  // UTILS__GENERAL_UTILS_HPP_
