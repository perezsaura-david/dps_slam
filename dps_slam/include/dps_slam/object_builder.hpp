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
 *  \file       graph_node_types.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__OBJECT_DETECTION_BUILDER_HPP_
#define AS2_SLAM__OBJECT_DETECTION_BUILDER_HPP_

#include <unordered_map>
#include <memory>
#include <string>
#include <object_detection_types.hpp>
#include <graph_node_types.hpp>

struct GraphObject
{
  // GraphObject(GraphNode * _node, ObjectDetection * _object_detection);
  GraphNode * node_;
  ObjectDetection * object_detection_;
};

// class ObjectDetectionFactory
// {
// public:
//   static std::unique_ptr<ObjectDetection> createObjectFromNode(GraphNode * node)
//   {
//     auto it = registry().find(node->getNodeName());
//     if (it != registry().end()) {
//       return it->second(node);
//     }
//     throw std::runtime_error("Unknown node type: " + node->getNodeName());
//   }

//   static void registerObject(
//     const std::string & name,
//     std::function<std::unique_ptr<ObjectDetection>(GraphNode *)> creator)
//   {
//     registry()[name] = creator;
//   }

// private:
//   static std::unordered_map<std::string,
//     std::function<std::unique_ptr<ObjectDetection>(GraphNode *)>> & registry()
//   {
//     static std::unordered_map<std::string,
//       std::function<std::unique_ptr<ObjectDetection>(GraphNode *)>> instance;
//     return instance;
//   }
// };

#endif  // AS2_SLAM__OBJECT_DETECTION_BUILDER_HPP_
