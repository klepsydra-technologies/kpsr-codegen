/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef PRIMITIVE_TYPES_VECTOR_SHARED_PTR_MAPPER_ROS
#define PRIMITIVE_TYPES_VECTOR_SHARED_PTR_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <klepsydra/codegen/primitive_types_vector_shared_ptr.h>
#include <kpsr_ros_codegen/PrimitiveTypesVectorSharedPtr.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::PrimitiveTypesVectorSharedPtr, kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr> {
public:
   void fromMiddleware(const kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr & message, kpsr::codegen::PrimitiveTypesVectorSharedPtr & event) {
      event.seq = message.seq;
      
      event.aaa.resize(message.aaa.size());
      std::transform(message.aaa.begin(), message.aaa.end(), event.aaa.begin(),
                     [](const signed char messageData) {
            return std::make_shared<signed char>(messageData);
      });
      
      event.bbb.resize(message.bbb.size());
      std::transform(message.bbb.begin(), message.bbb.end(), event.bbb.begin(),
                     [](const unsigned char messageData) {
            return std::make_shared<unsigned char>(messageData);
      });
      
      event.ccc.resize(message.ccc.size());
      std::transform(message.ccc.begin(), message.ccc.end(), event.ccc.begin(),
                     [](const short int messageData) {
            return std::make_shared<short int>(messageData);
      });
      
      event.ddd.resize(message.ddd.size());
      std::transform(message.ddd.begin(), message.ddd.end(), event.ddd.begin(),
                     [](const unsigned short int messageData) {
            return std::make_shared<unsigned short int>(messageData);
      });
      
      event.eee.resize(message.eee.size());
      std::transform(message.eee.begin(), message.eee.end(), event.eee.begin(),
                     [](const unsigned int messageData) {
            return std::make_shared<unsigned int>(messageData);
      });
      
      event.fff.resize(message.fff.size());
      std::transform(message.fff.begin(), message.fff.end(), event.fff.begin(),
                     [](const long long int messageData) {
            return std::make_shared<long long int>(messageData);
      });
      
      event.ggg.resize(message.ggg.size());
      std::transform(message.ggg.begin(), message.ggg.end(), event.ggg.begin(),
                     [](const unsigned long long int messageData) {
            return std::make_shared<unsigned long long int>(messageData);
      });
      
      event.hhh.resize(message.hhh.size());
      std::transform(message.hhh.begin(), message.hhh.end(), event.hhh.begin(),
                     [](const float messageData) {
            return std::make_shared<float>(messageData);
      });
      
      event.iii.resize(message.iii.size());
      std::transform(message.iii.begin(), message.iii.end(), event.iii.begin(),
                     [](const double messageData) {
            return std::make_shared<double>(messageData);
      });
      
      event.jjj.resize(message.jjj.size());
      std::transform(message.jjj.begin(), message.jjj.end(), event.jjj.begin(),
                     [](const bool messageData) {
            return std::make_shared<bool>(messageData);
      });
      
      event.kkk.resize(message.kkk.size());
      std::transform(message.kkk.begin(), message.kkk.end(), event.kkk.begin(),
                     [](const std::string messageData) {
            return std::make_shared<std::string>(messageData);
      });
   }

   void toMiddleware(const kpsr::codegen::PrimitiveTypesVectorSharedPtr &event, kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr &message) {
      message.seq = event.seq;
      
      message.aaa.resize(event.aaa.size());
      std::transform(event.aaa.begin(), event.aaa.end(), message.aaa.begin(),
                     [&](std::shared_ptr<signed char> eventData) {
         return * eventData.get();
      });
      
      message.bbb.resize(event.bbb.size());
      std::transform(event.bbb.begin(), event.bbb.end(), message.bbb.begin(),
                     [&](std::shared_ptr<unsigned char> eventData) {
         return * eventData.get();
      });
      
      message.ccc.resize(event.ccc.size());
      std::transform(event.ccc.begin(), event.ccc.end(), message.ccc.begin(),
                     [&](std::shared_ptr<short int> eventData) {
         return * eventData.get();
      });
      
      message.ddd.resize(event.ddd.size());
      std::transform(event.ddd.begin(), event.ddd.end(), message.ddd.begin(),
                     [&](std::shared_ptr<unsigned short int> eventData) {
         return * eventData.get();
      });
      
      message.eee.resize(event.eee.size());
      std::transform(event.eee.begin(), event.eee.end(), message.eee.begin(),
                     [&](std::shared_ptr<unsigned int> eventData) {
         return * eventData.get();
      });
      
      message.fff.resize(event.fff.size());
      std::transform(event.fff.begin(), event.fff.end(), message.fff.begin(),
                     [&](std::shared_ptr<long long int> eventData) {
         return * eventData.get();
      });
      
      message.ggg.resize(event.ggg.size());
      std::transform(event.ggg.begin(), event.ggg.end(), message.ggg.begin(),
                     [&](std::shared_ptr<unsigned long long int> eventData) {
         return * eventData.get();
      });
      
      message.hhh.resize(event.hhh.size());
      std::transform(event.hhh.begin(), event.hhh.end(), message.hhh.begin(),
                     [&](std::shared_ptr<float> eventData) {
         return * eventData.get();
      });
      
      message.iii.resize(event.iii.size());
      std::transform(event.iii.begin(), event.iii.end(), message.iii.begin(),
                     [&](std::shared_ptr<double> eventData) {
         return * eventData.get();
      });
      
      message.jjj.resize(event.jjj.size());
      std::transform(event.jjj.begin(), event.jjj.end(), message.jjj.begin(),
                     [&](std::shared_ptr<bool> eventData) {
         return * eventData.get();
      });
      
      message.kkk.resize(event.kkk.size());
      std::transform(event.kkk.begin(), event.kkk.end(), message.kkk.begin(),
                     [&](std::shared_ptr<std::string> eventData) {
         return * eventData.get();
      });
   }
};
}

#endif // PRIMITIVE_TYPES_VECTOR_SHARED_PTR_MAPPER_ROS