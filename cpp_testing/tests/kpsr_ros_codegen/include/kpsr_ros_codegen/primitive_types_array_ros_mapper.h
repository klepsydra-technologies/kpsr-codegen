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

#ifndef PRIMITIVE_TYPES_ARRAY_MAPPER_ROS
#define PRIMITIVE_TYPES_ARRAY_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <kpsr_ros_codegen/PrimitiveTypesArray.h>
#include <klepsydra/codegen/primitive_types_array.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::PrimitiveTypesArray, kpsr_ros_codegen::PrimitiveTypesArray> {
public:
   void fromMiddleware(const kpsr_ros_codegen::PrimitiveTypesArray & message, kpsr::codegen::PrimitiveTypesArray & event) {
      event.seq = message.seq;
      
      std::transform(message.aa.begin(), message.aa.end(), event.aa.begin(),
                     [](const signed char messageData) {
            return messageData;
      });
      
      std::transform(message.bb.begin(), message.bb.end(), event.bb.begin(),
                     [](const unsigned char messageData) {
            return messageData;
      });
      
      std::transform(message.cc.begin(), message.cc.end(), event.cc.begin(),
                     [](const short int messageData) {
            return messageData;
      });
      
      std::transform(message.dd.begin(), message.dd.end(), event.dd.begin(),
                     [](const unsigned short int messageData) {
            return messageData;
      });
      
      std::transform(message.ee.begin(), message.ee.end(), event.ee.begin(),
                     [](const unsigned int messageData) {
            return messageData;
      });
      
      std::transform(message.ff.begin(), message.ff.end(), event.ff.begin(),
                     [](const long long int messageData) {
            return messageData;
      });
      
      std::transform(message.gg.begin(), message.gg.end(), event.gg.begin(),
                     [](const unsigned long long int messageData) {
            return messageData;
      });
      
      std::transform(message.hh.begin(), message.hh.end(), event.hh.begin(),
                     [](const float messageData) {
            return messageData;
      });
      
      std::transform(message.ii.begin(), message.ii.end(), event.ii.begin(),
                     [](const double messageData) {
            return messageData;
      });
      
      std::transform(message.jj.begin(), message.jj.end(), event.jj.begin(),
                     [](const bool messageData) {
            return messageData;
      });
      
      std::transform(message.kk.begin(), message.kk.end(), event.kk.begin(),
                     [](const std::string messageData) {
            return messageData;
      });
   }

   void toMiddleware(const kpsr::codegen::PrimitiveTypesArray &event, kpsr_ros_codegen::PrimitiveTypesArray &message) {
      message.seq = event.seq;
      
      std::transform(event.aa.begin(), event.aa.end(), message.aa.begin(),
                     [&](signed char eventData) {
         return eventData;
      });
      
      std::transform(event.bb.begin(), event.bb.end(), message.bb.begin(),
                     [&](unsigned char eventData) {
         return eventData;
      });
      
      std::transform(event.cc.begin(), event.cc.end(), message.cc.begin(),
                     [&](short int eventData) {
         return eventData;
      });
      
      std::transform(event.dd.begin(), event.dd.end(), message.dd.begin(),
                     [&](unsigned short int eventData) {
         return eventData;
      });
      
      std::transform(event.ee.begin(), event.ee.end(), message.ee.begin(),
                     [&](unsigned int eventData) {
         return eventData;
      });
      
      std::transform(event.ff.begin(), event.ff.end(), message.ff.begin(),
                     [&](long long int eventData) {
         return eventData;
      });
      
      std::transform(event.gg.begin(), event.gg.end(), message.gg.begin(),
                     [&](unsigned long long int eventData) {
         return eventData;
      });
      
      std::transform(event.hh.begin(), event.hh.end(), message.hh.begin(),
                     [&](float eventData) {
         return eventData;
      });
      
      std::transform(event.ii.begin(), event.ii.end(), message.ii.begin(),
                     [&](double eventData) {
         return eventData;
      });
      
      std::transform(event.jj.begin(), event.jj.end(), message.jj.begin(),
                     [&](bool eventData) {
         return eventData;
      });
      
      std::transform(event.kk.begin(), event.kk.end(), message.kk.begin(),
                     [&](std::string eventData) {
         return eventData;
      });
   }
};
}

#endif // PRIMITIVE_TYPES_ARRAY_MAPPER_ROS