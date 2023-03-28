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

#ifndef PRIMITIVE_TYPES_VECTOR_POINTER_MAPPER_ROS
#define PRIMITIVE_TYPES_VECTOR_POINTER_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <kpsr_ros_codegen/PrimitiveTypesVectorPointer.h>
#include <klepsydra/codegen/primitive_types_vector_pointer.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::PrimitiveTypesVectorPointer, kpsr_ros_codegen::PrimitiveTypesVectorPointer> {
public:
   void fromMiddleware(const kpsr_ros_codegen::PrimitiveTypesVectorPointer & message, kpsr::codegen::PrimitiveTypesVectorPointer & event) {
      event.seq = message.seq;
      
      event.aaa.resize(message.aaa.size());
      std::transform(message.aaa.begin(), message.aaa.end(), event.aaa.begin(),
                     [](const signed char messageData) {
            return new signed char (messageData);
      });
      
      event.bbb.resize(message.bbb.size());
      std::transform(message.bbb.begin(), message.bbb.end(), event.bbb.begin(),
                     [](const unsigned char messageData) {
            return new unsigned char (messageData);
      });
      
      event.ccc.resize(message.ccc.size());
      std::transform(message.ccc.begin(), message.ccc.end(), event.ccc.begin(),
                     [](const short int messageData) {
            return new short int (messageData);
      });
      
      event.ddd.resize(message.ddd.size());
      std::transform(message.ddd.begin(), message.ddd.end(), event.ddd.begin(),
                     [](const unsigned short int messageData) {
            return new unsigned short int (messageData);
      });
      
      event.eee.resize(message.eee.size());
      std::transform(message.eee.begin(), message.eee.end(), event.eee.begin(),
                     [](const unsigned int messageData) {
            return new unsigned int (messageData);
      });
      
      event.fff.resize(message.fff.size());
      std::transform(message.fff.begin(), message.fff.end(), event.fff.begin(),
                     [](const long long int messageData) {
            return new long long int (messageData);
      });
      
      event.ggg.resize(message.ggg.size());
      std::transform(message.ggg.begin(), message.ggg.end(), event.ggg.begin(),
                     [](const unsigned long long int messageData) {
            return new unsigned long long int (messageData);
      });
      
      event.hhh.resize(message.hhh.size());
      std::transform(message.hhh.begin(), message.hhh.end(), event.hhh.begin(),
                     [](const float messageData) {
            return new float (messageData);
      });
      
      event.iii.resize(message.iii.size());
      std::transform(message.iii.begin(), message.iii.end(), event.iii.begin(),
                     [](const double messageData) {
            return new double (messageData);
      });
      
      event.jjj.resize(message.jjj.size());
      std::transform(message.jjj.begin(), message.jjj.end(), event.jjj.begin(),
                     [](const bool messageData) {
            return new bool (messageData);
      });
      
      event.kkk.resize(message.kkk.size());
      std::transform(message.kkk.begin(), message.kkk.end(), event.kkk.begin(),
                     [](const std::string messageData) {
            return new std::string (messageData);
      });
   }

   void toMiddleware(const kpsr::codegen::PrimitiveTypesVectorPointer &event, kpsr_ros_codegen::PrimitiveTypesVectorPointer &message) {
      message.seq = event.seq;
      
      message.aaa.resize(event.aaa.size());
      std::transform(event.aaa.begin(), event.aaa.end(), message.aaa.begin(),
                     [&](signed char * eventData) {
         return * eventData;
      });
      
      message.bbb.resize(event.bbb.size());
      std::transform(event.bbb.begin(), event.bbb.end(), message.bbb.begin(),
                     [&](unsigned char * eventData) {
         return * eventData;
      });
      
      message.ccc.resize(event.ccc.size());
      std::transform(event.ccc.begin(), event.ccc.end(), message.ccc.begin(),
                     [&](short int * eventData) {
         return * eventData;
      });
      
      message.ddd.resize(event.ddd.size());
      std::transform(event.ddd.begin(), event.ddd.end(), message.ddd.begin(),
                     [&](unsigned short int * eventData) {
         return * eventData;
      });
      
      message.eee.resize(event.eee.size());
      std::transform(event.eee.begin(), event.eee.end(), message.eee.begin(),
                     [&](unsigned int * eventData) {
         return * eventData;
      });
      
      message.fff.resize(event.fff.size());
      std::transform(event.fff.begin(), event.fff.end(), message.fff.begin(),
                     [&](long long int * eventData) {
         return * eventData;
      });
      
      message.ggg.resize(event.ggg.size());
      std::transform(event.ggg.begin(), event.ggg.end(), message.ggg.begin(),
                     [&](unsigned long long int * eventData) {
         return * eventData;
      });
      
      message.hhh.resize(event.hhh.size());
      std::transform(event.hhh.begin(), event.hhh.end(), message.hhh.begin(),
                     [&](float * eventData) {
         return * eventData;
      });
      
      message.iii.resize(event.iii.size());
      std::transform(event.iii.begin(), event.iii.end(), message.iii.begin(),
                     [&](double * eventData) {
         return * eventData;
      });
      
      message.jjj.resize(event.jjj.size());
      std::transform(event.jjj.begin(), event.jjj.end(), message.jjj.begin(),
                     [&](bool * eventData) {
         return * eventData;
      });
      
      message.kkk.resize(event.kkk.size());
      std::transform(event.kkk.begin(), event.kkk.end(), message.kkk.begin(),
                     [&](std::string * eventData) {
         return * eventData;
      });
   }
};
}

#endif // PRIMITIVE_TYPES_VECTOR_POINTER_MAPPER_ROS