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

#ifndef PRIMITIVE_TYPES_BASIC_MAPPER_ROS
#define PRIMITIVE_TYPES_BASIC_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <klepsydra/codegen/primitive_types_basic.h>
#include <kpsr_ros_codegen/PrimitiveTypesBasic.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::PrimitiveTypesBasic, kpsr_ros_codegen::PrimitiveTypesBasic> {
public:
   void fromMiddleware(const kpsr_ros_codegen::PrimitiveTypesBasic & message, kpsr::codegen::PrimitiveTypesBasic & event) {
      event.seq = message.seq;
      event.a = message.a;
      event.b = message.b;
      event.c = message.c;
      event.d = message.d;
      event.e = message.e;
      event.f = message.f;
      event.g = message.g;
      event.h = message.h;
      event.i = message.i;
      event.j = message.j;
      event.k = message.k;
   }

   void toMiddleware(const kpsr::codegen::PrimitiveTypesBasic &event, kpsr_ros_codegen::PrimitiveTypesBasic &message) {
      message.seq = event.seq;
      message.a = event.a;
      message.b = event.b;
      message.c = event.c;
      message.d = event.d;
      message.e = event.e;
      message.f = event.f;
      message.g = event.g;
      message.h = event.h;
      message.i = event.i;
      message.j = event.j;
      message.k = event.k;
   }
};
}

#endif // PRIMITIVE_TYPES_BASIC_MAPPER_ROS