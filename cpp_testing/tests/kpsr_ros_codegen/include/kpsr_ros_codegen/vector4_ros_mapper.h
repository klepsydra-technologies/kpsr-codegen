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

#ifndef VECTOR4_MAPPER_ROS
#define VECTOR4_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <klepsydra/codegen/vector4.h>
#include <kpsr_ros_codegen/Vector4.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::Vector4, kpsr_ros_codegen::Vector4> {
public:
   void fromMiddleware(const kpsr_ros_codegen::Vector4 & message, kpsr::codegen::Vector4 & event) {
      event.a = message.a;
      event.b = message.b;
      event.c = message.c;
      event.d = message.d;
   }

   void toMiddleware(const kpsr::codegen::Vector4 &event, kpsr_ros_codegen::Vector4 &message) {
      message.a = event.a;
      message.b = event.b;
      message.c = event.c;
      message.d = event.d;
   }
};
}

#endif // VECTOR4_MAPPER_ROS