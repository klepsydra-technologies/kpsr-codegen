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

#ifndef INHERITANCE_VECTOR4_MAPPER_ROS
#define INHERITANCE_VECTOR4_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <kpsr_ros_codegen/InheritanceVector4.h>
#include <klepsydra/codegen/inheritance_vector4.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::InheritanceVector4, kpsr_ros_codegen::InheritanceVector4> {
public:
   void fromMiddleware(const kpsr_ros_codegen::InheritanceVector4 & message, kpsr::codegen::InheritanceVector4 & event) {
      event.d = message.d;
      event.a = message.a;
      event.b = message.b;
      event.c = message.c;
   }

   void toMiddleware(const kpsr::codegen::InheritanceVector4 &event, kpsr_ros_codegen::InheritanceVector4 &message) {
      message.d = event.d;
      message.a = event.a;
      message.b = event.b;
      message.c = event.c;
   }
};
}

#endif // INHERITANCE_VECTOR4_MAPPER_ROS