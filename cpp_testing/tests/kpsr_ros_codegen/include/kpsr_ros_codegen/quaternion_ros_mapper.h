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

#ifndef QUATERNION_MAPPER_ROS
#define QUATERNION_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <geometry_msgs/Quaternion.h>
#include <klepsydra/codegen/quaternion.h>

namespace kpsr {
template <>

class Mapper<kpsr::geometry::Quaternion, geometry_msgs::Quaternion> {
public:
   void fromMiddleware(const geometry_msgs::Quaternion & message, kpsr::geometry::Quaternion & event) {
      event.x = message.x;
      event.y = message.y;
      event.z = message.z;
      event.w = message.w;
   }

   void toMiddleware(const kpsr::geometry::Quaternion &event, geometry_msgs::Quaternion &message) {
      message.x = event.x;
      message.y = event.y;
      message.z = event.z;
      message.w = event.w;
   }
};
}

#endif // QUATERNION_MAPPER_ROS