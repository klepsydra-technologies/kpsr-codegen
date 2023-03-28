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

#ifndef POSE_MAPPER_ROS
#define POSE_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <geometry_msgs/point_ros_mapper.h>
#include <geometry_msgs/quaternion_ros_mapper.h>
#include <klepsydra/codegen/pose.h>
#include <geometry_msgs/Pose.h>

namespace kpsr {
template <>

class Mapper<kpsr::geometry::Pose, geometry_msgs::Pose> {
public:
   void fromMiddleware(const geometry_msgs::Pose & message, kpsr::geometry::Pose & event) {
      _kpsr_geometry_point_mapper.fromMiddleware(message.position, event.position);
      _kpsr_geometry_quaternion_mapper.fromMiddleware(message.orientation, event.orientation);
   }

   void toMiddleware(const kpsr::geometry::Pose &event, geometry_msgs::Pose &message) {
      _kpsr_geometry_point_mapper.toMiddleware(event.position, message.position);
      _kpsr_geometry_quaternion_mapper.toMiddleware(event.orientation, message.orientation);
   }
    Mapper<kpsr::geometry::Point, geometry_msgs::Point> _kpsr_geometry_point_mapper;
    Mapper<kpsr::geometry::Quaternion, geometry_msgs::Quaternion> _kpsr_geometry_quaternion_mapper;
};
}

#endif // POSE_MAPPER_ROS