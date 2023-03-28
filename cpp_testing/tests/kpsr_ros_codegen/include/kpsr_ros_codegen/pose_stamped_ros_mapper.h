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

#ifndef POSE_STAMPED_MAPPER_ROS
#define POSE_STAMPED_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <klepsydra/codegen/pose_stamped.h>
#include <std_msgs/header_ros_mapper.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/pose_ros_mapper.h>

namespace kpsr {
template <>

class Mapper<kpsr::geometry::PoseStamped, geometry_msgs::PoseStamped> {
public:
   void fromMiddleware(const geometry_msgs::PoseStamped & message, kpsr::geometry::PoseStamped & event) {
      _kpsr_geometry_header_mapper.fromMiddleware(message.header, event.header);
      _kpsr_geometry_pose_mapper.fromMiddleware(message.pose, event.pose);
   }

   void toMiddleware(const kpsr::geometry::PoseStamped &event, geometry_msgs::PoseStamped &message) {
      _kpsr_geometry_header_mapper.toMiddleware(event.header, message.header);
      _kpsr_geometry_pose_mapper.toMiddleware(event.pose, message.pose);
   }
    Mapper<kpsr::geometry::Header, std_msgs::Header> _kpsr_geometry_header_mapper;
    Mapper<kpsr::geometry::Pose, geometry_msgs::Pose> _kpsr_geometry_pose_mapper;
};
}

#endif // POSE_STAMPED_MAPPER_ROS