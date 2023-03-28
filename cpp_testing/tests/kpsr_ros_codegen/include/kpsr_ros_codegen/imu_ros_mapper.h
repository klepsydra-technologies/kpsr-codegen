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

#ifndef IMU_MAPPER_ROS
#define IMU_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <klepsydra/codegen/imu.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/quaternion_ros_mapper.h>
#include <geometry_msgs/vector3_ros_mapper.h>

namespace kpsr {
template <>

class Mapper<kpsr::geometry::Imu, sensor_msgs::Imu> {
public:
   void fromMiddleware(const sensor_msgs::Imu & message, kpsr::geometry::Imu & event) {
      _kpsr_geometry_quaternion_mapper.fromMiddleware(message.orientation, event.orientation);
      
      std::transform(message.orientation_covariance.begin(), message.orientation_covariance.end(), event.orientation_covariance.begin(),
                     [](const double messageData) {
            return messageData;
      });
      _kpsr_geometry_vector3_mapper.fromMiddleware(message.angular_velocity, event.angular_velocity);
      
      std::transform(message.angular_velocity_covariance.begin(), message.angular_velocity_covariance.end(), event.angular_velocity_covariance.begin(),
                     [](const double messageData) {
            return messageData;
      });
      _kpsr_geometry_vector3_mapper.fromMiddleware(message.linear_acceleration, event.linear_acceleration);
      
      std::transform(message.linear_acceleration_covariance.begin(), message.linear_acceleration_covariance.end(), event.linear_acceleration_covariance.begin(),
                     [](const double messageData) {
            return messageData;
      });
   }

   void toMiddleware(const kpsr::geometry::Imu &event, sensor_msgs::Imu &message) {
      _kpsr_geometry_quaternion_mapper.toMiddleware(event.orientation, message.orientation);
      
      std::transform(event.orientation_covariance.begin(), event.orientation_covariance.end(), message.orientation_covariance.begin(),
                     [&](double eventData) {
         return eventData;
      });
      _kpsr_geometry_vector3_mapper.toMiddleware(event.angular_velocity, message.angular_velocity);
      
      std::transform(event.angular_velocity_covariance.begin(), event.angular_velocity_covariance.end(), message.angular_velocity_covariance.begin(),
                     [&](double eventData) {
         return eventData;
      });
      _kpsr_geometry_vector3_mapper.toMiddleware(event.linear_acceleration, message.linear_acceleration);
      
      std::transform(event.linear_acceleration_covariance.begin(), event.linear_acceleration_covariance.end(), message.linear_acceleration_covariance.begin(),
                     [&](double eventData) {
         return eventData;
      });
   }
    Mapper<kpsr::geometry::Quaternion, geometry_msgs::Quaternion> _kpsr_geometry_quaternion_mapper;
    Mapper<kpsr::geometry::Vector3, geometry_msgs::Vector3> _kpsr_geometry_vector3_mapper;
};
}

#endif // IMU_MAPPER_ROS