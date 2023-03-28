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

#ifndef COMPOSITION_TYPE_RELATED_MAPPER_ROS
#define COMPOSITION_TYPE_RELATED_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <sensor_msgs/gps_ros_mapper.h>
#include <klepsydra/codegen/composition_type_related.h>
#include <geometry_msgs/quaternion_ros_mapper.h>
#include <kpsr_ros_codegen/CompositionTypeRelated.h>
#include <kpsr_ros_codegen/vector4_ros_mapper.h>
#include <geometry_msgs/vector3_ros_mapper.h>

namespace kpsr {
template <>

class Mapper<kpsr::codegen::CompositionTypeRelated, kpsr_ros_codegen::CompositionTypeRelated> {
public:
   void fromMiddleware(const kpsr_ros_codegen::CompositionTypeRelated & message, kpsr::codegen::CompositionTypeRelated & event) {
      event.seq = message.seq;
      
      std::transform(message.quaternionArray.begin(), message.quaternionArray.end(), event.quaternionArray.begin(),
                     [&](const kpsr_ros_codegen::Vector4 messageData) {
         kpsr::codegen::Vector4 eventData;
         _kpsr_codegen_vector4_mapper.fromMiddleware(messageData, eventData);
         return eventData;
      });
      
      event.quaternionVector.resize(message.quaternionVector.size());
      std::transform(message.quaternionVector.begin(), message.quaternionVector.end(), event.quaternionVector.begin(),
                     [&](const kpsr_ros_codegen::Vector4 messageData) {
         kpsr::codegen::Vector4 eventData;
         _kpsr_codegen_vector4_mapper.fromMiddleware(messageData, eventData);
         return eventData;
      });
      
      event.quaternionVectorSharedPtr.resize(message.quaternionVectorSharedPtr.size());
      std::transform(message.quaternionVectorSharedPtr.begin(), message.quaternionVectorSharedPtr.end(), event.quaternionVectorSharedPtr.begin(),
                     [&](const kpsr_ros_codegen::Vector4 messageData) {
         std::shared_ptr<kpsr::codegen::Vector4> eventData(new kpsr::codegen::Vector4());
         _kpsr_codegen_vector4_mapper.fromMiddleware(messageData, * eventData.get());
         return eventData;
      });
      
      event.quaternionVectorPointer.resize(message.quaternionVectorPointer.size());
      std::transform(message.quaternionVectorPointer.begin(), message.quaternionVectorPointer.end(), event.quaternionVectorPointer.begin(),
                     [&](const kpsr_ros_codegen::Vector4 messageData) {
         kpsr::codegen::Vector4 * eventData = new kpsr::codegen::Vector4();
         _kpsr_codegen_vector4_mapper.fromMiddleware(messageData, * eventData);
         return eventData;
      });
      
      std::transform(message.positionArray.begin(), message.positionArray.end(), event.positionArray.begin(),
                     [&](const geometry_msgs::Vector3 messageData) {
         kpsr::geometry::Vector3 eventData;
         _kpsr_geometry_vector3_mapper.fromMiddleware(messageData, eventData);
         return eventData;
      });
      
      event.positionVector.resize(message.positionVector.size());
      std::transform(message.positionVector.begin(), message.positionVector.end(), event.positionVector.begin(),
                     [&](const geometry_msgs::Vector3 messageData) {
         kpsr::geometry::Vector3 eventData;
         _kpsr_geometry_vector3_mapper.fromMiddleware(messageData, eventData);
         return eventData;
      });
      
      event.positionVectorSharedPtr.resize(message.positionVectorSharedPtr.size());
      std::transform(message.positionVectorSharedPtr.begin(), message.positionVectorSharedPtr.end(), event.positionVectorSharedPtr.begin(),
                     [&](const geometry_msgs::Vector3 messageData) {
         std::shared_ptr<kpsr::geometry::Vector3> eventData(new kpsr::geometry::Vector3());
         _kpsr_geometry_vector3_mapper.fromMiddleware(messageData, * eventData.get());
         return eventData;
      });
      
      event.positionVectorPointer.resize(message.positionVectorPointer.size());
      std::transform(message.positionVectorPointer.begin(), message.positionVectorPointer.end(), event.positionVectorPointer.begin(),
                     [&](const geometry_msgs::Vector3 messageData) {
         kpsr::geometry::Vector3 * eventData = new kpsr::geometry::Vector3();
         _kpsr_geometry_vector3_mapper.fromMiddleware(messageData, * eventData);
         return eventData;
      });
      event.newEnum = (kpsr::codegen::NewEnum) message.newEnum;
      
      std::transform(message.newEnumArray.begin(), message.newEnumArray.end(), event.newEnumArray.begin(),
                     [](const unsigned short int messageData) {
            return (kpsr::codegen::NewEnum) messageData;
      });
      
      event.newEnumVector.resize(message.newEnumVector.size());
      std::transform(message.newEnumVector.begin(), message.newEnumVector.end(), event.newEnumVector.begin(),
                     [](const unsigned short int messageData) {
            return (kpsr::codegen::NewEnum) messageData;
      });
      
      event.newEnumVectorSharedPtr.resize(message.newEnumVectorSharedPtr.size());
      std::transform(message.newEnumVectorSharedPtr.begin(), message.newEnumVectorSharedPtr.end(), event.newEnumVectorSharedPtr.begin(),
                     [](const unsigned short int messageData) {
            return std::make_shared<kpsr::codegen::NewEnum>( (kpsr::codegen::NewEnum) messageData);
      });
      
      event.newEnumVectorPointer.resize(message.newEnumVectorPointer.size());
      std::transform(message.newEnumVectorPointer.begin(), message.newEnumVectorPointer.end(), event.newEnumVectorPointer.begin(),
                     [](const unsigned short int messageData) {
            return new kpsr::codegen::NewEnum ( (kpsr::codegen::NewEnum) messageData);
      });
      event.oldEnum = (kpsr::codegen::OldEnum) message.oldEnum;
      
      std::transform(message.oldEnumArray.begin(), message.oldEnumArray.end(), event.oldEnumArray.begin(),
                     [](const unsigned short int messageData) {
            return (kpsr::codegen::OldEnum) messageData;
      });
      
      event.oldEnumVector.resize(message.oldEnumVector.size());
      std::transform(message.oldEnumVector.begin(), message.oldEnumVector.end(), event.oldEnumVector.begin(),
                     [](const unsigned short int messageData) {
            return (kpsr::codegen::OldEnum) messageData;
      });
      
      event.oldEnumVectorSharedPtr.resize(message.oldEnumVectorSharedPtr.size());
      std::transform(message.oldEnumVectorSharedPtr.begin(), message.oldEnumVectorSharedPtr.end(), event.oldEnumVectorSharedPtr.begin(),
                     [](const unsigned short int messageData) {
            return std::make_shared<kpsr::codegen::OldEnum>( (kpsr::codegen::OldEnum) messageData);
      });
      
      event.oldEnumVectorPointer.resize(message.oldEnumVectorPointer.size());
      std::transform(message.oldEnumVectorPointer.begin(), message.oldEnumVectorPointer.end(), event.oldEnumVectorPointer.begin(),
                     [](const unsigned short int messageData) {
            return new kpsr::codegen::OldEnum ( (kpsr::codegen::OldEnum) messageData);
      });
      _kpsr_geometry_quaternion_mapper.fromMiddleware(message.quat, event.quat);
      _kpsr_geometry_gps_mapper.fromMiddleware(message.gpsData, event.gpsData);
   }

   void toMiddleware(const kpsr::codegen::CompositionTypeRelated &event, kpsr_ros_codegen::CompositionTypeRelated &message) {
      message.seq = event.seq;
      
      std::transform(event.quaternionArray.begin(), event.quaternionArray.end(), message.quaternionArray.begin(),
                     [&](kpsr::codegen::Vector4 eventData) {
         kpsr_ros_codegen::Vector4 messageData;
         _kpsr_codegen_vector4_mapper.toMiddleware(eventData, messageData);
         return messageData;
      });
      
      message.quaternionVector.resize(event.quaternionVector.size());
      std::transform(event.quaternionVector.begin(), event.quaternionVector.end(), message.quaternionVector.begin(),
                     [&](kpsr::codegen::Vector4 eventData) {
         kpsr_ros_codegen::Vector4 messageData;
         _kpsr_codegen_vector4_mapper.toMiddleware(eventData, messageData);
         return messageData;
      });
      
      message.quaternionVectorSharedPtr.resize(event.quaternionVectorSharedPtr.size());
      std::transform(event.quaternionVectorSharedPtr.begin(), event.quaternionVectorSharedPtr.end(), message.quaternionVectorSharedPtr.begin(),
                     [&](std::shared_ptr<kpsr::codegen::Vector4> eventData) {
         kpsr_ros_codegen::Vector4 messageData;
         _kpsr_codegen_vector4_mapper.toMiddleware(* eventData.get(), messageData);
         return messageData;
      });
      
      message.quaternionVectorPointer.resize(event.quaternionVectorPointer.size());
      std::transform(event.quaternionVectorPointer.begin(), event.quaternionVectorPointer.end(), message.quaternionVectorPointer.begin(),
                     [&](kpsr::codegen::Vector4 * eventData) {
         kpsr_ros_codegen::Vector4 messageData;
         _kpsr_codegen_vector4_mapper.toMiddleware(* eventData, messageData);
         return messageData;
      });
      
      std::transform(event.positionArray.begin(), event.positionArray.end(), message.positionArray.begin(),
                     [&](kpsr::geometry::Vector3 eventData) {
         geometry_msgs::Vector3 messageData;
         _kpsr_geometry_vector3_mapper.toMiddleware(eventData, messageData);
         return messageData;
      });
      
      message.positionVector.resize(event.positionVector.size());
      std::transform(event.positionVector.begin(), event.positionVector.end(), message.positionVector.begin(),
                     [&](kpsr::geometry::Vector3 eventData) {
         geometry_msgs::Vector3 messageData;
         _kpsr_geometry_vector3_mapper.toMiddleware(eventData, messageData);
         return messageData;
      });
      
      message.positionVectorSharedPtr.resize(event.positionVectorSharedPtr.size());
      std::transform(event.positionVectorSharedPtr.begin(), event.positionVectorSharedPtr.end(), message.positionVectorSharedPtr.begin(),
                     [&](std::shared_ptr<kpsr::geometry::Vector3> eventData) {
         geometry_msgs::Vector3 messageData;
         _kpsr_geometry_vector3_mapper.toMiddleware(* eventData.get(), messageData);
         return messageData;
      });
      
      message.positionVectorPointer.resize(event.positionVectorPointer.size());
      std::transform(event.positionVectorPointer.begin(), event.positionVectorPointer.end(), message.positionVectorPointer.begin(),
                     [&](kpsr::geometry::Vector3 * eventData) {
         geometry_msgs::Vector3 messageData;
         _kpsr_geometry_vector3_mapper.toMiddleware(* eventData, messageData);
         return messageData;
      });
      message.newEnum = event.newEnum;
      
      std::transform(event.newEnumArray.begin(), event.newEnumArray.end(), message.newEnumArray.begin(),
                     [&](kpsr::codegen::NewEnum eventData) {
         return eventData;
      });
      
      message.newEnumVector.resize(event.newEnumVector.size());
      std::transform(event.newEnumVector.begin(), event.newEnumVector.end(), message.newEnumVector.begin(),
                     [&](kpsr::codegen::NewEnum eventData) {
         return eventData;
      });
      
      message.newEnumVectorSharedPtr.resize(event.newEnumVectorSharedPtr.size());
      std::transform(event.newEnumVectorSharedPtr.begin(), event.newEnumVectorSharedPtr.end(), message.newEnumVectorSharedPtr.begin(),
                     [&](std::shared_ptr<kpsr::codegen::NewEnum> eventData) {
         return * eventData.get();
      });
      
      message.newEnumVectorPointer.resize(event.newEnumVectorPointer.size());
      std::transform(event.newEnumVectorPointer.begin(), event.newEnumVectorPointer.end(), message.newEnumVectorPointer.begin(),
                     [&](kpsr::codegen::NewEnum * eventData) {
         return * eventData;
      });
      message.oldEnum = event.oldEnum;
      
      std::transform(event.oldEnumArray.begin(), event.oldEnumArray.end(), message.oldEnumArray.begin(),
                     [&](kpsr::codegen::OldEnum eventData) {
         return eventData;
      });
      
      message.oldEnumVector.resize(event.oldEnumVector.size());
      std::transform(event.oldEnumVector.begin(), event.oldEnumVector.end(), message.oldEnumVector.begin(),
                     [&](kpsr::codegen::OldEnum eventData) {
         return eventData;
      });
      
      message.oldEnumVectorSharedPtr.resize(event.oldEnumVectorSharedPtr.size());
      std::transform(event.oldEnumVectorSharedPtr.begin(), event.oldEnumVectorSharedPtr.end(), message.oldEnumVectorSharedPtr.begin(),
                     [&](std::shared_ptr<kpsr::codegen::OldEnum> eventData) {
         return * eventData.get();
      });
      
      message.oldEnumVectorPointer.resize(event.oldEnumVectorPointer.size());
      std::transform(event.oldEnumVectorPointer.begin(), event.oldEnumVectorPointer.end(), message.oldEnumVectorPointer.begin(),
                     [&](kpsr::codegen::OldEnum * eventData) {
         return * eventData;
      });
      _kpsr_geometry_quaternion_mapper.toMiddleware(event.quat, message.quat);
      _kpsr_geometry_gps_mapper.toMiddleware(event.gpsData, message.gpsData);
   }
    Mapper<kpsr::codegen::Vector4, kpsr_ros_codegen::Vector4> _kpsr_codegen_vector4_mapper;
    Mapper<kpsr::geometry::Vector3, geometry_msgs::Vector3> _kpsr_geometry_vector3_mapper;
    Mapper<kpsr::geometry::Quaternion, geometry_msgs::Quaternion> _kpsr_geometry_quaternion_mapper;
    Mapper<kpsr::geometry::Gps, sensor_msgs::NavSatFix> _kpsr_geometry_gps_mapper;
};
}

#endif // COMPOSITION_TYPE_RELATED_MAPPER_ROS