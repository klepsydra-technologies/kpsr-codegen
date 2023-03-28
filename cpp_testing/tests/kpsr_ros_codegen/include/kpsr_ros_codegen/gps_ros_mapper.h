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

#ifndef GPS_MAPPER_ROS
#define GPS_MAPPER_ROS

#include <klepsydra/serialization/mapper.h>

#include <sensor_msgs/NavSatFix.h>
#include <klepsydra/codegen/gps.h>

namespace kpsr {
template <>

class Mapper<kpsr::geometry::Gps, sensor_msgs::NavSatFix> {
public:
   void fromMiddleware(const sensor_msgs::NavSatFix & message, kpsr::geometry::Gps & event) {
      event.latitude = message.latitude;
      event.longitude = message.longitude;
      event.altitude = message.altitude;
   }

   void toMiddleware(const kpsr::geometry::Gps &event, sensor_msgs::NavSatFix &message) {
      message.latitude = event.latitude;
      message.longitude = event.longitude;
      message.altitude = event.altitude;
   }
};
}

#endif // GPS_MAPPER_ROS