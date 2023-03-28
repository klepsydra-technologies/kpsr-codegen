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

#include <string>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>

#include "inheritance_vector4_ros_mapper.h"

TEST(KpsrRosCodegeTest, inheritanceMapperMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> basicProvider(nullptr,
                                                                                          "test",
                                                                                          0,
                                                                                          nullptr,
                                                                                          nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider
        .registerToTopic<kpsr::codegen::InheritanceVector4, kpsr_ros_codegen::InheritanceVector4>(
            "kpsr_ros_codegen_test_topic1", 10, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    rate.sleep();
    ros::Publisher stringPublisher =
        nodeHandle.advertise<kpsr_ros_codegen::InheritanceVector4>("kpsr_ros_codegen_test_topic1",
                                                                   10,
                                                                   true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::InheritanceVector4> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::codegen::InheritanceVector4,
                                             kpsr_ros_codegen::InheritanceVector4>(
            "kpsr_ros_codegen_test_topic1", 1, nullptr, stringPublisher);

    kpsr::codegen::InheritanceVector4 event(5.0, 5.1, 5.2, 5.3);
    int numEvents = 10;
    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);
    for (int i = 0; i < numEvents; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while ((cacheListener.counter == 0) && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, numEvents);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);
}
